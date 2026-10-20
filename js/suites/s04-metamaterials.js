// Suite 04 — Advanced metamaterial unit-cell design, mechanics and homogenisation.
import { homogenize, bistableBeam } from '../core/fem.js';
import { range, fmt, linspace, rad, trapz } from '../core/num.js';
import { MATERIALS, get as getMat } from '../core/materials.js';
import { cmap, cssColor } from '../core/chart.js';
import { h } from '../core/ui.js';

const CELLS = { reentrant: 'Re-entrant honeycomb (auxetic)', hexagonal: 'Regular / convex hexagonal honeycomb', tetrachiral: 'Tetrachiral (rings + tangent ligaments)', zeroPoisson: 'Zero-Poisson brick (θ = 0 hexagonal)', triangular: 'Triangular (stretch-dominated)', square: 'Square grid', };
const latticeMats = MATERIALS.filter(m => ['Metal', 'Polymer', 'Composite', 'SMA'].includes(m.family) && m.E);

// Unit-cell generators: nodes, struts with periodic image shifts, lattice vectors (metres)
export function makeCell(type, { l, hr, theta, rr }) {
  const th = rad(theta);
  if (type === 'reentrant' || type === 'hexagonal' || type === 'zeroPoisson') {
    const t2 = type === 'zeroPoisson' ? 0 : type === 'reentrant' ? -Math.abs(th) : Math.abs(th), hh = hr * l;
    return { nodes: [[0, 0], [0, hh]], struts: [{ i: 0, j: 1 }, { i: 1, j: 0, shift: [0, 1] }, { i: 1, j: 0, shift: [-1, 1] }], a1: [2 * l * Math.cos(t2), 0], a2: [l * Math.cos(t2), hh + l * Math.sin(t2)], th: t2, hh };
  }
  if (type === 'triangular') return { nodes: [[0, 0]], struts: [{ i: 0, j: 0, shift: [1, 0] }, { i: 0, j: 0, shift: [0, 1] }, { i: 0, j: 0, shift: [-1, 1] }], a1: [l, 0], a2: [l / 2, l * Math.sqrt(3) / 2] };
  if (type === 'square') return { nodes: [[0, 0]], struts: [{ i: 0, j: 0, shift: [1, 0] }, { i: 0, j: 0, shift: [0, 1] }], a1: [l, 0], a2: [0, l] };
  // tetrachiral: rings joined by ligaments exactly tangent to both rings (internal tangents), rings ≈ rigid
  const r = Math.min(rr, 0.45) * l, th0 = Math.acos(Math.min(0.99, 2 * r / l)), nodes = range(8).map(k => [r * Math.cos(th0 + k * Math.PI / 4), r * Math.sin(th0 + k * Math.PI / 4)]), struts = range(8).map(k => ({ i: k, j: (k + 1) % 8, tf: 2.5 })); // thick rings, slender ligaments
  struts.push({ i: 0, j: 4, shift: [1, 0] }, { i: 2, j: 6, shift: [0, 1] });
  return { nodes, struts, a1: [l, 0], a2: [0, l], ring: true };
}
function analytic(type, c, t, l, Es) {
  if (type === 'reentrant' || type === 'hexagonal') { const s = Math.sin(c.th), co = Math.cos(c.th), hl = c.hh / l, k3 = (t / l) ** 3; return { Ex: Es * k3 * co / ((hl + s) * s * s), Ey: Es * k3 * (hl + s) / co ** 3, nuxy: co * co / ((hl + s) * s), Gxy: Es * k3 * (hl + s) / (hl * hl * (1 + 2 * hl) * co), model: 'Gibson–Ashby (bending-dominated)' }; }
  if (type === 'triangular') return { Ex: Es * (2 / Math.sqrt(3)) * t / l, Ey: Es * (2 / Math.sqrt(3)) * t / l, nuxy: 1 / 3, Gxy: Es * t / l * (2 / Math.sqrt(3)) * 3 / 8, model: 'Stretch-dominated triangular lattice' };
  if (type === 'square') return { Ex: Es * t / l, Ey: Es * t / l, nuxy: 0, Gxy: 0.5 * Es * (t / l) ** 3, model: 'Square grid (axial stretch, ligament bending in shear)' };
  if (type === 'tetrachiral') return { Ex: NaN, Ey: NaN, nuxy: -1, Gxy: NaN, model: 'Ideal chiral rigid-ring limit (ν → −1)' };
  return { Ex: NaN, Ey: NaN, nuxy: 0, Gxy: NaN, model: 'n/a' };
}

export default {
  description: 'Generates architected unit cells (auxetic re-entrant, hexagonal, chiral, zero-Poisson, triangular, square, bistable curved beams), computes their effective anisotropic constitutive law by periodic frame-FE homogenisation with Hill–Mandel energy consistency, compares with closed-form cellular-solid theory, and evaluates bistable snap-through energy storage for the wing core and the energy buffer.',
  literature: 'auxetic chiral metamaterial homogenization bistable curved beam energy absorption',
  credibility: 'Numerically verified',
  equations: ['Linear and finite-strain elasticity of the constituent; Euler–Bernoulli frame elements with axial, bending and (via end forces) shear deformation', 'Representative-volume-element theory: displacement = macroscopic affine field ε·x + periodic fluctuation; periodic boundary conditions on rotations and fluctuations', 'Hill–Mandel macro-homogeneity: ½ ε:C:ε·V = Σ strut strain energy → effective stiffness tensor C and compliance S', 'Gibson–Ashby / Masters–Evans cellular-solid relations for honeycombs (E₁, E₂, ν₁₂, G₁₂ vs t/l, h/l, θ)', 'Re-entrant cell kinematics, rotating-ring chiral deformation, shear–extension coupling (C₁₆/√(C₁₁C₆₆))', 'Bistable cosine-shaped clamped beam: bending + mid-plane stretching energy, Rayleigh–Ritz buckling-mode expansion, potential-energy minimisation under displacement control, snap-through/negative stiffness, energy barrier', 'Strain-energy density and recoverable energy per unit volume/mass; elastic-limit check ε_max ≤ σ_y/E'],
  methods: ['Direct stiffness assembly of periodic frame cell, LU solution of fluctuation field for six macroscopic strain cases', 'Damped-Newton energy minimisation with Hessian regularisation and line search along the travel path (continuation)', 'Imperfection seeding of the antisymmetric mode to expose the lower-energy snap path', 'Parametric sweep of cell angle for Poisson-ratio design maps'],
  assumptions: ['2D lattice, plane-stress per unit out-of-plane depth; strut joints rigid', 'Small strain linear homogenisation (geometric nonlinearity only in the bistable element)', 'Manufacturing imperfections and node fillets not represented (Suite 22 tolerances)'],
  ibc: { initial: ['Undeformed cell geometry, lattice orientation, constituent properties, relative density', 'Zero initial stress/strain; bistable beam in its as-fabricated (first stable) shape with apex height h'], boundary: ['Periodic displacement/rotation conditions with prescribed macroscopic strain (εxx, εyy, γxy and combinations)', 'One fluctuation node pinned to remove rigid translation', 'Bistable element: clamped–clamped ends, displacement-controlled mid-span loading and unloading'] },
  cvv: { calibration: ['Unit-cell force–displacement tests (tension/compression/shear)', 'Digital-image-correlation Poisson ratio', 'Snap-through load and second stable position of printed bistable cells'], verification: ['FE vs Gibson–Ashby for a regular honeycomb (bending-dominated limit)', 'Square-grid axial modulus E = E_s t/l (exact)', 'Symmetry and positive-definiteness of C; Hill–Mandel energy check on a mixed strain state', 'Bistable: work ∫F dd equals stored energy; snap force vs Qiu–Lang–Slocum 740 EIh/L³ (symmetric mode)'], validation: ['Independent printed specimens (reserved) under quasi-static and cyclic loading', 'Fatigue of bistable cells over repeated snap cycles'] },
  formats: { import: ['CSV force–displacement data', 'STL/OBJ cell geometry (inspection)', 'JSON cell definitions'], export: ['JSON effective constitutive law (C, S)', 'CSV sweeps & F–d curves', 'SVG lattice drawing', 'STL / OBJ / glTF extruded lattice (3D view)'] },
  params: [
    { key: 'cell', label: 'Unit-cell topology', type: 'select', group: 'Unit cell', default: 'tetrachiral', help: 'Revision 2 (D2): PP tetrachiral core at ≈ 3 % relative density for the morphing trailing region.', options: Object.entries(CELLS).map(([value, label]) => ({ value, label })) },
    { key: 'l', label: 'Strut length l', unit: 'mm', type: 'number', min: 1, max: 120, step: 0.5, group: 'Unit cell', default: 75 },
    { key: 'hr', label: 'Vertical wall ratio h/l', type: 'range', min: 0.5, max: 3, step: 0.05, group: 'Unit cell', default: 2 },
    { key: 'theta', label: 'Cell angle |θ|', unit: '°', type: 'range', min: 5, max: 45, step: 1, group: 'Unit cell', default: 20 },
    { key: 'tr', label: 'Strut slenderness t/l', type: 'range', min: 0.004, max: 0.25, step: 0.0005, group: 'Unit cell', default: 0.0055, help: 't = 0.41 mm at the default 75 mm cell — the powder-bed-fusion minimum for PP; 3 % relative density.' },
    { key: 'rr', label: 'Chiral ring radius r/L', type: 'range', min: 0.1, max: 0.4, step: 0.01, group: 'Unit cell', default: 0.25 },
    { key: 'depth', label: 'Out-of-plane depth b', unit: 'mm', type: 'number', min: 1, max: 100, step: 1, group: 'Unit cell', default: 60 },
    { key: 'mat', label: 'Constituent material', type: 'select', group: 'Constituent', default: 'pp', options: latticeMats.map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.material('lattice')?.id || 'pp', prov: 'library' }) },
    { key: 'strainViz', label: 'Visualised macroscopic strain εxx', unit: '%', type: 'range', min: 0.1, max: 10, step: 0.1, group: 'Visualisation', default: 4 },
    { key: 'bL', label: 'Bistable beam span L', unit: 'mm', type: 'number', min: 10, max: 200, step: 1, group: 'Bistable cell', default: 80 },
    { key: 'bh', label: 'Bistable apex height h', unit: 'mm', type: 'number', min: 0.5, max: 20, step: 0.1, group: 'Bistable cell', default: 3 },
    { key: 'bt', label: 'Bistable beam thickness t', unit: 'mm', type: 'number', min: 0.1, max: 5, step: 0.05, group: 'Bistable cell', default: 0.6 },
    { key: 'bb', label: 'Bistable beam width b', unit: 'mm', type: 'number', min: 1, max: 50, step: 1, group: 'Bistable cell', default: 10 },
    { key: 'bmat', label: 'Bistable cell material', type: 'select', group: 'Bistable cell', default: 'ti-beta21s', options: latticeMats.map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.material('buffer')?.id || 'ti-beta21s', prov: 'library' }) },
    { key: 'asym', label: 'Allow antisymmetric snap mode', type: 'bool', group: 'Bistable cell', default: true, help: 'Single beams snap through the antisymmetric mode at lower force; twin/constrained beams suppress it.' },
  ],
  presets: [{ label: 'Auxetic wing core', values: { cell: 'reentrant', theta: 25, hr: 2, tr: 0.07 } }, { label: 'Chiral bend–twist core', values: { cell: 'tetrachiral', rr: 0.25, tr: 0.06 } }, { label: 'Stiff triangular rib', values: { cell: 'triangular', tr: 0.06 } }, { label: 'Zero-Poisson morphing skin', values: { cell: 'zeroPoisson', hr: 1.5, tr: 0.06 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const M = getMat(p.mat) || getMat('ti-6al4v'), Es = M.E * 1e9, l = p.l / 1000, t = p.tr * l, depth = p.depth / 1000;
    const cell = makeCell(p.cell, { l, hr: p.hr, theta: p.theta, rr: p.rr });
    if (p.cell === 'reentrant' && cell.hh + l * Math.sin(cell.th) <= 0.05 * l) out.verdicts.push({ test: 'Cell geometry admissible', status: 'fail', detail: 'h/l + sinθ ≤ 0: re-entrant walls collapse onto each other. Increase h/l or reduce θ.' });
    ctx.progress(0.2, 'homogenising');
    const H = homogenize(cell, Es, t, depth), A = analytic(p.cell, cell, t, l, Es);
    const rhoEff = H.relDensity * M.rho, Kbt = H.C[0][2] / Math.sqrt(Math.abs(H.C[0][0] * H.C[2][2]) || 1);
    // verification
    const ref = homogenize(makeCell('hexagonal', { l, hr: 1, theta: 30 }), Es, t, depth), refA = analytic('hexagonal', makeCell('hexagonal', { l, hr: 1, theta: 30 }), t, l, Es);
    out.checks.push({ name: 'Regular honeycomb E₁: FE / Gibson–Ashby', value: ref.Ex / refA.Ex, target: '1 ± 0.1 for t/l ≤ 0.1 (stretching adds stiffness for thick struts)', pass: Math.abs(ref.Ex / refA.Ex - 1) < (p.tr <= 0.1 ? 0.1 : 0.35) });
    const sq = homogenize(makeCell('square', { l }), Es, t, depth); out.checks.push({ name: 'Square grid E_x / (E_s t/l)', value: sq.Ex / (Es * t / l), target: '1.000 (exact)', pass: Math.abs(sq.Ex / (Es * t / l) - 1) < 1e-6 });
    out.checks.push({ name: 'C symmetric positive-definite', value: Math.min(...H.eigC), target: '> 0', pass: H.positiveDefinite, detail: 'Eigenvalues of C: ' + H.eigC.map(v => fmt(v / 1e6, 3)).join(', ') + ' MPa' });
    const em = [0.6, -0.3, 0.4], Uhm = H.energyAt(em), Uq = 0.5 * em.reduce((s, ei, i) => s + ei * em.reduce((s2, ej, j) => s2 + H.C[i][j] * ej, 0), 0);
    out.checks.push({ name: 'Hill–Mandel energy consistency (mixed strain)', value: Uhm / Uq, target: '1.000', pass: Math.abs(Uhm / Uq - 1) < 1e-6, detail: 'Strut energy sum vs ½εᵀCε for an independent strain state' });
    // sweeps over cell angle (hexagonal family) for Poisson-ratio design map
    const thetas = linspace(-40, 40, 33).filter(v => Math.abs(v) > 1.5), sw = thetas.map(th => { const c = makeCell(th < 0 ? 'reentrant' : 'hexagonal', { l, hr: p.hr, theta: Math.abs(th) }); if (c.hh + l * Math.sin(c.th) <= 0.05 * l) return null; const r = homogenize(c, Es, t, depth), a = analytic('hexagonal', c, t, l, Es); return { th, nu: r.nuxy, nuA: a.nuxy, Ex: r.Ex, ExA: a.Ex, Ey: r.Ey, rel: r.relDensity }; }).filter(Boolean);
    const trs = linspace(0.02, 0.2, 19), swT = trs.map(tr => { const r = homogenize(cell, Es, tr * l, depth); return { tr, Ex: r.Ex, Ey: r.Ey, G: r.Gxy, nu: r.nuxy, rel: r.relDensity }; });
    ctx.progress(0.5, 'bistable element');
    // bistable cell
    const BM = getMat(p.bmat) || getMat('ti-beta21s'), bL = p.bL / 1000, bh = p.bh / 1000, bt = p.bt / 1000, bb = p.bb / 1000;
    const B = bistableBeam({ L: bL, h: bh, t: bt, b: bb, E: BM.E * 1e9, allowAsym: !!p.asym });
    const Bsym = bistableBeam({ L: bL, h: bh, t: bt, b: bb, E: BM.E * 1e9, allowAsym: false, nSteps: 80 });
    const epsY = (BM.sy || 500) / (BM.E * 1e3), elastic = B.maxStrain <= epsY;
    const envVol = bL * (bh + bt) * bb, matVol = bL * bt * bb, eDens = B.bistable ? B.Ustored / envVol : 0, eMass = B.bistable ? B.Ustored / (matVol * BM.rho) : 0;
    const qiu = 740 * BM.E * 1e9 * B.I * bh / bL ** 3, idxEnd = B.bistable ? B.d.findIndex(d => d >= B.d2) : B.d.length - 1, workInt = trapz(B.F.slice(0, idxEnd + 1), B.d.slice(0, idxEnd + 1));
    out.checks.push({ name: 'Bistable: symmetric-mode snap force / 740·EIh/L³ (Qiu et al.)', value: Bsym.Ftop / qiu, target: '1 ± 0.25 (Rayleigh–Ritz basis)', pass: Math.abs(Bsym.Ftop / qiu - 1) < 0.25 });
    out.checks.push({ name: 'Bistable: work ∫F dd vs stored energy', value: B.bistable ? workInt / B.Ustored : NaN, target: '1 ± 0.02', pass: B.bistable ? Math.abs(workInt / B.Ustored - 1) < 0.02 : undefined, detail: 'Conservative path: input work equals strain energy at the second state' });
    // KPIs
    const k = (label, value, unit, explain, prov = 'calc', status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('Effective modulus E_x', H.Ex / 1e6, 'MPa', 'Homogenised in-plane Young’s modulus along x (per unit depth) from periodic frame FE.');
    k('Effective modulus E_y', H.Ey / 1e6, 'MPa', 'Homogenised modulus along y.');
    k('Effective shear modulus G_xy', H.Gxy / 1e6, 'MPa', 'In-plane shear modulus; governs torsional compliance when used as a wing core.');
    k('Poisson ratio ν_xy', H.nuxy, '', 'Negative values = auxetic: the cell expands laterally under tension (re-entrant, chiral).', 'calc', H.nuxy < 0 ? 'ok' : undefined);
    const nuC = H.C[0][1] / H.C[1][1]; k('Poisson ratio, shear constrained', nuC, '', 'ν with in-plane shear held at zero (γ = 0), as when the lattice is bonded between stiff skins or mirrored chirality cancels the shear coupling: ν_c = C₁₂/C₂₂ (strain-controlled).', 'calc', nuC < 0 ? 'ok' : undefined);
    k('Anisotropy E_x/E_y', H.Ex / H.Ey, '', 'Directional stiffness ratio used for aeroelastic tailoring.');
    k('Shear–extension coupling', Kbt, '', 'C₁₆/√(C₁₁C₆₆): non-zero for chiral/oriented cells; laminated into a wing skin it produces bend–twist coupling.');
    k('Relative density ρ*/ρs', H.relDensity, '', 'Solid fraction of the cell (strut length × t / cell area).');
    k('Effective density', rhoEff, 'kg/m³', 'Lattice mass per volume for the selected constituent.');
    k('Specific stiffness E_x/ρ*', H.Ex / rhoEff / 1e3, 'kJ/kg', 'Stiffness per mass – compare cells at equal mass.');
    k('Bistable snap-through force', B.Ftop, 'N', 'Peak force to push the curved beam over its energy barrier (charging threshold).', 'calc');
    k('Second stable position', B.bistable ? B.d2 * 1000 : '—', 'mm', 'Mid-span travel at which the beam rests stably again (≈ 2h when bistable).', 'calc', B.bistable ? 'ok' : 'bad');
    k('Stored elastic energy per cell', B.bistable ? B.Ustored : 0, 'J', 'Strain energy held in the second stable state; released when triggered back.', 'calc');
    k('Energy density (envelope)', eDens / 1e3, 'kJ/m³', 'Stored energy per device envelope volume (L × (h+t) × b).');
    k('Specific energy (material)', eMass, 'J/kg', 'Stored energy per kg of beam material — compare with springs (≈ 100–300 J/kg for steel).');
    k('Peak bending strain', B.maxStrain * 100, '%', `Must stay below σy/E = ${fmt(epsY * 100, 3)} % for reversible, fatigue-safe cycling.`, 'calc', elastic ? 'ok' : 'bad');
    // verdicts
    { const nuC = H.C[0][1] / H.C[1][1], aux = Math.min(H.nuxy, nuC); out.verdicts.push({ test: 'Auxetic behaviour', status: p.cell === 'reentrant' || p.cell === 'tetrachiral' ? (aux < 0 ? 'pass' : 'fail') : 'info', detail: `ν_xy = ${fmt(H.nuxy, 3)} (free shear), ${fmt(nuC, 3)} (shear constrained)${p.cell === 'tetrachiral' ? '. Chiral cells convert extension into shear (coupling ' + fmt(Kbt, 3) + '); auxeticity appears when that shear is restrained.' : ''}` }); }
    out.verdicts.push({ test: 'Homogenised law is physically admissible', status: H.positiveDefinite ? 'pass' : 'fail', detail: 'C positive-definite and symmetric; usable as the effective constitutive input to Suites 06, 09, 17, 18.' });
    out.verdicts.push({ test: 'Bistability of the curved-beam cell', status: B.bistable ? 'pass' : 'fail', detail: B.bistable ? `Second stable state at ${fmt(B.d2 * 1000, 3)} mm; Q = h/t = ${fmt(B.Q, 3)}.` : `Not bistable (Q = h/t = ${fmt(B.Q, 3)}). Increase apex height or reduce thickness (Q ≳ 2.3–2.5 required).`, fix: 'Increase h/t above ≈ 2.5 while keeping peak strain below the elastic limit.' });
    out.verdicts.push({ test: 'Elastic (reversible, fatigue-safe) snap-through', status: elastic ? (B.maxStrain < 0.6 * epsY ? 'pass' : 'warn') : 'fail', detail: `Peak strain ${fmt(B.maxStrain * 100, 3)} % vs limit ${fmt(epsY * 100, 3)} % (${BM.name}).`, fix: 'Reduce t or h, or increase span L — strain scales ≈ t·h/L².' });
    // charts
    out.charts.push({ title: 'Poisson-ratio design map vs cell angle (hexagonal family)', type: 'line', x: { label: 'Cell angle θ (negative = re-entrant)', unit: '°' }, y: { label: 'ν_xy', min: -4, max: 4 }, series: [{ name: 'Periodic frame FE', x: sw.map(s => s.th), y: sw.map(s => s.nu) }, { name: 'Gibson–Ashby', x: sw.map(s => s.th), y: sw.map(s => s.nuA), dash: '5 3' }], annotations: [{ type: 'hline', y: 0, color: '#94a3b8' }], explain: 'How the in-plane Poisson ratio of a honeycomb changes with its wall angle at the current h/l and t/l. Negative angles give auxetic (re-entrant) behaviour. FE includes strut stretching, so it departs from the bending-only theory near θ = 0.' });
    out.charts.push({ title: 'Effective moduli vs cell angle', type: 'line', x: { label: 'Cell angle θ', unit: '°' }, y: { label: 'Modulus', unit: 'MPa', log: true }, series: [{ name: 'E_x FE', x: sw.map(s => s.th), y: sw.map(s => s.Ex / 1e6) }, { name: 'E_y FE', x: sw.map(s => s.th), y: sw.map(s => s.Ey / 1e6) }, { name: 'E_x Gibson–Ashby', x: sw.map(s => s.th), y: sw.map(s => s.ExA / 1e6), dash: '5 3' }], explain: 'Directional stiffness map used to tailor a graded wing core: stiff where the wing must hold shape, compliant where passive camber/twist is wanted.' });
    out.charts.push({ title: 'Scaling with strut slenderness t/l (current topology)', type: 'line', x: { label: 't/l', log: true }, y: { label: 'Modulus', unit: 'MPa', log: true }, series: [{ name: 'E_x', x: swT.map(s => s.tr), y: swT.map(s => s.Ex / 1e6) }, { name: 'E_y', x: swT.map(s => s.tr), y: swT.map(s => s.Ey / 1e6) }, { name: 'G_xy', x: swT.map(s => s.tr), y: swT.map(s => s.G / 1e6) }], explain: 'Log–log slope ≈ 3 indicates bending-dominated deformation (compliant, morphing-friendly); slope ≈ 1 indicates stretch-dominated (stiff, structural).' });
    out.charts.push({ title: 'Bistable cell force–travel curve', type: 'line', x: { label: 'Mid-span travel d', unit: 'mm' }, y: { label: 'Force', unit: 'N' }, series: [{ name: p.asym ? 'Single beam (asymmetric mode free)' : 'Constrained (symmetric)', x: B.d.map(v => v * 1000), y: B.F }, { name: 'Symmetric-mode only', x: Bsym.d.map(v => v * 1000), y: Bsym.F, dash: '5 3' }], annotations: [{ type: 'hline', y: 0, color: '#94a3b8' }, B.bistable ? { type: 'point', x: B.d2 * 1000, y: 0, label: '2nd stable state' } : { type: 'hline', y: 0 }], explain: 'Positive force = resistance during charging; the region of negative force (negative stiffness) is the snap-through. A zero crossing with positive slope beyond it is the second stable state that stores energy.' });
    out.charts.push({ title: 'Bistable cell strain-energy landscape', type: 'area', x: { label: 'Mid-span travel d', unit: 'mm' }, y: { label: 'Energy', unit: 'J' }, series: [{ name: 'Total U', x: B.d.map(v => v * 1000), y: B.U }, { name: 'Bending', x: B.d.map(v => v * 1000), y: B.Ub }, { name: 'Stretching', x: B.d.map(v => v * 1000), y: B.Us }], explain: 'Potential energy along the snap path: the hump is the energy barrier (protects against unintended release), the local minimum on the right is the stored charged state.' });
    out.tables.push({ title: 'Effective constitutive law (plane, per unit depth) — exported to Suites 06, 09, 17, 18', cols: ['', 'ε_xx', 'ε_yy', 'γ_xy'], rows: ['σ_xx', 'σ_yy', 'τ_xy'].map((r, i) => [r + ' [MPa]', ...H.C[i].map(v => v / 1e6)]), explain: 'Stiffness matrix C from homogenisation. Unit-cell geometry and this constitutive law are different deliverables and are exported separately.' });
    out.tables.push({ title: 'FE vs closed-form theory', cols: ['Property', 'Periodic FE', A.model, 'Ratio'], rows: [['E_x [MPa]', H.Ex / 1e6, A.Ex / 1e6, H.Ex / A.Ex], ['E_y [MPa]', H.Ey / 1e6, A.Ey / 1e6, H.Ey / A.Ey], ['ν_xy', H.nuxy, A.nuxy, H.nuxy / A.nuxy], ['G_xy [MPa]', H.Gxy / 1e6, A.Gxy / 1e6, H.Gxy / A.Gxy]] });
    out.tables.push({ title: 'Strut-level results for ε_xx loading', cols: ['Strut', 'Length [mm]', 'Axial force [N]', 'Max moment [N·mm]', 'Peak stress [MPa]', 'Energy share'], rows: H.els.map((e, i) => [i + 1, e.L * 1000, H.caseX.forces[i].axial, H.caseX.forces[i].Mmax * 1000, H.caseX.forces[i].sigmaMax / 1e6, H.caseX.forces[i].U / H.caseX.U]) });
    out.views.push({ label: 'Unit cell & lattice', icon: '⬡', render: (el) => el.append(latticeSVG(cell, H, p.strainViz / 100, t)) });
    out.views.unshift({ label: '3D lattice under load sweep (solved homogenisation)', icon: '🎞', render: (el) => latticeDynamic3D(el, cell, H, t, depth, p, M) });
    out.views.splice(1, 0, { label: '3D bistable snap-through (solved path)', icon: '🎞', render: (el) => bistable3D(el, B, bL, bh, bt, bb) });
    out.views.push({ label: '2D animation: unit-cell deformation & snap-through trace', icon: '🎞', render: (el) => meta2D(el, cell, H, t, B, p) });
    out.views.push({ label: '3D lattice', icon: '🧊', render: (el) => lattice3D(el, cell, H, t, depth) });
    out.views.push({ label: 'Bistable snap animation', icon: '🎞', render: (el) => bistableAnim(el, B, bL, bh, bt) });
    // recommendations
    if (p.cell === 'reentrant' && H.nuxy < 0) out.recs.push({ level: 'good', text: `Re-entrant core is auxetic (ν = ${fmt(H.nuxy, 3)}). Use it in the wing trailing region so camber curvature does not induce anticlastic bulging of the skin.`, basis: 'Auxetic synclastic curvature' });
    if (Math.abs(Kbt) < 0.02 && p.cell !== 'tetrachiral') out.recs.push({ level: 'info', text: 'This cell has negligible shear–extension coupling; passive bend–twist must then come from the spar layup (Suite 06). For coupling from the core itself, use a chiral cell or rotate the lattice orientation.', basis: 'C₁₆ ≈ 0' });
    if (!elastic) out.recs.push({ level: 'critical', text: `Bistable cell yields (strain ${fmt(B.maxStrain * 100, 3)} %). Choose a higher-resilience alloy (β-titanium, spring steel) or reduce t·h/L².`, basis: 'Elastic-limit check' });
    if (B.bistable) out.recs.push({ level: 'info', text: `Each cell stores ${fmt(B.Ustored, 3)} J. Storing 30 kJ for 200 s at 150 W would need ≈ ${fmt(30000 / B.Ustored, 3)} cells (≈ ${fmt(30000 / (eMass || 1), 3)} kg of beams). Evaluate in Suite 18 — the buffer is finite and cannot be modelled as an energy source.`, basis: 'Energy density' });
    out.recs.push({ level: 'sustain', text: `Relative density ${fmt(H.relDensity * 100, 3)} %: lattice material use is ${fmt(H.relDensity * 100, 3)} % of a solid part. Prefer recyclable constituents (Ti, Al) and specify powder reuse in additive manufacture (Suite 22).`, basis: 'Material efficiency' });
    out.recs.push({ level: 'good', text: 'Validate by printing at least three cells per design and measuring force–displacement and DIC Poisson ratio; reserve one batch for blind validation (Suite 24).', basis: 'Validation hierarchy level 2' });
    out.explain.push({ title: 'What homogenisation means here', text: 'The lattice is replaced by an equivalent continuum whose stiffness C stores the same strain energy as the struts for every average strain (Hill–Mandel). This lets the wing, chassis and buffer solvers use the metamaterial without meshing every strut, while the strut-level table tells you where stresses concentrate.' });
    out.explain.push({ title: 'Bistable energy storage', text: 'Pushing the pre-curved beam past its energy barrier moves it to a second stable shape that holds strain energy without any latch. Releasing it back delivers that energy. It is finite storage of previously harvested work, never a source.' });
    out.outputs = { cellType: p.cell, Eeff_Pa: H.Ex, Ey_Pa: H.Ey, Geff_Pa: H.Gxy, nuEff: H.nuxy, rhoRel: H.relDensity, rhoEff, C: H.C, Kbt, energyDensity_J_m3: eDens, specificEnergy_J_kg: eMass, snapForce_N: B.Ftop, storedPerCell_J: B.bistable ? B.Ustored : 0, bistable: B.bistable, cellMass_kg: matVol * BM.rho, constituent: M.id };
    out.headline = `${CELLS[p.cell]} in ${M.name}: E_x ${fmt(H.Ex / 1e6, 3)} MPa, E_y ${fmt(H.Ey / 1e6, 3)} MPa, ν ${fmt(H.nuxy, 3)}, ρ*/ρs ${fmt(H.relDensity, 3)}. Bistable cell ${B.bistable ? `stores ${fmt(B.Ustored, 3)} J` : 'is not bistable'}.`;
    return out;
  },
};

// ---------- visual helpers ----------
function strutDraw(cell, H, eps, scaleDef) {
  const E = [[eps, 0], [0, 0]], segs = [], uf = H.caseX.uf, s = eps / 1e-3; // caseX computed at εxx = 1e-3
  const fmax = Math.max(...H.caseX.forces.map(f => f.sigmaMax)) || 1;
  for (let a = -1; a <= 2; a++) for (let b = -1; b <= 2; b++) {
    const off = [a * cell.a1[0] + b * cell.a2[0], a * cell.a1[1] + b * cell.a2[1]];
    H.els.forEach((e, k) => {
      const P1 = [e.p1[0] + off[0], e.p1[1] + off[1]], P2 = [e.p2[0] + off[0], e.p2[1] + off[1]];
      const d1 = [E[0][0] * P1[0] + uf[3 * e.i] * s, uf[3 * e.i + 1] * s], d2 = [E[0][0] * P2[0] + uf[3 * e.j] * s, uf[3 * e.j + 1] * s];
      const r1 = uf[3 * e.i + 2] * s, r2 = uf[3 * e.j + 2] * s;
      segs.push({ P1, P2, Q1: [P1[0] + d1[0] * scaleDef, P1[1] + d1[1] * scaleDef], Q2: [P2[0] + d2[0] * scaleDef, P2[1] + d2[1] * scaleDef], r1: r1 * scaleDef, r2: r2 * scaleDef, sig: H.caseX.forces[k].sigmaMax / fmax, axial: H.caseX.forces[k].axial });
    });
  }
  return segs;
}
function latticeSVG(cell, H, eps, t) {
  const segs = strutDraw(cell, H, eps, 1), xs = segs.flatMap(s => [s.P1[0], s.P2[0], s.Q1[0], s.Q2[0]]), ys = segs.flatMap(s => [s.P1[1], s.P2[1], s.Q1[1], s.Q2[1]]);
  const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys), W = 640, Hh = Math.min(560, W * (y1 - y0) / (x1 - x0)), sc = Math.min((W - 20) / (x1 - x0), (Hh - 20) / (y1 - y0));
  const X = (x) => 10 + (x - x0) * sc, Y = (y) => Hh - 10 - (y - y0) * sc, sw = Math.max(1.2, t * sc);
  let svg = `<svg viewBox="0 0 ${W} ${Hh}" width="100%" style="background:var(--panel2);border-radius:10px">`;
  for (const s of segs) svg += `<line x1="${X(s.P1[0])}" y1="${Y(s.P1[1])}" x2="${X(s.P2[0])}" y2="${Y(s.P2[1])}" stroke="#94a3b8" stroke-opacity=".35" stroke-width="${sw}" stroke-linecap="round"/>`;
  for (const s of segs) { const L = Math.hypot(s.Q2[0] - s.Q1[0], s.Q2[1] - s.Q1[1]), ang = Math.atan2(s.Q2[1] - s.Q1[1], s.Q2[0] - s.Q1[0]); const c1 = [s.Q1[0] + L / 3 * Math.cos(ang + s.r1), s.Q1[1] + L / 3 * Math.sin(ang + s.r1)], c2 = [s.Q2[0] - L / 3 * Math.cos(ang + s.r2), s.Q2[1] - L / 3 * Math.sin(ang + s.r2)]; svg += `<path d="M${X(s.Q1[0])},${Y(s.Q1[1])} C${X(c1[0])},${Y(c1[1])} ${X(c2[0])},${Y(c2[1])} ${X(s.Q2[0])},${Y(s.Q2[1])}" fill="none" stroke="${cssColor(cmap(s.sig, 'turbo'))}" stroke-width="${sw}" stroke-linecap="round"/>`; }
  svg += `</svg>`;
  return h('div', {}, h('div', { html: svg }), h('p', { class: 'small muted' }, `Grey: undeformed 4 × 4 tessellation. Coloured: deformed under ε_xx = ${fmt(eps * 100, 3)} % (exact periodic FE fluctuation field, end rotations drawn as cubic curves), colour = peak strut stress (blue low → red high). Lateral contraction or expansion of the tile visualises the Poisson ratio ν = ${fmt(H.nuxy, 3)}.`));
}
async function lattice3D(el, cell, H, t, depth) {
  const [{ viewer }, { box }] = await Promise.all([import('../core/viz3d.js'), import('../core/cad.js')]);
  const V = await viewer(el, { height: 420, grid: false }); const fmax = Math.max(...H.caseX.forces.map(f => f.sigmaMax)) || 1;
  for (let a = 0; a < 4; a++) for (let b = 0; b < 4; b++) {
    const off = [a * cell.a1[0] + b * cell.a2[0], a * cell.a1[1] + b * cell.a2[1]];
    H.els.forEach((e, k) => { const L = e.L, m = box(`strut ${k + 1}`, L + t, t, depth, { color: cssColor(cmap(H.caseX.forces[k].sigmaMax / fmax, 'turbo')), group: 'Lattice' }); m.rotZ(Math.atan2(e.p2[1] - e.p1[1], e.p2[0] - e.p1[0])); m.translate((e.p1[0] + e.p2[0]) / 2 + off[0], (e.p1[1] + e.p2[1]) / 2 + off[1], 0); V.add(m, { metalness: 0.6, roughness: 0.35 }); });
  }
  V.fit(); el.append(h('p', { class: 'small muted' }, 'Extruded 4 × 4 lattice (manufacturing geometry), struts coloured by peak stress under ε_xx. Use GLB to export for printing preparation.'));
}
function bistableAnim(el, B, L, hgt, t) {
  const W = 640, Hh = 260, n = 80, xs = range(n + 1).map(i => L * i / n), k = 2 * Math.PI / L;
  const shape = (a) => xs.map(x => a[0] * (1 - Math.cos(k * x)) + a[1] * (1 - Math.cos(2 * k * x)) + a[2] * (1 - Math.cos(3 * k * x)) + a[3] * Math.sin(k * x) * (1 - Math.cos(k * x)));
  const sc = (W - 40) / L, Y = (w) => Hh / 2 - w * sc * 0.9, X = (x) => 20 + x * sc;
  const svg = h('div', { html: `<svg viewBox="0 0 ${W} ${Hh}" width="100%" style="background:var(--panel2);border-radius:10px"><path id="b0" fill="none" stroke="#94a3b8" stroke-dasharray="4 3"/><path id="b1" fill="none" stroke="#38bdf8" stroke-width="${Math.max(2, t * sc)}"/><text id="bt" x="12" y="20" fill="currentColor" font-size="12"></text></svg>` });
  const s0 = shape(B.coeffs[0]); svg.querySelector('#b0').setAttribute('d', 'M' + xs.map((x, i) => `${X(x)},${Y(s0[i])}`).join('L'));
  const slider = h('input', { type: 'range', min: 0, max: B.d.length - 1, value: 0, style: 'width:100%' });
  const draw = (i) => { const s = shape(B.coeffs[i]); svg.querySelector('#b1').setAttribute('d', 'M' + xs.map((x, j) => `${X(x)},${Y(s[j])}`).join('L')); svg.querySelector('#bt').textContent = `travel ${fmt(B.d[i] * 1000, 3)} mm · force ${fmt(B.F[i], 3)} N · energy ${fmt(B.U[i], 3)} J`; };
  slider.oninput = () => draw(+slider.value); draw(0);
  let playing = true, i = 0, seen = false; const tick = () => { if (el.isConnected) seen = true; else if (seen) return; if (playing) { i = (i + 1) % B.d.length; slider.value = i; draw(i); } setTimeout(tick, 60); }; tick();
  el.append(svg, slider, h('button', { class: 'btn sm', onclick: (e) => { playing = !playing; e.target.textContent = playing ? 'Pause' : 'Play'; } }, 'Pause'), h('p', { class: 'small muted' }, `Solved equilibrium shapes along the displacement-controlled path (not an interpolated cartoon). Apex ${fmt(hgt * 1000, 3)} mm, thickness ${fmt(t * 1000, 3)} mm.`));
}

// ---------------- 3-D lattice under a macroscopic load sweep ----------------
// Solved: periodic frame-FE unit-cell responses to three independent macroscopic strains (affine + periodic
// fluctuation displacements and strut end forces) and the homogenised compliance S. Displayed: the 4 × 4
// tessellation deformed by the linear superposition for uniaxial σxx, σyy and shear τxy paths (ε = S·σ), so the
// Poisson (auxetic) response is the solved one. Deformation magnified for visibility; colour = solved axial stress.
async function latticeDynamic3D(el, cell, H, t, depth, p, M) {
  const [{ viewer }, { playback, hud, legend3d }, { cmap }] = await Promise.all([import('../core/viz3d.js'), import('../core/sim3d.js'), import('../core/chart.js')]);
  const epsIn = h('input', { type: 'range', min: 0.2, max: 10, step: 0.1, value: Math.min(10, p.strainViz || 4) }), epsL = h('span', { class: 'small mono' }, '');
  el.append(h('p', { class: 'small muted' }, 'Sequence: uniaxial stress σxx → σyy → shear τxy, each loaded and unloaded. Macroscopic strains follow ε = S·σ from the solved homogenised compliance, so lateral contraction (or expansion for auxetic cells) is the computed Poisson response. Node displacements = affine strain field + solved periodic fluctuation; colour = solved strut axial stress (linear superposition). Displacements are drawn at the selected macroscopic strain amplitude (the linear solution scaled to it — no extra magnification); colour range is fixed over the whole load path.'), h('label', {}, 'Peak macroscopic strain (display) ', epsIn, epsL));
  const box = h('div'); el.append(box);
  const V = await viewer(box, { height: 460, grid: false }), THREE = V.THREE, n = 4;
  const cases = [H.caseX, H.caseY, H.caseS], unit = 1e-3; // solved for 1e-3 strain each
  const struts = [], geo = new THREE.BoxGeometry(1, 1, 1);
  for (let a = 0; a < n; a++) for (let b = 0; b < n; b++) { const off = [a * cell.a1[0] + b * cell.a2[0], a * cell.a1[1] + b * cell.a2[1]]; H.els.forEach((e, k) => { const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.5, roughness: 0.4 })); V.root.add(mesh); struts.push({ e, k, off, mesh }); }); }
  // node spheres at joints (visual continuity)
  const sgeo = new THREE.SphereGeometry(t * 0.6, 10, 8), joints = []; for (const st of struts) for (const end of [1, 2]) { const m = new THREE.Mesh(sgeo, new THREE.MeshStandardMaterial({ color: 0xcbd5e1, metalness: 0.5, roughness: 0.4 })); V.root.add(m); joints.push({ st, end, m }); }
  const H2 = hud(box), sAll = []; for (const c of cases) for (const f of c.forces) sAll.push(Math.abs(f.axial)); const leg = legend3d(box, { title: 'Strut axial stress (MPa)', lo: 0, hi: 1, cmapName: 'coolwarm' });
  const T = 18;
  // fixed colour range: peak strut axial stress over the whole load path at the chosen amplitude (solved, linear)
  const sRange = (Amp) => { let m = 1e-12; for (let ph = 0; ph < 3; ph++) { const e = [0, 1, 2].map(i => H.S[i][ph]), mx = Math.max(...e.map(Math.abs)) || 1, coef = e.map(v => v / mx * Amp / unit); for (const st of struts.slice(0, H.els.length)) m = Math.max(m, Math.abs(coef.reduce((s2, c, i) => s2 + c * cases[i].forces[st.k].axial, 0) / (depth * st.e.tt))); } return m; };
  const pathAt = (tt) => { const ph = Math.floor((tt % T) / (T / 3)), u = ((tt % T) % (T / 3)) / (T / 3), lam = Math.sin(Math.PI * u), sig = [0, 0, 0]; sig[ph] = 1; const e = [0, 1, 2].map(i => H.S[i][ph]); const mx = Math.max(...e.map(Math.abs)) || 1; return { ph, lam, eps: e.map(v => v / mx) }; };
  const pb = playback(box, { duration: T, label: 'Load path time', onFrame: (tt) => {
    const { ph, lam, eps } = pathAt(tt), A = (+epsIn.value / 100) * lam, coef = eps.map(v => v * A / unit);
    const E = [[eps[0] * A, eps[2] * A / 2], [eps[2] * A / 2, eps[1] * A]], aff = (q) => [E[0][0] * q[0] + E[0][1] * q[1], E[1][0] * q[0] + E[1][1] * q[1]];
    const fl = (node) => [0, 1].map(d => coef.reduce((s2, c, i) => s2 + c * cases[i].uf[3 * node + d], 0));
    let smax = 1e-12; const sig = struts.map(st => { const ax = coef.reduce((s2, c, i) => s2 + c * cases[i].forces[st.k].axial, 0) / (depth * st.e.tt); smax = Math.max(smax, Math.abs(ax)); return ax; }); const sFix = sRange(+epsIn.value / 100);
    struts.forEach((st, i) => {
      const p1 = [st.e.p1[0] + st.off[0], st.e.p1[1] + st.off[1]], p2 = [st.e.p2[0] + st.off[0], st.e.p2[1] + st.off[1]], a1 = aff(p1), a2 = aff(p2), f1 = fl(st.e.i), f2 = fl(st.e.j);
      const q1 = [p1[0] + a1[0] + f1[0], p1[1] + a1[1] + f1[1]], q2 = [p2[0] + a2[0] + f2[0], p2[1] + a2[1] + f2[1]], L = Math.hypot(q2[0] - q1[0], q2[1] - q1[1]);
      st.mesh.position.set((q1[0] + q2[0]) / 2, (q1[1] + q2[1]) / 2, 0); st.mesh.rotation.set(0, 0, Math.atan2(q2[1] - q1[1], q2[0] - q1[0])); st.mesh.scale.set(L + t * 0.2, st.e.tt, depth);
      const c = cmap(0.5 + 0.5 * sig[i] / sFix, 'coolwarm'); st.mesh.material.color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255); st.q1 = q1; st.q2 = q2;
    });
    joints.forEach(j => { const q = j.end === 1 ? j.st.q1 : j.st.q2; j.m.position.set(q[0], q[1], 0); });
    leg.set(-sFix / 1e6, sFix / 1e6);
    H2.set([['Load path', ['uniaxial σxx', 'uniaxial σyy', 'shear τxy'][ph]], ['ε_xx', fmt(eps[0] * A * 100, 3) + ' %'], ['ε_yy', fmt(eps[1] * A * 100, 3) + ' %'], ['γ_xy', fmt(eps[2] * A * 100, 3) + ' %'], ['Effective ν_xy (solved)', fmt(H.nuxy, 3)], ['Peak |σ_axial|', fmt(smax / 1e6, 3) + ' MPa']]);
  } });
  epsIn.oninput = () => { epsL.textContent = epsIn.value + ' %'; }; epsIn.oninput();
  V.fit(); V.viewDir(0, -0.001, 1.4);
  void M;
  return { dispose: () => pb.dispose() };
}

// ---------------- 3-D bistable cell: solved snap-through path ----------------
async function bistable3D(el, B, L, hgt, t, b) {
  const [{ viewer }, { playback, hud, legend3d }, { cmap }] = await Promise.all([import('../core/viz3d.js'), import('../core/sim3d.js'), import('../core/chart.js')]);
  el.append(h('p', { class: 'small muted' }, 'Clamped–clamped pre-curved beam animated through the solved displacement-controlled equilibrium path (Rayleigh–Ritz coefficients at each step) — including the snap-through to the second stable state. Colour = bending surface strain t/2·|w″| from the same solution. True scale.'));
  const box = h('div'); el.append(box);
  const V = await viewer(box, { height: 420, grid: false }), THREE = V.THREE, n = 120, k = 2 * Math.PI / L, xs = range(n + 1).map(i => L * i / n);
  const shape = (a, x) => a[0] * (1 - Math.cos(k * x)) + a[1] * (1 - Math.cos(2 * k * x)) + a[2] * (1 - Math.cos(3 * k * x)) + a[3] * Math.sin(k * x) * (1 - Math.cos(k * x));
  const curv = (a, x) => a[0] * k * k * Math.cos(k * x) + a[1] * 4 * k * k * Math.cos(2 * k * x) + a[2] * 9 * k * k * Math.cos(3 * k * x) + a[3] * k * k * (-Math.sin(k * x) + 2 * Math.sin(2 * k * x));
  const pos = new Float32Array((n + 1) * 4 * 3), col = new Float32Array((n + 1) * 4 * 3), idx = [];
  for (let i = 0; i < n; i++) for (let f = 0; f < 4; f++) { const a = i * 4 + f, bq = i * 4 + (f + 1) % 4, c = (i + 1) * 4 + (f + 1) % 4, d = (i + 1) * 4 + f; idx.push(a, bq, c, a, c, d); }
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3)); g.setAttribute('color', new THREE.BufferAttribute(col, 3)); g.setIndex(idx);
  const mesh = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ vertexColors: true, side: THREE.DoubleSide, metalness: 0.5, roughness: 0.35 })); V.root.add(mesh);
  const clampM = new THREE.MeshStandardMaterial({ color: 0x475569 }); for (const x0 of [0, L]) { const c = new THREE.Mesh(new THREE.BoxGeometry(L * 0.06, b * 1.4, hgt * 2.5 + t * 4), clampM); c.position.set(x0 + (x0 ? L * 0.03 : -L * 0.03), 0, 0); V.root.add(c); }
  const ref = new THREE.Line(new THREE.BufferGeometry().setFromPoints(xs.map(x => new THREE.Vector3(x, b / 2 + 1e-4, shape(B.coeffs[0], x)))), new THREE.LineDashedMaterial({ color: 0x94a3b8 })); V.root.add(ref);
  const H2 = hud(box), epsMax = Math.max(...B.coeffs.map(a => Math.max(...xs.map(x => Math.abs(curv(a, x))))) ) * t / 2, leg = legend3d(box, { title: 'Bending strain (%)', lo: 0, hi: epsMax * 100 });
  const steps = B.d.length, T = 8;
  const pb = playback(box, { duration: T, label: 'Path time', onFrame: (tt) => {
    const u = tt / T, si = Math.min(steps - 1, Math.floor((u < 0.5 ? 2 * u : 2 - 2 * u) * (steps - 1))), a = B.coeffs[si];
    xs.forEach((x, i) => { const w = shape(a, x), e = Math.abs(curv(a, x)) * t / 2, c = cmap(e / (epsMax || 1), 'turbo'); const corners = [[-b / 2, -t / 2], [b / 2, -t / 2], [b / 2, t / 2], [-b / 2, t / 2]]; corners.forEach(([y, dz], f) => { pos.set([x, y, w + dz], (i * 4 + f) * 3); col.set([c[0] / 255, c[1] / 255, c[2] / 255], (i * 4 + f) * 3); }); });
    g.attributes.position.needsUpdate = true; g.attributes.color.needsUpdate = true; g.computeVertexNormals();
    H2.set([['Apex travel', fmt(B.d[si] * 1000, 3) + ' mm'], ['Force (solved)', fmt(B.F[si], 3) + ' N', B.F[si] < 0 ? '#ef4444' : undefined], ['Stored energy', fmt(B.U[si] * 1000, 3) + ' mJ'], ['State', B.bistable && B.d[si] >= B.d2 ? 'second stable state' : B.F[si] < 0 ? 'snap-through (negative stiffness)' : 'loading']]);
  } });
  V.fit(); V.viewDir(0.2, -1.1, 0.35);
  void hgt;
  return { dispose: () => pb.dispose() };
}

// ---------------- 2-D animation: 3 × 3 lattice under the solved load path + bistable force–displacement trace ----------------
async function meta2D(el, cell, H, t, B, p) {
  const { anim2d, axes2d, polyline } = await import('../core/sim3d.js');
  el.append(h('p', { class: 'small muted' }, 'Left: 3 × 3 cells deformed by the solved homogenisation fields along uniaxial σxx → σyy → shear (ε = S·σ, affine + periodic fluctuation), at the visualised strain amplitude; struts coloured by solved axial force sign. Right: the bistable cell’s solved force–displacement path with a tracing marker and the current beam shape.'));
  const cases = [H.caseX, H.caseY, H.caseS], T = 12, n = 3, epsA = (p.strainViz || 4) / 100;
  const pts = []; for (let a = 0; a < n; a++) for (let b = 0; b < n; b++) pts.push([a * cell.a1[0] + b * cell.a2[0], a * cell.a1[1] + b * cell.a2[1]]);
  const all = pts.flatMap(o => H.els.flatMap(e => [[e.p1[0] + o[0], e.p1[1] + o[1]], [e.p2[0] + o[0], e.p2[1] + o[1]]])), xs = all.map(q => q[0]), ys = all.map(q => q[1]), bx = [Math.min(...xs), Math.max(...xs)], by = [Math.min(...ys), Math.max(...ys)];
  const Lb = (B.d && B.d.length) ? B.d : [0], kk = 2 * Math.PI / (p.bL / 1000), shape = (a, x) => a[0] * (1 - Math.cos(kk * x)) + a[1] * (1 - Math.cos(2 * kk * x)) + a[2] * (1 - Math.cos(3 * kk * x)) + a[3] * Math.sin(kk * x) * (1 - Math.cos(kk * x));
  return anim2d(el, { duration: T, label: 'Time', height: 400, draw: (g, tt, W, Hh, c) => {
    const ph = Math.floor((tt % T) / (T / 3)), u = ((tt % T) % (T / 3)) / (T / 3), lam = Math.sin(Math.PI * u), e = [0, 1, 2].map(i => H.S[i][ph]), mx = Math.max(...e.map(Math.abs)) || 1, coef = e.map(v => v / mx * epsA * lam / 1e-3);
    const E = [[e[0] / mx * epsA * lam, e[2] / mx * epsA * lam / 2], [e[2] / mx * epsA * lam / 2, e[1] / mx * epsA * lam]], aff = (q) => [q[0] + E[0][0] * q[0] + E[0][1] * q[1], q[1] + E[1][0] * q[0] + E[1][1] * q[1]];
    const fl = (node) => [0, 1].map(d => coef.reduce((acc, cf, i) => acc + cf * cases[i].uf[3 * node + d], 0));
    const hw = W / 2 - 20, sc = Math.min((hw - 40) / ((bx[1] - bx[0]) * 1.3), (Hh - 80) / ((by[1] - by[0]) * 1.3)), X = (x) => 30 + (x - bx[0] + 0.15 * (bx[1] - bx[0])) * sc, Y = (y) => Hh - 50 - (y - by[0] + 0.15 * (by[1] - by[0])) * sc;
    for (const o of pts) H.els.forEach((el2, k) => { const p1 = [el2.p1[0] + o[0], el2.p1[1] + o[1]], p2 = [el2.p2[0] + o[0], el2.p2[1] + o[1]], f1 = fl(el2.i), f2 = fl(el2.j), q1 = aff(p1), q2 = aff(p2); const ax = coef.reduce((acc, cf, i) => acc + cf * cases[i].forces[k].axial, 0); g.strokeStyle = ax > 0 ? '#ef4444' : ax < 0 ? '#3b82f6' : c.mute; g.lineWidth = 2; g.beginPath(); g.moveTo(X(q1[0] + f1[0]), Y(q1[1] + f1[1])); g.lineTo(X(q2[0] + f2[0]), Y(q2[1] + f2[1])); g.stroke(); });
    g.fillStyle = c.fg; g.fillText(`${['uniaxial σxx', 'uniaxial σyy', 'shear τxy'][ph]} · εxx ${fmt(E[0][0] * 100, 3)} % · εyy ${fmt(E[1][1] * 100, 3)} % (ν_xy = ${fmt(H.nuxy, 3)})`, 30, 18);
    if (!B.F) return; const si = Math.min(Lb.length - 1, Math.floor(((tt % T) / T) * Lb.length * 2) % (2 * Lb.length - 1)), idx = si < Lb.length ? si : 2 * Lb.length - 2 - si;
    const A2 = axes2d(g, { x0: hw + 70, y0: 30, w: hw - 60, h: Hh * 0.55 - 30, xr: [0, Math.max(...Lb) * 1e3], yr: [Math.min(...B.F) * 1.1, Math.max(...B.F) * 1.1], xl: 'Apex travel (mm)', yl: 'Force (N)', fg: c.fg, mute: c.mute });
    polyline(g, Lb.map((d, i) => [A2.X(d * 1e3), A2.Y(B.F[i])]), '#22c55e', 2); g.fillStyle = '#facc15'; g.beginPath(); g.arc(A2.X(Lb[idx] * 1e3), A2.Y(B.F[idx]), 5, 0, 7); g.fill();
    const L = p.bL / 1000, bx0 = hw + 70, bw = hw - 60, by0 = Hh * 0.82, sY = bw / L * 1.5; polyline(g, range(81).map(i => { const x = L * i / 80; return [bx0 + x / L * bw, by0 - shape(B.coeffs[idx], x) * sY]; }), '#38bdf8', 3);
    g.fillStyle = c.fg; g.fillText(`F = ${fmt(B.F[idx], 3)} N · d = ${fmt(Lb[idx] * 1000, 3)} mm${B.F[idx] < 0 ? ' · snap-through' : ''}`, hw + 70, Hh - 12);
  } });
}
