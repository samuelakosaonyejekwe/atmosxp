// Suite 09 — Carbon-composite monocoque, structural analysis and crashworthiness.
import { lamina, laminate, parseLayup, plyStresses, structuredMesh, meshQuality, planeStress, lowestModes, cstB, planeStressD, bandCholesky } from '../core/fem.js';
import { fmt, gci, max, min, SparseBuilder, csrMul, dot } from '../core/num.js';
import { MATERIALS, get as getMat } from '../core/materials.js';
import { cmap, cssColor } from '../core/chart.js';
import { h } from '../core/ui.js';

const G = 9.80665;
const faceMats = MATERIALS.filter(m => m.uses.includes('chassis') && m.family !== 'Core');
const coreMats = MATERIALS.filter(m => m.family === 'Core');
const SEA = { cfrp: 80, afrp: 55, gfrp: 50, nfrp: 35, al: 25, mg: 30, ti: 30, steel: 15, core: 10 }; // kJ/kg, crush tubes (literature means)

function profile(veh) {
  const L = veh.length, Hb = veh.height, R = veh.wheelRadius;
  const halfH = (x) => { const u = Math.min(1, Math.max(0, (x + L * 0.42) / L)); return Hb * 0.36 * Math.max(0.06, Math.sin(Math.PI * u) ** 0.7); };
  const cz = (x) => R + 0.02 + halfH(x) * 0.9;
  return { x0: -0.42 * L, x1: 0.58 * L, ybot: (x) => cz(x) - halfH(x), ytop: (x) => cz(x) + halfH(x), halfH };
}

function analyse(p, ctx, nx, ny, keep = false) {
  const veh = ctx.vehicle, P = profile(veh), mesh = structuredMesh(P.x0 + 0.04 * veh.length, P.x1 - 0.04 * veh.length, P.ybot, P.ytop, nx, ny);
  const FM = getMat(p.faceMat) || getMat('cfrp-woven'), L = lamina(FM), lam = laminate(parseLayup(p.layup, L, p.tPly / 1000)), hf = lam.h;
  const CM = getMat(p.coreMat) || getMat('core-pet'), tc = p.tCore / 1000;
  const tWall = 2 * 2 * hf; // two side walls, two faces each (in-plane load carried by faces)
  // floor & roof as flanges lumped into bottom/top element rows (their area / row height)
  const W = veh.width * 0.8, flangeA = W * 2 * hf;
  const nE = mesh.tris.length, tEl = new Float64Array(nE), cen = [];
  const rowOf = (tri) => { const jj = tri.map(k => k % (ny + 1)); return Math.min(...jj); };
  mesh.tris.forEach((tri, e) => { const r = rowOf(tri), P0 = tri.map(k => mesh.nodes[k]); const yc = (P0[0][1] + P0[1][1] + P0[2][1]) / 3, xc = (P0[0][0] + P0[1][0] + P0[2][0]) / 3; cen.push([xc, yc]); const rowH = (P.ytop(xc) - P.ybot(xc)) / ny; tEl[e] = tWall + ((r === 0 || r === ny - 1) ? flangeA / Math.max(rowH, 1e-3) : 0); });
  // assemble with variable thickness: scale stiffness per element by thickness via repeated calls (planeStress supports scalar t) -> use per-element wrapper
  const totalMass = veh.mass, occupant = Math.min(veh.payload || 90, totalMass * 0.5);
  const areaShell = 2 * Math.PI * (veh.length * 0.5) * ((veh.width + veh.height) / 2) * 0.55; // approximate monocoque surface area
  const shellMass = areaShell * (2 * lam.rhoA + tc * (CM.rho || 100));
  const wingMass = (ctx.upstream('s06')?.wingMass ?? 22) * (veh.nWings || 2);
  const other = Math.max(0, totalMass - occupant - shellMass - wingMass);
  const loads = {}; const add = (d, v) => loads[d] = (loads[d] || 0) + v;
  const nearest = (x, y) => { let bi = 0, bd = Infinity; mesh.nodes.forEach((n, k) => { const d = (n[0] - x) ** 2 + (n[1] - y) ** 2; if (d < bd) { bd = d; bi = k; } }); return bi; };
  const nodeMass = new Float64Array(mesh.nodes.length);
  // distributed shell + other components by element area; occupant at the cockpit floor; wing masses at roots
  let Atot = 0; const Ael = mesh.tris.map(tri => { const [a, b, c] = tri.map(k => mesh.nodes[k]); const A = Math.abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2; Atot += A; return A; });
  mesh.tris.forEach((tri, e) => { const m = (shellMass + other) * Ael[e] / Atot; tri.forEach(k => nodeMass[k] += m / 3); });
  const seatX = 0.1 * veh.length - 0.1, seat = [nearest(seatX - 0.25, P.ybot(seatX - 0.25)), nearest(seatX + 0.25, P.ybot(seatX + 0.25))]; seat.forEach(k => nodeMass[k] += occupant / 2);
  // wing-root stations from the parametric CAD (Revision-2 tandem layout: gap ≥ 1.12 root chords)
  const crW = (veh.wingArea / (veh.nWings || 2) / veh.wingHeight) * 1.1, x1W = -veh.length * 0.42 + 0.10 * veh.length + 0.25 * crW, wingX = [x1W, x1W + Math.max(0.36 * veh.length, 1.12 * crW)].slice(0, veh.nWings || 2);
  const roots = wingX.map(x => nearest(x, P.ytop(x))); roots.forEach(k => nodeMass[k] += wingMass / roots.length);
  const az = p.loadFactor * G, ax = p.brakeDecel; // vertical load factor; braking deceleration (inertia acts forward)
  nodeMass.forEach((m, k) => { add(2 * k + 1, -m * az); if (p.case !== 'vertical') add(2 * k, m * ax * (p.case === 'braking' || p.case === 'combined' ? 1 : 0)); });
  // wing aerodynamic loads at roots: thrust (forward) and its pitching couple (thrust × CE height)
  const thrust = (p.case === 'vertical' ? 0 : p.wingThrust) / roots.length, ceH = p.ceHeight, arm = 0.3;
  roots.forEach((k, i) => { add(2 * k, thrust); const xr = mesh.nodes[k][0], kf = nearest(xr + arm, P.ytop(xr + arm)), kb = nearest(xr - arm, P.ytop(xr - arm)), Fc = thrust * ceH / (2 * arm); add(2 * kb + 1, Fc); add(2 * kf + 1, -Fc); void i; });
  // supports: tyre vertical springs at contact nodes; rear contact fixed horizontally
  const wb = veh.wheelbase, xf = 0.58 * wb, xrr = -0.42 * wb, kf = nearest(xf, P.ybot(xf)), kr = nearest(xrr, P.ybot(xrr));
  const nFront = veh.layout === 'delta' ? 1 : 2, nRear = 4 - 1 - nFront + (veh.nWheels === 4 ? 1 : 0);
  const springs = { [2 * kf + 1]: p.kTyre * 1e3 * nFront, [2 * kr + 1]: p.kTyre * 1e3 * Math.max(1, nRear) }, fixed = { [2 * kr]: 0 };
  // variable thickness: planeStress uses a single t, so scale the constitutive matrix per element through a thickness map
  const sol = planeStressVar(mesh, { E: lam.Ex, nu: Math.min(0.45, Math.max(0.05, lam.nuxy)), tEl, fixed, springs, loads, nodeMass });
  // results per element: face strains -> ply Tsai–Wu strength ratio; von Mises in the face laminate (equivalent)
  const els = sol.els.map((e, i) => { const ps = plyStresses(lam, [e.eps[0], e.eps[1], e.eps[2]]); return { ...e, R: Math.min(...ps.map(q => q.R)), c: cen[i] }; });
  // bending deflection relative to the wheel line
  const ur = sol.u[2 * kr + 1], uf = sol.u[2 * kf + 1], xa = mesh.nodes[kr][0], xb = mesh.nodes[kf][0];
  const rel = mesh.nodes.map((n, k) => sol.u[2 * k + 1] - (ur + (uf - ur) * (n[0] - xa) / (xb - xa)));
  const qual = meshQuality(mesh.nodes, mesh.tris);
  const out = { mesh, sol, els, rel, lam, hf, tc, FM, CM, qual, kf, kr, roots, nodeMass, shellMass, wingMass, occupant, other, maxRel: max(rel.map(Math.abs)), probe: null };
  // probe: mid-wheelbase bottom flange axial stress (smooth quantity for mesh convergence)
  const xm = (xa + xb) / 2; let bi = 0, bd = Infinity; els.forEach((e, i) => { const d = Math.abs(e.c[0] - xm) + 5 * Math.abs(e.c[1] - P.ybot(xm)); if (d < bd) { bd = d; bi = i; } }); out.probe = els[bi].sig[0];
  if (keep) { // structural modes with near-rigid wheel stations (separates elastic bending from heave/pitch on tyres)
    const stiff = Object.fromEntries(Object.entries(springs).map(([d2, k2]) => [d2, k2 * 1e4]));
    const R2 = planeStressVar(mesh, { E: lam.Ex, nu: Math.min(0.45, Math.max(0.05, lam.nuxy)), tEl, fixed, springs: stiff, loads, nodeMass });
    out.solStiff = R2;
  }
  return out;
}

// Plane-stress solve with per-element thickness (thin wrapper around fem.planeStress assembly logic)
function planeStressVar(mesh, { E, nu, tEl, fixed, springs, loads, nodeMass }) {
  // planeStress assembles t*A*BᵀDB; we pass unit thickness and pre-scale by building a mesh copy per thickness group
  const groups = new Map(); mesh.tris.forEach((t, e) => { const key = tEl[e].toPrecision(6); if (!groups.has(key)) groups.set(key, []); groups.get(key).push(e); });
  if (groups.size === 1) { const r = planeStress(mesh, { E, nu, t: tEl[0], fixed, springs, loads }); r.Mdiag = Float64Array.from({ length: 2 * mesh.nodes.length }, (_, d) => nodeMass[d >> 1]); return r; }
  // general case: assemble with effective modulus E·t_e / t_ref (identical stiffness to thickness scaling)
  const tref = Math.min(...tEl), Escaled = { E, nu }; void Escaled;
  const sub = { nodes: mesh.nodes, tris: mesh.tris, Escale: Array.from(tEl, t => t / tref) };
  const r = planeStressScaled(sub, { E, nu, t: tref, fixed, springs, loads });
  r.Mdiag = Float64Array.from({ length: 2 * mesh.nodes.length }, (_, d) => nodeMass[d >> 1]);
  return r;
}
function planeStressScaled(mesh, { E, nu, t, fixed, springs, loads }) {
  const nN = mesh.nodes.length, n = 2 * nN, D = planeStressD(E, nu), sb = new SparseBuilder(n), F = new Float64Array(n), els = [];
  mesh.tris.forEach((tri, e) => {
    const P = tri.map(k => mesh.nodes[k]), { B, A } = cstB(P), map = [2 * tri[0], 2 * tri[0] + 1, 2 * tri[1], 2 * tri[1] + 1, 2 * tri[2], 2 * tri[2] + 1], s = mesh.Escale[e];
    const DB = D.map(r => [0, 1, 2, 3, 4, 5].map(j => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));
    for (let a = 0; a < 6; a++) for (let b = 0; b < 6; b++) sb.add(map[a], map[b], s * t * A * (B[0][a] * DB[0][b] + B[1][a] * DB[1][b] + B[2][a] * DB[2][b]));
    els.push({ tri, B, A, map });
  });
  for (const [d, v] of Object.entries(loads)) F[+d] += v;
  for (const [d, k] of Object.entries(springs)) sb.add(+d, +d, k);
  const isFixed = new Uint8Array(n); for (const d of Object.keys(fixed)) isFixed[+d] = 1;
  const freeIdx = new Int32Array(n).fill(-1), freeList = []; for (let d = 0; d < n; d++) if (!isFixed[d]) { freeIdx[d] = freeList.length; freeList.push(d); }
  const full = sb.toCSR(), rb = new SparseBuilder(freeList.length), Fr = new Float64Array(freeList.length);
  for (let i = 0; i < n; i++) { if (isFixed[i]) continue; const ri = freeIdx[i]; Fr[ri] += F[i]; for (let k = full.ptr[i]; k < full.ptr[i + 1]; k++) { const j = full.col[k]; if (!isFixed[j]) rb.add(ri, freeIdx[j], full.val[k]); } }
  const Kr = rb.toCSR(), fac = bandCholesky(Kr), x = fac.solve(Fr), rr = csrMul(Kr, x); let rn = 0, bn = 0; for (let i = 0; i < Fr.length; i++) { rn += (rr[i] - Fr[i]) ** 2; bn += Fr[i] ** 2; } const solv = { x, res: Math.sqrt(rn / (bn || 1)), ok: Math.sqrt(rn / (bn || 1)) < 1e-9, it: 'direct', bandwidth: fac.bandwidth }, u = new Float64Array(n); for (let d = 0; d < n; d++) u[d] = isFixed[d] ? 0 : solv.x[freeIdx[d]];
  const res = els.map(e => { const ue = e.map.map(d => u[d]); const eps = e.B.map(r => r.reduce((s, v, k) => s + v * ue[k], 0)); const sig = D.map(r => r[0] * eps[0] + r[1] * eps[1] + r[2] * eps[2]); return { eps, sig, vm: Math.sqrt(sig[0] ** 2 - sig[0] * sig[1] + sig[1] ** 2 + 3 * sig[2] ** 2) }; });
  const Ku = csrMul(full, u), work = dot(Array.from(F), Array.from(u)), Ustrain = 0.5 * dot(Array.from(Ku), Array.from(u));
  let sumF = [0, 0], sumR = [0, 0]; for (let d = 0; d < n; d++) { sumF[d % 2] += F[d]; if (isFixed[d] || springs[d] !== undefined) sumR[d % 2] += Ku[d] - F[d] + (springs[d] !== undefined ? 0 : 0); }
  // spring reactions: k·u at spring DOFs
  let Rspring = [0, 0]; for (const [d, k] of Object.entries(springs)) Rspring[+d % 2] += k * u[+d];
  return { u, els: res, Kr, fac, freeList, freeIdx, isFixed, F, solver: solv, work, Ustrain, sumF, Rspring };
}

export default {
  description: 'Structural analysis of the carbon-sandwich monocoque: plane-stress finite elements of the chassis side section with floor/roof flanges, loaded by real wing-root forces, occupant and component masses, braking inertia and road load factors on compliant tyre supports. Delivers deformation, stress and Tsai–Wu first-ply-failure maps, laminate failure indices, closed-section torsional stiffness, sandwich panel buckling and face wrinkling, natural frequencies, crash energy management, mesh-convergence (GCI) and a fully-stressed mass-optimisation proposal.',
  literature: 'carbon fibre sandwich monocoque chassis torsional stiffness crashworthiness specific energy absorption',
  credibility: 'Numerically verified',
  equations: ['3D/plane-stress elastic equilibrium ∇·σ + b = 0, strain–displacement compatibility, generalised Hooke’s law', 'Classical laminate theory (ABD) for the faces; ply stresses by transformation; Tsai–Wu, Tsai–Hill, maximum stress and Hashin failure criteria', 'Sandwich theory: face bending D = E_f h_f d²/2, core shear stiffness S = G_c d; shear-flexible panel buckling N_cr = kπ²D/b² /(1 + kπ²D/(b²S)); face wrinkling σ_wr = 0.5(E_f E_c G_c)^{1/3}', 'Bredt–Batho closed thin-walled torsion GJ = 4A²/∮ds/(Gt)', 'Generalised eigenproblem K φ = ω² M φ for natural frequencies (subspace iteration)', 'Crashworthiness: kinetic energy ½mv², specific energy absorption of progressive composite crushing, mean crush force F = m·a_lim, stroke = E/F', 'Discretisation error: Richardson extrapolation and grid-convergence index'],
  methods: ['Constant-strain triangle FE on a structured body-fitted mesh, sparse assembly, Jacobi-preconditioned conjugate gradients', 'Supports as tyre springs (vertical) + rear longitudinal restraint — no unrealistic fully fixed supports', 'Three-level mesh refinement (ratio 1.5) on a smooth probe quantity'],
  assumptions: ['Side-section model: two side walls carry in-plane loads; floor and roof are lumped as flanges in the outer element rows', 'Faces carry membrane loads, the core only shear (classical sandwich assumption)', 'Lateral wing loads are evaluated through closed-section torsion of the body; full 3D shell analysis is a future refinement', 'Linear elastic, small displacement; crash by energy method (no explicit dynamics)'],
  ibc: { initial: ['Undeformed geometry from the parametric CAD body profile; zero initial stress; laminate cure residual stress neglected'], boundary: ['Tyre contact springs at front and rear axle stations, rear longitudinal restraint', 'Gravity × load factor on distributed shell/component masses, occupant at the seat, wing masses at roots', 'Wing-root thrust and pitching couple from Suites 06/08; braking inertia from Suite 12', 'Torsion from the wing overturning moment reacted by the wheels'] },
  cvv: { calibration: ['Coupon and laminate tests of the face material; core shear tests', 'Full-structure static stiffness tests (bending and torsion rigs)', 'Modal test of the bare chassis'], verification: ['Global equilibrium: Σ reactions = Σ applied loads', 'Energy consistency: external work = 2 × strain energy', 'Mesh convergence GCI on a smooth probe stress', 'Mesh quality: minimum angle, scaled Jacobian'], validation: ['Strain-gauge and DIC measurements on the prototype under prescribed loads (reserved data set)', 'Drop/crush tests of crash-box specimens'] },
  formats: { import: ['CSV strain-gauge data', 'STL/OBJ scanned chassis geometry (inspection)', 'Gmsh MSH meshes'], export: ['Nastran BDF / Abaqus INP (via mesh nodes & elements CSV)', 'VTK legacy mesh with stress field', 'CSV element results', 'PNG/SVG contours'] },
  params: [
    { key: 'faceMat', label: 'Face material', type: 'select', group: 'Laminate', default: 'cfrp-woven', options: faceMats.map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.material('chassis')?.id || 'cfrp-woven', prov: 'library' }) },
    { key: 'layup', label: 'Face layup (deg)', type: 'text', group: 'Laminate', default: '0,45,-45,0' },
    { key: 'tPly', label: 'Ply thickness', unit: 'mm', type: 'number', min: 0.02, max: 2, step: 0.005, group: 'Laminate', default: 0.25, auto: (c) => { const o = c.upstream('s01'); if (!o?.skin_mm) return undefined; const n = String(c.store.params('s09').layup || '0,45,-45,0').split(/[,/ ]+/).filter(Boolean).length || 4; return { value: +(o.skin_mm / n).toFixed(4), prov: 'coupled' }; }, help: 'Coupled from Suite 01 face thickness ÷ number of plies in the layup.' },
    { key: 'coreMat', label: 'Core material', type: 'select', group: 'Laminate', default: 'core-pet', options: coreMats.map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.material('chassisCore')?.id || 'core-pet', prov: 'library' }) },
    { key: 'tCore', label: 'Core thickness', unit: 'mm', type: 'number', min: 2, max: 60, step: 0.5, group: 'Laminate', default: 15, auto: (c) => { const o = c.upstream('s01'); return o?.core_mm ? { value: +o.core_mm, prov: 'coupled' } : undefined; } },
    { key: 'case', label: 'Load case', type: 'select', group: 'Loads', default: 'combined', options: [{ value: 'combined', label: 'Combined: bump + braking + wing thrust' }, { value: 'vertical', label: 'Vertical bump only' }, { value: 'braking', label: 'Emergency braking + wing thrust' }] },
    { key: 'loadFactor', label: 'Vertical load factor', unit: 'g', type: 'number', min: 1, max: 5, step: 0.1, group: 'Loads', default: 3, help: 'Road-bump design factor on all masses.' },
    { key: 'brakeDecel', label: 'Braking deceleration', unit: 'm/s²', type: 'number', min: 0, max: 12, step: 0.1, group: 'Loads', default: 7.8, auto: (c) => { const o = c.upstream('s12'); return o?.decel_ms2 ? { value: +o.decel_ms2.toFixed(2), prov: 'coupled' } : undefined; } },
    { key: 'pylonType', label: 'Centreline pylon flexure concept', type: 'select', group: 'Centreline pylon flexure (Rev 5.1)', default: 'cage', options: [{ value: 'cage', label: 'Ti rod cage + negative-stiffness compensator (statically balanced)' }, { value: 'hybrid', label: 'Ti torsion rod + elastomer shear pads' }, { value: 'cruciform', label: 'Ti cruciform leaves' }, { value: 'slitTube', label: 'Slit Ti tube' }] },
    { key: 'cageN', label: 'Cage: number of Ti rods', type: 'number', min: 3, max: 12, step: 1, group: 'Centreline pylon flexure (Rev 5.1)', default: 6 },
    { key: 'cageD', label: 'Cage: rod diameter', unit: 'mm', type: 'number', min: 3, max: 25, step: 0.1, group: 'Centreline pylon flexure (Rev 5.1)', default: 11.0 },
    { key: 'cageL', label: 'Cage: rod free length', unit: 'mm', type: 'number', min: 100, max: 800, step: 10, group: 'Centreline pylon flexure (Rev 5.1)', default: 550 },
    { key: 'cageR', label: 'Cage: rod pitch radius', unit: 'mm', type: 'number', min: 20, max: 200, step: 1, group: 'Centreline pylon flexure (Rev 5.1)', default: 105 },
    { key: 'stopGap', label: 'Roll-over stop gap at the crossbeam ends', unit: 'mm', type: 'number', min: 0.5, max: 30, step: 0.5, group: 'Centreline pylon flexure (Rev 5.1)', default: 1 },
    { key: 'trimTol', label: 'Compensator trim tolerance at assembly', unit: '%', type: 'number', min: 0.1, max: 10, step: 0.1, group: 'Centreline pylon flexure (Rev 5.1)', default: 0.5 },
    { key: 'driftRes', label: 'Residual thermal stiffness drift (same-alloy compensator)', unit: '%', type: 'number', min: 0, max: 5, step: 0.1, group: 'Centreline pylon flexure (Rev 5.1)', default: 0.3, help: 'Ti modulus drifts ≈ 1.6 % over ±40 K; a compensator of the same alloy drifts with it, leaving only the residual mismatch.' },
    { key: 'rodD', label: 'Torsion rod diameter', unit: 'mm', type: 'number', min: 3, max: 20, step: 0.1, group: 'Centreline pylon flexure (Rev 5.1)', default: 5.5 },
    { key: 'rodL', label: 'Torsion rod length (into the keel)', unit: 'mm', type: 'number', min: 100, max: 1200, step: 10, group: 'Centreline pylon flexure (Rev 5.1)', default: 800 },
    { key: 'preload', label: 'Rod clamping preload', unit: 'kN', type: 'number', min: 0, max: 30, step: 0.5, group: 'Centreline pylon flexure (Rev 5.1)', default: 12 },
    { key: 'padR', label: 'Shear-pad radius from the axis', unit: 'mm', type: 'number', min: 80, max: 500, step: 5, group: 'Centreline pylon flexure (Rev 5.1)', default: 250 },
    { key: 'padW', label: 'Shear-pad width (square)', unit: 'mm', type: 'number', min: 30, max: 200, step: 1, group: 'Centreline pylon flexure (Rev 5.1)', default: 70 },
    { key: 'padH', label: 'Shear-pad elastomer height (laminated)', unit: 'mm', type: 'number', min: 10, max: 150, step: 1, group: 'Centreline pylon flexure (Rev 5.1)', default: 65 },
    { key: 'padG', label: 'Elastomer shear modulus', unit: 'MPa', type: 'number', min: 0.2, max: 1.5, step: 0.05, group: 'Centreline pylon flexure (Rev 5.1)', default: 0.3 },
    { key: 'crucB', label: 'Cruciform leaf width', unit: 'mm', type: 'number', min: 10, max: 200, step: 1, group: 'Centreline pylon flexure (Rev 5.1)', default: 70 },
    { key: 'crucT', label: 'Cruciform leaf thickness', unit: 'mm', type: 'number', min: 0.5, max: 10, step: 0.1, group: 'Centreline pylon flexure (Rev 5.1)', default: 5.2 },
    { key: 'crucL', label: 'Cruciform length', unit: 'mm', type: 'number', min: 50, max: 600, step: 5, group: 'Centreline pylon flexure (Rev 5.1)', default: 300 },
    { key: 'tubeD', label: 'Slit tube diameter', unit: 'mm', type: 'number', min: 30, max: 200, step: 1, group: 'Centreline pylon flexure (Rev 5.1)', default: 100 },
    { key: 'tubeT', label: 'Slit tube wall', unit: 'mm', type: 'number', min: 0.5, max: 6, step: 0.1, group: 'Centreline pylon flexure (Rev 5.1)', default: 2 },
    { key: 'tubeL', label: 'Slit tube length', unit: 'mm', type: 'number', min: 50, max: 600, step: 5, group: 'Centreline pylon flexure (Rev 5.1)', default: 300 },
    { key: 'flexAmp', label: 'Fatigue yaw amplitude', unit: '°', type: 'number', min: 1, max: 15, step: 0.5, group: 'Centreline pylon flexure (Rev 5.1)', default: 8 },
    { key: 'archMass', label: 'Arch + crossbeam mass on the pylon', unit: 'kg', type: 'number', min: 5, max: 80, step: 0.5, group: 'Centreline pylon flexure (Rev 5.1)', default: 28.8 },
    { key: 'rollM', label: 'Roll-over lateral moment at the pylon', unit: 'N·m', type: 'number', min: 0, max: 10000, step: 50, group: 'Centreline pylon flexure (Rev 5.1)', default: 2750 },
    { key: 'negPerG', label: 'Axial-load negative yaw stiffness', unit: 'N·m/rad per g', type: 'number', min: 0, max: 5, step: 0.01, group: 'Centreline pylon flexure (Rev 5.1)', default: 0.62 },
    { key: 'stopK', label: 'Progressive elastomer stop stiffness', unit: 'N·m/rad', type: 'number', min: 100, max: 20000, step: 100, group: 'Centreline pylon flexure (Rev 5.1)', default: 2000 },
    { key: 'insR', label: 'Base-plate insert pattern radius (4 inserts)', unit: 'mm', type: 'number', min: 50, max: 500, step: 5, group: 'Centreline pylon flexure (Rev 5.1)', default: 220 },
    { key: 'bridgeSparD', label: 'Bridge roll-over tube diameter (0 = none)', unit: 'mm', type: 'number', min: 0, max: 120, step: 1, group: 'Centreline pylon flexure (Rev 5.1)', default: 70, help: 'Carbon tube inside the bridge section; the bridge doubles as the roll-over hoop. Must fit the bridge thickness.' },
    { key: 'bridgeSparT', label: 'Bridge roll-over tube wall', unit: 'mm', type: 'number', min: 0.5, max: 5, step: 0.1, group: 'Centreline pylon flexure (Rev 5.1)', default: 1.2 },
    { key: 'bootT', label: 'Root boot membrane thickness', unit: 'mm', type: 'number', min: 0.3, max: 5, step: 0.1, group: 'Centreline pylon flexure (Rev 5.1)', default: 1.2 },
    { key: 'bootG', label: 'Root boot elastomer shear modulus', unit: 'MPa', type: 'number', min: 0.1, max: 2, step: 0.05, group: 'Centreline pylon flexure (Rev 5.1)', default: 0.4 },
    { key: 'bootGamMax', label: 'Boot allowable shear strain at the stop', type: 'number', min: 0.3, max: 4, step: 0.05, group: 'Centreline pylon flexure (Rev 5.1)', default: 1.5 },
    { key: 'bootGamFat', label: 'Boot allowable shear-strain amplitude (10⁷ cycles)', type: 'number', min: 0.1, max: 1.5, step: 0.05, group: 'Centreline pylon flexure (Rev 5.1)', default: 0.5 },
    { key: 'brushMu', label: 'Brush seal friction coefficient', type: 'number', min: 0.05, max: 0.8, step: 0.01, group: 'Centreline pylon flexure (Rev 5.1)', default: 0.3 },
    { key: 'brushPn', label: 'Brush contact preload', unit: 'N/m', type: 'number', min: 0, max: 50, step: 0.5, group: 'Centreline pylon flexure (Rev 5.1)', default: 1.5 },
    { key: 'brushW', label: 'Bristle pack width', unit: 'mm', type: 'number', min: 1, max: 20, step: 0.5, group: 'Centreline pylon flexure (Rev 5.1)', default: 4 },
    { key: 'brushLen', label: 'Usable bristle wear length', unit: 'mm', type: 'number', min: 1, max: 20, step: 0.5, group: 'Centreline pylon flexure (Rev 5.1)', default: 5 },
    { key: 'brushK', label: 'Bristle wear coefficient (Archard)', unit: 'm³/(N·m)', type: 'number', min: 1e-17, max: 1e-12, step: 1e-17, group: 'Centreline pylon flexure (Rev 5.1)', default: 1e-14 },
    { key: 'kAeroLow', label: 'Aerodynamic yaw stiffness at light wind', unit: 'N·m/rad', type: 'number', min: 1, max: 500, step: 1, group: 'Centreline pylon flexure (Rev 5.1)', default: 30 },
    { key: 'gustRateH', label: 'Significant yaw oscillations per operating hour', type: 'number', min: 10, max: 10000, step: 10, group: 'Centreline pylon flexure (Rev 5.1)', default: 600 },
    { key: 'hoursYr', label: 'Operating hours per year', type: 'number', min: 50, max: 5000, step: 50, group: 'Centreline pylon flexure (Rev 5.1)', default: 800 },
    { key: 'insertCap', label: 'Potted Ti insert capacity', unit: 'kN', type: 'number', min: 1, max: 20, step: 0.5, group: 'Centreline pylon flexure (Rev 5.1)', default: 6 },
    { key: 'frameT', label: 'Mast-step bulkhead ring-frame laminate', unit: 'mm', type: 'number', min: 0.5, max: 20, step: 0.1, group: 'Wing-root load path (Rev 2.1)', default: 6, help: 'Sized for SF ≥ 2 on the deployed breakaway moment (Revision 2.2).' },
    { key: 'frameD', label: 'Ring-frame depth (flange)', unit: 'mm', type: 'number', min: 10, max: 200, step: 1, group: 'Wing-root load path (Rev 2.1)', default: 70 },
    { key: 'stepDepth', label: 'Mast partner-to-step bearing separation', unit: 'm', type: 'number', min: 0.1, max: 1, step: 0.01, group: 'Wing-root load path (Rev 2.1)', default: 0.35 },
    { key: 'keelW', label: 'Keel beam width × height', unit: 'mm', type: 'text', group: 'Wing-root load path (Rev 2.1)', default: '80x60' },
    { key: 'keelT', label: 'Keel beam wall (CFRP box)', unit: 'mm', type: 'number', min: 0.5, max: 10, step: 0.1, group: 'Wing-root load path (Rev 2.1)', default: 2.5 },
    { key: 'gustFactor', label: 'Root-load dynamic factor', type: 'number', min: 1, max: 3, step: 0.05, group: 'Wing-root load path (Rev 2.1)', default: 1.5, auto: (c) => { const o = c.upstream('s08'); return o?.gustPeakRootMoment && o?.rootMomentStatic ? { value: +Math.max(1, o.gustPeakRootMoment / o.rootMomentStatic).toFixed(2), prov: 'coupled' } : undefined; } },
    { key: 'wingThrust', label: 'Wing thrust (all wings)', unit: 'N', type: 'number', min: 0, max: 5000, step: 5, group: 'Loads', default: 300, auto: (c) => { const o = c.upstream('s08') || c.upstream('s06'); return o?.thrustN !== undefined ? { value: Math.round(Math.abs(o.thrustN)), prov: 'coupled' } : undefined; } },
    { key: 'ceHeight', label: 'Wing centre of effort above root', unit: 'm', type: 'number', min: 0.3, max: 4, step: 0.05, group: 'Loads', default: 1.4, auto: (c) => ({ value: +((c.vehicle.wingHeight) * 0.46).toFixed(2), prov: 'calc' }) },
    { key: 'rollMoment', label: 'Wing overturning moment (torsion case)', unit: 'N·m', type: 'number', min: 0, max: 20000, step: 10, group: 'Loads', default: 900, auto: (c) => { const o = c.upstream('s06'); const m8 = c.upstream('s08')?.gustPeakRootMoment; if (m8) return { value: Math.round(m8 * (c.vehicle.nWings || 2)), prov: 'coupled' }; return o?.rollMomentN ? { value: Math.round(o.rollMomentN), prov: 'coupled' } : undefined; } },
    { key: 'openKnock', label: 'Cockpit-opening torsional retention factor', type: 'range', min: 0.03, max: 1, step: 0.01, group: 'Stability', default: 0.15, help: 'Fraction of closed-tube torsional stiffness retained with an open cockpit (≈ 0.1–0.2 with a non-structural canopy, ≈ 0.5–0.8 with a bonded structural canopy frame).' },
    { key: 'kTyre', label: 'Tyre vertical stiffness (per tyre)', unit: 'kN/m', type: 'number', min: 20, max: 500, step: 5, group: 'Supports', default: 120 },
    { key: 'nx', label: 'Mesh divisions along length', type: 'number', min: 20, max: 160, step: 2, group: 'Mesh', default: 96, help: 'Default refined (Revision 2, D10) so that the three-level GCI on deflection is below 5 %.' },
    { key: 'ny', label: 'Mesh divisions through height', type: 'number', min: 4, max: 24, step: 1, group: 'Mesh', default: 12 },
    { key: 'panelA', label: 'Largest unsupported panel length', unit: 'm', type: 'number', min: 0.2, max: 2, step: 0.05, group: 'Stability', default: 0.7 },
    { key: 'vCrash', label: 'Frontal impact speed', unit: 'km/h', type: 'number', min: 10, max: 60, step: 1, group: 'Crash', default: 30 },
    { key: 'gLim', label: 'Occupant deceleration limit', unit: 'g', type: 'number', min: 5, max: 40, step: 1, group: 'Crash', default: 20 },
    { key: 'sfTarget', label: 'Target strength ratio', type: 'number', min: 1.2, max: 4, step: 0.1, group: 'Design targets', default: 2.0 },
    { key: 'ktTarget', label: 'Target torsional stiffness', unit: 'kN·m/deg', type: 'number', min: 0.5, max: 30, step: 0.5, group: 'Design targets', default: 3 },
  ],
  presets: [{ label: 'Bump 3 g', values: { case: 'vertical', loadFactor: 3 } }, { label: 'Emergency braking', values: { case: 'braking', brakeDecel: 9 } }, { label: 'Glass faces', values: { faceMat: 'gfrp-e', layup: '0,45,-45,90,90,-45,45,0', tPly: 0.3 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const veh = ctx.vehicle; ctx.progress(0.1, 'assembling FE model');
    const A = analyse(p, ctx, Math.round(p.nx), Math.round(p.ny), true);
    const { els, lam, FM, CM, sol, mesh } = A, hf = A.hf, tc = A.tc;
    const vmMax = max(els.map(e => e.vm)), Rmin = min(els.map(e => e.R)), iCrit = els.findIndex(e => e.R === Rmin);
    ctx.progress(0.35, 'mesh refinement study');
    const r = 1.5, lv = [[p.nx / (r * r), p.ny / (r * r)], [p.nx / r, p.ny / r], [p.nx, p.ny]].map(([a, b]) => [Math.max(8, Math.round(a)), Math.max(3, Math.round(b))]);
    const studies = lv.map(([a, b]) => { const s = a === Math.round(p.nx) && b === Math.round(p.ny) ? A : analyse(p, ctx, a, b); return { nx: a, ny: b, n: s.mesh.tris.length, probe: s.probe, defl: s.maxRel }; });
    const hs = studies.map(s => 1 / Math.sqrt(s.n)), g1 = gci(studies[2].defl, studies[1].defl, studies[0].defl, hs[2], hs[1], hs[0]), g2 = gci(studies[2].probe, studies[1].probe, studies[0].probe, hs[2], hs[1], hs[0]);
    ctx.progress(0.55, 'modal analysis');
    const Mr = Float64Array.from(A.sol.freeList, d => Math.max(1e-6, A.sol.Mdiag[d]));
    let modes = [], modesS = []; try { modes = lowestModes(A.sol.Kr, Mr, 3, 18, A.sol.fac); modesS = lowestModes(A.solStiff.Kr, Float64Array.from(A.solStiff.freeList, d => Math.max(1e-6, A.solStiff.Mdiag[d])), 3, 18, A.solStiff.fac); } catch (e) { void e; }
    // torsion (closed section at max body section)
    const a = veh.width / 2 * 0.82, b = veh.height * 0.36, n = 200; let Ar = 0, Per = 0; for (let k = 0; k < n; k++) { const t1 = 2 * Math.PI * k / n, t2 = 2 * Math.PI * (k + 1) / n, pt = (t) => [a * Math.sign(Math.cos(t)) * Math.abs(Math.cos(t)) ** (2 / 2.6), b * Math.sign(Math.sin(t)) * Math.abs(Math.sin(t)) ** (2 / 2.6)]; const p1 = pt(t1), p2 = pt(t2); Ar += 0.5 * (p1[0] * p2[1] - p2[0] * p1[1]); Per += Math.hypot(p2[0] - p1[0], p2[1] - p1[1]); }
    const Gt = 2 * lam.A[2][2], GJ = p.openKnock * 4 * Ar * Ar * Gt / Per, Kt = GJ / veh.wheelbase, KtDeg = Kt * Math.PI / 180, twist = p.rollMoment * veh.wheelbase * 0.5 / GJ * 180 / Math.PI;
    const tauTorsion = p.rollMoment / (2 * Ar) / (2 * hf); // face shear stress from Bredt flow
    // sandwich stability
    const Ef = lam.Ex, d = tc + hf, D = Ef * hf * d * d / 2, Gc = (CM.G12 ?? CM.E * 0.3) * 1e9, Ec = (CM.E ?? 0.1) * 1e9, S = Gc * d, bpan = veh.height * 0.72 * 0.5;
    const Ncr = 4 * Math.PI ** 2 * D / bpan ** 2 / (1 + 4 * Math.PI ** 2 * D / (bpan ** 2 * S)), sigComp = Math.abs(Math.min(0, min(els.map(e => e.sig[0])))), Nx = sigComp * 2 * hf, BF = Nx > 0 ? Ncr / Nx : Infinity;
    const sWr = 0.5 * Math.cbrt(Ef * Ec * Gc), WF = sigComp > 0 ? sWr / sigComp : Infinity;
    // crash
    const v = p.vCrash / 3.6, Ek = 0.5 * veh.mass * v * v, sea = (SEA[FM.cls] ?? 30) * 1e3, crushMass = Ek / (sea * 0.8), Fmean = veh.mass * p.gLim * G, stroke = Ek / Fmean, overhang = 0.58 * (veh.length - veh.wheelbase);
    // mass & optimisation (fully-stressed scaling of faces, buckling ~ t, torsion ~ t)
    const chassisMass = A.shellMass, scale = Math.max(p.sfTarget / Rmin, p.ktTarget * 1e3 / KtDeg, 1.5 / Math.max(BF, 1e-9)), faceMassNow = A.shellMass * 2 * lam.rhoA / (2 * lam.rhoA + tc * (CM.rho || 100));
    const newMass = chassisMass + faceMassNow * (scale - 1);
    // checks
    const Fy = sol.sumF[1], Ry = A.sol.Rspring[1]; out.checks.push({ name: 'Vertical equilibrium Σ spring reactions / Σ loads', value: Ry / (Fy || 1), target: '1.000', pass: Math.abs(Ry - Fy) / Math.abs(Fy || 1) < 1e-4, detail: `Σ F_y = ${fmt(Fy, 5)} N, Σ R_y = ${fmt(Ry, 5)} N` });
    out.checks.push({ name: 'Energy consistency W_ext / (2 U)', value: sol.work / (2 * sol.Ustrain), target: '1.000', pass: Math.abs(sol.work / (2 * sol.Ustrain) - 1) < 1e-3 });
    out.checks.push({ name: 'Linear solver residual (banded Cholesky)', value: sol.solver.res, target: '< 1e-9', pass: sol.solver.ok, detail: `half-bandwidth ${sol.solver.bandwidth}` });
    out.checks.push({ name: 'Mesh quality: minimum angle', value: A.qual.minAngle, target: '> 10°', pass: A.qual.minAngle > 10 });
    out.checks.push({ name: 'Mesh quality: minimum scaled Jacobian', value: A.qual.minScaledJacobian, target: '> 0.2', pass: A.qual.minScaledJacobian > 0.2 });
    out.checks.push({ name: 'GCI — max bending deflection', value: g1.gci21 * 100, target: '< 5 %', pass: g1.gci21 < 0.05, detail: `p = ${fmt(g1.p, 3)}, extrapolated ${fmt(g1.fext * 1000, 4)} mm` });
    out.checks.push({ name: 'GCI — mid-wheelbase floor stress', value: g2.gci21 * 100, target: '< 5 %', pass: g2.gci21 < 0.05, detail: `p = ${fmt(g2.p, 3)}` });
    // KPIs
    const k = (label, value, unit, explain, status, prov = 'calc') => out.kpis.push({ label, value, unit, explain, status, prov });
    k('Max equivalent (von Mises) face stress', vmMax / 1e6, 'MPa', 'Peak in-plane equivalent stress in the face laminate (indicator; composite failure uses Tsai–Wu below).');
    k('Min Tsai–Wu strength ratio', Rmin, '', 'Load multiplier to first-ply failure at the most critical element.', Rmin >= p.sfTarget ? 'ok' : Rmin >= 1.2 ? 'warn' : 'bad');
    k('Max bending deflection', A.maxRel * 1000, 'mm', 'Chassis deflection relative to the line between the wheel contacts (tyre compression removed).');
    k('Torsional stiffness', KtDeg / 1e3, 'kN·m/deg', 'Closed-section (Bredt) torsional stiffness over the wheelbase — resists wing overturning moments and preserves handling.', KtDeg / 1e3 >= p.ktTarget ? 'ok' : 'warn');
    k('Twist under wing overturning moment', twist, '°', `Rotation between wing roots and axles for ${fmt(p.rollMoment, 4)} N·m.`);
    k('Torsional face shear stress', tauTorsion / 1e6, 'MPa', 'Bredt shear flow divided by the two face thicknesses.');
    k('Panel buckling factor', BF, '', 'Critical / applied compressive line load of the largest side panel (shear-flexible sandwich).', BF >= 1.5 ? 'ok' : 'bad');
    k('Face-wrinkling factor', WF, '', 'Wrinkling stress / maximum compressive face stress.', WF >= 1.5 ? 'ok' : 'bad');
    k('Heave / pitch on tyres', `${fmt(modes[0]?.f ?? NaN, 3)} / ${fmt(modes[1]?.f ?? NaN, 3)}`, 'Hz', 'Lowest modes of the loaded vehicle bouncing and pitching on its tyre springs (ride modes).');
    k('1st elastic bending frequency', modesS[0]?.f ?? NaN, 'Hz', 'Lowest structural (elastic) mode with wheel stations held; keep clear of wheel-hub cartridge passing and wing frequencies.', (modesS[0]?.f ?? 0) > 25 ? 'ok' : 'warn');
    k('Monocoque shell mass (est.)', chassisMass, 'kg', 'Faces + core over the estimated body surface area.');
    k('Crash energy to absorb', Ek / 1e3, 'kJ', `Kinetic energy at ${p.vCrash} km/h for ${veh.mass} kg.`);
    k('Required crush stroke', stroke, 'm', `At a mean deceleration of ${p.gLim} g; available front overhang ≈ ${fmt(overhang, 3)} m.`, stroke < 0.8 * overhang ? 'ok' : 'bad');
    k('Crash-structure mass needed', crushMass, 'kg', `Progressive crushing of ${FM.name} (SEA ≈ ${SEA[FM.cls] ?? 30} kJ/kg, 80 % crush efficiency).`);
    k('Mesh elements / DOFs', `${mesh.tris.length} / ${2 * mesh.nodes.length}`, '', 'Selected mesh (refinement study in Verification and Tables).');
    // verdicts
    out.verdicts.push({ test: 'First-ply strength', status: Rmin >= p.sfTarget ? 'pass' : Rmin >= 1.2 ? 'warn' : 'fail', detail: `Min strength ratio ${fmt(Rmin, 3)} (target ${p.sfTarget}) at x = ${fmt(els[iCrit].c[0], 3)} m, y = ${fmt(els[iCrit].c[1], 3)} m.`, fix: 'Locally thicken the faces (doublers) at the critical zone or add 0° plies along the load path.' });
    out.verdicts.push({ test: 'Torsional stiffness target', status: KtDeg / 1e3 >= p.ktTarget ? 'pass' : 'fail', detail: `${fmt(KtDeg / 1e3, 3)} kN·m/deg vs ${p.ktTarget} target.`, fix: 'Add ±45° plies, enlarge the enclosed section, or close cockpit openings with a structural canopy frame.' });
    out.verdicts.push({ test: 'Sandwich panel stability', status: BF >= 1.5 && WF >= 1.5 ? 'pass' : BF >= 1 && WF >= 1 ? 'warn' : 'fail', detail: `Buckling factor ${fmt(BF, 3)}, wrinkling factor ${fmt(WF, 3)}.`, fix: 'Increase core thickness (buckling ∝ d²) or use a stiffer core (wrinkling ∝ (E_c G_c)^{1/3}).' });
    out.verdicts.push({ test: 'Crash survival space', status: stroke < 0.8 * overhang ? 'pass' : 'fail', detail: `Stroke ${fmt(stroke, 3)} m needed vs ${fmt(overhang, 3)} m available.`, fix: 'Lower impact speed rating, extend the nose, or accept a higher (still survivable) mean deceleration with load-limiting seats.' });
    out.verdicts.push({ test: 'Mesh independence', status: g1.gci21 < 0.05 ? 'pass' : 'warn', detail: `GCI ${fmt(g1.gci21 * 100, 3)} % on deflection, observed order ${fmt(g1.p, 3)}.` });
    // charts
    const xsN = mesh.nodes.filter((_, kk) => kk % (Math.round(p.ny) + 1) === 0);
    out.charts.push({ title: 'Bending deflection along the chassis (bottom line)', type: 'line', x: { label: 'x (rear → front)', unit: 'm' }, y: { label: 'Deflection', unit: 'mm' }, series: [{ name: 'Relative to wheel line', x: xsN.map(n2 => n2[0]), y: mesh.nodes.map((_, kk) => kk).filter(kk => kk % (Math.round(p.ny) + 1) === 0).map(kk => A.rel[kk] * 1000) }, { name: 'Absolute (incl. tyre compression)', x: xsN.map(n2 => n2[0]), y: mesh.nodes.map((_, kk) => kk).filter(kk => kk % (Math.round(p.ny) + 1) === 0).map(kk => A.sol.u[2 * kk + 1] * 1000), dash: '4 3' }], annotations: [{ type: 'vline', x: mesh.nodes[A.kr][0], label: 'rear axle' }, { type: 'vline', x: mesh.nodes[A.kf][0], label: 'front axle' }], explain: 'Deflected floor line: the relative curve is the structural bending; the dashed curve includes tyre compression.' });
    const byX = new Map(); els.forEach(e => { const key = Math.round(e.c[0] * 20) / 20; const r0 = byX.get(key); if (!r0 || e.R < r0) byX.set(key, e.R); }); const keys = [...byX.keys()].sort((x1, x2) => x1 - x2);
    out.charts.push({ title: 'Critical strength ratio along the chassis', type: 'line', x: { label: 'x', unit: 'm' }, y: { label: 'Min Tsai–Wu strength ratio', log: true }, series: [{ name: 'min over section', x: keys, y: keys.map(kk => byX.get(kk)) }], annotations: [{ type: 'hline', y: p.sfTarget, label: 'target', color: '#ef4444' }], explain: 'Lowest first-ply-failure margin at each station. Dips mark wing roots, seat attachments and wheel stations needing local reinforcement.' });
    out.charts.push({ title: 'Mesh refinement study', type: 'line', x: { label: 'Elements', log: true }, y: { label: 'Normalised value' }, series: [{ name: 'Max deflection / extrapolated', x: studies.map(s => s.n), y: studies.map(s => s.defl / g1.fext), mode: 'lines+markers' }, { name: 'Probe stress / extrapolated', x: studies.map(s => s.n), y: studies.map(s => s.probe / g2.fext), mode: 'lines+markers' }], annotations: [{ type: 'hline', y: 1, color: '#22c55e' }], explain: 'Convergence of smooth quantities with mesh refinement (ratio 1.5). Values approaching 1 indicate mesh-independent results; point-load peak stresses are singular and are not used for convergence.' });
    if (modes.length) out.charts.push({ title: 'Natural frequencies vs excitation bands', type: 'bar', categories: modes.map((m, i) => `Mode ${i + 1}`), y: { label: 'Frequency', unit: 'Hz' }, series: [{ name: 'On tyre springs (ride)', values: modes.map(m => m.f) }, { name: 'Wheel stations held (elastic)', values: modes.map((_, i) => modesS[i]?.f ?? NaN) }], showValues: true, explain: 'Lowest vibration modes of the loaded chassis. Wheel rotation (≈ 3 Hz at 20 km/h), cartridge passing (× number of cartridges) and wing modes (Suite 06) should not coincide with these.' });
    out.tables.push({ title: 'Mesh refinement levels', cols: ['nx × ny', 'Elements', 'Max deflection [mm]', 'Probe σx [MPa]'], rows: studies.map(s => [`${s.nx} × ${s.ny}`, s.n, s.defl * 1000, s.probe / 1e6]).concat([['Extrapolated', '', g1.fext * 1000, g2.fext / 1e6]]) });
    out.tables.push({ title: 'Face laminate (CLT)', cols: ['Property', 'Value'], rows: [['Layup', p.layup], ['Face thickness [mm]', hf * 1000], ['E_x [GPa]', lam.Ex / 1e9], ['E_y [GPa]', lam.Ey / 1e9], ['G_xy [GPa]', lam.Gxy / 1e9], ['ν_xy', lam.nuxy], ['Areal mass per face [kg/m²]', lam.rhoA], ['Core', CM.name], ['Core thickness [mm]', tc * 1000], ['Sandwich D [N·m]', D], ['Core shear stiffness S [N/m]', S]] });
    out.tables.push({ title: 'Mass distribution used for loads', cols: ['Item', 'Mass [kg]'], rows: [['Monocoque shell', A.shellMass], ['Occupant', A.occupant], ['Wings', A.wingMass], ['Other components', A.other], ['Total', veh.mass]] });
    out.tables.push({ title: 'Critical element ply stresses (Tsai–Wu)', cols: ['Ply [°]', 'σ₁ [MPa]', 'σ₂ [MPa]', 'τ₁₂ [MPa]', 'Tsai–Wu', 'R', 'Hashin fibre', 'Hashin matrix'], rows: plyStresses(lam, els[iCrit].eps).map(q => [q.theta, q.s1 / 1e6, q.s2 / 1e6, q.t12 / 1e6, q.tsaiWu, q.R, q.hashinFiber, q.hashinMatrix]) });
    out.views.push({ label: 'Stress & deformation contours', icon: '🗺', render: (el) => contourView(el, A) });
    out.views.unshift({ label: '3D dynamic monocoque: load ramp & mode shapes', icon: '🎞', render: (el) => monocoqueDynamic3D(el, A, ctx, p, modes, modesS) });
    out.views.push({ label: 'Stress on 3D monocoque', icon: '🧊', render: (el) => mapped3D(el, A, ctx) });
    out.views.push({ label: '2D animation: von Mises contour under load ramp & mode shape', icon: '🎞', render: (el) => chassis2D(el, A, modesS) });
    out.exports = [{ label: 'Mesh + results (VTK legacy)', action: () => exportVTK(A) }, { label: 'Nastran BDF (nodes, CTRIA3, PSHELL)', action: () => exportBDF(A, p) }];
    // recommendations
    if (scale > 1.02) out.recs.push({ level: 'warn', text: `To meet all targets the faces need ≈ ${fmt((scale - 1) * 100, 3)} % more thickness (governing: ${scale === p.sfTarget / Rmin ? 'strength' : scale === 1.5 / Math.max(BF, 1e-9) ? 'panel buckling' : 'torsional stiffness'}), raising shell mass to ≈ ${fmt(newMass, 3)} kg. Prefer local doublers at the critical stations over a global increase.`, basis: 'Fully-stressed design scaling' });
    else out.recs.push({ level: 'good', text: `All structural targets are met with margin; faces could be thinned by up to ≈ ${fmt((1 - scale) * 100, 3)} % (≈ ${fmt(chassisMass - newMass, 3)} kg saving). Re-check crash and fatigue before thinning.`, basis: 'Fully-stressed design scaling' });
    if (stroke >= 0.8 * overhang) out.recs.push({ level: 'critical', text: `Frontal crash at ${p.vCrash} km/h needs ${fmt(stroke, 3)} m of crush at ${p.gLim} g but only ≈ ${fmt(overhang, 3)} m of nose is available: add a tapered composite crash cone (${fmt(crushMass, 3)} kg) and limit the speed rating.`, basis: 'Energy method' });
    out.recs.push({ level: 'sustain', text: `Face material embodied carbon ≈ ${FM.co2} kgCO₂e/kg; the shell holds ≈ ${fmt(A.shellMass * 2 * lam.rhoA / (2 * lam.rhoA + tc * (CM.rho || 100)) * FM.co2, 3)} kgCO₂e. Recycled carbon-fibre faces cut this substantially if the knocked-down stiffness still meets the targets — compare with the material selector.`, basis: 'Materials registry' });
    out.recs.push({ level: 'good', text: 'Validate with a torsion rig (wheel-station loading) and strain-gauged bump test before occupied testing (specification gate 4).', basis: 'Validation hierarchy' });
    out.explain.push({ title: 'Why springs instead of fixed supports', text: 'A real chassis sits on compliant tyres; clamping it artificially stiffens the structure and hides bending. The model therefore supports it on tyre springs with only one longitudinal restraint, exactly enough to remove rigid-body motion.' });
    // ---- Revision 2.1 wing-root load path: mast-step bulkhead ring frame + keel beam (deployed, worst case) ----
    {
      const nW = veh.nWings || 2, mu = ctx.upstream('s10')?.muPeak ?? 0.95, ssf = ctx.upstream('s11')?.ssf ?? ctx.upstream('s01')?.ssf ?? (veh.track / 2) * (2 / 3) / veh.cgHeight;
      const zRoot = veh.wheelRadius + veh.height * 0.7, hCE = zRoot + 0.45 * veh.wingHeight, FySafe = Math.min(0.7 * mu * veh.mass * G, 0.8 * ssf * veh.mass * G * veh.cgHeight / hCE);
      const isArch = veh.wingConfig === 'arch', a6 = ctx.upstream('s06')?.arch;
      // tandem: cantilever root moment at breakaway; AeroArch: closed-frame root moment (portal action) at breakaway × gust DAF
      const Mroot = isArch ? (a6?.rootMoment_Nm ?? 0.62 * FySafe / nW * 0.45 * veh.wingHeight * p.gustFactor) : FySafe / nW * (hCE - zRoot) * p.gustFactor;
      const sepArch = isArch ? (archGeom(veh).rootChord * 0.8) : p.stepDepth, Fbear = Mroot / sepArch; // AeroArch: couple reacted by the fore/aft ring-track bulkheads
      // ring frame: couple applied at the crown of a closed ring of radius ≈ body half-height; max ring moment ≈ M/2 (Roark, ring under a couple)
      const t = p.frameT / 1000, d = p.frameD / 1000, Zf = t * d * d / 6, sigF = 0.5 * Mroot / Zf, Fx = (FM.Xt ?? 600) * 1e6 * 0.35; // quasi-isotropic allowable
      const [kw, kh] = String(p.keelW).split('x').map(v => +v / 1000), tk = p.keelT / 1000, Ik = (kw * kh ** 3 - (kw - 2 * tk) * (kh - 2 * tk) ** 3) / 12, Lk = 0.5 * veh.wheelbase, Mk = Fbear * Lk / 4, sigK = Mk * (kh / 2) / Ik;
      const SFf = Fx / sigF, SFk = Fx / sigK;
      // sized-to-SF 2.2 frames and keel (thin-wall: SF ∝ wall thickness at fixed section depth), min 0.6 mm laminate
      { const rhoF = (FM.rho || 1550), SFt = 2.2, tF = Math.max(0.6, p.frameT * SFt / Math.max(1e-9, SFf)), tK = Math.max(0.6, p.keelT * SFt / Math.max(1e-9, SFk)), rEq = ((veh.width || 1.6) * 0.9 + (veh.height || 0.75)) / 4, Lk = 0.8 * veh.length;
        out._sized = { frameT_mm: tF, keelT_mm: tK, frameMass_kg: 2 * Math.PI * rEq * (tF / 1000) * d * rhoF * 1.15, keelMass_kg: 2 * (kw + kh) * (tK / 1000) * Lk * rhoF * 1.15 };
        out.kpis.push({ label: 'Frames & keel sized to SF 2.2: wall / mass', value: `frame ${fmt(tF, 3)} mm / ${fmt(out._sized.frameMass_kg, 3)} kg each · keel ${fmt(tK, 3)} mm / ${fmt(out._sized.keelMass_kg, 3)} kg`, unit: '', explain: `Current SF ${fmt(SFf, 3)} (frames) and ${fmt(SFk, 3)} (keel) at ${p.frameT} / ${p.keelT} mm walls; ring radius ≈ ${fmt(rEq, 3)} m, keel ${fmt(Lk, 3)} m long; +15 % joints. Minimum 0.6 mm laminate.`, prov: 'calc' }); }
      out.kpis.push({ label: 'Design wing-root moment (deployed, at breakaway × dyn.)', value: Mroot, unit: 'N·m', explain: `F_y,safe/n_wings × lever (${fmt(hCE - zRoot, 3)} m) × ${p.gustFactor}. Worst case is the deployed wing just before the breakaway releases.`, prov: 'calc' });
      out.kpis.push({ label: 'Bulkhead ring-frame SF / keel-beam SF', value: `${fmt(SFf, 3)} / ${fmt(SFk, 3)}`, unit: '', explain: `Ring frame σ = 0.5·M/Z = ${fmt(sigF / 1e6, 3)} MPa; keel σ = ${fmt(sigK / 1e6, 3)} MPa from bearing couple ${fmt(Fbear, 3)} N over ${fmt(Lk, 3)} m.`, prov: 'calc', status: Math.min(SFf, SFk) >= 2 ? 'ok' : 'bad' });
      out.verdicts.push({ test: isArch ? 'AeroArch load path: ring-track bulkheads & keel' : 'Wing-root load path: bulkhead ring frame & keel (Rev 2.1)', status: Math.min(SFf, SFk) >= 2 ? 'pass' : Math.min(SFf, SFk) >= 1.3 ? 'warn' : 'fail', detail: `Root moment ${fmt(Mroot, 3)} N·m → partner/step forces ${fmt(Fbear, 3)} N; ring-frame SF ${fmt(SFf, 3)}, keel SF ${fmt(SFk, 3)} (target ≥ 2).`, fix: 'Increase frame laminate/depth, partner-to-step separation or keel section.' });
      out.tables.push({ title: 'Wing-root load path (mast-step bulkheads + keel)', cols: ['Item', 'Value'], rows: [['Breakaway side force (both wings)', `${fmt(FySafe, 4)} N`], ['Centre of effort above root (deployed)', `${fmt(hCE - zRoot, 4)} m`], ['Design root moment per wing', `${fmt(Mroot, 4)} N·m`], ['Partner / step bearing force', `${fmt(Fbear, 4)} N`], ['Ring-frame bending stress', `${fmt(sigF / 1e6, 4)} MPa (SF ${fmt(SFf, 3)})`], ['Keel-beam bending stress', `${fmt(sigK / 1e6, 4)} MPa (SF ${fmt(SFk, 3)})`]], explain: 'Hand-calculation check of the discrete load path that the side-section FE does not represent (out-of-plane wing moment).' });
      if (isArch && (veh.archRoot || 'flexure') === 'flexure') { // Revision 5.1: ONE centreline pylon flexure under the root crossbeam
        // Both arch legs stand on one CFRP root crossbeam, so the closed arch yaws about a single axis (twin leg flexures 1.09 m
        // apart lock kinematically). Concepts compared: (a) hybrid — central Ti torsion rod (preloaded, soft in yaw) + two
        // laminated-elastomer shear pads at ±R (overturning in compression, yaw in shear); (b) Ti cruciform; (c) slit Ti tube.
        const Ti = getMat('ti-beta21s') || { E: 105, sy: 1300, su: 1400, fatigue: 600, nu: 0.33 }, E = Ti.E * 1e9, Gm = E / (2 * (1 + (Ti.nu ?? 0.33))), sy = Ti.sy * 1e6, su = (Ti.su || 1400) * 1e6, Se = (Ti.fatigue || 600) * 1e6, nu = Ti.nu ?? 0.33;
        const AMg = archGeom(veh), s6 = ctx.upstream('s06')?.flexure, s07 = ctx.upstream('s07'), thMax = (ctx.store?.params?.('s07')?.flexMax ?? 25) * Math.PI / 180, thA = p.flexAmp * Math.PI / 180;
        const kTarget = ctx.store?.params?.('s07')?.flexK ?? 60, mA = p.archMass, gN = 9.81, zA = 0.52 * (AMg?.legHeight ?? 2), legSep = AMg?.gap ?? 1.09;
        const FyW = s07?.FySafe ?? FySafe, armS = 0.45 * (AMg?.legHeight ?? veh.wingHeight);
        const cases = [['Side force at the washout cap × 1.5', 1.5 * FyW * armS, mA * gN, 'roll'], ['Braking 0.9 g on the arch', 0.9 * gN * mA * zA, mA * gN, 'pitch'], [`Bump ${p.loadFactor} g, one leg`, p.loadFactor * gN * mA / 2 * (legSep / 2), p.loadFactor * gN * mA, 'roll'], ['Roll-over (lateral, inverted on the bridge)', p.rollM, 0, 'roll']];
        const kNegPerG = p.negPerG, Mmax = Math.max(...cases.map(c => c[1]));
        // (a) hybrid rod + shear pads
        const d = p.rodD / 1000, Lr = p.rodL / 1000, R = p.padR / 1000, w = p.padW / 1000, hp = p.padH / 1000, Ap = w * w, Gr = p.padG * 1e6, P = p.preload * 1000;
        const kRod = Gm * Math.PI * d ** 4 / (32 * Lr), kPad = 2 * Gr * Ap * R * R / hp, kH = kRod + kPad, Arod = Math.PI * d * d / 4;
        const padRows = cases.map(([n, M, Fz]) => { const hi = P / 2 + Fz / 2 + M / (2 * R), lo = P / 2 + Fz / 2 - M / (2 * R); return [n, M, hi / Ap / 1e6, lo]; });
        const sigP = Math.max(...padRows.map(r => r[2])), upl = padRows.filter(r => r[3] < 0), gam = R * thMax / hp, gamA = R * thA / hp;
        const sR = P / Arod, tR = Gm * (d / 2) * thMax / Lr, tRa = Gm * (d / 2) * thA / Lr, SFrod = sy / Math.hypot(sR, Math.sqrt(3) * tR), SFrodF = 1 / (Math.sqrt(3) * tRa / Se + sR / su);
        const SFpad = 7 / sigP, padOk = gam <= 1.0 && gamA <= 0.5 && w / hp >= 1 && !upl.length;
        const SFh = Math.min(SFrod, SFrodF, SFpad);
        // (b) cruciform (two crossed leaves b × t × L) and (c) slit tube (open section; warping restrained by the clamped ends)
        const bc = p.crucB / 1000, tc2 = p.crucT / 1000, Lc = p.crucL / 1000, kC = 2 * Gm * bc * tc2 ** 3 / (3 * Lc), sigC = 6 * Mmax / (tc2 * bc * bc), sigCr = 0.43 * Math.PI ** 2 * E / (12 * (1 - nu * nu)) * (2 * tc2 / bc) ** 2, tauC = Gm * tc2 * thMax / Lc, SFc = Math.min(sy / Math.hypot(sigC, Math.sqrt(3) * tauC), sigCr / sigC);
        const rt = p.tubeD / 2000, tt = p.tubeT / 1000, Lt = p.tubeL / 1000, kSV = Gm * 2 * Math.PI * rt * tt ** 3 / (3 * Lt), Gam = 2 * Math.PI * rt ** 5 * tt * (Math.PI ** 2 - 6) / 3, kW = 12 * E * Gam / Lt ** 3, kT2 = kSV + kW, sigT = Mmax / (Math.PI * rt * rt * tt), sigTcr = 0.3 * 0.6 * E * tt / rt, tauT = Gm * tt * thMax / Lt, SFt = Math.min(sy / Math.hypot(sigT, Math.sqrt(3) * tauT), sigTcr / sigT);
        // (d) Ti rod cage: n rods Ø dc at pitch radius Rc between the crossbeam plate and the root frame. Yaw bends each rod
        // (guided ends, tangential offset Rc·θ) and twists it by θ; overturning is carried as rod tension/compression. A soft
        // flexure that also carries the overturning moment is bounded by k ≥ 0.61·SF·M·Rc/L (bending) + n·G·J/L (twist), so the
        // cage is statically balanced by a pre-buckled bistable Ti strip compensator of stiffness −(k_pos − k_target).
        // Roll-over beyond the stop gap is carried by contact of the crossbeam ends on the root-frame stops.
        const nC = p.cageN, dc = p.cageD / 1000, Lc2 = p.cageL / 1000, Rc = p.cageR / 1000, Ic = Math.PI * dc ** 4 / 64, Ac = Math.PI * dc * dc / 4, Jc = 2 * Ic;
        const kBend = nC * 12 * E * Ic * Rc * Rc / Lc2 ** 3, kTw = nC * Gm * Jc / Lc2, kCage = kBend + kTw, kComp = kCage - kTarget;
        const Mop = Math.max(...cases.filter(c => !/Roll-over/.test(c[0])).map(c => c[1])), Ffz = mA * gN * p.loadFactor / nC, Frod = 2 * Mop / (nC * Rc) + Ffz, PcrR = 4 * Math.PI ** 2 * E * Ic / Lc2 ** 2;
        const KrollC = nC / 2 * E * Ac / Lc2 * Rc * Rc, xEnd = (AMg?.gap ?? 0.88) / 2 + 0.06, Mstop = KrollC * (p.stopGap / 1000) / xEnd, FrodStop = 2 * Mstop / (nC * Rc) + Ffz;
        const sBc = 6 * E * (dc / 2) * Rc * thMax / Lc2 ** 2, tTc = Gm * (dc / 2) * thMax / Lc2, sAx = Frod / Ac, SFcSt = sy / Math.hypot(sAx + sBc, Math.sqrt(3) * tTc), SFcBk = PcrR / Math.max(Frod, FrodStop);
        const sBa = 6 * E * (dc / 2) * Rc * thA / Lc2 ** 2, tTa = Gm * (dc / 2) * thA / Lc2, SFcF = 1 / (Math.hypot(sBa, Math.sqrt(3) * tTa) / Se + (0.5 * sAx) / su);
        const SFcage = Math.min(SFcSt, SFcBk, SFcF), dNet = kCage * Math.hypot(p.trimTol / 100, p.driftRes / 100), stopsOk = Mstop < Math.min(PcrR / 2, Ac * sy / 2) * nC * Rc / 2;
        const kSel = p.pylonType === 'cage' ? kTarget : p.pylonType === 'cruciform' ? kC : p.pylonType === 'slitTube' ? kT2 : kH, k1 = kSel - kNegPerG, k3 = kSel - kNegPerG * p.loadFactor, SFsel = p.pylonType === 'cage' ? SFcage : p.pylonType === 'cruciform' ? SFc : p.pylonType === 'slitTube' ? SFt : SFh;
        // stop impact: the arch swings into the ±θmax progressive elastomer stop at the aero pitch-mode rate
        const Iz = s6?.Iθ ?? 5.26, fA = s6?.fAero ?? 2.3, om = thMax * 2 * Math.PI * fA, Est = 0.5 * Iz * om * om, Mst = Math.sqrt(2 * Est * p.stopK), dSt = Mst / p.stopK;
        // root frame (x ≈ 0.37 L) to keel: potted Ti inserts, two per pad station
        const padMax = Math.max(...padRows.map(r => r[2])) * Ap * 1e6, Fins = p.pylonType === 'cage' ? 2 * Mop / (4 * p.insR / 1000) + mA * gN * p.loadFactor / 4 : Math.max(padMax / 2, (Mst / (2 * R)) / 2), SFins = p.insertCap * 1000 / Fins;
        const kOk = p.pylonType === 'cage' ? k1 - dNet >= 5 && k3 - dNet > 0 : k1 >= 5 && k3 > 0;
        out.kpis.push({ label: 'Centreline pylon yaw stiffness: element / net at 1 g / net at bump', value: `${fmt(kSel, 3)} / ${fmt(k1, 3)} / ${fmt(k3, 3)}`, unit: 'N·m/rad', explain: `Axial load on the pylon gives a negative (P-δ) contribution of −${kNegPerG} N·m/rad per g. Target from Suite 07: ${kTarget} N·m/rad; the aerodynamic yaw stiffness (≈ 30 N·m/rad at 4 m/s) keeps self-trim.`, prov: 'calc', status: kOk ? 'ok' : 'bad' });
        out.kpis.push({ label: `Pylon strength SF (${p.pylonType}) / root-frame insert SF`, value: `${fmt(SFsel, 3)} / ${fmt(SFins, 3)}`, unit: '', explain: `Worst of the 3-axis load cases (max overturning ${fmt(Mmax, 4)} N·m), buckling, ±${fmt(thMax * 180 / Math.PI, 3)}° yaw shear and fatigue at 1e7 cycles. Inserts: ${p.insertCap} kN each.`, prov: 'calc', status: Math.min(SFsel, SFins) >= 2 ? 'ok' : 'bad' });
        out.verdicts.push({ test: 'Centreline pylon flexure: strength, buckling, fatigue ≥ 1e7 cycles (SF ≥ 2)', status: Math.min(SFsel, SFins) >= 2 && (p.pylonType !== 'hybrid' || padOk) && (p.pylonType !== 'cage' || stopsOk) ? 'pass' : 'fail', detail: `${p.pylonType === 'cage' ? `Rod cage ${nC} × Ø${p.cageD} × ${p.cageL} mm at R ${p.cageR} mm: static SF ${fmt(SFcSt, 3)}, buckling SF ${fmt(SFcBk, 3)} (rod load ${fmt(Math.max(Frod, FrodStop), 4)} N vs P_cr ${fmt(PcrR, 4)} N), fatigue SF ${fmt(SFcF, 3)}; roll-over stops engage at ${fmt(Mstop, 4)} N·m (${p.stopGap} mm gap)` : p.pylonType === 'hybrid' ? `Rod static SF ${fmt(SFrod, 3)}, rod fatigue SF ${fmt(SFrodF, 3)}, pad compression SF ${fmt(SFpad, 3)} (${fmt(sigP, 3)} MPa vs 7 MPa), pad shear strain ${fmt(gam, 3)} (≤ 1.0) / fatigue ±${fmt(gamA, 3)} (≤ 0.5)${upl.length ? `, UPLIFT in: ${upl.map(r => r[0]).join(', ')} — raise the preload` : ', no pad uplift'}` : p.pylonType === 'cruciform' ? `Leaf bending ${fmt(sigC / 1e6, 3)} MPa vs local buckling ${fmt(sigCr / 1e6, 3)} MPa; SF ${fmt(SFc, 3)}` : `Tube bending ${fmt(sigT / 1e6, 3)} MPa vs shell buckling ${fmt(sigTcr / 1e6, 3)} MPa; SF ${fmt(SFt, 3)}`}; inserts SF ${fmt(SFins, 3)}.`, fix: 'Hybrid: raise the rod preload or pad radius; cruciform: lower b/t (stiffens yaw); slit tube: only with warping-free ends.' });
        out.verdicts.push({ test: 'Pylon net yaw stiffness ≥ 5 N·m/rad and positive at the bump load', status: kOk ? 'pass' : 'fail', detail: p.pylonType === 'cage' ? `Cage ${fmt(kCage, 4)} N·m/rad (bending ${fmt(kBend, 4)} + rod twist ${fmt(kTw, 4)}) balanced by a −${fmt(kComp, 4)} N·m/rad compensator → net ${fmt(k1, 3)} at 1 g, ${fmt(k3, 3)} at ${p.loadFactor} g; uncertainty ±${fmt(dNet, 3)} N·m/rad (trim ±${p.trimTol} % + residual thermal drift ±${p.driftRes} %).` : `Net ${fmt(k1, 3)} N·m/rad at 1 g, ${fmt(k3, 3)} N·m/rad at ${p.loadFactor} g (element ${fmt(kSel, 3)}).`, fix: 'Tighter trim of the compensator preload at assembly, or accept a stiffer pylon (see the speed penalty in Suite 07 with a higher k_θ).' });
        out.tables.push({ title: 'Centreline pylon flexure — concept comparison (same load cases)', cols: ['Concept', 'Yaw k_θ [N·m/rad]', 'Governing check', 'SF', 'Verdict'], rows: [
          ['(d) Ti rod cage + negative-stiffness compensator (recommended)', `${fmt(kCage, 4)} − ${fmt(kComp, 4)} = ${fmt(kTarget, 3)} ± ${fmt(dNet, 3)}`, `${nC} × Ø${p.cageD} × ${p.cageL} mm at R ${p.cageR} mm; stops at ${fmt(Mstop, 4)} N·m`, SFcage, SFcage >= 2 && stopsOk && kTarget - p.negPerG - dNet >= 5 ? 'pass' : 'fail'],
          ['(d′) same cage without compensator', kCage, 'soft-yaw bound k ≥ 0.61·SF·M·R/L + n·G·J/L', SFcage, 'too stiff for self-trim'],
          ['(a) Hybrid: Ti rod + 2 elastomer shear pads', kH, `rod Ø${p.rodD}×${p.rodL} mm @ ${p.preload} kN; pads ${p.padW}² × ${p.padH} mm at ±${p.padR} mm`, SFh, SFh >= 2 && padOk && kH - kNegPerG >= 5 ? (kH <= 3 * kTarget ? 'pass' : 'too stiff in yaw (pads in shear)') : 'fail'],
          ['(b) Ti cruciform', kC, `leaf ${p.crucB}×${p.crucT}×${p.crucL} mm: bending ${fmt(sigC / 1e6, 3)} vs buckling ${fmt(sigCr / 1e6, 3)} MPa`, SFc, SFc >= 2 ? (kC <= 3 * kTarget ? 'pass' : 'too stiff in yaw') : 'fail'],
          ['(c) Slit Ti tube, clamped ends', kT2, `Ø${p.tubeD}×${p.tubeT}×${p.tubeL} mm: St Venant ${fmt(kSV, 3)} + restrained warping ${fmt(kW, 3)}`, SFt, kT2 > 20 * kTarget ? 'too stiff in yaw (warping)' : SFt >= 2 ? 'pass' : 'fail']],
          explain: 'A soft yaw element cannot also carry the arch overturning moment without buckling: for any flexure the yaw stiffness and the buckling margin scale together (k ≥ 0.61·SF·M·R/L), and elastomer pads at a useful radius are stiff in yaw (2·G·A·R²/h). No compact pure flexure reaches 5 N·m/rad at SF ≥ 2; the recommended pylon therefore balances a strong rod cage with a pre-buckled negative-stiffness compensator, trimmed at assembly.' });
        out.tables.push({ title: 'Pylon 3-axis load cases (hybrid pad forces)', cols: ['Case', 'Overturning moment [N·m]', 'Max pad pressure [MPa]', 'Min pad force [N]'], rows: padRows });
        out.tables.push({ title: 'Stop impact and root-frame tie', cols: ['Item', 'Value'], rows: [['Arch yaw inertia / swing rate at the stop', `${fmt(Iz, 3)} kg·m² / ${fmt(om, 3)} rad/s`], ['Impact energy', `${fmt(Est, 3)} J`], ['Progressive elastomer stop: stiffness / peak moment / travel', `${p.stopK} N·m/rad / ${fmt(Mst, 4)} N·m / ${fmt(dSt * 180 / Math.PI, 3)}°`], ['Root frame', 'Transverse CFRP frame at x ≈ 0.37 L tied to the keel; crossbeam 20–30 mm above the deck'], ['Max insert load / capacity', `${fmt(Fins, 4)} N / ${p.insertCap} kN (SF ${fmt(SFins, 3)})`]] });
        // root boot (continuous elastomer membrane deck → leg roots, Suite 07 rootBootH): torsional shear of the membrane wall as the
        // arch yaws; the farthest point of the boot footprint is the leg trailing edge, r_max = √((0.865 c_root)² + (g/2)²)
        if ((veh.rootBootH ?? 0) > 0) { const hB = veh.rootBootH, cR = AMg?.rootChord ?? 1.5, gp = AMg?.gap ?? 0.9, rMx = Math.hypot(0.865 * cR, gp / 2), per = 2 * (2 * cR + gp), rAv = 0.6 * rMx, Gb = p.bootG * 1e6, tb = p.bootT / 1000;
          const gMax = rMx * thMax / hB, gAmp = rMx * thA / hB, kB = Gb * tb * per * rAv * rAv / hB, okB = gMax <= p.bootGamMax && gAmp <= p.bootGamFat;
          out.kpis.push({ label: 'Root boot: shear strain at the stop / fatigue amplitude / added yaw stiffness', value: `${fmt(gMax, 3)} / ${fmt(gAmp, 3)} / ${fmt(kB, 3)} N·m/rad`, unit: '', explain: `Membrane ${p.bootT} mm, G ${p.bootG} MPa, height ${hB} m, footprint perimeter ${fmt(per, 3)} m; r_max ${fmt(rMx, 3)} m at ±${fmt(thMax * 180 / Math.PI, 3)}°. Limits γ ≤ ${p.bootGamMax} at the stop, ±${p.bootGamFat} for 10⁷ cycles (elastomer shear fatigue, cf. Gent, Engineering with Rubber). The added stiffness is taken off the compensator.`, prov: 'calc', status: okB ? 'ok' : 'bad' });
          out.verdicts.push({ test: 'Root boot membrane strain & fatigue (no open gap, no sliding)', status: okB ? 'pass' : 'fail', detail: `γ ${fmt(gMax, 3)} (≤ ${p.bootGamMax}) at the stop, ±${fmt(gAmp, 3)} (≤ ${p.bootGamFat}) at the ±${p.flexAmp}° fatigue amplitude; minimum height for both: ${fmt(Math.max(rMx * thMax / p.bootGamMax, rMx * thA / p.bootGamFat), 3)} m.`, fix: 'Raise the boot (costs leg span), reduce the yaw range or the fatigue amplitude, or shorten the root chord aft of the pylon.' }); }
        // brush root seal (Suite 07 deckSeal): bristles on a plate that yaws with the arch wipe a polished deck strip. Friction torque
        // T_f = μ·p_n·P·r̄ must stay small against the trim stiffness: dead band δ_f = T_f/(k_net + k_aero); Archard wear k·F·s.
        if ((veh.deckSeal ?? 0) > 0 && !((veh.rootBootH ?? 0) > 0)) { const cR = AMg?.rootChord ?? 1.5, gp = AMg?.gap ?? 0.9, per = 2 * (2 * cR + gp), rb = 0.6 * Math.hypot(0.865 * cR, gp / 2), Tf = p.brushMu * p.brushPn * per * rb, kAero = p.kAeroLow, dB = Tf / Math.max(1e-9, kTarget + kAero) * 180 / Math.PI;
          const sYr = p.gustRateH * p.hoursYr * 2 * rb * 2 * thA, wear = p.brushK * p.brushPn * per * sYr / (per * p.brushW / 1000) * 1e6, okBr = dB <= 2 && wear * 10 <= p.brushLen * 1000; // wear in µm/yr, bristle length in mm
          out.kpis.push({ label: 'Brush root seal: friction torque / trim dead band / tip wear', value: `${fmt(Tf, 3)} N·m / ±${fmt(dB, 3)}° / ${fmt(wear, 3)} µm per year`, unit: '', explain: `μ ${p.brushMu}, contact preload ${p.brushPn} N/m over a ${fmt(per, 3)} m footprint at mean radius ${fmt(rb, 3)} m; dead band against k_net ${kTarget} + light-wind aerodynamic yaw stiffness ${kAero} N·m/rad. Wear (Archard k ${p.brushK} m³/N·m) over ${fmt(sYr / 1000, 3)} km of tip sliding per year.`, prov: 'calc', status: okBr ? 'ok' : 'warn' });
          out.verdicts.push({ test: 'Brush root seal does not stop self-trim (dead band ≤ 2°) and lasts ≥ 10 years', status: okBr ? 'pass' : 'warn', detail: `Dead band ±${fmt(dB, 3)}°, wear ${fmt(wear, 3)} µm/yr vs ${p.brushLen} mm usable bristle length. Bristles PBT or stainless (no moisture-absorbing polymers); deck strip polished stainless.`, fix: 'Lower the preload (near-contact brush), use stainless bristles, or raise k_net.' }); }
        out._pylon = { type: p.pylonType, kCage, kComp, kUncert: dNet, k: kSel, kNet1g: k1, kNetBump: k3, SF: SFsel, SFinserts: SFins, Mmax };
      }
      if (isArch) { // roll-over hoop: vehicle inverted on the bridge, 1.5 × vehicle weight
        const g6 = archGeom(veh), P = 1.5 * veh.mass * G, legLen = g6.legHeight / Math.cos(g6.cant * Math.PI / 180), Nleg = P / 2 / Math.cos(g6.cant * Math.PI / 180);
        const EIleg = a6?.EIroot ?? archEI(ctx).leg, EIbr = a6?.EItip ?? archEI(ctx).bridge, Pcr = Math.PI ** 2 * EIleg / (1.0 * legLen) ** 2; // sway portal with stiff bridge: K ≈ 1.0
        const rT = p.bridgeSparD / 2000, tT = p.bridgeSparT / 1000, ET = 110e9, EIt = rT > 0 && tT > 0 ? ET * Math.PI * rT ** 3 * tT : 0, EIbrT = EIbr + EIt; // bridge carbon roll-over tube inside the bridge section
        const Mbr = P * g6.bridgeSpan / 12, hsb = archEI(ctx).hsTip, kap = Mbr / EIbrT, sigB = kap * (hsb / 2) * archEI(ctx).Ecap, sigT = kap * rT * ET, Xc = (FM.Xc ?? FM.Xt ?? 600) * 1e6 * 0.5, SFb = Pcr / Nleg, SFbr = Math.min(Xc / Math.max(1, sigB), EIt > 0 ? 0.5 * 1100e6 / Math.max(1, sigT) : Infinity);
        const mTube = 2 * Math.PI * rT * tT * g6.bridgeSpan * 1550, sfTube = EIt > 0 ? 0.5 * 1100e6 / Math.max(1, sigT) : Infinity, tT22 = Math.max(0.5e-3, tT * 2.2 / Math.min(sfTube, 1e9)), mTube22 = 2 * Math.PI * rT * tT22 * g6.bridgeSpan * 1550; out._sized = { ...(out._sized || {}), rolloverTubeWall_mm: tT22 * 1000, rolloverTubeMass_kg: mTube22 };
        out.kpis.push({ label: 'Roll-over hoop: leg buckling SF / bridge bending SF', value: `${fmt(SFb, 3)} / ${fmt(SFbr, 3)}`, unit: '', explain: `Inverted vehicle on the arch bridge, 1.5 × weight = ${fmt(P, 3)} N: each canted leg carries ${fmt(Nleg, 3)} N vs Euler P_cr ${fmt(Pcr, 3)} N (K = 1.0, L = ${fmt(legLen, 3)} m); bridge (fixed–fixed, distributed contact) M = ${fmt(Mbr, 3)} N·m over ${fmt(g6.bridgeSpan, 3)} m, carried by the bridge skin plus a Ø${p.bridgeSparD} × ${p.bridgeSparT} mm carbon tube (${fmt(mTube, 3)} kg; tube allowable 1100 MPa × 0.5).`, prov: 'calc', status: Math.min(SFb, SFbr) >= 2 ? 'ok' : 'bad' });
        out.verdicts.push({ test: 'AeroArch roll-over protection (inverted, 1.5 × weight)', status: Math.min(SFb, SFbr) >= 2 ? 'pass' : Math.min(SFb, SFbr) >= 1 ? 'warn' : 'fail', detail: `Leg buckling SF ${fmt(SFb, 3)}, bridge bending SF ${fmt(SFbr, 3)} (target ≥ 2). Survival space: bridge underside ${fmt(g6.zTop - 0.12 - veh.wheelRadius - veh.height * 0.62, 3)} m above the seated occupant’s head line.`, fix: 'Thicken the leg caps / torsion tube or add a compression strut in the legs; the arch doubles as the roll-over hoop only if it does not buckle.' });
      }
    }
    out.outputs = { ...(out._sized || {}), pylon: out._pylon ?? null, maxVonMises_MPa: vmMax / 1e6, minSF: Rmin, maxDefl_m: A.maxRel, torsionalStiffness_Nm_deg: KtDeg, bucklingFactor: BF, wrinklingFactor: WF, chassisMass, tsaiWuMax: 1 / Rmin, f1Hz: modesS[0]?.f, rideHz: modes[0]?.f, crashStroke_m: stroke, crushMass_kg: crushMass }; delete out._pylon; delete out._sized;
    out.headline = `Min strength ratio ${fmt(Rmin, 3)}, max deflection ${fmt(A.maxRel * 1000, 3)} mm, torsional stiffness ${fmt(KtDeg / 1e3, 3)} kN·m/deg, buckling factor ${fmt(BF, 3)}, crash stroke ${fmt(stroke, 3)} m of ${fmt(overhang, 3)} m available.`;
    out.headlineKind = Rmin >= p.sfTarget && BF >= 1.5 ? 'ok' : 'warn';
    return out;
  },
};

function contourView(el, A) {
  const fields = { vm: ['Equivalent face stress', 'MPa', (e) => e.vm / 1e6], R: ['Tsai–Wu strength ratio (log)', 'log₁₀ R', (e) => Math.log10(Math.max(1e-3, e.R))], sx: ['σ_x', 'MPa', (e) => e.sig[0] / 1e6], txy: ['τ_xy', 'MPa', (e) => e.sig[2] / 1e6] };
  const sel = h('select', {}, ...Object.entries(fields).map(([k, v]) => h('option', { value: k }, v[0]))), scaleIn = h('input', { type: 'range', min: 0, max: 400, value: 100 }), box = h('div');
  const draw = () => {
    const [name, unit, f] = fields[sel.value], vals = A.els.map(f), lo = Math.min(...vals), hi = Math.max(...vals), m = A.mesh, maxU = Math.max(...m.nodes.map((_, k) => Math.hypot(A.sol.u[2 * k], A.rel[k]))) || 1;
    const xs = m.nodes.map(n => n[0]), ys = m.nodes.map(n => n[1]), x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys), W = 900, H = Math.max(220, W * (y1 - y0) / (x1 - x0) * 1.6 + 60), sc = (W - 90) / (x1 - x0), def = (+scaleIn.value / 100) * 0.05 * (x1 - x0) / maxU;
    const X = (k) => 20 + (m.nodes[k][0] + A.sol.u[2 * k] * def - x0) * sc, Y = (k) => H - 30 - (m.nodes[k][1] + A.rel[k] * def - y0) * sc * 1.6;
    let svg = `<svg viewBox="0 0 ${W} ${H}" width="100%" style="background:var(--panel2);border-radius:10px">`;
    m.tris.forEach((t, i) => { svg += `<polygon points="${t.map(k => X(k).toFixed(1) + ',' + Y(k).toFixed(1)).join(' ')}" fill="${cssColor(cmap((vals[i] - lo) / ((hi - lo) || 1), 'turbo'))}" stroke="${cssColor(cmap((vals[i] - lo) / ((hi - lo) || 1), 'turbo'))}" stroke-width=".3"><title>${name}: ${fmt(vals[i], 4)} ${unit}</title></polygon>`; });
    for (const k of [A.kf, A.kr]) svg += `<path d="M${X(k)},${Y(k)} l-9,16 h18z" fill="#22c55e"/>`;
    for (const k of A.roots) svg += `<path d="M${X(k)},${Y(k) - 4} v-26" stroke="#f97316" stroke-width="3" marker-end="url(#a1)"/>`;
    svg += `<defs><marker id="a1" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="5" markerHeight="5" orient="auto"><path d="M0,0L10,5L0,10z" fill="#f97316"/></marker></defs>`;
    for (let i = 0; i <= 10; i++) svg += `<rect x="${W - 60}" y="${20 + (10 - i) * (H - 60) / 11}" width="14" height="${(H - 60) / 11 + 1}" fill="${cssColor(cmap(i / 10, 'turbo'))}"/><text x="${W - 42}" y="${32 + (10 - i) * (H - 60) / 11}" font-size="10" fill="currentColor">${fmt(lo + (hi - lo) * i / 10, 3)}</text>`;
    svg += `<text x="20" y="16" font-size="12" fill="currentColor">${name} [${unit}] · deformation ×${fmt(def, 3)} (vertical axis ×1.6 for legibility) · green = tyre supports · orange = wing roots</text></svg>`;
    box.innerHTML = svg;
  };
  sel.onchange = draw; scaleIn.oninput = draw; draw();
  el.append(h('div', { class: 'row gap wrap' }, h('label', {}, 'Field ', sel), h('label', {}, 'Deformation scale ', scaleIn)), box, h('p', { class: 'small muted' }, 'Element-wise solved fields on the body-fitted FE mesh of the side section (hover an element for its value).'));
}
async function mapped3D(el, A, ctx) {
  const [{ viewer }, { vehicleAssembly }] = await Promise.all([import('../core/viz3d.js'), import('../core/cad.js')]);
  const V = await viewer(el, { height: 440 }), asm = vehicleAssembly(ctx.vehicle, {}), vals = A.els.map(e => e.vm / 1e6);
  const grid = new Map(), key = (x, y) => Math.round(x * 20) + ',' + Math.round(y * 20); A.els.forEach((e, i) => { const kk = key(e.c[0], e.c[1]); if (!grid.has(kk) || grid.get(kk) < vals[i]) grid.set(kk, vals[i]); });
  const look = (x, y) => { for (let r = 0; r < 6; r++) for (let dx = -r; dx <= r; dx++) for (let dy = -r; dy <= r; dy++) { const v = grid.get((Math.round(x * 20) + dx) + ',' + (Math.round(y * 20) + dy)); if (v !== undefined) return v; } return 0; };
  for (const m of asm.parts) { if (m.name === 'Monocoque body') { const f = []; for (let k = 0; k < m.p.length; k += 3) f.push(look(m.p[k], m.p[k + 2])); V.add(m, { field: { name: 'Equivalent face stress (side-section FE)', unit: 'MPa', values: f, cmap: 'turbo' } }); } else if (m.group !== 'Wings') V.add(m, { opacity: 0.25 }); }
  V.fit(); el.append(h('p', { class: 'small muted' }, 'Side-section FE stress projected onto the parametric monocoque surface (x–z mapping). Wings hidden; other parts translucent.'));
}
function exportVTK(A) { const m = A.mesh; let s = `# vtk DataFile Version 3.0\nPHOENIX-ATMOS-XP chassis side section\nASCII\nDATASET UNSTRUCTURED_GRID\nPOINTS ${m.nodes.length} float\n${m.nodes.map(n => `${n[0]} ${n[1]} 0`).join('\n')}\nCELLS ${m.tris.length} ${m.tris.length * 4}\n${m.tris.map(t => `3 ${t.join(' ')}`).join('\n')}\nCELL_TYPES ${m.tris.length}\n${m.tris.map(() => 5).join('\n')}\nCELL_DATA ${m.tris.length}\nSCALARS von_mises_Pa float 1\nLOOKUP_TABLE default\n${A.els.map(e => e.vm).join('\n')}\nSCALARS tsai_wu_R float 1\nLOOKUP_TABLE default\n${A.els.map(e => Math.min(1e6, e.R)).join('\n')}\nPOINT_DATA ${m.nodes.length}\nVECTORS displacement float\n${m.nodes.map((_, k) => `${A.sol.u[2 * k]} ${A.sol.u[2 * k + 1]} 0`).join('\n')}\n`; dl(s, 'atmosxp-chassis.vtk'); }
function exportBDF(A, p) { const m = A.mesh, f8 = (v) => String(typeof v === 'number' ? (Number.isInteger(v) ? v : v.toPrecision(6)) : v).slice(0, 8).padEnd(8); let s = `$ PHOENIX-ATMOS-XP chassis side section, units m N Pa\nBEGIN BULK\n`; m.nodes.forEach((n, k) => s += `GRID    ${f8(k + 1)}        ${f8(n[0])}${f8(n[1])}${f8(0)}\n`); m.tris.forEach((t, e) => s += `CTRIA3  ${f8(e + 1)}${f8(1)}${f8(t[0] + 1)}${f8(t[1] + 1)}${f8(t[2] + 1)}\n`); s += `PSHELL  ${f8(1)}${f8(1)}${f8(4 * A.hf)}\nMAT1    ${f8(1)}${f8(A.lam.Ex)}        ${f8(A.lam.nuxy)}${f8(1550)}\nENDDATA\n`; void p; dl(s, 'atmosxp-chassis.bdf'); }
function dl(text, name) { const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([text], { type: 'text/plain' })); a.download = name; a.click(); }

// ---------------- 3-D dynamic monocoque: solved load case ramped in time, and solved mode shapes ----------------
// Solved: plane-stress FE displacement & von Mises field for the selected load case; subspace-iteration modes
// (on tyre springs and with stiff wheel stations). Displayed: section displacements mapped onto the 3-D body
// surface by (x, z) projection, magnified by the slider; load-case magnitude ramped 0 → 1 → 0 (linear response).
async function monocoqueDynamic3D(el, A, ctx, p, modes, modesS) {
  const [{ viewer }, { vehicleAssembly }, { playback, hud, legend3d }] = await Promise.all([import('../core/viz3d.js'), import('../core/cad.js'), import('../core/sim3d.js')]);
  const cases = [{ v: 'ramp', l: `Load case “${p.case}” ramped 0 → 1 → 0 (linear static response)` }, ...modesS.map((m, i) => ({ v: 's' + i, l: `Elastic mode ${i + 1}: ${fmt(m.f, 3)} Hz (wheel stations held)` })), ...modes.map((m, i) => ({ v: 'r' + i, l: `Ride mode ${i + 1} on tyre springs: ${fmt(m.f, 3)} Hz` }))];
  const sel = h('select', {}, ...cases.map(c => h('option', { value: c.v }, c.l)));
  const mag = h('input', { type: 'range', min: 1, max: 500, step: 1, value: 50 }), magL = h('span', { class: 'small mono' }, '');
  el.append(h('p', { class: 'small muted' }, 'Displacements and stresses are the solved finite-element fields of the side-section model, projected onto the parametric monocoque surface by their (x, z) position; magnification is shown on the slider. Mode shapes are mass-normalised eigenvectors scaled for display.'), h('div', { class: 'row gap wrap' }, h('label', {}, 'Case ', sel), h('label', {}, 'Magnification ', mag, magL)));
  const box = h('div'); el.append(box);
  const V = await viewer(box, { height: 470 }), asm = vehicleAssembly(ctx.vehicle, {});
  const body = asm.parts.find(m => m.name === 'Monocoque body'); const obj = V.add(body, { field: { name: 'σ_vM', unit: 'MPa', values: new Array(body.p.length / 3).fill(0), cmap: 'turbo' } });
  for (const m of asm.parts) if (m !== body && m.group !== 'Wings' && m.group !== 'Thermal') V.add(m, { opacity: 0.18 });
  const nodes = A.mesh.nodes, base = Float32Array.from(body.p), nv = base.length / 3, nn = new Int32Array(nv);
  const grid = new Map(), key = (x, y) => Math.round(x * 25) + ',' + Math.round(y * 25); nodes.forEach((n, k) => { const kk = key(n[0], n[1]); if (!grid.has(kk)) grid.set(kk, k); });
  for (let v = 0; v < nv; v++) { const x = base[v * 3], z = base[v * 3 + 2]; let best = -1; for (let r = 0; r < 12 && best < 0; r++) for (let dx = -r; dx <= r && best < 0; dx++) for (let dy = -r; dy <= r; dy++) { const k = grid.get((Math.round(x * 25) + dx) + ',' + (Math.round(z * 25) + dy)); if (k !== undefined) { best = k; break; } } if (best < 0) { let bd = Infinity; nodes.forEach((n, k) => { const d = (n[0] - x) ** 2 + (n[1] - z) ** 2; if (d < bd) { bd = d; best = k; } }); } nn[v] = best; }
  const nodeVM = new Float64Array(nodes.length), cnt = new Float64Array(nodes.length); A.mesh.tris.forEach((t, e) => t.forEach(k => { nodeVM[k] += A.els[e].vm; cnt[k]++; })); for (let k = 0; k < nodes.length; k++) nodeVM[k] /= cnt[k] || 1;
  const vmMax = Math.max(...nodeVM) / 1e6, H = hud(box), leg = legend3d(box, { title: 'von Mises (MPa)', lo: 0, hi: vmMax });
  const full = (sol, vec) => { const u = new Float64Array(2 * nodes.length); sol.freeList.forEach((d, j) => u[d] = vec[j]); return u; };
  let pb = null;
  const start = () => {
    pb?.dispose(); box.querySelector('.sim-bar')?.remove();
    const c = sel.value; let U, f = 0, isMode = c !== 'ramp';
    if (!isMode) { // elastic part only: vertical relative to the wheel line (rigid heave/pitch on tyres removed), mean surge removed
      let mx = 0; for (let k = 0; k < nodes.length; k++) mx += A.sol.u[2 * k] / nodes.length; U = new Float64Array(2 * nodes.length); for (let k = 0; k < nodes.length; k++) { U[2 * k] = A.sol.u[2 * k] - mx; U[2 * k + 1] = A.rel[k]; } } else { const M = (c[0] === 's' ? modesS : modes)[+c.slice(1)], sol = c[0] === 's' ? A.solStiff : A.sol; U = full(sol, M.vec); f = M.f; }
    let umax = 0; for (let k = 0; k < nodes.length; k++) umax = Math.max(umax, Math.hypot(U[2 * k], U[2 * k + 1]));
    const autoMag = Math.max(1, Math.round(0.1 * ctx.vehicle.length / Math.max(1e-12, umax))); mag.max = String(Math.max(1000, autoMag * 4)); mag.value = String(autoMag);
    const lab = () => { magL.textContent = isMode ? `Deformation ×${mag.value} (auto ×${autoMag}; mode shape normalised)` : `Deformation ×${mag.value} (auto ×${autoMag}; true-scale peak elastic displacement = ${fmt(umax * 1000, 3)} mm)`; }; mag.oninput = lab; lab();
    leg.set(0, isMode ? 1 : vmMax);
    const T = isMode ? 3 / Math.max(f, 1e-3) : 6;
    pb = playback(box, { duration: T, rate: isMode ? Math.min(1, 0.7 / Math.max(f, 1e-3)) : 1, label: isMode ? 'Modal time' : 'Load ramp time', onFrame: (t) => {
      const lam = isMode ? Math.sin(2 * Math.PI * f * t) : (t < T / 2 ? t / (T / 2) : 2 - t / (T / 2)), m = +mag.value * (isMode ? 1 : 1), P = obj.geometry.attributes.position.array, vals = new Array(nv);
      for (let v = 0; v < nv; v++) { const k = nn[v]; P[v * 3] = base[v * 3] + lam * m * U[2 * k]; P[v * 3 + 1] = base[v * 3 + 1]; P[v * 3 + 2] = base[v * 3 + 2] + lam * m * U[2 * k + 1]; vals[v] = isMode ? Math.abs(lam) * Math.hypot(U[2 * k], U[2 * k + 1]) / umax : Math.abs(lam) * nodeVM[k] / 1e6; }
      obj.geometry.attributes.position.needsUpdate = true; obj.geometry.computeVertexNormals(); V.updateField(obj, { name: isMode ? 'Modal amplitude (normalised)' : 'σ_vM', unit: isMode ? '' : 'MPa', values: vals, min: 0, max: isMode ? 1 : vmMax, cmap: 'turbo' });
      H.set(isMode ? [['Mode frequency', fmt(f, 3) + ' Hz'], ['Displayed scale', '×' + mag.value]] : [['Load factor λ', fmt(lam, 3)], ['Max von Mises', fmt(lam * vmMax, 3) + ' MPa'], ['Max deflection (true)', fmt(lam * A.maxRel * 1000, 3) + ' mm'], ['Displayed scale', '×' + mag.value]]);
    } });
  };
  sel.onchange = start;
  V.fit(); start();
  return { dispose: () => pb?.dispose() };
}

// ---------------- 2-D animation: deformed side-section FE mesh, von Mises under a load ramp, then mode 1 ----------------
async function chassis2D(el, A, modesS) {
  const { anim2d } = await import('../core/sim3d.js');
  el.append(h('p', { class: 'small muted' }, 'First 6 s: the solved load case ramped 0 → 1 → 0 on the side-section mesh, elements coloured by von Mises stress (fixed range), displacements magnified (factor shown). Next 4 s: the first elastic mode shape oscillating (eigenvector, normalised).'));
  const nodes = A.mesh.nodes, tris = A.mesh.tris, vm = A.els.map(e => e.vm), vmMax = Math.max(...vm);
  let mx = 0; for (let k = 0; k < nodes.length; k++) mx += A.sol.u[2 * k] / nodes.length; const U = nodes.map((_, k) => [A.sol.u[2 * k] - mx, A.rel[k]]), uMax = Math.max(1e-12, ...U.map(q => Math.hypot(...q)));
  const M = modesS[0]; let Um = null; if (M) { const f = new Float64Array(2 * nodes.length); A.solStiff.freeList.forEach((d, j) => f[d] = M.vec[j]); Um = nodes.map((_, k) => [f[2 * k], f[2 * k + 1]]); const mm = Math.max(1e-12, ...Um.map(q => Math.hypot(...q))); Um = Um.map(q => [q[0] / mm, q[1] / mm]); }
  const xs = nodes.map(n => n[0]), ys = nodes.map(n => n[1]), x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
  return anim2d(el, { duration: Um ? 10 : 6, label: 'Time', height: 360, draw: (g, t, W, H, c) => {
    const sc = Math.min((W - 60) / (x1 - x0), (H - 70) / (y1 - y0)), X = (x) => 30 + (x - x0) * sc, Y = (y) => H - 40 - (y - y0) * sc;
    const isMode = t > 6 && Um, lam = isMode ? Math.sin(2 * Math.PI * (t - 6) / 1.5) : (t < 3 ? t / 3 : 2 - t / 3), mag = isMode ? 0.06 * (x1 - x0) : 0.05 * (x1 - x0) / uMax;
    for (let e = 0; e < tris.length; e++) { const tri = tris[e]; g.beginPath(); tri.forEach((k, i) => { const d = isMode ? Um[k] : U[k], px = X(nodes[k][0] + lam * mag * d[0]), py = Y(nodes[k][1] + lam * mag * d[1]); i ? g.lineTo(px, py) : g.moveTo(px, py); }); g.closePath(); const v = isMode ? Math.abs(lam) * tri.reduce((a, k) => a + Math.hypot(...Um[k]), 0) / 3 : Math.abs(lam) * vm[e] / vmMax; const col = cmapRGB(v); g.fillStyle = col; g.fill(); }
    g.fillStyle = c.fg; g.fillText(isMode ? `Elastic mode 1: ${fmt(M.f, 3)} Hz (normalised shape)` : `Load factor λ = ${fmt(lam, 3)} · max σ_vM = ${fmt(lam * vmMax / 1e6, 3)} MPa · deformation ×${fmt(mag, 3)}`, 30, 18);
  } });
}
function cmapRGB(v) { const c = cmap(Math.max(0, Math.min(1, v)), 'turbo'); return `rgb(${c.map(Math.round).join(',')})`; }

// AeroArch geometry & section stiffness helpers (geometry from the parametric CAD, sections from the Suite-06 spar model)
import { vehicleAssembly as _va } from '../core/cad.js';
import { WING_PARAMS as _WP, buildWing as _bw } from './s06-wing.js';
function archGeom(veh) { return _va(veh, {}).meta.arch; }
function archEI(ctx) {
  const p6 = { ...Object.fromEntries(_WP.map(q => [q.key, q.default])), span: ctx.vehicle.wingHeight, area: ctx.vehicle.wingArea / (ctx.vehicle.nWings || 2), ...(ctx.store?.params?.('s06') || {}), ...(ctx.upstream('s06')?.sizedSpar || {}) };
  const W = _bw(p6, ctx), r = W.props(0), t = W.props(W.span); return { leg: r.EI, bridge: t.EI, hsTip: t.hs, Ecap: W.cover.Ex };
}
