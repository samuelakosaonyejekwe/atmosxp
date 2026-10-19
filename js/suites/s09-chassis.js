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
    { key: 'tPly', label: 'Ply thickness', unit: 'mm', type: 'number', min: 0.05, max: 2, step: 0.05, group: 'Laminate', default: 0.25 },
    { key: 'coreMat', label: 'Core material', type: 'select', group: 'Laminate', default: 'core-pet', options: coreMats.map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.material('chassisCore')?.id || 'core-pet', prov: 'library' }) },
    { key: 'tCore', label: 'Core thickness', unit: 'mm', type: 'number', min: 3, max: 60, step: 1, group: 'Laminate', default: 15 },
    { key: 'case', label: 'Load case', type: 'select', group: 'Loads', default: 'combined', options: [{ value: 'combined', label: 'Combined: bump + braking + wing thrust' }, { value: 'vertical', label: 'Vertical bump only' }, { value: 'braking', label: 'Emergency braking + wing thrust' }] },
    { key: 'loadFactor', label: 'Vertical load factor', unit: 'g', type: 'number', min: 1, max: 5, step: 0.1, group: 'Loads', default: 2.5, help: 'Road-bump design factor on all masses.' },
    { key: 'brakeDecel', label: 'Braking deceleration', unit: 'm/s²', type: 'number', min: 0, max: 12, step: 0.1, group: 'Loads', default: 7.8, auto: (c) => { const o = c.upstream('s12'); return o?.decel_ms2 ? { value: +o.decel_ms2.toFixed(2), prov: 'coupled' } : undefined; } },
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
  presets: [{ label: 'Bump 3 g', values: { case: 'vertical', loadFactor: 3 } }, { label: 'Emergency braking', values: { case: 'braking', brakeDecel: 9 } }, { label: 'Glass faces', values: { faceMat: 'gfrp-e', layup: '0,45,-45,90,90,-45,45,0', tPly: 0.3 } }, { label: 'Flax faces (low carbon)', values: { faceMat: 'nfrp-flax', layup: '0,45,-45,0,0,-45,45,0', tPly: 0.3 } }],

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
    out.recs.push({ level: 'sustain', text: `Face material embodied carbon ≈ ${FM.co2} kgCO₂e/kg; the shell holds ≈ ${fmt(A.shellMass * 2 * lam.rhoA / (2 * lam.rhoA + tc * (CM.rho || 100)) * FM.co2, 3)} kgCO₂e. Flax/bio-epoxy faces cut this by ≈ 80 % if stiffness targets can be met — use the preset to compare.`, basis: 'Materials registry' });
    out.recs.push({ level: 'good', text: 'Validate with a torsion rig (wheel-station loading) and strain-gauged bump test before occupied testing (specification gate 4).', basis: 'Validation hierarchy' });
    out.explain.push({ title: 'Why springs instead of fixed supports', text: 'A real chassis sits on compliant tyres; clamping it artificially stiffens the structure and hides bending. The model therefore supports it on tyre springs with only one longitudinal restraint, exactly enough to remove rigid-body motion.' });
    // ---- Revision 2.1 wing-root load path: mast-step bulkhead ring frame + keel beam (deployed, worst case) ----
    {
      const nW = veh.nWings || 2, mu = ctx.upstream('s10')?.muPeak ?? 0.95, ssf = ctx.upstream('s11')?.ssf ?? ctx.upstream('s01')?.ssf ?? (veh.track / 2) * (2 / 3) / veh.cgHeight;
      const zRoot = veh.wheelRadius + veh.height * 0.7, hCE = zRoot + 0.45 * veh.wingHeight, FySafe = Math.min(0.7 * mu * veh.mass * G, 0.8 * ssf * veh.mass * G * veh.cgHeight / hCE);
      const Mroot = FySafe / nW * (hCE - zRoot) * p.gustFactor; // deployed wing at the breakaway setting × dynamic factor (worst case before release)
      const Fbear = Mroot / p.stepDepth; // couple reacted as equal and opposite lateral forces at partner (deck) and step (keel)
      // ring frame: couple applied at the crown of a closed ring of radius ≈ body half-height; max ring moment ≈ M/2 (Roark, ring under a couple)
      const t = p.frameT / 1000, d = p.frameD / 1000, Zf = t * d * d / 6, sigF = 0.5 * Mroot / Zf, Fx = (FM.Xt ?? 600) * 1e6 * 0.35; // quasi-isotropic allowable
      const [kw, kh] = String(p.keelW).split('x').map(v => +v / 1000), tk = p.keelT / 1000, Ik = (kw * kh ** 3 - (kw - 2 * tk) * (kh - 2 * tk) ** 3) / 12, Lk = 0.5 * veh.wheelbase, Mk = Fbear * Lk / 4, sigK = Mk * (kh / 2) / Ik;
      const SFf = Fx / sigF, SFk = Fx / sigK;
      out.kpis.push({ label: 'Design wing-root moment (deployed, at breakaway × dyn.)', value: Mroot, unit: 'N·m', explain: `F_y,safe/n_wings × lever (${fmt(hCE - zRoot, 3)} m) × ${p.gustFactor}. Worst case is the deployed wing just before the breakaway releases.`, prov: 'calc' });
      out.kpis.push({ label: 'Bulkhead ring-frame SF / keel-beam SF', value: `${fmt(SFf, 3)} / ${fmt(SFk, 3)}`, unit: '', explain: `Ring frame σ = 0.5·M/Z = ${fmt(sigF / 1e6, 3)} MPa; keel σ = ${fmt(sigK / 1e6, 3)} MPa from bearing couple ${fmt(Fbear, 3)} N over ${fmt(Lk, 3)} m.`, prov: 'calc', status: Math.min(SFf, SFk) >= 2 ? 'ok' : 'bad' });
      out.verdicts.push({ test: 'Wing-root load path: bulkhead ring frame & keel (Rev 2.1)', status: Math.min(SFf, SFk) >= 2 ? 'pass' : Math.min(SFf, SFk) >= 1.3 ? 'warn' : 'fail', detail: `Root moment ${fmt(Mroot, 3)} N·m → partner/step forces ${fmt(Fbear, 3)} N; ring-frame SF ${fmt(SFf, 3)}, keel SF ${fmt(SFk, 3)} (target ≥ 2).`, fix: 'Increase frame laminate/depth, partner-to-step separation or keel section.' });
      out.tables.push({ title: 'Wing-root load path (mast-step bulkheads + keel)', cols: ['Item', 'Value'], rows: [['Breakaway side force (both wings)', `${fmt(FySafe, 4)} N`], ['Centre of effort above root (deployed)', `${fmt(hCE - zRoot, 4)} m`], ['Design root moment per wing', `${fmt(Mroot, 4)} N·m`], ['Partner / step bearing force', `${fmt(Fbear, 4)} N`], ['Ring-frame bending stress', `${fmt(sigF / 1e6, 4)} MPa (SF ${fmt(SFf, 3)})`], ['Keel-beam bending stress', `${fmt(sigK / 1e6, 4)} MPa (SF ${fmt(SFk, 3)})`]], explain: 'Hand-calculation check of the discrete load path that the side-section FE does not represent (out-of-plane wing moment).' });
    }
    out.outputs = { maxVonMises_MPa: vmMax / 1e6, minSF: Rmin, maxDefl_m: A.maxRel, torsionalStiffness_Nm_deg: KtDeg, bucklingFactor: BF, wrinklingFactor: WF, chassisMass, tsaiWuMax: 1 / Rmin, f1Hz: modesS[0]?.f, rideHz: modes[0]?.f, crashStroke_m: stroke, crushMass_kg: crushMass };
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
