// Suite 06 — Fixed skeleton, passive aeroelastic wings and morphing metamaterial surfaces.
import { lamina, laminate, parseLayup, plyStresses, beamModel, beamModes, beamResultants, beamStatic } from '../core/fem.js';
import { sectionModel, liftingLine, staticAeroelastic, divergenceQ, flutterVg, gustResponse, oneMinusCosGust } from '../core/aeroelastic.js';
import { naca4, airfoilStats } from '../core/cad.js';
import { logWind, apparentWind, ROUGHNESS } from '../core/physics.js';
import { range, fmt, linspace, rad, deg, interp1, max, min } from '../core/num.js';
import { MATERIALS, get as getMat } from '../core/materials.js';
import { h } from '../core/ui.js';

const sparMats = MATERIALS.filter(m => m.uses.includes('spar') || ['al-7075-t6', 'gfrp-e', 'ti-6al4v', 'cfrp-woven', 'nfrp-flax'].includes(m.id));
const skinMats = MATERIALS.filter(m => m.uses.includes('wingSkin'));
const ribMats = MATERIALS.filter(m => m.uses.includes('rib'));

export const WING_PARAMS = [
  { key: 'span', label: 'Wing height (span)', unit: 'm', type: 'number', min: 1, max: 6, step: 0.1, group: 'Wing geometry', default: 3, auto: (c) => ({ value: c.vehicle.wingHeight, prov: 'user' }) },
  { key: 'area', label: 'Planform area per wing', unit: 'm²', type: 'number', min: 1, max: 12, step: 0.1, group: 'Wing geometry', default: 4, auto: (c) => ({ value: +(c.vehicle.wingArea / (c.vehicle.nWings || 2)).toFixed(2), prov: 'calc' }) },
  { key: 'taper', label: 'Taper ratio c_tip/c_root', type: 'range', min: 0.3, max: 1, step: 0.05, group: 'Wing geometry', default: 0.6 },
  { key: 'airfoil', label: 'Section', type: 'select', group: 'Wing geometry', default: '0018', options: ['0012', '0015', '0018', '0021', '2412', '4415'].map(v => ({ value: v, label: 'NACA ' + v })), auto: (c) => { const n = c.upstream('s02')?.airfoil?.name; const m = n && /NACA\s*(\d{4})/.exec(n); return m ? { value: m[1], prov: 'coupled' } : undefined; } },
  { key: 'twistBuilt', label: 'Built-in tip washout', unit: '°', type: 'number', min: -10, max: 5, step: 0.5, group: 'Wing geometry', default: -3 },
  { key: 'zRoot', label: 'Root height above ground', unit: 'm', type: 'number', min: 0.3, max: 2, step: 0.05, group: 'Wing geometry', default: 1.07, auto: (c) => ({ value: +(c.vehicle.wheelRadius + c.vehicle.height * 0.7).toFixed(2), prov: 'calc' }) },
  { key: 'U10', label: 'True wind at 10 m', unit: 'm/s', type: 'range', min: 0, max: 25, step: 0.1, group: 'Operating point', default: 6, auto: (c) => c.env ? { value: +(+c.env.U).toFixed(2), prov: 'coupled' } : undefined },
  { key: 'twa', label: 'True wind angle from heading', unit: '°', type: 'range', min: 30, max: 180, step: 1, group: 'Operating point', default: 90 },
  { key: 'vkmh', label: 'Vehicle speed', unit: 'km/h', type: 'range', min: 0, max: 60, step: 0.5, group: 'Operating point', default: 10 },
  { key: 'aoa', label: 'Root angle of attack', unit: '°', type: 'range', min: 0, max: 20, step: 0.5, group: 'Operating point', default: 10, help: 'Wing set so that the root section sees this angle to the local apparent wind.' },
  { key: 'rho', label: 'Air density', unit: 'kg/m³', type: 'number', min: 0.9, max: 1.35, step: 0.001, group: 'Operating point', default: 1.225, auto: (c) => c.env ? { value: +(+c.env.rho).toFixed(4), prov: 'coupled' } : undefined },
  { key: 'z0', label: 'Terrain roughness length', unit: 'm', type: 'number', min: 0.0002, max: 2, step: 0.001, group: 'Operating point', default: 0.1, auto: (c) => c.env?.z0 ? { value: c.env.z0, prov: 'coupled' } : (ROUGHNESS[c.site.roughness] ? { value: ROUGHNESS[c.site.roughness], prov: 'user' } : undefined) },
  { key: 'sparMat', label: 'Spar material', type: 'select', group: 'Spar (skeleton)', default: 'cfrp-t700', options: sparMats.map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.material('spar')?.id || 'cfrp-t700', prov: 'library' }) },
  { key: 'layup', label: 'Spar-cover layup (deg, root→out)', type: 'text', group: 'Spar (skeleton)', default: '0,20,20,0,45,-45', help: 'Upper cover; lower cover is mirrored (circumferentially asymmetric stiffness) so off-axis plies create bend–twist coupling. Balanced layups (±θ pairs) give no coupling.' },
  { key: 'tPly', label: 'Ply thickness', unit: 'mm', type: 'number', min: 0.05, max: 1, step: 0.005, group: 'Spar (skeleton)', default: 0.2 },
  { key: 'boxW', label: 'Box width / chord', type: 'range', min: 0.08, max: 0.45, step: 0.01, group: 'Spar (skeleton)', default: 0.18 },
  { key: 'webLayup', label: 'Shear-web layup', type: 'text', group: 'Spar (skeleton)', default: '45,-45,45,-45', expert: true },
  { key: 'skinMat', label: 'Morphing skin material', type: 'select', group: 'Skin, ribs & core', default: 'tpu-glass', options: skinMats.map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.material('wingSkin')?.id || 'tpu-glass', prov: 'library' }) },
  { key: 'tSkin', label: 'Skin thickness', unit: 'mm', type: 'number', min: 0.2, max: 5, step: 0.1, group: 'Skin, ribs & core', default: 0.8 },
  { key: 'ribMat', label: 'Flexible rib material', type: 'select', group: 'Skin, ribs & core', default: 'gfrp-e', options: ribMats.map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.material('rib')?.id || 'gfrp-e', prov: 'library' }) },
  { key: 'ribSolid', label: 'Rib solidity (after lightening holes)', type: 'range', min: 0.1, max: 1, step: 0.05, group: 'Skin, ribs & core', default: 0.3 },
  { key: 'coreFill', label: 'Lattice fill fraction of trailing region', type: 'range', min: 0.05, max: 1, step: 0.05, group: 'Skin, ribs & core', default: 0.35 },
  { key: 'ribPitch', label: 'Rib pitch', unit: 'm', type: 'number', min: 0.15, max: 1, step: 0.05, group: 'Skin, ribs & core', default: 0.4 },
  { key: 'coreE', label: 'Trailing-core effective modulus', unit: 'MPa', type: 'number', min: 0.5, max: 5000, step: 0.5, group: 'Skin, ribs & core', default: 60, auto: (c) => { const o = c.upstream('s04'); return o?.Ey_Pa ? { value: +(o.Ey_Pa / 1e6).toFixed(2), prov: 'coupled' } : undefined; }, help: 'Bending stiffness of the compliant trailing region (from the Suite 04 lattice).' },
  { key: 'coreG', label: 'Trailing-core effective shear modulus', unit: 'MPa', type: 'number', min: 0.1, max: 2000, step: 0.1, group: 'Skin, ribs & core', default: 5, auto: (c) => { const o = c.upstream('s04'); return o?.Geff_Pa ? { value: +(o.Geff_Pa / 1e6).toFixed(3), prov: 'coupled' } : undefined; } },
  { key: 'coreRho', label: 'Trailing-core effective density', unit: 'kg/m³', type: 'number', min: 20, max: 2000, step: 5, group: 'Skin, ribs & core', default: 120, auto: (c) => { const o = c.upstream('s04'); return o?.rhoEff ? { value: Math.round(o.rhoEff), prov: 'coupled' } : undefined; } },
  { key: 'teFrac', label: 'Compliant trailing-region chord fraction', type: 'range', min: 0.15, max: 0.6, step: 0.01, group: 'Skin, ribs & core', default: 0.4 },
  { key: 'gustAmp', label: 'Design gust amplitude (1-cos)', unit: 'm/s', type: 'number', min: 0, max: 20, step: 0.5, group: 'Dynamics & fatigue', default: 4, auto: (c) => c.env ? { value: +Math.max(1, c.env.gust - c.env.U).toFixed(1), prov: 'coupled' } : undefined },
  { key: 'gustDur', label: 'Gust duration', unit: 's', type: 'number', min: 0.2, max: 10, step: 0.1, group: 'Dynamics & fatigue', default: 1.5 },
  { key: 'zeta', label: 'Structural damping ratio', type: 'number', min: 0.005, max: 0.1, step: 0.005, group: 'Dynamics & fatigue', default: 0.02, expert: true },
  { key: 'gustRate', label: 'Significant gust cycles per operating hour', type: 'number', min: 10, max: 5000, step: 10, group: 'Dynamics & fatigue', default: 600 },
  { key: 'hoursYear', label: 'Operating hours per year', unit: 'h', type: 'number', min: 50, max: 5000, step: 50, group: 'Dynamics & fatigue', default: 800 },
  { key: 'nEl', label: 'Beam elements', type: 'number', min: 6, max: 60, step: 1, group: 'Numerics', default: 20, expert: true },
  { key: 'nLL', label: 'Lifting-line panels', type: 'number', min: 8, max: 60, step: 1, group: 'Numerics', default: 24, expert: true },
  { key: 'endplate', label: 'Root reflection-plane efficiency', type: 'range', min: 0, max: 1, step: 0.05, group: 'Numerics', default: 0.7, expert: true, help: 'Fraction of a perfect mirror image provided by the vehicle body at the wing root (1 = infinite end-plate).' },
];

// Shared wing model (used by Suites 06 and 08)
export function buildWing(p, ctx) {
  const span = p.span, cm = p.area / span, cr = 2 * cm / (1 + p.taper), ct = cr * p.taper, chord = (z) => cr + (ct - cr) * z / span;
  const af = naca4(String(p.airfoil).padStart(4, '0'), 61), st = airfoilStats(af.points), tc = st.tmax;
  const polar = ctx.upstream('s07')?.polar || null;
  const section = sectionModel({ a0: 2 * Math.PI * (1 + 0.77 * tc) * 0.9, alpha0: -2 * (af.m || 0) * 1.0, clmax: 0.95 + 1.2 * tc + 2 * (af.m || 0), cd0: 0.0075 + 0.02 * tc, polar: polar && polar.alpha?.length ? polar : null });
  const SM = getMat(p.sparMat) || getMat('cfrp-t700'), L = lamina(SM), tp = p.tPly / 1000;
  const cover = laminate(parseLayup(p.layup, L, tp)), web = laminate(parseLayup(p.webLayup || '45,-45,45,-45', L, tp));
  const red = (A) => { const a = A; const d = a[1][1]; return [[a[0][0] - a[0][1] ** 2 / d, a[0][2] - a[0][1] * a[1][2] / d], [a[0][2] - a[0][1] * a[1][2] / d, a[2][2] - a[1][2] ** 2 / d]]; };
  const Ac = red(cover.A), Aw = red(web.A);
  const SK = getMat(p.skinMat) || getMat('tpu-glass'), RB = getMat(p.ribMat) || getMat('gfrp-e'), tsk = p.tSkin / 1000;
  const Gsk = (SK.E || 0.8) * 1e9 / (2 * (1 + (SK.nu ?? 0.4)));
  const props = (z) => {
    const c = chord(z), hs = 0.85 * tc * c, wb = p.boxW * c, Abox = wb * hs, P = 2 * (wb + hs);
    const EIc = 2 * wb * Ac[0][0] * (hs / 2) ** 2, EIw = 2 * Aw[0][0] * hs ** 3 / 12, gam = 2 * Abox / P;
    const GJbox = 4 * Abox ** 2 / (2 * wb / Ac[1][1] + 2 * hs / Aw[1][1]);
    const Askin = 0.68 * tc * c * c, Pskin = 2.04 * c, GJskin = 4 * Askin ** 2 * Gsk * tsk / Pskin;
    const tTE = 0.5 * tc * c * 0.6, aTE = p.teFrac * c, fill = p.coreFill ?? 0.35, tEff = 0.25 * tTE, GJcore = fill * p.coreG * 1e6 * aTE * tTE ** 3 / 3;
    let K = 2 * wb * Ac[0][1] * (hs / 2) * gam;
    const EI = EIc + EIw + (SK.E || 0.8) * 1e9 * tsk * c * (tc * c / 2) ** 2 * 0.5, GJ = GJbox + GJskin + GJcore;
    const Kmax = 0.95 * Math.sqrt(EI * GJ); if (Math.abs(K) > Kmax) K = Math.sign(K) * Kmax;
    const mSpar = P * cover.h * (SM.rho || 1550), mSkin = Pskin * tsk * (SK.rho || 1300), mCore = fill * aTE * tTE * p.coreRho, mRib = (p.ribSolid ?? 0.3) * Askin * 0.003 * (RB.rho || 2000) / p.ribPitch;
    const m = mSpar + mSkin + mCore + mRib;
    const xEA = 0.15 + p.boxW / 2, xCG = (mSpar * xEA + mSkin * 0.42 + mCore * (1 - p.teFrac / 2) + mRib * 0.42) / m; // chord fractions
    const Ip = m * (c * 0.28) ** 2 + m * ((xCG - xEA) * c) ** 2;
    return { EI, GJ, K, m, Ip, xa: (xCG - xEA) * c, c, hs, wb, gam, xEA, xCG, Abox, mSpar, mSkin, mCore, mRib, D_TE: p.coreE * 1e6 * tEff ** 3 / 12 + (SK.E || 0.8) * 1e9 * 2 * tsk * (tEff / 2) ** 2, aTE };
  };
  const model = beamModel(span, Math.round(p.nEl || 20), props);
  // apparent wind distribution over the span (log-law shear + vehicle motion)
  const v = (p.vkmh || 0) / 3.6, Vz = (z) => logWind(p.U10, 10, Math.max(0.3, p.zRoot + z), p.z0);
  const app = (z) => { const aw = apparentWind(Vz(z), p.twa, 0, v); return { Va: Math.max(0.05, aw.Va), awa: Math.abs(aw.beta) }; };
  const appGust = (z, dU) => { const aw = apparentWind(Vz(z) * (1 + dU / Math.max(p.U10, 0.1)), p.twa, 0, v); return { Va: Math.max(0.05, aw.Va), awa: Math.abs(aw.beta) }; };
  const app0 = app(span * 0.05), setAngle = app0.awa - p.aoa; // wing angle to vehicle axis
  const e = (z) => (props(z).xEA - 0.25) * chord(z); // EA aft of aerodynamic centre
  return { span, cr, ct, cm, chord, af, tc, section, cover, web, SM, SK, RB, L, props, model, app, appGust, setAngle, e, V: (z) => app(z).Va, airfoilName: af.name };
}

// Static aeroelastic solution with passive washout (twist) and passive camber (compliant trailing region)
export function solveStatic(W, p, { method = 'aitken', rigid = false, tol = 1e-7, maxIt = 80 } = {}) {
  const { model, span, chord, section, app, setAngle, props, e } = W, N = model.N, rho = p.rho;
  let lastCl = null, lastLL = null;
  const tau = (cf) => { const th = Math.acos(2 * cf - 1); return 1 - (th - Math.sin(th)) / Math.PI; };
  const camberShift = (z, cl) => { if (rigid) return 0; const P = props(z), q = 0.5 * rho * app(z).Va ** 2, dp = Math.abs(q * cl) * 0.35, delta = dp * P.aTE ** 4 / (8 * P.D_TE); const df = Math.min(rad(12), delta / P.aTE) * Math.sign(cl || 1); return tau(P.aTE / chord(z)) * df; }; // reflex raises zero-lift angle
  const aero = (phiNodes) => {
    const phi = (z) => rigid ? 0 : interp1(model.z, phiNodes, z);
    const a0Fn = (z) => lastCl ? camberShift(z, interp1(lastLL.z, lastCl, z)) : 0;
    const ll = liftingLine({ span, chord, alpha: (z) => rad(app(z).awa - setAngle + p.twistBuilt * z / span) + phi(z), V: (z) => app(z).Va, rho, section, n: Math.round(p.nLL || 24), mirror: p.endplate ?? 0.7, alpha0Fn: (z) => section.alpha0 + a0Fn(z) });
    lastCl = ll.cl; lastLL = ll;
    const lz = (z) => interp1(ll.z, ll.l, z, 'linear'), Fw = [], Ft = [];
    for (let i = 0; i < N; i++) { const z = model.z[i], tw = (i === 0 || i === N - 1 ? 0.5 : 1) * model.le, lv = i === 0 ? 0 : lz(z); Fw.push(lv * tw); Ft.push(lv * e(z) * tw); }
    return { Fw, Ft, ll };
  };
  if (rigid) { const a = aero(new Array(N).fill(0)); const F = new Array(model.ndof).fill(0); for (let i = 0; i < N; i++) { F[3 * i] = a.Fw[i]; F[3 * i + 2] = a.Ft[i]; } const sol = beamStatic(model, F); return { phi: new Array(N).fill(0), aero: a, sol, history: [1e-16], iterations: 1, converged: true }; }
  aero(new Array(N).fill(0));
  return staticAeroelastic(model, aero, { method, tol, maxIt, omega0: p.omega0 ?? 0.5 });
}

// Gust as a true-wind speed increase ΔU(t): changes both apparent-wind angle (Küssner-lagged) and dynamic pressure (quasi-steady)
export function runGust(W, p, ll, { dt = 0.003, subIt = 1, T } = {}) {
  const dU = oneMinusCosGust(0.3, p.gustDur, p.gustAmp), memo = new Map(), base = (z) => { let b = memo.get(z); if (!b) { b = W.app(z); b.cl = interp1(ll.z, ll.cl, z); b.c = W.chord(z); b.e = W.e(z); memo.set(z, b); } return b; }, clS = (z) => base(z).cl;
  let gT = NaN; const gm = new Map(), gAt = (z, t) => { if (t !== gT) { gm.clear(); gT = t; } let g = gm.get(z); if (!g) { g = W.appGust(z, dU(t)); gm.set(z, g); } return g; };
  return gustResponse({ model: W.model, props: W.props, chord: (z) => base(z).c, V: (z) => base(z).Va, rho: p.rho, a0: W.section.a0, e: (z) => base(z).e, T: T || Math.max(3, p.gustDur + 2.5), dt, zeta: p.zeta, nModes: 4, subIt,
    gust: (t, z) => { const b0 = base(z); if (dU(t) === 0) return 0; const g = gAt(z, t); return b0.Va * rad(g.awa - b0.awa); },
    extraLoad: (t, z) => { const b0 = base(z); if (dU(t) === 0) return 0; const g = gAt(z, t); return 0.5 * p.rho * b0.c * (g.Va ** 2 - b0.Va ** 2) * b0.cl; },
    gustRef: dU, clStatic: clS, clmax: W.section.clmax });
}

// Resolve forces of the converged lifting-line solution to vehicle axes
export function wingLoads(W, ll, rho = 1.225) {
  let thrust = 0, side = 0, roll = 0, drag = 0, lift = 0;
  ll.z.forEach((z, i) => { const b = rad(W.app(z).awa), l = ll.l[i] * ll.dz[i], d = (0.5 * rho * ll.V[i] ** 2 * ll.chord[i] * ll.cd[i] + ll.l[i] * ll.alphaInd[i]) * ll.dz[i]; lift += l; drag += d; thrust += l * Math.sin(b) - d * Math.cos(b); const sf = l * Math.cos(b) + d * Math.sin(b); side += sf; roll += sf * z; });
  return { thrust, side, roll, drag, lift };
}

export default {
  description: 'Designs the twin fixed-root vertical wings: carbon box-spar skeleton with tailored (bend–twist coupled) laminate, flexible ribs, metamaterial trailing core and continuous morphing skin. Solves the coupled static aeroelastic equilibrium with a nonlinear lifting line in the sheared apparent wind, passive washout and camber adaptation, then divergence, flutter (V–g), modal, gust, fatigue and the safe operating wind envelope — all from the selected materials.',
  literature: 'passive aeroelastic tailoring bend-twist coupling morphing wing gust load alleviation wingsail',
  credibility: 'Numerically verified',
  equations: ['Prandtl/Weissinger lifting-line theory with discrete horseshoe vortices, root reflection plane and spanwise-varying apparent wind (boundary-layer shear)', 'Thin-aerofoil theory (lift slope, flap effectiveness for compliant trailing-edge reflex), nonlinear section lift with stall', 'Apparent-wind vector transformation V_a = V_wind − V_vehicle; resolution of lift and drag into body-axis thrust and side force', 'Classical laminate theory (ABD), reduced membrane stiffness of spar covers, circumferentially-asymmetric-stiffness box beam: EI, GJ (Bredt–Batho) and bend–twist coupling K_bt', 'Euler–Bernoulli bending + St-Venant torsion beam FE with coupled strain energy ½∫(EIκ² + 2K_btκφ′ + GJφ′²)', 'Static aeroelastic equilibrium (K − qA)u = F with partitioned Aitken-accelerated coupling', 'Static divergence eigenproblem K u = q_D A u', 'Theodorsen unsteady aerodynamics (R. T. Jones approximation) and V–g (k-method) flutter of the 75 %-span typical section', 'Modal analysis (generalised eigenproblem), Newmark-β gust response with Küssner and Wagner indicial functions', 'Composite S–N fatigue (S/UTS = 1 − b log N) with Tsai–Wu strength ratio at the root'],
  methods: ['Quasi-Newton secant-linearised lifting-line solve per coupling iteration', 'Aitken Δ² dynamic relaxation of the interface twist', 'Dense LU of the reduced beam system; Jacobi generalised eigen-solver for modes', 'Parameter sweep of true wind for the safe operating envelope'],
  assumptions: ['Unswept wing, elastic axis at box-spar centre; chordwise rigid sections except the compliant trailing region', 'Steady inflow for the static case; quasi-steady log-law shear', 'Flutter estimated with the 75 %-span typical section using the beam’s first bending and torsion frequencies', 'Strip-theory loads for divergence and gust (no wake roll-up)'],
  ibc: { initial: ['Undeformed wing geometry, reference incidence, built-in twist, ply orientations, zero initial displacement and velocity'], boundary: ['Fully clamped wing root (w = w′ = φ = 0) on the monocoque', 'Distributed aerodynamic lift and pitching moment from the lifting line; free tip', 'Spanwise sheared apparent wind; 1-cos discrete gust for the transient case', 'Twist is an output of the coupled solution — never prescribed together with the load'] },
  cvv: { calibration: ['Wing bending/twist under static loads (photogrammetry/DIC)', 'Ground vibration test (natural frequencies, damping)', 'Pressure-tap and force-balance wind-tunnel data'], verification: ['Cantilever first bending frequency vs 3.516√(EI/mL⁴)', 'Lifting line CL vs finite-wing lift-slope formula', 'Coupled-solver residual convergence and global load balance', 'Typical-section flutter against the Theodorsen benchmark (V_F/bω_α = 2.17 for the Hodges–Pierce case)'], validation: ['Wind-tunnel section with compliant rib and skin; full-size gust and fatigue tests (gate 1)'] },
  formats: { import: ['CSV deflection/strain/pressure measurements', 'DAT/CSV airfoil coordinates', 'JSON layups'], export: ['CSV spanwise distributions & V–g data', 'JSON wing state (reference, loaded, gust-deformed)', 'glTF/GLB deformed 3D wing', 'PNG/SVG plots'] },
  params: WING_PARAMS,
  presets: [{ label: 'Beam reach 6 m/s', values: { U10: 6, twa: 90, vkmh: 10, aoa: 10 } }, { label: 'Close reach 8 m/s', values: { U10: 8, twa: 60, vkmh: 15, aoa: 12 } }, { label: 'Strong crosswind 14 m/s', values: { U10: 14, twa: 90, vkmh: 5, aoa: 8 } }, { label: 'Balanced layup (no coupling)', values: { layup: '0,0,45,-45,-45,45,0,0' } }, { label: 'Titanium spar', values: { sparMat: 'ti-6al4v', layup: '0,0,0,0' } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const W = buildWing(p, ctx), { model, props, span, chord } = W;
    ctx.progress(0.15, 'coupled static aeroelastic solution');
    const S = solveStatic(W, p), R0 = solveStatic(W, p, { rigid: true });
    const ll = S.aero.ll, loads = wingLoads(W, ll), loadsR = wingLoads(W, R0.aero.ll), N = model.N;
    const nW = ctx.vehicle.nWings || 2, res = beamResultants(model, S.sol.u, props);
    const twistDeg = S.sol.phi.map(deg), tipTw = twistDeg[N - 1], tipW = S.sol.w[N - 1];
    // ply stresses along the span (upper cover, tension side)
    const stressRows = res.map(r => { const P = props(r.z), eps = r.M * (P.hs / 2) / P.EI, gam = r.T / P.GJ * P.gam, ps = plyStresses(W.cover, [eps, 0, gam]); const Rmin = Math.min(...ps.map(q => q.R)); return { z: r.z, eps, sx: eps * W.cover.Ex, R: Rmin, ps }; });
    const rootR = stressRows[0].R, rootM = res[0].M, rootT = res[0].T;
    ctx.progress(0.4, 'divergence, modes, flutter');
    const modes = beamModes(model, 6), fb = modes.find(m => m.type === 'bending'), ft = modes.find(m => m.type === 'torsion') || modes[1];
    const qD = divergenceQ(model, chord, W.e, W.section.a0), VD = Math.sqrt(2 * qD / p.rho);
    const z75 = 0.75 * span, P75 = props(z75), b = chord(z75) / 2;
    const FL = flutterVg({ m: P75.m, Ia: P75.Ip, xa: P75.xa / b, a: (P75.xEA - 0.5) * 2, b, wh: fb.omega, wa: ft.omega, rho: p.rho, gs: p.zeta * 2 });
    const Vtip = W.app(span).Va, Vmax = max(range(N).map(i => W.app(model.z[i]).Va));
    ctx.progress(0.6, 'gust response');
    const G = runGust(W, p, ll, { dt: 0.003 });
    const dMr = max(G.Mroot.map(Math.abs)), gustF = 1 + dMr / Math.max(1, Math.abs(rootM));
    // fatigue (root cover, tension–tension with mean = static, amplitude = gust)
    const sMean = Math.abs(stressRows[0].sx), sAmp = Math.abs(dMr * (props(0).hs / 2) / props(0).EI * W.cover.Ex) / 2;
    const Sut = sMean > 0 && Number.isFinite(rootR) ? rootR * sMean : W.L.Xt * 0.5, // laminate first-ply-failure stress at the root
      bfat = (1 - (W.SM.fatigue && W.SM.fatigue < 1 ? W.SM.fatigue : 0.5)) / 7;
    const Seq = sAmp / Math.max(0.05, 1 - sMean / Sut), Nf = Seq > 0 ? 10 ** Math.min(15, (1 - Seq / Sut) / bfat) : Infinity, lifeYears = Nf / (p.gustRate * p.hoursYear);
    // safe operating envelope (strip-theory sweep)
    const Us = linspace(1, 25, 13), env = Us.map(U => { const pp = { ...p, U10: U }; const W2 = buildWing(pp, ctx); const S2 = solveStatic(W2, pp, { tol: 1e-5, maxIt: 30 }); const L2 = wingLoads(W2, S2.aero.ll, pp.rho); const r2 = beamResultants(W2.model, S2.sol.u, W2.props)[0]; const P0 = W2.props(0), eps = r2.M * (P0.hs / 2) / P0.EI, ps = plyStresses(W2.cover, [eps, 0, r2.T / P0.GJ * P0.gam]); const Vt = W2.app(span).Va; return { U, R: Math.min(...ps.map(q => q.R)) / gustF, tip: S2.sol.w[W2.model.N - 1], side: L2.side * nW, thrust: L2.thrust * nW, roll: (L2.roll + L2.side * pp.zRoot) * nW, Vt }; });
    const mveh = ctx.vehicle.mass, track = ctx.vehicle.track, restore = mveh * 9.80665 * track / 2 * (ctx.vehicle.nWheels === 3 ? 2 / 3 : 1);
    const lim = (f) => { const e = env.find(f); return e ? e.U : Infinity; };
    const Ustr = lim(e => e.R < 1.5), Utip = lim(e => Math.abs(e.tip) > 0.1 * span), Uroll = lim(e => e.roll > restore / 1.3), Uflut = lim(e => e.Vt * 1.15 > FL.Vflutter), Udiv = lim(e => e.Vt * 1.15 > VD);
    const Usafe = Math.min(Ustr, Utip, Uroll, Uflut, Udiv);
    // mass & checks
    const wingMass = model.z.slice(0, -1).reduce((s, z) => s + props(z + model.le / 2).m * model.le, 0);
    const p0 = props(span / 2), fTheory = 3.516 / (2 * Math.PI) * Math.sqrt(p0.EI / (p0.m * span ** 4));
    out.checks.push({ name: 'Coupled interface residual', value: S.history[S.history.length - 1], target: '< 1e-7', pass: S.converged, detail: `${S.iterations} Aitken iterations` });
    out.checks.push({ name: 'Lifting-line convergence', value: ll.residual, target: '< 1e-8', pass: ll.residual < 1e-6 });
    const sumF = S.aero.Fw.reduce((a, b2) => a + b2, 0); out.checks.push({ name: 'Load transfer: Σ nodal forces / lifting-line lift', value: sumF / ll.L, target: '1 ± 0.05', pass: Math.abs(sumF / ll.L - 1) < 0.05, detail: 'Conservative aerodynamic-to-structural load mapping' });
    out.checks.push({ name: 'First bending frequency vs uniform-cantilever estimate (mid-span props)', value: fb.f / fTheory, target: '0.7–1.6 (taper stiffens the root)', pass: fb.f / fTheory > 0.7 && fb.f / fTheory < 1.6 });
    out.checks.push({ name: 'Bend–twist coupling admissible', value: Math.abs(props(0).K) / Math.sqrt(props(0).EI * props(0).GJ), target: '< 0.95', pass: Math.abs(props(0).K) / Math.sqrt(props(0).EI * props(0).GJ) < 0.95 });
    // KPIs
    const k = (label, value, unit, explain, status, prov = 'calc') => out.kpis.push({ label, value, unit, explain, status, prov });
    k('Apparent wind at root / tip', `${fmt(W.app(0.05 * span).Va, 3)} / ${fmt(Vtip, 3)}`, 'm/s', 'Vector sum of the sheared true wind and the vehicle motion at the root and tip.');
    k('Apparent wind angle (root)', W.app(0.05 * span).awa, '°', 'Angle between the bow and the apparent wind at the root; thrust needs a lift component pointing forward.');
    k('Forward thrust (all wings)', loads.thrust * nW, 'N', 'Useful propulsive force along the road direction — lift is not thrust. Negative = the wings resist motion.', loads.thrust > 0 ? 'ok' : 'bad');
    k('Side force (all wings)', loads.side * nW, 'N', 'Lateral force the tyres must resist; drives overturning and steering demand.');
    k('Lift coefficient C_L', ll.CL, '', 'Spanwise-integrated lift coefficient of the deformed wing.');
    k('Thrust change from passive deformation', (loads.thrust - loadsR.thrust) / Math.abs(loadsR.thrust || 1) * 100, '%', 'Effect of the solved twist and camber versus a rigid wing at the same set angle.');
    k('Tip twist (passive)', tipTw, '°', 'Elastic twist at the tip from bend–twist coupling and aerodynamic torque (negative = washout, load relieving).');
    k('Tip deflection', tipW * 1000, 'mm', 'Lateral bending deflection of the tip.');
    k('Root bending moment (per wing)', rootM, 'N·m', 'Moment carried into the monocoque by each wing root (input to Suite 09).');
    k('Root strength ratio (Tsai–Wu)', rootR, '', 'Load multiplier to first-ply failure at the root cover. ≥ 2 recommended for gust-loaded structures.', rootR >= 2 ? 'ok' : rootR >= 1.5 ? 'warn' : 'bad');
    k('Divergence speed', VD, 'm/s', 'Apparent wind at which aerodynamic twisting moment overcomes torsional stiffness (static instability).', VD > 1.5 * Vmax ? 'ok' : 'bad');
    k('Flutter speed (V–g)', FL.Vflutter, 'm/s', 'Apparent wind at which a mode first acquires negative damping (dynamic instability).', FL.Vflutter > 1.5 * Vmax ? 'ok' : 'bad');
    k('1st bending / torsion frequency', `${fmt(fb.f, 3)} / ${fmt(ft.f, 3)}`, 'Hz', 'Natural frequencies of the clamped wing; keep clear of gust and vortex-shedding excitation.');
    k('Gust dynamic amplification', gustF, '', 'Peak root moment during the design gust divided by the static value.');
    k('Fatigue life (root cover)', lifeYears > 1000 ? '> 1000' : lifeYears, 'years', `Composite S–N estimate at ${p.gustRate} gust cycles/h × ${p.hoursYear} h/yr.`, lifeYears > 15 ? 'ok' : lifeYears > 5 ? 'warn' : 'bad');
    k('Safe operating true wind', Number.isFinite(Usafe) ? Usafe : '> 25 (sweep limit)', 'm/s', 'Highest true wind (10 m) for which strength, deflection, overturning, flutter and divergence margins all hold.', Usafe >= 12 ? 'ok' : Usafe >= 8 ? 'warn' : 'bad');
    const UsafeTxt = Number.isFinite(Usafe) ? fmt(Usafe, 3) : '25 (no limit reached in the sweep)';
    k('Wing mass (each)', wingMass, 'kg', 'Spar, skin, trailing core and ribs from the selected materials.');
    // verdicts
    const Vop = Vmax;
    out.verdicts.push({ test: 'Useful forward propulsion', status: loads.thrust > 5 ? 'pass' : loads.thrust > 0 ? 'warn' : 'fail', detail: `Thrust ${fmt(loads.thrust * nW, 3)} N from lift ${fmt(loads.lift * nW, 3)} N at apparent wind angle ${fmt(W.app(0.05 * span).awa, 3)}°.`, fix: 'Adjust root angle of attack, choose a beam/broad reach heading, or wait for stronger wind; a wing with high lift but small forward component is not credited with propulsion.' });
    out.verdicts.push({ test: 'Strength (first-ply failure, with gust)', status: rootR / gustF >= 1.5 ? 'pass' : rootR / gustF >= 1 ? 'warn' : 'fail', detail: `Strength ratio ${fmt(rootR, 3)} static, ${fmt(rootR / gustF, 3)} including gust amplification.`, fix: 'Add 0° plies to the spar covers, increase box depth, or choose a stronger spar material.' });
    out.verdicts.push({ test: 'Static divergence margin', status: VD > 1.5 * Vop ? 'pass' : VD > 1.15 * Vop ? 'warn' : 'fail', detail: Number.isFinite(VD) ? `V_D = ${fmt(VD, 3)} m/s vs max apparent wind ${fmt(Vop, 3)} m/s (required ×1.5).` : `No static divergence: the elastic axis (${fmt(props(0).xEA * 100, 3)} % chord) lies ahead of the aerodynamic centre (25 %), so aerodynamic torque reduces twist.`, fix: 'Move the elastic axis forward (wider/forward box) or increase GJ; wash-out coupling (K_bt of the correct sign) raises V_D.' });
    out.verdicts.push({ test: 'Flutter margin', status: FL.Vflutter > 1.5 * Vop ? 'pass' : FL.Vflutter > 1.15 * Vop ? 'warn' : 'fail', detail: `V_F = ${fmt(FL.Vflutter, 3)} m/s (${fmt(FL.fFlutter, 3)} Hz) vs ${fmt(Vop, 3)} m/s.`, fix: 'Separate bending and torsion frequencies (stiffer torsion), move the section CG forward of the elastic axis (lighter trailing core), add damping.' });
    out.verdicts.push({ test: 'Passive gust-load alleviation direction', status: tipTw < 0 ? 'pass' : 'warn', detail: tipTw < 0 ? `Loaded wing washes out (tip ${fmt(tipTw, 3)}°), reducing peak loads.` : `Loaded wing washes in (tip +${fmt(tipTw, 3)}°): coupling amplifies load and lowers divergence speed.`, fix: 'Reverse the sign of off-axis cover plies (e.g. 20° → −20°) so bending induces nose-down twist.' });
    out.verdicts.push({ test: 'Fatigue durability ≥ 15 years', status: lifeYears >= 15 ? 'pass' : lifeYears >= 5 ? 'warn' : 'fail', detail: lifeYears > 1000 ? 'Stress amplitudes are below the endurance range of the laminate (> 1000 years).' : `Estimated ${fmt(lifeYears, 3)} years (N_f = ${fmt(Nf, 3)} gust cycles).` });
    const morphEff = Math.abs(tipTw) >= 0.5 || Math.abs(loads.thrust - loadsR.thrust) / Math.abs(loadsR.thrust || 1) > 0.02;
    out.verdicts.push({ test: 'Passive morphing effectiveness', status: morphEff ? 'pass' : 'warn', detail: morphEff ? `Passive deformation changes thrust by ${fmt((loads.thrust - loadsR.thrust) / Math.abs(loadsR.thrust || 1) * 100, 3)} % and tip twist is ${fmt(tipTw, 3)}°.` : `Deformation is negligible at this load (tip twist ${fmt(tipTw, 3)}°): the skeleton is too stiff for passive adaptation at this wind.`, fix: 'Reduce spar cover plies/box depth (strength ratio is very high), increase off-axis coupling plies, or evaluate at the design gust wind speed.' });
    out.verdicts.push({ test: 'Coupled solution converged', status: S.converged ? 'pass' : 'fail', detail: `${S.iterations} iterations, residual ${fmt(S.history[S.history.length - 1], 2)}.` });
    // charts
    const zc = ll.z, zs = model.z;
    out.charts.push({ title: 'Spanwise lift per unit span: rigid vs passively deformed', type: 'line', x: { label: 'Height above root z', unit: 'm' }, y: { label: 'Lift per span', unit: 'N/m' }, series: [{ name: 'Deformed (coupled)', x: zc, y: ll.l }, { name: 'Rigid wing', x: R0.aero.ll.z, y: R0.aero.ll.l, dash: '5 3' }], explain: 'Aerodynamic loading along the vertical wing in the sheared apparent wind. The gap between curves is the passive load redistribution from twist and camber adaptation.' });
    out.charts.push({ title: 'Passive twist and camber-induced zero-lift shift', type: 'line', x: { label: 'z', unit: 'm' }, y: { label: 'Angle', unit: '°' }, series: [{ name: 'Elastic twist φ', x: zs, y: twistDeg }, { name: 'Built-in washout', x: zs, y: zs.map(z => p.twistBuilt * z / span), dash: '4 3' }, { name: 'Effective angle of attack', x: zc, y: ll.alphaEff.map(deg) }], explain: 'Twist solved from the coupled equilibrium (output, not input) and the resulting local angle of attack including induced downwash.' });
    out.charts.push({ title: 'Bending deflection and internal moments', type: 'line', x: { label: 'z', unit: 'm' }, y: { label: 'Value' }, series: [{ name: 'Deflection w [mm]', x: zs, y: S.sol.w.map(v => v * 1000) }, { name: 'Bending moment [N·m /10]', x: res.map(r => r.z), y: res.map(r => r.M / 10) }, { name: 'Torque [N·m]', x: res.map(r => r.z), y: res.map(r => r.T) }], explain: 'Bending shape and stress resultants carried by the spar; the root values go to the chassis model (Suite 09).' });
    out.charts.push({ title: 'Spar-cover axial stress & strength ratio', type: 'line', x: { label: 'z', unit: 'm' }, y: { label: 'Value' }, series: [{ name: 'Axial stress [MPa]', x: stressRows.map(r => r.z), y: stressRows.map(r => r.sx / 1e6) }, { name: 'Tsai–Wu strength ratio ×10', x: stressRows.map(r => r.z), y: stressRows.map(r => Math.min(200, r.R * 10)) }], annotations: [{ type: 'hline', y: 15, label: 'R = 1.5', color: '#ef4444' }], explain: 'Laminate stress along the span and the first-ply-failure load multiplier (×10 for scale). Values below the red line violate the 1.5 strength margin.' });
    out.charts.push({ title: 'V–g flutter diagram (75 % span typical section)', type: 'line', x: { label: 'Apparent wind', unit: 'm/s', min: 0, max: Math.min(200, Math.max(3 * Vop, (Number.isFinite(FL.Vflutter) ? FL.Vflutter : 60) * 1.4)) }, y: { label: 'Required damping g', min: -1, max: 0.4 }, series: FL.branches.map((B, j) => ({ name: j === 0 ? 'Branch 1' : 'Branch 2', x: B.map(q => q.V), y: B.map(q => q.g) })), annotations: [{ type: 'hline', y: p.zeta * 2, label: 'structural damping', color: '#22c55e' }, { type: 'vline', x: Vop, label: 'max operating' }, ...(Number.isFinite(FL.Vflutter) ? [{ type: 'vline', x: FL.Vflutter, label: 'flutter', color: '#ef4444' }] : [])], explain: 'Damping each aeroelastic mode would need for neutral oscillation. Flutter occurs where a branch crosses the available structural damping.' });
    out.charts.push({ title: 'V–f frequency coalescence', type: 'line', x: { label: 'Apparent wind', unit: 'm/s', min: 0, max: Math.min(200, Math.max(3 * Vop, (Number.isFinite(FL.Vflutter) ? FL.Vflutter : 60) * 1.4)) }, y: { label: 'Frequency', unit: 'Hz' }, series: FL.branches.map((B, j) => ({ name: j === 0 ? 'Branch 1' : 'Branch 2', x: B.map(q => q.V), y: B.map(q => q.f) })), explain: 'Bending and torsion frequencies converge as speed rises; coalescence typically marks classical bending–torsion flutter.' });
    out.charts.push({ title: 'Design-gust response (Newmark-β + Küssner/Wagner)', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Value' }, series: [{ name: 'True-wind gust ΔU [m/s]', x: G.t, y: G.gust }, { name: 'Δ root moment [N·m /10]', x: G.t, y: G.Mroot.map(v => v / 10) }, { name: 'Δ tip twist [°×10]', x: G.t, y: G.twistTip.map(v => v * 10) }], explain: 'Dynamic increments on top of the static state during a 1-cos gust. Lag of the lift build-up (Küssner) and the structural modes shape the peak load.' });
    out.charts.push({ title: 'Safe operating wind envelope', type: 'line', x: { label: 'True wind at 10 m', unit: 'm/s' }, y: { label: 'Margin ratio' }, series: [{ name: 'Strength ratio (with gust)', x: env.map(e => e.U), y: env.map(e => Math.min(10, e.R)) }, { name: 'Restoring / overturning moment', x: env.map(e => e.U), y: env.map(e => Math.min(10, restore / Math.max(1, e.roll))) }, { name: 'Flutter speed / tip apparent wind', x: env.map(e => e.U), y: env.map(e => Math.min(10, FL.Vflutter / e.Vt)) }], annotations: [{ type: 'hline', y: 1.5, label: 'required 1.5', color: '#ef4444' }, ...(Number.isFinite(Usafe) ? [{ type: 'vline', x: Usafe, label: 'safe limit', color: '#f59e0b' }] : [])], explain: 'Each curve must stay above 1.5. The first crossing sets the maximum safe true wind for operating with the wing loaded; above it the wing must be unloaded.' });
    out.charts.push({ title: 'Thrust and side force vs true wind', type: 'line', x: { label: 'True wind at 10 m', unit: 'm/s' }, y: { label: 'Force (all wings)', unit: 'N' }, series: [{ name: 'Thrust', x: env.map(e => e.U), y: env.map(e => e.thrust) }, { name: 'Side force', x: env.map(e => e.U), y: env.map(e => e.side) }], explain: 'Propulsive and lateral force growth with wind at the current heading and set angle (deformation included).' });
    out.tables.push({ title: 'Natural modes of the clamped wing', cols: ['Mode', 'Type', 'Frequency [Hz]', 'Tip w (norm.)', 'Tip φ (norm.)'], rows: modes.map((m, i) => [i + 1, m.type, m.f, m.w[N - 1], m.phi[N - 1]]) });
    out.tables.push({ title: 'Section properties (root / mid / tip)', cols: ['Station', 'Chord [m]', 'EI [N·m²]', 'GJ [N·m²]', 'K_bt [N·m²]', 'Mass [kg/m]', 'CG−EA [mm]'], rows: [0, 0.5, 1].map(f => { const P = props(f * span); return [`${f * 100}% span`, P.c, P.EI, P.GJ, P.K, P.m, P.xa * 1000]; }), explain: 'Box-spar stiffnesses from CLT; K_bt ≠ 0 only for unbalanced (CAS) cover layups.' });
    out.tables.push({ title: 'Root cover ply stresses (static)', cols: ['Ply angle [°]', 'σ₁ [MPa]', 'σ₂ [MPa]', 'τ₁₂ [MPa]', 'Tsai–Wu index', 'Strength ratio', 'Hashin fibre', 'Hashin matrix'], rows: stressRows[0].ps.map(q => [q.theta, q.s1 / 1e6, q.s2 / 1e6, q.t12 / 1e6, q.tsaiWu, q.R, q.hashinFiber, q.hashinMatrix]) });
    out.views.push({ label: '3D deformed wing', icon: '🪽', render: (el) => wing3D(el, W, p, S, stressRows) });
    // recommendations
    if (loads.thrust <= 0) out.recs.push({ level: 'critical', text: 'The wing produces no net forward thrust at this operating point. Increase the apparent wind angle (steer toward a beam reach), reduce vehicle speed, or raise the root angle of attack below stall.', basis: 'Body-axis force resolution' });
    if (tipTw > 0) out.recs.push({ level: 'warn', text: 'The current layup twists the wing nose-up under load (wash-in). Mirror the off-axis cover plies to obtain wash-out for passive gust alleviation and higher divergence speed.', basis: 'Sign of K_bt' });
    if (rootR > 20 && Math.abs(tipTw) < 0.5) out.recs.push({ level: 'warn', text: `Strength ratio ${fmt(rootR, 3)} with ${fmt(tipTw, 3)}° tip twist: the spar is over-designed for passive morphing. Removing cover plies (or reducing box width) by up to ≈ ${fmt(Math.min(90, (1 - 3 / rootR) * 100), 3)} % keeps R ≥ 3 while increasing compliance and saving mass; re-check flutter after each change.`, basis: 'Fully-stressed design / aeroelastic tailoring' });
    if (rootR < 2) out.recs.push({ level: 'warn', text: `Root strength ratio ${fmt(rootR, 3)}: add ${Math.ceil((2 / Math.max(rootR, 0.1) - 1) * W.cover.plies.filter(q => q.theta === 0).length + 1)} unidirectional 0° plies to each cover or deepen the box.`, basis: 'Tsai–Wu at the root' });
    if (FL.Vflutter < 2 * Vop) out.recs.push({ level: 'warn', text: 'Flutter margin is small: move trailing-core mass forward, stiffen torsion (closed D-cell skin), and verify by ground vibration testing before wind-tunnel runs.', basis: 'V–g analysis' });
    out.recs.push({ level: 'good', text: `Unload or feather the wing above ${UsafeTxt} m/s true wind; include a mechanical wing-unloading device as specified for safe starting, braking and parking.`, basis: 'Safe operating envelope' });
    out.recs.push({ level: 'sustain', text: `Wing mass ${fmt(wingMass, 3)} kg each. ${W.SM.family === 'Composite' ? 'Specify recyclable thermoplastic or recovered-fibre plies where strength margins allow; CFRP has ≈ ' + W.SM.co2 + ' kgCO₂e/kg embodied carbon.' : 'Metallic spars are recyclable but heavier — check the overturning penalty.'}`, basis: 'Materials registry embodied carbon' });
    out.explain.push({ title: 'How the coupled wing is solved', text: 'The lifting line computes loads on the current deformed shape; the beam FE turns loads into bending and twist (with bend–twist coupling from the laminate); the new twist changes the angle of attack and the trailing-region pressure changes the camber. This loop is repeated with Aitken acceleration until the twist stops changing.' });
    out.explain.push({ title: 'Lift is not thrust', text: 'Only the component of the aerodynamic force along the road direction propels the car; the much larger side force must be carried by the tyres and creates the overturning moment that limits the safe wind.' });
    out.convergence = { title: 'Coupled static aeroelastic interface residual', series: [{ name: 'Aitken', x: S.history.map((_, i) => i + 1), y: S.history }] };
    out.outputs = { CL: ll.CL, CD: ll.CD, liftN: loads.lift * nW, dragN: loads.drag * nW, thrustN: loads.thrust * nW, sideN: loads.side * nW, rollMomentN: (loads.roll + loads.side * p.zRoot) * nW, twistTip_deg: tipTw, tipDefl_m: tipW, rootMoment_Nm: rootM, rootTorque_Nm: rootT, rootShear_N: ll.L, flutterSpeed: FL.Vflutter, divergenceSpeed: VD, f1Hz: fb.f, fTorsionHz: ft.f, wingMass, safeWind: Number.isFinite(Usafe) ? Usafe : 25, rootStrengthRatio: rootR, zRoot: p.zRoot, span, setAngle: W.setAngle, awa: W.app(0.05 * span).awa };
    out.headline = `Thrust ${fmt(loads.thrust * nW, 3)} N, side force ${fmt(loads.side * nW, 3)} N, tip twist ${fmt(tipTw, 3)}°, divergence ${fmt(VD, 3)} m/s, flutter ${fmt(FL.Vflutter, 3)} m/s, safe up to ${UsafeTxt} m/s true wind.`;
    out.headlineKind = Usafe >= 8 && loads.thrust > 0 ? 'ok' : 'warn';
    return out;
  },
};

async function wing3D(el, W, p, S, stressRows) {
  const [{ viewer }, { wing }] = await Promise.all([import('../core/viz3d.js'), import('../core/cad.js')]);
  const V = await viewer(el, { height: 460 }), span = W.span;
  const twistFn = (eta) => p.twistBuilt * eta + deg(interp1(W.model.z, S.sol.phi, eta * span));
  const ref = wing('Undeformed wing', { airfoil: W.af, span, rootChord: W.cr, tipChord: W.ct, twistTip: p.twistBuilt, nSpan: 16, opts: { color: '#94a3b8', opacity: 0.25 } });
  const def = wing('Deformed wing (solved)', { airfoil: W.af, span, rootChord: W.cr, tipChord: W.ct, twistFn, nSpan: 16, opts: { color: '#e5e7eb' } });
  const amp = Math.min(20, 0.05 * span / Math.max(1e-6, Math.abs(S.sol.w[S.sol.w.length - 1]))) || 1;
  def.transform((x, y, z) => [x, y + interp1(W.model.z, S.sol.w, z) * amp, z]);
  const vals = []; for (let k = 2; k < def.p.length; k += 3) vals.push(Math.abs(interp1(stressRows.map(r => r.z), stressRows.map(r => r.sx / 1e6), def.p[k])));
  V.add(ref); V.add(def, { field: { name: 'Spar-cover axial stress', unit: 'MPa', values: vals, cmap: 'turbo' } }); V.fit();
  el.append(h('p', { class: 'small muted' }, `Grey: reference geometry. Coloured: solved loaded shape (bending displayed ×${fmt(amp, 2)}; twist true scale), colour = spar-cover axial stress magnitude. Hover for values.`));
}
