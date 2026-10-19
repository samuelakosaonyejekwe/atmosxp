// Suite 07 — Aerodynamics and computational fluid dynamics.
import { naca4, cst } from '../core/cad.js';
import { panelSolver, viscous, polar, coeffs, finiteWing } from '../core/aero.js';
import { resampleAirfoil } from '../core/mesh.js';
import { poiseuille } from '../core/lbm-core.js';
import { apparentWind, wingForces, moistAir } from '../core/physics.js';
import { linspace, range, fmt, gci, deg, rad, max, min, mean, G0, clamp } from '../core/num.js';
import { h, table, callout } from '../core/ui.js';
import { cmap } from '../core/chart.js';
import { vehicleFlowModel, sampledField, bodyStations } from '../core/flow3d.js';

const panelPts = (pts, N) => { const r = resampleAirfoil(pts, N); r.push(r[0].slice()); return r; };

export function resolveAirfoil(p, ctx) {
  const imp = ctx.store.project.imports?.airfoil || (ctx.experiments || []).find(d => d.kind === 'airfoil' && d.points);
  if (p.airfoilSrc === 'imported' && imp?.points?.length > 10) return { name: imp.name || 'Imported airfoil', points: imp.points, prov: 'user' };
  if (p.airfoilSrc === 'suite02') { const a = ctx.upstream('s02')?.airfoil; if (a?.points?.length) return { ...a, prov: 'coupled' }; }
  let code = /^\d{4}$/.test(String(p.naca)) ? String(p.naca) : '0018';
  // passive leeward camber morphing (Revision 2.2): the compliant section cambers to leeward on either tack, so the solved
  // section is the cambered shape NACA m4tt (m = morphed camber %); with symmetric loading it relaxes to the base section.
  if (p.camberMorph && (p.camberPct || 0) > 0) code = `${Math.min(9, Math.max(1, Math.round(p.camberPct)))}4${code.slice(2)}`;
  return { ...naca4(code, 161), prov: p.camberMorph ? 'calc' : 'generated' };
}

export default {
  description: 'Resolves the aerodynamic force on the twin vertical wings and body for the live apparent wind: viscous airfoil polars at the operating Reynolds number (panel method coupled to an integral boundary layer with transition and separation), finite-wing corrections, resolution of lift and drag into body-frame forward thrust and side force, optimal incidence, discretisation-convergence studies and a live 2-D lattice-Boltzmann large-eddy simulation of the unsteady wake.',
  literature: 'land yacht wing sail aerodynamics apparent wind thrust coefficient',
  credibility: 'Numerically verified',
  equations: [
    'Continuity and incompressible Navier–Stokes momentum equations (low-Mach regime; verified M < 0.3)',
    'Potential-flow limit: Laplace equation for the velocity potential, Hess–Smith source/vortex panel boundary-integral formulation with Kutta condition',
    'Kutta–Joukowski theorem (cross-check of pressure-integrated lift), Bernoulli equation for surface pressure coefficient',
    'Integral boundary-layer momentum equation: Thwaites method (laminar), Michel transition criterion, Head entrainment method with Ludwieg–Tillmann skin friction (turbulent), Squire–Young wake drag',
    'Kirchhoff separated-flow normal-force model and flat-plate post-stall normal-force law',
    'Prandtl lifting-line finite-wing correction (induced angle and induced drag, Oswald efficiency, end-plate effective aspect ratio)',
    'Apparent-wind vector transformation V_a = V_wind − V_vehicle; resolution of lift (⟂ V_a) and drag (∥ V_a) into ground- and body-fixed axes',
    'Lattice-Boltzmann (D2Q9, BGK) recovery of the Navier–Stokes equations with Smagorinsky sub-grid eddy viscosity (LES); momentum-exchange force integration',
  ],
  methods: ['Dense LU solution of the panel influence system; cosine-clustered panelling', 'Marching integral boundary layer from the stagnation point on both surfaces', 'Grid-convergence index (Roache/Celik) on panel count', 'LBM: half-way bounce-back walls, equilibrium inlet, zero-gradient outlet, specular free-slip lateral boundaries, moving-ground option', 'Optimal sheeting by exhaustive incidence search for each apparent-wind angle'],
  assumptions: ['2-D polars are corrected to 3-D with lifting-line theory; wing–wing and wing–body interference represented by an efficiency factor', 'The live LBM is a reduced-Reynolds-number 2-D simulation for flow-physics insight (wake, shedding, separation), not a replacement for full-Re polars', 'Semi-empirical stall model: separation location from Head’s method, normal-force reduction from Kirchhoff flow'],
  ibc: {
    initial: ['Uniform free-stream velocity field at the apparent-wind speed, atmospheric pressure and density from Suite 05', 'LBM: equilibrium distributions at inflow velocity, zero turbulence; previous solution used for restart'],
    boundary: ['Velocity inlet (uniform or log-law profile), pressure / zero-gradient outlet, far-field for panel method (velocity at infinity)', 'No-slip vehicle and wing surfaces; moving-ground boundary at vehicle speed in the body frame (avoids artificial floor boundary layer)', 'Kutta condition at the trailing edge; specular free-slip lateral boundaries; transient gust inflow from Suite 05 where applicable', 'Wind direction transformed from Earth (meteorological) coordinates into the vehicle body frame'],
  },
  cvv: {
    calibration: ['Wind-tunnel force-balance lift and drag polars', 'Airfoil pressure-tap Cp distributions', 'Transition/stall location from oil-flow or IR thermography'],
    verification: ['Thin-airfoil lift slope 2π(1+0.77 t/c)', 'Zero lift for symmetric section at zero incidence; d’Alembert zero inviscid drag', 'Kutta–Joukowski vs pressure-integrated lift', 'Panel-count grid convergence with GCI', 'LBM Poiseuille-flow analytical benchmark'],
    validation: ['Full apparent-wind-angle and speed envelope against wind-tunnel and controlled-track thrust measurements (not a single favourable point)', 'Wake and separation patterns against PIV / smoke visualisation'],
  },
  formats: { import: ['Airfoil coordinates .dat (Selig / Lednicer) via Data portal or Import mode', 'Wind-tunnel polars (CSV/XLSX: alpha, CL, CD)'], export: ['Polar CSV', 'Cp distribution CSV', 'Results JSON', 'PNG/SVG plots'] },
  params: [
    { key: 'speed', label: 'Vehicle speed', unit: 'km/h', type: 'range', min: 0, max: 80, step: 1, default: 20, group: 'Operating point' },
    { key: 'heading', label: 'Vehicle heading (travel direction)', unit: '° from N', type: 'range', min: 0, max: 359, step: 1, default: 0, group: 'Operating point', help: 'Combined with the true wind direction this sets the apparent-wind angle.' },
    { key: 'Uw', label: 'True wind at wing height', unit: 'm/s', type: 'number', min: 0, max: 40, step: 0.1, default: 6, group: 'Operating point', auto: (c) => { const e = c.upstream('s05') || c.env; return e ? { value: +(+e.Uz || 0).toFixed(2), prov: 'coupled' } : undefined; } },
    { key: 'dir', label: 'True wind from', unit: '°', type: 'number', min: 0, max: 359, step: 1, default: 90, group: 'Operating point', auto: (c) => { const e = c.upstream('s05') || c.env; return e ? { value: Math.round(e.dir ?? 90), prov: 'coupled' } : undefined; } },
    { key: 'incMode', label: 'Wing incidence', type: 'select', default: 'optimum', options: [{ value: 'optimum', label: 'Optimum for maximum thrust' }, { value: 'fixed', label: 'Fixed (strict fixed-root configuration)' }], group: 'Wing' },
    { key: 'incidence', label: 'Fixed incidence (chord to bow)', unit: '°', type: 'range', min: -30, max: 90, step: 1, default: 20, group: 'Wing' },
    { key: 'airfoilSrc', label: 'Airfoil source', type: 'select', default: 'naca', options: [{ value: 'naca', label: 'NACA 4-digit' }, { value: 'suite02', label: 'From Suite 02 (CAD)' }, { value: 'imported', label: 'Imported (.dat via Data portal / Import)' }], group: 'Wing' },
    { key: 'naca', label: 'NACA code (unloaded base section)', type: 'text', default: '0018', group: 'Wing', help: 'Specification starting section: NACA 0018 (symmetric when unloaded).' },
    { key: 'camberMorph', label: 'Passive leeward camber morphing', type: 'bool', default: true, group: 'Wing', help: 'Compliant trailing region cambers to leeward on either tack (Revision 2.2); the panel/boundary-layer solver then analyses the cambered section.' },
    { key: 'camberPct', label: 'Morphed camber', unit: '% chord', type: 'range', min: 0, max: 9, step: 1, default: 7, group: 'Wing' },
    { key: 'bodyLift', label: 'Body acts as a lifting surface', type: 'bool', default: true, group: 'Body aerodynamics', help: 'The monocoque plan-form (sharp tail, Kutta condition) is a low-aspect-ratio lifting body with ground reflection; its side force and forward thrust add to the wings.' },
    { key: 'bodyCamberPct', label: 'Passive side-panel camber (leeward)', unit: '% chord', type: 'range', min: 0, max: 8, step: 0.5, default: 4, group: 'Body aerodynamics', help: 'Metamaterial side panels camber to the leeward side under load on either tack.' },
    { key: 'bodyFill', label: 'Side-area fill factor', type: 'range', min: 0.5, max: 1, step: 0.01, default: 0.78, group: 'Body aerodynamics', expert: true },
    { key: 'bodyE', label: 'Body span efficiency', type: 'range', min: 0.5, max: 1, step: 0.01, default: 0.8, group: 'Body aerodynamics', expert: true },
    { key: 'wingState', label: 'Wing configuration', type: 'select', default: 'deployed', group: 'Wing', options: [{ value: 'deployed', label: 'Deployed (full span)' }, { value: 'reefed', label: 'Reefed (upper stage telescoped)' }] },
    { key: 'endplate', label: 'Body end-plate factor on wing aspect ratio (sealed root gap)', type: 'number', min: 1, max: 2, step: 0.05, default: 1.6, group: 'Wing', expert: true, help: 'Wing roots sit on the body, which acts as a partial reflection plane.' },
    { key: 'oswald', label: 'Oswald span efficiency', type: 'number', min: 0.5, max: 1, step: 0.01, default: 0.85, group: 'Wing', expert: true },
    { key: 'interf', label: 'Other interference efficiency (body, gap)', type: 'number', min: 0.5, max: 1.1, step: 0.01, default: 1.0, group: 'Wing', expert: true, help: 'Tandem downwash is now modelled explicitly; this factor covers remaining body/gap effects.' },
    { key: 'transition', label: 'Transition model', type: 'select', default: 'free', options: [{ value: 'free', label: 'Free (Michel criterion)' }, { value: 'forced', label: 'Forced at x/c (trip)' }], group: 'Numerics' },
    { key: 'xtr', label: 'Trip location', unit: 'x/c', type: 'number', min: 0.01, max: 1, step: 0.01, default: 0.05, group: 'Numerics', expert: true },
    { key: 'panels', label: 'Panels', type: 'number', min: 40, max: 300, step: 10, default: 140, group: 'Numerics', expert: true },
    { key: 'alphaMax', label: 'Polar maximum incidence', unit: '°', type: 'number', min: 10, max: 30, step: 1, default: 22, group: 'Numerics', expert: true },
    { key: 'breakaway', label: 'Free-feathering breakaway (D3)', type: 'bool', default: true, group: 'Wing', help: 'Incidence lock releases at the safe side force; above it the wings weathervane (C_L ≈ 0). Thrust results respect this safe envelope.' },
    { key: 'lbmShape', label: 'Live LBM geometry', type: 'select', default: 'airfoil', options: [{ value: 'airfoil', label: 'Wing section at operating α' }, { value: 'vehicle', label: 'Vehicle side profile + moving ground' }], group: 'Live 2-D LBM-LES' },
    { key: 'lbmRe', label: 'LBM Reynolds number (reduced)', type: 'number', min: 100, max: 20000, step: 100, default: 1500, group: 'Live 2-D LBM-LES' },
    { key: 'lbmCs', label: 'Smagorinsky constant Cs', type: 'number', min: 0, max: 0.3, step: 0.01, default: 0.14, group: 'Live 2-D LBM-LES', expert: true },
    { key: 'lbmNx', label: 'LBM grid cells (x)', type: 'number', min: 160, max: 640, step: 20, default: 360, group: 'Live 2-D LBM-LES', expert: true },
  ],
  presets: [{ label: 'Beam reach 6 m/s', values: { Uw: 6, dir: 90, heading: 0, speed: 15 } }, { label: 'Close-hauled', values: { Uw: 7, dir: 45, heading: 0, speed: 15 } }, { label: 'Broad reach', values: { Uw: 7, dir: 135, heading: 0, speed: 20 } }, { label: 'Headwind', values: { Uw: 6, dir: 0, heading: 0, speed: 15 } }, { label: 'Stationary start', values: { speed: 0, Uw: 6, dir: 90 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const veh = ctx.vehicle, env = ctx.upstream('s05') || ctx.env || {};
    const air = env.rho ? { rho: env.rho, nu: env.nu || 1.5e-5 } : moistAir(101325, 20, 60);
    const rho = air.rho, nu = air.nu;
    const v = p.speed / 3.6, AW = apparentWind(p.Uw, p.dir, p.heading, v), Va = Math.max(AW.Va, 1e-3), beta = AW.beta, ab = Math.abs(beta);
    const w2 = ctx.upstream('s02')?.wing, nW = veh.nWings || 2, spanDep = w2?.span || veh.wingHeight, reefSpan = Math.min(spanDep, veh.wingReefSpan ?? spanDep);
    const span = p.wingState === 'reefed' ? reefSpan : spanDep, S = (w2?.area || veh.wingArea) * span / spanDep, Sw = S / nW, cmean = Sw / span;
    const AR = span * span / Sw, ARe = AR * p.endplate, Re = Math.max(2e4, Va * cmean / nu);
    const af = resolveAirfoil(p, ctx), pts = panelPts(af.points, p.panels);
    ctx.progress(0.1, 'polar');
    const xtr = p.transition === 'forced' ? p.xtr : 1;
    const pol = polar(pts, Re, { alphas: range(p.alphaMax + 7).map(i => i - 6), xtrU: xtr, xtrL: xtr });
    await ctx.yield();
    const C3 = (a) => { const c = coeffs(pol, a); if (Math.abs(a) > pol.alphaStall + 8) return { ...c, CDi: 0 }; const f = finiteWing(c.CL, c.CD, ARe, p.oswald); return { CL: f.CL, CD: f.CD, CDi: f.CDi, regime: c.regime }; };
    // Body as a lifting surface: plan-form airfoil of chord L (sharp tail → Kutta), height h with ground reflection,
    // AR_eff = 2h²/S_side; low-AR lift CL = (πAR/2)(α + 2m) + 2 sin²α cosα (attached + cross-flow), induced drag CL²/(πAR e).
    // Zero-lift drag is already in the vehicle C_dA term, so only lift and induced drag are added here.
    const Lb = veh.length, hb = veh.height, Sb = p.bodyFill * Lb * hb, ARb = 2 * hb * hb / Sb, mB = (p.bodyCamberPct || 0) / 100, hCEb = veh.wheelRadius + 0.5 * hb;
    const bodyAt = (bAbs, VA = Va) => {
      if (!p.bodyLift || bAbs < 0.5) return { L: 0, Di: 0, thrust: 0, side: 0, CL: 0 };
      const a = rad(Math.min(bAbs, 180 - bAbs)), sgn = bAbs > 90 ? -1 : 1; // the body axis is fixed; beyond 90° the tail leads (no camber benefit)
      const CL = (Math.PI * ARb / 2) * (a + (bAbs <= 90 ? 2 * mB : 0)) * Math.cos(a) ** 2 + 2 * Math.sin(a) ** 2 * Math.cos(a), q = 0.5 * rho * VA * VA, L = q * Sb * CL, Di = q * Sb * CL * CL / (Math.PI * ARb * p.bodyE);
      const b = rad(bAbs); void sgn; return { L, Di, CL, thrust: L * Math.sin(b) - Di * Math.cos(b), side: L * Math.abs(Math.cos(b)) + Di * Math.sin(b) };
    };
    // Tandem interference: the downstream wing sits in the front wing's induced sidewash ε ≈ 2C_L/(πAR_eff), weighted by
    // how much it is actually downstream (|cos β|) and decaying with the cross-flow offset gap·|sin β| (in chords).
    const cr = cmean * 1.1, gapX = Math.max(0.36 * veh.length, 1.12 * cr);
    const epsAt = (b, CLf) => nW < 2 ? 0 : deg(2 * CLf / (Math.PI * ARe)) * Math.abs(Math.cos(rad(b))) * Math.exp(-gapX * Math.abs(Math.sin(rad(b))) / (0.5 * cmean));
    const forceAt = (bAbs, alpha, VA = Va, tandem = true) => {
      const c1 = C3(alpha); let c = c1, eps = 0;
      if (tandem && nW >= 2) { eps = epsAt(bAbs, c1.CL); const c2 = C3(alpha - eps), e = rad(eps); c = { ...c1, CL: (c1.CL + c2.CL * Math.cos(e)) / 2, CD: (c1.CD + c2.CD + c2.CL * Math.sin(e)) / 2 }; }
      const F = wingForces(VA, bAbs, c.CL * p.interf, c.CD, S, rho), Db = 0.5 * rho * veh.CdA * VA * VA, B = bodyAt(bAbs, VA);
      return { ...F, c, eps, Db, body: B, net: F.thrust + B.thrust - Db * Math.cos(rad(bAbs)), sideTot: F.side + B.side + Db * Math.sin(rad(bAbs)) };
    };
    // D3 free-feathering root: safe side force and the feathered (weathervaned) state, C_L ≈ 0, C_D ≈ 0.02
    const mu0 = ctx.upstream('s10')?.muPeak ?? 0.95, ssf0 = ctx.upstream('s11')?.ssf ?? ctx.upstream('s01')?.ssf ?? (veh.track / 2) * (2 / 3) / veh.cgHeight, hCE0 = veh.wheelRadius + veh.height * 0.7 + span * 0.45;
    const FyGrip = 0.7 * mu0 * veh.mass * G0, FyRoll = 0.8 * ssf0 * veh.mass * G0 * veh.cgHeight / hCE0, FySafe = Math.min(FyGrip, FyRoll);
    const feathered = (bAbs, VA = Va) => { const F = wingForces(VA, bAbs, 0, 0.02, S, rho), Db = 0.5 * rho * veh.CdA * VA * VA, B = bodyAt(bAbs, VA); return { ...F, c: { CL: 0, CD: 0.02, regime: 'feathered' }, eps: 0, Db, body: B, net: F.thrust + B.thrust - Db * Math.cos(rad(bAbs)), sideTot: F.side + B.side + Db * Math.sin(rad(bAbs)), feathered: true }; };
    const safe = (F, bAbs) => (p.breakaway === false || F.sideTot <= FySafe) ? F : feathered(bAbs);
    // optimum incidence at current apparent-wind angle (α = β − δ)
    const alphaScan = linspace(-10, 40, 251); let best = { net: -Infinity };
    for (const a of alphaScan) { const F = forceAt(ab, a); if ((p.breakaway === false || F.sideTot <= FySafe) && F.net > best.net) best = { ...F, alpha: a }; }
    if (!Number.isFinite(best.net)) best = { ...feathered(ab), alpha: 0 };
    const alphaOp = p.incMode === 'optimum' ? best.alpha : ab - p.incidence, delta = ab - alphaOp;
    const FopLocked = forceAt(ab, alphaOp), Fop = safe(FopLocked, ab), FopNoTandem = forceAt(ab, alphaOp, Va, false);
    // thrust vs apparent-wind angle
    const awa = range(37).map(i => i * 5), thrOpt = [], sideOpt = [], incOpt = [], thrFix = [], cxOpt = [];
    const thrUnsafe = [], thrNoTandem = [];
    for (const b of awa) { let bb = { net: -Infinity }, bu = -Infinity, bn = -Infinity; for (const a of linspace(-5, 35, 81)) { const F = forceAt(b, a); bu = Math.max(bu, F.net); bn = Math.max(bn, forceAt(b, a, Va, false).net); if ((p.breakaway === false || F.sideTot <= FySafe) && F.net > bb.net) bb = { ...F, alpha: a }; } if (!Number.isFinite(bb.net)) bb = { ...feathered(b), alpha: b }; thrOpt.push(bb.net); sideOpt.push(bb.sideTot); incOpt.push(b - bb.alpha); thrFix.push(safe(forceAt(b, b - p.incidence), b).net); cxOpt.push(bb.net / (0.5 * rho * Va * Va * S)); thrUnsafe.push(bu); thrNoTandem.push(bn); }
    ctx.progress(0.5, 'verification');
    // geometry levers: centre of effort & CG
    const L = veh.length, x1W = -L * 0.42 + 0.10 * L + 0.25 * cr, wingX = [x1W, x1W + gapX].slice(0, nW), xCE = mean(wingX) + 0.05 * cmean, cg = ctx.upstream('s01')?.cg || [0, 0, veh.cgHeight];
    const zCE = veh.wheelRadius + veh.height * 0.7 + span * 0.45, yaw = Fop.sideTot * (xCE - cg[0]), heel = (Fop.sideTot - (Fop.body?.side || 0)) * zCE + (Fop.body?.side || 0) * hCEb, restoring = veh.mass * G0 * (veh.track / 2) * 0.6;
    const Froll = veh.mass * G0 * veh.Crr;
    // panel-count convergence (inviscid CL at 6°)
    const Ns = [50, 100, 200], cls = Ns.map(N => panelSolver(panelPts(af.points, N)).solve(6).CL), G = gci(cls[2], cls[1], cls[0], 1 / Ns[2], 1 / Ns[1], 1 / Ns[0]);
    // verification benchmarks
    const thin = panelSolver(panelPts(naca4('0006', 161).points, 160)), clT = thin.solve(2).CL / rad(2), clTh = 2 * Math.PI * (1 + 0.77 * 0.06);
    const sym = panelSolver(panelPts(naca4('0012', 161).points, 160)), s0 = sym.solve(0), s5 = sym.solve(5);
    const pois = poiseuille({ ny: 21, steps: 6000 });
    const solOp = pol.solver.solve(Math.max(-15, Math.min(25, alphaOp))), vOp = viscous(pol.solver, Math.max(-6, Math.min(p.alphaMax, alphaOp)), Re, { xtrU: xtr, xtrL: xtr });
    out.checks.push({ name: 'Thin-airfoil lift slope (NACA 0006)', value: clT, target: `2π(1+0.77t/c) = ${fmt(clTh, 4)} /rad ± 3 %`, pass: Math.abs(clT / clTh - 1) < 0.03 });
    out.checks.push({ name: 'Symmetric section at α = 0: |CL|', value: Math.abs(s0.CL), target: '< 1e-6', pass: Math.abs(s0.CL) < 1e-6 });
    out.checks.push({ name: 'd’Alembert: inviscid pressure drag at 5°', value: s5.CDp, target: '|CDp| < 2e-3', pass: Math.abs(s5.CDp) < 2e-3, detail: 'Closed body in potential flow must have zero drag' });
    out.checks.push({ name: 'Kutta–Joukowski vs pressure-integrated lift', value: s5.CLkj / s5.CL, target: '1.00 ± 0.02', pass: Math.abs(s5.CLkj / s5.CL - 1) < 0.02 });
    out.checks.push({ name: 'Net source strength (closed body)', value: s5.srcSum, target: '|Σq·l| < 5e-3', pass: Math.abs(s5.srcSum) < 5e-3 });
    out.checks.push({ name: 'Panel-count GCI (fine grid)', value: G.gci21, target: '< 1 %', pass: G.gci21 < 0.01, detail: `observed order p = ${fmt(G.p, 3)}, extrapolated CL = ${fmt(G.fext, 5)}` });
    out.checks.push({ name: 'LBM Poiseuille benchmark (relative L2 error)', value: pois.relL2, target: '< 2 %', pass: pois.relL2 < 0.02, detail: 'D2Q9 with Guo forcing and half-way bounce-back walls' });
    out.checks.push({ name: 'Force-resolution identity √(T²+S²) = √(L²+D²)', value: Math.hypot(Fop.thrust, Fop.side) / Math.hypot(Fop.L, Fop.D), target: '1.000', pass: Math.abs(Math.hypot(Fop.thrust, Fop.side) / (Math.hypot(Fop.L, Fop.D) || 1) - 1) < 1e-9 || Fop.L === 0 });
    out.checks.push({ name: 'Mach number', value: Va / 343, target: '< 0.3 (incompressible valid)', pass: Va / 343 < 0.3 });
    // KPIs
    const k = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('Apparent wind speed', Va, 'm/s', 'Magnitude of V_wind − V_vehicle at the wing; dynamic pressure scales with its square.', 'calc');
    k('Apparent-wind angle (from bow)', beta, '°', '0° = head-on, 90° = beam, 180° = from astern; sign gives the side the wind comes from.', 'calc');
    k('Reynolds number (mean chord)', Re, '', `V_a·c̄/ν with c̄ = ${fmt(cmean, 3)} m.`, 'calc');
    k('Angle of attack', alphaOp, '°', p.incMode === 'optimum' ? 'Incidence that maximises net forward force at this apparent-wind angle.' : 'From the fixed chord incidence: α = β − δ.', 'calc');
    k('Wing incidence δ (chord to bow)', delta, '°', 'Sheet angle relative to the vehicle centre-line.', p.incMode === 'optimum' ? 'generated' : 'user');
    k('Section CL / CD (2-D)', `${fmt(coeffs(pol, alphaOp).CL, 3)} / ${fmt(coeffs(pol, alphaOp).CD, 3)}`, '', 'Viscous section coefficients at the operating Reynolds number.', 'calc');
    k('Total aerodynamic lift', Fop.L, 'N', 'Perpendicular to the apparent wind. Lift is NOT propulsion — only its forward component is.', 'calc');
    k('Total aerodynamic drag', Fop.D, 'N', 'Parallel to the apparent wind (profile + induced).', 'calc');
    k('Wing forward thrust', Fop.thrust, 'N', 'L·sinβ − D·cosβ: the component along the direction of travel.', 'calc', Fop.thrust > 0 ? 'ok' : 'bad');
    k('Net forward force (wings − body drag)', Fop.net, 'N', 'Wing thrust minus the longitudinal component of body drag (C_dA from vehicle).', 'calc', Fop.net > Froll ? 'ok' : Fop.net > 0 ? 'warn' : 'bad');
    k('Rolling resistance (reference)', Froll, 'N', 'm·g·C_rr — the net force must exceed this to start and sustain motion (Suite 20 solves the full balance).', 'calc');
    k('Side force', Fop.sideTot, 'N', 'Lateral aerodynamic load reacted by tyre grip; drives overturning and steering demand (Suites 10–11).', 'calc');
    k('Thrust / lift ratio', Fop.L ? Fop.thrust / Fop.L : 0, '', 'How much of the lift is converted into useful propulsion at this angle.', 'calc');
    k('Yaw moment about CG', yaw, 'N·m', `Side force × (x_CE − x_CG) with x_CE ≈ ${fmt(xCE, 3)} m.`, 'calc');
    k('Heeling moment / restoring capacity', heel / restoring, '', 'Overturning moment relative to an approximate gravity restoring moment (detailed in Suite 11).', 'calc', heel / restoring < 0.5 ? 'ok' : heel / restoring < 0.9 ? 'warn' : 'bad');
    k('CL,max (2-D) / stall angle', `${fmt(pol.CLmax, 3)} @ ${pol.alphaStall}°`, '', 'From the boundary-layer separation model.', 'calc');
    k('Best thrust over all AWA', max(thrOpt), 'N', `Achieved near ${awa[thrOpt.indexOf(max(thrOpt))]}° apparent-wind angle with optimum incidence.`, 'calc');
    k('Tandem interference: rear-wing sidewash ε', FopLocked.eps, '°', `Induced sidewash of the front wing at the rear wing (gap ${fmt(gapX, 3)} m, |cos β| weighting, cross-flow decay).`, 'calc');
    k('Tandem interference penalty on net thrust', FopNoTandem.net - FopLocked.net, 'N', 'Net thrust lost because the rear wing operates in the front wing’s downwash (same incidence).', 'calc');
    k('Safe side force (breakaway)', FySafe, 'N', `min(0.7 μmg = ${fmt(FyGrip, 3)} N, 0.8·SSF·mg·h_cg/h_CE = ${fmt(FyRoll, 3)} N), SSF ${fmt(ssf0, 3)}, h_CE ${fmt(hCE0, 3)} m.`, 'calc');
    k('Wing root state', Fop.feathered ? 'FEATHERED' : 'locked', '', Fop.feathered ? `Locked side force ${fmt(FopLocked.sideTot, 3)} N would exceed ${fmt(FySafe, 3)} N; reported forces are the feathered loads.` : 'Operating inside the safe envelope.', 'calc', Fop.feathered ? 'warn' : 'ok');
    const bodyAWA = range(37).map(i => i * 5), bodyTh = bodyAWA.map(b => bodyAt(b).thrust), bodySd = bodyAWA.map(b => bodyAt(b).side), Bop = bodyAt(ab);
    k('Body lifting-surface thrust / side force', `${fmt(Bop.thrust, 3)} / ${fmt(Bop.side, 3)}`, 'N', `Monocoque as a low-AR airfoil: AR_eff = 2h²/S = ${fmt(ARb, 3)} with ground reflection, C_L = ${fmt(Bop.CL, 3)} (side-panel camber ${p.bodyCamberPct} %). Included in the net force.`, 'calc');
    k('Wing-root end-plate factor (body)', p.endplate, '×AR', 'Sealed root gap: the body acts as an end plate, raising the wings’ effective aspect ratio.', 'calc');
    out.charts.push({ title: 'Body as a wing: thrust and side force vs apparent-wind angle', x: { label: 'Apparent-wind angle β', unit: '°' }, y: { label: 'Force', unit: 'N' }, series: [{ name: 'Body thrust', x: bodyAWA, y: bodyTh }, { name: 'Body side force', x: bodyAWA, y: bodySd, dash: '5 3' }], annotations: [{ type: 'vline', x: ab, label: 'current β' }], explain: 'The plan-form acts as a very low aspect-ratio wing with a sharp trailing edge; ground reflection doubles its effective aspect ratio and the metamaterial side panels camber it to leeward. Its centre of effort is low (≈ wheel radius + h/2), so it adds thrust with little overturning moment.' });
    // Deployed vs reefed at 4 / 6 / 10 m/s beam true wind (current vehicle speed), optimum safe incidence, camber morphing,
    // tandem interference and the state-dependent breakaway limit (lower centre of effort when reefed).
    const perfAt = (U, spanX) => {
      const SX = (w2?.area || veh.wingArea) * spanX / spanDep, cmx = SX / nW / spanX, ARx = spanX * spanX / (SX / nW) * p.endplate, aw = apparentWind(U, p.heading + 90, p.heading, v), b = Math.abs(aw.beta), VA = Math.max(aw.Va, 1e-3);
      const hx = veh.wheelRadius + veh.height * 0.7 + spanX * 0.45, FyS = Math.min(FyGrip, 0.8 * ssf0 * veh.mass * G0 * veh.cgHeight / hx), gx = Math.max(0.36 * veh.length, 1.12 * cmx * 1.1);
      const cw = (a) => { const c = coeffs(pol, a); if (Math.abs(a) > pol.alphaStall + 8) return c; const f = finiteWing(c.CL, c.CD, ARx, p.oswald); return { CL: f.CL, CD: f.CD }; };
      let best = null;
      for (const a of linspace(-5, 35, 81)) { const c1 = cw(a), e = nW < 2 ? 0 : deg(2 * c1.CL / (Math.PI * ARx)) * Math.abs(Math.cos(rad(b))) * Math.exp(-gx * Math.abs(Math.sin(rad(b))) / (0.5 * cmx)), c2 = cw(a - e), CL = nW < 2 ? c1.CL : (c1.CL + c2.CL * Math.cos(rad(e))) / 2, CD = nW < 2 ? c1.CD : (c1.CD + c2.CD + c2.CL * Math.sin(rad(e))) / 2;
        const F = wingForces(VA, b, CL * p.interf, CD, SX, rho), Db = 0.5 * rho * veh.CdA * VA * VA, Bd = bodyAt(b, VA), net = F.thrust + Bd.thrust - Db * Math.cos(rad(b)), side = F.side + Bd.side + Db * Math.sin(rad(b));
        if ((p.breakaway === false || side <= FyS) && (!best || net > best.net)) best = { net, side, CL }; }
      if (!best) { const F = wingForces(VA, b, 0, 0.02, SX, rho), Db = 0.5 * rho * veh.CdA * VA * VA, Bd = bodyAt(b, VA); best = { net: F.thrust + Bd.thrust - Db * Math.cos(rad(b)), side: F.side + Bd.side + Db * Math.sin(rad(b)), CL: 0, feathered: true }; }
      return { ...best, FyS };
    };
    const windsP = [4, 6, 10], perfD = windsP.map(U => perfAt(U, spanDep)), perfR = windsP.map(U => perfAt(U, reefSpan));
    out.tables.push({ title: `Deployed (${fmt(spanDep, 3)} m) vs reefed (${fmt(reefSpan, 3)} m) — net forward force and side force, beam true wind at ${p.speed} km/h`, cols: ['True wind', 'Deployed net thrust [N]', 'Deployed side [N]', 'Reefed net thrust [N]', 'Reefed side [N]', 'Safe side limit dep./reef. [N]'], rows: windsP.map((U, i) => [`${U} m/s`, fmt(perfD[i].net, 4) + (perfD[i].feathered ? ' (feathered)' : ''), fmt(perfD[i].side, 4), fmt(perfR[i].net, 4) + (perfR[i].feathered ? ' (feathered)' : ''), fmt(perfR[i].side, 4), `${fmt(perfD[i].FyS, 4)} / ${fmt(perfR[i].FyS, 4)}`]), explain: `Optimum safe incidence, ${p.camberMorph ? `leeward camber ${p.camberPct} % (solved NACA ${af.name.replace('NACA ', '')})` : 'symmetric section'}, tandem downwash interference and the free-feathering limit. Deploy in light wind for area; reef in strong wind for a lower centre of effort and a higher breakaway margin.` });
    k('Net thrust at 4 / 6 m/s beam wind (deployed)', `${fmt(perfD[0].net, 3)} / ${fmt(perfD[1].net, 3)}`, 'N', 'Light-wind performance of the default rig with camber morphing (beam reach, current vehicle speed).', 'calc', perfD[0].net > Froll ? 'ok' : 'warn');
    // verdicts
    out.verdicts.push({ test: 'Useful forward propulsion', status: Fop.net > Froll ? 'pass' : Fop.net > 0 ? 'warn' : 'fail', detail: `Net forward force ${fmt(Fop.net, 3)} N vs rolling resistance ${fmt(Froll, 3)} N at ${fmt(Va, 3)} m/s apparent wind, β = ${fmt(beta, 3)}°.`, fix: 'Change heading toward a beam/broad reach (see thrust vs AWA), use optimum incidence, or wait for stronger wind; high lift alone does not give thrust.' });
    out.verdicts.push({ test: 'Wing operating regime', status: Math.abs(alphaOp) < pol.alphaStall - 2 ? 'pass' : Math.abs(alphaOp) <= pol.alphaStall + 2 ? 'warn' : 'fail', detail: `α = ${fmt(alphaOp, 3)}° vs stall ≈ ${pol.alphaStall}° (${C3(alphaOp).regime}).` });
    out.verdicts.push({ test: 'Overturning tendency', status: heel / restoring < 0.5 ? 'pass' : heel / restoring < 0.9 ? 'warn' : 'fail', detail: `Heeling moment ${fmt(heel, 3)} N·m vs approx. restoring ${fmt(restoring, 3)} N·m.`, fix: 'Reduce incidence/ sheet out, lower the centre of effort, widen the track or add ballast (Suite 11 quantifies).' });
    out.verdicts.push({ test: 'Side force within the safe envelope (D3)', status: Fop.feathered ? 'warn' : 'pass', detail: Fop.feathered ? `At this operating point the breakaway releases (locked side force ${fmt(FopLocked.sideTot, 3)} N > ${fmt(FySafe, 3)} N): wings feathered, thrust ${fmt(Fop.net, 3)} N. Depower (lower incidence) to stay locked.` : `Side force ${fmt(Fop.sideTot, 3)} N ≤ ${fmt(FySafe, 3)} N.` });
    out.verdicts.push({ test: 'Numerical verification', status: out.checks.every(c => c.pass) ? 'pass' : 'warn', detail: `${out.checks.filter(c => c.pass).length}/${out.checks.length} verification checks passed.` });
    // charts
    const A = pol.alpha;
    out.charts.push({ title: 'Lift curve: inviscid, viscous 2-D and finite-wing 3-D', x: { label: 'Angle of attack α', unit: '°' }, y: { label: 'CL' }, series: [{ name: 'Inviscid panel', x: A, y: pol.CLinv, dash: '4 3' }, { name: `Viscous 2-D (Re ${fmt(Re, 3)})`, x: A, y: pol.CL }, { name: `3-D (AR_eff ${fmt(ARe, 3)})`, x: A, y: A.map(a => C3(a).CL) }], annotations: [{ type: 'vline', x: alphaOp, label: 'operating α' }, { type: 'vline', x: pol.alphaStall, label: 'stall', color: '#ef4444' }], explain: 'The boundary layer reduces the lift slope and causes stall when separation moves forward; the finite span further reduces the slope through induced downwash.' });
    out.charts.push({ title: 'Drag polar and lift-to-drag ratio', x: { label: 'CD' }, y: { label: 'CL' }, series: [{ name: 'Section (2-D)', x: pol.CD, y: pol.CL, mode: 'lines+markers', r: 2 }, { name: 'Wing (3-D incl. induced)', x: A.map(a => C3(a).CD), y: A.map(a => C3(a).CL), mode: 'lines+markers', r: 2 }], explain: 'For land-sailing the relevant optimum is not maximum L/D but maximum forward component L·sinβ − D·cosβ, which depends on the apparent-wind angle.' });
    const nU = vOp.upper, nL = vOp.lower;
    const ple = solOp.panels, xs = ple.map(q => q.xm), cpN = Array.from(solOp.Cp).map(c => -c);
    out.charts.push({ title: `Surface pressure −Cp at α = ${fmt(alphaOp, 3)}° (inviscid)`, x: { label: 'x/c' }, y: { label: '−Cp' }, series: [{ name: '−Cp', x: xs, y: cpN, mode: 'markers', r: 2 }], explain: 'Suction peak near the leading edge and pressure recovery toward the trailing edge; steep adverse gradients drive transition and separation.' });
    out.charts.push({ title: 'Boundary layer on the suction side', x: { label: 'x/c' }, y: { label: 'Value' }, series: [{ name: 'θ×1000 (momentum thickness)', x: nU.x, y: nU.theta.map(t => t * 1000) }, { name: 'H (shape factor)', x: nU.x, y: nU.H }, { name: 'Cf×1000', x: nU.x, y: nU.cf.map(c => c * 1000) }], annotations: [{ type: 'vline', x: nU.xtr, label: 'transition', color: '#f59e0b' }, ...(nU.xsep !== null ? [{ type: 'vline', x: nU.xsep, label: 'separation', color: '#ef4444' }] : [])], explain: 'Momentum-thickness growth gives the wake drag; a shape factor above ≈2.4 indicates turbulent separation.' });
    out.charts.push({ title: 'Transition and separation location vs incidence (suction side)', x: { label: 'α', unit: '°' }, y: { label: 'x/c', min: 0, max: 1.05 }, series: [{ name: 'Transition', x: A, y: pol.xtrU }, { name: 'Separation', x: A, y: pol.xsepU.map(s => s === null ? 1 : s) }, { name: 'Attached fraction f', x: A, y: pol.f, dash: '4 3' }], explain: 'As incidence grows, transition moves to the leading edge and the separation point creeps forward until the section stalls.' });
    out.charts.push({ title: 'Forward thrust vs apparent-wind angle', x: { label: 'Apparent-wind angle β', unit: '°' }, y: { label: 'Force', unit: 'N' }, series: [{ name: 'Net thrust, optimum safe incidence', x: awa, y: thrOpt }, { name: `Net thrust, fixed δ=${p.incidence}°`, x: awa, y: thrFix, dash: '5 3' }, { name: 'Unconstrained (unsafe) maximum', x: awa, y: thrUnsafe, dash: '2 3' }, { name: 'Without tandem interference', x: awa, y: thrNoTandem, dash: '6 2' }, { name: 'Side force (optimum safe)', x: awa, y: sideOpt }], annotations: [{ type: 'hline', y: Froll, label: 'rolling resistance', color: '#22c55e' }, { type: 'hline', y: FySafe, label: 'safe side-force limit', color: '#ef4444' }, { type: 'vline', x: ab, label: 'current β' }], wide: true, explain: 'Optimum incidence is chosen inside the safe envelope (side force ≤ breakaway limit); the dotted curve shows what an unprotected wing would produce. At the current apparent-wind speed: head winds give no useful thrust, beam to broad reaches give the most. The gap between the two thrust curves is the penalty of a fixed (non-adjustable) wing root.' });
    out.charts.push({ title: 'Optimum wing incidence vs apparent-wind angle', x: { label: 'β', unit: '°' }, y: { label: 'Incidence δ', unit: '°' }, series: [{ name: 'δ_opt', x: awa, y: incOpt }], explain: 'Required chord angle to the vehicle axis; a strict fixed-root wing only works where this stays near its fixed value.' });
    const incScan = linspace(-20, 100, 121), thrInc = incScan.map(d2 => forceAt(ab, ab - d2).net);
    out.charts.push({ title: `Net thrust vs incidence at β = ${fmt(beta, 3)}°`, x: { label: 'Incidence δ', unit: '°' }, y: { label: 'Net forward force', unit: 'N' }, series: [{ name: 'Net thrust', x: incScan, y: thrInc }], annotations: [{ type: 'vline', x: delta, label: 'operating' }, { type: 'hline', y: 0, color: '#94a3b8' }], explain: 'Sheeting curve: too little incidence gives no lift, too much stalls the wing and turns the force into drag.' });
    out.charts.push({ title: 'Panel-count convergence (inviscid CL at 6°)', type: 'line', x: { label: 'Panels', log: true }, y: { label: 'CL' }, series: [{ name: 'CL(N)', x: Ns, y: cls, mode: 'lines+markers' }], annotations: [{ type: 'hline', y: G.fext, label: 'Richardson extrapolation', color: '#22c55e' }], explain: `Observed order ${fmt(G.p, 3)}; GCI on the finest discretisation ${fmt(100 * G.gci21, 3)} %.` });
    // flow field around the section (potential flow)
    const gx = linspace(-0.6, 1.6, 110), gy = linspace(-0.6, 0.6, 60), Z = [], Uf = [], Vf = [], mask = [];
    const poly = solOp.panels.map(q => [q.x1, q.y1]);
    const ins = (x, y) => { let c = false; for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) { const [xi, yi] = poly[i], [xj, yj] = poly[j]; if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) c = !c; } return c; };
    for (const y of gy) { const zr = [], ur = [], vr = [], mr = []; for (const x of gx) { const m = ins(x, y); const [u, w] = m ? [0, 0] : pol.solver.velocity(solOp, x, y, Math.max(-15, Math.min(25, alphaOp))); zr.push(m ? NaN : Math.min(2.5, Math.hypot(u, w))); ur.push(u); vr.push(w); mr.push(m); } Z.push(zr); Uf.push(ur); Vf.push(vr); mask.push(mr); }
    out.charts.push({ title: `Velocity magnitude |V|/V∞ around the section (α = ${fmt(alphaOp, 3)}°, potential flow)`, type: 'heatmap', x: { label: 'x/c', values: gx }, y: { label: 'y/c', values: gy }, z: Z, mask, cmap: 'turbo', isolines: 10, equal: true, vectors: { u: Uf, v: Vf, stride: 5 }, overlay: [{ pts: poly, closed: true, fill: '#64748b', stroke: '#fff' }], z_label: '|V|/V∞', wide: true, explain: 'Contours and vectors from the panel singularity solution: acceleration over the suction side and stagnation at the leading edge.' });
    out.charts.push({ title: 'Force resolution at the operating point', type: 'bar', categories: ['Lift (⟂ V_a)', 'Drag (∥ V_a)', 'Wing thrust', 'Body drag (long.)', 'Net forward', 'Side force'], y: { label: 'Force', unit: 'N' }, series: [{ name: 'N', values: [Fop.L, Fop.D, Fop.thrust, -Fop.Db * Math.cos(rad(ab)), Fop.net, Fop.sideTot], colors: ['#3b82f6', '#f97316', '#22c55e', '#ef4444', '#10b981', '#a855f7'] }], showValues: true, explain: 'Lift and drag are aerodynamic-axis quantities; thrust and side force are vehicle-axis quantities. Large lift with a head wind still gives negative thrust.' });
    out.tables.push({ title: 'Viscous polar', cols: ['α (°)', 'CL', 'CD', 'CM c/4', 'CL/CD', 'x_tr upper', 'x_sep upper'], rows: A.map((a, i) => [a, pol.CL[i], pol.CD[i], pol.CM[i], pol.CL[i] / pol.CD[i], pol.xtrU[i], pol.xsepU[i] ?? '—']) });
    out.tables.push({ title: 'Force resolution', cols: ['Quantity', 'Value', 'Unit', 'Axis'], rows: [['Apparent wind speed', Va, 'm/s', 'aero'], ['Apparent-wind angle', beta, '°', 'body'], ['Lift', Fop.L, 'N', 'aero (⟂ V_a)'], ['Drag', Fop.D, 'N', 'aero (∥ V_a)'], ['Thrust', Fop.thrust, 'N', 'body x'], ['Side force', Fop.sideTot, 'N', 'body y'], ['Body drag', Fop.Db, 'N', 'aero'], ['Yaw moment', yaw, 'N·m', 'body z'], ['Heeling moment', heel, 'N·m', 'body x']] });
    out.views.push({ label: 'Force diagram', icon: '🧭', render: (el) => el.append(forceDiagram(p, AW, Fop, beta)) });
    out.views.push({ label: '2D animation: live LBM-LES flow (reduced Re)', icon: '🌀', render: (el) => liveLBM(el, p, af, alphaOp, veh) });
    out.views.push({ label: '2D animation: incidence sweep — streamlines & Cp', icon: '🎞', render: (el) => sweep2D(el, { pol, alphaOp, Re, p }) });
    out.views.unshift({ label: '3D flow around the vehicle', icon: '🌬', render: (el) => flow3DView(el, { p, ctx, veh, af, pol, forceAt, Va, beta, AW, ab, alphaOp, delta, rho, nW, span, env, xtr, FySafe, bodyAt }) });
    // recommendations
    if (Fop.net <= Froll) out.recs.push({ level: 'critical', text: `At β = ${fmt(beta, 3)}° the wings cannot overcome rolling resistance. Best available heading gives ${fmt(max(thrOpt), 3)} N near β ≈ ${awa[thrOpt.indexOf(max(thrOpt))]}°; re-plan the route heading or wait for wind.`, basis: 'Thrust–AWA sweep' });
    if (p.incMode === 'fixed' && Math.abs(best.net - Fop.net) > 0.15 * Math.abs(best.net)) out.recs.push({ level: 'warn', text: `A fixed incidence of ${p.incidence}° loses ${fmt(best.net - Fop.net, 3)} N versus optimum (δ = ${fmt(ab - best.alpha, 3)}°). The practical configuration with a mechanically adjustable root incidence recovers it.`, basis: 'Sheeting curve' });
    if (Math.abs(alphaOp) > pol.alphaStall - 2) out.recs.push({ level: 'warn', text: 'Operating close to stall: gusts will push the section into separated flow with sudden lift loss; tune the morphing skin for wash-out (Suite 06) or reduce incidence.', basis: 'Separation model' });
    out.recs.push({ level: 'good', text: `Verify these polars in a wind tunnel across the full AWA envelope (0–180°) at Re ≈ ${fmt(Re, 2)}, with a force balance resolving body-axis thrust and side force — not only lift and drag.`, basis: 'Specification CVV §30 (wind propulsion validation)' });
    if (Re < 3e5) out.recs.push({ level: 'info', text: 'Low Reynolds number: laminar separation bubbles are likely; a boundary-layer trip at 5 % chord (forced transition) usually gives more repeatable performance.', basis: 'Thwaites / Michel analysis' });
    out.explain.push({ title: 'Lift is not thrust', text: `Lift acts perpendicular to the apparent wind and drag parallel to it. Only the projection L·sinβ − D·cosβ onto the travel direction propels the car. At β = ${fmt(beta, 3)}° the ${fmt(Fop.L, 3)} N of lift yields ${fmt(Fop.thrust, 3)} N of thrust and ${fmt(Fop.sideTot, 3)} N of side force that the tyres must resist.` });
    out.explain.push({ title: 'Solver hierarchy', text: 'Full-Reynolds-number section polars come from the coupled panel / integral-boundary-layer method (verified against thin-airfoil theory and Kutta–Joukowski). The live LBM tab adds an unsteady, reduced-Re view of separation and vortex shedding. Higher-fidelity 3-D RANS/LES escalation is reserved for configurations that pass this screening.' });
    out.convergence = { title: 'LBM Poiseuille benchmark profile error', series: [{ name: '|u_num − u_exact| / u_max', x: pois.y, y: pois.num.map((u, i) => Math.max(1e-8, Math.abs(u - pois.exact[i]) / pois.umax)) }] };
    out.outputs = { bodyForces: { awa: bodyAWA, thrust: bodyTh, side: bodySd, hCE_m: hCEb }, endplateFactor: p.endplate, wingState: p.wingState, camberPct: p.camberMorph ? p.camberPct : 0, perfStates: { winds: windsP, deployed: perfD, reefed: perfR }, feathered: !!Fop.feathered, FySafe, tandemPenaltyN: FopNoTandem.net - FopLocked.net, rearSidewashDeg: FopLocked.eps, polar: { alpha: A, CL: pol.CL, CD: pol.CD }, CLmax: pol.CLmax, alphaStall: pol.alphaStall, Re, Va, awa: beta, alpha: alphaOp, incidence: delta, thrustN: Fop.net, wingThrustN: Fop.thrust, liftN: Fop.L, dragN: Fop.D, sideN: Fop.sideTot, yawMoment: yaw, heelMoment: heel, CdA_body: veh.CdA, thrustVsAWA: { awa, thrust: thrOpt, side: sideOpt, incidence: incOpt }, rho, airfoil: af.name };
    out.headline = `At ${p.speed} km/h with ${fmt(p.Uw, 3)} m/s true wind from ${p.dir}°: apparent wind ${fmt(Va, 3)} m/s at ${fmt(beta, 3)}° → lift ${fmt(Fop.L, 3)} N, net forward force ${fmt(Fop.net, 3)} N, side force ${fmt(Fop.sideTot, 3)} N.`;
    out.headlineKind = Fop.net > Froll ? 'ok' : 'warn';
    out.credibility = out.checks.every(c => c.pass) ? 'Numerically verified' : 'Analytical screening';
    return out;
  },
};

function forceDiagram(p, AW, F, beta) {
  // body frame: x forward (screen up), y to the right (screen right)
  const W = 560, H = 440, cx = 280, cy = 230, d = [AW.ax / (AW.Va || 1), AW.ay / (AW.Va || 1)];
  const n = -d[1] >= 0 ? [-d[1], d[0]] : [d[1], -d[0]], sgn = Math.sign(F.c?.CL ?? 1) || 1;
  const Lv = [n[0] * F.L * sgn, n[1] * F.L * sgn], Dv = [d[0] * F.D, d[1] * F.D], R = [Lv[0] + Dv[0], Lv[1] + Dv[1]];
  const sc = 150 / Math.max(Math.abs(F.L), Math.abs(F.D), Math.hypot(...R), 1), S = (v) => [cx + v[1] * sc, cy - v[0] * sc];
  const ar = (v, c, t, from = [0, 0]) => { const [x1, y1] = [cx + from[1] * sc, cy - from[0] * sc], [x2, y2] = S([from[0] + v[0], from[1] + v[1]]); return `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${c}" stroke-width="3" marker-end="url(#fa${c.slice(1)})"/><marker id="fa${c.slice(1)}" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto"><path d="M0,0L10,5L0,10z" fill="${c}"/></marker><text x="${x2 + 6}" y="${y2 - 6}" fill="${c}" font-size="12">${t}</text>`; };
  const aw0 = [-d[0] * 170 / sc, -d[1] * 170 / sc];
  let s = `<svg viewBox="0 0 ${W} ${H}" style="max-width:680px;width:100%;color:var(--fg)">`;
  s += `<rect x="${cx - 24}" y="${cy - 95}" width="48" height="190" rx="24" fill="#334155" stroke="#94a3b8"/><text x="${cx}" y="${cy - 104}" text-anchor="middle" fill="currentColor" font-size="12">bow ↑ direction of travel</text>`;
  s += ar([d[0] * 110 / sc, d[1] * 110 / sc], '#38bdf8', `apparent wind ${fmt(AW.Va, 3)} m/s`, aw0);
  s += ar(Lv, '#3b82f6', `lift ${fmt(F.L, 3)} N`) + ar(Dv, '#f97316', `drag ${fmt(F.D, 3)} N`);
  s += ar([R[0], 0], R[0] >= 0 ? '#22c55e' : '#ef4444', `thrust ${fmt(R[0], 3)} N`) + ar([0, R[1]], '#a855f7', `side ${fmt(R[1], 3)} N`);
  s += `<line x1="${cx}" y1="${cy}" x2="${S(R)[0]}" y2="${S(R)[1]}" stroke="#e5e7eb" stroke-dasharray="4 3"/>`;
  s += `<text x="10" y="${H - 12}" fill="currentColor" font-size="12">Top view in body axes · β = ${fmt(beta, 3)}° · thrust and side force are the projections of the resultant (dashed) of lift + drag.</text></svg>`;
  return h('div', { class: 'card', html: s });
}

function liveLBM(el, p, af, alphaOp, veh) {
  const nx = p.lbmNx, ny = Math.round(nx * 0.4), field = h('select', {}, h('option', { value: 'vort' }, 'Vorticity ωc/U'), h('option', { value: 'speed' }, 'Speed |u|/U'), h('option', { value: 'pres' }, 'Pressure coefficient'));
  const reIn = h('input', { type: 'number', value: p.lbmRe, min: 100, max: 20000, step: 100, style: 'width:90px' }), csIn = h('input', { type: 'number', value: p.lbmCs, min: 0, max: 0.3, step: 0.01, style: 'width:70px' });
  const btn = h('button', { class: 'btn primary sm' }, '▶ Run'), rst = h('button', { class: 'btn sm' }, '↺ Reset');
  const info = h('div', { class: 'small muted' }), cv = h('canvas', { width: nx, height: ny, class: 'lbm-canvas' }), hc = h('canvas', { width: 600, height: 140, class: 'lbm-hist' });
  el.append(callout('warn', h('b', {}, 'Reduced-Reynolds-number 2-D simulation. '), 'This lattice-Boltzmann LES resolves the unsteady wake, separation and vortex shedding at Re ≈ 10²–10⁴ for physical insight. Full-Re design forces come from the verified polar on the Plots tab.'),
    h('div', { class: 'row gap wrap' }, btn, rst, h('label', {}, 'Field ', field), h('label', {}, 'Re ', reIn), h('label', {}, 'Cs ', csIn)), cv, info, hc);
  const u0 = 0.08; let worker = null, running = false, last = null;
  const geom = () => {
    if (p.lbmShape === 'vehicle') { const Lr = nx * 0.35, x0 = nx * 0.25, gap = Lr * veh.wheelRadius / veh.length * 0.6, hgt = Lr * veh.height / veh.length * 0.9; const poly = range(60).map(i => { const t = 2 * Math.PI * i / 60, c = Math.cos(t), s = Math.sin(t), u = (1 - c) / 2; return [x0 + Lr * u, 2 + gap + hgt * 0.5 + (s >= 0 ? hgt * 0.5 * Math.sin(Math.PI * Math.min(1, u * 1.1)) ** 0.6 : -hgt * 0.5 * Math.sin(Math.PI * u) ** 0.3) * Math.abs(s) ** 0.8]; }); return { poly, Lref: Lr, ground: true }; }
    const c = nx * 0.2, a = rad(Math.max(-20, Math.min(25, alphaOp))), x0 = nx * 0.28, y0 = ny * 0.5;
    const poly = af.points.map(([x, y]) => { const xc = x - 0.25; return [x0 + c * (xc * Math.cos(a) + y * Math.sin(a)), y0 + c * (-xc * Math.sin(a) + y * Math.cos(a))]; });
    return { poly, Lref: c, ground: false };
  };
  const start = () => {
    worker?.terminate(); worker = new Worker(new URL('../workers/lbm.js', import.meta.url), { type: 'module' });
    const g = geom(); worker.postMessage({ type: 'init', cfg: { nx, ny, u0, Re: +reIn.value, Cs: +csIn.value, poly: g.poly, Lref: g.Lref, ground: g.ground } });
    worker.onmessage = (e) => {
      const m = e.data; if (!cv.isConnected) { worker.terminate(); return; }
      if (m.type === 'ready') { info.textContent = `τ = ${fmt(m.tau, 4)} · ν_lattice = ${fmt(m.nu, 3)} · Mach ${fmt(m.mach, 3)} · lattice CFL = u₀ = ${u0} · solid cells ${m.solidCells}${m.tau < 0.505 ? ' · ⚠ τ close to 0.5: rely on LES (Cs > 0) for stability' : ''}`; if (running) worker.postMessage({ type: 'run' }); }
      if (m.type === 'frame') { last = m; draw(m); info.textContent = `step ${m.step} · ${fmt(m.mlups, 3)} MLUPS · CL ${fmt(m.CL, 3)} · CD ${fmt(m.CD, 3)} · residual ${fmt(m.residual, 2)} · Re ${reIn.value} · Cs ${csIn.value}`; drawHist(m.hist); }
    };
  };
  const draw = (m) => {
    const ctx2 = cv.getContext('2d'), img = ctx2.createImageData(nx, ny), F = m[field.value]; let lo, hi;
    if (field.value === 'vort') { lo = -8; hi = 8; } else if (field.value === 'speed') { lo = 0; hi = 1.8; } else { lo = -2.5; hi = 1; }
    for (let y = 0; y < ny; y++) for (let x = 0; x < nx; x++) { const i = y * nx + x, k = ((ny - 1 - y) * nx + x) * 4, v = F[i]; if (!Number.isFinite(v)) { img.data[k] = 90; img.data[k + 1] = 100; img.data[k + 2] = 115; img.data[k + 3] = 255; continue; } const c = cmap((v - lo) / (hi - lo), field.value === 'vort' ? 'coolwarm' : 'turbo'); img.data[k] = c[0]; img.data[k + 1] = c[1]; img.data[k + 2] = c[2]; img.data[k + 3] = 255; }
    ctx2.putImageData(img, 0, 0);
  };
  const drawHist = (H) => {
    const c = hc.getContext('2d'), W = hc.width, Hh = hc.height; c.clearRect(0, 0, W, Hh); if (H.t.length < 2) return;
    const all = H.cl.concat(H.cd).filter(Number.isFinite), lo = Math.min(...all), hi = Math.max(...all), sy = (v) => Hh - 14 - (v - lo) / ((hi - lo) || 1) * (Hh - 24);
    for (const [arr, col, nm] of [[H.cl, '#3b82f6', 'CL'], [H.cd, '#f97316', 'CD']]) { c.strokeStyle = col; c.lineWidth = 1.5; c.beginPath(); arr.forEach((v, i) => { const x = i / (arr.length - 1) * W; i ? c.lineTo(x, sy(v)) : c.moveTo(x, sy(v)); }); c.stroke(); c.fillStyle = col; c.fillText(`${nm} ${fmt(arr[arr.length - 1], 3)} (mean last half ${fmt(mean(arr.slice(arr.length >> 1)), 3)})`, nm === 'CL' ? 6 : 260, 12); }
  };
  btn.onclick = () => { running = !running; btn.textContent = running ? '⏸ Pause' : '▶ Run'; if (!worker) start(); worker.postMessage({ type: running ? 'run' : 'pause' }); };
  const go = () => { start(); if (running) worker.postMessage({ type: 'run' }); };
  rst.onclick = go;
  field.onchange = () => last && draw(last);
  csIn.onchange = () => worker?.postMessage({ type: 'set', Cs: +csIn.value });
  reIn.onchange = go;
  // start automatically so the field evolves as soon as the tab opens; pause when scrolled out of view
  running = true; btn.textContent = '⏸ Pause'; go();
  new IntersectionObserver((en) => { if (!worker) return; if (!en[0].isIntersecting) worker.postMessage({ type: 'pause' }); else if (running) worker.postMessage({ type: 'run' }); }).observe(cv);
  void min;
}

// ---------------- 3-D flow simulation around the complete vehicle ----------------
// Solved: section polar & Cp (panel + boundary layer), finite-wing CL/CD, optimum incidence and the lift/thrust/side
// forces at every apparent-wind angle. Modelled for display: the 3-D velocity field from a lifting-line horseshoe
// system carrying exactly the solved lift, a doublet-line body and ground images, sampled on a grid.
async function flow3DView(el, o) {
  const { p, ctx, veh, af, pol, forceAt, Va, beta, AW, ab, alphaOp, delta, rho, env } = o;
  const [{ vehicleScene, playback, particles, streamlines, hud, legend3d }, mats] = await Promise.all([import('../core/sim3d.js'), import('../core/materials.js')]);
  const ctl = h('div', { class: 'row gap wrap', style: 'margin-bottom:8px' });
  const modeSel = h('select', {}, h('option', { value: 'sweep' }, 'Animate: apparent-wind-angle sweep (optimum incidence)'), h('option', { value: 'op' }, 'Steady operating point'));
  const showStream = h('input', { type: 'checkbox', checked: true }), showPart = h('input', { type: 'checkbox', checked: true });
  ctl.append(h('label', {}, 'Mode ', modeSel), h('label', {}, showPart, ' particles'), h('label', {}, showStream, ' streamlines'));
  el.append(h('p', { class: 'small muted' }, 'Vehicle-fixed frame. Solved quantities: section polar & Cp, finite-wing coefficients, optimum incidence and the resulting lift, thrust and side force at each apparent-wind angle. Displayed 3-D field: boundary-layer freestream + lifting-line horseshoe vortices carrying exactly the solved lift (half-elliptic loading, ground images) + doublet-line monocoque — a visual approximation of the flow, not a 3-D CFD solution. Wing colour = section Cp (inviscid panel solution) scaled by the spanwise loading.'), ctl);
  const box = h('div'); el.append(box);
  const materials = Object.fromEntries(Object.keys(mats.COMPONENTS).map(k => [k, ctx.material(k)]));
  const S = await vehicleScene(box, { vehicle: veh, materials, wingIncidence: delta, airfoil: af, follow: false, height: 480 });
  const { V, THREE } = S; const H = hud(box);
  const leg = legend3d(box, { title: 'Air speed |V| (m/s)', lo: 0.7 * Va, hi: 1.3 * Va });
  // side of the apparent wind in the scene frame (x forward, y left; physics fy is to the right)
  const side = (-AW.ay) >= 0 ? 1 : -1, z0 = env.z0 ?? 0.1;
  const wingInfo = S.wingObjs.map((w, k) => { const B = w.base; let zmin = Infinity, zmax = -Infinity; for (let i = 2; i < B.length; i += 3) { zmin = Math.min(zmin, B[i]); zmax = Math.max(zmax, B[i]); } return { x: S.asm.meta.wingX[k], z0: zmin, z1: zmax }; });
  const zCE = mean(wingInfo.map(w => 0.5 * (w.z0 + w.z1)));
  const prof = (z) => Math.log(Math.max(z, 2 * z0) / z0) / Math.log(Math.max(zCE, 2 * z0) / z0);
  // sweep table from the solved force model (constant apparent-wind speed)
  const betas = range(141).map(i => 20 + i), sw = betas.map(b => { let bb = { net: -Infinity }; for (const a of linspace(-5, 35, 81)) { const F = forceAt(b, a); if ((p.breakaway === false || F.sideTot <= o.FySafe) && F.net > bb.net) bb = { ...F, alpha: a }; } if (!Number.isFinite(bb.net)) { const F = forceAt(b, 0); bb = { ...F, alpha: 0 }; } return bb; });
  const opF = forceAt(ab, alphaOp);
  const state = (t) => { if (modeSel.value === 'op') return { b: ab, alpha: alphaOp, d: delta, F: opF }; const T = 28, u = (t % T) / T, b = 20 + 140 * (u < 0.5 ? 2 * u : 2 - 2 * u), i = Math.min(139, Math.floor(b - 20)), f = b - 20 - i, A = sw[i], B = sw[i + 1]; const mix = (k) => A[k] + f * (B[k] - A[k]); return { b, alpha: mix('alpha'), d: b - mix('alpha'), F: { L: mix('L'), D: mix('D'), thrust: mix('thrust'), net: mix('net'), sideTot: mix('sideTot'), Db: mix('Db') } }; };
  const bounds = [[-9, 9], [-7, 7], [0.02, 7]];
  let field = null, fieldKey = null, streamG = null, P = null, cpLo = -2.5, cpHi = 1;
  const build = (st) => {
    const br = rad(st.b), airAt = (z) => { const f = prof(z), tw = [(-Va * Math.cos(br) + 0) , side * Va * Math.sin(br)]; // relative air at CE height
      // true wind = relative + vehicle velocity; shear acts on the true wind only
      const v = p.speed / 3.6, wx = tw[0] + v, wy = tw[1]; return [wx * f - v, wy * f]; };
    const Bst = o.bodyAt ? o.bodyAt(st.b) : { L: 0 }, bodyLL = Bst.L ? [{ x: -0.42 * veh.length + 0.25 * veh.length, z0: veh.wheelRadius + 0.02, z1: veh.wheelRadius + veh.height, L: Bst.L }] : [];
    const M = vehicleFlowModel({ air: airAt, wings: [...wingInfo.map(w => ({ ...w, L: st.F.L / wingInfo.length })), ...bodyLL], body: bodyStations(veh), rho, Va });
    field = sampledField(M.vel, bounds, [31, 25, 13]);
    if (streamG) { V.root.remove(streamG); streamG.traverse(c => c.geometry?.dispose()); streamG = null; }
    if (showStream.checked) { const ux = airAt(zCE), sp = Math.hypot(...ux), dx = ux[0] / sp, dy = ux[1] / sp, seeds = []; for (const zz of [0.6, 1.4, 2.2, 3.0, 3.8, 4.6]) for (const off of linspace(-3, 3, 9)) seeds.push([-dx * 8 - dy * off, -dy * 8 + dx * off, zz]); streamG = streamlines(V, seeds, (x, y, z) => field.sample(x, y, z), { steps: 360, ds: 0.06, colorRange: [0.7 * Va, 1.3 * Va] }); }
    // wing colouring: section Cp at the current α, scaled by the half-elliptic spanwise loading
    const sol = pol.solver.solve(clamp(st.alpha, -15, 25)), pan = sol.panels, cp = Array.from(sol.Cp), np = af.points.length, cpOf = af.points.map(([x, y]) => { let bi = 0, bd = Infinity; for (let i = 0; i < pan.length; i++) { const d = (pan[i].xm - x) ** 2 + (pan[i].ym - y) ** 2; if (d < bd) { bd = d; bi = i; } } return cp[bi]; });
    S.wingObjs.forEach((w, k) => { const wi = wingInfo[k], g = w.o.geometry, n = g.attributes.position.count, col = new Float32Array(n * 3), Pz = w.base; for (let i = 0; i < n; i++) { const j = i % np, eta = Math.min(1, Math.max(0, (Pz[i * 3 + 2] - wi.z0) / (wi.z1 - wi.z0))), c = i < n - 2 ? cpOf[j] * (4 / Math.PI) * Math.sqrt(Math.max(0, 1 - eta * eta)) * Math.sign(st.F.L || 1) : 0, rgb = cmap((c - cpLo) / (cpHi - cpLo), 'coolwarm'); col.set([rgb[0] / 255, rgb[1] / 255, rgb[2] / 255], i * 3); } g.setAttribute('color', new THREE.BufferAttribute(col, 3)); w.o.material.vertexColors = true; w.o.material.color.set(0xffffff); w.o.material.needsUpdate = true; });
    leg.set(0.7 * Va, 1.3 * Va);
    return M;
  };
  // wing incidence: rotate each wing about its quarter-chord axis relative to the as-built incidence δ
  const setIncidence = (d) => { S.wingObjs.forEach((w, k) => { const P2 = w.o.geometry.attributes.position.array, B = w.base, a = rad(d - delta), c = Math.cos(a), sn = Math.sin(a), xc = wingInfo[k].x; for (let i = 0; i < B.length; i += 3) { const dx = B[i] - xc, dy = B[i + 1]; P2[i] = xc + c * dx - sn * dy; P2[i + 1] = sn * dx + c * dy; P2[i + 2] = B[i + 2]; } w.o.geometry.attributes.position.needsUpdate = true; w.o.geometry.computeVertexNormals(); }); };
  const st0 = state(0); build(st0); fieldKey = Math.round(st0.b / 2);
  P = particles(V, { count: matchMedia('(max-width:600px)').matches ? 1200 : 2600, bounds, vel: (x, y, z) => field.sample(x, y, z), colorRange: [0.7 * Va, 1.3 * Va], size: 0.07 });
  let lastT = 0; const zR = S.asm.meta.height - S.asm.meta.span * 0.45;
  const pb = playback(box, { duration: 28, label: 'Animation time', onFrame: (t) => {
    const st = state(t), key = modeSel.value === 'op' ? 'op' : Math.round(st.b / 2);
    if (key !== fieldKey) { fieldKey = key; build(st); }
    setIncidence(st.d);
    const dt = Math.min(0.05, Math.max(0, t - lastT)) || 0.016; lastT = t; P.pts.visible = showPart.checked; if (showPart.checked) P.step(dt, t);
    const br = rad(st.b), ax = -Math.cos(br), ay = side * Math.sin(br);
    S.setArrow('thrust', [st.F.net, 0, 0], 0.01); S.setArrow('side', [0, side * st.F.sideTot, 0], 0.004); // lateral force acts downwind of the apparent wind (L·cosβ + D·sinβ) S.setArrow('drag', [ax * st.F.Db, ay * st.F.Db, 0], 0.02); S.setArrow('wind', [ax * Va, ay * Va, 0], 0.25); S.setArrow('hub', [0, 0, 0]);
    H.set([['Apparent-wind angle β', fmt(st.b, 3) + '°'], ['Incidence δ (solved opt.)', fmt(st.d, 3) + '°'], ['Angle of attack α', fmt(st.alpha, 3) + '°'], ['Lift (solved)', fmt(st.F.L, 3) + ' N'], ['Net forward force', fmt(st.F.net, 3) + ' N', st.F.net > 0 ? '#22c55e' : '#ef4444'], ['Side force', fmt(st.F.sideTot, 3) + ' N', '#ef4444'], ['Apparent wind', fmt(Va, 3) + ' m/s']]);
  } });
  modeSel.onchange = () => { fieldKey = null; }; showStream.onchange = () => { fieldKey = null; };
  void zR; void beta;
  return pb;
}

// ---------------- 2-D animation: incidence sweep with streamlines and surface pressure ----------------
// Solved at every frame: panel-method solution (singularity strengths, surface Cp) at α(t); streamlines integrated
// through the induced velocity field of that solution; boundary-layer CL/CD read from the viscous polar.
async function sweep2D(el, o) {
  const { pol, alphaOp, Re } = o;
  const { anim2d, axes2d, polyline, rgb } = await import('../core/sim3d.js');
  el.append(h('p', { class: 'small muted' }, `Angle of attack sweeps −4° → ${fmt(pol.alphaStall + 4, 3)}° and back. Streamlines are integrated through the solved panel-method velocity field at each α; surface colour/arrows = solved Cp (suction blue, pressure red). The polar trace (right) follows the viscous CL(α) at Re = ${fmt(Re, 3)}; beyond the solved stall angle the inviscid streamlines no longer represent the separated flow (see the LBM tab).`));
  const aMin = -4, aMax = Math.min(25, pol.alphaStall + 4), T = 16, cache = new Map();
  const solveAt = (a) => { const key = Math.round(a * 4) / 4; if (cache.has(key)) return cache.get(key); const sol = pol.solver.solve(key), poly = sol.panels.map(q => [q.x1, q.y1]); const lines = []; for (const y0 of linspace(-0.5, 0.5, 15)) { const pts = []; let x = -0.7, y = y0; for (let i = 0; i < 260; i++) { const [u, w] = pol.solver.velocity(sol, x, y, key), sp = Math.hypot(u, w) || 1; pts.push([x, y, sp]); x += 0.012 * u / sp; y += 0.012 * w / sp; if (x > 1.8) break; } lines.push(pts); } const r = { sol, poly, lines }; cache.set(key, r); if (cache.size > 200) cache.delete(cache.keys().next().value); return r; };
  const { cmap } = await import('../core/chart.js');
  const pb = anim2d(el, { duration: T, label: 'Sweep time', height: 440, draw: (g, t, W, H, c) => {
    const u = (t % T) / T, a = aMin + (aMax - aMin) * (u < 0.5 ? 2 * u : 2 - 2 * u), R = solveAt(a);
    const fw = Math.min(W * 0.62, W - 260), sc = fw / 2.6, ox = 0.7 * sc + 10, oy = H / 2, X = (x) => ox + x * sc, Y = (y) => oy - y * sc;
    for (const L of R.lines) { for (let i = 1; i < L.length; i++) { g.strokeStyle = rgb(cmap((L[i][2] - 0.4) / 1.4, 'turbo')); g.lineWidth = 1.3; g.beginPath(); g.moveTo(X(L[i - 1][0]), Y(L[i - 1][1])); g.lineTo(X(L[i][0]), Y(L[i][1])); g.stroke(); } }
    // animated tracer dots along the streamlines (speed-proportional phase)
    const ph = (t * 0.6) % 1; g.fillStyle = '#fff'; for (const L of R.lines) for (let k = 0; k < L.length; k += 18) { const i = Math.min(L.length - 1, k + Math.floor(ph * 18)); g.beginPath(); g.arc(X(L[i][0]), Y(L[i][1]), 1.8, 0, 7); g.fill(); }
    g.fillStyle = '#475569'; g.beginPath(); R.poly.forEach(([x, y], i) => i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y))); g.closePath(); g.fill();
    // Cp arrows normal to the surface
    R.sol.panels.forEach((q, i) => { const cp = R.sol.Cp[i], len = Math.max(-3, Math.min(1, cp)) * 0.08 * sc, col = cp < 0 ? '#3b82f6' : '#ef4444'; g.strokeStyle = col; g.lineWidth = 1.5; g.beginPath(); g.moveTo(X(q.xm), Y(q.ym)); g.lineTo(X(q.xm) + q.nx * len, Y(q.ym) - q.ny * len); g.stroke(); });
    g.fillStyle = c.fg; g.font = '13px system-ui'; const cv = coeffs(pol, a); g.fillText(`α = ${fmt(a, 3)}°   CL(viscous) = ${fmt(cv.CL, 3)}   CD = ${fmt(cv.CD, 3)}   ${Math.abs(a) > pol.alphaStall ? 'STALLED (separated)' : ''}`, 10, 20);
    // polar trace
    const A = axes2d(g, { x0: W - 230, y0: 40, w: 200, h: H - 100, xr: [aMin, aMax], yr: [Math.min(0, ...pol.CL), Math.max(...pol.CL) * 1.1], xl: 'α (°)', yl: 'CL', fg: c.fg, mute: c.mute });
    polyline(g, pol.alpha.map((aa, i) => [A.X(aa), A.Y(pol.CL[i])]), '#22c55e', 2);
    g.fillStyle = '#facc15'; g.beginPath(); g.arc(A.X(a), A.Y(cv.CL), 5, 0, 7); g.fill();
    g.strokeStyle = '#a855f7'; g.setLineDash([4, 3]); g.beginPath(); g.moveTo(A.X(alphaOp), 40); g.lineTo(A.X(alphaOp), H - 60); g.stroke(); g.setLineDash([]);
  } });
  return pb;
}
