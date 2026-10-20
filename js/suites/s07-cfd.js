// Suite 07 — Aerodynamics and computational fluid dynamics.
import { naca4, cst, vehicleAssembly } from '../core/cad.js';
import { panelSolver, viscous, polar, coeffs, finiteWing } from '../core/aero.js';
import { resampleAirfoil } from '../core/mesh.js';
import { poiseuille } from '../core/lbm-core.js';
import { apparentWind, wingForces, moistAir } from '../core/physics.js';
import { linspace, range, fmt, gci, deg, rad, max, min, mean, G0, clamp } from '../core/num.js';
import { h, table, callout } from '../core/ui.js';
import { cmap } from '../core/chart.js';
import { vehicleFlowModel, sampledField, bodyStations } from '../core/flow3d.js';
import { flexComputeAsync } from '../core/s07flex.js';
import { safeSideForce } from '../core/safeside.js';
import * as AA from '../core/archaero.js';

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

const S07_MEMO = new Map();
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
    { key: 'flexK', label: 'Pylon flexure net yaw stiffness k_θ', unit: 'N·m/rad', type: 'number', min: 0.1, max: 1000, step: 0.1, default: 60, group: 'Compliant flexure root (Rev 5)', help: 'Single centreline titanium pylon flexure under the root crossbeam (both legs stand on one crossbeam, so the closed arch yaws about ONE axis; twin leg flexures would lock kinematically). Net ≥ 5 N·m/rad (60 by default: in light wind the flexure holds the arch nearer the body axis, raising α and C_L where it helps, while at higher q the aerodynamic moment dominates and the arch trims to the lower C_L,trim; above ≈ 75 N·m/rad the 2 m/s speed collapses (stall); the statically balanced rod-cage pylon of Suite 09 holds ±4 N·m/rad after trim); the aerodynamic yaw stiffness (≈ 30 N·m/rad at 4 m/s) still dominates, so the arch self-trimming wingsail); the flexure stop limits ±25°.' },
    { key: 'flexAxis', label: 'Pylon yaw-axis chordwise position (on the centreline)', unit: 'x/c', type: 'range', min: 0.03, max: 0.24, step: 0.005, default: 0.135, group: 'Compliant flexure root (Rev 5)', help: 'Ahead of the aerodynamic centre (0.25 c) for static stability.' },
    { key: 'flexCm0', label: 'Reflexed bistable skin C_m,ac', type: 'number', min: 0, max: 0.2, step: 0.005, default: 0.05, group: 'Compliant flexure root (Rev 5)', help: 'Positive (nose-up) moment of the reflexed skin; trim C_L = C_m0 / (0.25 − x_axis).' },
    { key: 'flexMax', label: 'Pylon elastic yaw limit (progressive stop)', unit: '°', type: 'number', min: 5, max: 45, step: 1, default: 25, auto: (c) => (c.vehicle.yawLimit != null ? { value: c.vehicle.yawLimit, prov: 'coupled' } : undefined), group: 'Compliant flexure root (Rev 5)' },
    { key: 'incMode', label: 'Wing incidence', type: 'select', default: 'optimum', options: [{ value: 'optimum', label: 'Optimum for maximum thrust' }, { value: 'fixed', label: 'Fixed (strict fixed-root configuration)' }], group: 'Wing' },
    { key: 'incidence', label: 'Fixed incidence (chord to bow)', unit: '°', type: 'range', min: -30, max: 90, step: 1, default: 20, group: 'Wing' },
    { key: 'airfoilSrc', label: 'Airfoil source', type: 'select', default: 'naca', options: [{ value: 'naca', label: 'NACA 4-digit' }, { value: 'suite02', label: 'From Suite 02 (CAD)' }, { value: 'imported', label: 'Imported (.dat via Data portal / Import)' }], group: 'Wing' },
    { key: 'naca', label: 'NACA code (unloaded base section)', type: 'text', default: '0012', group: 'Wing', help: 'Specification starting section was NACA 0018; Revision 5.1 uses NACA 0012 legs (lower profile drag at Re ≈ 3–5·10⁵ raises the light-wind speed; the bridge keeps a thicker section for the roll-over tube). Symmetric when unloaded.' },
    { key: 'camberMorph', label: 'Passive leeward camber morphing', type: 'bool', default: true, group: 'Wing', help: 'Compliant trailing region cambers to leeward on either tack (Revision 2.2); the panel/boundary-layer solver then analyses the cambered section.' },
    { key: 'camberPct', label: 'Morphed camber', unit: '% chord', type: 'range', min: 0, max: 9, step: 1, default: 7, group: 'Wing' },
    { key: 'bodyLift', label: 'Body acts as a lifting surface', type: 'bool', default: true, group: 'Body aerodynamics', help: 'The monocoque plan-form (sharp tail, Kutta condition) is a low-aspect-ratio lifting body with ground reflection; its side force and forward thrust add to the wings.' },
    { key: 'bodyCamberPct', label: 'Passive side-panel camber (leeward)', unit: '% chord', type: 'range', min: 0, max: 8, step: 0.5, default: 4, group: 'Body aerodynamics', help: 'Metamaterial side panels camber to the leeward side under load on either tack.' },
    { key: 'bodyFill', label: 'Side-area fill factor', type: 'range', min: 0.5, max: 1, step: 0.01, default: 0.78, group: 'Body aerodynamics', expert: true },
    { key: 'dragModel', label: 'Body drag area', type: 'select', group: 'Body aerodynamics', default: 'buildup', options: [{ value: 'buildup', label: 'Geometry build-up (Hoerner body + components, same for every revision)' }, { value: 'input', label: 'Fixed C_dA input (vehicle)' }] },
    { key: 'stackLL', label: 'Stacked lifting line: ground image, body as the lower lifting surface', type: 'bool', default: true, group: 'Body aerodynamics', help: 'Applied identically to every revision: the vertical lifting system (ground gap or skirt → body side panel → root junction → wing) is solved as one lifting line with the ground as an image plane; gaps shed trailing vortices.' },
    { key: 'deckSeal', label: 'Root-junction seal σ (brush/elastomer skirt between crossbeam and deck)', type: 'range', min: 0, max: 1, step: 0.05, group: 'Body aerodynamics', default: 0, auto: (c) => ((c.vehicle.wingConfig || 'arch') === 'arch' && (c.vehicle.archRoot || 'flexure') === 'flexure' ? { value: c.vehicle.deckSeal ?? 0, prov: 'coupled' } : { value: c.vehicle.deckSeal ?? 0, prov: 'coupled' }), help: 'Fraction of the root chord that carries load across the 20–30 mm crossbeam gap. Rev 5.1: dense PBT/stainless brush on a yawing root plate wiping the deck — brush seals leak far less than labyrinths or open clearances (Chupp et al., J. Propulsion & Power 22(2), 2006), but bristle-pack porosity and tip clearance remain, so the conservative lower bound 0.5 is used (half the gap treated as open). 0 = open gap.' },
    { key: 'groundSeal', label: 'Underbody side-skirt seal σ (ground as reflection plane)', type: 'range', min: 0, max: 1, step: 0.05, group: 'Body aerodynamics', default: 0, auto: (c) => ({ value: c.vehicle.groundSeal ?? ((c.vehicle.wingConfig || 'arch') === 'arch' && (c.vehicle.archRoot || 'flexure') === 'flexure' ? 0.6 : 0), prov: 'coupled' }), help: 'Side skirts down to ≈ 20 mm above the road: fraction of the body length that carries load across the ground clearance. 0 = open clearance.' },
    { key: 'archFill', label: 'Arch opening (front view)', type: 'select', group: 'Wing', default: 'hollow', auto: (c) => ({ value: c.vehicle.archFill || 'hollow', prov: 'coupled' }), options: [{ value: 'hollow', label: 'Hollow (two legs + bridge)' }, { value: 'solid', label: 'Solid (continuous centreline web, crossbeam → bridge)' }, { value: 'hybrid', label: 'Hybrid (root gusset web over the lower 30 % + corner fairings)' }] },
    { key: 'fillStudy', label: 'Compare hollow / solid / hybrid arch openings', type: 'bool', group: 'Wing', default: true },
    { key: 'rootPlateW', label: 'Root endplate width (lateral, yaws with the arch)', unit: 'm', type: 'number', min: 0, max: 2.0, step: 0.05, group: 'Body aerodynamics', default: 0, auto: (c) => ({ value: c.vehicle.rootPlateW ?? 0, prov: 'coupled' }), help: 'Flat plate under the leg roots. Credit from the end-plate correlation ΔAR/AR ≈ 1.9 h/b (Hoerner, Fluid-Dynamic Lift, 1985), with h = half the plate width and b = twice the leg height, scaled by plate length / root chord (≤ 1).' },
    { key: 'rootPlateL', label: 'Root endplate length (fore–aft)', unit: 'm', type: 'number', min: 0, max: 2.5, step: 0.05, group: 'Body aerodynamics', default: 0, auto: (c) => ({ value: c.vehicle.rootPlateL ?? 0, prov: 'coupled' }) },
    { key: 'rootBootH', label: 'Root boot: continuous elastomer membrane height (deck → leg roots)', unit: 'm', type: 'number', min: 0, max: 0.8, step: 0.01, group: 'Body aerodynamics', default: 0, auto: (c) => ({ value: c.vehicle.rootBootH ?? 0, prov: 'coupled' }), help: '0 = no boot (the deck seal σ applies). With a boot there is no open gap: the membrane carries the pressure difference (lift slope × 0.7 for bulging) and the crossbeam is raised by the boot height.' },
    { key: 'skirtGap', label: 'Underbody skirt: remaining gap to the road', unit: 'm', type: 'number', min: 0, max: 0.3, step: 0.005, group: 'Body aerodynamics', default: 0, auto: (c) => ({ value: c.vehicle.skirtGap ?? 0, prov: 'coupled' }), help: '0 = no skirt (the ground seal σ applies). Compliant skirt with a wear strip; only this gap to the road remains open.' },
    { key: 'bodyEta', label: 'Body side-panel lift-slope factor η_b (× 2π)', type: 'range', min: 0.2, max: 1, step: 0.05, default: 0.6, group: 'Body aerodynamics', expert: true, help: 'Thick, rounded-edge body with a sharp tail: 2-D lift slope taken as η_b·2π (thin-airfoil 2π reduced for thickness-edge separation and the bluff base); the body section stalls at C_l ≈ 0.6.' },
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
    // memoised: identical inputs (parameters, vehicle, upstream outputs it reads) return the previous result instantly
    let memoKey = null; try { memoKey = JSON.stringify([p, ctx.vehicle, ...['s01', 's02', 's05', 's10', 's11', 's14'].map(id => ctx.upstream(id) ?? null), ctx.store?.project?.imports?.airfoil ?? null]); } catch { memoKey = null; }
    if (memoKey && S07_MEMO.has(memoKey)) return S07_MEMO.get(memoKey);
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    // measured seal effectiveness (Suite 24 'Seal/endplate validation' import: columns awa, CL_brush, CL_nobrush): the lift-slope
    // ratio with/without the brush is matched to the model ratio between σ = 1 and σ = 0 → σ_measured overrides the assumed value
    const sealMeas = (() => { const sets = [...(ctx.store?.project?.experiments?.s24 || []), ...(ctx.experiments || [])].filter(d => d.columns && d.header?.some(h => /cl_?brush/i.test(h)) && d.header?.some(h => /cl_?no_?brush/i.test(h)));
      if (!sets.length) return null; const d = sets[0], H = (re) => d.header.find(h => re.test(h)), A = d.columns[H(/^awa|beta/i)] || [], Cb = d.columns[H(/cl_?brush/i)], Cn = d.columns[H(/cl_?no_?brush/i)];
      const slope = (x, y) => { const pts = x.map((v, i) => [v, y[i]]).filter(([u, w]) => Number.isFinite(u) && Number.isFinite(w)); if (pts.length < 2) return NaN; const mx = mean(pts.map(q => q[0])), my = mean(pts.map(q => q[1])); return pts.reduce((a, q) => a + (q[0] - mx) * (q[1] - my), 0) / Math.max(1e-12, pts.reduce((a, q) => a + (q[0] - mx) ** 2, 0)); };
      const R = slope(A, Cb) / slope(A, Cn); return Number.isFinite(R) && R > 0 ? { R, name: d.name, n: A.length } : null; })();
    // ONE drag build-up (js/core/dragbuild.js), shared with Suite 01: root plate and thermal-store fins included there
    const o14 = ctx.upstream('s14'), finCdA = o14?.finDragCdA_m2 > 0 ? o14.finDragCdA_m2 : o14?.finDrag_W > 0 ? o14.finDrag_W / (0.5 * 1.225 * (20 / 3.6) ** 3) : 0;
    const DBU = AA.dragBuildup({ ...ctx.vehicle, rootPlateW: p.rootPlateW || 0, rootPlateL: p.rootPlateL || 0 }, { finCdA });
    const veh = { ...ctx.vehicle, CdA: p.dragModel === 'input' ? ctx.vehicle.CdA : DBU.CdA, rootPlateW: p.rootPlateW || 0, rootPlateL: p.rootPlateL || 0, archFill: p.archFill || ctx.vehicle.archFill || 'hollow', stackLL: p.stackLL !== false, deckSeal: p.deckSeal ?? 0, groundSeal: p.groundSeal ?? 0, bodyEta: p.bodyEta ?? 0.6, sealMeasured: sealMeas, rootBootH: p.rootBootH || 0, skirtGap: p.skirtGap > 0 ? p.skirtGap : null, bodyCamberPct: p.bodyCamberPct }, env = ctx.upstream('s05') || ctx.env || {};
    const air = env.rho ? { rho: env.rho, nu: env.nu || 1.5e-5 } : moistAir(101325, 20, 60);
    const rho = air.rho, nu = air.nu;
    const v = p.speed / 3.6, AW = apparentWind(p.Uw, p.dir, p.heading, v), Va = Math.max(AW.Va, 1e-3), beta = AW.beta, ab = Math.abs(beta);
    const w2 = ctx.upstream('s02')?.wing, nW = veh.nWings || 2, spanDep = w2?.span || veh.wingHeight, reefSpan = Math.min(spanDep, veh.wingReefSpan ?? spanDep);
    const span = p.wingState === 'reefed' ? reefSpan : spanDep, S = (w2?.area || veh.wingArea) * span / spanDep, Sw = S / nW, cmean = Sw / span;
    // Revision 4 AeroArch: one closed box wing (two canted, swept legs + top bridge) on a ring track. Mirrored about the
    // body/ground plane the legs form a Prandtl box wing of span b = 2·leg length and mean gap g (root gap ↔ bridge span):
    // induced-drag factor k = (1 + 0.45 g/b)/(1.04 + 2.81 g/b) → AR_eff = (b²/S_mirrored)/k; lift slope × cos Λ.
    const arch = veh.wingConfig === 'arch', VM = arch ? vehicleAssembly(veh, {}).meta : null, AM = VM?.arch;
    const legLen = arch ? AM.legHeight / Math.cos(rad(AM.cant)) : span, gMean = arch ? (AM.gap + AM.bridgeSpan) / 2 : 0, bBox = 2 * legLen, kBox = arch ? (1 + 0.45 * gMean / bBox) / (1.04 + 2.81 * gMean / bBox) : 1;
    const cosL = arch ? Math.cos(rad(AM.sweep)) : 1, AR = arch ? bBox * bBox / (2 * S) : span * span / Sw, ARe = arch ? AR / kBox : AR * p.endplate, Re = Math.max(2e4, Va * cmean / nu);
    const af = resolveAirfoil(p, ctx), pts = panelPts(af.points, p.panels);
    ctx.progress(0.1, 'polar');
    const xtr = p.transition === 'forced' ? p.xtr : 1;
    const pol = polar(pts, Re, { alphas: range(p.alphaMax + 7).map(i => i - 6), xtrU: xtr, xtrL: xtr });
    await ctx.yield();
    const C3 = (a, P = pol) => { const c = coeffs(P, a); if (Math.abs(a) > P.alphaStall + 8) return { ...c, CDi: 0 }; const f = finiteWing(c.CL * cosL, c.CD, ARe, p.oswald); return { CL: f.CL, CD: f.CD, CDi: f.CDi, regime: c.regime }; };
    // Body as a lifting surface: plan-form airfoil of chord L (sharp tail → Kutta), height h with ground reflection,
    // AR_eff = 2h²/S_side; low-AR lift CL = (πAR/2)(α + 2m) + 2 sin²α cosα (attached + cross-flow), induced drag CL²/(πAR e).
    // Zero-lift drag is already in the vehicle C_dA term, so only lift and induced drag are added here.
    const Lb = veh.length, hb = veh.height, Sb = p.bodyFill * Lb * hb, ARb = 2 * hb * hb / Sb, mB = (p.bodyCamberPct || 0) / 100, hCEb = veh.wheelRadius + 0.5 * hb;
    const bodyAt = (bAbs, VA = Va) => {
      if (!p.bodyLift || bAbs < 0.5) return { L: 0, Di: 0, thrust: 0, side: 0, CL: 0, xcp: 0 };
      const wS = p.stackLL !== false ? clamp((45 - bAbs) / 10, 0, 1) : 0; if (wS >= 1) return { L: 0, Di: 0, thrust: 0, side: 0, CL: 0, xcp: 0.08 * veh.length, inStack: true }; // inside the stacked lifting line
      const a = rad(Math.min(bAbs, 180 - bAbs)), sgn = bAbs > 90 ? -1 : 1; // the body axis is fixed; beyond 90° the tail leads (no camber benefit)
      const CLp = (Math.PI * ARb / 2) * (a + (bAbs <= 90 ? 2 * mB : 0)) * Math.cos(a) ** 2, CLc = 2 * Math.sin(a) ** 2 * Math.cos(a), CL = CLp + CLc, q = 0.5 * rho * VA * VA, L = q * Sb * CL, Di = q * Sb * CL * CL / (Math.PI * ARb * p.bodyE);
      // centre of pressure: potential (low-AR) lift near the leading quarter, cross-flow lift at the plan-area centroid
      const xLE = bAbs <= 90 ? 0.58 * veh.length : -0.42 * veh.length, dir = bAbs <= 90 ? -1 : 1, xcp = (CLp * (xLE + dir * 0.25 * veh.length) + CLc * (0.08 * veh.length)) / Math.max(1e-9, CL);
      const b = rad(bAbs), f = 1 - wS; void sgn; return { L: f * L, Di: f * Di, CL: f * CL, xcp, thrust: f * (L * Math.sin(b) - Di * Math.cos(b)), side: f * (L * Math.abs(Math.cos(b)) + Di * Math.sin(b)) };
    };
    // Tandem interference: the downstream wing sits in the front wing's induced sidewash ε ≈ 2C_L/(πAR_eff), weighted by
    // how much it is actually downstream (|cos β|) and decaying with the cross-flow offset gap·|sin β| (in chords).
    const cr = cmean * 1.1, gapX = Math.max(0.36 * veh.length, 1.12 * cr);
    const epsArch = (b, CLf) => deg(CLf / (Math.PI * ARe)) * Math.abs(Math.sin(rad(b))) * Math.exp(-gMean * Math.abs(Math.cos(rad(b))) / (0.5 * cmean));
    const epsAt = (b, CLf) => nW < 2 ? 0 : deg(2 * CLf / (Math.PI * ARe)) * Math.abs(Math.cos(rad(b))) * Math.exp(-gapX * Math.abs(Math.sin(rad(b))) / (0.5 * cmean));
    if (sealMeas && veh.wingConfig === 'arch') { // convert the measured lift-slope ratio to σ with this model's own sensitivity
      const pS = (sg) => { const v2 = { ...veh, deckSeal: sg }, ST2 = AA.vehicleStack(v2, AA.stackOpts(v2)); return AA.stackForces(ST2, AA.sectionPolar('0012', 5e5), v2, 12, 6, 8, rho, { camberPct: 0 }).CLsys; };
      const r1 = pS(1) / Math.max(1e-9, pS(0)); veh.deckSeal = clamp((sealMeas.R - 1) / Math.max(1e-6, r1 - 1), 0, 1); sealMeas.sigma = veh.deckSeal; sealMeas.modelRatio = r1; }
    const ST = p.stackLL !== false ? AA.vehicleStack(veh, AA.stackOpts(veh)) : null;
    const forceAt = (bAbs, alpha, VA = Va, tandem = true, P = pol) => {
      if (ST && bAbs <= 45 && Math.abs(alpha) <= (P.alphaStall ?? 15) + 4) { // stacked lifting line (blended with the root-mirror model over 35–45°)
        const Fs = AA.stackForces(ST, P, veh, bAbs, alpha, VA, rho, { camberPct: p.camberMorph ? p.camberPct : 0, bridgeCdA: arch ? 0.012 * AM.bridgeArea : 0 });
        const Fz = { L: Fs.L, D: Fs.D, thrust: Fs.wingThrust, side: Fs.wingSide, c: { CL: Fs.CLsys / (p.interf || 1), CD: Fs.CDsys }, eps: 0, Db: Fs.Db, body: { thrust: 0, side: 0, L: Fs.Lb, xcp: 0.08 * veh.length }, net: Fs.net, sideTot: Fs.side, stack: true };
        if (bAbs <= 35) return Fz; const w = (45 - bAbs) / 10, Fo = forceAtOld(bAbs, alpha, VA, tandem, P), m = (k) => w * Fz[k] + (1 - w) * Fo[k];
        return { ...Fo, L: m('L'), D: m('D'), thrust: m('thrust'), side: m('side'), net: m('net'), sideTot: m('sideTot'), c: { ...Fo.c, CL: w * Fz.c.CL + (1 - w) * Fo.c.CL, CD: w * Fz.c.CD + (1 - w) * Fo.c.CD } };
      }
      return forceAtOld(bAbs, alpha, VA, tandem, P);
    };
    const forceAtOld = (bAbs, alpha, VA = Va, tandem = true, P = pol) => {
      const c1 = C3(alpha, P); let c = c1, eps = 0;
      if (arch && tandem) { // lateral biplane interference: in a crosswind the windward leg (upwash) precedes the leeward leg (downwash)
        eps = epsArch(bAbs, c1.CL); const cw = C3(alpha + 0.3 * eps, P), cl = C3(alpha - eps, P), e = rad(eps); c = { ...c1, CL: (cw.CL + cl.CL * Math.cos(e)) / 2, CD: (cw.CD + cl.CD + cl.CL * Math.sin(e)) / 2 };
      } else if (tandem && nW >= 2) { eps = epsAt(bAbs, c1.CL); const c2 = C3(alpha - eps, P), e = rad(eps); c = { ...c1, CL: (c1.CL + c2.CL * Math.cos(e)) / 2, CD: (c1.CD + c2.CD + c2.CL * Math.sin(e)) / 2 }; }
      const F = wingForces(VA, bAbs, c.CL * p.interf, c.CD, S, rho), Db = 0.5 * rho * (veh.CdA + (arch ? 0.012 * AM.bridgeArea : 0)) * VA * VA, B = bodyAt(bAbs, VA);
      return { ...F, c, eps, Db, body: B, net: F.thrust + B.thrust - Db * Math.cos(rad(bAbs)), sideTot: F.side + B.side + Db * Math.sin(rad(bAbs)) };
    };
    // D3 free-feathering root: safe side force and the feathered (weathervaned) state, C_L ≈ 0, C_D ≈ 0.02
    const mu0 = ctx.upstream('s10')?.muPeak ?? 0.95, ssf0 = ctx.upstream('s11')?.ssf ?? ctx.upstream('s01')?.ssf ?? (veh.track / 2) * (2 / 3) / veh.cgHeight, hCE0 = ctx.upstream('s01')?.hCE_m ?? (arch ? AA.sideCE(ctx.vehicle) ?? AM.zRoot + 0.45 * AM.legHeight : veh.wheelRadius + veh.height * 0.7 + span * 0.45);
    const SS0 = ctx.upstream('s01')?.safeSide ?? safeSideForce({ mass: veh.mass, cgHeight: veh.cgHeight, ssf: ssf0, hCE: hCE0, mu: mu0 }), FyGrip = SS0.Fgrip, FyRoll = SS0.Froll, FySafe = ctx.upstream('s01')?.FySafe_N ?? SS0.Fs; /* the one cap: Suite 01 safe side force (js/core/safeside.js at the lifting-line h_CE) */
    const feathered = (bAbs, VA = Va) => { const F = wingForces(VA, bAbs, 0, 0.02, S, rho), Db = 0.5 * rho * veh.CdA * VA * VA, B = bodyAt(bAbs, VA); return { ...F, c: { CL: 0, CD: 0.02, regime: 'feathered' }, eps: 0, Db, body: B, net: F.thrust + B.thrust - Db * Math.cos(rad(bAbs)), sideTot: F.side + B.side + Db * Math.sin(rad(bAbs)), feathered: true }; };
    const safe = (F, bAbs) => (p.breakaway === false || F.sideTot <= FySafe) ? F : feathered(bAbs);
    // optimum incidence at current apparent-wind angle (α = β − δ)
    const alphaScan = linspace(-10, 40, 251); let best = { net: -Infinity };
    for (const a of alphaScan) { const F = forceAt(ab, a); if ((p.breakaway === false || F.sideTot <= FySafe) && F.net > best.net) best = { ...F, alpha: a }; }
    if (!Number.isFinite(best.net)) best = { ...feathered(ab), alpha: 0 };
    // Revision 5: compliant flexure root — the arch self-trims (no mechanism); lift is capped passively by de-cambering
    const flexRoot = arch && (veh.archRoot || 'flexure') === 'flexure', GA = flexRoot ? AA.archGeometry(veh) : null, flexP = { kTheta: p.flexK, xAxis: p.flexAxis, Cm0: p.flexCm0, deltaMax: p.flexMax };
    const TR = flexRoot ? AA.trimState(GA, pol, veh, ab, Va, rho, { ...flexP, FyS: FySafe }) : null;
    const toF = (t) => ({ L: t.L, D: t.D, thrust: t.wingThrust, side: t.wingSide, body: t.body, Db: t.Db, net: t.net, sideTot: t.side, c: { CL: t.CL, CD: t.CD, regime: t.capped ? 'lift-capped (de-cambered)' : 'trimmed' }, eps: t.eps, capped: t.capped });
    const alphaOp = TR ? TR.alpha : p.incMode === 'optimum' ? best.alpha : ab - p.incidence, delta = ab - alphaOp;
    const FopLocked = TR ? toF(TR) : forceAt(ab, alphaOp), Fop = TR ? FopLocked : safe(FopLocked, ab), FopNoTandem = TR ? FopLocked : forceAt(ab, alphaOp, Va, false);
    // thrust vs apparent-wind angle
    const awa = range(37).map(i => i * 5), wThr = [], wSide = [], wAlpha = [], thrOpt = [], sideOpt = [], incOpt = [], thrFix = [], cxOpt = [];
    const thrUnsafe = [], thrNoTandem = [], thrIdeal = [], dTrim = [];
    // heavy flexure-arch work (Re-band polars, envelope, Suite 20 grid, sustained speeds) runs in a Web Worker when available
    const W5s = [2, 2.5, 3, 4, 6, 10], strip = (q) => ({ alpha: q.alpha, CL: q.CL, CD: q.CD, alphaStall: q.alphaStall, CLmax: q.CLmax });
    const FX = flexRoot ? await flexComputeAsync({ veh, pol: strip(pol), pts, polOpts: { alphas: range(p.alphaMax + 7).map(i => i - 6), xtrU: xtr, xtrL: xtr }, Re, cmean, nu, Va, rho, flexP, FySafe, flexMax: p.flexMax, S, awa, VaGrid: [2, 3.5, 5, 7, 10, 14, 19, 26], W5: W5s, fills: p.fillStudy !== false }) : null;
    if (FX) { const E = FX.E; thrOpt.push(...E.thrOpt); sideOpt.push(...E.sideOpt); incOpt.push(...E.incOpt); wThr.push(...E.wThr); wSide.push(...E.wSide); wAlpha.push(...E.wAlpha); thrFix.push(...E.thrFix); cxOpt.push(...E.cxOpt); thrUnsafe.push(...E.thrUnsafe); thrNoTandem.push(...E.thrNoTandem); thrIdeal.push(...E.thrIdeal); dTrim.push(...E.dTrim); }
    // speed-resolved coefficient grid (trimmed, uncapped) for Suite 20: the trim angle depends on q through k_θ, so a
    // single-V_a table scaled by V_a² misstates light-wind thrust (thrust is a small difference of lift and drag terms)
    const VaGrid = [2, 3.5, 5, 7, 10, 14, 19, 26], grid = flexRoot ? { Va: VaGrid, awa, CL: [], CD: [], CL0: [], CD0: [], delta: [] } : null;
    // Reynolds number follows the grid speed: polars at three Re bands (light / moderate / fresh apparent wind)
    const reBands = FX ? FX.bands : [3.5, 8, 16].map(V => ({ V, pol: Math.abs(V * cmean / nu - Re) / Re < 0.15 ? pol : polar(pts, Math.max(2e4, V * cmean / nu), { alphas: range(p.alphaMax + 7).map(i => i - 6), xtrU: xtr, xtrL: xtr }) }));
    await ctx.yield();
    const polAt = (V) => reBands.reduce((a, b) => Math.abs(Math.log(b.V / V)) < Math.abs(Math.log(a.V / V)) ? b : a).pol;
    if (FX) Object.assign(grid, FX.grid);
    // (grid filled by the worker above)
    // driver-trimmed (ring / tandem) wings: wing-only coefficients vs incidence at each AWA and Re band, so Suite 20 picks
    // the best incidence under its own side-force cap at the actual apparent-wind speed (same physics as the flexure grid)
    const trimAlphas = linspace(-5, 35, 41), trimTab = !flexRoot ? { alphas: trimAlphas, awa, bands: reBands.map(r => ({ V: r.V, Re: Math.round(r.V * cmean / nu), CL: awa.map(b => trimAlphas.map(a => forceAt(Math.max(0.5, b), a, Va, true, r.pol).c.CL * p.interf)), CD: awa.map(b => trimAlphas.map(a => forceAt(Math.max(0.5, b), a, Va, true, r.pol).c.CD)) })) } : null;
    if (!flexRoot) for (const b of awa) { let bb = { net: -Infinity }, bu = -Infinity, bn = -Infinity; for (const a of linspace(-5, 35, 81)) { const F = forceAt(b, a); bu = Math.max(bu, F.net); bn = Math.max(bn, forceAt(b, a, Va, false).net); if ((p.breakaway === false || F.sideTot <= FySafe) && F.net > bb.net) bb = { ...F, alpha: a }; } if (!Number.isFinite(bb.net)) bb = { ...feathered(b), alpha: b }; thrOpt.push(bb.net); sideOpt.push(bb.sideTot); incOpt.push(b - bb.alpha); thrFix.push(safe(forceAt(b, b - p.incidence), b).net); cxOpt.push(bb.net / (0.5 * rho * Va * Va * S)); thrUnsafe.push(bu); thrNoTandem.push(bn); }
    ctx.progress(0.5, 'verification');
    // geometry levers: centre of effort & CG
    const L = veh.length, x1W = -L * 0.42 + 0.10 * L + 0.25 * cr, wingX = arch ? [VM.wingX[0]] : [x1W, x1W + gapX].slice(0, nW), xCE = mean(wingX) + 0.05 * cmean, cg = ctx.upstream('s01')?.cg || [0, 0, veh.cgHeight];
    const zCE = hCE0, yaw = Fop.sideTot * (xCE - cg[0]), heel = (Fop.sideTot - (Fop.body?.side || 0)) * zCE + (Fop.body?.side || 0) * hCEb, restoring = veh.mass * G0 * (veh.track / 2) * 0.6;
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
    if (arch) { k('Box-wing induced-drag factor k (Prandtl)', kBox, '', `Closed arch mirrored in the body/ground plane: span b = ${fmt(bBox, 3)} m, mean gap g = ${fmt(gMean, 3)} m (g/b = ${fmt(gMean / bBox, 3)}). k < 1 means less induced drag than a monoplane of the same span.`, 'calc'); k('Effective aspect ratio (box wing)', ARe, '', `b²/S (mirrored) = ${fmt(AR, 3)} divided by k; lift slope also × cos Λ = ${fmt(cosL, 3)} for ${AM.sweep}° sweep.`, 'calc'); k('Lateral leg interference ε (leeward leg)', FopLocked.eps, '°', 'In a crosswind the windward leg precedes the leeward one: leeward α reduced by ε, windward raised by 0.3ε (lift redistributes, total induced drag per Munk’s stagger theorem).', 'calc'); }
    else k('Tandem interference: rear-wing sidewash ε', FopLocked.eps, '°', `Induced sidewash of the front wing at the rear wing (gap ${fmt(gapX, 3)} m, |cos β| weighting, cross-flow decay).`, 'calc');
    if (!arch) k('Tandem interference penalty on net thrust', FopNoTandem.net - FopLocked.net, 'N', 'Net thrust lost because the rear wing operates in the front wing’s downwash (same incidence).', 'calc');
    k('Safe side force (breakaway)', FySafe, 'N', `min(grip 0.7 μmg = ${fmt(FyGrip, 3)} N, rollover reserve ≥ 0.5 g → ${fmt(FyRoll, 3)} N, tip-over / 1.4 = ${fmt(SS0.Ftip / 1.4, 3)} N) — binding: ${ctx.upstream('s01')?.safeSide?.binding ?? SS0.binding}; value published by Suite 01; SSF ${fmt(ssf0, 3)}, h_CE ${fmt(hCE0, 3)} m.`, 'calc');
    k('Wing root state', Fop.feathered ? 'FEATHERED' : 'locked', '', Fop.feathered ? `Locked side force ${fmt(FopLocked.sideTot, 3)} N would exceed ${fmt(FySafe, 3)} N; reported forces are the feathered loads.` : 'Operating inside the safe envelope.', 'calc', Fop.feathered ? 'warn' : 'ok');
    const bodyAWA = range(37).map(i => i * 5), bodyTh = bodyAWA.map(b => bodyAt(b).thrust), bodySd = bodyAWA.map(b => bodyAt(b).side), Bop = bodyAt(ab);
    k('Body lifting-surface thrust / side force', `${fmt(Bop.thrust, 3)} / ${fmt(Bop.side, 3)}`, 'N', `Monocoque as a low-AR airfoil: AR_eff = 2h²/S = ${fmt(ARb, 3)} with ground reflection, C_L = ${fmt(Bop.CL, 3)} (side-panel camber ${p.bodyCamberPct} %). Included in the net force.`, 'calc');
    k('Wing-root end-plate factor (body)', p.endplate, '×AR', 'Sealed root gap: the body acts as an end plate, raising the wings’ effective aspect ratio.', 'calc');
    out.charts.push({ title: 'Body as a wing: thrust and side force vs apparent-wind angle', x: { label: 'Apparent-wind angle β', unit: '°' }, y: { label: 'Force', unit: 'N' }, series: [{ name: 'Body thrust', x: bodyAWA, y: bodyTh }, { name: 'Body side force', x: bodyAWA, y: bodySd, dash: '5 3' }], annotations: [{ type: 'vline', x: ab, label: 'current β' }], explain: 'The plan-form acts as a very low aspect-ratio wing with a sharp trailing edge; ground reflection doubles its effective aspect ratio and the metamaterial side panels camber it to leeward. Its centre of effort is low (≈ wheel radius + h/2), so it adds thrust with little overturning moment.' });
    // Directional (weathercock) balance: centre of effort of the total aerodynamic side force vs the centre of lateral resistance
    // of the tyres (cornering stiffness ∝ axle load → CLR at the axle-load-weighted position, i.e. the CG station).
    {
      const xWing = xCE, xBody = Fop.body?.xcp ?? 0.08 * veh.length, sW = Fop.sideTot - (Fop.body?.side || 0), sB = Fop.body?.side || 0;
      const xCEtot = (sW * xWing + sB * xBody) / Math.max(1e-9, sW + sB), wl = ctx.upstream('s01')?.wheelLoads, wb = veh.wheelbase, xf = 0.58 * wb, xr = -0.42 * wb;
      const fF = Array.isArray(wl) && wl.length >= 3 ? (wl[0] + wl[1]) / (wl[0] + wl[1] + wl[2]) : 0.6, xCLR = fF * xf + (1 - fF) * xr, lead = xCEtot - xCLR, Nyaw = Fop.sideTot * lead;
      k('Aerodynamic centre of effort vs centre of lateral resistance', `${fmt(xCEtot, 3)} / ${fmt(xCLR, 3)} m (lead ${fmt(lead, 3)} m)`, '', `x positions in the vehicle frame (body reference, +x forward): wing CE ${fmt(xWing, 3)} m (arch station), body CE ${fmt(xBody, 3)} m, CLR from the axle-load split (front ${fmt(fF * 100, 3)} %). Lead > 0: CE ahead of CLR (bears away — directionally unstable “lee helm”); lead < 0: CE behind (luffs into the wind — weathercock stable “weather helm”).`, 'calc', Math.abs(lead) < 0.15 * wb ? 'ok' : 'warn');
      k('Aerodynamic yaw moment about the CLR', Nyaw, 'N·m', 'Side force × lead; the steering must trim this moment (Suite 11).', 'calc');
      out.verdicts.push({ test: 'Directional balance (CE vs CLR, weathercock stability)', status: lead <= 0 && -lead < 0.2 * wb ? 'pass' : lead > 0 && lead < 0.05 * wb ? 'warn' : lead > 0 ? 'fail' : 'warn', detail: `CE ${fmt(Math.abs(lead), 3)} m ${lead > 0 ? 'ahead of' : 'behind'} the CLR → yaw moment ${fmt(Nyaw, 3)} N·m (${lead > 0 ? 'tends to bear away; unstable' : 'mild weather helm; stable'}).`, fix: 'Move the arch aft (or the axle-load split forward) until the CE sits slightly behind the CLR.' });
      out.yawBalance = { xCE: xCEtot, xCLR, lead, Nyaw };
    }
    // Deployed vs reefed at 4 / 6 / 10 m/s beam true wind (current vehicle speed), optimum safe incidence, camber morphing,
    // tandem interference and the state-dependent breakaway limit (lower centre of effort when reefed).
    const perfAt = (U, spanX) => {
      if (flexRoot) { const aw = apparentWind(U, p.heading + 90, p.heading, v), b = Math.abs(aw.beta), t = AA.trimState(GA, pol, veh, b, Math.max(aw.Va, 1e-3), rho, { ...flexP, FyS: FySafe }); return { net: t.net, side: t.side, CL: t.CL, delta: t.delta, capped: t.capped, FyS: FySafe }; }
      if (arch) { const aw = apparentWind(U, p.heading + 90, p.heading, v), b = Math.abs(aw.beta), VA = Math.max(aw.Va, 1e-3); let best = null; for (const a of linspace(-5, 35, 81)) { const F = forceAt(b, a, VA); if ((p.breakaway === false || F.sideTot <= FySafe) && (!best || F.net > best.net)) best = { net: F.net, side: F.sideTot, CL: F.c.CL }; } if (!best) { const F = feathered(b, VA); best = { net: F.net, side: F.sideTot, CL: 0, feathered: true }; } return { ...best, FyS: FySafe }; }
      const SX = (w2?.area || veh.wingArea) * spanX / spanDep, cmx = SX / nW / spanX, ARx = spanX * spanX / (SX / nW) * p.endplate, aw = apparentWind(U, p.heading + 90, p.heading, v), b = Math.abs(aw.beta), VA = Math.max(aw.Va, 1e-3);
      const hx = veh.wheelRadius + veh.height * 0.7 + spanX * 0.45, FyS = safeSideForce({ mass: veh.mass, cgHeight: veh.cgHeight, ssf: ssf0, hCE: hx, mu: mu0 }).Fs, gx = Math.max(0.36 * veh.length, 1.12 * cmx * 1.1);
      const cw = (a) => { const c = coeffs(pol, a); if (Math.abs(a) > pol.alphaStall + 8) return c; const f = finiteWing(c.CL, c.CD, ARx, p.oswald); return { CL: f.CL, CD: f.CD }; };
      let best = null;
      for (const a of linspace(-5, 35, 81)) { const c1 = cw(a), e = nW < 2 ? 0 : deg(2 * c1.CL / (Math.PI * ARx)) * Math.abs(Math.cos(rad(b))) * Math.exp(-gx * Math.abs(Math.sin(rad(b))) / (0.5 * cmx)), c2 = cw(a - e), CL = nW < 2 ? c1.CL : (c1.CL + c2.CL * Math.cos(rad(e))) / 2, CD = nW < 2 ? c1.CD : (c1.CD + c2.CD + c2.CL * Math.sin(rad(e))) / 2;
        const F = wingForces(VA, b, CL * p.interf, CD, SX, rho), Db = 0.5 * rho * veh.CdA * VA * VA, Bd = bodyAt(b, VA), net = F.thrust + Bd.thrust - Db * Math.cos(rad(b)), side = F.side + Bd.side + Db * Math.sin(rad(b));
        if ((p.breakaway === false || side <= FyS) && (!best || net > best.net)) best = { net, side, CL }; }
      if (!best) { const F = wingForces(VA, b, 0, 0.02, SX, rho), Db = 0.5 * rho * veh.CdA * VA * VA, Bd = bodyAt(b, VA); best = { net: F.thrust + Bd.thrust - Db * Math.cos(rad(b)), side: F.side + Bd.side + Db * Math.sin(rad(b)), CL: 0, feathered: true }; }
      return { ...best, FyS };
    };
    const windsP = [4, 6, 10], perfD = windsP.map(U => perfAt(U, spanDep)), perfR = windsP.map(U => perfAt(U, reefSpan));
    let sustained = null;
    if (flexRoot) {
      const W5 = W5s; sustained = FX.sustained; // steady force balance, as Suite 20 (computed in the worker)
      out.tables.push({ title: `AeroArch (compliant flexure, no mechanism) — beam reach at ${p.speed} km/h and best-heading sustainable speed`, cols: ['True wind', 'Beam-reach net thrust [N]', 'Side force [N]', 'Flexure deflection δ', 'Best heading (TWA)', 'Sustainable speed [km/h]'], rows: W5.map((U, i) => { const q = perfAt(U); return [`${U} m/s`, fmt(q.net, 4) + (q.capped ? ' (lift-capped)' : ''), fmt(q.side, 4), fmt(q.delta, 3) + '°', `${sustained[i].twa}°`, fmt(sustained[i].v * 3.6, 4)]; }), explain: `The arch trims itself on its flexure (C_L,trim = C_m0/(0.25 − x_axis) = ${fmt(p.flexCm0 / Math.max(1e-6, 0.25 - p.flexAxis), 3)}) within ±${p.flexMax}°; the driver picks the heading that gives the highest reachable speed (sailing practice). Same tyres (C_rr ${veh.Crr}) and drag (C_dA ${veh.CdA}) for every revision.` });
      const inEnv = awa.filter((b, i) => thrOpt[i] > veh.mass * G0 * veh.Crr), env = inEnv.length ? `${Math.min(...inEnv)}°–${Math.max(...inEnv)}°` : 'none';
      k('Wind-direction envelope (self-trimmed, net thrust > rolling resistance)', env, '', `Apparent-wind angles at ${fmt(Va, 3)} m/s for which the ±${p.flexMax}° flexure lets the arch produce useful thrust; outside it, choose a heading that brings the apparent wind into the sector.`, 'calc');
      k('Self-trim lift coefficient / flexure deflection now', `${fmt(TR.CL, 3)} / ${fmt(TR.delta, 3)}°`, '', TR.atStop ? 'At the flexure stop — the apparent wind is outside the trim sector here.' : 'Trimmed by the aerodynamic moment about the flexure axis.', 'calc', TR.atStop ? 'warn' : 'ok');
      out.verdicts.push({ test: 'Passive self-trim inside the ±25° envelope (no mechanism)', status: TR.atStop ? 'warn' : 'pass', detail: `δ = ${fmt(TR.delta, 3)}°, α = ${fmt(TR.alpha, 3)}°, C_L = ${fmt(TR.CL, 3)}${TR.capped ? ' — lift capped by de-cambering (gust protection)' : ''}.` });
    }
    if (arch && !flexRoot) out.tables.push({ title: `AeroArch box wing — net forward force and side force, beam true wind at ${p.speed} km/h`, cols: ['True wind', 'Net thrust [N]', 'Side force [N]', 'Safe side limit [N]', 'State'], rows: windsP.map((U, i) => [`${U} m/s`, fmt(perfD[i].net, 4), fmt(perfD[i].side, 4), fmt(perfD[i].FyS, 4), perfD[i].feathered ? 'feathered (breakaway)' : 'locked']), explain: `Optimum safe incidence (whole arch rotates on the ring track), ${p.camberMorph ? `leeward camber ${p.camberPct} %` : 'symmetric section'}, box-wing induced drag (k = ${fmt(kBox, 3)}), sweep ${AM.sweep}°, lateral leg interference, bridge profile drag and the lifting body. The arch does not reef; it folds for parking.` });
    else out.tables.push({ title: `Deployed (${fmt(spanDep, 3)} m) vs reefed (${fmt(reefSpan, 3)} m) — net forward force and side force, beam true wind at ${p.speed} km/h`, cols: ['True wind', 'Deployed net thrust [N]', 'Deployed side [N]', 'Reefed net thrust [N]', 'Reefed side [N]', 'Safe side limit dep./reef. [N]'], rows: windsP.map((U, i) => [`${U} m/s`, fmt(perfD[i].net, 4) + (perfD[i].feathered ? ' (feathered)' : ''), fmt(perfD[i].side, 4), fmt(perfR[i].net, 4) + (perfR[i].feathered ? ' (feathered)' : ''), fmt(perfR[i].side, 4), `${fmt(perfD[i].FyS, 4)} / ${fmt(perfR[i].FyS, 4)}`]), explain: `Optimum safe incidence, ${p.camberMorph ? `leeward camber ${p.camberPct} % (solved NACA ${af.name.replace('NACA ', '')})` : 'symmetric section'}, tandem downwash interference and the free-feathering limit. Deploy in light wind for area; reef in strong wind for a lower centre of effort and a higher breakaway margin.` });
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
    out.charts.push({ title: 'Forward thrust vs apparent-wind angle', x: { label: 'Apparent-wind angle β', unit: '°' }, y: { label: 'Force', unit: 'N' }, series: [{ name: 'Net thrust, optimum safe incidence', x: awa, y: thrOpt }, { name: `Net thrust, fixed δ=${p.incidence}°`, x: awa, y: thrFix, dash: '5 3' }, ...(flexRoot ? [{ name: `Best α inside ±${p.flexMax}° (ideal adjustable root)`, x: awa, y: thrIdeal, dash: '6 2' }, { name: 'Unlimited incidence (reference)', x: awa, y: thrUnsafe, dash: '2 3' }] : [{ name: 'Unconstrained (unsafe) maximum', x: awa, y: thrUnsafe, dash: '2 3' }, { name: 'Without tandem interference', x: awa, y: thrNoTandem, dash: '6 2' }]), { name: 'Side force (optimum safe)', x: awa, y: sideOpt }], annotations: [{ type: 'hline', y: Froll, label: 'rolling resistance', color: '#22c55e' }, { type: 'hline', y: FySafe, label: 'safe side-force limit', color: '#ef4444' }, { type: 'vline', x: ab, label: 'current β' }], wide: true, explain: 'Optimum incidence is chosen inside the safe envelope (side force ≤ breakaway limit); the dotted curve shows what an unprotected wing would produce. At the current apparent-wind speed: head winds give no useful thrust, beam to broad reaches give the most. The gap between the two thrust curves is the penalty of a fixed (non-adjustable) wing root.' });
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
    out.explain.push({ title: 'Solver hierarchy', text: 'Full-Reynolds-number section polars come from the coupled panel / integral-boundary-layer method (checked against thin-airfoil theory and Kutta–Joukowski). The live LBM tab adds an unsteady, reduced-Re view of separation and vortex shedding. Higher-fidelity 3-D RANS/LES escalation is reserved for configurations that pass this screening.' });
    out.convergence = { title: 'LBM Poiseuille benchmark profile error', series: [{ name: '|u_num − u_exact| / u_max', x: pois.y, y: pois.num.map((u, i) => Math.max(1e-8, Math.abs(u - pois.exact[i]) / pois.umax)) }] };
    let fillOut = null;
    if (FX?.fills) { const am = ctx.upstream('s01')?.wingArealMass ?? 2.4, m0 = veh.mass, cg0 = veh.cgHeight, VF0 = ctx.upstream('s06')?.flutterSpeed, H = FX.fills.find(f => f.fill === 'hollow');
      const rowsF = FX.fills.map(f => { const dm = f.areaFill * am * 1.15, dcg = dm * (f.zc - cg0) / (m0 + dm); return [f.fill, ...f.v.map(x => +x.toFixed(1)), +f.LD.toFixed(2), +f.tip.toFixed(2), +f.parkedRatio.toFixed(3), +dm.toFixed(2), +(dcg * 1000).toFixed(1), VF0 ? +(VF0 * f.torsionFactor).toFixed(1) : '—', f.fill === 'hollow' ? 'baseline (2 legs + bridge)' : f.fill === 'solid' ? '+1 large sandwich web, 2 extra bonded joints' : '+2 small root gussets, 2 corner fairings']; });
      const hy = FX.fills.find(f => f.fill === 'hybrid'), so = FX.fills.find(f => f.fill === 'solid'), better = (f) => f.v.every((x, i) => x >= H.v[i] - 0.05) && f.tip >= H.tip - 0.05;
      const rec = better(hy) && hy.torsionFactor > 1 ? 'hybrid' : 'hollow';
      out.tables.push({ title: 'Arch opening: hollow vs solid vs hybrid (identical conditions, beam wind)', cols: ['Opening', '2 m/s', '2.5', '3', '4', '6', '10 m/s', 'L/D at β 12°, 9 m/s', 'Arch-locked tip-over [m/s]', 'Parked side area ratio', 'Added mass [kg]', 'CG rise [mm]', 'Flutter speed estimate [m/s]', 'Manufacturing'], rows: rowsF,
        explain: `For a sail-car the useful force is horizontal, so a vertical fill IS a propulsive surface — but the light-wind thrust of this arch is span-limited (induced drag ∝ L²/b_e²), so extra chord at fixed span adds mostly profile drag, side area and mass. Solid: ${so.v.map(x => fmt(x, 3)).join('/')} km/h vs hollow ${H.v.map(x => fmt(x, 3)).join('/')}; parked side area ×${fmt(so.parkedRatio, 3)}. Hybrid root gusset: ${hy.v.map(x => fmt(x, 3)).join('/')} km/h, a stiffer leg root (estimated torsion-frequency factor ×${fmt(hy.torsionFactor, 3)} on the flutter speed) for ${fmt(hy.areaFill * am * 1.15, 3)} kg. Flutter estimate scales the Suite 06 V–g result by the torsion-frequency factor of the constrained leg length (an estimate; confirm with Suite 06/08 modal analysis of the actual web). Recommendation: ${rec === 'hybrid' ? 'adopt the hybrid — no speed or tip-over loss and more flutter margin' : 'keep the hollow arch unless the flutter margin governs; the hybrid gusset is the fallback for flutter'}.` });
      fillOut = { rows: FX.fills, recommended: rec }; }
    out.tables.push({ title: `Geometry-based drag-area build-up (C_dA ${fmt(DBU.CdA, 3)} m²${p.dragModel === 'input' ? ` — NOT used: fixed input ${ctx.vehicle.CdA} m²` : ' — used by Suites 07 and 20'})`, cols: ['Item', 'C_dA [m²]'], rows: [...DBU.rows.map(r => [r[0], +r[1].toFixed(4)]), ['Total', +DBU.CdA.toFixed(4)]], explain: 'Same formulae for every revision, each from its own CAD geometry: Hoerner streamlined-body drag on frontal area with a turbulent skin-friction coefficient at V = 8 m/s and a ground-proximity factor, plus wheels, arms, junctions, canopy/leakage and 15 % interference. The arch legs and bridge profile drag are in the wing model.' });
    out.outputs = { FySafe_N: FySafe, hCE_m: hCE0, hCEEnvelope: ctx.upstream('s01')?.hCEEnvelope ?? null, sealMeasured: sealMeas, deckSealUsed: veh.deckSeal, archFillStudy: fillOut, CdA_model: veh.CdA, dragBuildup: DBU.rows.map(r => ({ item: r[0], CdA: r[1] })), stackModel: { on: p.stackLL !== false, deckSeal: p.deckSeal ?? 0, groundSeal: p.groundSeal ?? 0, bodyEta: p.bodyEta ?? 0.6 }, flexure: flexRoot ? { ...flexP, CLtrim: p.flexCm0 / Math.max(1e-6, 0.25 - p.flexAxis), deltaNow: TR.delta, envelope: { awa, thrustTrim: thrOpt, thrustIdeal: thrIdeal, delta: dTrim, wingThrust: wThr, wingSide: wSide, alpha: wAlpha, Va, Sw: S, grid, minScale: 0.6 } } : null, sustainableSpeeds: sustained ? sustained.map(q => ({ U: q.U, v_kmh: q.v * 3.6, twa: q.twa })) : null, yawBalance: out.yawBalance, bodyForces: { awa: bodyAWA, thrust: bodyTh, side: bodySd, hCE_m: hCEb, Va, note: 'lift-induced thrust/side of the body plan-form at V_a; the C_dA drag is NOT included' }, bridgeCdA: GA ? 0.012 * (GA.AM.bridgeArea || 0) : 0, endplateFactor: arch ? ARe / (legLen * legLen / Sw) : p.endplate, wingConfig: arch ? 'arch' : 'tandem', boxWing: arch ? { k: kBox, gOverB: gMean / bBox, ARmirrored: AR, ARe, sweepFactor: cosL } : null, wingState: p.wingState, camberPct: p.camberMorph ? p.camberPct : 0, perfStates: { winds: windsP, deployed: perfD, reefed: perfR }, feathered: !!Fop.feathered, FySafe, tandemPenaltyN: FopNoTandem.net - FopLocked.net, rearSidewashDeg: FopLocked.eps, polar: { alpha: A, CL: pol.CL, CD: pol.CD }, polarBands: reBands.map(r => ({ V: r.V, alpha: r.pol.alpha, CL: r.pol.CL, CD: r.pol.CD, alphaStall: r.pol.alphaStall, CLmax: r.pol.CLmax })), CLmax: pol.CLmax, alphaStall: pol.alphaStall, Re, Va, awa: beta, alpha: alphaOp, incidence: delta, thrustN: Fop.net, wingThrustN: Fop.thrust, liftN: Fop.L, dragN: Fop.D, sideN: Fop.sideTot, yawMoment: yaw, heelMoment: heel, CdA_body: veh.CdA, thrustVsAWA: { awa, thrust: thrOpt, side: sideOpt, incidence: incOpt, note: 'net incl. body forces and body drag' }, trimTable: trimTab, rho, airfoil: af.name };
    out.headline = `At ${p.speed} km/h with ${fmt(p.Uw, 3)} m/s true wind from ${p.dir}°: apparent wind ${fmt(Va, 3)} m/s at ${fmt(beta, 3)}° → lift ${fmt(Fop.L, 3)} N, net forward force ${fmt(Fop.net, 3)} N, side force ${fmt(Fop.sideTot, 3)} N.`;
    out.headlineKind = Fop.net > Froll ? 'ok' : 'warn';
    out.credibility = out.checks.every(c => c.pass) ? 'Numerically verified' : 'Analytical screening';
    if (memoKey) { if (S07_MEMO.size > 12) S07_MEMO.delete(S07_MEMO.keys().next().value); S07_MEMO.set(memoKey, out); }
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
  el.append(callout('warn', h('b', {}, 'Reduced-Reynolds-number 2-D simulation. '), 'This lattice-Boltzmann LES resolves the unsteady wake, separation and vortex shedding at Re ≈ 10²–10⁴ for physical insight. Full-Re design forces come from the panel/boundary-layer polar on the Plots tab (predicted, simulation).'),
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
  // AeroArch: the two legs are canted lifting lines closed by the bridge; the whole arch (legs, bridge, spars, fairings) turns on the ring track
  const archMode = veh.wingConfig === 'arch';
  if (archMode) { S.wingObjs.length = 0; for (const o2 of S.car.children) if (/^Arch (leg|top bridge|.*collector)/.test(o2.name) || /^Solar-selective collector \(arch/.test(o2.name)) S.wingObjs.push({ o: o2, base: Float32Array.from(o2.geometry.attributes.position.array), leg: /^Arch leg (port|starboard) \(skin/.test(o2.name) }); }
  const endPt = (B, hi) => { let zx = hi ? -Infinity : Infinity; for (let k = 2; k < B.length; k += 3) zx = hi ? Math.max(zx, B[k]) : Math.min(zx, B[k]); let sx = 0, sy = 0, n = 0; for (let k = 0; k < B.length; k += 3) if (Math.abs(B[k + 2] - zx) < 0.06) { sx += B[k]; sy += B[k + 1]; n++; } return [sx / n, sy / n, zx]; };
  const brObj = archMode ? S.wingObjs.find(w => /bridge/.test(w.o.name)) : null; let closure = null; if (brObj) { const B = brObj.base; let sx = 0, sy = 0, sz = 0; for (let k = 0; k < B.length; k += 3) { sx += B[k]; sy += B[k + 1]; sz += B[k + 2]; } closure = [3 * sx / B.length, 3 * sy / B.length, 3 * sz / B.length]; }
  const wingInfo = archMode ? S.wingObjs.filter(w => w.leg).map(w => { const a = endPt(w.base, false), b = endPt(w.base, true); return { x: a[0], ya: a[1], z0: a[2], xb: b[0], yb: b[1], z1: b[2], loading: 'box', closure }; })
    : S.wingObjs.map((w, k) => { const B = w.base; let zmin = Infinity, zmax = -Infinity; for (let i = 2; i < B.length; i += 3) { zmin = Math.min(zmin, B[i]); zmax = Math.max(zmax, B[i]); } return { x: S.asm.meta.wingX[k], z0: zmin, z1: zmax }; });
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
    if (archMode) { S.wingObjs.filter(w => w.leg).forEach((w, k) => { const wi = wingInfo[k], g = w.o.geometry, n = g.attributes.position.count, col = new Float32Array(n * 3), B = w.base; for (let i = 0; i < n; i++) { const eta = Math.min(1, Math.max(0, (B[i * 3 + 2] - wi.z0) / (wi.z1 - wi.z0))), v2 = (0.55 + 0.45 * Math.sqrt(1 - eta * eta)) * Math.min(1.5, Math.abs(st.F.L) / Math.max(1, Math.abs(opF.L))), rgb = cmap(v2 / 1.5, 'turbo'); col.set([rgb[0] / 255, rgb[1] / 255, rgb[2] / 255], i * 3); } g.setAttribute('color', new THREE.BufferAttribute(col, 3)); w.o.material.vertexColors = true; w.o.material.color.set(0xffffff); w.o.material.needsUpdate = true; }); } else
    S.wingObjs.forEach((w, k) => { const wi = wingInfo[k], g = w.o.geometry, n = g.attributes.position.count, col = new Float32Array(n * 3), Pz = w.base; for (let i = 0; i < n; i++) { const j = i % np, eta = Math.min(1, Math.max(0, (Pz[i * 3 + 2] - wi.z0) / (wi.z1 - wi.z0))), c = i < n - 2 ? cpOf[j] * (4 / Math.PI) * Math.sqrt(Math.max(0, 1 - eta * eta)) * Math.sign(st.F.L || 1) : 0, rgb = cmap((c - cpLo) / (cpHi - cpLo), 'coolwarm'); col.set([rgb[0] / 255, rgb[1] / 255, rgb[2] / 255], i * 3); } g.setAttribute('color', new THREE.BufferAttribute(col, 3)); w.o.material.vertexColors = true; w.o.material.color.set(0xffffff); w.o.material.needsUpdate = true; });
    leg.set(0.7 * Va, 1.3 * Va);
    return M;
  };
  // wing incidence: rotate each wing about its quarter-chord axis relative to the as-built incidence δ
  const setIncidence = (d) => { S.wingObjs.forEach((w, k) => { const P2 = w.o.geometry.attributes.position.array, B = w.base, a = rad(d - delta), c = Math.cos(a), sn = Math.sin(a), xc = archMode ? (closure ? closure[0] : 0) : wingInfo[k].x; for (let i = 0; i < B.length; i += 3) { const dx = B[i] - xc, dy = B[i + 1]; P2[i] = xc + c * dx - sn * dy; P2[i + 1] = sn * dx + c * dy; P2[i + 2] = B[i + 2]; } w.o.geometry.attributes.position.needsUpdate = true; w.o.geometry.computeVertexNormals(); }); };
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
