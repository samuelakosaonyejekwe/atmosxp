// Suite 23 — Safety, reliability, structural durability & failure prediction.
import { h, table } from '../core/ui.js';
import { fmt, linspace, range, rng, histogram, mean, G0 } from '../core/num.js';
import { form } from '../core/analysis.js';

const FMEA = [
  ['Wing / aeroelastic', 'Flutter or divergence of morphing wing', 'Structural failure, loss of control', 10, 3, 5, 'Flutter margin ≥ 1.15 × max gust (Suite 06/08); ground vibration test; mass balancing'],
  ['Wing / aeroelastic', 'Gust overload at wing root', 'Root fracture, wing loss', 10, 4, 4, 'FORM reliability ≥ β 3.8; proof test 1.5 × limit; gust-load-alleviating twist'],
  ['Wing / aeroelastic', 'Wing cannot be unloaded (mechanism jam)', 'Uncontrolled thrust while braking / parking', 9, 4, 5, 'Redundant unloading path; weekly function check; fail-safe feathering spring'],
  ['Wing / aeroelastic', 'Telescoping reef joint jams (cannot reef in rising wind)', 'Full span exposed above the reefing wind', 9, 3, 4, 'Dry-film / PTFE sliding bushings; reef before trips above forecast threshold; pre-trip function check'],
  ['Wing / aeroelastic', 'Reef latch fails — unintended collapse under load', 'Sudden thrust loss, pinch hazard', 8, 2, 4, 'Positive-locking latch with visual indicator; proof load 1.5× limit at assembly'],
  ['Wing / aeroelastic', 'Breakaway lock fails to release (no feathering)', 'Overturning in gusts', 10, 2, 5, 'Torque-limited detent calibrated at assembly; annual release-torque test (Suite 22 tolerances)'],
  ['Wing / aeroelastic', 'Breakaway releases prematurely', 'Loss of propulsion in moderate wind', 5, 4, 3, 'Release-torque window ±10 %; re-latch from cockpit'],
  ['Wing / aeroelastic', 'Skin delamination / tearing', 'Loss of aerodynamic shape, debris', 6, 4, 4, 'UV-stabilised skin; peel-ply inspection every 6 months'],
  ['Vehicle dynamics', 'Rollover under crosswind', 'Occupant injury', 10, 5, 3, 'Envelope limit on apparent wind; automatic wing unloading threshold; wider track / lower CP'],
  ['Vehicle dynamics', 'Loss of tyre adhesion during gust', 'Lateral departure', 8, 4, 4, 'Grip margin check (Suite 11); speed restriction in wet conditions'],
  ['Brakes', 'Brake fade during descent with wing thrust', 'Increased stopping distance', 9, 3, 4, 'Thermal capacity check (Suite 12); dual circuits; wing unloading before braking'],
  ['Brakes', 'Single circuit failure', 'Reduced deceleration', 8, 2, 3, 'Dual independent mechanical circuits; regulation-compliant secondary brake'],
  ['Steering', 'Linkage joint wear / disconnection', 'Loss of steering', 10, 2, 4, 'Safety-wired fasteners; annual torque audit; play measurement'],
  ['Hub drive', 'Cam seizure / cartridge jamming', 'Wheel lock, skid', 9, 4, 5, 'Clearance Cpk ≥ 1.33 (Suite 22); torque-limiting compliant shoe; disengagement path'],
  ['Hub drive', 'Unexpected torque generation (solar heating at rest)', 'Unintended motion', 8, 5, 4, 'Parking brake rated for maximum hub torque; thermal shutter'],
  ['Hub drive', 'NiTi functional fatigue (stroke loss)', 'Backup power decay', 5, 7, 4, 'Strain amplitude limit; replacement interval from Miner damage'],
  ['Hub drive', 'Cartridge fracture under centrifugal load', 'Debris in hub, imbalance', 7, 3, 5, 'Containment ring; CT inspection of AM housings'],
  ['Hub drive', 'Failure to disengage when wind returns', 'Drag, overheating', 5, 4, 4, 'Hysteretic switch qualification (Suite 19)'],
  ['Passive switching', 'Switching chatter in gusty wind', 'Torque fluctuation, wear', 4, 6, 3, 'Engage/disengage hysteresis band; damping'],
  ['Energy buffer', 'Uncontrolled snap-through release', 'Sudden wheel torque', 8, 3, 5, 'Mechanical latch with two-step release; shielding'],
  ['Structure', 'Composite impact damage (BVID)', 'Strength reduction', 7, 4, 6, 'Damage-tolerant laminate; tap/ultrasonic inspection'],
  ['Structure', 'Wing-root insert crack growth', 'Insert pull-out', 9, 3, 4, 'Paris-law inspection interval; fail-safe dual inserts'],
  ['Thermal', 'Heat-pipe dry-out / leak', 'Loss of backup power', 3, 4, 4, 'Leak test; condition monitoring of ΔT'],
  ['Thermal', 'Collector overheating at standstill', 'Polymer degradation', 5, 4, 4, 'Over-temperature venting; material T_max margin (Suite 03)'],
];

const ARCH_FMEA = [
  ['Arch / flexure pylon', 'Flexure leaf fatigue crack', 'Loss of root stiffness, arch trim change, eventual fracture', 10, 3, 4, 'Strain ≤ 0.4 %, shot-peened chemically milled leaves, 6-monthly eddy-current / dye-penetrant check, redundant leaf stack (Suite 22)'],
  ['Arch / flexure pylon', 'Flexure over-travel into end stop in a gust', 'Impact load, local damage', 6, 4, 4, 'Progressive stiffening curve, elastomer-faced stops, gust load case in Suite 08'],
  ['Arch / camber skin', 'Bistable camber skin fails to snap to the leeward side', 'Reduced or reversed thrust on that tack', 6, 4, 3, 'Snap-load check per panel, temperature-compensated prestress, visual tell-tale'],
  ['Arch / washout', 'Passive washout insufficient at gust load', 'Wing overload or overturning in a severe gust', 10, 3, 4, 'Washout predicted by Suites 06/08 (simulation; awaiting gust test); reduced operating envelope above the design gust'],
  ['Conspicuity / legal', 'Vehicle not seen in low light (no electric lamps by specification)', 'Collision at dusk, night or in fog', 9, 4, 3, 'Passive conspicuity only (retroreflectors front/rear/side, retroreflective and photoluminescent tape, mirrors, high-visibility flag); legal limitation: daylight / good-visibility public-road operation only; night and low-visibility scenarios remain for closed-course and research use'],
  ['Arch / structure', 'Closed-frame delamination at a leg–bridge corner', 'Loss of box-wing stiffness, flutter risk', 8, 3, 5, 'Corner reinforcement, tap test each service, flutter margin check (Suite 06)'],
];
export default {
  description: 'Deterministic and probabilistic safety assessment of the complete vehicle: FMEA with risk priority, fault-tree structure with order-of-magnitude bounds, FORM and Monte Carlo reliability of the wing root, NiTi cartridge fatigue with Miner damage, Paris-law crack growth with inspection intervals, crosswind rollover envelope and a consolidated safety-margin register from all coupled suites.',
  literature: 'structural reliability FORM fatigue Miner rule Paris law wind-powered land vehicle rollover safety',
  credibility: 'Analytical screening',
  equations: [
    'Limit-state function g = R − S; first-order reliability method (Hasofer–Lind / Rackwitz–Fiessler), reliability index β, P_f = Φ(−β)',
    'Crude Monte Carlo reliability simulation with coefficient-of-variation estimate',
    'Thin-walled box-beam bending σ = M/Z; elliptic span loading root moment M = F·4b/(3π)',
    'Strain–life (Coffin–Manson-type) power law for NiTi functional/structural fatigue N_f = N_ref(ε_ref/ε_a)^k',
    'Palmgren–Miner linear damage accumulation D = Σ nᵢ/Nᵢ',
    'Linear-elastic fracture mechanics K = Yσ√(πa); Paris law da/dN = C(ΔK)^m; critical crack a_c from K_Ic',
    'Static rollover: overturning moment of aerodynamic side force about the tadpole tipping axis vs gravitational restoring moment',
    'FMEA risk priority number RPN = S × O × D; fault-tree AND/OR gate algebra with common-cause β-factor',
  ],
  methods: ['FORM by HL-RF iteration (analysis.form) cross-checked with 200 000-sample Monte Carlo', 'Numerical Paris-law integration (log-spaced crack increments)', 'Interval (lower/upper bound) evaluation of the fault tree'],
  assumptions: ['Basic-event probabilities are order-of-magnitude judgements (hypothetical) until component test data exist', 'NiTi fatigue parameters are literature-range estimates', 'Static rollover (no dynamic overshoot) — dynamic margins are lower'],
  ibc: {
    initial: ['Component health, accumulated fatigue damage, initial defect size', 'Residual strength, operating age, environmental exposure', 'Probability distributions only where evidence supports them'],
    boundary: ['Extreme wind & gust events, overloads, road impacts, sudden braking', 'Actuator jamming, cam seizure, failed attachments, loss of propulsion energy', 'Fault combinations: overload during braking, low traction during gust, loss of backup in a manoeuvre, unexpected hub engagement'],
  },
  cvv: {
    calibration: ['Coupon and component fatigue (S–N, ε–N) data', 'Fracture toughness and crack-growth tests (ASTM E399, E647)', 'Impact and structural failure tests'],
    verification: ['FORM vs Monte Carlo P_f agreement', 'Paris integration vs closed form for constant Y', 'RPN arithmetic'],
    validation: ['Proof-load tests of wing root', 'Tilt-table rollover test of the vehicle', 'Field failure records (independently documented)'],
  },
  formats: { import: ['CSV fatigue/test data', 'XLSX FMEA'], export: ['FMEA CSV', 'Reliability JSON', 'Plots PNG/SVG'] },
  params: [
    { key: 'gust', label: 'Design gust speed', unit: 'm/s', type: 'number', min: 5, max: 45, step: 0.5, group: 'Wing-root reliability', default: 20, auto: (c) => { const g = c.upstream('s05')?.gust ?? c.env?.gust; return g ? { value: Math.max(15, +(g * 1.4).toFixed(1)), prov: 'coupled' } : undefined; } },
    { key: 'gustCov', label: 'Gust COV', type: 'range', min: 0.05, max: 0.4, step: 0.01, group: 'Wing-root reliability', default: 0.15 },
    { key: 'CLmax', label: 'Max section force coefficient', type: 'number', min: 0.5, max: 2, step: 0.05, group: 'Wing-root reliability', default: 1.3, auto: (c) => { const o = c.upstream('s07'); return o?.CLmax ? { value: +o.CLmax.toFixed(2), prov: 'coupled' } : undefined; } },
    { key: 'wall', label: 'Spar wall thickness', unit: 'mm', type: 'number', min: 1, max: 20, step: 0.5, group: 'Wing-root reliability', default: 4 },
    { key: 'knock', label: 'Laminate allowable / UD strength', type: 'range', min: 0.2, max: 1, step: 0.05, group: 'Wing-root reliability', default: 0.45, help: 'Accounts for ±45/90 plies, environmental knock-down and damage tolerance (B-basis-type reduction).' },
    { key: 'betaTarget', label: 'Target reliability index β', type: 'number', min: 2, max: 5.5, step: 0.1, group: 'Wing-root reliability', default: 3.8 },
    { key: 'epsA', label: 'NiTi strain amplitude', unit: '%', type: 'number', min: 0.2, max: 4, step: 0.1, group: 'Cartridge fatigue', default: 1.0, auto: (c) => { const o = c.upstream('s15'); return o?.epsAmp ? { value: o.epsAmp, prov: 'coupled' } : { value: 1.0, prov: 'estimated' }; }, help: 'Revision 2 (D6): 1 % amplitude.' },
    { key: 'feather', label: 'Gust load shedding above the trip load (breakaway / passive washout)', type: 'bool', group: 'Rollover envelope', default: true },
    { key: 'washout', label: 'Flexure arch: side-force fraction retained after passive washout', type: 'number', min: 0.05, max: 1, step: 0.05, group: 'Rollover envelope', default: 0.35, help: 'Revision 5: no release mechanism — the flexure stiffening and skin de-cambering shed lift; verify in Suites 06/08.' },
    { key: 'Nref', label: 'NiTi cycles at 3 % reference', type: 'number', min: 1e3, max: 1e6, step: 1e3, group: 'Cartridge fatigue', default: 1e4, help: 'Literature range 10³–10⁵ depending on alloy, processing and test mode.' },
    { key: 'kexp', label: 'NiTi strain–life exponent k', type: 'number', min: 2, max: 8, step: 0.1, group: 'Cartridge fatigue', default: 4.5 },
    { key: 'kmYear', label: 'Distance per year', unit: 'km', type: 'number', min: 100, max: 50000, step: 100, group: 'Usage', default: 5000 },
    { key: 'backupShare', label: 'Share of distance on hub backup', unit: '%', type: 'range', min: 0, max: 100, step: 1, group: 'Usage', default: 20 },
    { key: 'hoursYear', label: 'Operating hours per year', type: 'number', min: 10, max: 4000, step: 10, group: 'Usage', default: 400 },
    { key: 'gustFreq', label: 'Gust load cycles per hour', type: 'number', min: 10, max: 5000, step: 10, group: 'Usage', default: 360 },
    { key: 'dSigIns', label: 'Wing-root insert stress range', unit: 'MPa', type: 'number', min: 5, max: 400, step: 5, group: 'Crack growth (insert)', default: 90 },
    { key: 'a0', label: 'Initial flaw size', unit: 'mm', type: 'number', min: 0.05, max: 5, step: 0.05, group: 'Crack growth (insert)', default: 0.25 },
    { key: 'aDet', label: 'Reliably detectable crack', unit: 'mm', type: 'number', min: 0.2, max: 10, step: 0.1, group: 'Crack growth (insert)', default: 1.0 },
    { key: 'Cs', label: 'Side-force coefficient (worst case)', type: 'number', min: 0.5, max: 2, step: 0.05, group: 'Rollover envelope', default: 1.3 },
  ],
  presets: [{ label: 'Storm gust 30 m/s', values: { gust: 30 } }, { label: 'Heavy backup use', values: { backupShare: 70 } }, { label: 'Thicker spar 8 mm', values: { wall: 8 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const v = ctx.vehicle, rho = ctx.env?.rho || 1.225, spar = ctx.material('spar'), ins = ctx.material('rootInsert');
    // ----- wing-root reliability -----
    const nW = v.nWings || 2, S1 = v.wingArea / nW, span = v.wingHeight, cr = S1 / span * 1.25, b = cr * 0.12, hh = cr * 0.1, t = p.wall / 1000;
    const I = (b * hh ** 3 - (b - 2 * t) * Math.max(0, hh - 2 * t) ** 3) / 12, Z = 2 * I / hh, arm = 4 * span / (3 * Math.PI);
    const Mroot = (U, CL) => 0.5 * rho * U * U * S1 * CL * arm;
    const UD = Math.min(spar.Xt ?? spar.su ?? 500, spar.Xc ?? spar.sy ?? spar.su ?? 500), allow = UD * p.knock, uS = (spar.unc || 10) / 100;
    const sigma = (U, CL) => Mroot(U, CL) / Z / 1e6;
    const g = (x) => x[2] - sigma(x[0], x[1]);
    const vars = [{ mu: p.gust, sigma: p.gust * p.gustCov }, { mu: p.CLmax, sigma: p.CLmax * 0.1 }, { mu: allow, sigma: allow * Math.max(0.05, uS) }];
    const F = form(g, vars);
    const R = rng(2301), Nmc = 200000; let fails = 0; const sS = [];
    for (let i = 0; i < Nmc; i++) { const x = vars.map(vv => R.normal(vv.mu, vv.sigma)); const s = sigma(x[0], x[1]); if (i < 20000) sS.push(s); if (x[2] - s <= 0) fails++; }
    const pfMC = fails / Nmc, sfWing = allow / sigma(p.gust, p.CLmax), up06 = ctx.upstream('s06');
    // ----- NiTi cartridge fatigue -----
    const revKm = 1000 / (2 * Math.PI * v.wheelRadius), cycYr = p.kmYear * p.backupShare / 100 * revKm;
    const Nf = (ea) => p.Nref * (3 / ea) ** p.kexp;
    const spec = [[0.6, p.epsA], [0.3, 0.7 * p.epsA], [0.1, 1.2 * p.epsA]];
    const Dyr = spec.reduce((s, [f, e]) => s + f * cycYr / Nf(e), 0), lifeYr = Dyr > 0 ? 1 / Dyr : Infinity, lifeCycles = Dyr > 0 ? cycYr / Dyr : Infinity, lifeKm = lifeYr * p.kmYear;
    // ----- Paris-law crack growth (Ti insert) -----
    const C = 5e-12, m = 3.2, KIc = ins.cls === 'ti' ? 55 : ins.cls === 'steel' ? 80 : ins.cls === 'al' ? 25 : 40, Y = 1.12, smax = p.dSigIns * 1.1;
    const acK = (KIc / (Y * smax)) ** 2 / Math.PI, ligament = 0.015, ac = Math.min(acK, ligament); // fracture- or ligament-limited (insert half-width ≈ 15 mm)
    const grow = (a1, a2) => { if (a1 >= a2) return 0; const n = 400; let N = 0; const as = linspace(Math.log(a1), Math.log(a2), n + 1).map(Math.exp); for (let i = 0; i < n; i++) { const a = 0.5 * (as[i] + as[i + 1]); N += (as[i + 1] - as[i]) / (C * (Y * p.dSigIns * Math.sqrt(Math.PI * a)) ** m); } return N; };
    const a0 = p.a0 / 1000, aD = p.aDet / 1000, Ntot = grow(a0, ac), Ninsp = grow(Math.min(aD, ac), ac) / 3, cycYrGust = p.hoursYear * p.gustFreq;
    const closed = (2 / ((m - 2) * C * (Y * p.dSigIns * Math.sqrt(Math.PI)) ** m)) * (a0 ** (1 - m / 2) - ac ** (1 - m / 2));
    // ----- rollover envelope (tadpole tipping axis) -----
    const L = v.wheelbase, tr = v.track, s01 = ctx.upstream('s01'), aFront = 0.58 * L;
    const tipArm = (v.layout || 'tadpole') === 'tadpole' ? (tr / 2) * (L - aFront) / Math.hypot(L, tr / 2) : (tr / 2) * aFront / Math.hypot(L, tr / 2);
    const cosPsi = L / Math.hypot(L, tr / 2), hcp = v.wheelRadius + v.height * 0.7 + span / 2, mass = s01?.mass || v.mass;
    const Mrest = mass * G0 * tipArm, UrollLoaded = Math.sqrt(Mrest / (0.5 * rho * v.wingArea * p.Cs * hcp * cosPsi));
    // D3: wings carry load only up to the breakaway side force, then weathervane (C_s ≈ 0.03 incl. residual drag);
    // body side force (side area ≈ 0.75·L·H, C_s 0.8 at mid-height) always acts. Rollover is checked with these loads.
    const mu = ctx.upstream('s10')?.muPeak ?? 0.9, ssf = ctx.upstream('s11')?.ssf ?? s01?.ssf ?? tipArm / v.cgHeight;
    const FyBreak = s01?.washoutCap_N ?? Math.min(0.7 * mu * mass * G0, 0.8 * ssf * mass * G0 * v.cgHeight / hcp), // shared Suite 01 cap
      Ubreak = Math.sqrt(FyBreak / (0.5 * rho * v.wingArea * p.Cs));
    const Abody = 0.75 * v.length * (v.height + v.wheelRadius), hBody = (v.height + v.wheelRadius) / 2;
    // Revision 2.2: deployed and reefed wing states (reefed span telescopes into the lower section)
    const archV = (v.wingConfig || 'tandem') === 'arch', flexV = archV && v.archRoot !== 'ring', spanR = archV ? span : Math.min(span, v.wingReefSpan || span), areaR = flexV ? v.wingArea : archV ? v.wingArea * 0.12 : v.wingArea * spanR / span, hcpR = flexV ? hcp : archV ? 0.9 * (v.wheelRadius + v.height) : v.wheelRadius + v.height * 0.7 + spanR / 2; // arch: second state = folded over the body (parking)
    const washF = (v.wingConfig === 'arch' && v.archRoot !== 'ring') ? (p.washout ?? 0.35) : 0.03 / p.Cs; // flexure arch: passive washout keeps ≈35 % of the side-force coefficient above the trip load
    const rollAt = (area, hc) => { // breakaway calibrated so wing + body overturning moment stays ≤ 0.8 × restoring moment (and within 70 % of grip)
      const UBm = Math.sqrt(0.8 * Mrest / (0.5 * rho * (area * p.Cs * hc + Abody * 0.8 * hBody) * cosPsi)), UBg = Math.sqrt(0.7 * mu * mass * G0 / (0.5 * rho * area * p.Cs)), UB = Math.min(UBm, UBg), FyB = 0.5 * rho * UB * UB * area * p.Cs; const Mo = (U) => { const q = 0.5 * rho * U * U, Fw = p.feather ? (U <= UB ? q * area * p.Cs : q * area * p.Cs * washF) : q * area * p.Cs; return (Fw * hc + q * Abody * 0.8 * hBody) * cosPsi; }; let Ur = 0; for (let U = 0.5; U <= 80; U += 0.1) { if (Mo(U) > Mrest) { Ur = U; break; } } return { Ur: Ur || 80, UB, FyB, Mo }; };
    const RD = rollAt(v.wingArea, hcp), RR = rollAt(areaR, hcpR), Mover = RD.Mo; const UbreakD = RD.UB, FyBreakD = RD.FyB;
    const Uroll = Math.min(RD.Ur, RR.Ur), UrollReefed = RR.Ur, UrollDeployed = RD.Ur;
    // ----- fault tree (interval bounds, per trip) -----
    const be = { brakeA: [1e-5, 1e-4], brakeB: [1e-5, 1e-4], beta: [0.05, 0.1], jam: [1e-4, 1e-3], windHigh: [1e-2, 1e-1], adhesion: [1e-4, 1e-3], highSpeed: [0.05, 0.2] };
    const envSeries = ctx.upstream('s05')?.series; if (envSeries?.length) { const fr = envSeries.filter(d => d.gust > Uroll).length / envSeries.length; be.windHigh = [Math.max(1e-4, fr * 0.5), Math.min(1, Math.max(2e-4, fr * 1.5))]; }
    const G1 = [be.brakeA[0] * be.brakeB[0] + be.beta[0] * be.brakeA[0], be.brakeA[1] * be.brakeB[1] + be.beta[1] * be.brakeA[1]];
    const G2 = [be.jam[0] * be.windHigh[0], be.jam[1] * be.windHigh[1]], G3 = [be.adhesion[0] * be.highSpeed[0], be.adhesion[1] * be.highSpeed[1]];
    const top = [1 - (1 - G1[0]) * (1 - G2[0]) * (1 - G3[0]), 1 - (1 - G1[1]) * (1 - G2[1]) * (1 - G3[1])];
    // ----- safety margin register -----
    const reg = [];
    reg.push(['Wing-root strength (deterministic SF)', sfWing, '≥ 1.5', sfWing >= 1.5, 'this suite']);
    if (up06?.flutterSpeed) reg.push(['Flutter speed / (1.15 × design gust)', up06.flutterSpeed / (1.15 * p.gust), '≥ 1', up06.flutterSpeed >= 1.15 * p.gust, 'Suite 06']);
    const o09 = ctx.upstream('s09'); if (o09?.minSF) reg.push(['Chassis minimum safety factor', o09.minSF, '≥ 1.5', o09.minSF >= 1.5, 'Suite 09']);
    const o11 = ctx.upstream('s11'); if (o11?.rolloverMargin !== undefined) reg.push(['Rollover margin (handling)', o11.rolloverMargin, '≥ 1.2', o11.rolloverMargin >= 1.2, 'Suite 11']);
    const o12 = ctx.upstream('s12'); if (o12?.stoppingDistance_m) reg.push(['Stopping distance', o12.stoppingDistance_m, 'per Suite 12 requirement', o12.parkingHoldOK !== false, 'Suite 12']);
    const o17 = ctx.upstream('s17'); if (o17) reg.push(['Hub drive feasibility', o17.feasible ? 1 : 0, 'feasible', !!o17.feasible, 'Suite 17']);
    reg.push(['NiTi cartridge life', lifeYr, '≥ 2 years between replacements', lifeYr >= 2, 'this suite']);
    reg.push(['Insert crack growth life / service life (20 y)', Ntot / cycYrGust / 20, '≥ 3 (scatter factor)', Ntot / cycYrGust / 20 >= 3, 'this suite']);
    reg.push(['Rollover wind limit (wings loaded)', Uroll, '≥ design gust', Uroll >= p.gust, 'this suite']);
    const sfVals = reg.filter(r => /SF|safety factor/i.test(r[0])).map(r => r[1]).filter(Number.isFinite), minSF = Math.min(sfWing, ...sfVals);
    const reefableV = (ctx.vehicle.wingReefSpan ?? ctx.vehicle.wingHeight) < ctx.vehicle.wingHeight - 1e-6;
    const isArchV = (ctx.vehicle.wingConfig || 'tandem') === 'arch', FMEA_USE = isArchV ? FMEA.filter(r => !/reef|breakaway|cannot be unloaded/i.test(r[1])).concat(ARCH_FMEA) : FMEA;
    const fm = FMEA_USE.filter(r => reefableV || !/reef/i.test(r[1])).map(r => [...r.slice(0, 6), r[3] * r[4] * r[5], r[6]]).sort((a, b) => b[6] - a[6]);
    // KPIs
    const k = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('Wing-root bending stress at design gust', sigma(p.gust, p.CLmax), 'MPa', `M = ½ρU²S·C·4b/3π = ${fmt(Mroot(p.gust, p.CLmax), 4)} N·m on a ${fmt(b * 1000, 3)} × ${fmt(hh * 1000, 3)} × ${p.wall} mm box spar.`, 'calc');
    k('Allowable laminate stress', allow, 'MPa', `${spar.name} UD strength × ${p.knock} knock-down.`, 'library');
    k('Deterministic safety factor', sfWing, '', 'Allowable / applied stress at the design gust and maximum force coefficient.', 'calc', sfWing >= 1.5 ? 'ok' : sfWing >= 1 ? 'warn' : 'bad');
    k('Reliability index β (FORM)', F.beta, '', `Target β ≥ ${p.betaTarget} (≈ P_f 7×10⁻⁵ for 3.8).`, 'calc', F.beta >= p.betaTarget ? 'ok' : F.beta >= 2.5 ? 'warn' : 'bad');
    k('Failure probability (FORM)', F.pf, 'per event', 'Probability that a design-gust event exceeds the root allowable.', 'calc');
    k('Failure probability (Monte Carlo)', pfMC, 'per event', `${Nmc.toLocaleString()} samples — cross-check of FORM.`, 'calc');
    k('NiTi cartridge cycles per year', cycYr, 'cycles', `${fmt(revKm, 3)} revolutions per km × backup distance.`, 'calc');
    k('NiTi cartridge fatigue life', lifeYr, 'years', `Miner damage ${fmt(Dyr, 3)} per year from a 3-level strain spectrum (literature-range ε–N estimate).`, 'estimated', lifeYr >= 2 ? 'ok' : lifeYr >= 0.5 ? 'warn' : 'bad');
    k('Cartridge life in distance', lifeKm, 'km', 'Distance before cumulative damage D = 1.', 'estimated');
    k('Critical crack length (insert)', ac * 1000, 'mm', acK > ligament ? `Ligament-limited (insert half-width 15 mm); fracture-mechanics a_c would be ${fmt(acK * 1000, 3)} mm.` : `a_c from K_Ic = ${KIc} MPa√m at σ_max ${fmt(smax, 3)} MPa.`, 'calc');
    k('Crack-growth life (insert)', Ntot, 'cycles', `Paris law from a₀ = ${p.a0} mm (≈ ${fmt(Ntot / cycYrGust, 3)} years at ${cycYrGust.toLocaleString()} gust cycles/year).`, 'calc');
    k('Inspection interval (insert)', Ninsp / cycYrGust, 'years', `Growth from detectable ${p.aDet} mm to critical ÷ scatter factor 3.`, 'calc');
    if (archV) { // AeroArch doubles as a roll-over protection hoop: two leg box-spars as frame columns plus the bridge
      const Aleg = 2 * t * (b + hh), Ileg = Math.min(b * hh ** 3 - (b - 2 * t) * Math.max(0, hh - 2 * t) ** 3, hh * b ** 3 - (hh - 2 * t) * Math.max(0, b - 2 * t) ** 3) / 12, Esp = (spar.E || 70) * 1e9;
      const Pcr = Math.PI ** 2 * Esp * Ileg / (0.7 * span) ** 2, Pcap = 2 * Math.min(allow * 1e6 * Aleg, Pcr), need = 2 * mass * G0, Mlat = 1.5 * mass * G0 * span, Mcap = 2 * allow * 1e6 * (2 * Ileg / hh);
      k('Arch as roll-over hoop (vertical / lateral capacity ratio)', `${fmt(Pcap / need, 3)} / ${fmt(Mcap / Mlat, 3)}`, '×', `Estimated: two leg box spars (${fmt(b * 1000, 3)} × ${fmt(hh * 1000, 3)} × ${p.wall} mm) as frame columns — vertical crush capacity vs 2 mg and lateral bending vs 1.5 mg at the bridge. Benefit of the closed frame; confirm with Suite 09 and a static ROPS test.`, 'estimated', Pcap >= need && Mcap >= Mlat ? 'ok' : 'warn');
      out.verdicts.push({ test: 'Roll-over protection by the arch (benefit)', status: Pcap >= need && Mcap >= Mlat ? 'pass' : 'warn', detail: `Vertical ${fmt(Pcap / 1000, 3)} kN vs ${fmt(need / 1000, 3)} kN required; lateral ${fmt(Mcap / 1000, 3)} kN·m vs ${fmt(Mlat / 1000, 3)} kN·m. Bulkhead attachment and occupant head clearance must be verified.` });
    }
    // ----- Revision 5.1 part-audit crash & restraint checks -----
    let noseConeKg = null, noseConeLen_m = null;
    { const { vehicleAssembly } = await import('../core/cad.js'); let Lavail = 0.45; try { const asmC = vehicleAssembly(ctx.vehicle), bodyM = asmC.parts.find(m => /Monocoque body/i.test(m.name)), ck = asmC.meta.cockpitX; if (bodyM && ck) Lavail = bodyM.bbox()[3] - Math.max(...ck); } catch { /* default */ }
      const mV = mass, vF = 50 / 3.6, Ef = 0.5 * mV * vF ** 2, rhoHc = 72, sigP = 6.6e6 * (rhoHc / 130) ** 1.5, gTarget = 25, gFit = Ef / (mV * G0 * 0.75 * Lavail), gMax = Math.max(gTarget, gFit), Fm = gMax * G0 * mV, Ahc = Fm / sigP, stroke = Ef / Fm, Lc = stroke / 0.75, mCone = 1.1 * rhoHc * Ahc * Lc;
      k('Frontal crash 50 km/h: mean deceleration / nose-cone mass', `${fmt(gMax, 3)} g / ${fmt(mCone, 3)} kg`, '', `${fmt(Ef / 1000, 3)} kJ. Crush length available ahead of the cockpit (CAD) ${fmt(Lavail * 1000, 3)} mm; target ${gTarget} g needs ${fmt(Ef / (gTarget * G0 * mV) / 0.75 * 1000, 3)} mm. Aluminium honeycomb ${rhoHc} kg/m³ (plateau ${fmt(sigP / 1e6, 3)} MPa): crush section ${fmt(Ahc * 1e4, 3)} cm² (e.g. ${fmt(Math.sqrt(Ahc) * 1000, 3)} mm square), length ${fmt(Lc * 1000, 3)} mm.`, 'calc', gMax <= gTarget && mCone <= 1.65 ? 'ok' : 'bad');
      noseConeKg = mCone; noseConeLen_m = Lc;
      out.verdicts.push({ test: 'Frontal crash 50 km/h ≤ 25 g mean (nose cone)', status: gMax <= gTarget && mCone <= 1.65 ? 'pass' : 'fail', detail: gMax <= gTarget ? `${fmt(Ef / 1000, 3)} kJ at ${gTarget} g mean in a ${fmt(Lc * 1000, 3)} mm × ${fmt(Ahc * 1e4, 3)} cm² honeycomb column (${fmt(mCone, 3)} kg) — fits the ${fmt(Lavail * 1000, 3)} mm ahead of the cockpit; the cone in the CAD must be lengthened to this and its crush section limited to ${fmt(Ahc * 1e4, 3)} cm² (a full-width cone would be far too stiff).` : `Binding length: only ${fmt(Lavail * 1000, 3)} mm is available ahead of the cockpit, so the mean is ${fmt(gMax, 3)} g; 25 g needs ${fmt(Ef / (gTarget * G0 * mV) / 0.75 * 1000, 3)} mm.`, fix: 'Lengthen the crush zone ahead of the footwell or accept a higher mean deceleration with a stiffer survival cell.' });
      const vS = 25 / 3.6, Es = 0.5 * mV * vS ** 2, SEA = 40e3, mSillNeed = Es / (SEA * 0.5), Fs = 40e3, intr = Es / Fs, sillM = 2.2;
      k('Side impact 25 km/h pole: energy / sill mass needed', `${fmt(Es / 1000, 3)} kJ / ${fmt(mSillNeed, 3)} kg (fitted ${sillM} kg)`, '', `Recycled-CFRP crush tubes, SEA ${SEA / 1000} kJ/kg at 50 % efficiency in lateral crush; mean force ${Fs / 1000} kN → ${fmt(intr * 1000, 3)} mm intrusion.`, 'calc', mSillNeed <= sillM && intr <= 0.18 ? 'ok' : 'bad');
      out.verdicts.push({ test: 'Side impact (side sills)', status: mSillNeed <= sillM && intr <= 0.18 ? 'pass' : 'fail', detail: `${fmt(Es / 1000, 3)} kJ; sills need ${fmt(mSillNeed, 3)} kg (fitted ${sillM} kg); intrusion ${fmt(intr * 1000, 3)} mm vs 180 mm sill depth.` });
      const Fseat = 90 * 20 * G0, nIns = 6, insCap = 6000, cap = nIns * insCap;
      k('Seat / harness anchorage (90 kg × 20 g)', `${fmt(Fseat / 1000, 3)} kN vs ${fmt(cap / 1000, 3)} kN (${nIns} inserts × ${insCap / 1000} kN)`, '', 'Six potted Ti inserts in the keel beam (two lap, two shoulder, two crotch/seat); target SF ≥ 2.', 'calc', cap / Fseat >= 2 ? 'ok' : 'bad');
      out.verdicts.push({ test: 'Seat and harness anchorage (SF ≥ 2)', status: cap / Fseat >= 2 ? 'pass' : 'fail', detail: `Margin ${fmt(cap / Fseat, 3)} (${fmt(cap / 1000, 3)} kN capacity vs ${fmt(Fseat / 1000, 3)} kN).` });
      if (flexV) { const fx = ctx.material('flexure') || { sy: 1300 }, nL = 6, tL = 2, hL = 80, Zp = nL * tL * hL ** 3 / 12 / (hL / 2) * 1e-9, Mcap = (fx.sy || 1300) * 1e6 * Zp, Mreq = 2 * 2750, sfP = Mcap / Mreq;
        k('Roll-over load path through the single pylon', `${fmt(Mcap / 1000, 3)} kN·m capacity vs ${fmt(Mreq / 1000, 3)} kN·m (2 × 2.75 per leg)`, '', `${nL} leaves ${tL} × ${hL} mm bending in-plane (stiff axis); yaw remains compliant. Safety factor ${fmt(sfP, 3)}.`, 'calc', sfP >= 2 ? 'ok' : 'bad');
        out.verdicts.push({ test: 'Lateral roll-over load path (pylon)', status: sfP >= 2 ? 'pass' : 'fail', detail: `SF ${fmt(sfP, 3)} for the roll moment of both arch legs through the centreline pylon into the CFRP crossbeam and keel.` }); }
    }
    k(flexV ? 'Rollover wind — arch (passive washout), driving / parked' : archV ? 'Rollover wind — arch up / folded' : 'Rollover wind — deployed / reefed', `${fmt(UrollDeployed, 3)} / ${fmt(UrollReefed, 3)}`, 'm/s', flexV ? `Flexure-root arch (no fold, no release mechanism): loaded up to the trip load, then passive washout retains ${fmt(washF * 100, 3)} % of the side-force coefficient; the parked car keeps the arch up, so both states are the same.` : archV ? `Arch up (${fmt(v.wingArea, 3)} m², ring-track breakaway feathering${p.feather ? '' : ' disabled'}) vs folded over the body for parking (exposed ≈ ${fmt(areaR, 3)} m²).` : `Deployed span ${fmt(span, 3)} m (${fmt(v.wingArea, 3)} m²) vs reefed ${fmt(spanR, 3)} m (${fmt(areaR, 3)} m²), both with breakaway feathering${p.feather ? '' : ' disabled'}.`, 'calc', Math.min(UrollDeployed, UrollReefed) >= p.gust ? 'ok' : 'bad');
    k(flexV ? 'Passive washout trip wind (flexure stiffening)' : 'Wing breakaway (feathering) wind', p.feather ? UbreakD : 'washout disabled in this run', p.feather ? 'm/s' : '', `${flexV ? 'Revision 5: no release mechanism — flexure stiffening and skin de-cambering begin shedding lift at this load (target set like the former breakaway). ' : ''}D3 torque-limited breakaway calibrated so wing + body overturning moment ≤ 0.8 × restoring moment and side force ≤ 0.7 μmg: F_y = ${fmt(FyBreakD, 4)} N.`, 'calc');
    k('Rollover apparent-wind limit', Uroll, 'm/s', `${p.feather ? 'Wings feathered above breakaway; ' : ''}Wings loaded at C_s = ${p.Cs}; tipping arm ${fmt(tipArm, 3)} m, centre of pressure ${fmt(hcp, 3)} m.`, 'calc', Uroll >= p.gust ? 'ok' : Uroll >= 0.6 * p.gust ? 'warn' : 'bad');
    k('Highest RPN', fm[0][6], '', `${fm[0][1]} (${fm[0][0]}).`, 'hypothetical', fm[0][6] > 200 ? 'bad' : 'warn');
    k('“Unable to stop safely” per trip (bounds)', `${top[0].toExponential(1)} … ${top[1].toExponential(1)}`, '', 'Order-of-magnitude interval from judgement-based basic events — NOT a validated probability.', 'hypothetical');
    // verdicts
    out.verdicts.push({ test: 'Wing-root structural reliability', status: F.beta >= p.betaTarget ? 'pass' : F.beta >= 2.5 ? 'warn' : 'fail', detail: `β = ${fmt(F.beta, 3)} (target ${p.betaTarget}); deterministic SF ${fmt(sfWing, 3)}.`, fix: 'Increase spar wall/depth, use higher-strength laminate, or rely on validated gust-load alleviation (Suite 08) to cut the root moment.' });
    out.verdicts.push({ test: 'Crosswind rollover envelope', status: Uroll >= p.gust ? 'pass' : 'fail', detail: p.feather ? `Wings loaded up to ${flexV ? 'the washout trip' : 'breakaway'} at ${fmt(UbreakD, 3)} m/s (side force ${fmt(FyBreakD, 4)} N), ${flexV ? `passive washout (${fmt(washF * 100, 3)} % of C_s) above` : 'feathered above'}: static tipping only at ≈ ${fmt(Uroll, 3)} m/s vs ${p.gust} m/s design gust (wings permanently loaded would tip at ${fmt(UrollLoaded, 3)} m/s).` : `Static tipping at ≈ ${fmt(Uroll, 3)} m/s apparent wind with wings loaded, ${Uroll >= p.gust ? 'above' : 'below'} the ${p.gust} m/s design gust — the wings MUST be unloaded/feathered above ≈ ${fmt(Uroll / 1.25, 3)} m/s.`, fix: 'Widen the track, lower the centre of pressure (shorter wings), add mass low, or adopt automatic passive wing unloading with a validated threshold.' });
    out.verdicts.push({ test: 'Hub cartridge durability', status: lifeYr >= 2 ? 'pass' : lifeYr >= 0.5 ? 'warn' : 'fail', detail: `${fmt(lifeYr, 3)} years (${fmt(lifeKm, 3)} km) at ${p.epsA} % strain amplitude.`, fix: 'Reduce strain amplitude below ~1 %, use fatigue-optimised NiTiCu, or design cartridges as scheduled replaceables.' });
    out.verdicts.push({ test: 'Insert damage tolerance', status: Ninsp / cycYrGust >= 1 ? 'pass' : 'warn', detail: `Inspect every ${fmt(Ninsp / cycYrGust, 3)} years; total growth life ${fmt(Ntot / cycYrGust, 3)} years.` });
    out.verdicts.push({ test: 'Quantified probability claims', status: 'info', detail: 'Fault-tree and FMEA occurrence values are judgement-based; this suite deliberately reports intervals, not precise probabilities, until component reliability data are imported.' });
    if (Math.abs(F.pf - pfMC) > Math.max(5e-4, 0.5 * pfMC) && pfMC > 50 / Nmc) out.verdicts.push({ test: 'FORM vs Monte Carlo agreement', status: 'warn', detail: `FORM ${F.pf.toExponential(2)} vs MC ${pfMC.toExponential(2)} — nonlinear limit state; trust MC.` });
    // checks
    out.checks.push({ name: 'Paris integration vs closed form (constant Y)', value: Ntot / closed, target: '1.000 ± 0.01', pass: Math.abs(Ntot / closed - 1) < 0.01 });
    out.checks.push({ name: 'FORM P_f vs MC P_f', value: pfMC > 0 ? F.pf / pfMC : NaN, target: '≈ 1 when P_f resolvable', pass: pfMC < 50 / Nmc || Math.abs(F.pf / pfMC - 1) < 0.5, detail: pfMC < 50 / Nmc ? 'P_f below MC resolution — FORM governs' : '' });
    out.checks.push({ name: 'RPN arithmetic', value: fm[0][6], target: `${fm[0][3]}×${fm[0][4]}×${fm[0][5]}`, pass: fm[0][6] === fm[0][3] * fm[0][4] * fm[0][5] });
    // charts
    const Us = linspace(0, 30, 61);
    out.charts.push({ title: 'Crosswind rollover envelope (static)', type: 'line', x: { label: 'Apparent wind', unit: 'm/s' }, y: { label: 'Moment about tipping axis', unit: 'N·m' }, series: [0.6, 1.0, p.Cs].map(c => ({ name: `Overturning, C_s=${c}`, x: Us, y: Us.map(U => 0.5 * rho * U * U * v.wingArea * c * hcp * cosPsi) })), annotations: [{ type: 'hline', y: Mrest, label: 'gravity restoring moment', color: '#22c55e' }, { type: 'vline', x: Uroll, label: 'tip', color: '#ef4444' }], explain: 'Aerodynamic side force on the tall wings acting at the centre of pressure tips the three-wheeler about the line joining the outer front wheel and the rear wheel. Where the curves cross the green line the vehicle overturns — this sets the wind-loading envelope.' });
    const hs = histogram(sS, 40);
    out.charts.push({ title: 'Wing-root stress distribution vs allowable', type: 'line', x: { label: 'Root stress', unit: 'MPa' }, y: { label: 'Samples' }, series: [{ name: 'Applied stress (MC)', x: hs.centers, y: hs.counts, area: true }], annotations: [{ type: 'vline', x: allow, label: 'mean allowable', color: '#ef4444' }], explain: 'Uncertain gust and force coefficient produce a stress distribution; its overlap with the (also uncertain) allowable is the failure probability.' });
    const gs = linspace(8, 40, 33), betas = gs.map(G => form(g, [{ mu: G, sigma: G * p.gustCov }, vars[1], vars[2]]).beta);
    out.charts.push({ title: 'Reliability index vs design gust', type: 'line', x: { label: 'Design gust', unit: 'm/s' }, y: { label: 'β' }, series: [{ name: 'β (FORM)', x: gs, y: betas }], annotations: [{ type: 'hline', y: p.betaTarget, label: 'target', color: '#22c55e' }], explain: 'How quickly structural reliability erodes in stronger winds — the crossing with the target defines the maximum certified gust.' });
    const ea = linspace(0.3, 4, 40);
    out.charts.push({ title: 'NiTi strain–life curve and operating spectrum', type: 'line', x: { label: 'Cycles to failure', log: true }, y: { label: 'Strain amplitude', unit: '%', log: true }, series: [{ name: 'ε–N estimate', x: ea.map(Nf), y: ea }, { name: 'operating levels', x: spec.map(s => Nf(s[1])), y: spec.map(s => s[1]), mode: 'markers', r: 6 }], explain: 'Literature-range strain–life estimate (labelled estimate). Each cartridge cycles once per wheel revolution in backup mode, so even modest distances accumulate millions of cycles.' });
    const yrs = linspace(0, Math.min(20, Math.max(2, lifeYr * 1.5)), 50);
    out.charts.push({ title: 'Miner damage accumulation', type: 'line', x: { label: 'Years in service', unit: 'y' }, y: { label: 'Damage D' }, series: [{ name: 'D(t)', x: yrs, y: yrs.map(y => y * Dyr) }], annotations: [{ type: 'hline', y: 1, label: 'failure (D = 1)', color: '#ef4444' }], explain: 'Linear damage accumulation; cartridge replacement must be scheduled before D reaches 1 (with a safety factor).' });
    const aa = linspace(Math.log(a0), Math.log(ac), 60).map(Math.exp);
    out.charts.push({ title: 'Insert crack growth (Paris law)', type: 'line', x: { label: 'Gust load cycles', log: true }, y: { label: 'Crack length', unit: 'mm' }, series: [{ name: 'a(N)', x: aa.map(a => Math.max(1, grow(a0, a))), y: aa.map(a => a * 1000) }], annotations: [{ type: 'hline', y: p.aDet, label: 'detectable', color: '#f59e0b' }, { type: 'hline', y: ac * 1000, label: 'critical', color: '#ef4444' }], explain: 'Crack growth from the initial flaw; the interval between detectable and critical size defines the inspection programme.' });
    out.charts.push({ title: 'FMEA risk priority numbers', type: 'bar', horizontal: true, categories: fm.map(r => r[1]), y: { label: 'RPN' }, series: [{ name: 'RPN', values: fm.map(r => r[6]), colors: fm.map(r => r[6] > 200 ? '#ef4444' : r[6] > 120 ? '#f59e0b' : '#3b82f6') }], height: 620, wide: true, explain: 'Severity × occurrence × detection. Occurrence/detection scores are expert judgement (hypothetical) to be updated with test evidence.' });
    // tables
    out.tables.push({ title: 'FMEA', cols: ['Subsystem', 'Failure mode', 'Effect', 'S', 'O', 'D', 'RPN', 'Mitigation / detection'], rows: fm });
    out.tables.push({ title: 'Fault tree — “Unable to stop safely” (per trip, interval bounds)', cols: ['Gate / event', 'Logic', 'Lower', 'Upper', 'Basis'], rows: [
      ['TOP: unable to stop safely', 'OR(G1,G2,G3)', top[0], top[1], 'Derived'], ['G1 both brake circuits fail', 'AND + common cause β', G1[0], G1[1], 'Generic mechanical demand failure (judgement)'],
      ['G2 wing not unloadable in high wind', 'AND', G2[0], G2[1], envSeries ? 'Wind exceedance from Suite 05 series' : 'Judgement'], ['G3 adhesion loss at speed', 'AND', G3[0], G3[1], 'Judgement'],
      ...Object.entries(be).map(([n, [lo, hi]]) => [n, 'basic event', lo, hi, 'hypothetical']),
    ], explain: 'Bounds only — the specification forbids converting unsupported assumed reliability into precise probabilities.' });
    out.tables.push({ title: 'Safety-margin register (all suites)', cols: ['Margin', 'Value', 'Criterion', 'Status', 'Source'], rows: reg.map(r => [r[0], r[1], r[2], r[3] ? '✔' : '✖', r[4]]), explain: 'Run Suites 06, 09, 11, 12 and 17 to populate their margins.' });
    out.tables.push({ title: 'Operating restrictions (derived)', cols: ['Restriction', 'Value'], rows: [
      ['Max apparent wind with wings loaded', `${fmt(Uroll / 1.25, 3)} m/s (static tip ÷ 1.25)`], ['Design gust for structure', `${p.gust} m/s (β ${fmt(F.beta, 3)})`],
      ['Hub cartridge replacement', `every ${fmt(Math.min(lifeYr / 2, 5), 3)} years or ${fmt(lifeKm / 2, 3)} km`], ['Insert NDT inspection', `every ${fmt(Math.min(Ninsp / cycYrGust, 5), 3)} years`],
      ['Parking', 'Wings unloaded and secured; parking brake rated for maximum hub torque'], ['Occupant testing', 'Only after Gate 4 structural, stability and stopping tests are passed (see Dossier)'],
    ] });
    // recs
    if (Uroll < p.gust) out.recs.push({ level: 'critical', text: `Rollover is the governing hazard: tipping at ≈${fmt(Uroll, 3)} m/s apparent wind. Make wing unloading mandatory above ${fmt(Uroll / 1.25, 3)} m/s and consider a wider track (tipping arm grows with track) or lower aspect-ratio wings.`, basis: 'Static rollover envelope' });
    if (F.beta < p.betaTarget) out.recs.push({ level: 'warn', text: `Increase spar section: wall ${p.wall} mm gives β ${fmt(F.beta, 3)}. Re-run with thicker walls or higher-strength laminate until β ≥ ${p.betaTarget}.`, basis: 'FORM' });
    if (sfWing > 4) out.recs.push({ level: 'info', text: `The spar is strength-oversized (SF ${fmt(sfWing, 3)}): stiffness, flutter and buckling — not strength — will set its size. Use Suites 06/08 to size the wall for aeroelastic margins and save mass.`, basis: 'Deterministic SF' });
    if (lifeYr < 2) out.recs.push({ level: 'warn', text: `NiTi cartridges reach D = 1 in ${fmt(lifeYr, 3)} years — design them as quick-change cartridges and lower the strain amplitude; fatigue-life testing per Experiment A is mandatory.`, basis: 'Miner damage' });
    // Revision 6: the specification forbids electricity, so there are no electric lamps — conspicuity is passive only
    out.verdicts.push({ test: 'Night / low-visibility road use (legal limitation)', status: 'warn', detail: 'No lamps of any kind (specification: no electricity, no combustion). Passive conspicuity only: white front, red rear and amber side retroreflectors, retroreflective and photoluminescent tape on the body, arch and spats, two mirrors, high-visibility flag — seen only when another road user\'s lamps illuminate them. Legal limitation: public-road operation in daylight and good visibility only. Night and low-visibility scenarios stay active in the simulations for closed-course and research use.', fix: 'Restrict public-road use to daylight and good visibility.' });
    out.recs.push({ level: 'critical', text: 'Legal limitation: daylight and good-visibility public-road operation only. Passive retroreflectors and tape do not replace head, tail and stop lamps, and the specification excludes every lamp type (electric or combustion). Night and fog scenarios remain available for closed-course and research use.', basis: 'Specification (no electricity, no combustion) vs road-lighting requirements' });
    out.recs.push({ level: 'good', text: 'Adopt a recognised functional-safety style process (hazard log, safety requirements, verification evidence) and a staged test programme: unoccupied first, then an independently approved occupied programme.', basis: 'Specification Module 23 & Part B §30' });
    out.recs.push({ level: 'sustain', text: 'Design cartridges, skins and pads for replacement and recycling (NiTi and titanium have high scrap value); durability is part of sustainability — a short-lived light part can be worse over the life cycle.', basis: 'Durability–sustainability coupling' });
    out.explain.push({ title: 'Deterministic vs probabilistic', text: 'The safety factor compares nominal stress to an allowable; the reliability index β measures how many standard deviations separate the mean margin from failure, accounting for uncertain wind, aerodynamic coefficient and strength. Both are reported because a comfortable SF can hide a large scatter.' });
    out.views.push({ label: '2D animation: crack growth & fatigue damage', icon: '🎞', render: (el) => anim2dDamage(el, { grow, a0, ac, aD, Ntot, cycYrGust, Dyr, lifeYr, epsA: p.epsA }) });
    out.views.push({ label: '3D risk map & failure scenarios', icon: '🛡', render: (el) => view3dRisk(el, { fm, sigma, allow, CLmax: p.CLmax, gust: p.gust, Uroll, sfWing, ctx }) });
    out.outputs = { noseConeMass_kg: noseConeKg, noseConeLength_m: noseConeLen_m, minSafetyFactor: minSF, pfWingRoot: F.pf, betaWingRoot: F.beta, fatigueLife_cycles: lifeCycles, cartridgeLife_years: lifeYr, rolloverWind_ms: Uroll, topRisks: fm.slice(0, 5).map(r => ({ mode: r[1], rpn: r[6] })) };
    out.headline = `Governing hazard: crosswind rollover at ≈${fmt(Uroll, 3)} m/s apparent wind. Wing root β = ${fmt(F.beta, 3)}, NiTi cartridges ≈ ${fmt(lifeYr, 3)} years, insert inspection every ${fmt(Ninsp / cycYrGust, 3)} years.`;
    out.headlineKind = Uroll < p.gust || F.beta < p.betaTarget ? 'warn' : 'ok';
    return out;
  },
};
void range; void mean;

// 3D vehicle coloured by FMEA risk (max RPN per component) with failure-scenario playback driven by the solved
// models of this suite: wing-root stress σ(U) from the gust load model against the allowable, and the solved static
// rollover wind speed. Motion after tipping onset / skid kinematics are illustrative (dynamics in Suites 11–12).
const RISK_MAP = { 'Arch / flexure pylon': ['rootInsert', 'flexure'], 'Arch / camber skin': ['wingSkin'], 'Arch / washout': ['wingSkin', 'spar'], 'Arch / structure': ['spar', 'wingSkin'], 'Wing / aeroelastic': ['wingSkin', 'spar', 'rootInsert', 'leadingEdge'], 'Vehicle dynamics': ['tyre', 'wheel'], Brakes: ['brake'], Steering: ['wheel'], 'Hub drive': ['actuator'], 'Passive switching': ['actuator'], 'Energy buffer': ['actuator'], Structure: ['chassis', 'rootInsert'], Thermal: ['collector', 'radiator'] };
async function view3dRisk(el, { fm, sigma, allow, CLmax, gust, Uroll, sfWing, ctx }) {
  const [{ viewer }, { playback, hud, legend3d }, { vehicleAssembly }, { cmap }, { h }, mats] = await Promise.all([import('../core/viz3d.js'), import('../core/sim3d.js'), import('../core/cad.js'), import('../core/chart.js'), import('../core/ui.js'), import('../core/materials.js')]);
  const rpn = {}; for (const r of fm) for (const c of RISK_MAP[r[0]] || []) rpn[c] = Math.max(rpn[c] || 0, r[6]);
  const top = {}; for (const r of fm) for (const c of RISK_MAP[r[0]] || []) if (!top[c]) top[c] = r[1];
  let mode = 'risk';
  const sel = h('select', { onchange: (e) => { mode = e.target.value; ctl.set(0); } }, h('option', { value: 'risk' }, 'Risk map (max RPN per component)'), h('option', { value: 'gust' }, 'Scenario: gust overload → wing-root stress → rollover'), h('option', { value: 'seize' }, 'Scenario: hub seizure → wheel lock skid'));
  el.append(h('p', { class: 'small muted' }, `Risk colours = highest FMEA RPN affecting each component (hover a part for its top failure mode). Gust scenario: wind ramps to ${fmt(1.3 * Math.max(gust, Uroll), 3)} m/s; wing colour = solved root stress / allowable (${fmt(allow, 4)} MPa) at C_L,max ${CLmax}; tipping begins at the solved static rollover speed ${fmt(Uroll, 3)} m/s (post-tip motion illustrative). Seizure scenario: one locked wheel at 20 km/h decelerating at ≈ μg/3 (kinematic illustration — see Suites 11/12 for solved dynamics).`), sel);
  const wrap = h('div'); el.append(wrap);
  const V = await viewer(wrap, { height: 460, grid: true }); const THREE = V.THREE;
  const matMap = Object.fromEntries(Object.keys(mats.COMPONENTS).map(k => [k, ctx.material(k)]));
  const asm = vehicleAssembly(ctx.vehicle, { materials: matMap });
  const objs = asm.parts.map(m => { m.materialName = rpn[m.component] ? `max RPN ${rpn[m.component]} — ${top[m.component]}` : 'no FMEA entry'; return { comp: m.component, group: m.group, o: V.add(m) }; });
  const pivot = new THREE.Group(); V.root.add(pivot); for (const x of objs) pivot.add(x.o);
  V.viewDir(0.9, -1.2, 0.6);
  let spin = 0; V.animate = (dt) => { if (mode === 'risk') { spin += dt * 0.25; V.root.rotation.z = spin; } else V.root.rotation.z = 0; };
  const H = hud(wrap); const rv = Object.values(rpn), rLo = Math.min(...rv) * 0.8, rHi = Math.max(...rv); const leg = legend3d(wrap, { title: 'Risk priority number', lo: rLo, hi: rHi, cmapName: 'turbo' });
  const paint = (o, v, name) => { const c = cmap(v, name); o.material.color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255); };
  const yE = -ctx.vehicle.track / 2, Umax = 1.3 * Math.max(gust, Uroll);
  const ctl = playback(wrap, { duration: 12, label: 'Scenario time', unit: 's', onFrame: (t) => {
    pivot.rotation.set(0, 0, 0); pivot.position.set(0, 0, 0);
    if (mode === 'risk') { leg.set(rLo, rHi); for (const x of objs) paint(x.o, rpn[x.comp] ? (rpn[x.comp] - rLo) / ((rHi - rLo) || 1) : 0, 'turbo'); const r0 = fm[0]; H.set([['Highest RPN', `${r0[6]} — ${r0[1]}`, '#ef4444'], ['Second', `${fm[1][6]} — ${fm[1][1]}`], ['Third', `${fm[2][6]} — ${fm[2][1]}`], ['Wing-root SF', fmt(sfWing, 3)], ['Rollover wind', fmt(Uroll, 3) + ' m/s']]); return; }
    if (mode === 'gust') {
      const U = Umax * 0.5 * (1 - Math.cos(Math.PI * Math.min(t, 10) / 10)), ratio = sigma(U, CLmax) / allow;
      leg.set(0, 1.5); for (const x of objs) paint(x.o, ['wingSkin', 'spar', 'rootInsert'].includes(x.comp) ? Math.min(1, ratio / 1.5) : 0.05, 'inferno');
      const th = U > Uroll ? Math.min(28, 4 * (U - Uroll) ** 1.5) * Math.PI / 180 : 0; pivot.rotation.x = -th; pivot.position.set(0, yE - yE * Math.cos(th), yE * Math.sin(th));
      H.set([['Wind (apparent)', fmt(U, 3) + ' m/s'], ['Root stress / allowable', fmt(ratio, 3), ratio >= 1 ? '#ef4444' : ratio > 0.67 ? '#f59e0b' : '#22c55e'], ['Root failure', ratio >= 1 ? 'YES — exceeds allowable' : 'no'], ['Rollover', U > Uroll ? 'TIPPING (beyond solved static limit)' : `margin ${fmt(Uroll - U, 3)} m/s`, U > Uroll ? '#ef4444' : '#22c55e'], ['Mitigation', 'unload/feather wings above ' + fmt(Uroll / 1.25, 3) + ' m/s']]);
      return;
    }
    const v0 = 20 / 3.6, a = (ctx.upstream?.('s10')?.muPeak ?? 0.8) * 9.81 / 3, ts = v0 / a, tt = Math.min(t, ts), x = v0 * tt - 0.5 * a * tt * tt, v = Math.max(0, v0 - a * t), yaw = Math.min(25, 30 * x / (v0 * ts / 2 || 1)) * Math.PI / 180 * (t < ts ? tt / ts : 1);
    leg.set(0, 1); for (const x2 of objs) paint(x2.o, x2.comp === 'actuator' ? 1 : x2.comp === 'tyre' ? 0.6 : 0.08, 'inferno');
    pivot.position.set(x, 0, 0); pivot.rotation.z = yaw;
    H.set([['Event', 'Cam seizure → wheel locked'], ['Speed', fmt(v * 3.6, 3) + ' km/h'], ['Skid distance', fmt(x, 3) + ' m'], ['Yaw (illustrative)', fmt(yaw * 180 / Math.PI, 3) + '°'], ['Mitigation', 'torque-limiting shoe + disengagement']]);
  } });
}

// 2D animation: Paris-law crack growth a(N) of the wing-root insert (solved by integration) with the crack drawn in a
// schematic section, and Miner fatigue damage accumulation D(t) of the NiTi cartridges at the design strain amplitude.
async function anim2dDamage(el, { grow, a0, ac, aD, Ntot, cycYrGust, Dyr, lifeYr, epsA }) {
  const [{ anim2d }, { h }] = await Promise.all([import('../core/sim3d.js'), import('../core/ui.js')]);
  const as = linspace(Math.log(a0), Math.log(ac * 0.999), 120).map(Math.exp), Ns = as.map(a => grow(a0, a));
  const yrs = Ns.map(n => n / Math.max(1, cycYrGust)), Y = yrs[yrs.length - 1] || 1, tY = Math.max(Y, Number.isFinite(lifeYr) ? lifeYr * 1.3 : 0, 1);
  el.append(h('p', { class: 'small muted' }, `Left: solved Paris-law growth from a₀ = ${fmt(a0 * 1000, 3)} mm to the critical size ${fmt(ac * 1000, 3)} mm (${fmt(Ntot, 3)} gust cycles ≈ ${fmt(Y, 3)} years); detection threshold ${fmt(aD * 1000, 3)} mm. Right: Miner damage of NiTi cartridges at ${epsA} % strain amplitude (D = 1 at ${fmt(lifeYr, 3)} years → replacement).`));
  anim2d(el, { height: 420, duration: tY, rate: tY / 8, label: 'Service time', unit: 'years', onFrame: (t, g) => {
    g.clear(); let j = yrs.findIndex(y => y >= t); if (j < 0) j = yrs.length - 1; const a = as[j] * 1000;
    const A = g.panel(0, 0, g.W * 0.45, g.H * 0.62, { xr: [0, Y * 1.05], yr: [0, ac * 1000 * 1.1], xlabel: 'Years (gust cycles / year)', ylabel: 'Crack length a [mm]', title: `Insert crack a = ${fmt(a, 3)} mm` });
    A.line(yrs, as.map(v => v * 1000), '#ef4444', 2); A.hline(aD * 1000, '#f59e0b'); A.hline(ac * 1000, '#ef4444'); A.vline(Math.min(t, Y)); A.point(yrs[j], a);
    // schematic section with the crack
    const sx = 40, sy = g.H * 0.7, sw = g.W * 0.38, sh = g.H * 0.22; g.ctx.fillStyle = 'rgba(148,163,184,.35)'; g.ctx.fillRect(sx, sy, sw, sh); g.ctx.strokeStyle = g.fg; g.ctx.strokeRect(sx, sy, sw, sh);
    g.ctx.strokeStyle = '#ef4444'; g.ctx.lineWidth = 3; g.ctx.beginPath(); g.ctx.moveTo(sx, sy + sh / 2); g.ctx.lineTo(sx + sw * Math.min(1, a / (ac * 1000)), sy + sh / 2 + 3 * Math.sin(t * 9)); g.ctx.stroke(); g.ctx.lineWidth = 1;
    g.text(t >= Y ? 'CRITICAL — insert pull-out' : a >= aD * 1000 ? 'Detectable at inspection' : 'Below detection threshold', sx, sy - 6, { size: 11, color: t >= Y ? '#ef4444' : a >= aD * 1000 ? '#f59e0b' : '#22c55e' });
    const D = Dyr * t, B = g.panel(g.W * 0.5, 0, g.W * 0.5, g.H, { xr: [0, tY], yr: [0, Math.max(1.2, Dyr * tY * 1.05)], xlabel: 'Service years', ylabel: 'Miner damage D', title: `NiTi cartridges: D = ${fmt(D, 3)}${D >= 1 ? ' — replace' : ''}` });
    B.line([0, tY], [0, Dyr * tY], '#a855f7', 2); B.hline(1, '#ef4444'); B.vline(t); B.point(t, D, D >= 1 ? '#ef4444' : '#22c55e');
    // service-time progress bar (elapsed share of the displayed life) so the evolution is visible at a glance
    const fr = Math.min(1, t / tY), by = g.H - 14, bw = g.W - 80; g.ctx.fillStyle = 'rgba(148,163,184,.25)'; g.ctx.fillRect(40, by, bw, 8); g.ctx.fillStyle = D >= 1 ? '#ef4444' : '#22c55e'; g.ctx.fillRect(40, by, bw * fr, 8);
    g.text(`${fmt(t, 3)} of ${fmt(tY, 3)} service years`, 40, by - 4, { size: 10 });
  } });
}
