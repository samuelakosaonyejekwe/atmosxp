// Suite 25 — Sensitivity, uncertainty quantification, surrogate modelling & multidisciplinary optimisation.
import { h, table } from '../core/ui.js';
import { fmt, linspace, range, mean, std, clamp, bisect, lhs, rng, scaleSample, histogram, metrics, G0 } from '../core/num.js';
import { morris, sobol, tornado, nsga2, gpFit, hypervolume2, propagate } from '../core/analysis.js';

// Reduced whole-vehicle model (physics-based, millisecond evaluation) used by every study in this suite.
const NITI = { rho: 6450, c: 837, k: 18, Lh: 19000, swing: 15 };
function model(x, c) {
  const span = c.span0 * Math.sqrt(x.S / c.S0), AR = span ** 2 / (x.S / c.nW), a = x.alpha * Math.PI / 180;
  const CL = Math.min(1.3, 2 * Math.PI * a * AR / (AR + 2)), CD = 0.025 + CL ** 2 / (Math.PI * 0.8 * AR);
  const mExtra = 2.5 * x.S + 6 * x.Ac + 2 * x.nCart * x.mc + 20, mTot = x.m + mExtra;
  const road = (v) => mTot * G0 * x.Crr + 0.5 * c.rho * x.CdA * v * v;
  const hcp0 = c.R + c.h * 0.7 + span / 2, Mrest = mTot * G0 * c.tipArm;
  // wing force at speed v with mandatory depowering: the side force may not exceed the rollover allowance (÷1.25);
  // tyre side-force drag F_y²/C_α (C_α total cornering stiffness) is charged to propulsion.
  const wingAt = (v) => { const Va2 = x.U ** 2 + v * v, b = Math.atan2(x.U, v), q = 0.5 * c.rho * Va2; const CLsafe = (Mrest / 1.25 / (q * x.S * hcp0 * c.cosPsi) - CD * Math.sin(b)) / Math.max(1e-6, Math.cos(b)); const CLe = Math.max(0, Math.min(CL, CLsafe)); const Fy = q * x.S * (CLe * Math.cos(b) + CD * Math.sin(b)); return { T: q * x.S * (CLe * Math.sin(b) - CD * Math.cos(b)) - Fy * Fy / c.Calpha, Fy, depowered: CLe < CL - 1e-9 }; };
  const thrust = (v) => wingAt(v).T;
  const f = (v) => thrust(v) - road(v);
  let vW = 0; if (f(0.01) > 0) { let hi = 0.01; while (hi < 40 && f(hi) > 0) hi += 0.5; vW = hi >= 40 ? 40 : bisect(f, Math.max(0.01, hi - 0.5), hi, 1e-6).x; }
  // backup at 20 km/h (or the target speed)
  const v = c.vTarget, Preq = road(v) * v, T0 = 293.15, etaC = x.dT / (T0 + x.dT), etaII = 0.5;
  const Pex = x.Ac * x.q * etaC * etaII;                                   // exergy-limited (finite-time, second-law fraction)
  const half = x.t / 2000, tau = NITI.rho * NITI.c * half * half * 4 / (Math.PI ** 2 * NITI.k) + NITI.rho * NITI.c * half / c.hContact;
  const fTh = 1 / (2 * tau), fWh = v / (2 * Math.PI * c.R), fOp = Math.min(fTh, fWh);
  const mAct = 2 * x.nCart * x.mc, Pact = mAct * x.w * 1000 * fOp * c.mechEff;     // actuator capacity
  const qCycle = NITI.Lh + NITI.c * NITI.swing, etaAct = x.w * 1000 / qCycle, Pthr = x.Ac * x.q * etaAct * c.mechEff; // heat-throughput limit
  const Pb = Math.max(0, Math.min(Pex, Pact, Pthr));
  const hcp = hcp0, Uroll = Math.sqrt(mTot * G0 * c.tipArm / (0.5 * c.rho * x.S * 1.3 * hcp * c.cosPsi));
  const sigRoot = c.sig0 * (x.S / c.S0) * (span / c.span0) * CL / 1.0;
  const wv = wingAt(vW);
  return { vWind_kmh: vW * 3.6, depowered: wv.depowered, Fy: wv.Fy, Pb, Preq, closure: 100 * Pb / Preq, mTot, Uroll, sigRoot, limit: Pb === Pex ? 'exergy' : Pb === Pact ? (fTh < fWh ? 'thermal cycling rate' : 'actuator work') : 'heat throughput', Pex, Pact, Pthr, fTh, fWh, CL, CD, etaC };
}
const VARS = [
  ['S', 'Wing area', 'm²', 4, 14], ['alpha', 'Wing incidence (effective AoA)', '°', 3, 16], ['m', 'Base vehicle mass', 'kg', 150, 320], ['Crr', 'Rolling resistance Crr', '', 0.003, 0.012], ['CdA', 'Drag area CdA', 'm²', 0.15, 0.45],
  ['nCart', 'Cartridges per wheel', '', 8, 64], ['mc', 'Active NiTi per cartridge', 'kg', 0.01, 0.2], ['Ac', 'Collector area', 'm²', 1, 8], ['dT', 'Effective hot–cold ΔT', 'K', 2, 40],
  ['q', 'Environmental heat flux', 'W/m²', 10, 700], ['w', 'Net specific work per cycle', 'J/g', 0.05, 5], ['t', 'Actuator element thickness', 'mm', 0.05, 2], ['U', 'True wind (beam)', 'm/s', 1, 12],
];

export default {
  description: 'Local and global sensitivity (tornado, Morris, Sobol), uncertainty propagation by Latin hypercube, Gaussian-process surrogate with independent validation, NSGA-II multi-objective optimisation with rollover, stress and exergy constraints, and inverse design of the breakthrough needed to close the atmospheric power gap.',
  literature: 'global sensitivity analysis Sobol multi-objective optimization NSGA-II surrogate model vehicle design shape memory heat engine',
  credibility: 'Analytical screening',
  equations: [
    'Reduced whole-vehicle model: lifting-line C_L = 2πα·AR/(AR+2), induced drag C_L²/πeAR, apparent-wind thrust T = q S (C_L sinβ − C_D cosβ), road load m g C_rr + ½ρC_dA v²',
    'Backup power bottleneck P = min(P_exergy, P_actuator, P_heat-throughput) with P_exergy = A q η_Carnot η_II',
    'Actuator thermal time constant τ = ρc(t/2)²·4/(π²k) + ρc(t/2)/h_contact; cycle frequency f = min(1/2τ, v/2πR)',
    'Static rollover limit U_roll² = m g d / (½ρ S C_s h_cp cosψ)',
    'Local elasticities; Morris elementary effects (μ*, σ); Sobol first-order and total indices (Saltelli/Jansen estimators)',
    'Latin hypercube Monte Carlo propagation; Gaussian-process regression (squared-exponential kernel, marginal-likelihood length scales)',
    'NSGA-II non-dominated sorting with crowding distance, SBX crossover and polynomial mutation; constraint-domination; 2-D hypervolume',
  ],
  methods: ['All studies call the same reduced model so results are mutually consistent', 'Quasi-random Halton designs for Sobol', 'Surrogate trained on 60 LHS points and validated on 40 independent points', 'Reproducible random seeds'],
  assumptions: ['Reduced model for screening — escalate promising candidates to the high-fidelity suites (multifidelity strategy)', 'Second-law fraction η_II = 0.5 is an optimistic finite-time bound', 'The AI/surrogate component never replaces governing physics; it accelerates exploration and is validated'],
  ibc: {
    initial: ['Initial design vector, starting geometry and material selections', 'Initial optimiser population / search point, random seed, parameter uncertainty, reference state'],
    boundary: ['Variable bounds, equality/inequality constraints, material admissibility', 'Thermodynamic (exergy) limits, maximum stresses, minimum fatigue life, manufacturing restrictions, vehicle dimensions'],
  },
  cvv: {
    calibration: ['Surrogate predictions vs independently solved high-fidelity designs', 'Experimental testing of selected optimised candidates'],
    verification: ['Sobol indices on the Ishigami benchmark', 'Sensitivities vs finite differences', 'Constraint enforcement and reproducibility of NSGA-II'],
    validation: ['Hardware tests of the Pareto-selected configurations'],
  },
  formats: { import: ['CSV design-of-experiments tables'], export: ['Pareto front CSV', 'Sensitivity JSON', 'Plots PNG/SVG'] },
  params: [
    { key: 'vTarget_kmh', label: 'Backup design speed', unit: 'km/h', type: 'number', min: 5, max: 60, step: 1, group: 'Study', default: 20 },
    { key: 'Ptarget', label: 'Target continuous backup power (inverse design)', unit: 'W', type: 'number', min: 20, max: 2000, step: 10, group: 'Study', default: 200 },
    { key: 'q', label: 'Environmental heat flux to collector', unit: 'W/m²', type: 'number', min: 5, max: 800, step: 5, group: 'Baseline design', default: 60, help: 'Night radiative cooling ≈ 20–100 W/m²; absorbed sunlight ≈ 300–700 W/m².', auto: (c) => { const o = c.upstream('s13'); return o?.qNetNight_W_m2 ? { value: Math.max(5, Math.round(o.qNetNight_W_m2)), prov: 'coupled' } : undefined; } },
    { key: 'dT', label: 'Effective hot–cold ΔT', unit: 'K', type: 'number', min: 1, max: 60, step: 0.5, group: 'Baseline design', default: 5, auto: (c) => { const o = c.upstream('s14'); return o?.ThotIface_C !== undefined && o?.TcoldIface_C !== undefined ? { value: +Math.max(0.5, o.ThotIface_C - o.TcoldIface_C).toFixed(1), prov: 'coupled' } : undefined; } },
    { key: 'Ac', label: 'Collector area', unit: 'm²', type: 'number', min: 0.5, max: 10, step: 0.1, group: 'Baseline design', default: 4 },
    { key: 'nCart', label: 'Cartridges per driven wheel', type: 'number', min: 4, max: 64, step: 1, group: 'Baseline design', default: 24 },
    { key: 'mc', label: 'Active NiTi per cartridge', unit: 'kg', type: 'number', min: 0.005, max: 0.3, step: 0.005, group: 'Baseline design', default: 0.05 },
    { key: 'w', label: 'Net specific work per cycle', unit: 'J/g', type: 'number', min: 0.01, max: 10, step: 0.01, group: 'Baseline design', default: 0.5, auto: (c) => { const o = c.upstream('s15'); return o?.wNet_J_kg ? { value: +(Math.max(0.01, o.wNet_J_kg / 1000)).toFixed(3), prov: 'coupled' } : undefined; } },
    { key: 't', label: 'Actuator element thickness', unit: 'mm', type: 'number', min: 0.05, max: 3, step: 0.05, group: 'Baseline design', default: 0.5 },
    { key: 'alpha', label: 'Wing effective incidence', unit: '°', type: 'number', min: 2, max: 18, step: 0.5, group: 'Baseline design', default: 10 },
    { key: 'U', label: 'Design true wind (beam)', unit: 'm/s', type: 'number', min: 1, max: 15, step: 0.5, group: 'Baseline design', default: 6, auto: (c) => { const e = c.upstream('s05') || c.env; return e?.Uz ? { value: +Math.max(1, e.Uz).toFixed(1), prov: 'coupled' } : undefined; } },
    { key: 'hContact', label: 'Actuator–interface heat-transfer coefficient', unit: 'W/(m²K)', type: 'number', min: 50, max: 20000, step: 50, group: 'Baseline design', default: 2000, expert: true },
    { key: 'mechEff', label: 'Cam/flexure transmission efficiency', type: 'range', min: 0.3, max: 1, step: 0.01, group: 'Baseline design', default: 0.7 },
    { key: 'Uroll_min', label: 'Required rollover wind limit', unit: 'm/s', type: 'number', min: 4, max: 25, step: 0.5, group: 'Constraints', default: 9 },
    { key: 'vWindMin', label: 'Required wind-mode speed at design wind', unit: 'km/h', type: 'number', min: 0, max: 60, step: 1, group: 'Constraints', default: 15 },
    { key: 'sigAllow', label: 'Wing-root allowable stress', unit: 'MPa', type: 'number', min: 10, max: 1500, step: 10, group: 'Constraints', default: 300 },
    { key: 'Nsobol', label: 'Sobol base sample size', type: 'select', group: 'Numerics', default: 256, options: [128, 256, 512, 1024], expert: true },
    { key: 'pop', label: 'NSGA-II population', type: 'select', group: 'Numerics', default: 60, options: [40, 60, 100], expert: true },
    { key: 'gens', label: 'NSGA-II generations', type: 'select', group: 'Numerics', default: 60, options: [30, 60, 120], expert: true },
    { key: 'seed', label: 'Random seed', type: 'number', min: 1, max: 99999, step: 1, group: 'Numerics', default: 25, expert: true },
  ],
  presets: [{ label: 'Calm clear night', values: { q: 60, dT: 5 } }, { label: 'Sunny calm day', values: { q: 550, dT: 35 } }, { label: 'Breakthrough actuator (5 J/g, 0.1 mm)', values: { w: 5, t: 0.1 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const v = ctx.vehicle, rho = ctx.env?.rho || 1.225, L = v.wheelbase, tr = v.track;
    const c = { span0: v.wingHeight, S0: v.wingArea, nW: v.nWings || 2, rho, R: v.wheelRadius, h: v.height, vTarget: p.vTarget_kmh / 3.6, hContact: p.hContact, mechEff: p.mechEff, tipArm: (tr / 2) * (0.42 * L) / Math.hypot(L, tr / 2), cosPsi: L / Math.hypot(L, tr / 2), sig0: 40, Calpha: 3 * 6000 };
    const base = { S: v.wingArea, alpha: p.alpha, Crr: v.Crr, CdA: v.CdA, nCart: p.nCart, mc: p.mc, Ac: p.Ac, dT: p.dT, q: p.q, w: p.w, t: p.t, U: p.U };
    base.m = Math.max(100, v.mass - (2.5 * base.S + 6 * base.Ac + 2 * base.nCart * base.mc + 20));
    const B = model(base, c);
    const keys = VARS.map(r => r[0]), bounds = VARS.map(r => [r[3], r[4]]);
    const toX = (arr) => Object.fromEntries(keys.map((kk, i) => [kk, arr[i]]));
    ctx.progress(0.1, 'tornado');
    const tspec = VARS.map(([key, label]) => ({ key, label, lo: base[key] * 0.8, hi: base[key] * 1.2 }));
    const tor = tornado(x => model(x, c).closure, base, tspec), torW = tornado(x => model(x, c).vWind_kmh, base, tspec);
    ctx.progress(0.25, 'Morris'); await ctx.yield();
    const mor = morris(arr => model(toX(arr), c).closure, bounds, { r: 30, seed: p.seed });
    ctx.progress(0.4, 'Sobol'); await ctx.yield();
    const sob = sobol(arr => Math.min(300, model(toX(arr), c).closure), bounds, { N: +p.Nsobol }), sobW = sobol(arr => model(toX(arr), c).vWind_kmh, bounds, { N: +p.Nsobol });
    // Ishigami benchmark verification (known S1 = 0.314, 0.442, 0)
    const ish = sobol(x => Math.sin(x[0]) + 7 * Math.sin(x[1]) ** 2 + 0.1 * x[2] ** 4 * Math.sin(x[0]), [[-Math.PI, Math.PI], [-Math.PI, Math.PI], [-Math.PI, Math.PI]], { N: 2048 });
    ctx.progress(0.55, 'uncertainty'); await ctx.yield();
    const uKeys = ['Crr', 'CdA', 'U', 'dT', 'q', 'w', 'mechEff'], dists = [{ type: 'normal', a: base.Crr, b: base.Crr * 0.15 }, { type: 'normal', a: base.CdA, b: base.CdA * 0.1 }, { type: 'uniform', a: base.U * 0.5, b: base.U * 1.5 }, { type: 'triangular', a: base.dT * 0.5, b: base.dT * 1.3, c: base.dT }, { type: 'triangular', a: base.q * 0.4, b: base.q * 1.2, c: base.q }, { type: 'lognormal', a: base.w, b: 0.35 }, { type: 'uniform', a: Math.max(0.3, p.mechEff - 0.15), b: Math.min(1, p.mechEff + 0.1) }];
    const prop = (key) => propagate(xs => { const x = { ...base }; uKeys.forEach((kk, i) => { if (kk === 'mechEff') return; x[kk] = Math.max(1e-4, xs[i]); }); const cc = { ...c, mechEff: xs[6] }; return model(x, cc)[key]; }, dists, { n: 3000, seed: p.seed });
    const UQc = prop('closure'), UQw = prop('vWind_kmh'), pClose = UQc.Y.filter(y => y >= 100).length / UQc.Y.length;
    ctx.progress(0.7, 'surrogate'); await ctx.yield();
    const gK = ['nCart', 'Ac', 'dT', 'w'], gB = gK.map(kk => bounds[keys.indexOf(kk)]);
    const gEval = (u) => { const x = { ...base }; gK.forEach((kk, i) => x[kk] = u[i]); return model(x, c).closure; };
    const Xtr = lhs(60, 4, rng(p.seed + 1)).map(u => scaleSample(u, gB)), Xte = lhs(40, 4, rng(p.seed + 2)).map(u => scaleSample(u, gB));
    const norm = (x) => x.map((vv, i) => (vv - gB[i][0]) / (gB[i][1] - gB[i][0]));
    const ytr = Xtr.map(gEval), yte = Xte.map(gEval), GP = gpFit(Xtr.map(norm), ytr, { noise: 1e-4 }), pred = Xte.map(x => GP.predict(norm(x)));
    const gpM = metrics(pred.map(q => q.mean), yte), cover = yte.filter((y, i) => Math.abs(y - pred[i].mean) <= 2 * pred[i].std + 1e-9).length / yte.length;
    ctx.progress(0.8, 'NSGA-II'); await ctx.yield();
    const DV = ['S', 'alpha', 'm', 'Crr', 'CdA', 'nCart', 'mc', 'Ac', 'w', 't'], dvB = DV.map(kk => bounds[keys.indexOf(kk)]);
    const toXd = (arr) => ({ ...base, ...Object.fromEntries(DV.map((kk, i) => [kk, arr[i]])) });
    const ob = (arr) => { const x = toXd(arr), r = model(x, c); return { f: [-Math.min(300, r.closure), r.mTot], g: [p.Uroll_min - r.Uroll, p.vWindMin - r.vWind_kmh, r.sigRoot / p.sigAllow - 1], extra: r }; };
    const NS = nsga2(ob, dvB, { pop: +p.pop, gens: +p.gens, seed: p.seed });
    const front = NS.front.slice().sort((a, b) => a.f[1] - b.f[1]);
    const refP = [0, 600], hv = hypervolume2(front.map(q => q.f), refP);
    const bestClose = front.length ? front.reduce((a, b2) => (a.f[0] < b2.f[0] ? a : b2)) : null;
    // ----- inverse design: what breakthrough closes the gap at the target power? -----
    const Pt = p.Ptarget, etaC = B.etaC, wReq = Pt / Math.max(1e-9, 2 * base.nCart * base.mc * 1000 * Math.min(B.fTh, B.fWh) * p.mechEff), fReq = Pt / Math.max(1e-9, 2 * base.nCart * base.mc * base.w * 1000 * p.mechEff);
    const AcReq = Pt / Math.max(1e-9, base.q * etaC * 0.5), dTreq = (() => { const r = bisect(d => base.Ac * base.q * (d / (293.15 + d)) * 0.5 - Pt, 0.01, 2000); return r.ok ? r.x : Infinity; })();
    const qReq = Pt / Math.max(1e-9, base.Ac * etaC * 0.5);
    const ratio = (a, b2) => a / (b2 || 1e-12);
    const alone = (patch) => { const r = model({ ...base, ...patch }, c); return r.Pb >= Pt * 0.999 ? 'yes' : `no — then limited by ${r.limit} (${fmt(r.Pb, 3)} W)`; };
    // ----- KPIs -----
    const k = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k(`Backup power at ${p.vTarget_kmh} km/h`, B.Pb, 'W', `Bottleneck: ${B.limit}. Exergy ${fmt(B.Pex, 3)} W · actuator ${fmt(B.Pact, 3)} W · heat-throughput ${fmt(B.Pthr, 3)} W.`, 'calc', B.closure >= 100 ? 'ok' : 'bad');
    k('Required wheel power', B.Preq, 'W', 'Road load at the backup design speed with the total mass including wings, collectors and cartridges.', 'calc');
    k('Power-gap closure (baseline)', B.closure, '%', 'Backup power / required power. 100 % means continuous atmospheric-only propulsion at that speed.', 'calc', B.closure >= 100 ? 'ok' : B.closure >= 30 ? 'warn' : 'bad');
    k('Probability of ≥100 % closure (UQ)', pClose * 100, '%', '3 000-sample Latin hypercube propagation of uncertain wind, ΔT, flux, work density, Crr, CdA and efficiency.', 'calc');
    k('Wind-mode safe sustainable speed', B.vWind_kmh, 'km/h', `Beam reach in ${base.U} m/s true wind; C_L ${fmt(B.CL, 3)}, C_D ${fmt(B.CD, 3)}${B.depowered ? ' — wing depowered to stay inside the rollover allowance' : ''}; tyre side-force drag included.`, 'calc');
    k('Thermal cycling limit', B.fTh, 'Hz', `Actuator ${base.t} mm thick; wheel requires ${fmt(B.fWh, 3)} Hz at ${p.vTarget_kmh} km/h.`, 'calc', B.fTh >= B.fWh ? 'ok' : 'warn');
    k('Most influential input (Sobol S_T)', VARS[sob.ST.indexOf(Math.max(...sob.ST))][1], '', 'Largest total-effect index on power-gap closure (includes interactions).', 'calc');
    k('Pareto-optimal designs', front.length, '', `Feasible non-dominated designs (closure vs mass) after ${p.gens} generations; hypervolume ${fmt(hv, 4)}.`, 'generated');
    k('Best feasible closure found', bestClose ? -bestClose.f[0] : NaN, '%', 'Highest closure on the constrained Pareto front within the variable bounds.', 'generated', bestClose && -bestClose.f[0] >= 100 ? 'ok' : 'bad');
    k('Surrogate validation R²', gpM.r2, '', `GP on 60 points, tested on 40 independent points; RMSE ${fmt(gpM.rmse, 3)} %, 2σ coverage ${fmt(cover * 100, 3)} %.`, 'calc', gpM.r2 > 0.9 ? 'ok' : 'warn');
    k('Required specific work (inverse)', wReq, 'J/g', `To deliver ${Pt} W at the current cycling rate — ${fmt(ratio(wReq, base.w), 3)}× today's assumption.`, 'calc');
    k('Required collector area (inverse)', AcReq, 'm²', `Exergy bound at ΔT = ${base.dT} K and q = ${base.q} W/m² — ${fmt(ratio(AcReq, base.Ac), 3)}× the current area.`, 'calc', AcReq <= 8 ? 'warn' : 'bad');
    // verdicts
    out.verdicts.push({ test: `Power-gap closure at ${p.vTarget_kmh} km/h (baseline)`, status: B.closure >= 100 ? 'pass' : 'fail', detail: `${fmt(B.closure, 3)} % (${fmt(B.Pb, 3)} W of ${fmt(B.Preq, 3)} W). Deficit ${fmt(Math.max(0, B.Preq - B.Pb), 3)} W. Limiting mechanism: ${B.limit}.`, fix: 'See inverse design: the bottleneck, not actuator force, determines what must improve.' });
    out.verdicts.push({ test: 'Optimised feasible design closes the gap', status: bestClose && -bestClose.f[0] >= 100 ? 'pass' : 'fail', detail: bestClose ? `Best constrained design reaches ${fmt(-bestClose.f[0], 3)} % at ${fmt(bestClose.f[1], 3)} kg with the environment held at q = ${base.q} W/m², ΔT = ${base.dT} K, U = ${base.U} m/s (design variables only: wing, mass, resistances, cartridges, collector, work density ≤ 5 J/g, element thickness).` : 'No feasible design satisfies rollover, wind-speed and stress constraints together.', fix: 'Relaxing constraints is a design decision; physics bounds (exergy) cannot be optimised away.' });
    out.verdicts.push({ test: 'Exergy constraint enforced', status: B.Pb <= B.Pex + 1e-9 ? 'pass' : 'fail', detail: `P_backup ${fmt(B.Pb, 3)} W ≤ P_exergy ${fmt(B.Pex, 3)} W — no design is credited with more work than the environment can supply.` });
    out.verdicts.push({ test: 'Sobol benchmark (Ishigami)', status: Math.abs(ish.S1[0] - 0.314) < 0.05 && Math.abs(ish.S1[1] - 0.442) < 0.05 ? 'pass' : 'warn', detail: `S₁ = ${ish.S1.map(s => fmt(s, 3)).join(', ')} vs analytic 0.314, 0.442, 0.000.` });
    // checks
    out.checks.push({ name: 'Ishigami S₁ (x₁)', value: ish.S1[0], target: '0.314 ± 0.05', pass: Math.abs(ish.S1[0] - 0.314) < 0.05 });
    out.checks.push({ name: 'Ishigami S₁ (x₂)', value: ish.S1[1], target: '0.442 ± 0.05', pass: Math.abs(ish.S1[1] - 0.442) < 0.05 });
    out.checks.push({ name: 'Ishigami S_T (x₃)', value: ish.ST[2], target: '0.244 ± 0.06', pass: Math.abs(ish.ST[2] - 0.244) < 0.06 });
    const h1 = 1e-4 * base.Ac, fd = (model({ ...base, Ac: base.Ac + h1 }, c).closure - model({ ...base, Ac: base.Ac - h1 }, c).closure) / (2 * h1);
    out.checks.push({ name: 'Tornado elasticity vs central finite difference (Ac)', value: fd, target: 'consistent sign with tornado', pass: Math.sign(fd) === Math.sign((tor.rows.find(r => r.key === 'Ac').hi - tor.rows.find(r => r.key === 'Ac').lo)) || Math.abs(fd) < 1e-9 });
    out.checks.push({ name: 'All Pareto designs satisfy constraints', value: front.filter(q => q.cv > 0).length, target: '0 violations', pass: front.every(q => q.cv === 0) });
    // charts
    out.charts.push({ title: 'Tornado — power-gap closure (±20 % inputs)', type: 'tornado', rows: tor.rows.slice(0, 10), base: tor.base, x: { label: 'Closure', unit: '%' }, explain: 'One-at-a-time ±20 % perturbations around the baseline. Only inputs on the active bottleneck move the result — inputs with zero bars are not limiting today.' });
    out.charts.push({ title: 'Tornado — wind-mode sustainable speed (±20 %)', type: 'tornado', rows: torW.rows.slice(0, 10), base: torW.base, x: { label: 'Speed', unit: 'km/h' }, explain: 'Primary propulsion sensitivity: wing area, true wind and incidence dominate; rolling resistance and mass follow.' });
    out.charts.push({ title: 'Sobol indices — closure (global, over full bounds)', type: 'bar', categories: VARS.map(r => r[1]), y: { label: 'Index' }, series: [{ name: 'First-order S₁', values: sob.S1 }, { name: 'Total S_T', values: sob.ST }], explain: 'Variance-based global sensitivity across the whole design space. S_T − S₁ measures interactions — large for multiplicative bottleneck models.', wide: true });
    out.charts.push({ title: 'Sobol indices — wind-mode speed', type: 'bar', categories: VARS.map(r => r[1]), y: { label: 'Index' }, series: [{ name: 'S₁', values: sobW.S1 }, { name: 'S_T', values: sobW.ST }], explain: 'Global importance of each variable for primary wind propulsion.', wide: true });
    out.charts.push({ title: 'Morris screening (μ* vs σ)', type: 'scatter', x: { label: 'μ* (mean |elementary effect|)' }, y: { label: 'σ (non-linearity / interaction)' }, series: [{ name: 'inputs', x: mor.map(m2 => m2.muStar), y: mor.map(m2 => m2.sigma), labels: VARS.map(r => r[0]), r: 6 }], explain: 'Inputs far from the origin matter; high σ/μ* means non-linear or interacting effects. Cheap screening before Sobol.' });
    const hc = histogram(UQc.Y.map(y => Math.min(250, y)), 40), hw = histogram(UQw.Y, 40);
    out.charts.push({ title: 'Uncertainty in power-gap closure (LHS, 3 000 samples)', type: 'line', x: { label: 'Closure', unit: '%' }, y: { label: 'Samples' }, series: [{ name: 'closure', x: hc.centers, y: hc.counts, area: true }], annotations: [{ type: 'vline', x: 100, label: '100 %', color: '#22c55e' }, { type: 'vline', x: UQc.p50, label: 'median', color: '#f59e0b' }], explain: `Distribution of closure given uncertain environment and material behaviour; P(≥100 %) = ${fmt(pClose * 100, 3)} %. 5–95 % range ${fmt(UQc.p05, 3)}–${fmt(UQc.p95, 3)} %.` });
    out.charts.push({ title: 'Uncertainty in wind-mode speed', type: 'line', x: { label: 'Speed', unit: 'km/h' }, y: { label: 'Samples' }, series: [{ name: 'speed', x: hw.centers, y: hw.counts, area: true }], explain: `Wind-mode speed under ±50 % wind variability: median ${fmt(UQw.p50, 3)} km/h.` });
    out.charts.push({ title: 'NSGA-II Pareto front: closure vs total mass', type: 'scatter', x: { label: 'Total mass', unit: 'kg' }, y: { label: 'Power-gap closure', unit: '%' }, series: [{ name: 'population (feasible & infeasible)', x: NS.population.map(q => q.f[1]), y: NS.population.map(q => -q.f[0]), r: 3, color: '#64748b' }, { name: 'Pareto front (feasible)', x: front.map(q => q.f[1]), y: front.map(q => -q.f[0]), mode: 'lines+markers', r: 5, color: '#22c55e' }], annotations: [{ type: 'hline', y: 100, label: 'full closure', color: '#ef4444' }, { type: 'point', x: B.mTot, y: B.closure, label: 'baseline', color: '#f97316' }], explain: 'Each green point is a design no other feasible design beats on both closure and mass. Distance below the red line is the residual power gap the materials must still close.', wide: true });
    out.charts.push({ title: 'NSGA-II convergence (front size per generation)', type: 'line', x: { label: 'Generation' }, y: { label: 'Feasible non-dominated designs' }, series: [{ name: 'front size', x: NS.history.map((_, i) => i + 1), y: NS.history }], explain: 'Front population stabilising indicates convergence; combine with hypervolume for a quantitative criterion.' });
    out.charts.push({ title: 'Surrogate validation: GP prediction vs model', type: 'scatter', x: { label: 'Reduced-model closure', unit: '%' }, y: { label: 'GP prediction', unit: '%' }, series: [{ name: 'test points', x: yte, y: pred.map(q => q.mean), r: 5 }, { name: 'ideal', x: [Math.min(...yte), Math.max(...yte)], y: [Math.min(...yte), Math.max(...yte)], dash: '5 4', color: '#94a3b8' }], explain: `Independent validation of the AI surrogate: R² ${fmt(gpM.r2, 3)}, ${fmt(cover * 100, 3)} % of test points inside ±2σ. The surrogate is only used where it validates.` });
    const dTs = linspace(1, 60, 60);
    out.charts.push({ title: 'Exergy-limited backup power vs ΔT and collector area', type: 'line', x: { label: 'Effective ΔT', unit: 'K' }, y: { label: 'Max backup power', unit: 'W', log: true }, series: [2, 4, 8].map(A => ({ name: `A = ${A} m², q = ${base.q} W/m²`, x: dTs, y: dTs.map(d => Math.max(0.01, A * base.q * d / (293.15 + d) * 0.5)) })), annotations: [{ type: 'hline', y: B.Preq, label: 'required', color: '#ef4444' }, { type: 'vline', x: base.dT, label: 'current ΔT' }], explain: 'Upper bound from the second law: no metamaterial geometry can exceed it. At night-time ΔT of a few kelvin the curves lie orders of magnitude below the road-load requirement.' });
    // tables
    out.tables.push({ title: `Inverse design — breakthrough needed for ${Pt} W continuous`, cols: ['Quantity', 'Current', 'Required for its own limit', 'Factor', 'Reaches the target alone?'], rows: [
      ['Net specific work per cycle (J/g)', base.w, wReq, ratio(wReq, base.w), alone({ w: wReq })], ['Cycle frequency (Hz)', Math.min(B.fTh, B.fWh), fReq, ratio(fReq, Math.min(B.fTh, B.fWh)), 'needs thinner elements / contact (see t)'],
      ['Collector area (m²)', base.Ac, AcReq, ratio(AcReq, base.Ac), alone({ Ac: AcReq })], ['Effective ΔT (K)', base.dT, dTreq, ratio(dTreq, base.dT), alone({ dT: dTreq })], ['Environmental heat flux (W/m²)', base.q, qReq, ratio(qReq, base.q), alone({ q: qReq })],
    ], explain: 'Each row varies one quantity with all others held at the baseline. Values beyond physical reach (e.g. heat fluxes far above solar or ΔT beyond the environment) mean that route cannot close the gap.' });
    out.tables.push({ title: 'Pareto-optimal designs (top 15 by closure)', cols: ['Closure %', 'Mass kg', ...DV], rows: front.slice().sort((a, b2) => a.f[0] - b2.f[0]).slice(0, 15).map(q => [-q.f[0], q.f[1], ...q.x]) });
    out.tables.push({ title: 'Global sensitivity indices', cols: ['Variable', 'Unit', 'Range', 'S₁ closure', 'S_T closure', 'S₁ wind speed', 'S_T wind speed', 'Morris μ*'], rows: VARS.map((r, i) => [r[1], r[2], `${r[3]}–${r[4]}`, sob.S1[i], sob.ST[i], sobW.S1[i], sobW.ST[i], mor[i].muStar]) });
    out.exports = [{ label: 'Pareto front (CSV)', action: () => { const rows = front.map(q => [-q.f[0], q.f[1], ...q.x].join(',')); const blob = new Blob([['closure_pct,mass_kg,' + DV.join(','), ...rows].join('\n')], { type: 'text/csv' }); import('../core/chart.js').then(m2 => m2.download(blob, 'atmosxp-pareto.csv')); } }];
    // recs
    out.recs.push({ level: B.closure >= 100 ? 'good' : 'critical', text: `The binding limit is “${B.limit}”. ${B.limit === 'exergy' ? 'More actuator, more force or better geometry cannot help; only a larger usable ΔT, more collector area or higher environmental heat flux raises the ceiling — or reduce the road load.' : B.limit.includes('thermal') ? 'Thinner active elements or better contact conductance raise the cycling rate; check that work per cycle survives miniaturisation.' : 'Improve net work per cycle (low-hysteresis alloy, optimised loading path) or add active mass.'}`, basis: 'Bottleneck analysis of the reduced model' });
    const top3 = sob.ST.map((s, i) => [s, VARS[i][1]]).sort((a, b2) => b2[0] - a[0]).slice(0, 3).map(s => s[1]);
    out.recs.push({ level: 'info', text: `Prioritise experiments on: ${top3.join(', ')} — they carry most of the output variance (Sobol S_T). Measuring them first reduces uncertainty fastest (automatic experiment prioritisation).`, basis: 'Global sensitivity' });
    out.recs.push({ level: 'good', text: 'Escalate only Pareto designs to high-fidelity Suites 06–08 and 13–17 (multifidelity), then build and test the selected candidate; record surrogate-vs-hardware discrepancies in Suite 24.', basis: 'Specification Module 25 & Part B §28' });
    out.recs.push({ level: 'sustain', text: 'Include embodied carbon of extra collector area and NiTi mass (Suite 26) as a third objective before committing — closing the gap with very large collectors may worsen life-cycle impact.', basis: 'Sustainability-aware optimisation' });
    out.explain.push({ title: 'Why a reduced model?', text: 'Global sensitivity and evolutionary optimisation need tens of thousands of evaluations. The reduced model keeps the governing physics (aerodynamic force resolution, road load, second-law limit, thermal time constants) at millisecond cost; promising designs are then verified with the detailed solvers.' });
    out.outputs = { sobol: Object.fromEntries(VARS.map((r, i) => [r[0], { S1: sob.S1[i], ST: sob.ST[i] }])), paretoFront: front.map(q => ({ closure: -q.f[0], mass: q.f[1], x: toXd(q.x) })), baselineClosure: B.closure, pClose, bottleneck: B.limit, hypervolume: hv, required: { w_J_g: wReq, Ac_m2: AcReq, dT_K: dTreq } };
    out.headline = `Baseline backup closes ${fmt(B.closure, 3)} % of the ${fmt(B.Preq, 3)} W need at ${p.vTarget_kmh} km/h (limit: ${B.limit}); best constrained Pareto design ${bestClose ? fmt(-bestClose.f[0], 3) : '—'} %. P(full closure) under uncertainty ${fmt(pClose * 100, 3)} %.`;
    out.headlineKind = B.closure >= 100 ? 'ok' : 'warn';
    void mean; void std; void clamp; void range;
    return out;
  },
};
