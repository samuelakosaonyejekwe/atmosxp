// Suite 10 — Road wheels, tyres, bearings & tyre–road interaction.
import { G0, fmt, linspace, range, sum, max, rad, deg } from '../core/num.js';
import { kmh, moistAir, flatPlateH } from '../core/physics.js';
import { ROADS, mfLateral, mfLongitudinal, fiala, dugoff, crrModel, wheelLayout, threeWheelLoads } from '../core/vehicle.js';

export default {
  description: 'Tyre–road force generation (Magic Formula, Fiala/brush, Dugoff combined slip, friction ellipse), rolling resistance versus speed, pressure and load, wheel-bearing friction (Palmgren) and loss power, Hertzian contact stresses, wheel rotational inertia, tyre hysteresis heating, and traction limits against the wing side force — for dry and wet roads.',
  literature: 'low rolling resistance tyre magic formula bearing friction torque lightweight vehicle',
  credibility: 'Analytical screening', sustain: true,
  equations: ['Rigid-body rotational equation J ω̇ = T_drive − T_brake − F_x R − M_bearing', 'Slip ratio κ = (ωR − v)/v and slip angle α = atan(v_y / v_x)', 'Pacejka Magic Formula y = D sin(C atan(Bx − E(Bx − atan Bx))) for F_x(κ) and F_y(α) with load-proportional stiffness', 'Fiala brush model with parabolic pressure; Dugoff combined-slip model; friction-ellipse combination', 'Rolling-resistance coefficient C_rr(p, F_z, v) ∝ p^−0.4 F_z^0.1 (1 + c v²) (SAE J2452-type form)', 'Palmgren bearing friction M = M₀ + M₁, M₀ = 10⁻⁷ f₀ (νn)^{2/3} d_m³, M₁ = f₁ P₁ d_m', 'Stribeck maximum ball load Q = 5F_r/Z; elliptical Hertz contact (Hamrock–Brewe) p₀ = 3Q/(2πab)', 'Tyre hysteresis heating P = C_rr F_z v; convective balance ΔT = P/(hA) with flat-plate Nusselt correlation', 'Coulomb friction with load sensitivity and wet-road reduction'],
  methods: ['Analytical curve generation over slip ranges', 'Load-sweep at 50/100/150 % of static wheel load', 'Equivalent rotating mass Σ J/R² added to vehicle inertia'],
  assumptions: ['Steady-state slip characteristics (no relaxation length)', 'Elliptical Hertz contact with 0.52 groove conformity (Hamrock–Brewe approximations)', 'Tread compound properties from the materials registry'],
  ibc: { initial: ['Wheel speed, vertical load, inflation pressure, tyre temperature, deformation, initial slip ratio/angle, camber, bearing temperature & preload, road-contact state'], boundary: ['Tyre–road normal contact and tangential friction, road velocity/profile, inclination, hub support, axle loads, braking torque, thermomechanical driving torque, bearing reactions, surface thermal exchange'] },
  cvv: { calibration: ['Rolling-resistance drum tests (ISO 28580)', 'Flat-track slip-force measurements (Fx–κ, Fy–α)', 'Vertical stiffness, contact-patch imprint, bearing friction torque vs speed/temperature'], verification: ['Magic Formula initial slope equals cornering stiffness', 'Fiala saturation equals μF_z', 'Dugoff reduces to pure slip when κ or α = 0'], validation: ['Coast-down test of the prototype (C_rr, C_dA separation)', 'Skid-pad lateral limit measurements'] },
  formats: { import: ['CSV tyre test data (slip, force)', 'XLSX', 'JSON'], export: ['CSV curves', 'JSON tyre parameter set', 'PNG/SVG plots'] },
  params: [
    { key: 'road', label: 'Road surface', type: 'select', group: 'Road', default: 'dry asphalt', options: Object.keys(ROADS) },
    { key: 'speed', label: 'Vehicle speed', unit: 'km/h', type: 'range', min: 1, max: 80, step: 1, group: 'Operating point', default: 20 },
    { key: 'Fz', label: 'Reference wheel load', unit: 'N', type: 'number', min: 100, max: 4000, step: 10, group: 'Operating point', default: 981, auto: (c) => { const o = c.upstream('s01'); return o?.wheelLoads ? { value: +max(o.wheelLoads).toFixed(1), prov: 'coupled' } : { value: +(c.vehicle.mass * G0 / 3).toFixed(1), prov: 'calc' }; } },
    { key: 'width', label: 'Tyre section width', unit: 'mm', type: 'number', min: 25, max: 200, step: 1, group: 'Tyre', default: 80 },
    { key: 'pInf', label: 'Inflation pressure', unit: 'kPa', type: 'range', min: 150, max: 900, step: 10, group: 'Tyre', default: 450 },
    { key: 'Crr0', label: 'Reference C_rr (450 kPa, ref. load)', type: 'number', min: 0.002, max: 0.03, step: 0.0005, group: 'Tyre', default: 0.006, auto: (c) => ({ value: c.vehicle.Crr, prov: 'user' }) },
    { key: 'kAlpha', label: 'Cornering stiffness / load', unit: '1/rad', type: 'number', min: 4, max: 25, step: 0.5, group: 'Tyre', default: 12, expert: true },
    { key: 'kKappa', label: 'Longitudinal stiffness / load', unit: '–', type: 'number', min: 6, max: 30, step: 0.5, group: 'Tyre', default: 18, expert: true },
    { key: 'Cy', label: 'Magic Formula shape C (lateral)', type: 'number', min: 1, max: 2, step: 0.05, group: 'Tyre', default: 1.3, expert: true },
    { key: 'Ey', label: 'Magic Formula curvature E (lateral)', type: 'number', min: -2, max: 1, step: 0.05, group: 'Tyre', default: -0.6, expert: true },
    { key: 'dBore', label: 'Bearing bore d', unit: 'mm', type: 'number', min: 10, max: 60, step: 1, group: 'Bearings', default: 25 },
    { key: 'dOuter', label: 'Bearing outer diameter D', unit: 'mm', type: 'number', min: 20, max: 110, step: 1, group: 'Bearings', default: 52 },
    { key: 'C0', label: 'Static load rating C₀', unit: 'kN', type: 'number', min: 2, max: 60, step: 0.1, group: 'Bearings', default: 7.8 },
    { key: 'nBalls', label: 'Balls per bearing Z', type: 'number', min: 6, max: 16, step: 1, group: 'Bearings', default: 9, expert: true },
    { key: 'dBall', label: 'Ball diameter', unit: 'mm', type: 'number', min: 3, max: 16, step: 0.01, group: 'Bearings', default: 7.94, expert: true },
    { key: 'f0', label: 'Lubrication factor f₀', type: 'number', min: 0.7, max: 4, step: 0.1, group: 'Bearings', default: 1.5, expert: true, help: '≈1–2 grease, 0.7–1 oil mist.' },
    { key: 'nu', label: 'Lubricant viscosity at temp.', unit: 'cSt', type: 'number', min: 5, max: 300, step: 1, group: 'Bearings', default: 40, expert: true },
    { key: 'sideForce', label: 'Wing side force to react', unit: 'N', type: 'number', min: 0, max: 6000, step: 10, group: 'Aerodynamic loads', default: 300, auto: (c) => { const o = c.upstream('s07') || c.upstream('s06'); if (o?.sideN !== undefined) return { value: +Math.abs(o.sideN).toFixed(1), prov: 'coupled' }; const e = c.env; return e ? { value: +(0.5 * e.rho * (e.Uz ** 2 + 0.01) * c.vehicle.wingArea * 1.0 + 0).toFixed(1), prov: 'calc' } : undefined; } },
  ],
  presets: [{ label: 'Dry, 20 km/h', values: { road: 'dry asphalt', speed: 20 } }, { label: 'Wet, 40 km/h', values: { road: 'wet asphalt', speed: 40 } }, { label: 'Under-inflated', values: { pInf: 250 } }, { label: 'Ice', values: { road: 'ice' } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const veh = ctx.vehicle, road = ROADS[p.road], mu = road.mu, v = kmh(p.speed), R = veh.wheelRadius, tyre = ctx.material('tyre'), rim = ctx.material('wheel');
    const s01 = ctx.upstream('s01'); const lay = wheelLayout(veh, s01?.frontFrac ?? veh.frontFrac); const loads = s01?.wheelLoads || threeWheelLoads(lay.pts, veh.mass * G0);
    const opts = { kAlpha: p.kAlpha, C: p.Cy, E: p.Ey };
    // force curves
    const al = linspace(-15, 15, 121), ka = linspace(-0.3, 0.3, 121);
    const Fz3 = [0.5, 1, 1.5].map(f => f * p.Fz);
    out.charts.push({ title: 'Lateral force vs slip angle (Magic Formula, load sensitivity)', type: 'line', x: { label: 'Slip angle α', unit: '°' }, y: { label: 'Lateral force F_y', unit: 'N' }, series: [...Fz3.map(F => ({ name: `F_z = ${fmt(F, 3)} N`, x: al, y: al.map(a => -mfLateral(rad(a), F, mu, opts)) })), { name: 'Fiala (ref. load)', x: al, y: al.map(a => -fiala(rad(a), p.Fz, mu, p.kAlpha * p.Fz)), dash: '5 3' }], explain: 'Linear at small slip (slope = cornering stiffness), then saturating at μF_z. The Fiala brush model gives a physically based cross-check. Higher load gives more force but proportionally less per newton of load.' });
    out.charts.push({ title: 'Longitudinal force vs slip ratio (dry vs wet)', type: 'line', x: { label: 'Slip ratio κ', unit: '–' }, y: { label: 'F_x', unit: 'N' }, series: [{ name: p.road, x: ka, y: ka.map(k => mfLongitudinal(k, p.Fz, mu, { kKappa: p.kKappa })) }, { name: 'wet asphalt', x: ka, y: ka.map(k => mfLongitudinal(k, p.Fz, ROADS['wet asphalt'].mu, { kKappa: p.kKappa })), dash: '4 3' }, { name: 'ice', x: ka, y: ka.map(k => mfLongitudinal(k, p.Fz, ROADS.ice.mu, { kKappa: p.kKappa })), dash: '2 3' }], explain: 'Traction/braking force builds with slip, peaks around 10–15 % slip and falls toward the locked-wheel value. This bounds both hub-drive traction and braking (Suite 12).' });
    // combined slip envelope (Dugoff)
    const Cx = p.kKappa * p.Fz, Cy = p.kAlpha * p.Fz, env = [];
    for (const a of [1, 2, 4, 6, 10]) { const xs = [], ys = []; for (const k of linspace(-0.25, 0.25, 61)) { const d = dugoff(k, rad(a), p.Fz, mu, Cx, Cy); xs.push(d.Fx); ys.push(-d.Fy); } env.push({ name: `α = ${a}°`, x: xs, y: ys }); }
    const th = linspace(0, 2 * Math.PI, 90); env.push({ name: 'Friction circle μF_z', x: th.map(t => mu * p.Fz * Math.cos(t)), y: th.map(t => mu * p.Fz * Math.sin(t)), dash: '5 4', color: '#ef4444' });
    out.charts.push({ title: 'Combined slip: friction ellipse (Dugoff)', type: 'line', x: { label: 'F_x', unit: 'N' }, y: { label: 'F_y', unit: 'N' }, series: env, explain: 'Using grip for braking or driving reduces the lateral force available, and vice versa. Wing side force plus cornering must fit inside the circle while braking.' });
    // rolling resistance
    const vs = linspace(0, 80, 41);
    out.charts.push({ title: 'Rolling-resistance coefficient vs speed & pressure', type: 'line', x: { label: 'Speed', unit: 'km/h' }, y: { label: 'C_rr', unit: '–' }, series: [250, 350, 450, 600, 800].map(pp => ({ name: `${pp} kPa`, x: vs, y: vs.map(s => crrModel(p.Crr0, { p: pp, Fz: p.Fz, Fz0: p.Fz, v: kmh(s), road: p.road })) })), explain: 'Higher inflation pressure lowers hysteresis losses; speed adds a small quadratic rise. Every 0.001 of C_rr costs ≈ 2.9 N (≈ 16 W at 20 km/h) for a 300 kg vehicle.' });
    const crrW = loads.map(F => crrModel(p.Crr0, { p: p.pInf, Fz: F, Fz0: p.Fz, v, road: p.road })), Froll = sum(loads.map((F, i) => crrW[i] * F)), CrrEff = Froll / sum(loads);
    // bearings (two per wheel)
    const dm = (p.dBore + p.dOuter) / 2, n = v / (2 * Math.PI * R) * 60, omega = v / R;
    const bearing = (Fr) => { const nun = p.nu * n, M0 = nun >= 2000 ? 1e-7 * p.f0 * nun ** (2 / 3) * dm ** 3 : 160e-7 * p.f0 * dm ** 3, f1 = 0.0007 * Math.sqrt(Math.min(1, Fr / (p.C0 * 1000))), M1 = f1 * Fr * dm; return { M0, M1, M: (M0 + M1) / 1000 }; };
    const bw = loads.map(F => { const b = bearing(F / 2); return { ...b, P: 2 * b.M * omega }; }), Pbear = sum(bw.map(b => b.P));
    const nsp = linspace(5, 80, 30); out.charts.push({ title: 'Bearing loss power vs speed (all wheels)', type: 'line', x: { label: 'Speed', unit: 'km/h' }, y: { label: 'Loss power', unit: 'W' }, series: [{ name: 'Palmgren total', x: nsp, y: nsp.map(s => { const vv = kmh(s), nn = vv / (2 * Math.PI * R) * 60; return sum(loads.map(F => { const nun = p.nu * nn, M0 = nun >= 2000 ? 1e-7 * p.f0 * nun ** (2 / 3) * dm ** 3 : 160e-7 * p.f0 * dm ** 3, M1 = 0.0007 * Math.sqrt(Math.min(1, F / 2 / (p.C0 * 1000))) * F / 2 * dm; return 2 * (M0 + M1) / 1000 * vv / R; })); }) }], explain: 'Load-dependent (M₁) and viscous (M₀) bearing friction converted to power. Small compared with rolling resistance, but it acts on every revolution of the hub drive too.' });
    // Hertz
    // Hertz elliptical point contact, ball on inner-race groove (Hamrock–Brewe approximations), groove conformity 0.52
    const E = 208e9, nu_ = 0.3, Ep = E / (1 - nu_ * nu_), rb_ = p.dBall / 2000, ri = (dm / 2 - p.dBall / 2) / 1000;
    const Rx = 1 / (1 / rb_ + 1 / ri), Ry = 1 / (1 / rb_ - 1 / (0.52 * p.dBall / 1000)), Rp = 1 / (1 / Rx + 1 / Ry), kE = 1.0339 * (Ry / Rx) ** 0.636, eps2 = 1.0003 + 0.5968 * Rx / Ry;
    const hertz = (Qb) => { const aa = Math.cbrt(6 * kE * kE * eps2 * Qb * Rp / (Math.PI * Ep)), bb = Math.cbrt(6 * eps2 * Qb * Rp / (Math.PI * kE * Ep)); return 3 * Qb / (2 * Math.PI * aa * bb); };
    const Q = 5 * (p.Fz / 2) / p.nBalls, p0 = hertz(Q);
    const Fl = linspace(0.2, 3, 30).map(f => f * p.Fz); out.charts.push({ title: 'Peak Hertz contact pressure (ball–inner race) vs wheel load', type: 'line', x: { label: 'Wheel load', unit: 'N' }, y: { label: 'p₀', unit: 'MPa' }, series: [{ name: 'p₀ elliptical contact', x: Fl, y: Fl.map(F => hertz(5 * F / 2 / p.nBalls) / 1e6) }], annotations: [{ type: 'hline', y: 4200, label: 'ISO 76 static limit 4 200 MPa', color: '#ef4444' }, { type: 'hline', y: 1500, label: 'typical running design ≤ 1 500 MPa', color: '#f59e0b' }], explain: 'Maximum contact pressure at the most heavily loaded ball (Stribeck 5F_r/Z) on a conforming groove. Exceeding ~1.5 GPa in service shortens rolling-contact fatigue life; above 4.2 GPa causes permanent brinelling.' });
    // contact patch & temperature
    const Apatch = p.Fz / (p.pInf * 1000 * 1.15), patchL = Apatch / (p.width / 1000), pc = p.Fz / Apatch / 1000;
    const air = moistAir(101325, ctx.env?.T ?? 20, ctx.env?.RH ?? 60), hc = flatPlateH(Math.max(v, 0.5), 2 * Math.PI * R * 0.25, air).h, Aty = 2 * Math.PI * R * (p.width / 1000) * 2.2, Ph = crrW[0] * loads[0] * v, dT = Ph / (hc * Aty);
    // inertia
    const mRim = (2 * Math.PI * (R - 0.075) * 0.05 * 0.003 + 6 * 0.5 * R * 0.012 * 0.03) * rim.rho, mTy = 2 * Math.PI * (R - 0.035) * 2 * Math.PI * 0.035 * 0.0035 * tyre.rho * 1.6;
    const cart = s01?.massBreakdown ? sum(Object.entries(s01.massBreakdown).filter(([k2]) => /Hub 1 (NiTi|housings)/.test(k2)).map(([, m]) => m)) : 0;
    const J = mRim * (R - 0.075) ** 2 + mTy * (R - 0.035) ** 2 + cart * (R * 0.33) ** 2, Jdisc = 2.1 * ((R * 0.55) ** 2 + (R * 0.3) ** 2) / 2, Jw = J + Jdisc, mRotEq = 3 * Jw / (R * R);
    // traction
    const FyCap = mu * sum(loads), Fs = p.sideForce, latMargin = FyCap / Math.max(1, Fs);
    const driven = loads.slice(0, veh.driveWheels || 2), FxCap = mu * sum(driven) * 0.95;
    // checks
    const slope = (-mfLateral(1e-4, p.Fz, mu, opts) + mfLateral(-1e-4, p.Fz, mu, opts)) / 2e-4;
    out.checks.push({ name: 'MF initial slope = cornering stiffness', value: slope, target: fmt(p.kAlpha * p.Fz, 5) + ' N/rad', pass: Math.abs(slope / (p.kAlpha * p.Fz) - 1) < 0.01 });
    out.checks.push({ name: 'Fiala saturation = μF_z', value: Math.abs(fiala(rad(40), p.Fz, mu, p.kAlpha * p.Fz)), target: fmt(mu * p.Fz, 5) + ' N', pass: Math.abs(Math.abs(fiala(rad(40), p.Fz, mu, p.kAlpha * p.Fz)) - mu * p.Fz) < 1e-6 * p.Fz });
    const dg = dugoff(0.05, 0, p.Fz, mu, Cx, Cy); out.checks.push({ name: 'Dugoff pure-slip limit (α = 0 ⇒ F_y = 0)', value: dg.Fy, target: '0', pass: Math.abs(dg.Fy) < 1e-9 });
    out.checks.push({ name: 'Σ wheel loads = weight', value: sum(loads), target: fmt(veh.mass * G0, 5) + ' N', pass: Math.abs(sum(loads) - veh.mass * G0) < 1e-3 * veh.mass * G0 });
    // kpis
    const k = (label, value, unit, explain, prov = 'calc', status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('Effective C_rr', CrrEff, '–', `Load-weighted rolling-resistance coefficient at ${p.pInf} kPa, ${p.speed} km/h on ${p.road}.`, 'calc', CrrEff <= veh.Crr * 1.1 ? 'ok' : 'warn');
    k('Total rolling force', Froll, 'N', 'Σ C_rr,i F_z,i over the three tyres.');
    k('Rolling power', Froll * v, 'W', 'Power dissipated by tyre hysteresis at this speed.');
    k('Bearing loss power', Pbear, 'W', 'Palmgren friction of 6 bearings (2 per wheel) at this speed.');
    k('Peak friction μ', mu, '–', 'Peak tyre–road friction of the selected surface.', 'library');
    k('Lateral grip capacity', FyCap, 'N', 'μ Σ F_z — the maximum lateral force all tyres can react.');
    k('Wing side force', Fs, 'N', 'Aerodynamic side force from the CFD/wing suites (or estimate).', ctx.upstream('s07') || ctx.upstream('s06') ? 'coupled' : 'estimated');
    k('Lateral margin', latMargin, '×', 'Grip capacity ÷ wing side force; must stay well above 1 to leave grip for cornering and braking.', 'calc', latMargin > 2 ? 'ok' : latMargin > 1.2 ? 'warn' : 'bad');
    k('Driven-wheel traction limit', FxCap, 'N', 'Max longitudinal force the hub-driven wheels can transmit.');
    k('Contact-patch mean pressure', pc, 'kPa', 'Approximately inflation pressure plus carcass stiffness contribution.');
    k('Contact-patch length', patchL * 1000, 'mm', 'Patch area ÷ section width.');
    k('Peak ball contact pressure', p0 / 1e6, 'MPa', 'Hertz elliptical contact on the inner-race groove at the most loaded ball (Stribeck 5F_r/Z).', 'calc', p0 < 1.5e9 ? 'ok' : p0 < 4.2e9 ? 'warn' : 'bad');
    k('Tyre steady temperature rise', dT, 'K', 'Hysteresis heating balanced by forced convection.', 'calc', dT < 25 ? 'ok' : 'warn');
    k('Wheel polar inertia', Jw, 'kg·m²', 'Rim, tyre, disc and rotating hub cartridges.');
    k('Equivalent rotating mass (3 wheels)', mRotEq, 'kg', 'Σ J/R² — adds to vehicle mass during acceleration and braking.');
    // verdicts
    out.verdicts.push({ test: 'Lateral grip vs wing side force', status: latMargin > 2 ? 'pass' : latMargin > 1.2 ? 'warn' : 'fail', detail: `Capacity ${fmt(FyCap, 4)} N vs side force ${fmt(Fs, 4)} N on ${p.road}.`, fix: 'Reduce wing side force (incidence/feathering), increase normal load or choose higher-grip compound; avoid wet/ice operation in strong crosswinds.' });
    out.verdicts.push({ test: 'Bearing contact stress', status: p0 < 1.5e9 ? 'pass' : p0 < 4.2e9 ? 'warn' : 'fail', detail: `p₀ = ${fmt(p0 / 1e6, 4)} MPa (elliptical contact, groove conformity 0.52).`, fix: 'Select a larger bearing (higher C₀) or more balls.' });
    out.verdicts.push({ test: 'Rolling resistance assumption', status: CrrEff <= veh.Crr * 1.1 ? 'pass' : 'warn', detail: `Effective ${fmt(CrrEff, 3)} vs global assumption ${veh.Crr}.`, fix: 'Raise inflation pressure, use a low-hysteresis silica compound, or update the global C_rr in Suite 01 so energy results stay honest.' });
    out.verdicts.push({ test: 'Tyre thermal state', status: dT < 25 ? 'pass' : 'warn', detail: `ΔT ≈ ${fmt(dT, 3)} K at ${p.speed} km/h.` });
    // tables
    out.tables.push({ title: 'Per-wheel results', cols: ['Wheel', 'F_z (N)', 'C_rr', 'Rolling force (N)', 'Bearing torque (N·m)', 'Bearing loss (W)', 'Lateral capacity (N)'], rows: loads.map((F, i) => [lay.pts[i][2], F, crrW[i], crrW[i] * F, 2 * bw[i].M, bw[i].P, mu * F]) });
    out.tables.push({ title: 'Magic Formula parameter set (export)', cols: ['Parameter', 'Lateral', 'Longitudinal'], rows: [['B (per rad / per unit slip)', p.kAlpha * p.Fz / (p.Cy * mu * p.Fz), p.kKappa * p.Fz / (1.65 * mu * p.Fz)], ['C', p.Cy, 1.65], ['D = μF_z (N)', mu * p.Fz, mu * p.Fz], ['E', p.Ey, 0.2]] });
    // recs
    if (latMargin < 2) out.recs.push({ level: 'critical', text: `Wing side force uses ${fmt(100 / latMargin, 3)} % of available lateral grip. Feather the wing in strong crosswinds and never brake hard on a wet road with the wing loaded.`, basis: 'Friction-circle budget' });
    if (p.pInf < 400) out.recs.push({ level: 'warn', text: `Inflation ${p.pInf} kPa raises C_rr by ≈ ${fmt(((p.pInf / 450) ** -0.4 - 1) * 100, 3)} % vs 450 kPa — directly enlarging the power gap the backup must close.`, basis: 'C_rr ∝ p^−0.4' });
    out.recs.push({ level: 'good', text: 'Measure C_rr with a coast-down (separating rolling from aerodynamic drag) on the actual route surface and import it here; it is one of the most sensitive parameters of the whole energy balance.', basis: 'Suite 25 sensitivity ranking' });
    out.recs.push({ level: 'sustain', text: 'Low-rolling-resistance silica tread and retreadable carcasses cut both energy demand and tyre-wear particle emissions; specify tyres with EU tyre-label class A rolling resistance where sizes allow.', basis: 'Rolling-resistance and life-cycle considerations' });
    out.explain.push({ title: 'Role of the tyres in a wind-driven car', text: 'In wind mode the tyres do not drive the car: they support it, react the wing’s large side force, and steer. In backup mode the driven wheels must also transmit hub torque. Grip, rolling losses and lateral capacity therefore set limits on every other suite.' });
    out.outputs = { Crr: CrrEff, rollingForce_N: Froll, muPeak: mu, Fymax_N: FyCap, FxDrivenMax_N: FxCap, bearingLoss_W: Pbear, wheelInertia: Jw, rotEqMass: mRotEq, contactPressure_kPa: pc, kAlpha: p.kAlpha, Calpha_N_rad: p.kAlpha * p.Fz, road: p.road, muLock: road.muLock };
    out.headline = `${p.road}: μ = ${mu}, effective C_rr ${fmt(CrrEff, 3)}, rolling ${fmt(Froll * v, 3)} W + bearings ${fmt(Pbear, 3)} W at ${p.speed} km/h; lateral grip ${fmt(FyCap, 4)} N vs wing side force ${fmt(Fs, 4)} N.`;
    void deg; void range;
    return out;
  },
};
