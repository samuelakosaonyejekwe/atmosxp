// Suite 11 — Steering, directional stability, handling & vehicle dynamics.
import { G0, fmt, linspace, rk45, rad, deg, max, min, mean, interp1 } from '../core/num.js';
import { kmh, oneMinusCos } from '../core/physics.js';
import { mfLateral, wheelLayout, threeWheelLoads, rolloverAy, trikeSSF, bicycle, ROADS } from '../core/vehicle.js';

export default {
  description: 'Linear and nonlinear handling of the three-wheel vehicle: understeer gradient, yaw-rate gain, characteristic/critical speed, step-steer and crosswind-gust transients (nonlinear tyres, adaptive Runge–Kutta), trajectory, steering effort, maximum cornering speed, lateral load transfer and rollover with the wing’s aerodynamic side force and yawing moment.',
  literature: 'three-wheeled vehicle handling rollover crosswind sensitivity yaw stability',
  credibility: 'Analytical screening',
  equations: ['Newton–Euler planar equations m(v̇ + u r) = ΣF_y, I_z ṙ = ΣM_z (3-DOF with pose X, Y, ψ)', 'Linear single-track (bicycle) model: understeer gradient K = (m/L)(b/C_f − a/C_r), yaw gain r/δ = u/(L + K u²), characteristic speed √(L/K), critical speed √(−L/K)', 'Slip angles α_f = atan((v + a r)/u) − δ, α_r = atan((v − b r)/u); Magic Formula lateral forces with degressive load sensitivity C_α ∝ F_z^0.8', 'Ackermann steering geometry δ = L/R; steering ratio; aligning moment from pneumatic + caster trail; hand-wheel effort', 'Three-support load transfer and trike tipping-axis rollover threshold with aerodynamic side force at the wing centre of pressure', 'Aerodynamic side force F_s = ½ρ U_c² S C_s and yawing moment N = F_s x_cp; one-minus-cosine gust'],
  methods: ['Dormand–Prince 5(4) adaptive integration (rtol 1e-7)', 'Bisection on minimum wheel load for rollover threshold', 'Eigenvalues of the linear 2-DOF state matrix for directional stability'],
  assumptions: ['Constant forward speed during manoeuvres', 'Quasi-static roll (load transfer without roll dynamics)', 'Steering system rigid (compliance steer neglected unless trail parameters are changed)'],
  ibc: { initial: ['Position, heading, longitudinal & lateral velocity, yaw rate, roll/pitch states, wheel speeds, steering angle, tyre slip, driver controls'], boundary: ['Tyre–road force relationships, road curvature, cross-slope, surface friction, steering travel and linkage limits, aerodynamic forces & moments, applied wheel and brake torques, transient wind loads'] },
  cvv: { calibration: ['Track tests: steady-state circle (ISO 4138), step steer (ISO 7401), crosswind sensitivity', 'Measured yaw inertia and axle loads'], verification: ['Linear limit of the nonlinear model reproduces bicycle-model yaw gain', 'Steady-state circle Ackermann angle at low speed', 'Energy-free yaw response without steer or wind (r stays 0)'], validation: ['Instrumented prototype with IMU/GNSS under crosswind exposure'] },
  formats: { import: ['CSV IMU / GNSS logs (time, yaw rate, lateral acceleration)', 'XLSX'], export: ['CSV time histories', 'PNG/SVG plots', 'JSON handling metrics'] },
  params: [
    { key: 'speed', label: 'Forward speed', unit: 'km/h', type: 'range', min: 5, max: 90, step: 1, group: 'Manoeuvre', default: 40 },
    { key: 'steer', label: 'Road-wheel step steer', unit: '°', type: 'number', min: 0, max: 15, step: 0.1, group: 'Manoeuvre', default: 2 },
    { key: 'radius', label: 'Corner radius (max-speed check)', unit: 'm', type: 'number', min: 5, max: 500, step: 1, group: 'Manoeuvre', default: 40, auto: (c) => { const r = c.store.project.imports?.route; if (!r || r.length < 3) return undefined; let best = Infinity; for (let i = 1; i < r.length - 1; i++) { const pt = (q) => [q.lon * 111320 * Math.cos(q.lat * Math.PI / 180), q.lat * 110540]; const [a, b, cc] = [pt(r[i - 1]), pt(r[i]), pt(r[i + 1])]; const A = Math.hypot(b[0] - a[0], b[1] - a[1]), B = Math.hypot(cc[0] - b[0], cc[1] - b[1]), C = Math.hypot(cc[0] - a[0], cc[1] - a[1]), ar = Math.abs((b[0] - a[0]) * (cc[1] - a[1]) - (b[1] - a[1]) * (cc[0] - a[0])) / 2; if (ar > 1e-6 && A > 3 && B > 3) best = Math.min(best, A * B * C / (4 * ar)); } return Number.isFinite(best) ? { value: Math.max(5, Math.round(best)), prov: 'calc' } : undefined; }, help: 'Auto-derived from the tightest bend of an imported route when one is present.' },
    { key: 'road', label: 'Road surface', type: 'select', group: 'Tyres', default: 'dry asphalt', options: Object.keys(ROADS), auto: (c) => c.upstream('s10')?.road },
    { key: 'kAlpha', label: 'Cornering stiffness / load', unit: '1/rad', type: 'number', min: 4, max: 25, step: 0.5, group: 'Tyres', default: 12, auto: (c) => c.upstream('s10')?.kAlpha },
    { key: 'loadSens', label: 'Load-sensitivity exponent', type: 'number', min: 0.5, max: 1, step: 0.01, group: 'Tyres', default: 0.8, expert: true, help: 'C_α ∝ F_z^n; n < 1 makes lightly loaded tyres stiffer per newton.' },
    { key: 'frontFrac', label: 'Front-axle load fraction', type: 'number', min: 0.2, max: 0.8, step: 0.01, group: 'Mass', default: 0.6, auto: (c) => { const f = c.upstream('s01')?.frontFrac ?? c.vehicle.frontFrac; return f ? { value: +f.toFixed(3), prov: 'coupled' } : undefined; } },
    { key: 'Iz', label: 'Yaw inertia I_z', unit: 'kg·m²', type: 'number', min: 20, max: 2000, step: 1, group: 'Mass', default: 180, auto: (c) => { const I = c.upstream('s01')?.inertia?.Izz; return I ? { value: +I.toFixed(1), prov: 'coupled' } : undefined; } },
    { key: 'gustU', label: 'Crosswind gust peak', unit: 'm/s', type: 'number', min: 0, max: 30, step: 0.5, group: 'Crosswind', default: 10, auto: (c) => c.env ? { value: +Math.max(c.env.gust ?? 8, 2).toFixed(1), prov: 'coupled' } : undefined },
    { key: 'gustDur', label: 'Gust duration', unit: 's', type: 'number', min: 0.5, max: 10, step: 0.5, group: 'Crosswind', default: 3 },
    { key: 'Cs', label: 'Wing side-force coefficient', type: 'number', min: 0, max: 2, step: 0.05, group: 'Crosswind', default: 1.2 },
    { key: 'xcp', label: 'Side-force centre of pressure ahead of CG', unit: 'm', type: 'number', min: -1.5, max: 1.5, step: 0.05, group: 'Crosswind', default: 0.15, help: 'Positive = ahead of CG → turns the vehicle downwind (destabilising).' },
    { key: 'steerRatio', label: 'Steering ratio', type: 'number', min: 4, max: 25, step: 0.5, group: 'Steering system', default: 12 },
    { key: 'trail', label: 'Pneumatic + caster trail', unit: 'mm', type: 'number', min: 5, max: 100, step: 1, group: 'Steering system', default: 45 },
    { key: 'rsw', label: 'Steering-wheel radius', unit: 'm', type: 'number', min: 0.1, max: 0.25, step: 0.01, group: 'Steering system', default: 0.15 },
  ],
  presets: [{ label: 'City 30 km/h', values: { speed: 30, steer: 4 } }, { label: 'Highway 70 km/h', values: { speed: 70, steer: 1 } }, { label: 'Strong gust 20 m/s', values: { gustU: 20 } }, { label: 'Wet road', values: { road: 'wet asphalt' } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const veh = ctx.vehicle, m = veh.mass, L = veh.wheelbase, u = kmh(p.speed), mu = ROADS[p.road].mu, rho = ctx.env?.rho ?? 1.225, S = veh.wingArea;
    const lay = wheelLayout(veh, p.frontFrac), { a, b } = lay, W = m * G0, Fzn = W / 3;
    const tad = veh.layout !== 'delta', nF = tad ? 2 : 1, nR = tad ? 1 : 2;
    const FzF = W * p.frontFrac, FzR = W - FzF, fzf = FzF / nF, fzr = FzR / nR;
    const kEff = (fz) => p.kAlpha * (fz / Fzn) ** (p.loadSens - 1);
    const Cf = nF * kEff(fzf) * fzf, Cr = nR * kEff(fzr) * fzr;
    const bi = bicycle({ m, Iz: p.Iz, a, b, Cf, Cr });
    const hs = veh.wheelRadius + veh.height * 0.7 + veh.wingHeight * 0.45;
    const tyreF = (al, fz, n) => n * mfLateral(al, fz, mu, { kAlpha: kEff(fz) });
    // generic nonlinear RHS
    const rhs = (delta, Fyw, Nw) => (t, y) => { const [vy, r, , , psi] = y, d = delta(t), aF = Math.atan2(vy + a * r, u) - d, aR = Math.atan2(vy - b * r, u); const FyF = tyreF(aF, fzf, nF), FyR = tyreF(aR, fzr, nR); return [(FyF * Math.cos(d) + FyR + Fyw(t)) / m - u * r, (a * FyF * Math.cos(d) - b * FyR + Nw(t)) / p.Iz, u * Math.cos(psi) - vy * Math.sin(psi), u * Math.sin(psi) + vy * Math.cos(psi), r]; };
    const T1 = 6, d0 = rad(p.steer);
    const step = rk45(rhs((t) => t >= 0.5 ? d0 : 0, () => 0, () => 0), 0, [0, 0, 0, 0, 0], T1, { rtol: 1e-7, atol: 1e-9, hmax: 0.01 });
    const ts = step.t, rr = step.y.map(y => y[1]), vys = step.y.map(y => y[0]), ay = step.y.map((y, i) => { const f = rhs((t) => t >= 0.5 ? d0 : 0, () => 0, () => 0)(ts[i], y); return (f[0] + u * y[1]); });
    const rSS = rr[rr.length - 1], rLin = bi.yawGain(u) * d0;
    const gustF = (t) => 0.5 * rho * S * p.Cs * oneMinusCos(t, 1, p.gustDur, p.gustU) ** 2;
    const cw = rk45(rhs(() => 0, gustF, (t) => gustF(t) * p.xcp), 0, [0, 0, 0, 0, 0], 8, { rtol: 1e-7, atol: 1e-9, hmax: 0.01 });
    const latDev = cw.y[cw.y.length - 1][3], headDev = deg(cw.y[cw.y.length - 1][4]), FsPeak = 0.5 * rho * S * p.Cs * p.gustU ** 2;
    // stability eigenvalues (linear 2-DOF)
    const A11 = -(Cf + Cr) / (m * u), A12 = -(a * Cf - b * Cr) / (m * u) - u, A21 = -(a * Cf - b * Cr) / (p.Iz * u), A22 = -(a * a * Cf + b * b * Cr) / (p.Iz * u);
    const tr = A11 + A22, dt_ = A11 * A22 - A12 * A21, disc = tr * tr / 4 - dt_, eig = disc >= 0 ? [tr / 2 + Math.sqrt(disc), tr / 2 - Math.sqrt(disc)].map(x => ({ re: x, im: 0 })) : [{ re: tr / 2, im: Math.sqrt(-disc) }, { re: tr / 2, im: -Math.sqrt(-disc) }];
    const stable = eig.every(e => e.re < 0);
    // rollover & cornering limits
    const ssf = trikeSSF(veh, p.frontFrac), aySteady = ctx.env ? 0.5 * rho * S * p.Cs * (ctx.env.Uz ?? 3) ** 2 : 0;
    const ayRollWind = rolloverAy(veh, m, { Fs: aySteady, hs, frontFrac: p.frontFrac }), ayRollGust = rolloverAy(veh, m, { Fs: FsPeak, hs, frontFrac: p.frontFrac });
    const ayGrip = Math.max(0, mu - aySteady / W), ayLim = Math.min(ayGrip, ayRollWind), vMax = Math.sqrt(ayLim * G0 * p.radius);
    const ayNow = max(ay.map(Math.abs)) / G0, loadsNow = threeWheelLoads(lay.pts, W, 0, [{ Fy: -m * ayNow * G0, z: veh.cgHeight }]);
    const ltr = tad ? Math.abs(loadsNow[0] - loadsNow[1]) / (loadsNow[0] + loadsNow[1]) : Math.abs(loadsNow[1] - loadsNow[2]) / (loadsNow[1] + loadsNow[2]);
    const rollMargin = 1 - ayNow / Math.max(1e-6, rolloverAy(veh, m, { frontFrac: p.frontFrac }));
    const FyFss = Cf * (d0 - (a * rSS) / u - (vys[vys.length - 1]) / u), effort = Math.abs(FyFss) * (p.trail / 1000) / p.steerRatio / p.rsw / 0.8;
    // checks
    out.checks.push({ name: 'Nonlinear steady yaw rate vs linear gain (small steer)', value: rSS, target: fmt(rLin, 4) + ' rad/s (linear)', pass: p.steer > 4 ? undefined : Math.abs(rSS - rLin) < 0.06 * Math.abs(rLin) + 1e-4, detail: 'Agreement verifies the implementation in the linear range.' });
    const zero = rk45(rhs(() => 0, () => 0, () => 0), 0, [0, 0, 0, 0, 0], 2); out.checks.push({ name: 'No input ⇒ no yaw (consistency)', value: zero.y[zero.y.length - 1][1], target: '0', pass: Math.abs(zero.y[zero.y.length - 1][1]) < 1e-12 });
    out.checks.push({ name: 'Integrator steps / rejections', value: `${step.steps} / ${step.rejected}`, target: 'adaptive', pass: true });
    // KPIs
    const k = (label, value, unit, explain, prov = 'calc', status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('Understeer gradient', bi.Kdeg, '°/g', 'Positive = understeer (stable), negative = oversteer. For a tadpole trike the lightly loaded front pair is stiffer per newton, which favours understeer.', 'calc', bi.Kdeg > 0 ? 'ok' : 'bad');
    k(bi.K > 0 ? 'Characteristic speed' : 'Critical speed', (bi.K > 0 ? bi.vch : bi.vcrit) * 3.6, 'km/h', bi.K > 0 ? 'Speed of maximum yaw-rate gain.' : 'Above this speed the linear vehicle is directionally unstable.', 'calc', bi.K > 0 ? 'ok' : (bi.vcrit * 3.6 > 100 ? 'warn' : 'bad'));
    k('Yaw-rate gain', bi.yawGain(u), '1/s', 'Steady r/δ at this speed (linear).');
    k('Steady yaw rate (nonlinear)', deg(rSS), '°/s', 'After the step steer settles.');
    k('Peak lateral acceleration', ayNow, 'g', 'During the step-steer manoeuvre.');
    k('Lateral load-transfer ratio', ltr, '–', 'ΔF_z / ΣF_z on the twin-wheel axle; 1 means wheel lift.', 'calc', ltr < 0.6 ? 'ok' : ltr < 0.9 ? 'warn' : 'bad');
    k('Static stability factor', ssf, 'g', 'Tipping-axis geometry of the trike.');
    k('Rollover threshold in current wind', ayRollWind, 'g', 'Lateral acceleration at wheel lift with the present wing side force acting outward.', 'calc', ayRollWind > 0.6 ? 'ok' : 'warn');
    k('Rollover threshold in design gust', ayRollGust, 'g', `With a ${p.gustU} m/s gust side force (${fmt(FsPeak, 3)} N).`, 'calc', ayRollGust > 0.3 ? 'ok' : 'bad');
    k('Max cornering speed on R = ' + p.radius + ' m', vMax * 3.6, 'km/h', 'Lower of grip-limited and rollover-limited speed, with adverse wing side force.');
    k('Crosswind lateral drift (8 s)', latDev, 'm', 'Hands-fixed lateral displacement after the gust — a measure of crosswind sensitivity.', 'calc', Math.abs(latDev) < 0.5 ? 'ok' : Math.abs(latDev) < 1.5 ? 'warn' : 'bad');
    k('Crosswind heading change', headDev, '°', 'Yaw angle change caused by the gust side force and its moment arm.');
    k('Steering hand force', effort, 'N', 'From aligning moment (trail × front lateral force) through the steering ratio.', 'calc', effort < 40 ? 'ok' : 'warn');
    // verdicts
    out.verdicts.push({ test: 'Directional stability (linear eigenvalues)', status: stable ? 'pass' : 'fail', detail: `λ = ${eig.map(e => fmt(e.re, 3) + (e.im ? ' ± ' + fmt(Math.abs(e.im), 3) + 'i' : '')).join(', ')} at ${p.speed} km/h.`, fix: 'Move mass forward on a tadpole (raise front fraction) or increase rear cornering stiffness (wider rear tyre).' });
    out.verdicts.push({ test: 'Rollover margin in manoeuvre', status: rollMargin > 0.3 ? 'pass' : rollMargin > 0 ? 'warn' : 'fail', detail: `${fmt(rollMargin * 100, 3)} % margin to inside-wheel lift at ${fmt(ayNow, 3)} g.` });
    out.verdicts.push({ test: 'Gust rollover resistance', status: ayRollGust > 0.3 ? 'pass' : ayRollGust > 0 ? 'warn' : 'fail', detail: `Threshold falls to ${fmt(ayRollGust, 3)} g under a ${p.gustU} m/s gust.`, fix: 'Wing feathering/unloading in gusts is required; lower wing height or widen the track.' });
    out.verdicts.push({ test: 'Crosswind sensitivity', status: Math.abs(latDev) < 0.5 ? 'pass' : Math.abs(latDev) < 1.5 ? 'warn' : 'fail', detail: `Drift ${fmt(latDev, 3)} m and heading ${fmt(headDev, 3)}° after a ${p.gustU} m/s, ${p.gustDur} s gust.`, fix: 'Move the side-force centre of pressure behind the CG (weathercock stability) and reduce side-force coefficient.' });
    // charts
    out.charts.push({ title: 'Step-steer response', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Response' }, series: [{ name: 'Yaw rate (°/s)', x: ts, y: rr.map(deg) }, { name: 'Lateral accel. (m/s²)', x: ts, y: ay }, { name: 'Sideslip β (°)', x: ts, y: vys.map(v => deg(Math.atan2(v, u))) }], explain: 'Transient after a step of road-wheel angle at 0.5 s. Overshoot and settling time characterise yaw damping; a diverging yaw rate would indicate instability.' });
    const traj = step.y; out.charts.push({ title: 'Trajectories: step steer and crosswind gust', type: 'line', x: { label: 'X (forward)', unit: 'm' }, y: { label: 'Y (left)', unit: 'm' }, series: [{ name: 'Step steer', x: traj.map(y => y[2]), y: traj.map(y => y[3]) }, { name: 'Gust, hands fixed', x: cw.y.map(y => y[2]), y: cw.y.map(y => y[3]) }], explain: 'Ground tracks of the two manoeuvres. The gust track shows how far the car is pushed off its lane without driver correction.' });
    out.charts.push({ title: 'Crosswind gust response', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Value' }, series: [{ name: 'Side force (kN)', x: cw.t, y: cw.t.map(t => gustF(t) / 1000) }, { name: 'Lateral offset (m)', x: cw.t, y: cw.y.map(y => y[3]) }, { name: 'Heading (°)', x: cw.t, y: cw.y.map(y => deg(y[4])) }], explain: 'A one-minus-cosine gust acting on the wing. The car responds with drift and yaw determined by tyre stiffness, mass, yaw inertia and the moment arm of the side force.' });
    const us = linspace(5, 120, 60); out.charts.push({ title: 'Yaw-rate gain vs speed', type: 'line', x: { label: 'Speed', unit: 'km/h' }, y: { label: 'r/δ', unit: '1/s' }, series: [{ name: 'This vehicle', x: us, y: us.map(s => { const g = bi.yawGain(kmh(s)); return Number.isFinite(g) && Math.abs(g) < 50 ? g : NaN; }) }, { name: 'Neutral steer (u/L)', x: us, y: us.map(s => kmh(s) / L), dash: '4 3' }], explain: 'Understeering vehicles fall below the neutral line and peak at the characteristic speed; oversteering vehicles rise toward infinity at the critical speed.' });
    const ayg = linspace(0, Math.min(0.9, mu), 40); out.charts.push({ title: 'Steady-state steer angle vs lateral acceleration (R = ' + p.radius + ' m)', type: 'line', x: { label: 'Lateral acceleration', unit: 'g' }, y: { label: 'Road-wheel angle', unit: '°' }, series: [{ name: 'Linear: L/R + K a_y', x: ayg, y: ayg.map(A => deg(L / p.radius + bi.K * A * G0)) }, { name: 'Ackermann L/R', x: ayg, y: ayg.map(() => deg(L / p.radius)), dash: '4 3' }], annotations: [{ type: 'vline', x: ayRollWind, label: 'wheel lift', color: '#ef4444' }], explain: 'The slope is the understeer gradient. For a trike the curve normally ends at wheel lift (red) before the tyres saturate.' });
    const Fss = linspace(0, Math.max(500, FsPeak * 1.2), 30); out.charts.push({ title: 'Rollover threshold vs aerodynamic side force', type: 'line', x: { label: 'Wing side force (outward)', unit: 'N' }, y: { label: 'Wheel-lift lateral acceleration', unit: 'g' }, series: [{ name: 'Threshold', x: Fss, y: Fss.map(F => rolloverAy(veh, m, { Fs: F, hs, frontFrac: p.frontFrac })) }], annotations: [{ type: 'vline', x: FsPeak, label: 'design gust', color: '#ef4444' }, { type: 'hline', y: 0.3, label: '0.3 g', color: '#f59e0b' }], explain: 'Every newton of outward side force at wing height uses up rollover margin. Where the line meets zero, the gust alone tips the car.' });
    out.tables.push({ title: 'Axle properties', cols: ['Axle', 'Tyres', 'Load (N)', 'Cornering stiffness (N/rad)', 'C_α / load (1/rad)'], rows: [['Front', nF, FzF, Cf, Cf / FzF], ['Rear', nR, FzR, Cr, Cr / FzR]] });
    out.tables.push({ title: 'Linear 2-DOF state matrix & eigenvalues', cols: ['', 'v', 'r'], rows: [['v̇', A11, A12], ['ṙ', A21, A22], ['eigenvalues', eig.map(e => fmt(e.re, 4) + (e.im ? (e.im > 0 ? ' + ' : ' − ') + fmt(Math.abs(e.im), 4) + 'i' : '')).join(' ; '), '']] });
    if (!stable || bi.K < 0) out.recs.push({ level: 'critical', text: 'The configuration oversteers. Shift mass toward the twin front wheels (tadpole) or fit a wider/stiffer single rear tyre before speed testing.', basis: 'Bicycle-model understeer gradient & eigenvalues' });
    if (ayRollGust < 0.3) out.recs.push({ level: 'critical', text: `A ${p.gustU} m/s gust leaves only ${fmt(ayRollGust, 3)} g of rollover margin. Restrict operation to lower wind speeds or add an automatic (passive) wing-feathering stop.`, basis: 'Rollover threshold with side force' });
    if (p.xcp > 0) out.recs.push({ level: 'warn', text: 'The wing side-force centre lies ahead of the CG, which yaws the car downwind in gusts. Moving it 0.2–0.4 m behind the CG gives weathercock stability.', basis: 'Yaw moment N = F_s x_cp' });
    out.recs.push({ level: 'good', text: 'Validate with ISO 4138 steady-state circle and ISO 7401 step-steer tests, then import the IMU logs here to calibrate cornering stiffness and yaw inertia.', basis: 'Specification validation hierarchy level 5' });
    out.explain.push({ title: 'Why a wind car is special', text: 'Unlike an ordinary car, ATMOS-XP carries a large lateral aerodynamic load all the time. That load consumes tyre grip and rollover margin before the driver even steers, and its gusts disturb the yaw motion. Handling must therefore be judged with the wing loads included.' });
    out.outputs = { understeerGradient: bi.Kdeg, criticalSpeed: bi.K < 0 ? bi.vcrit * 3.6 : Infinity, characteristicSpeed: bi.K > 0 ? bi.vch * 3.6 : Infinity, maxCornerSpeed_kmh: vMax * 3.6, rolloverMargin: rollMargin, ltr, ssf, rolloverGust_g: ayRollGust, crosswindDrift_m: latDev, stable };
    out.headline = `${bi.K > 0 ? 'Understeer' : 'Oversteer'} ${fmt(bi.Kdeg, 3)}°/g; ${stable ? 'directionally stable' : 'UNSTABLE'} at ${p.speed} km/h; max ${fmt(vMax * 3.6, 3)} km/h on R = ${p.radius} m; gust drift ${fmt(latDev, 3)} m.`;
    out.headlineKind = stable && ayRollGust > 0.3 ? 'info' : 'warn';
    void min; void mean; void interp1;
    return out;
  },
};
