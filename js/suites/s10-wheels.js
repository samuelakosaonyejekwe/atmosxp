// Suite 10 — Road wheels, tyres, bearings & tyre–road interaction.
import { G0, fmt, linspace, range, sum, max, rad, deg } from '../core/num.js';
import { kmh, moistAir, flatPlateH } from '../core/physics.js';
import { ROADS, mfLateral, mfLongitudinal, fiala, dugoff, crrModel, wheelLayout, threeWheelLoads } from '../core/vehicle.js';
import { h } from '../core/ui.js';
import { get as getMat } from '../core/materials.js';
import { vehicleAssembly } from '../core/cad.js';

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
    { key: 'width', label: 'Tyre section width', unit: 'mm', type: 'number', min: 25, max: 200, step: 1, group: 'Tyre', default: 40, help: 'Revision 4.1: narrow high-pressure low-loss tyres (velomobile practice) — rolling resistance dominates the light-wind power budget.' },
    { key: 'widthAuto', label: 'Auto-size tyre width to the load rating', type: 'bool', group: 'Tyre', default: true, help: 'Picks the narrowest standard width (ETRTO 35–80 mm) whose load rating covers 1.25 × the static wheel load and 1.15 × the outer-front load at the safe side-force limit. Rolling resistance is held at the narrow-tyre value (wider tyres at equal pressure roll no worse).' },
    { key: 'axleMat', label: 'Front axle / stub-arm material', type: 'select', group: 'Axle & rims', default: 'al-7075-t6', options: [{ value: 'al-7075-t6', label: 'Aluminium 7075-T6 tube' }, { value: 'ti-6al4v', label: 'Titanium Ti-6Al-4V tube' }, { value: 'st-4130', label: 'Steel 4130 tube' }, { value: 'cfrp-t700', label: 'CFRP T700 tube (±45/0 lay-up, 45 % of UD strength)' }] },
    { key: 'axleAuto', label: 'Auto-size the front axle tube', type: 'bool', group: 'Axle & rims', default: true, help: 'Lightest tube (Ø 25–80 mm, wall 1–6 mm) with static safety factor ≥ 2 on the worst load case, fatigue factor ≥ 1.5 and camber change ≤ 0.5°.' },
    { key: 'axleD', label: 'Axle tube outer diameter', unit: 'mm', type: 'number', min: 20, max: 100, step: 1, group: 'Axle & rims', default: 45 },
    { key: 'axleT', label: 'Axle tube wall', unit: 'mm', type: 'number', min: 0.8, max: 8, step: 0.1, group: 'Axle & rims', default: 3 },
    { key: 'spokeDepth', label: 'Rim spoke axial depth', unit: 'mm', type: 'number', min: 10, max: 60, step: 1, group: 'Axle & rims', default: 30, help: 'Six-spoke cast/forged wheel; spoke cross-section is thickness × axial depth.' },
    { key: 'spokeThk', label: 'Rim spoke thickness', unit: 'mm', type: 'number', min: 5, max: 30, step: 0.5, group: 'Axle & rims', default: 12 },
    { key: 'pInf', label: 'Inflation pressure', unit: 'kPa', type: 'range', min: 150, max: 900, step: 10, group: 'Tyre', default: 600 },
    { key: 'Crr0', label: 'Reference C_rr (450 kPa, ref. load)', type: 'number', min: 0.002, max: 0.03, step: 0.0005, group: 'Tyre', default: 0.006, auto: (c) => ({ value: c.vehicle.Crr, prov: 'user' }) },
    { key: 'kAlpha', label: 'Cornering stiffness / load', unit: '1/rad', type: 'number', min: 4, max: 25, step: 0.5, group: 'Tyre', default: 12, expert: true },
    { key: 'kKappa', label: 'Longitudinal stiffness / load', unit: '–', type: 'number', min: 6, max: 30, step: 0.5, group: 'Tyre', default: 18, expert: true },
    { key: 'Cy', label: 'Magic Formula shape C (lateral)', type: 'number', min: 1, max: 2, step: 0.05, group: 'Tyre', default: 1.3, expert: true },
    { key: 'Ey', label: 'Magic Formula curvature E (lateral)', type: 'number', min: -2, max: 1, step: 0.05, group: 'Tyre', default: -0.6, expert: true },
    { key: 'bearingAuto', label: 'Auto-select bearing for p₀ ≤ 1.5 GPa', type: 'bool', group: 'Bearings', default: true, help: 'Searches a deep-groove ball-bearing catalogue (62xx / 63xx series) for the smallest bearing whose peak Hertz pressure at the most loaded wheel is ≤ 1.5 GPa (Revision 2, D9).' },
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
    // ---- Structural load cases for tyres, rims and the front axle (Revision 5 audit) ----
    // A: 3 g vertical bump on the static wheel load. B: safe side-force limit (0.8 × 0.7 μ W, applied at CG height) × 1.5
    // dynamic factor, tyre lateral force = μ × wheel load. C: emergency braking μ F_z at 1.5 × static load (forward transfer).
    const Wt = veh.mass * G0, sideTot = 0.8 * 0.7 * mu * Wt, sideLoads = threeWheelLoads(lay.pts, Wt, 0, [{ Fy: -sideTot, z: veh.cgHeight }]);
    const iF = loads.indexOf(Math.max(...loads.slice(0, 2))), FzS = loads[iF], FzSide = Math.max(...sideLoads.slice(0, 2)), FzRear = loads[2];
    const tyreRating = (w, pk = p.pInf, D = 2 * R) => 1270 * (w / 40) ** 1.6 * (pk / 600) ** 0.6 * (D / 0.62) ** 0.6; // fit to ETRTO load tables for 35–80 mm high-pressure sizes
    const needTyre = Math.max(1.25 * FzS, 1.15 * FzSide), widthIn = p.width;
    if (p.widthAuto) { const W_ = [35, 40, 47, 50, 55, 62, 70, 80]; p.width = W_.find(w => tyreRating(w) >= needTyre) ?? 80; }
    const ratingT = tyreRating(p.width);
    const R0 = R + 0.12; // stub-arm height above ground (strut fairing line)
    // hard point = the real body half-width at the front-axle station (CAD body section); the shoulder pod fairs the arm out to the spat
    let axG = null; try { axG = vehicleAssembly(veh).meta.axle; } catch { axG = null; }
    const yHard = axG?.bodyHalfWidth ?? 0.9 * (veh.width / 2) * 0.82, Larm = Math.max(0.08, veh.track / 2 - yHard - 0.05);
    const amat = getMat(p.axleMat) || getMat('al-7075-t6'), isC = amat.cls === 'cfrp', sAllow = (isC ? 0.45 : 1) * amat.sy * 1e6, sFat = (isC ? 0.45 * 0.6 * amat.sy : amat.fatigue) * 1e6;
    const cases = [
      { name: 'A · 3 g bump', Fz: 3 * FzS, Fy: 0, Fx: 0 },
      { name: 'B · side-force limit × 1.5', Fz: 1.5 * FzSide, Fy: 1.5 * mu * FzSide * 0.7, Fx: 0 },
      { name: 'C · emergency braking', Fz: 1.5 * FzS, Fy: 0, Fx: mu * 1.5 * FzS },
    ];
    const tubeProps = (D, t) => { const Do = D / 1000, Di = Math.max(0, Do - 2 * t / 1000), I = Math.PI / 64 * (Do ** 4 - Di ** 4), J = 2 * I, A = Math.PI / 4 * (Do ** 2 - Di ** 2); return { I, J, A, c: Do / 2 }; };
    const axleEval = (D, t) => { const tp = tubeProps(D, t); let worst = null;
      for (const cs of cases) { const Mroll = cs.Fz * Larm + cs.Fy * R0, Myaw = cs.Fx * Larm, T = cs.Fx * R0, sig = Math.hypot(Mroll, Myaw) * tp.c / tp.I, tau = T * tp.c / tp.J, vm = Math.sqrt(sig * sig + 3 * tau * tau), SF = sAllow / vm; if (!worst || SF < worst.SF) worst = { ...cs, vm, SF }; }
      const sAmp = 0.5 * FzS * Larm * tp.c / tp.I, sMean = FzS * Larm * tp.c / tp.I, SFf = 1 / (sAmp / sFat + sMean / (amat.su * 1e6 * (isC ? 0.45 : 1))); // Goodman, 1 g ± 0.5 g
      const camber = deg((FzSide * Larm ** 2 / (2 * amat.E * 1e9 * tp.I)) + (FzSide * mu * 0.7 * R0 * Larm / (amat.E * 1e9 * tp.I)));
      return { D, t, ...tp, worst, SFf, camber, mass: 2 * tp.A * (Larm + 0.15) * amat.rho };
    };
    let axle = axleEval(p.axleD, p.axleT);
    if (p.axleAuto) { let best = null; for (let D = 25; D <= 80; D += 5) for (let t = 1; t <= 6; t += 0.5) { if (t > D / 6) continue; const e = axleEval(D, t); if (e.worst.SF >= 2 && e.SFf >= 1.5 && e.camber <= 0.5 && (!best || e.mass < best.mass)) best = e; } if (best) { axle = best; p.axleD = best.D; p.axleT = best.t; } }
    // six-spoke rim: out-of-plane bending at the spoke root from the tyre lateral force (2 spokes share the moment)
    const wmat = rim, FyWheel = 1.5 * mu * FzSide * 0.7, Mspoke = FyWheel * (R - 0.06) / 2, Zsp = (p.spokeThk / 1000) * (p.spokeDepth / 1000) ** 2 / 6, sigSp = Mspoke / Zsp, SFsp = (wmat.sy * 1e6) / sigSp;
    const sigSpB = 3 * FzS * 0.5 * (R - 0.06) * 0.15 / (2 * (p.spokeDepth / 1000) * (p.spokeThk / 1000) ** 2 / 6), SFspB = (wmat.sy * 1e6) / Math.max(sigSp, sigSpB);
    const structTbl = [['Outer-front tyre load at side-force limit', FzSide, 'N'], ['Tyre load rating (' + p.width + ' mm, ' + p.pInf + ' kPa)', ratingT, 'N'], ['Required rating', needTyre, 'N'], ['Front axle arm length (hard point → hub)', Larm * 1000, 'mm'], ['Axle tube', `Ø${axle.D} × ${axle.t} mm ${amat.name}`, ''], ['Axle governing case', axle.worst.name, ''], ['Axle static safety factor', axle.worst.SF, '×'], ['Axle fatigue factor (Goodman, 1 g ± 0.5 g)', axle.SFf, '×'], ['Camber change at side-force limit', axle.camber, '°'], ['Axle mass (both sides)', axle.mass, 'kg'], ['Rim spoke safety factor (lateral)', SFspB, '×'], ['Rear wheel load (static)', FzRear, 'N']];
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
    // Revision 2 (D9): automatic bearing selection for Hertz p₀ ≤ 1.5 GPa at the most loaded wheel (two bearings per wheel)
    const BRG = [['6205', 25, 52, 9, 7.94, 7.8], ['6305', 25, 62, 7, 11.5, 11.6], ['6206', 30, 62, 9, 9.53, 11.2], ['6306', 30, 72, 8, 12.3, 16], ['6207', 35, 72, 9, 11.11, 15.3], ['6307', 35, 80, 8, 13.5, 19], ['6208', 40, 80, 9, 12.7, 19], ['6308', 40, 90, 8, 15.08, 24], ['6209', 45, 85, 10, 12.7, 21.6], ['6210', 50, 90, 10, 12.7, 23.2], ['6309', 45, 100, 8, 17.46, 31.5], ['6310', 50, 110, 8, 19.05, 38]];
    const hertzP0 = (dBore, dOuter, Z, Dw, F) => { const dm_ = (dBore + dOuter) / 2, E_ = 208e9 / (1 - 0.09), rb = Dw / 2000, rin = (dm_ / 2 - Dw / 2) / 1000, Rx_ = 1 / (1 / rb + 1 / rin), Ry_ = 1 / (1 / rb - 1 / (0.52 * Dw / 1000)), Rp_ = 1 / (1 / Rx_ + 1 / Ry_), kE_ = 1.0339 * (Ry_ / Rx_) ** 0.636, e2 = 1.0003 + 0.5968 * Rx_ / Ry_, Qb = 5 * (F / 2) / Z, aa = Math.cbrt(6 * kE_ * kE_ * e2 * Qb * Rp_ / (Math.PI * E_)), bb = Math.cbrt(6 * e2 * Qb * Rp_ / (Math.PI * kE_ * E_)); return 3 * Qb / (2 * Math.PI * aa * bb); };
    const Fdesign = Math.max(p.Fz, ...loads);
    const brgTable = BRG.map(b => ({ name: b[0], d: b[1], D: b[2], Z: b[3], Dw: b[4], C0: b[5], p0: hertzP0(b[1], b[2], b[3], b[4], Fdesign) }));
    let brgChosen = null;
    if (p.bearingAuto) { brgChosen = brgTable.slice().sort((x, y) => x.D - y.D || x.d - y.d).find(b => b.p0 <= 1.5e9) || brgTable[brgTable.length - 1]; Object.assign(p, { dBore: brgChosen.d, dOuter: brgChosen.D, nBalls: brgChosen.Z, dBall: brgChosen.Dw, C0: brgChosen.C0, Fz: Math.max(p.Fz, Fdesign) }); }
    out.tables.push({ title: 'Bearing selection (deep-groove ball bearings, 2 per wheel, design load ' + fmt(Fdesign, 4) + ' N)', cols: ['Bearing', 'd (mm)', 'D (mm)', 'Balls', 'Ball Ø (mm)', 'C₀ (kN)', 'p₀ (MPa)', 'Status'], rows: brgTable.map(b => [b.name + (brgChosen && b.name === brgChosen.name ? ' ◀ selected' : ''), b.d, b.D, b.Z, b.Dw, b.C0, b.p0 / 1e6, b.p0 <= 1.5e9 ? 'OK ≤ 1.5 GPa' : 'too high']), explain: 'Peak Hertz pressure on the inner-race groove at the most loaded ball (Stribeck 5F_r/Z), groove conformity 0.52. Catalogue geometry is typical of the 62xx/63xx series; confirm with the supplier datasheet.' });
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
    out.verdicts.push({ test: 'Bearing contact stress', status: p0 < 1.5e9 ? 'pass' : p0 < 4.2e9 ? 'warn' : 'fail', detail: `p₀ = ${fmt(p0 / 1e6, 4)} MPa at ${fmt(p.Fz, 4)} N wheel load (elliptical contact, groove conformity 0.52)${brgChosen ? ` — selected bearing ${brgChosen.name} (${brgChosen.d}×${brgChosen.D} mm, ${brgChosen.Z} × ${brgChosen.Dw} mm balls)` : ''}.`, fix: 'Select a larger bearing (higher C₀) or more balls.' });
    out.verdicts.push({ test: 'Rolling resistance assumption', status: CrrEff <= veh.Crr * 1.1 ? 'pass' : 'warn', detail: `Effective ${fmt(CrrEff, 3)} vs global assumption ${veh.Crr}.`, fix: 'Raise inflation pressure, use a low-hysteresis silica compound, or update the global C_rr in Suite 01 so energy results stay honest.' });
    out.verdicts.push({ test: 'Tyre thermal state', status: dT < 25 ? 'pass' : 'warn', detail: `ΔT ≈ ${fmt(dT, 3)} K at ${p.speed} km/h.` });
    k('Tyre load margin', ratingT / Math.max(FzSide, 1), '×', `Load rating ${fmt(ratingT, 4)} N (${p.width} mm at ${p.pInf} kPa) ÷ outer-front load ${fmt(FzSide, 4)} N at the safe side-force limit.${p.widthAuto && p.width !== widthIn ? ` Auto-sized from ${widthIn} mm.` : ''}`, 'calc', ratingT >= needTyre ? 'ok' : 'bad');
    k('Front axle safety factor', axle.worst.SF, '×', `Ø${axle.D} × ${axle.t} mm ${amat.name}; governing case ${axle.worst.name} (von Mises ${fmt(axle.worst.vm / 1e6, 3)} MPa).`, 'calc', axle.worst.SF >= 2 ? 'ok' : axle.worst.SF >= 1.5 ? 'warn' : 'bad');
    k('Front axle fatigue factor', axle.SFf, '×', 'Goodman: mean 1 g, amplitude 0.5 g road loading for 10⁷ cycles.', 'calc', axle.SFf >= 1.5 ? 'ok' : 'warn');
    k('Rim spoke safety factor', SFspB, '×', `Six-spoke ${wmat.name} wheel, spokes ${p.spokeThk} × ${p.spokeDepth} mm, lateral tyre force ${fmt(FyWheel, 4)} N.`, 'calc', SFspB >= 2 ? 'ok' : SFspB >= 1.5 ? 'warn' : 'bad');
    out.verdicts.push({ test: 'Tyre load rating', status: ratingT >= needTyre ? 'pass' : 'fail', detail: `${p.width} mm tyre rated ${fmt(ratingT, 4)} N vs required ${fmt(needTyre, 4)} N (1.25 × static ${fmt(FzS, 4)} N; 1.15 × side-limit ${fmt(FzSide, 4)} N).`, fix: 'Use the auto-sized width or raise the pressure within the rim rating.' });
    out.verdicts.push({ test: 'Front axle strength, fatigue & camber', status: axle.worst.SF >= 2 && axle.SFf >= 1.5 && axle.camber <= 0.5 ? 'pass' : axle.worst.SF >= 1.5 ? 'warn' : 'fail', detail: `SF ${fmt(axle.worst.SF, 3)} (${axle.worst.name}), fatigue ${fmt(axle.SFf, 3)}, camber ${fmt(axle.camber, 3)}°, ${fmt(axle.mass, 3)} kg for both stub arms.`, fix: 'Enable auto-sizing, choose a stronger tube material or move the hard points outboard.' });
    out.verdicts.push({ test: 'Rim spokes', status: SFspB >= 2 ? 'pass' : SFspB >= 1.5 ? 'warn' : 'fail', detail: `Safety factor ${fmt(SFspB, 3)} on yield at the spoke root under ${fmt(FyWheel, 4)} N lateral and 3 g bump.`, fix: 'Deepen the spokes axially (bending ∝ depth²) or use a forged disc wheel.' });
    out.tables.push({ title: 'Tyre, rim & front-axle structural check (load cases A bump · B side limit · C braking)', cols: ['Item', 'Value', 'Unit'], rows: structTbl });
    out.explain.push({ title: 'Why the tyres and axle were re-sized', text: `In a strong side wind the wing pushes the car sideways and the outer front wheel carries most of the front load (${fmt(FzSide, 4)} N against ${fmt(FzS, 4)} N static). A narrow 40 mm tyre is rated ≈ ${fmt(tyreRating(40), 4)} N, so the width is auto-sized to ${p.width} mm. Each front wheel hangs on a ${fmt(Larm * 1000, 3)} mm stub arm from the body; it is sized for a 3 g bump, 1.5 × the side-force limit and full braking with a factor of 2 on yield.` });
    // tables
    out.tables.push({ title: 'Per-wheel results', cols: ['Wheel', 'F_z (N)', 'C_rr', 'Rolling force (N)', 'Bearing torque (N·m)', 'Bearing loss (W)', 'Lateral capacity (N)'], rows: loads.map((F, i) => [lay.pts[i][2], F, crrW[i], crrW[i] * F, 2 * bw[i].M, bw[i].P, mu * F]) });
    out.tables.push({ title: 'Magic Formula parameter set (export)', cols: ['Parameter', 'Lateral', 'Longitudinal'], rows: [['B (per rad / per unit slip)', p.kAlpha * p.Fz / (p.Cy * mu * p.Fz), p.kKappa * p.Fz / (1.65 * mu * p.Fz)], ['C', p.Cy, 1.65], ['D = μF_z (N)', mu * p.Fz, mu * p.Fz], ['E', p.Ey, 0.2]] });
    // recs
    if (latMargin < 2) out.recs.push({ level: 'critical', text: `Wing side force uses ${fmt(100 / latMargin, 3)} % of available lateral grip. Feather the wing in strong crosswinds and never brake hard on a wet road with the wing loaded.`, basis: 'Friction-circle budget' });
    if (p.pInf < 400) out.recs.push({ level: 'warn', text: `Inflation ${p.pInf} kPa raises C_rr by ≈ ${fmt(((p.pInf / 450) ** -0.4 - 1) * 100, 3)} % vs 450 kPa — directly enlarging the power gap the backup must close.`, basis: 'C_rr ∝ p^−0.4' });
    out.recs.push({ level: 'good', text: 'Measure C_rr with a coast-down (separating rolling from aerodynamic drag) on the actual route surface and import it here; it is one of the most sensitive parameters of the whole energy balance.', basis: 'Suite 25 sensitivity ranking' });
    out.recs.push({ level: 'sustain', text: 'Low-rolling-resistance silica tread and retreadable carcasses cut both energy demand and tyre-wear particle emissions; specify tyres with EU tyre-label class A rolling resistance where sizes allow.', basis: 'Rolling-resistance and life-cycle considerations' });
    out.explain.push({ title: 'Role of the tyres in a wind-driven car', text: 'In wind mode the tyres do not drive the car: they support it, react the wing’s large side force, and steer. In backup mode the driven wheels must also transmit hub torque. Grip, rolling losses and lateral capacity therefore set limits on every other suite.' });
    // 3D dynamic rolling-tyre close-up: brush-model contact patch solved at every frame of a slip-angle sweep
    out.views.push({ label: '3D rolling tyre & contact patch', icon: '🛞', render: async (el) => {
      const [{ viewer }, { playback, hud, legend3d }, { cmap }] = await Promise.all([import('../core/viz3d.js'), import('../core/sim3d.js'), import('../core/chart.js')]);
      const box = h('div'); el.append(box);
      const V = await viewer(box, { height: 460, grid: false }); const THREE = V.THREE;
      const rT = (p.width / 1000) / 2, Rc = R - rT, nu_ = 120, nv = 28, a = patchL / 2, Ca = p.kAlpha * p.Fz, cp = Ca / (2 * a * a), p0 = 1.5 * p.Fz / (2 * a * (p.width / 1000)) /* parabolic peak, Pa */;
      const kRad = p.pInf * 1000 * (p.width / 1000) * 2.2, defl = p.Fz / kRad; // radial deflection from inflation stiffness
      // brush model at slip angle al (rad): bristle deflection y(x), pressure q(x), adhesion/sliding split; x from +a (leading) to −a
      const brush = (al) => { const ta = Math.tan(Math.abs(al)), sg = Math.sign(al) || 1, N = 60; const xs = [], ys = [], qs = []; let Fy = 0, slide = 0; for (let k = 0; k <= N; k++) { const x = a - 2 * a * k / N, q = p0 * Math.max(0, 1 - (x / a) ** 2) * (p.width / 1000), yAd = (a - x) * ta, yMax = mu * q / cp; const y = Math.min(yAd, yMax); if (yAd > yMax) slide++; xs.push(x); ys.push(sg * y); qs.push(q); Fy += cp * y * (2 * a / N) * sg; } return { xs, ys, qs, Fy, slideFrac: slide / (N + 1) }; };
      // build tyre surface mesh (static topology; vertices updated per frame)
      const pos = new Float32Array(nu_ * nv * 3), col = new Float32Array(nu_ * nv * 3), idx = [];
      for (let i = 0; i < nu_; i++) for (let j = 0; j < nv; j++) { const i1 = (i + 1) % nu_, j1 = (j + 1) % nv; idx.push(i * nv + j, i1 * nv + j, i1 * nv + j1, i * nv + j, i1 * nv + j1, i * nv + j1); }
      const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3)); g.setAttribute('color', new THREE.BufferAttribute(col, 3)); g.setIndex(idx);
      const tyreMesh = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.85, side: THREE.DoubleSide })); V.root.add(tyreMesh);
      const rimG = new THREE.Group(); V.root.add(rimG);
      const rimMat = new THREE.MeshStandardMaterial({ color: 0xc0c6cf, metalness: 0.7, roughness: 0.3 });
      const rimRing = new THREE.Mesh(new THREE.TorusGeometry(Rc - rT * 0.9, rT * 0.25, 12, 64), rimMat); rimRing.rotation.x = Math.PI / 2; rimG.add(rimRing);
      for (let k2 = 0; k2 < 6; k2++) { const sp = new THREE.Mesh(new THREE.BoxGeometry(Rc * 0.85, 0.012, 0.03), rimMat); sp.position.x = Rc * 0.42; const pv = new THREE.Group(); pv.rotation.y = k2 * Math.PI / 3; pv.add(sp); rimG.add(pv); }
      const hubC = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 0.07, 24), new THREE.MeshStandardMaterial({ color: 0x64748b, metalness: 0.6 })); rimG.add(hubC);
      rimG.position.z = R; // wheel axis along y
      const road = new THREE.Mesh(new THREE.PlaneGeometry(4, 2), new THREE.MeshStandardMaterial({ color: 0x374151, roughness: 0.95 })); road.position.z = -0.0005; V.root.add(road);
      const rgrid = new THREE.GridHelper(4, 80, 0x9ca3af, 0x6b7280); rgrid.rotation.x = Math.PI / 2; rgrid.position.z = 0.0005; V.root.add(rgrid);
      const fyArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0.01), 0.3, 0xef4444, 0.06, 0.04); V.root.add(fyArrow);
      const fzArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0.01), 0.3, 0x22c55e, 0.06, 0.04); V.root.add(fzArrow);
      const vArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, R * 2 + 0.05), 0.3, 0x38bdf8, 0.06, 0.04); V.root.add(vArrow);
      // enlarged contact-patch footprint beside the wheel (×FP scale) coloured by solved pressure, displaced by bristle deflection
      const FP = Math.min(12, 0.18 / (2 * a)), nfx = 48, nfy = 12, fpos = new Float32Array(nfx * nfy * 3), fcol = new Float32Array(nfx * nfy * 3), fidx = [];
      for (let i = 0; i < nfx - 1; i++) for (let j = 0; j < nfy - 1; j++) { const q0 = i * nfy + j; fidx.push(q0, q0 + nfy, q0 + nfy + 1, q0, q0 + nfy + 1, q0 + 1); }
      const fg = new THREE.BufferGeometry(); fg.setAttribute('position', new THREE.BufferAttribute(fpos, 3)); fg.setAttribute('color', new THREE.BufferAttribute(fcol, 3)); fg.setIndex(fidx);
      const fmesh = new THREE.Mesh(fg, new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.DoubleSide })); V.root.add(fmesh);
      const fy0 = -0.32; const fpLabel = h('div', { class: 'sim-legend', style: 'bottom:auto;top:40px;left:8px;right:auto;width:230px' }, h('div', { class: 'small' }, `Contact-patch footprint (enlarged ×${fmt(FP, 3)}), leading edge at +x; lateral shift = bristle deflection`)); box.append(fpLabel);
      V.controls.target.set(0, -0.12, R * 0.45); V.camera.position.set(0.75, -1.05, 0.5); V.controls.update();
      const qMax = p0 * (p.width / 1000);
      legend3d(box, { title: 'Contact pressure (line load, brush model)', lo: 0, hi: qMax / 1000, unit: ' kN/m', cmapName: 'turbo' });
      const H = hud(box);
      const T = 16, alMax = 10, omega = v / R; let mag = 30;
      const magLab = h('span', { class: 'small' }, ''), magIn = h('input', { type: 'range', min: 1, max: 100, value: mag, oninput: (e) => { mag = +e.target.value; } });
      const pb = playback(box, { duration: T, rate: 1, label: 'Manoeuvre time', onFrame: (t) => {
        const alDeg = alMax * Math.sin(2 * Math.PI * t / 8), al = rad(alDeg), B = brush(al), FyMF = -mfLateral(al, p.Fz, mu, opts), rot = omega * t;
        const yMaxAbs = Math.max(...B.ys.map(Math.abs)); magLab.textContent = `Deformation magnification ×${mag} (true-scale peak tread deflection = ${fmt(yMaxAbs * 1000, 3)} mm, radial ${fmt(defl * 1000, 3)} mm)`;
        for (let i = 0; i < nu_; i++) { const u = 2 * Math.PI * i / nu_; for (let j = 0; j < nv; j++) { const w = 2 * Math.PI * j / nv; let x = (Rc + rT * Math.cos(w)) * Math.cos(u), z = R + (Rc + rT * Math.cos(w)) * Math.sin(u), y = rT * Math.sin(w) * 1.0; let q = 0;
          // flatten the bottom into the contact patch (deflection magnified) and add lateral bristle deflection
          const flat = Math.max(0, defl * Math.min(mag, 3));
          if (z < flat) { z = flat * (z <= 0 ? 1 : 1); }
          if (Math.abs(x) <= a * Math.max(1, Math.min(mag, 3) ** 0.5) && z <= flat + rT * 0.6) { const xx = Math.max(-a, Math.min(a, x)), kk = Math.round((a - xx) / (2 * a) * (B.xs.length - 1)); const wt = Math.max(0, 1 - (z - flat) / (rT * 0.6)); y += mag * B.ys[kk] * wt; if (z <= flat + 1e-4 || wt > 0.85) q = B.qs[kk]; }
          const k = (i * nv + j) * 3; pos[k] = x; pos[k + 1] = y; pos[k + 2] = Math.max(flat, z) ;
          // tread pattern rotating with the wheel (shows rolling) blended with pressure colour in the patch
          const tread = (Math.floor(((u + rot) % (2 * Math.PI) + 2 * Math.PI) / (2 * Math.PI) * 48) % 2) ? 0.10 : 0.16;
          if (q > 0) { const c = cmap(q / qMax, 'turbo'); col[k] = c[0] / 255; col[k + 1] = c[1] / 255; col[k + 2] = c[2] / 255; } else { col[k] = col[k + 1] = col[k + 2] = Math.abs(Math.cos(w)) > 0.55 ? tread : 0.07; }
        } }
        g.attributes.position.needsUpdate = true; g.attributes.color.needsUpdate = true; g.computeVertexNormals();
        for (let i = 0; i < nfx; i++) { const kk = Math.round(i / (nfx - 1) * (B.xs.length - 1)), x = B.xs[kk]; for (let j = 0; j < nfy; j++) { const eta = -1 + 2 * j / (nfy - 1), lat = Math.sqrt(Math.max(0, 1 - eta * eta)), k = (i * nfy + j) * 3; fpos[k] = x * FP; fpos[k + 1] = fy0 + eta * (p.width / 2000) * FP + B.ys[kk] * FP * Math.min(mag, 10) / 3; fpos[k + 2] = 0.003; const c = cmap(B.qs[kk] * lat / qMax, 'turbo'); fcol[k] = c[0] / 255; fcol[k + 1] = c[1] / 255; fcol[k + 2] = c[2] / 255; } }
        fg.attributes.position.needsUpdate = true; fg.attributes.color.needsUpdate = true;
        rimG.rotation.y = rot;
        const fy = B.Fy; fyArrow.setDirection(new THREE.Vector3(0, Math.sign(fy) || 1, 0)); fyArrow.setLength(Math.max(0.02, 0.45 * Math.abs(fy) / (mu * p.Fz)), 0.06, 0.04);
        fzArrow.setLength(0.35, 0.06, 0.04);
        H.set([['Time', fmt(t, 3) + ' s'], ['Speed', fmt(p.speed, 3) + ' km/h'], ['Wheel speed', fmt(omega * 60 / (2 * Math.PI), 4) + ' rpm'], ['Slip angle α', fmt(alDeg, 3) + '°'], ['F_y brush model', fmt(Math.abs(B.Fy), 4) + ' N'], ['F_y Magic Formula', fmt(Math.abs(FyMF), 4) + ' N'], ['Sliding fraction of patch', fmt(100 * B.slideFrac, 3) + ' %', B.slideFrac > 0.5 ? '#ef4444' : undefined], ['Patch length', fmt(2 * a * 1000, 4) + ' mm'], ['Peak pressure', fmt(p0 / 1000, 4) + ' kPa'], ['Wheel load F_z', fmt(p.Fz, 4) + ' N'], ['Rolling resistance power', fmt(crrW[0] * loads[0] * v, 3) + ' W']]);
      } });
      void pb;
      el.append(h('div', { class: 'row gap wrap', style: 'margin:6px 2px' }, magLab, magIn), h('p', { class: 'small muted' }, `Solved at every frame: brush-model contact patch for the current slip angle (bristle stiffness from the cornering stiffness, parabolic pressure, adhesion/sliding split with μ = ${mu}), lateral force (brush and Magic Formula), and radial deflection from the inflation stiffness. The slip angle follows a prescribed ±${alMax}° sinusoidal sweep at ${p.speed} km/h; the wheel rotates at the solved ω = v/R. Deformations are magnified (slider) — true-scale values are printed. Red arrow = lateral force, green = wheel load, blue = travel direction.`));
    } });
    out.views.push({ label: '2D animation: contact-patch pressure & slip', icon: '🛞', render: async (el) => {
      const A = await import('../core/anim2d.js'), { cmap } = await import('../core/chart.js');
      const a = patchL / 2, wP = p.width / 1000, Ca = p.kAlpha * p.Fz, cpB = Ca / (2 * a * a), p0 = 1.5 * p.Fz / (2 * a * wP);
      const brush = (al) => { const ta = Math.tan(Math.abs(al)), sg = Math.sign(al) || 1, N = 60, out2 = []; let Fy = 0, xs = null; for (let k2 = 0; k2 <= N; k2++) { const x = a - 2 * a * k2 / N, q = p0 * Math.max(0, 1 - (x / a) ** 2) * wP, yAd = (a - x) * ta, yMax = mu * q / cpB, sl = yAd > yMax; if (sl && xs === null) xs = x; const y = Math.min(yAd, yMax); out2.push({ x, q, y: sg * y, sl }); Fy += cpB * y * (2 * a / N) * sg; } return { pts: out2, Fy, xs }; };
      const alMax = 10, als = linspace(-alMax, alMax, 81), FyB = als.map(d => brush(rad(d)).Fy), FyM = als.map(d => -mfLateral(rad(d), p.Fz, mu, opts));
      const qMax = p0 * wP, yPk = Math.max(...brush(rad(alMax)).pts.map(q => Math.abs(q.y))), MG = Math.max(1, Math.round(0.3 * wP / Math.max(1e-9, yPk)));
      A.anim2d(el, { duration: 16, rate: 1, height: 430, label: 'Manoeuvre time', unit: 's', draw: (g, t, W, H) => {
        const alDeg = alMax * Math.sin(2 * Math.PI * t / 8), B = brush(rad(alDeg));
        const px = 16, py = 16, pw = W * 0.55, ph = H - 32, sx = (pw - 60) / (2 * a), sy = Math.min(sx, (ph - 80) / (wP * 1.6));
        g.fillStyle = A.theme().bg2; g.fillRect(px, py, pw, ph);
        const cx = px + pw / 2, cy = py + ph / 2;
        // pressure map: columns along x, elliptic lateral profile
        for (const q of B.pts) for (let j = 0; j <= 20; j++) { const eta = -1 + j / 10, lat = Math.sqrt(Math.max(0, 1 - eta * eta)), c = cmap(q.q * lat / qMax, 'turbo'); g.fillStyle = `rgb(${c.map(Math.round).join(',')})`; g.fillRect(cx + q.x * sx - sx * a / 30, cy + eta * wP / 2 * sy - sy * wP / 40 + q.y * sy * MG, sx * a / 15 + 1, sy * wP / 20 + 1); }
        // bristle deflection arrows (magnified ×25) and sliding boundary
        for (let k2 = 0; k2 < B.pts.length; k2 += 4) { const q = B.pts[k2]; A.arrow(g, cx + q.x * sx, cy, 0, q.y * sy * MG, q.sl ? '#ef4444' : '#e5e7eb', 1.5); }
        if (B.xs !== null) { g.strokeStyle = '#ef4444'; g.setLineDash([5, 4]); g.beginPath(); g.moveTo(cx + B.xs * sx, py + 30); g.lineTo(cx + B.xs * sx, py + ph - 20); g.stroke(); g.setLineDash([]); A.text(g, 'sliding | adhesion', cx + B.xs * sx - 52, py + 40, '#ef4444'); }
        A.arrow(g, px + 20, py + ph - 20, 60, 0, '#38bdf8', 2.5, 'rolling direction'); A.text(g, `Contact patch (top view) · α = ${fmt(alDeg, 3)}° · leading edge right · deflection ×${MG} (true peak ${fmt(Math.max(...B.pts.map(q => Math.abs(q.y))) * 1000, 3)} mm)`, px + 8, py + 16);
        // Fy–α curve with moving point
        const qx = px + pw + 14, qw = W - qx - 12, qh = ph * 0.62, X = (d) => qx + 30 + (d + alMax) / (2 * alMax) * (qw - 40), Ymax = Math.max(...FyM.map(Math.abs), ...FyB.map(Math.abs)), Y = (F) => py + qh / 2 - F / Ymax * (qh / 2 - 20);
        g.fillStyle = A.theme().bg2; g.fillRect(qx, py, qw, qh); A.text(g, 'Lateral force vs slip angle (N, °)', qx + 6, py + 14);
        for (const [arr, c] of [[FyM, '#a855f7'], [FyB, '#22c55e']]) { g.strokeStyle = c; g.lineWidth = 2; g.beginPath(); arr.forEach((F, j) => { j ? g.lineTo(X(als[j]), Y(F)) : g.moveTo(X(als[j]), Y(F)); }); g.stroke(); }
        g.lineWidth = 1; g.fillStyle = '#22c55e'; g.beginPath(); g.arc(X(alDeg), Y(B.Fy), 6, 0, 7); g.fill();
        A.text(g, 'Magic Formula', qx + 8, py + qh - 26, '#a855f7'); A.text(g, 'Brush model', qx + 8, py + qh - 10, '#22c55e');
        const sl = B.pts.filter(q => q.sl).length / B.pts.length;
        A.bar(g, { x: qx, y: py + qh + 14, w: qw, value: Math.abs(B.Fy), max: mu * p.Fz, label: 'F_y (brush)', unit: ' N', color: '#22c55e' });
        A.bar(g, { x: qx, y: py + qh + 34, w: qw, value: sl * 100, max: 100, label: 'Sliding fraction', unit: ' %', color: '#ef4444' });
        A.bar(g, { x: qx, y: py + qh + 54, w: qw, value: qMax * 1.0 / 1000, max: qMax / 1000, label: 'Peak line load', unit: ' kN/m', color: '#eab308' });
      }, explain: 'Brush-model contact patch solved at each instant of a ±10° slip-angle sweep: pressure distribution (colour), bristle lateral deflection (arrows, magnified as labelled), adhesion/sliding boundary (red) and the resulting lateral force traced on the brush and Magic-Formula curves.' });
    } });
    out.outputs = { tyreWidth_mm: p.width, tyreRating_N: ratingT, axleD_mm: axle.D, axleT_mm: axle.t, axleMass_kg: axle.mass, axleSF: axle.worst.SF, axleFatigueSF: axle.SFf, spokeSF: SFspB, FzSideLimit_N: FzSide, Crr: CrrEff, rollingForce_N: Froll, muPeak: mu, Fymax_N: FyCap, FxDrivenMax_N: FxCap, bearingLoss_W: Pbear, wheelInertia: Jw, rotEqMass: mRotEq, contactPressure_kPa: pc, kAlpha: p.kAlpha, Calpha_N_rad: p.kAlpha * p.Fz, road: p.road, muLock: road.muLock };
    out.headline = `${p.road}: μ = ${mu}, effective C_rr ${fmt(CrrEff, 3)}, rolling ${fmt(Froll * v, 3)} W + bearings ${fmt(Pbear, 3)} W at ${p.speed} km/h; lateral grip ${fmt(FyCap, 4)} N vs wing side force ${fmt(Fs, 4)} N.`;
    void deg; void range;
    return out;
  },
};
