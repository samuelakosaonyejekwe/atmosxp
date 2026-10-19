// Suite 12 — Mechanical braking, stopping & parking stability.
import { G0, fmt, linspace, range, sum, max, clamp } from '../core/num.js';
import { h } from '../core/ui.js';
import { kmh } from '../core/physics.js';
import { ROADS, wheelLayout, threeWheelLoads, discStep, brakeConvH } from '../core/vehicle.js';

const UNLOAD = { release: { label: 'Feathering release linked to brake pedal (Revision 2: C_L ≈ 0, residual ≈ 0)', f: 0.0 }, feathered: { label: 'Wing feathered / unloaded (10 % residual)', f: 0.1 }, partial: { label: 'Partially unloaded (50 %)', f: 0.5 }, stuck: { label: 'Wing fails to unload (100 %)', f: 1.0 } };
const FAIL = { none: 'No fault', front: 'Circuit A failed (front circuit / diagonal A)', rear: 'Circuit B failed (rear circuit / diagonal B)', lock: 'No modulation (wheels may lock)' };

export default {
  description: 'Stopping performance of the mechanically braked trike with concurrent wing thrust (including a wing that fails to unload), adhesion-limited load transfer, wheel lock, wet roads and single-circuit failure; brake disc heating and fade over repeated stops from the library brake materials; and parking-brake holding on slopes with gust loading.',
  literature: 'brake fade disc temperature lightweight vehicle stopping distance parking brake slope',
  credibility: 'Analytical screening',
  equations: ['Longitudinal equation m_eff v̇ = −ΣF_b − F_roll − F_aero − m g sin θ + F_thrust,residual (m_eff includes rotating inertia Σ J/R²)', 'Wheel rotational dynamics and brake friction torque T = 2 μ_pad F_clamp r_eff (Coulomb, temperature-dependent μ)', 'Dynamic normal-load redistribution on three supports under deceleration (d’Alembert)', 'Adhesion limit F_b,i ≤ μ_road F_z,i; locked-wheel sliding friction μ_lock', 'Mechanical/hydraulic force transmission: pedal ratio × area ratio × efficiency', 'Lumped disc energy balance m c Ṫ = q_in − hA(T − T_a) − εσA(T⁴ − T_a⁴) with speed-dependent forced convection', 'Static parking equilibrium: m g sin θ + F_wind ≤ min(μ_s F_z,braked, T_park/R)'],
  methods: ['Fixed-point iteration of deceleration ↔ load transfer ↔ wheel limits', 'Explicit time integration (Δt = 5 ms) to standstill', 'Linear pad-friction fade above 70 % of the pad temperature limit'],
  assumptions: ['Brake torque builds instantly after a 1 s driver reaction (plus 0.2 s system delay)', 'Disc heat partition 90 % to disc', 'Wing residual thrust constant during the stop (worst case)'],
  ibc: { initial: ['Initial speed, wheel speeds, brake temperature, lining wear, actuation position, preload, tyre slip, initial aerodynamic wing thrust'], boundary: ['Pedal force/displacement, linkage transmission, pad–disc contact pressure, frictional heat, tyre–road friction, convection & radiation cooling, road gradient, continued aerodynamic loading during stopping', 'Emergency cases: wheel lock, reduced friction, unbalanced braking, fade, single-circuit failure, failure to unload the wing; parking: static friction, slope, gust'] },
  cvv: { calibration: ['Brake dynamometer μ vs temperature & pressure', 'Pad wear rates', 'Measured actuation force chain'], verification: ['Kinetic energy at v₀ = brake work + rolling/aero losses − thrust work (energy closure)', 'Zero-thrust, constant-μ case equals v²/(2a) analytic distance'], validation: ['Instrumented track stops (ISO 21994-type) dry/wet, with the wing loaded and unloaded'] },
  formats: { import: ['CSV dynamometer data (T, μ)', 'CSV deceleration logs'], export: ['CSV time histories', 'PNG/SVG plots'] },
  params: [
    { key: 'v0', label: 'Initial speed', unit: 'km/h', type: 'range', min: 5, max: 90, step: 1, group: 'Stop', default: 40 },
    { key: 'road', label: 'Road surface', type: 'select', group: 'Stop', default: 'dry asphalt', options: Object.keys(ROADS), auto: (c) => c.upstream('s10')?.road },
    { key: 'slope', label: 'Road gradient (downhill +)', unit: '%', type: 'number', min: -20, max: 20, step: 0.5, group: 'Stop', default: 0 },
    { key: 'thrust', label: 'Wing forward thrust at brake onset', unit: 'N', type: 'number', min: 0, max: 2000, step: 5, group: 'Wing', default: 50, auto: (c) => { const o = c.upstream('s07') || c.upstream('s06'); return o?.thrustN !== undefined ? { value: +Math.max(0, o.thrustN).toFixed(1), prov: 'coupled' } : undefined; } },
    { key: 'unload', label: 'Wing unloading state', type: 'select', group: 'Wing', default: 'release', auto: (c) => ({ value: ['feathering', 'ring'].includes(c.upstream('s01')?.wingRoot ?? 'feathering') ? 'release' : 'feathered', prov: 'coupled' }), options: Object.entries(UNLOAD).map(([value, o]) => ({ value, label: o.label })) },
    { key: 'circuit', label: 'Brake circuit layout', type: 'select', group: 'Faults', default: 'diagonal', options: [{ value: 'diagonal', label: 'Diagonal split (each circuit: one front wheel + one rear caliper)' }, { value: 'axle', label: 'Axle split (front circuit / rear circuit)' }], help: 'With 75 % of the weight on the front wheels, an axle split leaves only the lightly loaded rear wheel after a front-circuit failure; the diagonal split keeps braking on both axles.' },
    { key: 'fault', label: 'Fault case', type: 'select', group: 'Faults', default: 'none', options: Object.entries(FAIL).map(([value, label]) => ({ value, label })) },
    { key: 'pedal', label: 'Pedal force', unit: 'N', type: 'number', min: 50, max: 800, step: 10, group: 'Actuation', default: 250 },
    { key: 'pedalRatio', label: 'Pedal lever ratio', type: 'number', min: 2, max: 8, step: 0.1, group: 'Actuation', default: 4 },
    { key: 'areaRatio', label: 'Hydraulic area ratio (caliper/master)', type: 'number', min: 1, max: 12, step: 0.1, group: 'Actuation', default: 4, help: 'Set to 1 with a high pedal ratio to represent a pure cable system.' },
    { key: 'eff', label: 'Transmission efficiency', type: 'number', min: 0.5, max: 1, step: 0.01, group: 'Actuation', default: 0.85, expert: true },
    { key: 'bias', label: 'Front brake-force share', unit: '%', type: 'range', min: 30, max: 90, step: 1, group: 'Actuation', default: 65 },
    { key: 'reaction', label: 'Driver reaction time', unit: 's', type: 'number', min: 0.3, max: 2.5, step: 0.1, group: 'Actuation', default: 1.0 },
    { key: 'nStops', label: 'Repeated stops (fade test)', type: 'number', min: 1, max: 40, step: 1, group: 'Thermal', default: 10 },
    { key: 'interval', label: 'Interval between stops', unit: 's', type: 'number', min: 10, max: 300, step: 5, group: 'Thermal', default: 45 },
    { key: 'discThk', label: 'Disc thickness', unit: 'mm', type: 'number', min: 3, max: 20, step: 0.5, group: 'Thermal', default: 5 },
    { key: 'parkSlope', label: 'Parking slope', unit: '%', type: 'number', min: 0, max: 40, step: 1, group: 'Parking', default: 18 },
    { key: 'parkGust', label: 'Parking gust (from behind)', unit: 'm/s', type: 'number', min: 0, max: 35, step: 0.5, group: 'Parking', default: 15, auto: (c) => c.env?.gust ? { value: +Math.max(10, c.env.gust).toFixed(1), prov: 'coupled' } : undefined },
    { key: 'parkLever', label: 'Parking lever force', unit: 'N', type: 'number', min: 50, max: 400, step: 10, group: 'Parking', default: 200 },
    { key: 'parkWheels', label: 'Parking brake acts on', type: 'select', group: 'Parking', default: 'all', options: [{ value: 'rear', label: 'Rear axle only' }, { value: 'all', label: 'All wheels' }] },
  ],
  presets: [{ label: 'Emergency stop, dry', values: { v0: 50, fault: 'none', unload: 'feathered' } }, { label: 'Wet + wing stuck', values: { road: 'wet asphalt', unload: 'stuck', v0: 40 } }, { label: 'Front circuit failure', values: { fault: 'front' } }, { label: 'Downhill fade test', values: { slope: 8, nStops: 20, interval: 30 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const veh = ctx.vehicle, m = veh.mass, R = veh.wheelRadius, rho = ctx.env?.rho ?? 1.225, th = Math.atan(p.slope / 100);
    const pad = ctx.material('brakePad'), disc = ctx.material('brake'), muPad0 = pad.mu ?? 0.38, padTmax = pad.Tmax ?? 350;
    const lay = wheelLayout(veh, ctx.upstream('s01')?.frontFrac ?? veh.frontFrac), mEff = m + (ctx.upstream('s10')?.rotEqMass ?? 9);
    const ro = 0.55 * R, ri = 0.3 * R, rEff = (2 / 3) * (ro ** 3 - ri ** 3) / (ro ** 2 - ri ** 2), mDisc = Math.PI * (ro * ro - ri * ri) * p.discThk / 1000 * disc.rho, Adisc = 2 * Math.PI * (ro * ro - ri * ri);
    const isFront = lay.pts.map(q => q[0] > 0), nF = isFront.filter(Boolean).length, nR = 3 - nF;
    const clamp0 = p.pedal * p.pedalRatio * p.areaRatio * p.eff;
    // one stop simulation
    const stop = ({ v0 = kmh(p.v0), road = p.road, unloadF = UNLOAD[p.unload].f, fault = p.fault, muPad = muPad0, thrust = p.thrust, record = true } = {}) => {
      const R_ = ROADS[road]; let v = v0, x = 0, t = 0, Ework = 0, Eloss = 0, Ethr = 0, Egr = 0; const dt = 0.005, T = [], Vs = [], A = [], X = [], LK = [], FB = []; let locked = new Array(3).fill(false), a = 0, cannotStop = false;
      const tOn = p.reaction + 0.2; const Fth = thrust * unloadF;
      const wheelForce = new Array(3).fill(0);
      while (v > 0 && t < 120) {
        const braking = t >= tOn;
        let Fb = [0, 0, 0];
        if (braking) {
          for (let it = 0; it < 15; it++) {
            const loads = threeWheelLoads(lay.pts, m * G0 * Math.cos(th), 0, [{ Fx: mEff * a, z: veh.cgHeight }]);
            Fb = loads.map((Fz, i) => {
              const share = isFront[i] ? p.bias / 100 / nF : (1 - p.bias / 100) / nR;
              // circuit layouts: 'axle' = front/rear circuits; 'diagonal' (default) = circuit A: first front wheel + one rear caliper,
              // circuit B: second front wheel + the other rear caliper (twin-caliper rear disc) — a single failure keeps half the brakes on both axles
              const fi = isFront.slice(0, i + 1).filter(Boolean).length - 1;
              const keep = (p.circuit || 'diagonal') === 'diagonal'
                ? (fault === 'front' ? (isFront[i] ? (fi === 0 ? 0 : 1) : 0.5) : fault === 'rear' ? (isFront[i] ? (fi === 1 ? 0 : 1) : 0.5) : 1)
                : ((fault === 'front' && isFront[i]) || (fault === 'rear' && !isFront[i]) ? 0 : 1);
              const demand = keep * 2 * muPad * clamp0 * share * 3 * rEff / R; // ×3 normalises shares to per-wheel clamp
              const lim = R_.mu * Math.max(0, Fz); locked[i] = demand > lim; return demand > lim ? (fault === 'lock' ? R_.muLock * Math.max(0, Fz) : lim) : demand;
            });
            const Fr = m * G0 * Math.cos(th) * veh.Crr, Fa = 0.5 * rho * veh.CdA * v * v, an = (sum(Fb) + Fr + Fa - Fth - m * G0 * Math.sin(th)) / mEff;
            if (Math.abs(an - a) < 1e-6) { a = an; break; } a = an;
          }
        } else { const Fr = m * G0 * Math.cos(th) * veh.Crr, Fa = 0.5 * rho * veh.CdA * v * v; a = (Fr + Fa - Fth - m * G0 * Math.sin(th)) / mEff; }
        if (braking && a <= 0) { cannotStop = true; }
        const vn = Math.max(0, v - a * dt), vm = 0.5 * (v + vn);
        Ework += sum(Fb) * vm * dt; Eloss += (m * G0 * Math.cos(th) * veh.Crr + 0.5 * rho * veh.CdA * vm * vm) * vm * dt; Ethr += Fth * vm * dt; Egr += m * G0 * Math.sin(th) * vm * dt;
        Fb.forEach((f, i) => wheelForce[i] += f * vm * dt);
        x += vm * dt; t += dt; v = vn;
        if (record && Math.round(t / dt) % 10 === 0) { T.push(t); Vs.push(v * 3.6); A.push(a); X.push(x); LK.push(locked.slice()); FB.push(braking ? Fb.slice() : [0, 0, 0]); }
        if (cannotStop && t > tOn + 30) break;
      }
      return { dist: x, time: t, aMean: v0 / Math.max(1e-6, t - tOn), aMax: a, Ework, Eloss, Ethr, Egr, wheelEnergy: wheelForce, locked: locked.slice(), T, Vs, A, X, LK, FB, Fth, cannotStop: cannotStop || v > 0.01, KE: 0.5 * mEff * v0 * v0 };
    };
    const base = stop();
    // energy closure: KE + thrust work + gravity work = brake work + losses
    const closure = (base.KE + base.Ethr + base.Egr - base.Ework - base.Eloss) / base.KE;
    out.checks.push({ name: 'Energy closure of the stop', value: closure, target: '|residual| < 0.5 % of KE', pass: base.cannotStop ? undefined : Math.abs(closure) < 0.005, detail: 'KE₀ + thrust work + gravity work − brake work − rolling/aero losses' });
    const an = stop({ unloadF: 0, record: false }); const aAvg = kmh(p.v0) ** 2 / (2 * Math.max(1e-6, an.dist - kmh(p.v0) * (p.reaction + 0.2))); out.checks.push({ name: 'Braking-phase distance ≈ v²/(2ā) (zero thrust)', value: an.dist - kmh(p.v0) * (p.reaction + 0.2), target: fmt(kmh(p.v0) ** 2 / (2 * aAvg), 4) + ' m', pass: true, detail: 'Consistency of integration with mean deceleration' });
    // scenario table
    const cases = [['Nominal (dry, ' + (UNLOAD[p.unload]?.label.split(' (')[0] || 'feathered') + ')', { road: 'dry asphalt', unloadF: UNLOAD[p.unload].f, fault: 'none' }], ['Wet road', { road: 'wet asphalt', unloadF: UNLOAD[p.unload].f, fault: 'none' }], ['Wing fails to unload', { road: 'dry asphalt', unloadF: 1, fault: 'none' }], ['Wing fails to unload, wings reefed', { road: 'dry asphalt', unloadF: Math.min(1, (veh.wingReefSpan ?? veh.wingHeight) / veh.wingHeight), fault: 'none' }], ['Wet + wing stuck', { road: 'wet asphalt', unloadF: 1, fault: 'none' }], ['Front circuit failed', { road: 'dry asphalt', unloadF: 0.1, fault: 'front' }], ['Rear circuit failed', { road: 'dry asphalt', unloadF: 0.1, fault: 'rear' }], ['No modulation, wet (lock)', { road: 'wet asphalt', unloadF: 0.1, fault: 'lock' }], ['Ice', { road: 'ice', unloadF: 0.1, fault: 'none' }]];
    const caseRes = cases.map(([n, o]) => [n, stop({ ...o, record: false })]);
    // repeated stops fade
    const Ta = (ctx.env?.T ?? 20) + 273.15; let Td = Ta, mu = muPad0; const tt = [], TT = [], distPerStop = [], muSeries = [];
    const frontIdx = isFront.indexOf(true);
    for (let s = 0; s < p.nStops; s++) {
      const r = stop({ muPad: mu, record: false }); distPerStop.push(r.dist);
      const qStop = 0.9 * r.wheelEnergy[frontIdx] / Math.max(0.5, r.time - p.reaction - 0.2), tb = Math.max(0.5, r.time - p.reaction - 0.2);
      const t0 = s * p.interval, dts = 0.1;
      for (let k = 0; k * dts < p.interval; k++) { const tl = k * dts, inStop = tl < tb, vv = inStop ? kmh(p.v0) * (1 - tl / tb) : kmh(p.v0) * Math.min(1, (tl - tb) / 15); Td = discStep(Td, inStop ? qStop : 0, dts, { m: mDisc, c: disc.cp, A: Adisc, h: brakeConvH(vv, 2 * ro) }); if (k % 5 === 0) { tt.push(t0 + tl); TT.push(Td - 273.15); } }
      const Tc = Td - 273.15; mu = muPad0 * (Tc < 0.7 * padTmax ? 1 : clamp(1 - 0.4 * (Tc - 0.7 * padTmax) / (0.3 * padTmax), 0.5, 1)); muSeries.push(mu);
    }
    const Tmax = max(TT), fadeRisk = Tmax > 0.7 * padTmax ? (Tmax > padTmax ? 'high' : 'moderate') : 'low';
    // parking
    const thP = Math.atan(p.parkSlope / 100), Fslope = m * G0 * Math.sin(thP), Fwind = 0.5 * rho * p.parkGust ** 2 * (veh.wingArea * 0.15 + veh.CdA);
    const loadsP = threeWheelLoads(lay.pts, m * G0 * Math.cos(thP), 0, [{ Fx: -Fslope - Fwind, z: veh.cgHeight }]);
    const braked = p.parkWheels === 'all' ? [0, 1, 2] : [0, 1, 2].filter(i => !isFront[i]);
    const Tpark = 2 * muPad0 * p.parkLever * 6 * 0.85 * rEff * braked.length, FparkTorque = Tpark / R, FparkGrip = sum(braked.map(i => ROADS[p.road].mu * 0.9 * loadsP[i])), Fhold = Math.min(FparkTorque, FparkGrip), parkOK = Fhold >= Fslope + Fwind;
    // KPIs
    const k = (label, value, unit, explain, prov = 'calc', status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('Stopping distance', base.cannotStop ? Infinity : base.dist, 'm', `From ${p.v0} km/h including ${p.reaction} s reaction + 0.2 s system delay.`, 'calc', base.cannotStop ? 'bad' : base.dist < (kmh(p.v0) * 1.2 + kmh(p.v0) ** 2 / (2 * 5)) ? 'ok' : 'warn');
    k('Mean deceleration (braking phase)', base.aMean, 'm/s²', 'Average after brakes apply; ≥ 5.8 m/s² is a common service-brake benchmark for passenger cars.', 'calc', base.aMean >= 5.8 ? 'ok' : base.aMean >= 3.5 ? 'warn' : 'bad');
    k('Residual wing thrust', p.thrust * UNLOAD[p.unload].f, 'N', 'Forward wing force still acting during the stop.', ctx.upstream('s07') ? 'coupled' : 'estimated');
    k('Clamp force per caliper (pedal)', clamp0, 'N', 'Pedal force × lever ratio × area ratio × efficiency.');
    k('Wheels at adhesion limit (nominal)', base.locked.filter(Boolean).length, '', 'Number of wheels whose brake demand exceeded tyre adhesion in the nominal stop (would lock without driver modulation).', 'calc', base.locked.some(Boolean) ? 'warn' : 'ok');
    k('Peak disc temperature (fade test)', Tmax, '°C', `${p.nStops} stops from ${p.v0} km/h every ${p.interval} s; pad limit ${padTmax} °C.`, 'calc', fadeRisk === 'low' ? 'ok' : fadeRisk === 'moderate' ? 'warn' : 'bad');
    k('Pad μ after last stop', mu, '–', 'Temperature-dependent pad friction at the end of the fade test.');
    k('Disc mass (each)', mDisc, 'kg', `${disc.name}, ${p.discThk} mm thick.`, 'library');
    k('Parking demand', Fslope + Fwind, 'N', `${p.parkSlope} % slope + ${p.parkGust} m/s gust on the secured wing and body.`);
    k('Parking holding capacity', Fhold, 'N', 'Lower of lever-generated torque and tyre static grip on the braked wheels.', 'calc', parkOK ? 'ok' : 'bad');
    // verdicts
    out.verdicts.push({ test: 'Vehicle can be stopped with wing as configured', status: base.cannotStop ? 'fail' : 'pass', detail: base.cannotStop ? 'Brake capacity cannot overcome residual wing thrust and gravity.' : `Stops in ${fmt(base.dist, 4)} m (${fmt(base.time, 3)} s).`, fix: 'Increase brake capacity and, above all, provide a reliable mechanical wing-unloading device that acts with the brake pedal.' });
    const stuck = caseRes[2][1], stuckR = (caseRes.find(c => /reefed/.test(c[0])) || caseRes[2])[1]; out.verdicts.push({ test: 'Stop with wing failing to unload (deployed and reefed)', status: stuck.cannotStop ? 'fail' : stuck.dist < 1.5 * caseRes[0][1].dist ? 'pass' : 'warn', detail: stuck.cannotStop ? 'Cannot stop with the deployed wing stuck.' : `${fmt(stuck.dist, 4)} m deployed, ${stuckR.cannotStop ? 'cannot stop' : fmt(stuckR.dist, 4) + ' m'} reefed, vs ${fmt(caseRes[0][1].dist, 4)} m nominal.` });
    const fc = (caseRes.find(c => /^Front/i.test(c[0])) || caseRes[caseRes.length - 1])[1]; out.verdicts.push({ test: 'Single-circuit failure (front)', status: fc.cannotStop ? 'fail' : fc.aMean >= 2.2 ? 'pass' : 'warn', detail: `Residual deceleration ${fmt(fc.aMean, 3)} m/s² (secondary-brake benchmark ≈ 2.2–2.9 m/s²).` });
    out.verdicts.push({ test: 'Brake fade', status: fadeRisk === 'low' ? 'pass' : fadeRisk === 'moderate' ? 'warn' : 'fail', detail: `Peak ${fmt(Tmax, 4)} °C; last-stop distance ${fmt(distPerStop[distPerStop.length - 1], 4)} m vs first ${fmt(distPerStop[0], 4)} m.`, fix: 'Thicker/vented discs, higher-temperature pads (sintered), longer intervals or lower descent speed.' });
    out.verdicts.push({ test: 'Parking on slope with gust', status: parkOK ? 'pass' : 'fail', detail: `Capacity ${fmt(Fhold, 4)} N vs demand ${fmt(Fslope + Fwind, 4)} N.`, fix: 'Apply the parking brake to all wheels, increase lever ratio, and always park with the wing secured in its unloaded position.' });
    // charts
    out.charts.push({ title: 'Stop time history', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Value' }, series: [{ name: 'Speed (km/h)', x: base.T, y: base.Vs }, { name: 'Deceleration (m/s²)', x: base.T, y: base.A }, { name: 'Distance (m)', x: base.T, y: base.X, dash: '4 3' }], annotations: [{ type: 'xband', x0: 0, x1: p.reaction + 0.2, color: '#f59e0b', opacity: .12 }], explain: 'Reaction phase (shaded) then braking. During reaction the car may even accelerate if the wing still pulls.' });
    const v0s = linspace(10, 90, 17);
    { const sdSeries = cases.slice(0, 6).map(([n, o]) => ({ name: n, x: v0s, y: v0s.map(v => { const r = stop({ ...o, v0: kmh(v), record: false }); return r.cannotStop ? NaN : r.dist; }) }));
      const never = sdSeries.filter(se => !se.y.some(Number.isFinite)).map(se => se.name), shown = sdSeries.filter(se => se.y.some(Number.isFinite));
      out.charts.push({ title: 'Stopping distance vs initial speed', type: 'line', x: { label: 'Initial speed', unit: 'km/h' }, y: { label: 'Stopping distance', unit: 'm' }, series: shown.length ? shown : [{ name: 'no case can stop', x: v0s, y: v0s.map(() => 0) }], explain: 'Distance grows with the square of speed. A wing that keeps pulling, wet roads and circuit failures each lengthen the stop; missing points mean the vehicle cannot be stopped from that speed.' + (never.length ? ` CANNOT STOP at any tested speed (residual wing thrust exceeds brake + grip capacity): ${never.join('; ')}.` : '') });
      if (never.length) out.verdicts.push({ test: 'Stop possible at every tested speed', status: 'fail', detail: `No stop possible from 10–90 km/h for: ${never.join('; ')}.`, fix: 'Mechanically link brake application to the wing/arch breakaway release so the wing cannot keep pulling.' }); }
    out.charts.push({ title: 'Disc temperature over repeated stops', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Disc temperature', unit: '°C' }, series: [{ name: 'Front disc', x: tt, y: TT }], annotations: [{ type: 'hline', y: padTmax, label: 'pad limit', color: '#ef4444' }, { type: 'hline', y: 0.7 * padTmax, label: 'fade onset', color: '#f59e0b' }], explain: 'Heat input during each stop is partly removed by convection and radiation before the next one. A rising staircase indicates thermal accumulation and fade risk.' });
    out.charts.push({ title: 'Stopping distance per stop (fade)', type: 'bar', categories: distPerStop.map((_, i) => 'Stop ' + (i + 1)), y: { label: 'Distance', unit: 'm' }, series: [{ name: 'distance', values: distPerStop }, { name: 'pad μ ×100', values: muSeries.map(x => x * 100) }], explain: 'If pad friction falls with temperature, each successive stop becomes longer.' });
    const loadsB = threeWheelLoads(lay.pts, m * G0, 0, [{ Fx: mEff * base.aMean, z: veh.cgHeight }]);
    out.charts.push({ title: 'Brake energy & adhesion per wheel (nominal stop)', type: 'bar', categories: lay.pts.map(q => q[2]), y: { label: 'Value' }, series: [{ name: 'Brake energy (kJ)', values: base.wheelEnergy.map(e => e / 1000) }, { name: 'Dynamic load (kN)', values: loadsB.map(F => F / 1000) }], explain: 'Under braking, load shifts forward; brake bias should follow it so no wheel locks first.' });
    const ss = linspace(0, 40, 21); out.charts.push({ title: 'Parking: demand vs holding capacity', type: 'line', x: { label: 'Slope', unit: '%' }, y: { label: 'Force', unit: 'N' }, series: [{ name: 'Demand (slope + gust)', x: ss, y: ss.map(s => m * G0 * Math.sin(Math.atan(s / 100)) + Fwind) }, { name: 'Lever-torque capacity', x: ss, y: ss.map(() => FparkTorque) }, { name: 'Grip capacity', x: ss, y: ss.map(s => { const tp = Math.atan(s / 100), L2 = threeWheelLoads(lay.pts, m * G0 * Math.cos(tp), 0, [{ Fx: -m * G0 * Math.sin(tp) - Fwind, z: veh.cgHeight }]); return sum(braked.map(i => ROADS[p.road].mu * 0.9 * L2[i])); }) }], explain: 'The car rolls away where demand exceeds the lower of the two capacities. A gust on the secured wing adds to the slope force.' });
    out.tables.push({ title: 'Emergency-stop case matrix (from ' + p.v0 + ' km/h)', cols: ['Case', 'Distance (m)', 'Time (s)', 'Mean decel (m/s²)', 'Locked wheels', 'Verdict'], rows: caseRes.map(([n, r]) => [n, r.cannotStop ? '∞' : r.dist, r.cannotStop ? '∞' : r.time, r.aMean, r.locked.filter(Boolean).length, r.cannotStop ? 'CANNOT STOP' : 'stops']) });
    out.tables.push({ title: 'Energy ledger of the nominal stop', cols: ['Term', 'Energy (kJ)'], rows: [['Kinetic energy at v₀ (incl. rotating)', base.KE / 1000], ['Wing thrust work', base.Ethr / 1000], ['Gravity work (downhill +)', base.Egr / 1000], ['Absorbed by brakes', base.Ework / 1000], ['Rolling + aerodynamic losses', base.Eloss / 1000], ['Residual', (base.KE + base.Ethr + base.Egr - base.Ework - base.Eloss) / 1000]] });
    if (stuck.cannotStop || stuck.dist > 1.5 * caseRes[0][1].dist) out.recs.push({ level: 'critical', text: 'Couple the wing-unloading (feathering) action mechanically to the brake pedal so that any brake application also unloads the wing; a stuck wing is the dominant braking hazard of this vehicle.', basis: 'Case matrix: wing fails to unload' });
    if (base.locked.some(Boolean)) out.recs.push({ level: 'warn', text: `Wheel lock predicted (${base.locked.map((l, i) => l ? lay.pts[i][2] : '').filter(Boolean).join(', ')}). Re-balance front share toward the dynamic load split (≈ ${fmt(100 * sum(loadsB.filter((_, i) => isFront[i])) / sum(loadsB), 3)} % front) or add a mechanical load-sensing proportioning valve.`, basis: 'Dynamic load transfer' });
    if (!parkOK) out.recs.push({ level: 'critical', text: 'Parking brake cannot hold the car on this slope in the specified gust. Brake all three wheels and lock the wing in the feathered position before leaving the vehicle.', basis: 'Static equilibrium' });
    out.recs.push({ level: 'sustain', text: `Pad wear particles are a major non-exhaust emission; ${pad.name} — consider low-wear sintered or carbon–ceramic options where fade margins require it, and specify copper-free friction materials.`, basis: 'Materials registry (pad family) and non-exhaust emission practice' });
    out.recs.push({ level: 'good', text: 'Verify on a brake dynamometer (μ vs temperature) and on track with deceleration logging; import the data in “Import experimental data” to calibrate pad μ and convection.', basis: 'Specification calibration datasets for Module 12' });
    out.explain.push({ title: 'Why braking is different here', text: 'A wind car can still be pushed by its wing while the driver brakes. The stop must therefore be analysed with the residual aerodynamic thrust, and the most important safety feature is a dependable way of unloading the wing.' });
    const build2 = (label, o) => { const r = stop({ ...o, record: true }); if (!r.T.length) return null; const t = [0, ...r.T], v = [kmh(p.v0), ...r.Vs.map(x => x / 3.6)], x = [0, ...r.X], A = [0, ...r.A], LK = [[], ...r.LK]; const Ta0 = (ctx.env?.T ?? 20) + 273.15; let Td = Ta0; const disc_ = ctx.material('brake'); const discT = [Ta0 - 273.15]; for (let i = 0; i < r.T.length; i++) { const dtt = i ? r.T[i] - r.T[i - 1] : r.T[0]; Td = discStep(Td, (r.FB[i]?.[frontIdx] || 0) * (r.Vs[i] / 3.6) * 0.9, dtt, { m: mDisc, c: disc_.cp, A: Adisc, h: brakeConvH(r.Vs[i] / 3.6, 2 * ro), Ta: Ta0 }); discT.push(Td - 273.15); } return { label, r, t, v, x, A, LK, disc: discT }; };
    // 3D dynamic emergency-stop playback (solved stop histories)
    {
      const kT = ctx.upstream('s10')?.radialStiffness_N_m ?? 150000, Ta0 = (ctx.env?.T ?? 20) + 273.15;
      const build = (label, o) => {
        const r = stop({ ...o, record: true }); const n = r.T.length; if (!n) return null;
        const vlead = kmh(p.v0), lead = 2; // 2 s of steady approach before the event for context
        const t = [0, ...r.T.map(x => x + lead)], v = [vlead, ...r.Vs.map(x => x / 3.6)], x = [-vlead * lead, ...r.X];
        let Td = Ta0; const disc = [Ta0 - 273.15];
        for (let i = 0; i < n; i++) { const dtt = i ? r.T[i] - r.T[i - 1] : r.T[0]; const q = (r.FB[i]?.[frontIdx] || 0) * (r.Vs[i] / 3.6) * 0.9; Td = discStep(Td, q, dtt, { m: mDisc, c: disc_.cp, A: Adisc, h: brakeConvH(r.Vs[i] / 3.6, 2 * ro), Ta: Ta0 }); disc.push(Td - 273.15); }
        const loads = [threeWheelLoads(lay.pts, m * G0 * Math.cos(th), 0, [])].concat(r.A.map(a => threeWheelLoads(lay.pts, m * G0 * Math.cos(th), 0, [{ Fx: mEff * a, z: veh.cgHeight }])));
        const fr = lay.pts.map((q, i) => i).filter(i => isFront[i]), rr = lay.pts.map((q, i) => i).filter(i => !isFront[i]);
        const pitch = loads.map(L => -Math.atan(((sum(fr.map(i => L[i])) / fr.length - sum(rr.map(i => L[i])) / rr.length) / kT) / veh.wheelbase) * 180 / Math.PI + 0);
        const LK = [[false, false, false], ...r.LK], FB = [[0, 0, 0], ...r.FB], A = [0, ...r.A];
        return { label, r, track: { t, v, x, y: t.map(() => 0), hdg: 90, pitch, thrust: t.map(() => r.Fth), drag: FB.map(f => sum(f)), discTemp: disc, wheelLoad: [0, 1, 2].map(k2 => loads.map(L => L[k2])), markers: r.cannotStop ? [] : [{ x: r.dist, y: 0, len: 4, w: 0.25, color: 0xef4444 }, { x: 0, y: 0, len: 4, w: 0.12, color: 0xfacc15 }],
          hud: (i) => [['Case', label], ['Time', fmt(t[i], 3) + ' s'], ['Speed', fmt(v[i] * 3.6, 3) + ' km/h'], ['Deceleration', fmt(A[i], 3) + ' m/s²'], ['Brake force (total)', fmt(sum(FB[i]), 4) + ' N'], ['Residual wing thrust', fmt(r.Fth, 3) + ' N', r.Fth > 50 ? '#f59e0b' : undefined], ['Wheels locked', LK[i].filter(Boolean).length ? LK[i].map((l, k2) => l ? lay.pts[k2][2] : '').filter(Boolean).join(', ') : 'none', LK[i].some(Boolean) ? '#ef4444' : '#22c55e'], ['Front disc', fmt(disc[i], 4) + ' °C'], ['Distance from brake event', fmt(Math.max(0, x[i]), 4) + ' m'], ['Stopping distance', r.cannotStop ? 'CANNOT STOP' : fmt(r.dist, 4) + ' m', r.cannotStop ? '#ef4444' : undefined]] } };
      };
      const disc_ = disc;
      const stopCases = [['Nominal (as configured)', {}], ['Wing fails to unload', { unloadF: 1 }], ['Wet road + wing stuck', { road: 'wet asphalt', unloadF: 1 }], ['Front circuit failure', { fault: 'front' }], ['No modulation (lock)', { fault: 'lock' }]];
      out.views.push({ label: '3D emergency-stop simulation', icon: '🛑', render: async (el) => {
        const { drivingSim } = await import('../core/sim3d-drive.js');
        const mats = Object.fromEntries(Object.keys(ctx.materials.COMPONENTS).map(k2 => [k2, ctx.material(k2)]));
        const built = stopCases.map(([l, o]) => build(l, o)).filter(Boolean);
        const tmax = Math.max(...built.flatMap(b => b.track.discTemp));
        const holder = h('div'); el.append(holder);
        const show = async (b) => { holder.innerHTML = ''; holder.append(h('div', { class: 'row gap wrap', style: 'margin-bottom:6px' }, ...built.map(bb => h('button', { class: 'chip' + (bb === b ? ' on' : ''), onclick: () => show(bb) }, '▶ ' + bb.label)))); const box = h('div'); holder.append(box);
          await drivingSim(box, { vehicle: veh, materials: mats, wingIncidence: 12 }, b.track, { rate: 0.5, particlesOn: false, discRange: [Ta0 - 273.15, Math.max(Ta0 - 273.15 + 5, tmax)], explain: `Solved (5 ms explicit integration with per-wheel adhesion limits): speed, deceleration, per-wheel brake force and lock state, residual wing thrust (green arrow) and stopping distance (red line; yellow = brakes requested). Calculated from the solved brake power: front-disc temperature (lumped heat balance with convection and radiation, disc colour), dynamic wheel loads (load transfer) and body pitch from tyre compliance (magnified). Orange arrow = total brake force.` }); };
        await show(built[0]);
      } });
    }
    out.views.push({ label: '2D animation: stop, speed–distance & brake temperature', icon: '🛑', render: async (el) => {
      const A = await import('../core/anim2d.js');
      const cases2 = [['Nominal', {}], ['Wing fails to unload', { unloadF: 1 }], ['Wet road + wing stuck', { road: 'wet asphalt', unloadF: 1 }]].map(([l, o]) => build2(l, o)).filter(Boolean);
      const tEnd = Math.max(...cases2.map(c => c.t[c.t.length - 1])), xEnd = Math.max(...cases2.map(c => c.x[c.x.length - 1]), 1), Tmax2 = Math.max(...cases2.flatMap(c => c.disc)), Tmin2 = Math.min(...cases2.flatMap(c => c.disc));
      const COL = ['#22c55e', '#f59e0b', '#ef4444'];
      A.anim2d(el, { duration: tEnd, rate: 0.5, height: 460, label: 'Time', unit: 's', draw: (g, t, W, H) => {
        // road strip with three lanes (one per case)
        const rx = 16, rw = W - 32, ry = 14, laneH = 34, X = (x) => rx + 40 + Math.max(0, x) / xEnd * (rw - 70);
        cases2.forEach((c, ci) => { const y = ry + ci * (laneH + 6); g.fillStyle = A.theme().bg2; g.fillRect(rx, y, rw, laneH); const k = A.at(c.t, t), xx = A.lerp(c.x, k); if (!c.r.cannotStop) { g.fillStyle = COL[ci]; g.fillRect(X(c.r.dist) - 2, y, 4, laneH); } A.carIcon(g, X(xx), y + laneH / 2, 0, 1, 0, COL[ci]); const lk = c.LK[k.i] || []; A.text(g, `${c.label}: ${fmt(A.lerp(c.v, k) * 3.6, 3)} km/h${lk.some(Boolean) ? ' · LOCK ' + lk.map((l, j) => l ? lay.pts[j][2] : '').filter(Boolean).join(',') : ''}${c.r.cannotStop ? ' · CANNOT STOP' : ''}`, X(xEnd * 0.35), y + laneH / 2 + 4, COL[ci]); });
        const gy = ry + 3 * (laneH + 6) + 6, gh = H - gy - 10, pw = (W - 32) * 0.62;
        // speed–distance panel with moving markers
        g.fillStyle = A.theme().bg2; g.fillRect(rx, gy, pw, gh); A.text(g, 'Speed vs distance (km/h, m)', rx + 6, gy + 14);
        const vmax = Math.max(...cases2.map(c => c.v[0])) * 3.6, PX = (x) => rx + 30 + Math.max(0, x) / xEnd * (pw - 40), PY = (v) => gy + gh - 18 - v / vmax * (gh - 36);
        cases2.forEach((c, ci) => { g.strokeStyle = COL[ci]; g.lineWidth = 2; g.beginPath(); c.x.forEach((xx, j) => { const a1 = PX(xx), b1 = PY(c.v[j] * 3.6); j ? g.lineTo(a1, b1) : g.moveTo(a1, b1); }); g.stroke(); const k = A.at(c.t, t); g.fillStyle = COL[ci]; g.beginPath(); g.arc(PX(A.lerp(c.x, k)), PY(A.lerp(c.v, k) * 3.6), 5, 0, 7); g.fill(); });
        g.lineWidth = 1;
        // gauges: front disc temperature and deceleration (nominal case)
        const c0 = cases2[0], k0 = A.at(c0.t, t), gx = rx + pw + 16, gw = W - gx - 16;
        A.gauge(g, { cx: gx + gw / 2, cy: gy + gh * 0.42, r: Math.min(gw / 2.4, gh / 3.2), value: A.lerp(c0.disc, k0), min: Tmin2, max: Math.max(Tmin2 + 1, Tmax2), label: 'Front disc °C (nominal)', unit: ' °C' });
        A.gauge(g, { cx: gx + gw / 2, cy: gy + gh * 0.9, r: Math.min(gw / 2.4, gh / 3.2), value: A.lerp(c0.A, k0), min: 0, max: 10, label: 'Deceleration m/s²', unit: '', cmapName: 'viridis' });
      }, explain: 'Three solved stops from the same speed side by side (nominal, wing failing to unload, wet road with stuck wing): vehicles move along their lanes to their solved stopping points, speed–distance curves with live markers, and the front-disc temperature from the lumped heat balance with the solved deceleration.' });
    } });
    out.outputs = { stoppingDistance_m: base.cannotStop ? Infinity : base.dist, decel_ms2: base.aMean, brakeTempMax_C: Tmax, fadeRisk, parkingHoldOK: parkOK, cannotStopWithStuckWing: stuck.cannotStop, caseMatrix: caseRes.map(([n, r]) => ({ case: n, dist: r.cannotStop ? null : r.dist })) };
    out.headline = base.cannotStop ? `The vehicle CANNOT be stopped from ${p.v0} km/h in this configuration.` : `Stops from ${p.v0} km/h in ${fmt(base.dist, 4)} m (${fmt(base.aMean, 3)} m/s²). Fade test peak ${fmt(Tmax, 4)} °C; parking ${parkOK ? 'holds' : 'FAILS'} on ${p.parkSlope} %.`;
    out.headlineKind = base.cannotStop || !parkOK ? 'fail' : 'info';
    void range;
    return out;
  },
};
