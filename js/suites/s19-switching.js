// Suite 19 — Passive mechanical mode switching & control.
import { fmt, range, linspace, rng, max, min, mean, interp1, rad } from '../core/num.js';
import { kmh, roadLoad, apparentWind, wingForces, kaimal, oneMinusCos } from '../core/physics.js';
import { simulateSwitching, MODES } from '../core/vehicle.js';
import { h } from '../core/ui.js';

const FAULTS = { none: 'No fault', jammed: 'Jammed coupling (never engages)', flexure: 'Failed flexure (thresholds drift to 40 %)', weak: 'Insufficient backup power (5 W)', brakeGust: 'Gust during braking (interlock check)' };

export default {
  description: 'Hybrid-automaton model of the proposed passive mechanical supervisor that hands propulsion between the wing, the thermomechanical hub and the elastic buffer. Thresholds act on useful forward aerodynamic thrust (not total wing load), with separate engage/disengage values (mechanical hysteresis), compliant-sensor dynamics and engagement delay; quantifies switching count, chatter, torque continuity, engagement shock and fault behaviour.',
  literature: 'passive mechanical switch bistable compliant mechanism hysteresis threshold hybrid system',
  credibility: 'Conceptual',
  equations: ['Threshold-based hybrid dynamical system (event-driven state machine) with hysteretic switching law: engage when F_x < F_e, disengage when F_x > F_d, F_d > F_e', 'Compliant sensor element m ẍ + c ẋ + k x = F_x (spring–damper; natural frequency ω_n = √(k/m))', 'Apparent-wind vector transformation and lift/drag resolution into forward thrust F_x = L sin β − D cos β (distinct from total load √(L² + D²))', 'Torque continuity at transitions τ_hub = F_hub R; engagement shock ΔF/Δt', 'Finite buffer energy balance Ė = −F_hub v while discharging', 'Kaimal turbulence spectrum and one-minus-cosine gusts for the inflow record'],
  methods: ['Explicit integration Δt = 0.25 s over a 10-minute record', 'Pending-transition timer reproduces mechanical engagement delay', 'Hysteresis-band sweep to locate the chatter-free design region'],
  assumptions: ['The automaton describes the intended switching sequence; the physical mechanism that would realise it is not yet designed or validated', 'Electronic simulation of a control law is not proof that a passive mechanism exists', 'Vehicle speed held at the target (the coupled speed response is in Suite 20)'],
  ibc: { initial: ['Starting propulsion mode, threshold-element displacement and preload, wing-linked sensor deflection, engagement condition, stored-energy state, hub torque, driver command, brake position'], boundary: ['Aerodynamic wing loading, mechanically transmitted control forces, engage/disengage thresholds, spring & damping forces, travel limits, contact constraints, wheel speed, thermal backup output, driver actions', 'Faults: jammed coupling, failed flexure, insufficient actuator output, unexpected gust, simultaneous braking'] },
  cvv: { calibration: ['Measured engage/disengage thresholds and hysteresis of the built switch', 'Transition duration, force response, vibration and chatter tests'], verification: ['No switching when thrust stays inside the hysteresis band', 'Zero hysteresis reproduces chatter under turbulence', 'Buffer energy never negative'], validation: ['Bench rig with programmable wing-load actuator and instrumented hub (failed-engagement and jamming tests)'] },
  formats: { import: ['CSV force/time logs from the switching rig'], export: ['CSV mode history', 'PNG/SVG plots'] },
  params: [
    { key: 'profile', label: 'Wind input', type: 'select', group: 'Inflow', default: 'declining', options: [{ value: 'declining', label: 'Declining then recovering wind (transition test)' }, { value: 'turbulent', label: 'Turbulent record from Suite 05' }, { value: 'gusty', label: 'Gusty (1-cos gusts every 90 s)' }] },
    { key: 'U', label: 'Mean wing-height wind', unit: 'm/s', type: 'number', min: 0, max: 20, step: 0.1, group: 'Inflow', default: 6, auto: (c) => c.env ? { value: +Math.max(0.5, c.env.Uz).toFixed(2), prov: 'coupled' } : undefined },
    { key: 'TI', label: 'Turbulence intensity', unit: '%', type: 'number', min: 0, max: 40, step: 1, group: 'Inflow', default: 15 },
    { key: 'twa', label: 'True wind angle from heading', unit: '°', type: 'number', min: 0, max: 180, step: 5, group: 'Inflow', default: 100, help: '90° = beam reach (crosswind).' },
    { key: 'speed', label: 'Target vehicle speed', unit: 'km/h', type: 'number', min: 5, max: 60, step: 1, group: 'Operating point', default: 20 },
    { key: 'CL', label: 'Trimmed wing C_L', type: 'number', min: 0.2, max: 1.6, step: 0.05, group: 'Operating point', default: 0.9, auto: (c) => c.upstream('s07')?.CLmax ? { value: +(0.85 * c.upstream('s07').CLmax).toFixed(2), prov: 'coupled' } : undefined },
    { key: 'CD', label: 'Wing C_D at trim', type: 'number', min: 0.02, max: 0.5, step: 0.01, group: 'Operating point', default: 0.12 },
    { key: 'engageFrac', label: 'Engage threshold (× required force)', type: 'number', min: 0.3, max: 1.2, step: 0.01, group: 'Switch design', default: 0.85 },
    { key: 'disengageFrac', label: 'Disengage threshold (× required force)', type: 'number', min: 0.5, max: 2, step: 0.01, group: 'Switch design', default: 1.15 },
    { key: 'autoBand', label: 'Auto-size the hysteresis band (narrowest chatter-free band)', type: 'bool', group: 'Switch design', default: true, help: 'Widens the engage/disengage band symmetrically about the required force until the 10-minute gust record produces no chatter; the passive switch is then built to that band.' },
    { key: 'kSens', label: 'Sensor flexure stiffness', unit: 'N/mm', type: 'number', min: 1, max: 500, step: 1, group: 'Switch design', default: 20 },
    { key: 'mSens', label: 'Sensor moving mass', unit: 'kg', type: 'number', min: 0.05, max: 10, step: 0.05, group: 'Switch design', default: 0.6 },
    { key: 'zeta', label: 'Sensor damping ratio', type: 'number', min: 0.05, max: 2, step: 0.05, group: 'Switch design', default: 0.5 },
    { key: 'delay', label: 'Mechanical engagement delay', unit: 's', type: 'number', min: 0, max: 5, step: 0.05, group: 'Switch design', default: 0.5 },
    { key: 'interlock', label: 'Brake interlock (brake ⇒ backup disengaged)', type: 'bool', group: 'Switch design', default: true },
    { key: 'Pbackup', label: 'Backup hub power available', unit: 'W', type: 'number', min: 0, max: 2000, step: 1, group: 'Backup', default: 0, auto: (c) => { const o = c.upstream('s17'); return o?.Pwheel_W !== undefined ? { value: +Math.max(0, o.Pwheel_W).toFixed(2), prov: 'coupled' } : { value: 0, prov: 'unsupported' }; }, help: 'Zero unless Suite 17 predicts positive continuous hub power (simulation).' },
    { key: 'reefU', label: 'Reef threshold (wind)', unit: 'm/s', type: 'number', min: 2, max: 20, step: 0.5, group: 'Wind', default: 7 },
    { key: 'Ebuf', label: 'Buffer usable energy', unit: 'kJ', type: 'number', min: 0, max: 600, step: 1, group: 'Backup', default: 10, auto: (c) => { const o = c.upstream('s18'); return o?.Euse_J !== undefined ? { value: +(o.Euse_J / 1000).toFixed(2), prov: 'coupled' } : undefined; } },
    { key: 'fault', label: 'Fault scenario', type: 'select', group: 'Faults', default: 'none', options: Object.entries(FAULTS).map(([value, label]) => ({ value, label })) },
    { key: 'seed', label: 'Random seed', type: 'number', min: 1, max: 99999, step: 1, group: 'Inflow', default: 7, expert: true },
  ],
  presets: [{ label: 'No hysteresis (chatter demo)', values: { engageFrac: 1, disengageFrac: 1 } }, { label: 'Wide hysteresis', values: { engageFrac: 0.75, disengageFrac: 1.3 } }, { label: 'Jammed coupling', values: { fault: 'jammed' } }, { label: 'Gust while braking, no interlock', values: { fault: 'brakeGust', interlock: false } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const veh = ctx.vehicle, v = kmh(p.speed), rho = ctx.env?.rho ?? 1.225, need = roadLoad(v, veh, { rho }).F;
    const dt = 0.25, N = 2400, t = range(N).map(i => i * dt);
    // wind record
    let Uw;
    const s05 = ctx.upstream('s05');
    if (p.profile === 'turbulent' && s05?.turbulence?.u?.length) { const tu = s05.turbulence; Uw = t.map(tt => interp1(tu.t, tu.u, tt % (tu.t[tu.t.length - 1] || 1))); }
    else {
      const R = rng(p.seed), fs = linspace(0.002, 1.5, 90), sig = p.U * p.TI / 100, Sv = fs.map(f => kaimal(f, Math.max(p.U, 0.5), sig)), ph = fs.map(() => 2 * Math.PI * R()), am = fs.map((f, k) => Math.sqrt(2 * Sv[k] * (k ? fs[k] - fs[k - 1] : fs[0])));
      const turb = (tt) => fs.reduce((s, f, k) => s + am[k] * Math.cos(2 * Math.PI * f * tt + ph[k]), 0);
      Uw = t.map(tt => { let base = p.U; if (p.profile === 'declining') base = p.U * (tt < 150 ? 1 : tt < 300 ? 1 - 0.9 * (tt - 150) / 150 : tt < 420 ? 0.1 : Math.min(1, 0.1 + 0.9 * (tt - 420) / 100)); if (p.profile === 'gusty') base = p.U + [60, 150, 240, 330, 420, 510].reduce((s, g) => s + oneMinusCos(tt, g, 4, p.U * 0.8), 0); return Math.max(0, base + turb(tt) * (base / Math.max(p.U, 0.1))); });
    }
    const s07 = ctx.upstream('s07');
    const thrustOf = (U) => { const aw = apparentWind(U, p.twa, 0, v); if (s07?.thrustVsAWA?.awa?.length && s07.Va) { const T0 = interp1(s07.thrustVsAWA.awa, s07.thrustVsAWA.thrust, Math.abs(aw.beta)); return { Fx: T0 * (aw.Va / s07.Va) ** 2, total: NaN, beta: aw.beta, Va: aw.Va }; } const wf = wingForces(aw.Va, aw.beta, p.CL, p.CD, veh.wingArea, rho); return { Fx: wf.thrust, total: Math.hypot(wf.L, wf.D), beta: aw.beta, Va: aw.Va }; };
    const TH = Uw.map(thrustOf), thrustRaw = TH.map(x => x.Fx).slice(), totalLoad = TH.map(x => Number.isFinite(x.total) ? x.total : Math.abs(x.Fx) * 2.2);
    // Revision 2 wing-root state machine: LOCKED → (side force > breakaway) → FEATHERED → (side force < 0.6 × breakaway for ≥ 3 s) → re-latched
    const a01 = ctx.upstream('s01'), washRoot = (a01?.wingRoot ?? ((veh.archRoot || 'flexure') === 'flexure' && (veh.wingConfig || 'arch') === 'arch' ? 'washout' : 'feathering')) === 'washout', featherRoot = !washRoot && ['feathering', 'ring'].includes(a01?.wingRoot ?? 'feathering'), FySafe = a01?.washoutCap_N ?? a01?.FySafe_N ?? 600;
    const sideRaw = TH.map((x, i) => Number.isFinite(x.total) ? Math.sqrt(Math.max(0, x.total ** 2 - x.Fx ** 2)) : Math.abs(totalLoad[i]) * 0.9);
    const wingState = [], thrust = [], reefState = []; let feath = false, calm = 0, featherEvents = 0, reefOn = false, reefEvents = 0;
    const spanD = veh.wingHeight, spanR = Math.min(spanD, veh.wingReefSpan ?? spanD), reefable = spanR < spanD - 1e-6, af = spanR / spanD, reefU = p.reefU ?? 7;
    for (let i = 0; i < t.length; i++) {
      // Revision 2.2: reef/deploy state with hysteresis (reef above reefU, deploy below reefU − 1.5 m/s)
      if (reefable) { const want = reefOn ? Uw[i] > reefU - 1.5 : Uw[i] > reefU; if (want !== reefOn) { reefOn = want; reefEvents++; } }
      reefState.push(reefOn ? 'REEFED' : 'DEPLOYED'); const sc = reefOn ? af : 1; thrustRaw[i] *= sc; sideRaw[i] *= sc;
      if (featherRoot) { if (!feath && sideRaw[i] > 1.3 * FySafe) { feath = true; featherEvents++; calm = 0; } else if (feath) { const Uh = Uw[i], Fl = sideRaw[i]; calm = Fl < 0.6 * FySafe ? calm + dt : 0; if (calm >= 3) feath = false; void Uh; } }
      wingState.push(feath ? 'FEATHERED' : 'LOCKED');
      // steady loads above the cap are trimmed (best thrust within the safe side force); breakaway only on spikes > 1.3 × cap
      const trimF = !feath && (featherRoot || washRoot) && sideRaw[i] > FySafe ? FySafe / sideRaw[i] : 1; if (trimF < 1) wingState[wingState.length - 1] = washRoot ? 'WASHOUT' : 'TRIMMED';
      thrust.push(feath ? -0.5 * rho * apparentWind(Uw[i], p.twa, 0, v).Va ** 2 * veh.wingArea * 0.02 * 0.5 : thrustRaw[i] * trimF);
    }
    const side = sideRaw.map((f, i) => wingState[i] === 'FEATHERED' ? 0.5 * rho * Uw[i] ** 2 * veh.wingArea * 0.02 : ['TRIMMED', 'WASHOUT'].includes(wingState[i]) ? FySafe : f);
    let engage = p.engageFrac * need, disengage = Math.max(p.disengageFrac, p.engageFrac) * need; const wn = Math.sqrt(p.kSens * 1000 / p.mSens);
    const brakeWin = (tt) => tt >= 200 && tt <= 230;
    const fault = p.fault, Pb = fault === 'weak' ? 5 : p.Pbackup;
    const run = (eF, dF, o = {}) => simulateSwitching(t, thrust, { need, engage: eF * need, disengage: Math.max(dF, eF) * need, wn, zeta: p.zeta, delay: p.delay, Pbackup: Pb, v, Ebuf0: p.Ebuf * 1000, braking: fault === 'brakeGust' && p.interlock ? brakeWin : () => false, jammed: fault === 'jammed', thresholdScale: fault === 'flexure' ? 0.4 : 1, ...o });
    let S = run(p.engageFrac, p.disengageFrac); const S0 = run(1, 1);
    if (p.autoBand && S.chatter > 0) { for (const bw of linspace(0.35, 1.2, 18)) { const eF = Math.max(0.3, 1 - bw / 2), dF = 1 + bw / 2, r = run(eF, dF); if (!r.chatter) { S = r; p.engageFrac = eF; p.disengageFrac = dF; engage = eF * need; disengage = dF * need; break; } } }
    // buffer charge state with solar-thermal recharge (re-scoped thermal backup, Suites 13–18) while the buffer is not discharging
    const thSrc = ['s18', 's17', 's15', 's14', 's13'].map(id => ctx.upstream(id)).find(o => o && Number.isFinite(o.thermalChargePower_W)), Pth = Math.max(0, thSrc?.thermalChargePower_W ?? 0), etaCh = ctx.upstream('s18')?.efficiency ?? 0.8, dayNow = ctx.upstream('s05')?.isDay ?? true;
    const Ecap = Math.max(p.Ebuf * 1000, ctx.upstream('s18')?.Etotal_J ?? 0), Ech = [p.Ebuf * 1000];
    for (let i = 1; i < t.length; i++) { const draw = Math.max(0, S.hubForce[i] * v - Pb) * dt / 0.85, chg = draw > 0 ? 0 : Pth * etaCh * dt * (dayNow ? 1 : 0); Ech.push(Math.min(Ecap, Math.max(0, Ech[i - 1] + chg - draw))); }
    const modeIdx = S.modes.map(mo => MODES.indexOf(mo));
    // brake interlock / unsafe engagement evaluation
    let unsafe = false; if (fault === 'brakeGust') unsafe = t.some((tt, i) => brakeWin(tt) && S.hubForce[i] > 0);
    const jumps = S.hubForce.map((f, i) => i ? Math.abs(f - S.hubForce[i - 1]) : 0), shock = max(jumps), jerk = shock / (veh.mass * dt);
    const deficitTime = S.modes.filter(mo => mo === 'DECEL').length * dt;
    // hysteresis sweep
    const bands = linspace(0, 0.5, 11), sweep = bands.map(bw => { const r = run(1 - bw / 2, 1 + bw / 2); return { s: r.switches, c: r.chatter }; });
    // checks
    const flat = simulateSwitching(t, t.map(() => need), { need, engage: 0.85 * need, disengage: 1.15 * need, wn, zeta: p.zeta, delay: p.delay, Pbackup: Pb, v });
    out.checks.push({ name: 'Constant thrust inside band ⇒ no switching', value: flat.switches, target: '0', pass: flat.switches === 0 });
    out.checks.push({ name: 'Zero hysteresis switches ≥ designed switch', value: S0.switches, target: '≥ ' + S.switches, pass: S0.switches >= S.switches });
    out.checks.push({ name: 'Buffer energy non-negative', value: S.EbufEnd, target: '≥ 0 J', pass: S.EbufEnd >= 0 });
    out.checks.push({ name: 'Sensor natural frequency vs gust band', value: wn / (2 * Math.PI), target: '> 1 Hz recommended', pass: wn / (2 * Math.PI) > 1, detail: 'A slow sensor filters turbulence but delays response.' });
    const k = (label, value, unit, explain, prov = 'calc', status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('Required forward force', need, 'N', `Road load at ${p.speed} km/h (rolling + drag).`);
    k('Buffer charge at end (with solar-thermal recharge)', Ech[Ech.length - 1] / 1000, 'kJ', `Recharge ${fmt(Pth, 3)} W × ${fmt(etaCh, 3)} efficiency while the buffer is not discharging (${dayNow ? 'daylight' : 'night: no solar recharge'}); ${thSrc ? 'coupled from Suites 13–18' : 'no predicted thermal charging power yet (0 W)'}. The switching automaton itself is evaluated with the finite buffer.`, thSrc ? 'calc' : 'unsupported');
    if (reefable) k('Reef / deploy changes', reefEvents, '', `Reefed (area ×${fmt(af, 3)}) above ${reefU} m/s wind, deployed below ${fmt(reefU - 1.5, 3)} m/s.`, 'calc');
    if (washRoot) k('Gust protection', 'passive washout (no feathering state)', '', `Compliant flexure arch: side force capped at ${fmt(FySafe, 4)} N by flexure stiffening and skin de-cambering; there is no release, so the automaton has no feathered state.`, 'calc');
    if (featherRoot) k('Wing feathering events', featherEvents, '', `Breakaway at ${fmt(FySafe, 4)} N side force (Suite 01); while feathered the wing gives only drag and the automaton treats thrust as lost; it re-latches after 3 s below 60 % of breakaway.`, 'calc', featherEvents ? 'warn' : 'ok');
    k('Engage threshold', engage, 'N', 'Forward thrust below which the backup is requested.', 'user');
    k('Disengage threshold', disengage, 'N', 'Forward thrust above which the wing resumes sole propulsion.', 'user');
    k('Hysteresis band in sensor travel', (disengage - engage) / (p.kSens * 1000) * 1000, 'mm', 'Mechanical displacement window the passive element must resolve.', 'calc', (disengage - engage) / p.kSens > 0.2 ? 'ok' : 'warn');
    k('Mode switches (10 min)', S.switches, '', 'Number of transitions with the designed hysteresis.', 'calc', S.switches < 12 ? 'ok' : 'warn');
    k('Chatter events', S.chatter, '', 'Transitions within 5 s of the previous one.', 'calc', S.chatter === 0 ? 'ok' : 'bad');
    k('Switches without hysteresis', S0.switches, '', 'Same record with engage = disengage — shows why hysteresis is required.');
    k('Engagement shock (max ΔF)', shock, 'N', 'Largest step change of hub force between samples (torque continuity).', 'calc', shock < 0.5 * need ? 'ok' : 'warn');
    k('Equivalent jerk', jerk, 'm/s³', 'ΔF/(mΔt) — passenger comfort typically < 2–3 m/s³.');
    k('Time in deficit (decelerating)', deficitTime, 's', 'Time when neither wing, hub nor buffer can supply the required force.', 'calc', deficitTime < 30 ? 'ok' : 'bad');
    k('Buffer energy remaining', S.EbufEnd / 1000, 'kJ', 'Finite store after the record.');
    k('Sensor natural frequency', wn / (2 * Math.PI), 'Hz', '√(k/m)/2π of the compliant thrust-sensing element.');
    // verdicts
    out.verdicts.push({ test: 'Switching stability (no chatter)', status: S.chatter === 0 ? 'pass' : S.chatter < 3 ? 'warn' : 'fail', detail: `${S.switches} transitions, ${S.chatter} chatter events (no-hysteresis reference: ${S0.switches}).`, fix: 'Widen the engage/disengage band or lower the sensor natural frequency.' });
    out.verdicts.push({ test: 'Speed continuity during wind loss', status: deficitTime < 30 ? 'pass' : 'fail', detail: Pb <= 0 && p.Ebuf <= 0 ? 'No backup power and no stored energy: the car must slow.' : `Deficit for ${fmt(deficitTime, 3)} s with ${fmt(Pb, 3)} W backup and ${p.Ebuf} kJ buffer.`, fix: 'Only a hub power predicted by Suite 17 (simulation) or a larger buffer (Suite 18) closes the gap; switching cannot create energy.' });
    if (fault !== 'none') out.verdicts.push({ test: `Fault: ${FAULTS[fault]}`, status: fault === 'brakeGust' ? (unsafe ? 'fail' : 'pass') : (fault === 'jammed' || fault === 'weak') ? 'warn' : (S.chatter > 0 ? 'fail' : 'warn'), detail: fault === 'brakeGust' ? (unsafe ? 'Backup torque applied while braking — UNSAFE without interlock.' : 'Interlock keeps the hub disengaged during braking.') : fault === 'jammed' ? 'Backup never engages; vehicle decelerates whenever wind is insufficient.' : fault === 'weak' ? 'Backup engages but cannot hold speed; buffer drains.' : 'Thresholds drifted; switching occurs at the wrong thrust level.' });
    out.verdicts.push({ test: 'Physical realisation of the control law', status: 'warn', detail: 'This is a digital simulation of the intended passive logic. It does not prove that a passive mechanism with these thresholds, hysteresis and fault behaviour exists; that requires a bench prototype.' });
    // charts
    out.charts.push({ title: 'Forward thrust vs thresholds (thrust, not total load)', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Force', unit: 'N' }, series: [{ name: 'Forward thrust F_x', x: t, y: thrust, width: 1 }, { name: 'Sensed (filtered)', x: t, y: S.sensed }, { name: 'Total wing load', x: t, y: totalLoad, dash: '3 3', width: 1 }], annotations: [{ type: 'hline', y: engage, label: 'engage', color: '#f59e0b' }, { type: 'hline', y: disengage, label: 'disengage', color: '#22c55e' }, { type: 'hline', y: need, label: 'required', color: '#ef4444' }], wide: true, explain: 'The switch must respond to useful forward thrust. Total wing load (dashed) is much larger and can stay high even when thrust is insufficient — switching on it would leave the car without propulsion.' });
    out.charts.push({ title: 'Operating-mode history', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Mode (0 WIND · 1 ASSIST · 2 BACKUP · 3 BUFFER · 4 DECEL · 5 BRAKING)' }, series: [{ name: 'with hysteresis', x: t, y: modeIdx }, { name: 'no hysteresis', x: t, y: S0.modes.map(mo => MODES.indexOf(mo) + 0.08), dash: '3 2', width: 1 }], wide: true, explain: 'State machine trajectory. Rapid toggling in the dashed (no-hysteresis) curve is chatter, which would wear the coupling and jolt the car.' });
    out.charts.push({ title: 'Buffer charge state', type: 'area', x: { label: 'Time', unit: 's' }, y: { label: 'Stored energy', unit: 'kJ' }, series: [{ name: 'buffer', x: t, y: Ech.map(e => e / 1000) }], annotations: [{ type: 'hline', y: Ecap / 1000, label: 'capacity', color: '#22c55e' }], explain: 'Finite elastic store: it falls while the buffer supplies backup force and rises only with solar-thermal recharging predicted by Suites 13–18 (simulation) — never from nothing.' });
    out.charts.push({ title: 'Hub force supplied by backup', type: 'area', x: { label: 'Time', unit: 's' }, y: { label: 'Hub force', unit: 'N' }, series: [{ name: 'F_hub', x: t, y: S.hubForce }], explain: 'Force delivered at the tyre by the backup sources. Steps at mode changes are the engagement shocks.' });
    out.charts.push({ title: 'Design map: hysteresis width vs switching', type: 'line', x: { label: 'Hysteresis band (fraction of required force)', unit: '–' }, y: { label: 'Count (10 min)' }, series: [{ name: 'Switches', x: bands, y: sweep.map(s => s.s) }, { name: 'Chatter events', x: bands, y: sweep.map(s => s.c) }], explain: 'Wider bands suppress chatter but make the car tolerate larger speed loss before the backup engages. Choose the smallest band with zero chatter.' });
    out.tables.push({ title: 'Transition events', cols: ['t (s)', 'From', 'To'], rows: S.events.slice(0, 60).map(e => [e.t, e.from, e.to]) });
    if (S.chatter > 0) out.recs.push({ level: 'warn', text: `Increase the hysteresis band to ≥ ${fmt(bands[sweep.findIndex(s => s.c === 0)] ?? 0.5, 2)} of the required force to eliminate chatter in this inflow.`, basis: 'Hysteresis sweep' });
    if (Pb <= 0) out.recs.push({ level: 'critical', text: 'No validated backup power is available, so every engagement only drains the finite buffer. Treat the switch as a speed-loss limiter, not as continuous propulsion.', basis: 'Coupled Suite 17 output / energy conservation' });
    out.recs.push({ level: 'critical', text: 'Make the brake pedal mechanically disengage the hub coupling and unload the wing (interlock); test jamming and failed-flexure cases on a bench rig before vehicle integration.', basis: 'Fault scenarios of this suite; specification Module 19 validation' });
    out.recs.push({ level: 'good', text: 'Prototype the threshold element as a preloaded bistable flexure sensing wing-root forward force; measure engage/disengage thresholds and hysteresis and import them here.', basis: 'Specification calibration datasets for Module 19' });
    out.explain.push({ title: 'Why thrust, not load', text: 'In a crosswind the wing may carry a large total load while delivering little forward force. A switch that senses total load would believe propulsion is fine and never request the backup. The model therefore resolves the aerodynamic force into the vehicle’s forward axis before applying thresholds.' });
    // 3D dynamic switching playback (solved hybrid-automaton history)
    {
      const MC = { WIND: '#22c55e', BACKUP: '#a855f7', ASSIST: '#84cc16', BUFFER: '#f59e0b', DECEL: '#ef4444', BRAKE: '#ef4444', COAST: '#94a3b8' };
      out.views.push({ label: '3D switching simulation', icon: '🔀', render: async (el) => {
        const { drivingSim } = await import('../core/sim3d-drive.js');
        const mats = Object.fromEntries(Object.keys(ctx.materials.COMPONENTS).map(k2 => [k2, ctx.material(k2)]));
        await drivingSim(el, { vehicle: veh, materials: mats, wingIncidence: 12 }, {
          t, v: t.map(() => v), hdg: 90, U: Uw, dirFrom: 90 + p.twa, thrust, side, hub: S.hubForce, drag: t.map(() => need),
          hud: (i) => [['Time', fmt(t[i], 4) + ' s'], ['Mode', S.modes[i], MC[S.modes[i]] || '#e5e7eb'], [washRoot ? 'Arch flexure' : 'Wing root', washRoot ? (wingState[i] === 'WASHOUT' ? 'washing out' : 'self-trimmed') : wingState[i], ['FEATHERED', 'WASHOUT'].includes(wingState[i]) ? '#f59e0b' : undefined], ...(reefable ? [['Wing span', reefState[i], reefState[i] === 'REEFED' ? '#38bdf8' : undefined]] : []), ['True wind', fmt(Uw[i], 3) + ' m/s'], ['Forward wing thrust', fmt(thrust[i], 4) + ' N'], ['Sensed thrust (filtered)', fmt(S.sensed[i], 4) + ' N'], ['Engage below', fmt(engage, 4) + ' N'], ['Disengage above', fmt(disengage, 4) + ' N'], ['Hub/buffer force', fmt(S.hubForce[i], 4) + ' N', S.hubForce[i] > 0 ? '#a855f7' : undefined], ['Required force', fmt(need, 4) + ' N'], ['Buffer charge', fmt(Ech[i] / 1000, 3) + ' kJ']],
        }, { explain: `Solved: wind record (${p.profile}), forward wing thrust from the apparent-wind model${s07?.thrustVsAWA ? ' (Suite 07 table)' : ''}, the mechanical sensor's filtered response, the hysteretic engage/disengage automaton and the hub/buffer force at every ${dt} s step. The car is held at the target speed (switching study, not a journey). Green arrow = wing thrust, purple = backup force, orange = required road force, red = wing side force, blue = true wind. A digital automaton is not proof that the passive mechanism exists.` });
      } });
    }
    out.views.push({ label: '2D animation: state machine & thresholds', icon: '🔀', render: async (el) => {
      const A = await import('../core/anim2d.js');
      const MC = { WIND: '#22c55e', ASSIST: '#84cc16', BACKUP: '#a855f7', BUFFER: '#f59e0b', DECEL: '#ef4444', BRAKING: '#ef4444', PARKED: '#64748b' };
      const nodes = ['WIND', 'ASSIST', 'BACKUP', 'BUFFER', 'DECEL'];
      const Fmax = Math.max(disengage * 1.6, ...S.sensed.filter(Number.isFinite)) || 1;
      A.anim2d(el, { duration: t[t.length - 1], rate: 16, height: 460, label: 'Time', unit: 's', draw: (g, tt, W, H) => {
        const k = A.at(t, tt), i = k.i, cur = S.modes[i];
        // state diagram
        const R0 = Math.min(W * 0.14, 110, (H - 120) / 2), cx = 20 + R0 + 30, cy = 44 + R0;
        nodes.forEach((nm, j) => { const a = -Math.PI / 2 + j * 2 * Math.PI / nodes.length, x = cx + R0 * Math.cos(a), y = cy + R0 * Math.sin(a), on = nm === cur; g.fillStyle = on ? MC[nm] : A.theme().bg2; g.strokeStyle = MC[nm]; g.lineWidth = on ? 3 : 1.5; g.beginPath(); g.arc(x, y, on ? 30 : 24, 0, 7); g.fill(); g.stroke(); g.lineWidth = 1; A.text(g, nm, x - g.measureText(nm).width / 2, y + 4, on ? '#0b1020' : A.theme().fg, 'bold 11px Inter,system-ui,sans-serif'); });
        if (reefable) A.text(g, `Wing span: ${reefState[i]}`, 20, cy + R0 + 72, reefState[i] === 'REEFED' ? '#38bdf8' : '#22c55e', 'bold 13px Inter,system-ui,sans-serif');
        A.text(g, washRoot ? `Arch flexure: ${wingState[i] === 'WASHOUT' ? 'WASHING OUT (side force capped)' : 'self-trimmed'}` : `Wing root: ${wingState[i]}`, 20, cy + R0 + 52, wingState[i] === 'FEATHERED' ? '#f59e0b' : '#22c55e', 'bold 13px Inter,system-ui,sans-serif');
        // threshold gauge (vertical)
        const gx = cx + R0 + 60, gy = 24, gh = 2 * R0 + 40, Y = (F) => gy + gh - Math.max(0, Math.min(1, F / Fmax)) * gh;
        g.fillStyle = A.theme().bg2; g.fillRect(gx, gy, 40, gh); const fs = S.sensed[i]; g.fillStyle = fs >= disengage ? '#22c55e' : fs <= engage ? '#a855f7' : '#eab308'; g.fillRect(gx, Y(fs), 40, gy + gh - Y(fs));
        for (const [F, lab, c] of [[engage, 'engage', '#a855f7'], [disengage, 'disengage', '#22c55e'], [need, 'required', '#ef4444']]) { g.strokeStyle = c; g.setLineDash([4, 3]); g.beginPath(); g.moveTo(gx - 6, Y(F)); g.lineTo(gx + 46, Y(F)); g.stroke(); g.setLineDash([]); A.text(g, `${lab} ${fmt(F, 3)} N`, gx + 50, Y(F) + 4, c); }
        A.text(g, `sensed thrust ${fmt(fs, 4)} N`, gx - 10, gy + gh + 16); A.bar(g, { x: 20, y: H - 26, w: gx - 40, value: Ech[i] / 1000, max: Math.max(1e-6, Ecap / 1000), label: 'Buffer charge', unit: ' kJ', color: '#eab308' });
        // time series
        const sx = gx + 150, sw = W - sx - 12;
        A.seriesPanel(g, { x: sx, y: 14, w: sw, h: (H - 40) / 2, T: t, t: tt, title: 'Wind & forward thrust', series: [{ name: 'true wind m/s', y: Uw, color: '#38bdf8' }, { name: 'thrust N', y: thrust, color: '#22c55e' }, { name: 'sensed N', y: S.sensed, color: '#eab308' }], hlines: [{ y: engage, label: 'engage', color: '#a855f7' }, { y: disengage, label: 'disengage', color: '#22c55e' }] });
        A.seriesPanel(g, { x: sx, y: 22 + (H - 40) / 2, w: sw, h: (H - 40) / 2, T: t, t: tt, title: 'Backup (hub/buffer) force', unit: 'N', series: [{ name: 'hub/buffer', y: S.hubForce, color: '#a855f7' }, { name: 'wing side force', y: side, color: '#ef4444' }] });
      }, explain: 'The solved hybrid automaton: the active mode lights up as the filtered (sensed) forward thrust crosses the separate engage/disengage thresholds; the wing-root lock state (locked / feathered at the breakaway side force) is shown below. Time series panels sweep with the playback.' });
    } });
    out.outputs = { switchCount: S.switches, chatter: S.chatter, engageN: engage, disengageN: disengage, safe: !unsafe && S.chatter === 0, deficitTime_s: deficitTime, requiredForce_N: need };
    out.headline = `${S.switches} transitions, ${S.chatter} chatter events in 10 min; ${fmt(deficitTime, 3)} s with insufficient propulsion. ${unsafe ? 'UNSAFE engagement during braking.' : ''}`;
    out.headlineKind = unsafe || S.chatter > 0 ? 'warn' : 'info';
    void min; void mean; void rad;
    return out;
  },
};
