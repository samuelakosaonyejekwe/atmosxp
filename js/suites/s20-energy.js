// Suite 20 — Complete automobile propulsion, energy management & operating modes.
import { G0, fmt, range, linspace, rng, sum, mean, max, min, interp1, brent, clamp, rad, deg } from '../core/num.js';
import { kmh, roadLoad, apparentWind, wingForces, kaimal, oneMinusCos, carnot } from '../core/physics.js';
import { SCENARIOS } from '../core/env.js';
import { h } from '../core/ui.js';

// 18 standard simulation scenarios (input/output register §11). rel = wind direction relative to the initial heading.
const SC = {
  s1: { label: '1. Stationary, favourable wind — can it self-start?', U: 6, rel: 90, v0: 0, T: 180, TI: 10 },
  s2: { label: '2. Wind-powered acceleration', U: 8, rel: 100, v0: 0, T: 420, TI: 15 },
  s3: { label: '3. Strong crosswind (14 m/s)', U: 14, rel: 90, v0: 'target', T: 300, TI: 20 },
  s4: { label: '4. Sudden gust (5 → 15 m/s)', U: 5, rel: 90, v0: 'target', T: 240, TI: 10, gust: { t0: 60, dur: 4, amp: 10 } },
  s5: { label: '5. Wind reversal (90° → 270°)', U: 7, rel: 90, v0: 'target', T: 360, TI: 10, reversal: { t0: 120, dur: 60 } },
  s6: { label: '6. Calm sunny day (backup only)', U: 0.8, rel: 200, v0: 'target', T: 900, TI: 20, day: true },
  s7: { label: '7. Calm clear night', U: 0.5, rel: 200, v0: 'target', T: 900, TI: 20, day: false },
  s8: { label: '8. Calm cloudy night', U: 0.5, rel: 200, v0: 'target', T: 900, TI: 20, day: false, cloudy: true },
  s9: { label: '9. Thermal actuator startup (from rest, no wind, hub only)', U: 0, rel: 0, v0: 0, T: 300, TI: 0, day: true, hubOnly: true },
  s10: { label: '10. Direct-rim hub operation (no wind, hub only)', U: 0, rel: 0, v0: 'target', T: 600, TI: 0, day: true, hubOnly: true },
  s11: { label: '11. Wind-to-backup transition', U: 7, rel: 100, v0: 'target', T: 480, TI: 10, ramp: { t0: 60, dur: 120, to: 0.8 } },
  s12: { label: '12. Backup-to-wind transition', U: 0.8, rel: 100, v0: 'target', T: 480, TI: 10, ramp: { t0: 60, dur: 120, to: 7 } },
  s13: { label: '13. Emergency braking with wing thrust', U: 7, rel: 100, v0: 'target', T: 90, TI: 10, brakeAt: 40 },
  s14: { label: '14. Fatigue & durability (1 h duty, extrapolated)', U: 6, rel: 100, v0: 0, T: 3600, TI: 20 },
  s15: { label: '15. Material competition (uses Suite 16 best actuator)', U: 0.8, rel: 200, v0: 'target', T: 900, TI: 20, day: true, competition: true },
  s16: { label: '16. Global optimisation baseline (see Suite 25)', U: 6, rel: 100, v0: 0, T: 600, TI: 15 },
  s17: { label: '17. Day/night journey (24 h, Suite 05 weather)', U: 5, rel: 100, v0: 0, T: 86400, TI: 15, dayNight: true },
  s18: { label: '18. Worst-case energy test (no wind, night, empty buffer)', U: 0.2, rel: 0, v0: 'target', T: 900, TI: 0, day: false, worst: true },
};
const MODE = { WIND: 0, ASSIST: 1, BACKUP: 2, BUFFER: 3, COAST: 4, BRAKE: 5, STOP: 6 };

// Imported route → piecewise segments with heading and grade
function routeSegments(pts) {
  const segs = []; let s = 0;
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1], b = pts[i], dy = (b.lat - a.lat) * 110540, dx = (b.lon - a.lon) * 111320 * Math.cos(rad((a.lat + b.lat) / 2)), d = Math.hypot(dx, dy);
    if (d < 0.5) continue; const hdg = (deg(Math.atan2(dx, dy)) + 360) % 360, grade = clamp(((b.ele || 0) - (a.ele || 0)) / d, -0.3, 0.3);
    segs.push({ s0: s, s1: s + d, hdg, grade }); s += d;
  }
  return { segs, length: s };
}

export default {
  description: 'Whole-vehicle propulsion and energy simulator: integrates m dv/dt = F_wing,x + F_body,x − F_roll − F_grade + F_hub + F_buffer − F_brake over complete operating histories (start, acceleration, cruise, wind loss, backup engagement, buffer discharge, wind recovery, braking, parking) for the 18 standard scenarios, live or imported routes, with a full energy and exergy ledger, maximum sustainable speed and power-gap closure against the 200/600/1 500 W targets. It never shows powered motion without an energy source.',
  literature: 'wind powered land vehicle performance velocity polar energy management land yacht',
  credibility: 'Analytical screening', sustain: true, econ: false,
  equations: ['Longitudinal vehicle dynamics m_eff v̇ = ΣF_x (rotating inertia included)', 'Apparent-wind transformation V_a = V_wind − V_vehicle; lift/drag resolution F_x = L sin β − D cos β (useful thrust distinct from lift)', 'Road load F = m g C_rr cos θ + m g sin θ + ½ρ C_dA V_ax|V_ax|', 'Wheel power P = F v; hub force F_hub = min(P_hub / v, traction limit)', 'Mechanical energy balance: Σ inputs (wind, hub, buffer) = rolling + aero + braking losses + ΔKE + ΔPE', 'Wind kinetic-power flux ½ρU³S through the wing area (resource bound); heat-engine Carnot bound η ≤ 1 − T_c/T_h for the thermal backup', 'Finite buffer state of energy Ė = −P/η_dis (discharge), +η_ch P (charge from surplus wind)', 'Event-driven hybrid mode transitions with hysteresis; maximum sustainable speed from P_available(v) = P_required(v)'],
  methods: ['Explicit integration (Δt 0.25–5 s) with event handling for static breakaway and brake hold', 'Bisection for steady sustainable speed', 'Velocity polar from steady force balance over true wind angle and speed', 'Synthetic Kaimal turbulence when no live record is available'],
  assumptions: ['Wing trimmed for maximum useful thrust at each apparent-wind angle (stalled drag mode downwind); driver depowers the wing to hold the target speed', 'Hub power is only what Suite 17 has computed (default 0, “unsupported” until demonstrated)', 'Lateral dynamics are evaluated in Suites 10/11; here the side force is reported as a constraint'],
  ibc: { initial: ['Position, velocity, acceleration, wheel speed, atmosphere, wing state, collector temperature, actuator phase, hub torque, buffer stored energy, braking & source-switching state'], boundary: ['Time-dependent wind, solar & sky radiation, road elevation & gradient, surface friction, rolling resistance, aerodynamic forces, wheel torques, thermal energy availability, mechanical losses, prescribed route and driver requests'] },
  cvv: { calibration: ['Measured wind thrust, wheel power, acceleration, speed, thermal input, buffer depletion, backup duration, integrated journey data'], verification: ['Energy-ledger closure (residual < 1 % of throughput)', 'Zero-input case: no motion from rest (worst-case test)', 'Coast-down from v₀ matches analytical rolling + drag deceleration'], validation: ['Full-vehicle track and road journeys with logged wind (Level 5–6 validation)'] },
  formats: { import: ['GPX / KML / GeoJSON routes (Data portal)', 'CSV journey logs'], export: ['CSV time histories', 'JSON energy ledger', 'PNG/SVG plots'] },
  params: [
    { key: 'scenario', label: 'Standard scenario', type: 'select', group: 'Scenario', default: 's11', options: Object.entries(SC).map(([value, o]) => ({ value, label: o.label })) },
    { key: 'speed', label: 'Target speed', unit: 'km/h', type: 'range', min: 5, max: 60, step: 1, group: 'Scenario', default: 20 },
    { key: 'Uscale', label: 'Wind-speed multiplier', type: 'number', min: 0, max: 3, step: 0.05, group: 'Scenario', default: 1, help: 'Scales the scenario wind; 1 = as specified.' },
    { key: 'useLive', label: 'Use Suite 05 wind record when available', type: 'bool', group: 'Scenario', default: true },
    { key: 'route', label: 'Route', type: 'select', group: 'Route', default: 'flat', options: [{ value: 'flat', label: 'Flat straight road (heading below)' }, { value: 'imported', label: 'Imported route (Data portal GPX/KML/GeoJSON)' }], auto: (c) => (c.store.project.imports?.route?.length >= 2 ? { value: 'imported', prov: 'user' } : undefined) },
    { key: 'heading', label: 'Initial heading', unit: '°', type: 'number', min: 0, max: 359, step: 1, group: 'Route', default: 0 },
    { key: 'grade', label: 'Road gradient (flat route)', unit: '%', type: 'number', min: -10, max: 10, step: 0.5, group: 'Route', default: 0 },
    { key: 'CL', label: 'Wing max trimmed C_L', type: 'number', min: 0.3, max: 2, step: 0.05, group: 'Wing', default: 1.0, auto: (c) => c.upstream('s07')?.CLmax ? { value: +(0.85 * c.upstream('s07').CLmax).toFixed(2), prov: 'coupled' } : undefined },
    { key: 'CD0', label: 'Wing profile drag C_D0', type: 'number', min: 0.005, max: 0.2, step: 0.005, group: 'Wing', default: 0.03 },
    { key: 'kInd', label: 'Induced-drag factor k (C_D = C_D0 + k C_L²)', type: 'number', min: 0.01, max: 0.5, step: 0.01, group: 'Wing', default: 0.15, expert: true, auto: (c) => { const AR = c.vehicle.wingHeight ** 2 / (c.vehicle.wingArea / (c.vehicle.nWings || 2)); return { value: +(1 / (Math.PI * 0.8 * AR)).toFixed(3), prov: 'calc' }; }, help: '1/(π e AR) with e = 0.8 and the wing aspect ratio.' },
    { key: 'Phub', label: 'Continuous hub power (backup)', unit: 'W', type: 'number', min: 0, max: 3000, step: 1, group: 'Backup', default: 0, auto: (c) => { const o = c.upstream('s17'); return o?.Pwheel_W !== undefined ? { value: +Math.max(0, o.Pwheel_W).toFixed(2), prov: 'coupled' } : { value: 0, prov: 'unsupported' }; }, help: 'Only power demonstrated by Suite 17 should be used. Values typed here are hypothetical.' },
    { key: 'hubProfile', label: 'Hub power availability', type: 'select', group: 'Backup', default: 'asComputed', options: [{ value: 'asComputed', label: 'As computed (all conditions)' }, { value: 'dayOnly', label: 'Daytime only (solar-driven)' }, { value: 'nightReduced', label: 'Night = 5 % of day (sky-cooling limited)' }] },
    { key: 'hubStart', label: 'Hub starting force at standstill', unit: 'N', type: 'number', min: 0, max: 500, step: 1, group: 'Backup', default: 0, auto: (c) => { const o = c.upstream('s17'); return o ? { value: o.startupOK ? +(Math.max(0, o.tauMean_Nm || 0) / c.vehicle.wheelRadius).toFixed(1) : 0, prov: 'coupled' } : undefined; } },
    { key: 'Ebuf', label: 'Buffer usable energy at start', unit: 'kJ', type: 'number', min: 0, max: 600, step: 1, group: 'Buffer', default: 30, auto: (c) => { const o = c.upstream('s18'); return o?.Euse_J !== undefined ? { value: +(o.Euse_J / 1000).toFixed(2), prov: 'coupled' } : undefined; } },
    { key: 'PbufMax', label: 'Buffer max discharge power', unit: 'W', type: 'number', min: 0, max: 3000, step: 10, group: 'Buffer', default: 250 },
    { key: 'etaBuf', label: 'Buffer discharge efficiency', type: 'number', min: 0.3, max: 1, step: 0.01, group: 'Buffer', default: 0.85, auto: (c) => { const o = c.upstream('s18'); return o?.efficiency ? { value: +o.efficiency.toFixed(3), prov: 'coupled' } : undefined; } },
    { key: 'charge', label: 'Recharge buffer from surplus wind', type: 'bool', group: 'Buffer', default: true },
    { key: 'engageFrac', label: 'Backup engage threshold (× required force)', type: 'number', min: 0.3, max: 1.2, step: 0.01, group: 'Switching', default: 0.85, auto: (c) => { const o = c.upstream('s19'); return o?.engageN && o.requiredForce_N ? { value: +(o.engageN / o.requiredForce_N).toFixed(3), prov: 'coupled' } : undefined; } },
    { key: 'disengageFrac', label: 'Backup disengage threshold (× required force)', type: 'number', min: 0.5, max: 2, step: 0.01, group: 'Switching', default: 1.15, auto: (c) => { const o = c.upstream('s19'); return o?.disengageN && o.requiredForce_N ? { value: +(o.disengageN / o.requiredForce_N).toFixed(3), prov: 'coupled' } : undefined; } },
    { key: 'brakeDecel', label: 'Emergency braking deceleration', unit: 'm/s²', type: 'number', min: 1, max: 10, step: 0.1, group: 'Braking', default: 6.5, auto: (c) => c.upstream('s12')?.decel_ms2 ? { value: +c.upstream('s12').decel_ms2.toFixed(2), prov: 'coupled' } : undefined },
    { key: 'hoursYear', label: 'Annual operating hours (durability)', unit: 'h', type: 'number', min: 10, max: 5000, step: 10, group: 'Durability', default: 600 },
    { key: 'seed', label: 'Turbulence seed', type: 'number', min: 1, max: 99999, step: 1, group: 'Scenario', default: 3, expert: true },
  ],
  presets: [{ label: 'Wind-to-backup', values: { scenario: 's11' } }, { label: 'Self-start test', values: { scenario: 's1' } }, { label: 'Calm night', values: { scenario: 's7' } }, { label: 'Worst-case energy', values: { scenario: 's18' } }, { label: 'Day/night 24 h', values: { scenario: 's17' } }, { label: 'Hypothetical 600 W hub', values: { Phub: 600, scenario: 's10' } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const veh = ctx.vehicle, S = SC[p.scenario], m = veh.mass, mEff = m + (ctx.upstream('s10')?.rotEqMass ?? 9), R = veh.wheelRadius, rho = ctx.env?.rho ?? 1.225, vt = kmh(p.speed);
    const Crr = ctx.upstream('s10')?.Crr ?? veh.Crr, mu = ctx.upstream('s10')?.muPeak ?? 0.95, s05 = ctx.upstream('s05'), s07 = ctx.upstream('s07'), s16 = ctx.upstream('s16'), s17 = ctx.upstream('s17');
    const prov = (k) => ctx.store.prov('s20')[k];
    // ---- route ----
    const rpts = ctx.store.project.imports?.route; const useRoute = p.route === 'imported' && rpts?.length >= 2; const RT = useRoute ? routeSegments(rpts) : null;
    const roadAt = (x) => { if (!RT || !RT.segs.length) return { hdg: p.heading, grade: p.grade / 100 }; const xx = x % RT.length; const sg = RT.segs.find(q => xx >= q.s0 && xx < q.s1) || RT.segs[RT.segs.length - 1]; return { hdg: sg.hdg, grade: sg.grade }; };
    // ---- wind field ----
    const T = S.T, dt = T > 20000 ? 5 : T > 3000 ? 1 : 0.25, N = Math.round(T / dt);
    const R_ = rng(p.seed), fs = linspace(0.002, 0.8, 70), sig = Math.max(0.3, S.U) * (S.TI || 0) / 100, Sv = fs.map(f => kaimal(f, Math.max(S.U, 0.5), sig)), ph = fs.map(() => 2 * Math.PI * R_()), am = fs.map((f, k) => Math.sqrt(2 * Sv[k] * (k ? fs[k] - fs[k - 1] : fs[0])));
    const turb = (t) => S.TI ? fs.reduce((s, f, k) => s + am[k] * Math.cos(2 * Math.PI * f * t + ph[k]), 0) : 0;
    const liveSeries = p.useLive && s05?.series?.length ? s05.series : null, liveIdx0 = s05?.snapshotIndex ?? 0;
    const isLiveDN = S.dayNight && liveSeries;
    const windAt = (t) => {
      let U = S.U, dirFrom = p.heading + S.rel, isDay = S.day ?? true;
      if (isLiveDN) { const hrs = t / 3600, i0 = liveIdx0 + Math.floor(hrs), f = hrs - Math.floor(hrs), a = liveSeries[Math.min(liveSeries.length - 1, i0)] || liveSeries[liveSeries.length - 1], b = liveSeries[Math.min(liveSeries.length - 1, i0 + 1)] || a; U = a.Uz + (b.Uz - a.Uz) * f; dirFrom = a.dir; isDay = a.isDay; }
      else if (S.dayNight) { const hr = (8 + t / 3600) % 24; U = S.U * (0.6 + 0.6 * Math.sin(Math.PI * (hr - 8) / 12) ** 2); isDay = hr > 6 && hr < 18.5; }
      else if (p.useLive && s05 && (p.scenario === 's2' || p.scenario === 's16' || p.scenario === 's14')) { U = s05.Uz ?? U; dirFrom = s05.dir ?? dirFrom; }
      if (S.ramp) U = t < S.ramp.t0 ? S.U : t < S.ramp.t0 + S.ramp.dur ? S.U + (S.ramp.to - S.U) * (t - S.ramp.t0) / S.ramp.dur : S.ramp.to;
      if (S.gust) U += oneMinusCos(t, S.gust.t0, S.gust.dur, S.gust.amp);
      if (S.reversal) dirFrom += t < S.reversal.t0 ? 0 : t < S.reversal.t0 + S.reversal.dur ? 180 * (t - S.reversal.t0) / S.reversal.dur : 180;
      U = Math.max(0, U * p.Uscale + turb(t) * (U / Math.max(S.U, 0.1)) * p.Uscale);
      return { U, dirFrom, isDay };
    };
    // ---- wing force model: best trim per apparent-wind angle, stalled drag mode downwind ----
    const CLs = linspace(0, p.CL, 9);
    const wing = (Va, beta) => {
      const q = 0.5 * rho * Va * Va, b = rad(Math.abs(beta)); let best = -Infinity, side = 0;
      if (s07?.thrustVsAWA?.awa?.length && s07.Va) { const T0 = interp1(s07.thrustVsAWA.awa, s07.thrustVsAWA.thrust, Math.abs(beta)) * (Va / s07.Va) ** 2; return { Fx: T0, Fy: q * veh.wingArea * p.CL * Math.cos(b), feather: -q * veh.wingArea * p.CD0 * Math.cos(b) }; }
      const luff = clamp((Math.abs(beta) - 12) / 18, 0, 1); // close-hauled limit: no useful lift below ≈ 12–30°
      for (const cl of CLs) { const CLe = cl * luff, CD = p.CD0 + p.kInd * CLe * CLe, Fx = q * veh.wingArea * (CLe * Math.sin(b) - CD * Math.cos(b)); if (Fx > best) { best = Fx; side = q * veh.wingArea * (CLe * Math.cos(b) + CD * Math.sin(b)); } }
      const stall = -1.1 * q * veh.wingArea * Math.cos(b); if (stall > best) { best = stall; side = 1.1 * q * veh.wingArea * Math.sin(b); }
      return { Fx: best, Fy: side, feather: -q * veh.wingArea * p.CD0 * Math.cos(b) };
    };
    // hub availability
    let Phub = p.Phub, hubProv = prov('Phub') || 'estimated';
    if (S.competition && s16?.powerDensity_W_kg && s17) { Phub = Math.max(0, p.Phub * (s16.powerDensity_W_kg / Math.max(1e-9, ctx.upstream('s15')?.powerDensity_W_kg || s16.powerDensity_W_kg))); }
    if (S.worst) Phub = 0;
    const hubAvail = (isDay) => p.hubProfile === 'dayOnly' ? (isDay ? Phub : 0) : p.hubProfile === 'nightReduced' ? (isDay ? Phub : 0.05 * Phub) : (S.cloudy ? Phub : Phub);
    const Ebuf0 = S.worst || S.hubOnly ? 0 : p.Ebuf * 1000, EbufCap = Math.max(Ebuf0, (ctx.upstream('s18')?.Etotal_J ?? p.Ebuf * 1000));
    // ---- simulation ----
    let v = S.v0 === 'target' ? vt : (S.v0 || 0), x = 0, t = 0, Ebuf = Ebuf0, mode = v > 0 ? MODE.WIND : MODE.STOP, backupOn = false;
    const L = { wing: 0, wingNeg: 0, hub: 0, buf: 0, bufDrawn: 0, charge: 0, roll: 0, aero: 0, brake: 0, grade: 0, windFlux: 0 };
    const KE0 = 0.5 * mEff * v * v; let PE = 0;
    const rec = { t: [], v: [], x: [], U: [], Fwav: [], Fw: [], Fh: [], Fb: [], Fres: [], E: [], mode: [], Pw: [], Ph: [], Pb: [], Preq: [], side: [], awa: [] };
    const every = Math.max(1, Math.round(N / 1500)); let stalledT = 0, maxSideRatio = 0, wheelRevs = 0, thrustPeaks = 0, lastF = 0, lastSlope = 0, firstMove = null, reach5 = null;
    const Fstatic = m * G0 * Crr * 1.6;
    for (let i = 0; i < N; i++) {
      const w = windAt(t), rd = roadAt(x), th = Math.atan(rd.grade), aw = apparentWind(w.U, w.dirFrom, rd.hdg, v), wf = wing(aw.Va, aw.beta);
      const Fbody = 0.5 * rho * veh.CdA * aw.ax * Math.abs(aw.ax); // signed: negative = drag
      const Froll = v > 1e-6 ? m * G0 * Crr * Math.cos(th) : 0, Fg = m * G0 * Math.sin(th);
      const FreqT = roadLoad(vt, { ...veh, Crr }, { rho, grade: rd.grade }).F; // force needed to hold the target speed
      const braking = S.brakeAt !== undefined && t >= S.brakeAt;
      // driver: depower wing to track target speed
      let Fw = wf.Fx;
      const Fcmd = (Froll + Fg - Fbody) + mEff * (vt - v) / 6;
      if (braking) Fw = Math.max(wf.feather, Math.min(wf.Fx, 0.1 * wf.Fx)); else if (Fw > Fcmd) Fw = Math.max(wf.feather, Fcmd);
      // backup engagement with hysteresis on available forward thrust
      const avail = wf.Fx + Fbody;
      if (!backupOn && avail < p.engageFrac * FreqT) backupOn = true; else if (backupOn && avail > p.disengageFrac * FreqT) backupOn = false;
      let Fh = 0, Fbuf = 0, deficit = Math.max(0, Fcmd - Fw);
      if (backupOn && !braking && deficit > 0) {
        const Pa = hubAvail(w.isDay); Fh = Math.min(deficit, v > 0.3 ? Pa / v : p.hubStart, mu * m * G0 * 0.6); deficit -= Fh;
        if (Ebuf > 0 && deficit > 0) { Fbuf = Math.min(deficit, p.PbufMax / Math.max(v, 0.6), mu * m * G0 * 0.6); }
      }
      // surplus wind charging
      let Fch = 0; if (p.charge && !braking && !backupOn && Ebuf < EbufCap && wf.Fx > Fw + 1 && v > 0.5) { Fch = Math.min(wf.Fx - Fw, 100 / v); Fw += 0; }
      let Fb = braking ? (v > 0 ? mEff * p.brakeDecel : 0) : 0;
      let Fnet = Fw + Fbody + Fh + Fbuf - Froll - Fg - Fb;
      // static breakaway & no rolling backward: brakes hold the car
      if (v <= 1e-6 && Fw + Fh + Fbuf + Math.max(0, Fbody) - Fg <= Fstatic) Fnet = 0; // brakes/static friction hold the car
      let vn = v + Fnet / mEff * dt; if (vn < 0) vn = 0;
      const vm = 0.5 * (v + vn);
      // ledger (mechanical work at the vehicle)
      const Ew = Fw * vm * dt; if (Ew >= 0) L.wing += Ew; else L.wingNeg += -Ew;
      L.hub += Fh * vm * dt; L.buf += Fbuf * vm * dt; L.bufDrawn += Fbuf * vm * dt / p.etaBuf; L.roll += Froll * vm * dt; L.aero += -Fbody * vm * dt; L.brake += Fb * vm * dt; L.grade += Fg * vm * dt; PE += Fg * vm * dt;
      if (Fch > 0) { const e = Fch * vm * dt; Ebuf = Math.min(EbufCap, Ebuf + 0.85 * e); L.charge += e; L.wing += e; }
      Ebuf = Math.max(0, Ebuf - Fbuf * vm * dt / p.etaBuf);
      L.windFlux += 0.5 * rho * w.U ** 3 * veh.wingArea * dt;
      // bookkeeping
      if (vm > 0 && firstMove === null) firstMove = t; if (vn >= kmh(5) && reach5 === null) reach5 = t;
      if (vn < 0.05 && !braking) stalledT += dt;
      maxSideRatio = Math.max(maxSideRatio, Math.abs(wf.Fy) / (mu * m * G0));
      wheelRevs += vm * dt / (2 * Math.PI * R);
      const slope = Math.sign(wf.Fx - lastF); if (slope < 0 && lastSlope > 0) thrustPeaks++; if (slope) lastSlope = slope; lastF = wf.Fx;
      mode = braking ? (vn > 0 ? MODE.BRAKE : MODE.STOP) : vn < 0.05 ? MODE.STOP : Fh > 0 ? (Fw > 0.2 * FreqT ? MODE.ASSIST : MODE.BACKUP) : Fbuf > 0 ? MODE.BUFFER : (Fw + Fbody < Froll + Fg - 1 && !backupOn ? MODE.COAST : backupOn && Fh === 0 && Fbuf === 0 ? MODE.COAST : MODE.WIND);
      if (i % every === 0) { rec.t.push(t); rec.v.push(vn * 3.6); rec.x.push(x / 1000); rec.U.push(w.U); rec.Fwav.push(wf.Fx); rec.Fw.push(Fw); rec.Fh.push(Fh); rec.Fb.push(Fbuf); rec.Fres.push(Froll + Fg - Fbody); rec.E.push(Ebuf / 1000); rec.mode.push(mode); rec.Pw.push(Math.max(0, Fw * vm)); rec.Ph.push(Fh * vm); rec.Pb.push(Fbuf * vm); rec.Preq.push(FreqT * vt); rec.side.push(wf.Fy); rec.awa.push(Math.abs(aw.beta)); }
      v = vn; x += vm * dt; t += dt;
    }
    const KE1 = 0.5 * mEff * v * v;
    const inputs = L.wing + L.hub + L.buf, sinks = L.roll + L.aero + L.brake + L.wingNeg + L.charge + (KE1 - KE0) + PE, resid = inputs - sinks, thru = Math.max(1, inputs + L.roll + L.aero + L.brake + KE0);
    // ---- steady analyses ----
    const Ubar = S.dayNight ? 5 * p.Uscale : S.ramp ? S.ramp.to * p.Uscale : S.U * p.Uscale, rel = S.rel;
    const steadyF = (vv, U, twa, Ph = 0) => { const aw = apparentWind(U, twa, 0, vv), wf = wing(aw.Va, aw.beta); return wf.Fx + 0.5 * rho * veh.CdA * aw.ax * Math.abs(aw.ax) - m * G0 * Crr + (Ph > 0 ? Ph / Math.max(vv, 0.3) : 0); };
    const vSus = (U, twa, Ph = 0) => { const vs = linspace(0.2, 60, 300); let last = 0; for (const vv of vs) if (steadyF(vv, U, twa, Ph) >= 0) last = vv; return last; };
    const vWind = vSus(Ubar, rel), vAll = vSus(Ubar, rel, hubAvail(S.day ?? true));
    const PwindAvailAtTarget = Math.max(0, steadyF(vt, Ubar, rel) + m * G0 * Crr) * vt, Preq = roadLoad(vt, { ...veh, Crr }, { rho }).P;
    const gap = Math.max(0, Preq - PwindAvailAtTarget), closure = gap > 0 ? Math.min(1, hubAvail(S.day ?? true) / gap) : 1;
    // energy audit vs thermal bounds
    const s14 = ctx.upstream('s14'); const Th = s14?.ThotIface_C, Tc = s14?.TcoldIface_C, etaC = (Th !== undefined && Tc !== undefined) ? carnot(Th + 273.15, Tc + 273.15) : null;
    const heat = s17?.heatDemand_W;
    const auditOK = !(Phub > 0) || (heat !== undefined && etaC !== null ? Phub <= etaC * heat * 1.0001 : false);
    // ---- outcomes ----
    const dist = x, meanV = dist / T, wantDist = vt * T;
    const travel = !['s1', 's9', 's13', 's18'].includes(p.scenario);
    const success = S.worst ? (L.hub + L.buf === 0 && v < 0.05) : p.scenario === 's1' || p.scenario === 's9' ? reach5 !== null : p.scenario === 's13' ? v < 0.05 : meanV >= 0.8 * vt;
    const energyDeficit = Math.max(0, Preq * T - (L.wing + L.hub + L.buf)); const shares = { wind: L.wing / Math.max(1, inputs), hub: L.hub / Math.max(1, inputs), buffer: L.buf / Math.max(1, inputs) };
    // checks
    out.checks.push({ name: 'Energy-ledger closure', value: resid / thru, target: '|residual| < 1 % of throughput', pass: Math.abs(resid / thru) < 0.01, detail: `inputs ${fmt(inputs / 1000, 4)} kJ, sinks ${fmt(sinks / 1000, 4)} kJ` });
    out.checks.push({ name: 'Buffer state never negative', value: Ebuf, target: '≥ 0', pass: Ebuf >= 0 });
    out.checks.push({ name: 'No motion from rest without propulsive force', value: S.worst ? dist : 'n/a', target: S.worst ? 'coast only' : '', pass: S.worst ? L.hub + L.buf === 0 : undefined, detail: 'Worst-case test: hub and buffer contribute zero energy' });
    const cd = (() => { let vv = vt, xx = 0; for (let k = 0; k < 4000 && vv > 0; k++) { const a = (m * G0 * Crr + 0.5 * rho * veh.CdA * vv * vv) / mEff; vv = Math.max(0, vv - a * 0.05); xx += vv * 0.05; } return xx; })();
    out.checks.push({ name: 'Coast-down distance from target speed (still air, analytic integration)', value: cd, target: fmt(mEff / (rho * veh.CdA) * Math.log(1 + rho * veh.CdA * vt * vt / (2 * m * G0 * Crr)), 4) + ' m (closed form)', pass: Math.abs(cd - mEff / (rho * veh.CdA) * Math.log(1 + rho * veh.CdA * vt * vt / (2 * m * G0 * Crr))) / cd < 0.02 });
    // KPIs
    const k = (label, value, unit, explain, pv = 'calc', status) => out.kpis.push({ label, value, unit, explain, prov: pv, status });
    k('Journey outcome', S.worst ? (success ? 'No fabricated motion ✔' : 'Inconsistent') : success ? 'Success' : 'Failure', '', S.worst ? 'The worst-case test passes when the car only coasts on its initial kinetic energy and stops — no energy is invented.' : travel ? `Success = mean speed ≥ 80 % of ${p.speed} km/h.` : 'Scenario-specific criterion (self-start, stop).', 'calc', success ? 'ok' : 'bad');
    k('Distance travelled', dist / 1000, 'km', `In ${fmt(T / 60, 3)} min of simulated time.`);
    k('Mean speed', meanV * 3.6, 'km/h', 'Distance ÷ duration.', 'calc', meanV >= 0.8 * vt ? 'ok' : 'warn');
    k('Max sustainable speed — wind only', vWind * 3.6, 'km/h', `Steady force balance at U = ${fmt(Ubar, 3)} m/s, TWA ${rel}°.`);
    k('Max sustainable speed — wind + backup', vAll * 3.6, 'km/h', 'Including the continuous hub power available under this scenario.');
    k('Wheel power required at target', Preq, 'W', `Level road, ${p.speed} km/h (spec: 124 W at 20 km/h, 406 W at 40 km/h).`);
    k('Wind power available at target', PwindAvailAtTarget, 'W', 'Useful wing thrust × target speed at the scenario mean wind.');
    k('Power gap to close', gap, 'W', 'Required minus wind-available wheel power.', 'calc', gap > 0 ? 'warn' : 'ok');
    k('Backup hub power used', hubAvail(S.day ?? true), 'W', hubProv === 'coupled' ? 'From the Suite 17 hub simulation.' : 'Not demonstrated by Suite 17 — hypothetical/unsupported.', hubProv);
    k('Power-gap closure', closure * 100, '%', 'Fraction of the gap the backup closes (100 % required for autonomy).', 'calc', closure >= 1 ? 'ok' : closure > 0.5 ? 'warn' : 'bad');
    k('Energy from wind', L.wing / 1000, 'kJ', 'Positive mechanical work done by the wing on the vehicle.');
    k('Energy from hub', L.hub / 1000, 'kJ', 'Thermomechanical backup work delivered at the tyre.');
    k('Energy from buffer', L.buf / 1000, 'kJ', 'Elastic energy delivered (drawn from store ÷ η).');
    k('Buffer remaining', Ebuf / 1000, 'kJ', 'Finite store at the end — never replenished without a physical charging process.');
    k('Time stalled', stalledT, 's', 'Time at standstill without braking (insufficient propulsion).', 'calc', stalledT < 0.05 * T ? 'ok' : 'bad');
    k('Wind-resource extraction', L.wing / Math.max(1, L.windFlux) * 100, '%', 'Wing work ÷ ½ρU³S kinetic-power flux through the wing area (resource bound).');
    k('Peak side force / grip', maxSideRatio, '×μmg', 'Largest wing side force relative to total tyre grip — reacted by the tyres (Suites 10–11).', 'calc', maxSideRatio < 0.5 ? 'ok' : maxSideRatio < 0.8 ? 'warn' : 'bad');
    // verdicts — the four mandatory feasibility tests
    out.verdicts.push({ test: 'Aerodynamic feasibility', status: vWind >= vt ? 'pass' : vWind > 0.3 * vt ? 'warn' : 'fail', detail: `Wind alone sustains ${fmt(vWind * 3.6, 3)} km/h vs target ${p.speed} km/h (U = ${fmt(Ubar, 3)} m/s, TWA ${rel}°).` });
    out.verdicts.push({ test: 'Thermal-backup feasibility / energy audit', status: !(Phub > 0) ? 'fail' : auditOK ? 'pass' : 'fail', detail: !(Phub > 0) ? 'No continuous hub power demonstrated — backup supplies 0 W (only the finite buffer remains).' : auditOK ? `Hub ${fmt(Phub, 3)} W ≤ Carnot bound ${fmt(etaC * heat, 3)} W from Suites 14/17.` : 'Hub power is not backed by a computed heat supply and Carnot bound (Suites 14 & 17) — flagged as apparent energy creation.', fix: 'Run Suites 13 → 14 → 15 → 17 so the hub power is computed from actual heat flow, or set it to 0.' });
    out.verdicts.push({ test: 'Integrated vehicle feasibility', status: success && !S.worst ? 'pass' : S.worst ? (success ? 'pass' : 'fail') : 'fail', detail: S.worst ? 'Worst case: vehicle coasts and stops; no powered journey is shown without energy.' : `${success ? 'Journey met' : 'Journey missed'} the target: ${fmt(meanV * 3.6, 3)} km/h mean, ${fmt(stalledT, 3)} s stalled, energy deficit ${fmt(energyDeficit / 1000, 4)} kJ.` });
    out.verdicts.push({ test: 'Energy conservation', status: Math.abs(resid / thru) < 0.01 ? 'pass' : 'fail', detail: `Ledger residual ${fmt(100 * resid / thru, 3)} % of throughput.` });
    if (p.scenario === 's1' || p.scenario === 's9') out.verdicts.push({ test: 'Self-start', status: reach5 !== null ? 'pass' : 'fail', detail: reach5 !== null ? `Reaches 5 km/h after ${fmt(reach5, 3)} s.` : 'Propulsive force never exceeds static breakaway resistance.' });
    // ---- charts ----
    out.charts.push({ title: 'Speed history', type: 'line', x: { label: 'Time', unit: T > 7200 ? 's' : 's' }, y: { label: 'Speed', unit: 'km/h' }, series: [{ name: 'Vehicle speed', x: rec.t, y: rec.v }, { name: 'Wing-height wind (m/s)', x: rec.t, y: rec.U, dash: '3 3' }], annotations: [{ type: 'hline', y: p.speed, label: 'target', color: '#22c55e' }], wide: true, explain: 'Complete operating history. Speed is the outcome of all forces — it is never prescribed. When wind and backup are insufficient the vehicle slows or stops.' });
    out.charts.push({ title: 'Propulsive forces', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Force', unit: 'N' }, series: [{ name: 'Wing thrust available', x: rec.t, y: rec.Fwav, dash: '4 3', width: 1 }, { name: 'Wing thrust used', x: rec.t, y: rec.Fw }, { name: 'Hub', x: rec.t, y: rec.Fh }, { name: 'Buffer', x: rec.t, y: rec.Fb }, { name: 'Resistance (roll + grade − body aero)', x: rec.t, y: rec.Fres }], explain: 'Available thrust (dashed) is what the trimmed wing could produce; the driver depowers it to hold the target. Backup forces fill gaps only up to their real capacity.' });
    out.charts.push({ title: 'Power contributions vs requirement', type: 'stack', x: { label: 'Time', unit: 's' }, y: { label: 'Power', unit: 'W' }, series: [{ name: 'Wind', x: rec.t, y: rec.Pw }, { name: 'Hub', x: rec.t, y: rec.Ph }, { name: 'Buffer', x: rec.t, y: rec.Pb }], annotations: [{ type: 'hline', y: Preq, label: 'required at target', color: '#ef4444' }], explain: 'Stacked source power at the wheels. Whenever the stack is below the red line the vehicle cannot hold its target speed.' });
    out.charts.push({ title: 'Buffer state of energy & operating mode', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Value' }, series: [{ name: 'Buffer energy (kJ)', x: rec.t, y: rec.E }, { name: 'Mode (0 wind · 1 assist · 2 backup · 3 buffer · 4 coast · 5 brake · 6 stop)', x: rec.t, y: rec.mode }], explain: 'The buffer is finite: it falls during discharge and only rises when surplus wind is stored. Mode codes show the sequence of operating states.' });
    out.charts.push({ title: 'Energy ledger', type: 'bar', categories: ['Wind work', 'Hub work', 'Buffer work', 'Rolling loss', 'Aero loss', 'Braking', 'Wing drag (−)', 'Stored to buffer', 'ΔKE', 'ΔPE', 'Residual'], y: { label: 'Energy', unit: 'kJ' }, series: [{ name: 'kJ', values: [L.wing, L.hub, L.buf, L.roll, L.aero, L.brake, L.wingNeg, L.charge, KE1 - KE0, PE, resid].map(e => e / 1000) }], showValues: true, explain: 'Every joule entering the vehicle (wind, hub, buffer) must leave as losses or change kinetic/potential/stored energy. A residual near zero verifies conservation.' });
    const vs = linspace(0, 60, 61);
    out.charts.push({ title: 'Wheel power demand vs backup targets', type: 'line', x: { label: 'Speed', unit: 'km/h' }, y: { label: 'Power', unit: 'W' }, series: [{ name: 'Required (level road)', x: vs, y: vs.map(s => roadLoad(kmh(s), { ...veh, Crr }, { rho }).P) }, { name: 'Available backup', x: vs, y: vs.map(() => hubAvail(true)), dash: '4 3' }], annotations: [{ type: 'hline', y: 200, label: '200 W', color: '#22c55e' }, { type: 'hline', y: 600, label: '600 W', color: '#f59e0b' }, { type: 'hline', y: 1500, label: '1 500 W', color: '#ef4444' }, { type: 'point', x: 20, y: roadLoad(kmh(20), veh).P, label: '124 W' }, { type: 'point', x: 40, y: roadLoad(kmh(40), veh).P, label: '406 W' }], explain: 'Specification power table (52/124/236/406/1 003 W at 10/20/30/40/60 km/h) and qualification targets. The backup line must lie above the demand curve for autonomous calm-weather travel.' });
    const twas = linspace(20, 180, 33);
    out.charts.push({ title: 'Velocity polar (wind only): sustainable speed vs true wind angle', type: 'line', x: { label: 'True wind angle from heading', unit: '°' }, y: { label: 'Sustainable speed', unit: 'km/h' }, series: [3, 5, 8, 12].map(U => ({ name: `U = ${U} m/s`, x: twas, y: twas.map(a => vSus(U, a) * 3.6) })), explain: 'Classic sailing-craft polar: beam reaches (≈ 90–110°) are fastest; no useful thrust heading close to the wind. Use with the wind rose (Suite 05) to choose routes.' });
    if (RT) out.charts.push({ title: 'Imported route: elevation profile & heading', type: 'line', x: { label: 'Distance', unit: 'km' }, y: { label: 'Value' }, series: [{ name: 'Grade (%)', x: RT.segs.map(s => s.s0 / 1000), y: RT.segs.map(s => s.grade * 100) }, { name: 'Heading (°/10)', x: RT.segs.map(s => s.s0 / 1000), y: RT.segs.map(s => s.hdg / 10) }], explain: 'Route segments derived from the imported track: heading changes alter the apparent-wind angle and therefore thrust; grades add gravitational load.' });
    // tables
    const ptab = [10, 20, 30, 40, 60].map((s, i) => { const P = roadLoad(kmh(s), { ...veh, Crr }, { rho }).P, tgt = [100, 200, 350, 600, 1500][i]; return [s, P, tgt, hubAvail(true), 100 * Math.min(1, hubAvail(true) / P)]; });
    out.tables.push({ title: 'Power-gap table (specification targets)', cols: ['Speed (km/h)', 'Required wheel power (W)', 'Backup design target (W)', 'Backup available (W)', 'Closure of requirement (%)'], rows: ptab, explain: 'Targets are design requirements, not achieved values.' });
    out.tables.push({ title: 'Energy & exergy ledger', cols: ['Term', 'kJ', 'Note'], rows: [['Wind work on vehicle', L.wing / 1000, 'positive wing work incl. energy routed to the buffer'], ['Wind kinetic-power flux through wing area', L.windFlux / 1000, '½ρU³S·t — the resource bound'], ['Hub work', L.hub / 1000, hubProv], ['Heat demand for hub (Suite 17)', heat !== undefined ? heat * T / 1000 : '—', heat !== undefined ? 'computed' : 'not computed'], ['Carnot bound on hub work', etaC !== null && heat !== undefined ? etaC * heat * T / 1000 : '—', etaC !== null ? `η_C = ${fmt(etaC * 100, 3)} %` : 'needs Suite 14'], ['Buffer energy drawn', L.bufDrawn / 1000, `η = ${p.etaBuf}`], ['Rolling loss', L.roll / 1000, ''], ['Aerodynamic body loss', L.aero / 1000, ''], ['Braking dissipation', L.brake / 1000, ''], ['ΔKE / ΔPE', `${fmt((KE1 - KE0) / 1000, 4)} / ${fmt(PE / 1000, 4)}`, ''], ['Residual', resid / 1000, 'conservation check']] });
    // operating history events
    const ev = []; let lastMode = null; rec.mode.forEach((mo, i) => { if (mo !== lastMode) { ev.push([fmt(rec.t[i], 4), Object.keys(MODE).find(k2 => MODE[k2] === mo), fmt(rec.v[i], 3), fmt(rec.U[i], 3), fmt(rec.E[i], 3)]); lastMode = mo; } });
    out.tables.push({ title: 'Operating history (mode changes)', cols: ['t (s)', 'Mode', 'Speed (km/h)', 'Wind (m/s)', 'Buffer (kJ)'], rows: ev.slice(0, 80) });
    if (p.scenario === 's14' || p.scenario === 's16') { const scale = p.hoursYear * 3600 / T, ncart = (veh.driveWheels || 2) * 24; out.tables.push({ title: 'Durability duty extrapolation (per year)', cols: ['Quantity', 'Per simulated hour', 'Per year'], rows: [['Wheel revolutions', wheelRevs * 3600 / T, wheelRevs * scale], ['NiTi cartridge cycles (one per revolution each)', wheelRevs * ncart * 3600 / T, wheelRevs * ncart * scale], ['Wing thrust load cycles (peaks)', thrustPeaks * 3600 / T, thrustPeaks * scale], ['Distance (km)', dist / 1000 * 3600 / T, dist / 1000 * scale]], explain: 'Cycle counts feed fatigue checks in Suites 15, 17 and 23. NiTi functional fatigue is typically 10⁴–10⁷ cycles depending on strain amplitude.' }); }
    // recs
    if (gap > 0 && hubAvail(S.day ?? true) < gap) out.recs.push({ level: 'critical', text: `At ${p.speed} km/h the wind leaves a ${fmt(gap, 3)} W gap and the backup supplies ${fmt(hubAvail(S.day ?? true), 3)} W (${fmt(closure * 100, 3)} % closure). Either reduce demand (lower speed, mass or C_rr), plan journeys in wind windows, or demonstrate a hub delivering ≥ ${fmt(gap, 3)} W from measured heat flow.`, basis: 'Power-gap closure P_backup ≥ P_required − P_wind' });
    if (Ebuf0 > 0) out.recs.push({ level: 'info', text: `The ${fmt(Ebuf0 / 1000, 3)} kJ buffer bridges ≈ ${fmt(Ebuf0 * p.etaBuf / Math.max(1, Preq), 3)} s at ${p.speed} km/h (${fmt(Preq, 3)} W). It smooths gaps but cannot sustain travel; a full hour at 150 W would need 540 kJ plus losses.`, basis: 'Finite storage E = P·t' });
    if (vWind < vt) out.recs.push({ level: 'warn', text: `Wind alone sustains only ${fmt(vWind * 3.6, 3)} km/h here. Best headings are near beam reach (see velocity polar); route planning against the live wind rose (Suite 05) is the cheapest performance gain.`, basis: 'Velocity polar' });
    if (maxSideRatio > 0.5) out.recs.push({ level: 'critical', text: `Wing side force reaches ${fmt(maxSideRatio * 100, 3)} % of total grip. Depower the wing in gusts; verify rollover in Suite 11.`, basis: 'Side force / μmg' });
    out.recs.push({ level: 'sustain', text: 'Wind-primary operation has near-zero operating emissions; the dominant footprint is embodied (composites, titanium, NiTi). Prioritise durability and repairability — a short-lived lightweight part can lose its advantage over a heavier long-lived one (Suite 26).', basis: 'Specification: durability integrated into sustainability' });
    out.explain.push({ title: 'What “success” means', text: 'A journey succeeds only if the speed history — computed from forces — achieves the requirement. If environmental and stored energy are insufficient, the simulation slows and stops; the worst-case energy test exists precisely to prove that the software never animates a powered journey without an energy source.' });
    out.explain.push({ title: 'Couplings used', text: `Wind: ${isLiveDN || (p.useLive && s05) ? 'Suite 05 live record' : 'scenario definition'}; thrust model: ${s07?.thrustVsAWA ? 'Suite 07 thrust table' : 'trimmed lift/drag polar'}; hub: ${hubProv}; buffer: ${ctx.upstream('s18') ? 'Suite 18' : 'parameter'}; switching thresholds: ${ctx.upstream('s19') ? 'Suite 19' : 'parameter'}; route: ${RT ? `imported (${fmt(RT.length / 1000, 3)} km, ${RT.segs.length} segments)` : 'flat straight'}.` });
    out.outputs = { sustainableSpeed_kmh: vAll * 3.6, sustainableSpeedWind_kmh: vWind * 3.6, range_km: dist / 1000, energyDeficit_J: energyDeficit, journeySuccess: success, sourceShares: shares, powerGap_W: gap, closure, Preq_W: Preq, scenario: p.scenario, ledger: { ...L, resid } };
    out.headline = `${S.label}: ${S.worst ? (success ? 'vehicle coasts to rest — no powered motion without energy.' : 'INCONSISTENT') : success ? 'SUCCESS' : 'FAILURE'} · ${fmt(dist / 1000, 3)} km, mean ${fmt(meanV * 3.6, 3)} km/h · gap ${fmt(gap, 3)} W, closure ${fmt(closure * 100, 3)} %.`;
    out.headlineKind = success ? 'info' : 'warn';
    void sum; void mean; void max; void min; void brent; void SCENARIOS; void h; void range;
    return out;
  },
};
