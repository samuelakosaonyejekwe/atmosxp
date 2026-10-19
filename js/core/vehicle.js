// Vehicle-dynamics helpers shared by Suites 01, 10, 11, 12, 19, 20, 21: tyre models, three-wheel load
// distribution and rollover, bicycle / planar models, brake thermal model, passive-switching automaton.
import { G0, SIGMA, luSolve, clamp, brent, rad } from './num.js';

// ---------------- Tyre models ----------------
export const ROADS = {
  'dry asphalt': { mu: 0.95, muLock: 0.75, crrFactor: 1.0 },
  'wet asphalt': { mu: 0.60, muLock: 0.45, crrFactor: 1.05 },
  'dry concrete': { mu: 0.90, muLock: 0.72, crrFactor: 0.95 },
  'gravel': { mu: 0.55, muLock: 0.50, crrFactor: 2.2 },
  'packed snow': { mu: 0.25, muLock: 0.20, crrFactor: 1.8 },
  'ice': { mu: 0.10, muLock: 0.07, crrFactor: 1.2 },
};
// Pacejka Magic Formula (pure slip): y = D sin(C atan(Bx − E(Bx − atan Bx)))
export const magic = (x, B, C, D, E) => D * Math.sin(C * Math.atan(B * x - E * (B * x - Math.atan(B * x))));
// Lateral force; alpha rad. Cornering stiffness Cα = kAlpha·Fz (N/rad). Returns force opposing slip sign convention: Fy = −f(α)
export function mfLateral(alpha, Fz, mu = 0.95, { kAlpha = 12, C = 1.3, E = -0.6 } = {}) { const D = mu * Fz, B = kAlpha * Fz / (C * D || 1); return -magic(alpha, B, C, D, E); }
export function mfLongitudinal(kappa, Fz, mu = 0.95, { kKappa = 18, C = 1.65, E = 0.2 } = {}) { const D = mu * Fz, B = kKappa * Fz / (C * D || 1); return magic(kappa, B, C, D, E); }
// Fiala / brush model (lateral), parabolic pressure, single friction coefficient
export function fiala(alpha, Fz, mu, Calpha) { const ta = Math.tan(alpha), asl = Math.atan(3 * mu * Fz / Calpha); if (Math.abs(alpha) >= asl) return -Math.sign(alpha) * mu * Fz; return -(Calpha * ta - Calpha ** 2 / (3 * mu * Fz) * Math.abs(ta) * ta + Calpha ** 3 / (27 * mu ** 2 * Fz ** 2) * ta ** 3); }
// Dugoff combined-slip model
export function dugoff(kappa, alpha, Fz, mu, Cx, Cy) {
  const ta = Math.tan(alpha), den = 2 * Math.sqrt((Cx * kappa) ** 2 + (Cy * ta) ** 2) || 1e-9, lam = mu * Fz * (1 + kappa) / den;
  const f = lam < 1 ? (2 - lam) * lam : 1;
  return { Fx: Cx * kappa / (1 + kappa) * f, Fy: -Cy * ta / (1 + kappa) * f };
}
// Friction-ellipse combined scaling of pure-slip forces
export function frictionEllipse(Fx0, Fy0, Fz, mu) { const lim = mu * Fz, r = Math.hypot(Fx0 / lim, Fy0 / lim); return r > 1 ? { Fx: Fx0 / r, Fy: Fy0 / r, saturated: true } : { Fx: Fx0, Fy: Fy0, saturated: false }; }
// Rolling-resistance coefficient: inflation, load and speed dependence (SAE J2452-type form)
export function crrModel(Crr0, { p = 450, p0 = 450, Fz = 1000, Fz0 = 1000, v = 5.56, road = 'dry asphalt' } = {}) { return Crr0 * (p / p0) ** -0.4 * (Fz / Fz0) ** 0.1 * (1 + 2.5e-4 * v * v) * (ROADS[road]?.crrFactor ?? 1); }

// ---------------- Three-wheel geometry, loads and rollover ----------------
// Wheel contact points relative to the CG ground projection (x forward, y left)
export function wheelLayout(veh, frontFrac) {
  const L = veh.wheelbase, t = veh.track, f = frontFrac ?? (veh.layout === 'delta' ? 0.42 : 0.6); // f = static front-axle load fraction
  const a = L * (1 - f), b = L * f; // a: CG→front axle, b: CG→rear axle
  return veh.layout === 'delta' ? { a, b, f, pts: [[a, 0, 'Front'], [-b, t / 2, 'Rear left'], [-b, -t / 2, 'Rear right']] } : { a, b, f, pts: [[a, t / 2, 'Front left'], [a, -t / 2, 'Front right'], [-b, 0, 'Rear']] };
}
// Statically determinate normal loads for three supports. forces: [{Fx, Fy, z}] applied above ground (incl. d'Alembert −m·a).
export function threeWheelLoads(pts, W, lift = 0, forces = []) {
  const Mx = forces.reduce((s, f) => s + (f.Fy || 0) * f.z, 0), My = forces.reduce((s, f) => s + (f.Fx || 0) * f.z, 0);
  const A = [[1, 1, 1], pts.map(p => p[1]), pts.map(p => p[0])];
  return luSolve(A, [W - lift, Mx, My]);
}
// Static-stability factor (lateral ay/g at wheel lift) for a trike: tipping about the line joining the outer front and rear contacts
export function trikeSSF(veh, frontFrac) { const { a, b } = wheelLayout(veh, frontFrac), L = veh.wheelbase, t = veh.track, h = veh.cgHeight; return veh.layout === 'delta' ? t * a / (2 * h * L) : t * b / (2 * h * L); }
// Lateral acceleration (g) at which an inside wheel lifts, with an aerodynamic side force Fs at height hs pushing outward
export function rolloverAy(veh, mass, { Fs = 0, hs = 2, frontFrac, lift = 0 } = {}) {
  const { pts } = wheelLayout(veh, frontFrac), W = mass * G0, h = veh.cgHeight;
  const minLoad = (ayg) => Math.min(...threeWheelLoads(pts, W, lift, [{ Fy: -mass * ayg * G0, z: h }, { Fy: -Fs, z: hs }]));
  if (minLoad(0) <= 0) return 0;
  const r = brent(minLoad, 0, 3); return r.ok ? r.x : 3;
}
// Crosswind speed at which the wing side force alone lifts the windward wheel(s) (stationary vehicle)
export function tipOverWind(veh, mass, { S, Cs = 1.2, hs, rho = 1.225, frontFrac } = {}) { const { pts } = wheelLayout(veh, frontFrac), W = mass * G0; const f = (U) => Math.min(...threeWheelLoads(pts, W, 0, [{ Fy: -0.5 * rho * U * U * S * Cs, z: hs }])); const r = brent(f, 0.1, 80); return r.ok ? r.x : Infinity; }

// ---------------- Planar vehicle models ----------------
// Linear bicycle model properties. Cf, Cr axle cornering stiffness (N/rad), a,b CG to axles
export function bicycle({ m, Iz, a, b, Cf, Cr }) {
  const L = a + b, K = m / L * (b / Cf - a / Cr); // understeer gradient rad/(m/s²)
  const Kg = K * G0; // rad/g
  return { L, K, Kg, Kdeg: Kg * 180 / Math.PI, vch: K > 0 ? Math.sqrt(L / K) : Infinity, vcrit: K < 0 ? Math.sqrt(-L / K) : Infinity, yawGain: (u) => u / (L + K * u * u), Iz, m, a, b, Cf, Cr };
}
// Nonlinear planar 3-DOF (vx const, vy, r) + global pose; tyre lateral forces from Magic Formula with load transfer.
// inputs: delta(t) rad, Fyw(t) aero side force (left +), Nw(t) aero yaw moment, u forward speed
export function planarRHS({ m, Iz, a, b, u, FzF, FzR, mu, kAlpha, delta, Fyw, Nw }) {
  return (t, y) => {
    const [vy, r, X, Y, psi] = y, d = delta(t);
    const aF = Math.atan2(vy + a * r, u) - d, aR = Math.atan2(vy - b * r, u);
    const FyF = mfLateral(aF, FzF, mu, { kAlpha }), FyR = mfLateral(aR, FzR, mu, { kAlpha });
    const Fy = FyF * Math.cos(d) + FyR + Fyw(t), Mz = a * FyF * Math.cos(d) - b * FyR + Nw(t);
    return [Fy / m - u * r, Mz / Iz, u * Math.cos(psi) - vy * Math.sin(psi), u * Math.sin(psi) + vy * Math.cos(psi), r];
  };
}

// ---------------- Brake thermal model ----------------
// Lumped disc: m c dT/dt = q_in − h A (T − Ta) − ε σ A (T⁴ − Ta⁴)
export function discStep(T, qIn, dt, { m, c, A, h, eps = 0.55, Ta = 293.15 }) { const qOut = h * A * (T - Ta) + eps * SIGMA * A * (T ** 4 - Ta ** 4); return T + dt * (qIn - qOut) / (m * c); }
export const brakeConvH = (v, D = 0.2) => 10 + 6.0 * Math.pow(Math.max(v, 0), 0.8) * Math.pow(0.2 / D, 0.2); // forced convection on a rotating disc (empirical)

// ---------------- Passive mode-switching automaton ----------------
export const MODES = ['WIND', 'ASSIST', 'BACKUP', 'BUFFER', 'DECEL', 'BRAKING', 'PARKED'];
// thrust[]: forward aerodynamic force history; need: force required; engage/disengage thresholds with hysteresis;
// sensor: 2nd-order mechanical filter (wn, zeta); delay: mechanical engagement latency
export function simulateSwitching(t, thrust, { need, engage, disengage, wn = 6, zeta = 0.5, delay = 0.4, Pbackup = 0, v = 5.56, Ebuf0 = 0, braking = () => false, jammed = false, thresholdScale = 1 } = {}) {
  const dt = t[1] - t[0]; let x = thrust[0], xd = 0, mode = thrust[0] >= need ? 'WIND' : 'BACKUP', pend = null, pendT = 0, Ebuf = Ebuf0;
  const modes = [], sensed = [], torqueSrc = [], events = []; let lastSwitch = -Infinity, chatter = 0;
  const eng = engage * thresholdScale, dis = disengage * thresholdScale;
  for (let i = 0; i < t.length; i++) {
    const F = thrust[i]; const xdd = wn * wn * (F - x) - 2 * zeta * wn * xd; xd += xdd * dt; x += xd * dt; sensed.push(x);
    let want = mode;
    if (braking(t[i])) want = 'BRAKING';
    else if (mode === 'WIND' || mode === 'BRAKING') { if (x < eng) want = Pbackup > 0 ? (x > 0.2 * need ? 'ASSIST' : 'BACKUP') : (Ebuf > 0 ? 'BUFFER' : 'DECEL'); else want = 'WIND'; }
    else { if (x > dis) want = 'WIND'; else if (Pbackup <= 0) want = Ebuf > 0 ? 'BUFFER' : 'DECEL'; }
    if (jammed && ['ASSIST', 'BACKUP', 'BUFFER'].includes(want)) want = 'DECEL';
    if (want !== mode) { if (pend !== want) { pend = want; pendT = t[i]; } if (t[i] - pendT >= delay) { if (t[i] - lastSwitch < 5) chatter++; events.push({ t: t[i], from: mode, to: want }); mode = want; lastSwitch = t[i]; pend = null; } } else pend = null;
    const deficitF = Math.max(0, need - Math.max(0, F));
    let hubF = 0; if (mode === 'ASSIST' || mode === 'BACKUP') hubF = Math.min(deficitF, Pbackup / Math.max(v, 0.3)); if (mode === 'BUFFER' && Ebuf > 0) { hubF = deficitF; Ebuf = Math.max(0, Ebuf - hubF * v * dt); }
    torqueSrc.push(hubF); modes.push(mode);
  }
  return { modes, sensed, hubForce: torqueSrc, events, chatter, switches: events.length, EbufEnd: Ebuf };
}

// ---------------- Acoustics helpers ----------------
export const THIRD_OCT = [25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000];
export function aWeight(f) { const f2 = f * f, ra = 12194 ** 2 * f2 * f2 / ((f2 + 20.6 ** 2) * Math.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2)) * (f2 + 12194 ** 2)); return 20 * Math.log10(ra) + 2.0; }
export const dbSum = (arr) => 10 * Math.log10(arr.reduce((s, L) => s + (Number.isFinite(L) ? 10 ** (L / 10) : 0), 0) || 1e-30);
export const massLawTL = (msurf, f) => Math.max(0, 20 * Math.log10(Math.max(1e-6, msurf * f)) - 47);
export { rad, clamp };
