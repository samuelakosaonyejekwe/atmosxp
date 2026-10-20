// Single Ackermann track rod (vehicle.steerTrackRod): the direct tiller acts on the middle of ONE rigid track rod joining the two
// steering arms (trapezoid behind the axle, arms aimed at the rear wheel). Valid because the front uprights sit on rigid stub-axle arms
// (no relative vertical travel between them). Checks: Ackermann error at full lock, track-rod buckling at the design load, lock travel.
export function trackRodCheck({ track = 1.79, wheelbase = 2.36, kpInset = 0.05, arm = 0.08, lock = 18, rodOD = 0.016, rodT = 0.0015, E = 200e9, Fdesign = 600, betaDeg = null } = {}) {
  const K = track - 2 * kpInset, beta = betaDeg != null ? betaDeg * Math.PI / 180 : Math.atan((K / 2) / wheelbase), tip = (s, d) => { const a0 = Math.PI + s * beta; return [arm * Math.cos(a0 + d), s * K / 2 + arm * Math.sin(a0 + d)]; };
  const Lrod = Math.hypot(...[0, 1].map(i => tip(1, 0)[i] - tip(-1, 0)[i])), rad = lock * Math.PI / 180;
  const pL = tip(1, rad); let dOut = 0, best = Infinity; for (let k = 0; k <= 30000; k++) { const d0 = rad * 1.5 * k / 30000, pR = tip(-1, d0), e = Math.abs(Math.hypot(pL[0] - pR[0], pL[1] - pR[1]) - Lrod); if (e < best) { best = e; dOut = d0; } }
  const dIdeal = Math.atan(1 / (1 / Math.tan(rad) + K / wheelbase)), errDeg = (dOut - dIdeal) * 180 / Math.PI;
  const I = Math.PI / 64 * (rodOD ** 4 - (rodOD - 2 * rodT) ** 4), Pcr = Math.PI ** 2 * E * I / (Lrod * Lrod), SFbuck = Pcr / Fdesign, travel = arm * Math.sin(rad) * 1000;
  const mass = Math.PI / 4 * (rodOD ** 2 - (rodOD - 2 * rodT) ** 2) * Lrod * 7850 + 0.12 /* 2 rod-end joints */;
  return { beta_deg: beta * 180 / Math.PI, rodLength_m: Lrod, innerLock_deg: lock, outer_deg: dOut * 180 / Math.PI, ideal_deg: dIdeal * 180 / Math.PI, ackermannError_deg: errDeg, Pcr_N: Pcr, SFbuckling: SFbuck, rodTravel_mm: travel, mass_kg: mass,
    ok: Math.abs(errDeg) <= 1.0 && SFbuck >= 2 };
}
