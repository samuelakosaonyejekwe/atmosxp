// Safe side force (washout cap) — ONE rule for every suite and every revision:
//   (1) rollover reserve: with the full side force F_s acting at h_CE, the vehicle must still corner at ≥ reserveG before a
//       wheel lifts: m·g·d = m·a·h_cg + F_s·h_CE with d = SSF·h_cg  ⇒  F_s ≤ m·g·h_cg·(SSF − reserveG)/h_CE;
//   (2) straight-line tip-over margin: F_tip = m·g·d/h_CE, F_s ≤ F_tip / tipRatio;
//   (3) tyre grip: F_s ≤ 0.7·μ·m·g.
// The passive washout (flexure stop + bistable de-camber) is set below this cap by the gust overshoot factor (Suite 08).
import { G0 } from './num.js';

export function safeSideForce({ mass, cgHeight, ssf, hCE, mu = 0.95, reserveG = 0.5, tipRatio = 1.4 }) {
  const W = mass * G0, d = ssf * cgHeight, Ftip = W * d / hCE, Froll = Math.max(0, W * cgHeight * (ssf - reserveG) / hCE), Fgrip = 0.7 * mu * W;
  const Fs = Math.min(Froll, Ftip / tipRatio, Fgrip), binding = Fs === Froll ? 'rollover reserve' : Fs === Fgrip ? 'grip' : 'tip-over margin';
  return { Fs, Ftip, Froll, Fgrip, binding, reserveAt: (F) => ssf - F * hCE / (W * cgHeight) }; // reserveAt: lateral g left with side force F
}
