// AeroArch aerodynamics shared by Suites 06, 07, 09 and 25 (Revisions 4–5): Prandtl box wing of the two canted legs
// mirrored in the body plane, lateral leg interference, lifting body, compliant-flexure self-trim within ±δmax,
// de-cambering lift cap (passive gust washout), best-heading sustainable speed, and the Revision-3 tandem comparator
// evaluated with identical tyre and drag assumptions.
import { coeffs, finiteWing, polar as makePolar } from './aero.js';
import { resampleAirfoil } from './mesh.js';
import { naca4, vehicleAssembly } from './cad.js';
import { apparentWind, wingForces } from './physics.js';
import { rad, deg, linspace, clamp, G0 } from './num.js';

const polarCache = new Map();
export function sectionPolar(code = '7418', Re = 5e5) {
  const key = code + '@' + Math.round(Math.log10(Re) * 10); if (polarCache.has(key)) return polarCache.get(key);
  const af = naca4(code, 161), pts = resampleAirfoil(af.points, 140), pol = makePolar(pts, Re, { alphas: Array.from({ length: 31 }, (_, i) => i - 6), xtrU: 0.05, xtrL: 0.05 });
  polarCache.set(key, pol); return pol;
}

// geometry & box-wing factors from the parametric CAD
export function archGeometry(veh) {
  const meta = vehicleAssembly(veh, {}).meta, AM = meta.arch; if (!AM) return null;
  const legLen = AM.legHeight / Math.cos(rad(AM.cant)), gMean = (AM.gap + AM.bridgeSpan) / 2, b = 2 * legLen, S = veh.wingArea;
  const k = (1 + 0.45 * gMean / b) / (1.04 + 2.81 * gMean / b), AR = b * b / (2 * S), ARe = AR / k, cmean = S / 2 / legLen;
  return { meta, AM, legLen, gMean, b, k, AR, ARe, S, cmean, cosL: Math.cos(rad(AM.sweep)), hCE: AM.zRoot + 0.45 * AM.legHeight, height: meta.height, xWing: meta.wingX[0] };
}

// lifting body (plan-form airfoil with ground reflection) — same model as Suite 07
export function bodyLift(veh, bAbs, VA, rho = 1.225, { camberPct = 4, fill = 0.78, e = 0.8 } = {}) {
  if (bAbs < 0.5) return { L: 0, Di: 0, thrust: 0, side: 0, CL: 0 };
  const Lb = veh.length, hb = veh.height, Sb = fill * Lb * hb, ARb = 2 * hb * hb / Sb, a = rad(Math.min(bAbs, 180 - bAbs)), m = camberPct / 100;
  const CL = (Math.PI * ARb / 2) * (a + (bAbs <= 90 ? 2 * m : 0)) * Math.cos(a) ** 2 + 2 * Math.sin(a) ** 2 * Math.cos(a), q = 0.5 * rho * VA * VA, L = q * Sb * CL, Di = q * Sb * CL * CL / (Math.PI * ARb * e), b = rad(bAbs);
  return { L, Di, CL, thrust: L * Math.sin(b) - Di * Math.cos(b), side: L * Math.abs(Math.cos(b)) + Di * Math.sin(b) };
}

// safe side force (tyre grip & static tip-over) — lift is capped (de-cambering) to stay inside it
export function safeSide(veh, hCE, { mu = 0.95, ssf } = {}) { const s = ssf ?? stabilityFactor(veh); return Math.min(0.7 * mu * veh.mass * G0, 0.8 * s * veh.mass * G0 * veh.cgHeight / hCE); }
// static stability factor of a tadpole trike: lateral distance CG → tipping axis (front contact → rear contact) / CG height
export function stabilityFactor(veh) { const fF = veh.frontLoad ?? veh.frontFrac ?? 0.6, a = fF * veh.wheelbase; return (veh.track / 2) * (a / veh.wheelbase) / veh.cgHeight; }

// forces on the arch at apparent-wind angle bAbs and angle of attack alpha (deg); no cap
export function archForces(G, pol, veh, bAbs, alpha, VA, rho = 1.225, opts = {}) {
  // stacked lifting line (ground image, body as the lower lifting surface) in attached flow; the earlier root-mirror model
  // is kept for large apparent-wind angles / post-stall, blended over β = 35…45°
  const useStack = (opts.stack ?? veh.stackLL ?? true) && Math.abs(alpha) <= (pol.alphaStall ?? 15) + 4 && bAbs <= 45;
  if (useStack) { const ST = G.ST || (G.ST = vehicleStack(veh, stackOpts(veh))), Fs = stackForces(ST, pol, veh, bAbs, alpha, VA, rho, { camberPct: opts.camberPct ?? 4, interf: opts.interf ?? 1, bridgeCdA: 0.012 * (G.AM.bridgeArea || 0) });
    if (bAbs <= 35) return Fs; const w = (45 - bAbs) / 10, Fo = archForcesRootMirror(G, pol, veh, bAbs, alpha, VA, rho, opts), mix = (k) => w * Fs[k] + (1 - w) * Fo[k];
    return { ...Fo, CL: mix('CL'), CD: mix('CD'), L: mix('L'), D: mix('D'), wingThrust: mix('wingThrust'), wingSide: mix('wingSide'), net: mix('net'), side: mix('side'), sideEq: mix('sideEq'), CLsys: w * Fs.CLsys + (1 - w) * Fo.CLsys, CDsys: w * Fs.CDsys + (1 - w) * Fo.CDsys }; }
  return archForcesRootMirror(G, pol, veh, bAbs, alpha, VA, rho, opts);
}
export const stackOpts = (veh) => ({ deckSeal: veh.deckSeal ?? 0, groundSeal: veh.groundSeal ?? 0, bodyEta: veh.bodyEta ?? 0.6, bodyClmax: 0.6, rootBootH: veh.rootBootH ?? 0, skirtGap: veh.skirtGap ?? null, archFill: veh.archFill || 'hollow', fillFrac: veh.fillFrac ?? 0.3 });
// opening-fill geometry: added area, its centroid height above the crossbeam and the torsion-constraint factor on the legs
export function archFillProps(veh, G) { const AM = G.AM, h = AM.legHeight * Math.cos(rad(AM.cant)), f = veh.fillFrac ?? 0.3, fill = veh.archFill || 'hollow';
  if (fill === 'solid') return { area: h * (AM.rootChord + AM.tipChord) / 2, zc: AM.zRoot + 0.45 * h, torsionFactor: 2.0, parts: 3 };
  if (fill === 'hybrid') return { area: 0.5 * f * h * AM.rootChord + 2 * 0.04, zc: AM.zRoot + f * h / 3, torsionFactor: 1 / Math.sqrt(Math.max(0.2, 1 - f)), parts: 3 };
  return { area: 0, zc: AM.zRoot, torsionFactor: 1, parts: 0 }; }
function archForcesRootMirror(G, pol, veh, bAbs, alpha, VA, rho = 1.225, { camberPct = 4, interf = 1, oswald = 0.85 } = {}) {
  const C3 = (a) => { const c = coeffs(pol, a); if (Math.abs(a) > pol.alphaStall + 8) return c; const f = finiteWing(c.CL * G.cosL, c.CD, G.ARe, oswald); return { CL: f.CL, CD: f.CD }; };
  const c1 = C3(alpha), eps = deg(c1.CL / (Math.PI * G.ARe)) * Math.abs(Math.sin(rad(bAbs))) * Math.exp(-G.gMean * Math.abs(Math.cos(rad(bAbs))) / (0.5 * G.cmean));
  const cw = C3(alpha + 0.3 * eps), cl = C3(alpha - eps), e = rad(eps), CL = (cw.CL + cl.CL * Math.cos(e)) / 2, CD = (cw.CD + cl.CD + cl.CL * Math.sin(e)) / 2;
  const F = wingForces(VA, bAbs, CL * interf, CD, G.S, rho), Db = 0.5 * rho * (veh.CdA + 0.012 * (G.AM.bridgeArea || 0)) * VA * VA, B = bodyLift(veh, bAbs, VA, rho, { camberPct });
  // sideEq: side force at the arch centre of effort giving the same overturning moment (the body forces act lower)
  const sB = B.side + Db * Math.sin(rad(bAbs)), zB = (veh.wheelRadius ?? 0.3) + (veh.height ?? 1) / 2;
  return { CL, CD, CLsys: CL * interf, CDsys: CD, eps, L: F.L, D: F.D, wingThrust: F.thrust, wingSide: F.side, body: B, Db, net: F.thrust + B.thrust - Db * Math.cos(rad(bAbs)), side: F.side + sB, sideEq: F.side + sB * zB / G.hCE };
}
// ---------------------------------------------------------------------------------------------------------------------
// Stacked vertical lifting line with the ground as an image plane (Revision 5.1 round 5, applied to EVERY revision).
// Prandtl lifting line discretised into horseshoe vortices along z (cosine spacing per segment); the ground is an exact
// image plane (Γ(−z) = Γ(z), method of images, e.g. Katz & Plotkin, Low-Speed Aerodynamics, 2001, ch. 8/12), so a lifting
// surface that reaches the ground behaves as one of twice the span. Gaps between surfaces carry no circulation, so their edges
// shed trailing vortices (gap loss) — a sealed gap (brush/elastomer skirt) is a segment of reduced chord σ·c. Stack, bottom up:
// ground gap (or underbody side-skirt seal) → body side panel (the monocoque as the lower part of the lifting surface, chord ≈
// body length, thick-section lift slope ηb·2π, camber from the side panels) → root junction (crossbeam gap / root fairing) →
// wing(s). Tandem wings sharing the same span stations are summed (Munk's stagger theorem: the induced drag of a multiplane
// is independent of stagger); the closed arch adds Prandtl's box-plane factor on the induced drag.
const llSys = new Map();
function luSolve(M, n) { const A = M.map(r => r.slice()), P = Array.from({ length: n }, (_, i) => i);
  for (let k = 0; k < n; k++) { let m = k; for (let i = k + 1; i < n; i++) if (Math.abs(A[i][k]) > Math.abs(A[m][k])) m = i; [A[k], A[m]] = [A[m], A[k]]; [P[k], P[m]] = [P[m], P[k]]; for (let i = k + 1; i < n; i++) { A[i][k] /= A[k][k]; for (let j = k + 1; j < n; j++) A[i][j] -= A[i][k] * A[k][j]; } }
  return (b) => { const y = P.map(i => b[i]); for (let i = 0; i < n; i++) for (let j = 0; j < i; j++) y[i] -= A[i][j] * y[j]; for (let i = n - 1; i >= 0; i--) { for (let j = i + 1; j < n; j++) y[i] -= A[i][j] * y[j]; y[i] /= A[i][i]; } return y; }; }
export function stackSystem(segs, { ground = true, nPer = 14 } = {}) {
  const key = JSON.stringify([segs.map(g => [g.z0, g.z1, g.c0, g.c1, g.a].map(v => +(+v).toFixed(4))), ground, nPer]); if (llSys.has(key)) return llSys.get(key);
  const pn = []; segs.forEach((g, si) => { if (g.z1 - g.z0 < 1e-4) return; const n = Math.max(2, Math.round(nPer * (g.n ?? 1))); for (let i = 0; i < n; i++) { const e0 = g.z0 + (g.z1 - g.z0) * (1 - Math.cos(Math.PI * i / n)) / 2, e1 = g.z0 + (g.z1 - g.z0) * (1 - Math.cos(Math.PI * (i + 1) / n)) / 2, zm = (e0 + e1) / 2, t = (zm - g.z0) / (g.z1 - g.z0); pn.push({ e0, e1, zm, dz: e1 - e0, c: g.c0 + (g.c1 - g.c0) * t, a: g.a, si }); } });
  const n = pn.length, A = pn.map(p => pn.map(q => { let w = 1 / (p.zm - q.e0) - 1 / (p.zm - q.e1); if (ground) w += 1 / (p.zm + q.e1) - 1 / (p.zm + q.e0); return w / (4 * Math.PI); }));
  const M = pn.map((p, i) => pn.map((q, j) => (i === j ? 1 : 0) + 0.5 * p.c * p.a * A[i][j])), solve = luSolve(M, n), sys = { pn, A, M, solve, n };
  if (llSys.size > 400) llSys.clear(); llSys.set(key, sys); return sys;
}
// solve for Γ/V given the geometric angle θ (rad, incl. zero-lift shift) and the section C_l cap of every segment
export function stackSolve(sys, theta, clmax) {
  const { pn, A, solve, n } = sys, rhs = pn.map((p, i) => 0.5 * p.c * p.a * theta[p.si]); let G = solve(rhs);
  const cap = pn.map(p => 0.5 * p.c * clmax[p.si]);
  // section stall: lower the effective angle of capped panels until their circulation sits at the cap (re-uses the LU)
  if (G.some((g, i) => Math.abs(g) > cap[i])) { const r = rhs.slice(); for (let it = 0; it < 12; it++) { let worst = 0; for (let i = 0; i < n; i++) { const ex = Math.abs(G[i]) - cap[i]; if (ex > 0) { r[i] -= Math.sign(G[i]) * ex * 1.1; worst = Math.max(worst, ex / Math.max(1e-9, cap[i])); } } if (worst < 0.01) break; G = solve(r); } G = G.map((g, i) => clamp(g, -cap[i], cap[i])); }
  const w = pn.map((_, i) => A[i].reduce((s2, a, j) => s2 + a * G[j], 0));
  return { G, w, aEff: pn.map((p, i) => theta[p.si] - w[i]) };
}
// vertical stack of a vehicle from its CAD geometry. opts: deckSeal σ (crossbeam brush skirt), groundSeal σ (underbody side
// skirts), bodyEta (thick-body lift-slope factor), bodyClmax, bodyFill (side-panel chord / body length)
export function vehicleStack(veh, { deckSeal = 0, groundSeal = 0, bodyEta = 0.6, bodyClmax = 0.6, bodyFill = 0.85, a2D = 2 * Math.PI, cosL = 1, rootBootH = 0, bootEta = 0.7, skirtGap = null, archFill = 'hollow', fillFrac = 0.3 } = {}) {
  const meta = vehicleAssembly(veh, {}).meta, BS = meta.bodyStation, AM = meta.arch, Lb = veh.length * bodyFill, segs = [];
  const zb = BS.zBot, zt = BS.zTop;
  // physical underbody skirt: a solid compliant skirt from the floor down to skirtGap above the road (only that open gap remains);
  // legacy: groundSeal σ as a fraction of the body length across the whole clearance
  if (skirtGap != null && zb > skirtGap + 0.002) segs.push({ kind: 'groundSkirt', z0: Math.max(0.002, skirtGap), z1: zb, c0: Lb, c1: Lb, a: bodyEta * a2D });
  else if (groundSeal > 0 && zb > 0.005) segs.push({ kind: 'groundSkirt', z0: 0.005, z1: zb, c0: groundSeal * Lb, c1: groundSeal * Lb, a: bodyEta * a2D });
  segs.push({ kind: 'body', z0: zb, z1: zt, c0: Lb, c1: Lb, a: bodyEta * a2D, n: 1 });
  let kBox = 1, zTop;
  if (AM) { const zR = AM.zRoot, hv = AM.legHeight * Math.cos(rad(AM.cant)); zTop = zR + hv;
    // physical root boot: a continuous elastomer membrane from the deck to the leg roots (no open gap; membrane lift slope η_m);
    // legacy: deckSeal σ as a fraction of the root chord across the crossbeam gap
    if (zR > zt + 1e-4) { if (rootBootH > 0) segs.push({ kind: 'deckSkirt', z0: zt, z1: zR, c0: 2 * AM.rootChord, c1: 2 * AM.rootChord, a: a2D * cosL * bootEta }); else segs.push({ kind: 'deckSkirt', z0: zt, z1: zR, c0: deckSeal * 2 * AM.rootChord, c1: deckSeal * 2 * AM.rootChord, a: a2D * cosL }); }
    // arch opening fill (front view): 'hollow' = two legs; 'solid' = a continuous centreline web/membrane from crossbeam to bridge
    // (a third vertical surface in parallel — Munk: summed chord at each height); 'hybrid' = a root gusset web over the lower
    // fillFrac of the height (chord tapering from the root chord to zero) plus corner fairings (no aerodynamic credit)
    const z0w = Math.max(zR, zt), aW = a2D * cosL * Math.cos(rad(AM.cant)), cAt = (z) => AM.rootChord + (AM.tipChord - AM.rootChord) * (z - z0w) / Math.max(1e-6, zTop - z0w);
    if (archFill === 'solid') segs.push({ kind: 'wing', z0: z0w, z1: zTop, c0: 3 * AM.rootChord, c1: 3 * AM.tipChord, a: aW, n: 1.6 });
    else if (archFill === 'hybrid') { const zg = z0w + fillFrac * (zTop - z0w); segs.push({ kind: 'wing', z0: z0w, z1: zg, c0: 3 * AM.rootChord, c1: 2 * cAt(zg), a: aW, n: 0.6 }, { kind: 'wing', z0: zg, z1: zTop, c0: 2 * cAt(zg), c1: 2 * AM.tipChord, a: aW, n: 1.2 }); }
    else segs.push({ kind: 'wing', z0: z0w, z1: zTop, c0: 2 * AM.rootChord, c1: 2 * AM.tipChord, a: aW, n: 1.6 });
    const gM = (AM.gap + AM.bridgeSpan) / 2, b = 2 * zTop; kBox = (1 + 0.45 * gM / b) / (1.04 + 2.81 * gM / b);
  } else { const nW = veh.nWings || 2, zR = zt - 0.02, sp = meta.span; zTop = zR + sp; segs[segs.length - 1].z1 = zR; segs.push({ kind: 'wing', z0: zR, z1: zTop, c0: nW * meta.cr, c1: nW * meta.ct, a: a2D * cosL, n: 1.6 }); }
  return { segs, kBox, zTop, Lb, meta };
}
// forces of the stacked system at apparent-wind angle bAbs (deg) with wing angle of attack alpha (deg); wing section from
// the polar, body at α = β with leeward side-panel camber; returns the archForces-compatible object (body lift is inside)
export function stackForces(ST, pol, veh, bAbs, alpha, VA, rho = 1.225, { camberPct = 4, interf = 1, bridgeCdA = 0 } = {}) {
  const c0 = coeffs(pol, 0), c4 = coeffs(pol, 4), a0w = (c4.CL - c0.CL) / rad(4), al0w = -c0.CL / a0w, b = rad(bAbs), q = 0.5 * rho * VA * VA, S = veh.wingArea;
  const kA = Math.round(a0w * 1e4), sc = ST._sys || (ST._sys = new Map()); let sys = sc.get(kA); if (!sys) { sys = stackSystem(ST.segs.map(g => g.kind === 'wing' || g.kind === 'deckSkirt' ? { ...g, a: g.a / (2 * Math.PI) * a0w } : g)); sc.set(kA, sys); } // memoised per section lift slope
  const mB = (veh.bodyCamberPct ?? camberPct) / 100 * (veh.bodyCamberWash === false ? 1 : interf), aB = rad(Math.min(bAbs, 180 - bAbs)) + (bAbs <= 90 ? 2 * mB : 0); // the bistable side panels de-camber with the arch skin
  const theta = ST.segs.map(g => g.kind === 'wing' || g.kind === 'deckSkirt' ? rad(alpha) - al0w * interf : aB);
  const clmax = ST.segs.map(g => g.kind === 'wing' || g.kind === 'deckSkirt' ? (pol.CLmax ?? 1.6) * interf : 0.6);
  const R = stackSolve(sys, theta, clmax); let Lw = 0, Lb = 0, Di = 0, Dp = 0;
  sys.pn.forEach((p, i) => { const g = ST.segs[p.si], Lp = 2 * q * R.G[i] * p.dz, Dip = 2 * q * R.G[i] * R.w[i] * p.dz; Di += Dip; if (g.kind === 'wing' || g.kind === 'deckSkirt') { Lw += Lp; Dp += q * p.c * p.dz * Math.max(0.006, coeffs(pol, deg(R.aEff[i])).CD); } else Lb += Lp; });
  Di *= ST.kBox; const L = Lw + Lb, D = Di + Dp, F = { L, D, thrust: L * Math.sin(b) - D * Math.cos(b), side: L * Math.abs(Math.cos(b)) + D * Math.sin(b) };
  const Db = 0.5 * rho * (veh.CdA + bridgeCdA) * VA * VA, sB = Db * Math.sin(b), zB = (veh.wheelRadius ?? 0.3) + (veh.height ?? 1) / 2, hCE = 0.5 * (ST.zTop + (ST.segs.find(g => g.kind === 'wing')?.z0 ?? 0));
  const CLw = Lw / (q * S), CDw = (Dp + Di * Lw / Math.max(1e-9, L)) / (q * S);
  // side force at the arch centre of effort: wing part at hCE, body part at the body centre (moment-equivalent)
  const sideW = F.side * Lw / Math.max(1e-9, L), sideBd = F.side - sideW;
  return { CL: CLw, CD: CDw, CLsys: L / (q * S), CDsys: D / (q * S), eps: 0, L, D, Lw, Lb, Di, Dp, wingThrust: F.thrust, wingSide: F.side, body: { thrust: 0, side: 0, L: Lb }, Db, net: F.thrust - Db * Math.cos(b), side: F.side + sB, sideEq: sideW + (sideBd + sB) * zB / hCE, stack: true };
}
// Geometry-based drag-area build-up, identical for every revision (computed from each design's own CAD geometry):
// body — Hoerner's streamlined-body formula on frontal area, C_D = C_f·(3 l/d + 4.5 (d/l)^½ + 21 (d/l)²) (Hoerner,
// Fluid-Dynamic Drag, 1965, ch. 6), turbulent C_f = 0.455/(log₁₀Re)^2.58, ground-proximity factor 1 + 0.4·e^(−(h/d)/0.15)
// (drag rise of streamlined bodies near the ground, cf. Hucho, Aerodynamics of Road Vehicles); wheels in spats / bare,
// exposed axle arms (C_d 1.0), spat–body junctions, wing-root junctions (Hoerner: ΔC_DA = t²·(0.8 (t/c)³ − 0.0003)),
// arch crossbeam, canopy/mirrors/lights/leakage, and +15 % mutual interference.
export function dragBuildup(veh, { V = 8, nu = 1.5e-5 } = {}) {
  const meta = vehicleAssembly(veh, {}).meta, BS = meta.bodyStation, AM = meta.arch, l = veh.length;
  const Af = 3.53 * BS.aMax * BS.bMax, d = Math.sqrt(4 * Af / Math.PI), Re = V * l / nu, Cf = 0.455 / Math.log10(Re) ** 2.58, CDb = Cf * (3 * l / d + 4.5 * Math.sqrt(d / l) + 21 * (d / l) ** 2), hd = Math.max(0, BS.zBotMin) / d, kg = 1 + 0.4 * Math.exp(-hd / 0.15);
  const nWh = veh.nWheels || 3, spats = veh.spats !== false, ax = meta.axle, arm = spats && ax ? ax.exposedArm : Math.max(0, (veh.track || 1.8) / 2 - (ax?.bodyHalfWidth ?? (veh.width || 1.6) / 2));
  const tc = 0.12, junc = (c) => { const t = tc * c; return Math.max(0, t * t * (0.8 * tc ** 3 - 0.0003)); };
  const rows = [[`Body (Hoerner, l/d ${(l / d).toFixed(2)}, A_f ${Af.toFixed(3)} m², C_f ${Cf.toFixed(4)}, ground ×${kg.toFixed(2)})`, CDb * Af * kg], [`Wheels ×${nWh} ${spats ? 'in spats' : 'bare'}`, (spats ? 0.012 : 0.03) * nWh], [`Exposed front axle arms (2 × ${(arm * 1000).toFixed(0)} mm × 30 mm, C_d 1.0)`, 2 * arm * 0.03],
    [spats ? 'Spat–body junctions (2)' : 'Wheel–body interference (open)', spats ? 0.008 : 0.024],
    ...(AM ? [['Arch root crossbeam fairing (C_d 0.1 on 0.1 m × span)', 0.1 * 0.1 * (AM.gap + 0.08)], ['Arch leg–crossbeam junctions (2, Hoerner)', 2 * junc(AM.rootChord)]] : [[`Wing–body root junctions (${veh.nWings || 2}, Hoerner)`, (veh.nWings || 2) * junc(meta.cr)]]),
    ['Canopy, mirrors, lights & leakage', 0.02]];
  const sub = rows.reduce((a, r) => a + r[1], 0), CdA = sub * 1.15;
  return { rows: [...rows, ['Mutual interference (+15 %)', sub * 0.15]], CdA, body: { Af, d, ld: l / d, Cf, CDb, kg } };
}
// lift-capped forces: if the side force exceeds the safe limit the bistable skin de-cambers and the flexure stiffens,
// shedding lift (modelled as the CL scale that brings the side force back to the limit)
export function cappedForces(G, pol, veh, bAbs, alpha, VA, rho, FyS, opts) {
  const F = archForces(G, pol, veh, bAbs, alpha, VA, rho, opts); if (F.sideEq <= FyS) return { ...F, capped: false };
  // de-cambering can only remove the camber lift: the scale cannot fall below the symmetric-section fraction (≈ minScale)
  const minS = opts?.minScale ?? (F.stack ? 0 : 0.6); let lo = minS, hi = 1; // stacked model: interf scales the camber lift only, so full de-cambering is interf = 0 if (archForces(G, pol, veh, bAbs, alpha, VA, rho, { ...opts, interf: minS }).sideEq > FyS) return { ...archForces(G, pol, veh, bAbs, alpha, VA, rho, { ...opts, interf: minS }), capped: true, liftScale: minS, overLimit: true };
  for (let i = 0; i < 22; i++) { const m = (lo + hi) / 2; if (archForces(G, pol, veh, bAbs, alpha, VA, rho, { ...opts, interf: m }).sideEq > FyS) hi = m; else lo = m; }
  return { ...archForces(G, pol, veh, bAbs, alpha, VA, rho, { ...opts, interf: lo }), capped: true, liftScale: lo };
}

// compliant-flexure self-trim: kθ·δ = q·S·c·(0.25 − x_axis)·C_L(β − δ), δ limited to ±δmax (flexure stop / stiffening)
// Self-trimming wingsail principle: axis ahead of the aerodynamic centre plus a reflexed (positive C_m,ac) bistable skin.
// Moment balance about the flexure axis: kθ·δ = q·S·c̄·[C_L(β−δ)·(0.25 − x_axis) − C_m0]. With a soft flexure the arch
// trims at the constant C_L,trim = C_m0/(0.25 − x_axis), independent of wind speed; the flexure only centres it at rest.
export function trimState(G, pol, veh, bAbs, VA, rho, { kTheta, xAxis, Cm0 = 0.06, deltaMax = 25, FyS, camberPct = 4, scanN = 100 }) {
  const q = 0.5 * rho * VA * VA, c = G.cmean;
  // aerodynamic moment about the axis (positive = nose-up, raising α): attached flow → C_L at the quarter chord + C_m0;
  // beyond stall the normal force C_N acts progressively further aft (towards mid-chord), weathervaning the arch
  const cmAxis = (a) => { const F = archForces(G, pol, veh, bAbs, a, VA, rho, { camberPct }), aa = Math.abs(a), t = clamp((aa - pol.alphaStall) / 25, 0, 1), xcp = 0.25 + 0.22 * t, CN = Math.hypot(F.CL, F.CD) * Math.sign(F.CL || 1), Cmc = Cm0 * (1 - t); return Cmc - CN * (xcp - xAxis); };
  const mom = (d) => kTheta * rad(d) + q * G.S * c * cmAxis(bAbs - d); // >0 → δ decreases? (δ+ rotates nose into wind, lowering α)
  // equilibrium: kθ·δ = q S c̄ C_m,axis(α)·(−1) … expressed as f(δ) = kθδ + qSc̄·C_m(α) = 0; stable when df/dδ > 0
  const f = (d) => kTheta * rad(d) + q * G.S * c * cmAxis(bAbs - d);
  let d = null; const N = scanN;
  for (let i = N; i > 0; i--) { const d1 = -deltaMax + 2 * deltaMax * i / N, d0 = -deltaMax + 2 * deltaMax * (i - 1) / N; if (f(d1) >= 0 && f(d0) < 0) { let lo = d0, hi = d1; for (let k = 0; k < 20; k++) { const m = (lo + hi) / 2; if (f(m) >= 0) hi = m; else lo = m; } d = (lo + hi) / 2; break; } }
  if (d === null) d = f(deltaMax) < 0 ? deltaMax : -deltaMax; // moment exceeds the flexure at the stop / below the other stop
  void mom;
  const F = cappedForces(G, pol, veh, bAbs, bAbs - d, VA, rho, FyS ?? Infinity, { camberPct });
  return { delta: d, alpha: bAbs - d, atStop: Math.abs(d) >= deltaMax - 1e-6, ...F };
}
// best α within the ±δmax envelope (for comparison: what an ideal adjustable root would do inside the same envelope)
export function bestInEnvelope(G, pol, veh, bAbs, VA, rho, { deltaMax = 25, FyS, camberPct = 4 } = {}) {
  let best = null; for (const d of linspace(-deltaMax, deltaMax, 41)) { const F = cappedForces(G, pol, veh, bAbs, bAbs - d, VA, rho, FyS ?? Infinity, { camberPct }); if (!best || (best.overLimit && !F.overLimit) || (!!F.overLimit === !!best.overLimit && F.net > best.net)) best = { ...F, delta: d, alpha: bAbs - d }; } return best;
}

// sustainable speed for true wind U at the best heading (true-wind angle 30…170°), flexure-trimmed arch
export function sustainableSpeed(G, pol, veh, U, flex, { rho = 1.225, headings = [35, 45, 60, 75, 90, 105, 120, 140, 160], vmax = 35, dv = 0.25, mode = 'trim', scanN = 100, reach = true, coarse = 0.5 } = {}) {
  // reach: accelerate from rest along the chosen heading while net thrust ≥ rolling resistance (no gaps);
  // reach = false: highest steady force-balance speed (as Suite 20, the vehicle may be brought up to speed by the hub)
  const FyS = flex.FyS ?? safeSide(veh, G.hCE), Fr = veh.mass * G0 * veh.Crr; let best = { v: 0, twa: 90 };
  // pol may be a function of the apparent-wind speed (Reynolds-resolved polars, as in Suite 07)
  const netAt = (v, twa) => { const aw = apparentWind(U, twa, 0, v), b = Math.abs(aw.beta), VA = Math.max(aw.Va, 1e-3), pV = typeof pol === 'function' ? pol(VA) : pol; const F = mode === 'ideal' ? bestInEnvelope(G, pV, veh, b, VA, rho, { deltaMax: flex.deltaMax ?? 25, FyS }) : trimState(G, pV, veh, b, VA, rho, { ...flex, FyS, scanN }); return F.overLimit ? -Infinity : F.net; }; // over the safe side force even when fully de-cambered: inadmissible
  for (const twa of headings) { if (!reach) { const st = Math.max(dv, coarse); for (let v = vmax; v > best.v; v -= st) if (netAt(v, twa) >= Fr) { let lo = v, hi = Math.min(vmax, v + st); if (hi > lo) for (let k = 0; k < 7 && hi - lo > dv / 4; k++) { const m = (lo + hi) / 2; if (netAt(m, twa) >= Fr) lo = m; else hi = m; } best = { v: lo, twa }; break; } continue; } /* steady: coarse 0.5 m/s scan from the top, then bisect the force-balance edge */ let v = 0.2; if (netAt(v, twa) < Fr) continue; while (v < vmax && netAt(v + dv, twa) >= Fr) v += dv; if (v > best.v) best = { v, twa }; }
  return best;
}

// Revision-3 tandem comparator (n wings in tandem, optimum free incidence, downwash between wings, same tyres/drag)
export function tandemSpeed(veh, pol, U, { rho = 1.225, nW = 3, span = 2.0, area = 4.34, endplate = 1.6, hCE, headings = [35, 45, 60, 75, 90, 105, 120, 140, 160] } = {}) {
  const Sw = area / nW, cm = Sw / span, ARe = span * span / Sw * endplate, gap = Math.max(0.36 * veh.length, 1.12 * cm * 1.1), Fr = veh.mass * G0 * veh.Crr;
  const hce = hCE ?? (veh.wheelRadius + veh.height * 0.7 + span * 0.45), FyS = safeSide(veh, hce);
  const C3 = (a) => { const c = coeffs(pol, a); if (Math.abs(a) > pol.alphaStall + 8) return c; const f = finiteWing(c.CL, c.CD, ARe, 0.85); return { CL: f.CL, CD: f.CD }; };
  const net = (b, VA) => { let best = -Infinity; for (const a of linspace(-4, 30, 35)) { const c1 = C3(a), e = deg(2 * c1.CL / (Math.PI * ARe)) * Math.abs(Math.cos(rad(b))) * Math.exp(-gap * Math.abs(Math.sin(rad(b))) / (0.5 * cm)), c2 = C3(a - e), CL = (c1.CL + (nW - 1) * c2.CL * Math.cos(rad(e))) / nW, CD = (c1.CD + (nW - 1) * (c2.CD + c2.CL * Math.sin(rad(e)))) / nW; const F = wingForces(VA, b, CL, CD, area, rho), Db = 0.5 * rho * veh.CdA * VA * VA, B = bodyLift(veh, b, VA, rho), side = F.side + B.side + Db * Math.sin(rad(b)); if (side > FyS) continue; best = Math.max(best, F.thrust + B.thrust - Db * Math.cos(rad(b))); } return best; };
  let best = { v: 0, twa: 90 };
  for (const twa of headings) { const ok = (v) => { const aw = apparentWind(U, twa, 0, v); return net(Math.abs(aw.beta), Math.max(aw.Va, 1e-3)) >= Fr; }; let v = 0.2; if (!ok(v)) continue; while (v < 35 && ok(v + 0.25)) v += 0.25; if (v > best.v) best = { v, twa }; }
  return best;
}

// stationary tip-over wind (beam, wings loaded at the flexure stop; lift capped only by stall — worst case without cap)
export function tipOverWind(G, pol, veh, { ssf, cap = true, deltaMax = 25 } = {}) {
  const s = ssf ?? stabilityFactor(veh), M = veh.mass * G0 * s * veh.cgHeight; // restoring moment = m g · d, d = SSF·h_cg
  for (let U = 2; U <= 40; U += 0.25) { const F = cap ? cappedForces(G, pol, veh, 90, 90 - deltaMax, U, 1.225, 0, {}) : archForces(G, pol, veh, 90, 90 - deltaMax, U, 1.225); const Mo = F.wingSide * G.hCE + F.body.side * (veh.wheelRadius + veh.height / 2); if (Mo >= M) return U; } // cap: flexure at its stop, skin fully de-cambered
  return 40;
}
export const wheelPower20 = (veh, rho = 1.225) => { const v = 20 / 3.6; return (veh.mass * G0 * veh.Crr + 0.5 * rho * veh.CdA * v * v) * v; };
