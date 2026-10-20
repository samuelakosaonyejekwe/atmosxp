// AeroArch aerodynamics shared by Suites 06, 07, 09 and 25 (Revisions 4–5): Prandtl box wing of the two canted legs
// mirrored in the body plane, lateral leg interference, lifting body, compliant-flexure self-trim within ±δmax,
// de-cambering lift cap (passive gust washout), best-heading sustainable speed, and the Revision-3 tandem comparator
// evaluated with identical tyre and drag assumptions.
import { coeffs, finiteWing, polar as makePolar } from './aero.js';
import { resampleAirfoil } from './mesh.js';
import { naca4, vehicleAssembly } from './cad.js';
import { apparentWind, wingForces } from './physics.js';
import { rad, deg, linspace, clamp, G0 } from './num.js';
import { safeSideForce } from './safeside.js';

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
export function safeSide(veh, hCE, { mu = 0.95, ssf } = {}) { const s = ssf ?? stabilityFactor(veh); return safeSideForce({ mass: veh.mass, cgHeight: veh.cgHeight, ssf: s, hCE, mu }).Fs; }
// static stability factor of a tadpole trike: lateral distance CG → tipping axis (front contact → rear contact) / CG height
export function stabilityFactor(veh) { const fF = veh.frontLoad ?? veh.frontFrac ?? 0.6, a = fF * veh.wheelbase; return (veh.track / 2) * (a / veh.wheelbase) / veh.cgHeight; }

// forces on the arch at apparent-wind angle bAbs and angle of attack alpha (deg); no cap
export function archForces(G, pol, veh, bAbs, alpha, VA, rho = 1.225, opts = {}) {
  if (opts.kv != null && opts.kv < 1) return ventScaled(archForces(G, pol, veh, bAbs, alpha, VA, rho, { ...opts, kv: null }), opts.kv, G);
  // stacked lifting line (ground image, body as the lower lifting surface) in attached flow; the earlier root-mirror model
  // is kept for large apparent-wind angles / post-stall, blended over β = 35…45°
  const useStack = (opts.stack ?? veh.stackLL ?? true) && Math.abs(alpha) <= (pol.alphaStall ?? 15) + 4 && bAbs <= 45;
  if (useStack) { const ST = G.ST || (G.ST = vehicleStack(veh, stackOpts(veh))), Fs = stackForces(ST, pol, veh, bAbs, alpha, VA, rho, { camberPct: opts.camberPct ?? 4, interf: opts.interf ?? 1, bridgeCdA: 0.012 * (G.AM.bridgeArea || 0) });
    if (bAbs <= 35) return Fs; const w = (45 - bAbs) / 10, Fo = archForcesRootMirror(G, pol, veh, bAbs, alpha, VA, rho, opts), mix = (k) => w * Fs[k] + (1 - w) * Fo[k];
    return { ...Fo, CL: mix('CL'), CD: mix('CD'), L: mix('L'), D: mix('D'), wingThrust: mix('wingThrust'), wingSide: mix('wingSide'), net: mix('net'), side: mix('side'), sideEq: mix('sideEq'), CLsys: w * Fs.CLsys + (1 - w) * Fo.CLsys, CDsys: w * Fs.CDsys + (1 - w) * Fo.CDsys, aSecMax: Math.max(Fs.aSecMax, Fo.aSecMax) }; }
  return archForcesRootMirror(G, pol, veh, bAbs, alpha, VA, rho, opts);
}
export const stackOpts = (veh) => ({ plateW: veh.rootPlateW ?? 0, plateL: veh.rootPlateL ?? 0, deckSeal: veh.deckSeal ?? 0, groundSeal: veh.groundSeal ?? 0, bodyEta: veh.bodyEta ?? 0.6, bodyClmax: 0.6, rootBootH: veh.rootBootH ?? 0, skirtGap: veh.skirtGap ?? null, archFill: veh.archFill || 'hollow', fillFrac: veh.fillFrac ?? 0.3 });
// opening-fill geometry: added area, its centroid height above the crossbeam and the torsion-constraint factor on the legs
export function archFillProps(veh, G) { const AM = G.AM, h = AM.legHeight * Math.cos(rad(AM.cant)), f = veh.fillFrac ?? 0.3, fill = veh.archFill || 'hollow';
  if (fill === 'solid') return { area: h * (AM.rootChord + AM.tipChord) / 2, zc: AM.zRoot + 0.45 * h, torsionFactor: 2.0, parts: 3 };
  if (fill === 'hybrid') return { area: 0.5 * f * h * AM.rootChord + 2 * 0.04, zc: AM.zRoot + f * h / 3, torsionFactor: 1 / Math.sqrt(Math.max(0.2, 1 - f)), parts: 3 };
  return { area: 0, zc: AM.zRoot, torsionFactor: 1, parts: 0 }; }
function archForcesRootMirror(G, pol, veh, bAbs, alpha, VA, rho = 1.225, { camberPct = 4, interf = 1, oswald = 0.85 } = {}) {
  const C3 = (a) => { const c = coeffs(pol, a); if (Math.abs(a) > pol.alphaStall + 8) return { ...c, CDi: 0 }; const f = finiteWing(c.CL * G.cosL, c.CD, G.ARe, oswald); return { CL: f.CL, CD: f.CD, CDi: f.CDi }; };
  const c1 = C3(alpha), eps = deg(c1.CL / (Math.PI * G.ARe)) * Math.abs(Math.sin(rad(bAbs))) * Math.exp(-G.gMean * Math.abs(Math.cos(rad(bAbs))) / (0.5 * G.cmean));
  const cw = C3(alpha + 0.3 * eps), cl = C3(alpha - eps), e = rad(eps), CL = (cw.CL + cl.CL * Math.cos(e)) / 2;
  /* induced drag (finite-wing term and the downwash tilt of the lee leg) belongs to the lift actually carried: when the skin
     de-cambers (lift × interf) it falls with interf², the profile drag stays */
  const CDi = (cw.CDi + cl.CDi + cl.CL * Math.sin(e)) / 2, CD = (cw.CD + cl.CD + cl.CL * Math.sin(e)) / 2 - CDi * (1 - interf * interf);
  const F = wingForces(VA, bAbs, CL * interf, CD, G.S, rho), Db = 0.5 * rho * (veh.CdA + 0.012 * (G.AM.bridgeArea || 0)) * VA * VA, B = bodyLift(veh, bAbs, VA, rho, { camberPct });
  // sideEq: side force at the arch centre of effort giving the same overturning moment (the body forces act lower)
  const sB = B.side + Db * Math.sin(rad(bAbs)), zB = (veh.wheelRadius ?? 0.3) + (veh.height ?? 1) / 2;
  const aI = deg(CL * interf / (Math.PI * G.ARe * oswald)), aSecMax = Math.max(alpha + 0.3 * eps, alpha - eps) - aI; /* effective section angle: windward/lee leg incidence minus the finite-wing induced angle */
  return { aSecMax, CL, CD, CLsys: CL * interf, CDsys: CD, eps, L: F.L, D: F.D, wingThrust: F.thrust, wingSide: F.side, body: B, Db, net: F.thrust + B.thrust - Db * Math.cos(rad(bAbs)), side: F.side + sB, sideEq: F.side + sB * zB / G.hCE };
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
export function stackSystem(segs, { ground = true, nPer = 14, plate = null } = {}) {
  const key = JSON.stringify([segs.map(g => [g.z0, g.z1, g.c0, g.c1, g.a].map(v => +(+v).toFixed(4))), ground, nPer, plate]); if (llSys.has(key)) return llSys.get(key);
  const pn = []; segs.forEach((g, si) => { if (g.z1 - g.z0 < 1e-4) return; const n = Math.max(2, Math.round(nPer * (g.n ?? 1))); for (let i = 0; i < n; i++) { const e0 = g.z0 + (g.z1 - g.z0) * (1 - Math.cos(Math.PI * i / n)) / 2, e1 = g.z0 + (g.z1 - g.z0) * (1 - Math.cos(Math.PI * (i + 1) / n)) / 2, zm = (e0 + e1) / 2, t = (zm - g.z0) / (g.z1 - g.z0); pn.push({ e0, e1, zm, dz: e1 - e0, c: g.c0 + (g.c1 - g.c0) * t, a: g.a, si }); } });
  // root endplate: a finite horizontal plate at z_p gives the wing panels a PARTIAL image of strength r about z_p (r = 1 would be an
  // infinite reflection plane); r from Hoerner's end-plate correlation ΔAR/AR ≈ 1.9 h/b (Fluid-Dynamic Lift, 1985, ch. 3)
  const isW = (q) => segs[q.si].kind === 'wing' || segs[q.si].kind === 'deckSkirt';
  const n = pn.length, A = pn.map(p => pn.map(q => { let w = 1 / (p.zm - q.e0) - 1 / (p.zm - q.e1); if (ground) w += 1 / (p.zm + q.e1) - 1 / (p.zm + q.e0); if (plate && plate.r > 0 && isW(p) && isW(q)) { const i0 = 2 * plate.z - q.e1, i1 = 2 * plate.z - q.e0; w += plate.r * (1 / (p.zm - i0) - 1 / (p.zm - i1)); } return w / (4 * Math.PI); }));
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
export function vehicleStack(veh, { deckSeal = 0, groundSeal = 0, bodyEta = 0.6, bodyClmax = 0.6, bodyFill = 0.85, a2D = 2 * Math.PI, cosL = 1, rootBootH = 0, bootEta = 0.7, skirtGap = null, archFill = 'hollow', fillFrac = 0.3, plateW = 0, plateL = 0, plateGap = 0.025 } = {}) {
  const meta = vehicleAssembly(veh, {}).meta, BS = meta.bodyStation, AM = meta.arch, Lb = veh.length * bodyFill, segs = [];
  const zb = BS.zBot, zt = BS.zTop;
  // physical underbody skirt: a solid compliant skirt from the floor down to skirtGap above the road (only that open gap remains);
  // legacy: groundSeal σ as a fraction of the body length across the whole clearance
  if (skirtGap != null && zb > skirtGap + 0.002) segs.push({ kind: 'groundSkirt', z0: Math.max(0.002, skirtGap), z1: zb, c0: Lb, c1: Lb, a: bodyEta * a2D });
  else if (groundSeal > 0 && zb > 0.005) segs.push({ kind: 'groundSkirt', z0: 0.005, z1: zb, c0: groundSeal * Lb, c1: groundSeal * Lb, a: bodyEta * a2D });
  segs.push({ kind: 'body', z0: zb, z1: zt, c0: Lb, c1: Lb, a: bodyEta * a2D, n: 1 });
  let kBox = 1, zTop, plate = null;
  if (AM) { const zR = AM.zRoot, hv = AM.legHeight * Math.cos(rad(AM.cant)); zTop = zR + hv;
    // physical root boot: a continuous elastomer membrane from the deck to the leg roots (no open gap; membrane lift slope η_m);
    // legacy: deckSeal σ as a fraction of the root chord across the crossbeam gap
    // with a root endplate the legs, crossbeam and plate form one solid junction down to the plate's underside; only the plate's
    // running gap to the deck (plateGap) stays open
    if (plateW > 0 && zR > zt + plateGap + 1e-4) segs.push({ kind: 'deckSkirt', z0: zt + plateGap, z1: zR, c0: 2 * AM.rootChord, c1: 2 * AM.rootChord, a: a2D * cosL });
    else if (zR > zt + 1e-4) { if (rootBootH > 0) segs.push({ kind: 'deckSkirt', z0: zt, z1: zR, c0: 2 * AM.rootChord, c1: 2 * AM.rootChord, a: a2D * cosL * bootEta }); else segs.push({ kind: 'deckSkirt', z0: zt, z1: zR, c0: deckSeal * 2 * AM.rootChord, c1: deckSeal * 2 * AM.rootChord, a: a2D * cosL }); }
    // arch opening fill (front view): 'hollow' = two legs; 'solid' = a continuous centreline web/membrane from crossbeam to bridge
    // (a third vertical surface in parallel — Munk: summed chord at each height); 'hybrid' = a root gusset web over the lower
    // fillFrac of the height (chord tapering from the root chord to zero) plus corner fairings (no aerodynamic credit)
    const z0w = Math.max(zR, zt), aW = a2D * cosL * Math.cos(rad(AM.cant)), cAt = (z) => AM.rootChord + (AM.tipChord - AM.rootChord) * (z - z0w) / Math.max(1e-6, zTop - z0w);
    if (archFill === 'solid') segs.push({ kind: 'wing', z0: z0w, z1: zTop, c0: 3 * AM.rootChord, c1: 3 * AM.tipChord, a: aW, n: 1.6 });
    else if (archFill === 'hybrid') { const zg = z0w + fillFrac * (zTop - z0w); segs.push({ kind: 'wing', z0: z0w, z1: zg, c0: 3 * AM.rootChord, c1: 2 * cAt(zg), a: aW, n: 0.6 }, { kind: 'wing', z0: zg, z1: zTop, c0: 2 * cAt(zg), c1: 2 * AM.tipChord, a: aW, n: 1.2 }); }
    else segs.push({ kind: 'wing', z0: z0w, z1: zTop, c0: 2 * AM.rootChord, c1: 2 * AM.tipChord, a: aW, n: 1.6 });
    const gM = (AM.gap + AM.bridgeSpan) / 2, b = 2 * zTop; kBox = (1 + 0.45 * gM / b) / (1.04 + 2.81 * gM / b);
    // root endplate (plate width W across the body, length L fore-aft ≥ root chord for full effect): h = W/2 per leg, b = 2·leg height
    if (plateW > 0) { const hv2 = AM.legHeight * Math.cos(rad(AM.cant)), lenF = clamp(plateL / Math.max(0.1, AM.rootChord), 0, 1); const gain = clamp(1.9 * (plateW / 2) / (2 * hv2) * lenF, 0, 0.89); plate = { z: zt + plateGap, gain, r: +plateImage(gain).toFixed(4) }; }
  } else { const nW = veh.nWings || 2, zR = zt - 0.02, sp = meta.span; zTop = zR + sp; segs[segs.length - 1].z1 = zR; segs.push({ kind: 'wing', z0: zR, z1: zTop, c0: nW * meta.cr, c1: nW * meta.ct, a: a2D * cosL, n: 1.6 }); }
  return { segs, kBox, zTop, Lb, meta, plate };
}
// image strength r that reproduces a target effective-aspect-ratio gain ΔAR/AR in this lifting line (calibrated on a rectangular
// wing of AR 4: r → gain), so the end-plate credit equals the published correlation rather than an arbitrary image factor
const PLATE_CAL = [[0, 0], [0.1, 0.0311], [0.2, 0.0662], [0.3, 0.1061], [0.4, 0.1524], [0.5, 0.2073], [0.6, 0.2741], [0.7, 0.3588], [0.8, 0.4717], [0.9, 0.6344], [0.95, 0.7481], [1, 0.8986]];
export function plateImage(gain) { for (let i = 1; i < PLATE_CAL.length; i++) if (gain <= PLATE_CAL[i][1]) { const [r0, g0] = PLATE_CAL[i - 1], [r1, g1] = PLATE_CAL[i]; return r0 + (r1 - r0) * (gain - g0) / (g1 - g0); } return 1; }
// forces of the stacked system at apparent-wind angle bAbs (deg) with wing angle of attack alpha (deg); wing section from
// the polar, body at α = β with leeward side-panel camber; returns the archForces-compatible object (body lift is inside)
export function stackForces(ST, pol, veh, bAbs, alpha, VA, rho = 1.225, { camberPct = 4, interf = 1, bridgeCdA = 0 } = {}) {
  const c0 = coeffs(pol, 0), c4 = coeffs(pol, 4), a0w = (c4.CL - c0.CL) / rad(4), al0w = -c0.CL / a0w, b = rad(bAbs), q = 0.5 * rho * VA * VA, S = veh.wingArea;
  const kA = Math.round(a0w * 1e4), sc = ST._sys || (ST._sys = new Map()); let sys = sc.get(kA); if (!sys) { sys = stackSystem(ST.segs.map(g => g.kind === 'wing' || g.kind === 'deckSkirt' ? { ...g, a: g.a / (2 * Math.PI) * a0w } : g), { plate: ST.plate }); sc.set(kA, sys); } // memoised per section lift slope
  const mB = (veh.bodyCamberPct ?? camberPct) / 100 * (veh.bodyCamberWash === false ? 1 : interf), aB = rad(Math.min(bAbs, 180 - bAbs)) + (bAbs <= 90 ? 2 * mB : 0); // the bistable side panels de-camber with the arch skin
  const theta = ST.segs.map(g => g.kind === 'wing' || g.kind === 'deckSkirt' ? rad(alpha) - al0w * interf : aB);
  /* de-cambering removes the camber lift only: the section C_l,max falls by the camber lift at α = 0, to the symmetric-section value (not to zero) */
  const clW = Math.max(0.3, (pol.CLmax ?? 1.6) - Math.max(0, c0.CL) * (1 - interf)), clmax = ST.segs.map(g => g.kind === 'wing' || g.kind === 'deckSkirt' ? clW : 0.6);
  const R = stackSolve(sys, theta, clmax); let Lw = 0, Lb = 0, Di = 0, Dp = 0;
  sys.pn.forEach((p, i) => { const g = ST.segs[p.si], Lp = 2 * q * R.G[i] * p.dz, Dip = 2 * q * R.G[i] * R.w[i] * p.dz; Di += Dip; if (g.kind === 'wing' || g.kind === 'deckSkirt') { Lw += Lp; Dp += q * p.c * p.dz * Math.max(0.006, coeffs(pol, deg(R.aEff[i])).CD); } else Lb += Lp; });
  let aSecMax = -Infinity; sys.pn.forEach((p, i) => { const g = ST.segs[p.si]; if (g.kind === 'wing') aSecMax = Math.max(aSecMax, deg(R.aEff[i] + al0w * interf)); }); /* effective local section angle (after downwash incl. ground/plate images), geometric-section convention */
  Di *= ST.kBox; const L = Lw + Lb, D = Di + Dp, F = { L, D, thrust: L * Math.sin(b) - D * Math.cos(b), side: L * Math.abs(Math.cos(b)) + D * Math.sin(b) };
  const Db = 0.5 * rho * (veh.CdA + bridgeCdA) * VA * VA, sB = Db * Math.sin(b), zB = (veh.wheelRadius ?? 0.3) + (veh.height ?? 1) / 2, hCE = 0.5 * (ST.zTop + (ST.segs.find(g => g.kind === 'wing')?.z0 ?? 0));
  const CLw = Lw / (q * S), CDw = (Dp + Di * Lw / Math.max(1e-9, L)) / (q * S);
  // side force at the arch centre of effort: wing part at hCE, body part at the body centre (moment-equivalent)
  const sideW = F.side * Lw / Math.max(1e-9, L), sideBd = F.side - sideW;
  return { CL: CLw, CD: CDw, CLsys: L / (q * S), CDsys: D / (q * S), eps: 0, L, D, Lw, Lb, Di, Dp, wingThrust: F.thrust, wingSide: F.side, body: { thrust: 0, side: 0, L: Lb }, Db, net: F.thrust - Db * Math.cos(b), side: F.side + sB, sideEq: sideW + (sideBd + sB) * zB / hCE, stack: true, aSecMax };
}
// drag-area build-up lives in ./dragbuild.js (one function shared by Suites 01 and 07)
export { dragBuildup } from './dragbuild.js';
// lift-capped forces: if the side force exceeds the safe limit the bistable skin de-cambers and the flexure stiffens,
// shedding lift (modelled as the CL scale that brings the side force back to the limit)
// PASSIVE PRESSURE-RELIEF VENTS — one monolithic KIRIGAMI RELIEF PANEL per leg and one in the bridge: a laser-cut slit pattern
// in the glass-reinforced TPU skin between the leading-edge fairing and the spar box (2–12 % chord) and aft of the box (30–97 %).
// The flaps are bistable (curved ligaments): they pop out of plane together when the mean pressure difference across the arch
// reaches Δp_open and fold back elastically below Δp_close (buckling-induced kirigami, Rafsanjani & Bertoldi, Phys. Rev. Lett.
// 118, 084301, 2017; bistable cut patterns, Rafsanjani & Pasini, Extreme Mech. Lett. 9, 2016). The normal force of the vented
// zone falls with the open-area ratio φ as for porous plates normal to a stream (normalised trend of Castro, J. Fluid Mech. 46,
// 1971); the cut kerf is sealed shut by a co-moulded TPU lip on the lee face that the flap lifts away when it opens (φ_closed = 0). To be confirmed on a panel in the Suite 24 test plan.
// k_v = 1 − zone·(1 − r(φ)).
export const VENT_DEFAULT = { zone: 0.76, phiMax: 0.85, phiClosed: 0, dpOpenK: 0.9, dpCloseK: 0.5 };
const POR = [[0, 1], [0.1, 0.85], [0.2, 0.70], [0.3, 0.56], [0.4, 0.42], [0.5, 0.30], [0.6, 0.20], [0.7, 0.12], [0.8, 0.06], [1, 0]];
export function porousRatio(phi) { for (let i = 1; i < POR.length; i++) if (phi <= POR[i][0]) { const [x0, y0] = POR[i - 1], [x1, y1] = POR[i]; return y0 + (y1 - y0) * (phi - x0) / (x1 - x0); } return 0; }
export const ventK = (V, phi) => 1 - V.zone * (1 - porousRatio(phi));
// vent opening for a solid-arch state F (wing normal force) at dynamic pressure q: fixed point of φ(Δp(k_v(φ))); thresholds are
// set from the safe side force so the vents stay shut while the arch loading is below the cap (no thrust lost in normal driving)
export function ventState(V, F, q, S, FyS, { forceOpen = false } = {}) {
  /* bistable kirigami flaps pop fully open (φ_max) at Δp_open and fold back below Δp_close (hysteresis); the driver lever
     (depower / park positions) tensions each panel spanwise through the shared cable, which pops the flaps open at any load.
     Δp_open is set from the safe side force so the panels stay shut while the arch loading is below the cap. */
  if (!V) return { phi: 0, kv: 1, dp: 0 };
  const dpO = V.dpOpen ?? V.dpOpenK * FyS / S, dpC = (V.dpCloseK ?? 0.5) * dpO, wL = F.Lw ?? F.L, wD = F.Lw != null ? (F.Dp ?? 0) + (F.Di ?? 0) * F.Lw / Math.max(1e-9, F.L) : F.D, CN = Math.hypot(wL, wD) / Math.max(1e-9, q * S), dp = CN * q; /* arch panels only (the stacked model's L also holds the body lift) */
  const open = forceOpen || dp >= dpO, phi = open ? V.phiMax : (V.phiClosed ?? 0);
  return { phi, kv: ventK(V, phi), dp: dp * ventK(V, phi), dpSolid: dp, dpOpen: dpO, dpClose: dpC, lever: forceOpen, open };
}
// scale the arch (wing) share of a force state by k_v; body forces unchanged
function ventScaled(F, kv, G) {
  const wS = F.Lw != null && F.L > 0 ? F.Lw / F.L : 1, k = 1 - (1 - kv) * wS, sideW = F.wingSide ?? F.side;
  const dSide = sideW * (1 - k), dThr = (F.wingThrust ?? 0) * (1 - k);
  return { ...F, CL: F.CL * kv, CD: F.CD * kv, L: F.L * k, D: F.D * k, Lw: F.Lw != null ? F.Lw * kv : undefined, wingThrust: (F.wingThrust ?? 0) - dThr, wingSide: sideW - dSide, net: F.net - dThr, side: F.side - dSide, sideEq: F.sideEq - dSide, kv, wingL: F.L * wS, wingD: F.D * wS };
}
export function cappedForces(G, pol, veh, bAbs, alpha, VA, rho, FyS, opts) {
  { const V = veh.vents === true ? VENT_DEFAULT : veh.vents; if (V && !opts?.kvDone) { const F0 = archForces(G, pol, veh, bAbs, alpha, VA, rho, { ...opts, interf: opts?.interf ?? 1 }), lev = !!opts?.lever && Math.abs(alpha) <= 90, /* directional flaps (ligaments on the upstream edge): the lever only biases them past the barrier; in reversed flow over the arch (|α| > 90°, wind abaft the arch) the stream folds them shut, so depower acts only where it cuts thrust */ vs = ventState(V, F0, 0.5 * rho * VA * VA, G.S, FyS === Infinity ? safeSide(veh, sideCE(veh) ?? G.hCE) : FyS, { forceOpen: lev });
    if (lev) { const R = cappedForces(G, pol, veh, bAbs, alpha, VA, rho, FyS, { ...opts, kv: vs.kv, kvDone: true }); return { ...R, vent: vs, lever: true, capped: true }; } /* lever: panels held open (all arch forces × k_v ≤ 1, so thrust and drag push can only fall); skin washout stays passive */
    if (vs.kv < 1) { const R = cappedForces(G, pol, veh, bAbs, alpha, VA, rho, FyS, { ...opts, kv: vs.kv, kvDone: true }); return { ...R, vent: vs, capped: true }; } } }
  const F = archForces(G, pol, veh, bAbs, alpha, VA, rho, opts); if (F.sideEq <= FyS) return { ...F, capped: false };
  // de-cambering can only remove the camber lift: the scale cannot fall below the symmetric-section fraction (≈ minScale)
  const minS = Math.min(opts?.minScale ?? (F.stack ? 0 : 0.6), opts?.interf ?? 1); let lo = minS, hi = opts?.interf ?? 1; /* stacked model: interf scales the camber lift only, so full de-cambering is interf = 0 */ if (archForces(G, pol, veh, bAbs, alpha, VA, rho, { ...opts, interf: minS }).sideEq > FyS) return { ...archForces(G, pol, veh, bAbs, alpha, VA, rho, { ...opts, interf: minS }), capped: true, liftScale: minS, overLimit: true };
  for (let i = 0; i < 22; i++) { const m = (lo + hi) / 2; if (archForces(G, pol, veh, bAbs, alpha, VA, rho, { ...opts, interf: m }).sideEq > FyS) hi = m; else lo = m; }
  return { ...archForces(G, pol, veh, bAbs, alpha, VA, rho, { ...opts, interf: lo }), capped: true, liftScale: lo };
}

// compliant-flexure self-trim: kθ·δ = q·S·c·(0.25 − x_axis)·C_L(β − δ), δ limited to ±δmax (flexure stop / stiffening)
// Self-trimming wingsail principle: axis ahead of the aerodynamic centre plus a reflexed (positive C_m,ac) bistable skin.
// Moment balance about the flexure axis: kθ·δ = q·S·c̄·[C_L(β−δ)·(0.25 − x_axis) − C_m0]. With a soft flexure the arch
// trims at the constant C_L,trim = C_m0/(0.25 − x_axis), independent of wind speed; the flexure only centres it at rest.
export function trimState(G, pol, veh, bAbs, VA, rho, { kTheta, xAxis, Cm0 = 0.06, deltaMax = 25, FyS, camberPct = 4, scanN = 100, lever = false, camber = 1 }) {
  /* camber: progressive skin-camber setting from the driver lever travel (1 = full reflexed camber, 0 = flat): scales the camber
     zero-lift shift and the camber pitching moment, so it also moves the self-trim incidence */
  if (camber !== 1 && !arguments[6]._camberDone) return trimState(G, pol, veh, bAbs, VA, rho, { ...arguments[6], Cm0: Cm0 * camber, camber: 1, _camberDone: true, _interf: camber });
  if (lever) { /* depower: in forward flow over the arch the tensioned panels stay open and the cam-free tendon also holds the reflexed skin
     flat (no camber lift or camber moment); in reversed flow (|α| > 90°) the stream folds the directional flaps shut and the skin
     is left to the passive washout — so depowering can only cut thrust and drag push */
    const T1 = trimState(G, pol, veh, bAbs, VA, rho, { kTheta, xAxis, Cm0: 0, deltaMax, FyS, camberPct: 0, scanN, lever: false, _lev: true });
    if (Math.abs(T1.alpha) <= 90) return T1; return trimState(G, pol, veh, bAbs, VA, rho, { kTheta, xAxis, Cm0, deltaMax, FyS, camberPct, scanN, lever: false }); }
  const q = 0.5 * rho * VA * VA, c = G.cmean;
  // aerodynamic moment about the axis (positive = nose-up, raising α): attached flow → C_L at the quarter chord + C_m0;
  // beyond stall the normal force C_N acts progressively further aft (towards mid-chord), weathervaning the arch
  const intf0 = arguments[6]._interf ?? 1, cmAxis = (a) => { const F = archForces(G, pol, veh, bAbs, a, VA, rho, { camberPct, interf: intf0 }), aa = Math.abs(a), t = clamp((aa - pol.alphaStall) / 25, 0, 1), xcp = 0.25 + 0.22 * t, CN = Math.hypot(F.CL, F.CD) * Math.sign(F.CL || 1), Cmc = Cm0 * (1 - t); return Cmc - CN * (xcp - xAxis); };
  const mom = (d) => kTheta * rad(d) + q * G.S * c * cmAxis(bAbs - d); // >0 → δ decreases? (δ+ rotates nose into wind, lowering α)
  // equilibrium: kθ·δ = q S c̄ C_m,axis(α)·(−1) … expressed as f(δ) = kθδ + qSc̄·C_m(α) = 0; stable when df/dδ > 0
  const f = (d) => kTheta * rad(d) + q * G.S * c * cmAxis(bAbs - d);
  let d = null; const N = scanN;
  for (let i = N; i > 0; i--) { const d1 = -deltaMax + 2 * deltaMax * i / N, d0 = -deltaMax + 2 * deltaMax * (i - 1) / N; if (f(d1) >= 0 && f(d0) < 0) { let lo = d0, hi = d1; for (let k = 0; k < 20; k++) { const m = (lo + hi) / 2; if (f(m) >= 0) hi = m; else lo = m; } d = (lo + hi) / 2; break; } }
  if (d === null) d = f(deltaMax) < 0 ? deltaMax : -deltaMax; // moment exceeds the flexure at the stop / below the other stop
  void mom;
  const F = cappedForces(G, pol, veh, bAbs, bAbs - d, VA, rho, FyS ?? Infinity, { camberPct, lever: arguments[6]._lev === true, interf: arguments[6]._lev ? 0 : (arguments[6]._interf ?? 1) });
  return { delta: d, alpha: bAbs - d, atStop: Math.abs(d) >= deltaMax - 1e-6, ...F };
}
// best α within the ±δmax envelope (for comparison: what an ideal adjustable root would do inside the same envelope)
export function bestInEnvelope(G, pol, veh, bAbs, VA, rho, { deltaMax = 25, FyS, camberPct = 4 } = {}) {
  let best = null; for (const d of linspace(-deltaMax, deltaMax, 41)) { const F = cappedForces(G, pol, veh, bAbs, bAbs - d, VA, rho, FyS ?? Infinity, { camberPct }); if (!best || (best.overLimit && !F.overLimit) || (!!F.overLimit === !!best.overLimit && F.net > best.net)) best = { ...F, delta: d, alpha: bAbs - d }; } return best;
}

// sustainable speed for true wind U at the best heading (true-wind angle 30…170°), flexure-trimmed arch
export function sustainableSpeed(G, pol, veh, U, flex, { rho = 1.225, headings = [35, 45, 60, 75, 90, 105, 120, 140, 160], vmax = 35, dv = 0.25, mode = 'trim', scanN = 100, reach = true, coarse = 0.5, gust = null, lever = 'off', progressive = false } = {}) {
  // reach: accelerate from rest along the chosen heading while net thrust ≥ rolling resistance (no gaps);
  // reach = false: highest steady force-balance speed (as Suite 20, the vehicle may be brought up to speed by the hub)
  const FyS = flex.FyS ?? safeSide(veh, sideCE(veh) ?? G.hCE), Fr = veh.mass * G0 * veh.Crr; let best = { v: 0, twa: 90 };
  // pol may be a function of the apparent-wind speed (Reynolds-resolved polars, as in Suite 07)
  /* progressive unloading (driver lever travel / storm detent sets the skin camber continuously): the best admissible camber
     setting — the highest thrust with the side force within the cap */
  const CAMB = progressive ? [1, 0.75, 0.5, 0.25, 0] : [1];
  const progBest = (pV, b, VA) => { let best = null; for (const cb of CAMB) { const T = trimState(G, pV, veh, b, VA, rho, { ...flex, FyS, scanN, camber: cb }); if (!best || (best.overLimit && !T.overLimit) || (!T.overLimit === !best.overLimit && T.net > best.net)) best = { ...T, camberSetting: cb }; } return best; };
  const netAt = (v, twa) => { const aw = apparentWind(U, twa, 0, v), b = Math.abs(aw.beta), VA = Math.max(aw.Va, 1e-3), pV = typeof pol === 'function' ? pol(VA) : pol; const F = mode === 'ideal' ? bestInEnvelope(G, pV, veh, b, VA, rho, { deltaMax: flex.deltaMax ?? 35, FyS }) : progBest(pV, b, VA); if (mode === 'ideal') return F.overLimit ? -Infinity : F.net;
    /* gust admissibility (same heading and speed, true wind × gust) and the driver lever in its depower position (kirigami panels held open) */
    const okG = (lev) => { if (!gust) return true; const ag = apparentWind(U * gust, twa, 0, v), bg = Math.abs(ag.beta), Vg = Math.max(ag.Va, 1e-3); return !trimState(G, typeof pol === 'function' ? pol(Vg) : pol, veh, bg, Vg, rho, { ...flex, FyS, scanN, lever: lev }).overLimit; };
    const nN = !F.overLimit && okG(false) ? F.net : -Infinity; if (lever === 'off' || !veh.vents) return nN;
    const FL = trimState(G, pV, veh, b, VA, rho, { ...flex, FyS, scanN, lever: true }), nL = !FL.overLimit && okG(true) ? FL.net : -Infinity; return lever === 'on' ? nL : Math.max(nN, nL); }; // over the safe side force even when fully de-cambered: inadmissible
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
// arch centre of side force: height of the lift centroid of the arch panels (legs + root junction) from the stacked lifting-line
// spanwise loading with the ground image and the root-endplate partial reflection, at a beam-wind cruise state (α 8°). This is
// THE h_CE of the safe side-force rule (js/core/safeside.js); Suite 01 publishes it and every suite reads it from there.
export function sideCE(veh, { alpha = 8 } = {}) {
  if ((veh.wingConfig || 'arch') !== 'arch') return null;
  const ST = vehicleStack(veh, stackOpts(veh)), isW = (g) => g.kind === 'wing' || g.kind === 'deckSkirt', a0 = 2 * Math.PI;
  const sys = stackSystem(ST.segs, { plate: ST.plate }), th = ST.segs.map(g => isW(g) ? rad(alpha) : 0), R = stackSolve(sys, th, ST.segs.map(g => isW(g) ? 1.6 : 0.6));
  let m = 0, f = 0; sys.pn.forEach((p, i) => { if (!isW(ST.segs[p.si])) return; const l = R.G[i] * p.dz; m += l * p.zm; f += l; }); void a0;
  return f > 0 ? m / f : null;
}
// h_CE over the whole operating envelope (safety cap must be conservative): apparent-wind angle 20–160° (sets the body side-panel
// angle that loads the stack from below), arch incidence −5…40° (self-trim, washout floor, gust +12°, yaw to the ±δ_max flexure limit (vehicle.yawLimit, ±35° by default) all land inside it),
// skin de-camber states (zero-lift shift and section C_l,max scaled 1 → 0.3, which moves load between tip and root through
// section stall), with and without the root-plate reflection credit, at apparent wind 3–35 m/s. The washout holds the arch side
// force at or below the cap F, so the heeling moment of a state is min(|F_state|, F)·h_state; the design h_CE is the largest
// such moment ÷ F (lightly loaded states with a high centre count only by the moment they actually make), + uncertainty.
const ceCache = new Map();
function ceStates(veh) { /* Γ/V moments of every envelope state (independent of the cap and of the wind speed) — memoised per geometry */
  const isW = (g) => g.kind === 'wing' || g.kind === 'deckSkirt', out = [];
  const stacks = [true, false].map(plateOn => { const v = plateOn ? veh : { ...veh, rootPlateW: 0, rootPlateL: 0 }, ST = vehicleStack(v, stackOpts(v)); return { plateOn, ST }; });
  const key = JSON.stringify(stacks.map(({ ST }) => [ST.segs.map(g => [g.kind, g.z0, g.z1, g.c0, g.c1, g.a].map(x => typeof x === 'number' ? +x.toFixed(4) : x)), ST.plate])); if (ceCache.has(key)) return ceCache.get(key);
  for (const { plateOn, ST } of stacks) { const sys = stackSystem(ST.segs, { plate: ST.plate });
    for (const beta of [20, 40, 60, 90, 120, 140, 160]) for (const al of [-5, 0, 4, 8, 12, 16, 20, 25, 30, 40]) for (const wash of [1, 0.7, 0.5, 0.3]) {
      const aB = rad(Math.min(beta, 180 - beta)) + (beta <= 90 ? 2 * 0.04 * wash : 0), al0 = rad(-4) * wash;
      const R = stackSolve(sys, ST.segs.map(g => isW(g) ? rad(al) - al0 : aB), ST.segs.map(g => isW(g) ? 1.6 * wash : 0.6));
      let m = 0, f = 0; sys.pn.forEach((p, i) => { if (!isW(ST.segs[p.si])) return; const l = R.G[i] * p.dz; m += l * p.zm; f += l; });
      out.push({ f, m, beta, alpha: al, wash, plate: plateOn });
    } }
  if (ceCache.size > 50) ceCache.clear(); ceCache.set(key, out); return out;
}
export function sideCEEnvelope(veh, { Fcap, uncertainty = 0.05, Vmin = 3, Vmax = 35, rho = 1.225 } = {}) {
  if ((veh.wingConfig || 'arch') !== 'arch') return null;
  const cases = [];
  for (const st of ceStates(veh)) for (const Va of [Vmin, 8, 15, 25, Vmax]) { const q2 = rho * Va * Va, F = Math.abs(q2 * st.f), M = Math.abs(q2 * st.m); /* lift = 2qΣ(Γ/V)dz */
    if (F > 1e-6) cases.push({ hEff: Math.min(F, Fcap) / F * M / Fcap, h: M / F, F, Va, beta: st.beta, alpha: st.alpha, wash: st.wash, plate: st.plate }); }
  const worst = cases.reduce((a, c) => c.hEff > a.hEff ? c : a), loaded = cases.filter(c => c.F >= Fcap).map(c => c.h);
  return { nominal: sideCE(veh), max: worst.hEff, atCapMin: Math.min(...loaded), atCapMax: Math.max(...loaded), worst, n: cases.length, uncertainty, design: worst.hEff * (1 + uncertainty) };
}
// conservative design h_CE and the cap it gives: fixed point of h_CE(F_cap) and F_cap(h_CE) (capOf(h) = the safe-side-force rule)
export function designCE(veh, capOf, opts = {}) {
  const h0 = sideCE(veh); if (h0 == null) return null; let h = h0 * 1.05, E = null;
  for (let it = 0; it < 12; it++) { E = sideCEEnvelope(veh, { ...opts, Fcap: capOf(h) }); if (Math.abs(E.design - h) < 1e-4) break; h = Math.max(h, E.design); }
  return { ...E, design: h, Fcap: capOf(h) };
}

// highest SAFE driving speed on a heading in true wind U with gusts × gust (side force within the cap in the mean and the gust
// state, best unloading mode), independent of whether the wind alone sustains it (hub- or store-assisted driving)
export function safeDrivingSpeed(G, pol, veh, U, flex, twa, { rho = 1.225, gust = 1.4, vmax = 35, dv = 0.5, scanN = 40 } = {}) {
  const FyS = flex.FyS ?? safeSide(veh, sideCE(veh) ?? G.hCE), modes = veh.vents ? [false, true] : [false];
  const ok = (v) => modes.some(lev => [1, gust].every(g => { const a = apparentWind(U * g, twa, 0, v), Va = Math.max(a.Va, 1e-3); return !trimState(G, typeof pol === 'function' ? pol(Va) : pol, veh, Math.abs(a.beta), Va, rho, { ...flex, FyS, scanN, lever: lev }).overLimit; }));
  if (!ok(0)) return { v: 0, standstill: false }; let v = 0; while (v < vmax && ok(v + dv)) v += dv; return { v, standstill: true };
}
// parked tip-over wind with the lever in PARK: the cable holds the kirigami relief panels fully open and the skin flat, and the
// yaw limiter either holds the arch aligned with the car (α = apparent-wind angle, limiter 'locked') or leaves it free on the
// ±δ flexure ('free'); worst over wind angles 0–180°. Overturning moment = arch side force at the centre of effort + the body
// side force × its height; pass body(U, β) → { Fy, z } to use the SAME body model as every other design (Suite 01), otherwise
// the lifting-line body of this model is used. Restoring moment M_res = m·g·SSF·h_cg. lever: false = parked in drive (solid).
export function parkedTipWind(veh, Mres, { lever = false, limiter = 'locked', body = null, flex = { kTheta: 10, xAxis: 0.10, Cm0: 0.09, deltaMax: veh.yawLimit ?? 35 }, pol = sectionPolar('7418', 5e5), rho = 1.225 } = {}) {
  const G = archGeometry(veh), V = veh.vents === true ? VENT_DEFAULT : veh.vents, kv = V ? ventK(V, lever ? V.phiMax : (V.phiClosed ?? 0)) : 1, rows = [];
  const hW = sideCE(veh) ?? G.hCE, zB = (veh.wheelRadius ?? 0.3) + (veh.height ?? 1) / 2;
  const momentAt = (b, U) => { const bb = Math.max(b, 0.1); let F;
    if (limiter === 'free') { const T = trimState(G, pol, veh, Math.min(bb, 179.9), U, rho, { ...flex, FyS: Infinity, scanN: 40, camber: lever ? 0 : 1 }); F = archForces(G, pol, veh, Math.min(bb, 179.9), T.alpha, U, rho, { kv, interf: lever ? 0 : 1 }); }
    else F = archForces(G, pol, veh, bb, (b <= 90 ? b : b - 180) || 0.1, U, rho, { kv, interf: lever ? 0 : 1 });
    if (!body) return Math.abs(F.sideEq) * G.hCE;
    const wS = F.Lw != null && F.L > 0 ? Math.abs(F.side) * F.Lw / F.L : Math.abs(F.wingSide), B = body(U, b); return wS * hW + Math.abs(B.Fy) * B.z; };
  for (const b of [0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180]) { let U = 4; for (; U < 60; U += 0.25) if (momentAt(b, U) > Mres) break; rows.push([b, U]); }
  const w = rows.reduce((a, r) => r[1] < a[1] ? r : a); return { U: w[1], angle: w[0], rows, limiter, lever };
}
