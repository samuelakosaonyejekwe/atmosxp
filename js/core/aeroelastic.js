// Aeroelastic library: nonlinear lifting line (discrete horseshoe vortices, root reflection plane, sheared inflow),
// partitioned static aeroelastic coupling (fixed-point / Aitken / IQN-ILS), Theodorsen V–g flutter,
// divergence, and transient gust response (Newmark-β modal structure + Wagner/Küssner indicial aerodynamics).
import { linspace, range, luSolve, eigGeneral, dot, norm, invert, matvec } from './num.js';
import { beamStatic, beamModes } from './fem.js';

// ---------- section aerodynamics ----------
// Builds a section lift curve with stall from slope a0 (1/rad), zero-lift angle alpha0 (rad), clmax.
export function sectionModel({ a0 = 2 * Math.PI * 0.95, alpha0 = 0, clmax = 1.15, cd0 = 0.011, k2 = 0.008, polar = null } = {}) {
  if (polar && polar.alpha?.length > 4) {
    const al = polar.alpha.map(a => a * Math.PI / 180), CL = polar.CL, CD = polar.CD;
    const interp = (x, xs, ys) => { if (x <= xs[0]) return ys[0]; if (x >= xs[xs.length - 1]) return ys[ys.length - 1]; let i = 1; while (xs[i] < x) i++; const t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]); return ys[i - 1] + t * (ys[i] - ys[i - 1]); };
    const sym = al[0] >= -1e-9; // polar given for positive α only → symmetric section assumed
    const cl = (a) => sym ? Math.sign(a) * interp(Math.abs(a), al, CL) : interp(a, al, CL);
    const cd = (c, a) => sym ? interp(Math.abs(a), al, CD) : interp(a, al, CD);
    const i0 = al.findIndex(a => a >= 0), i1 = Math.min(al.length - 1, i0 + 3), slope = (CL[i1] - CL[i0]) / ((al[i1] - al[i0]) || 1);
    return { cl, cd, a0: slope || a0, alpha0, clmax: Math.max(...CL.map(Math.abs)), source: 'polar' };
  }
  const as = clmax / a0 + alpha0;
  const cl = (a) => { const d = a - alpha0, sgn = Math.sign(d), ad = Math.abs(d), lin = a0 * ad; if (ad <= as - alpha0) return sgn * lin; const post = clmax * (0.6 + 0.4 * Math.exp(-(ad - (as - alpha0)) / 0.08)); return sgn * Math.max(post, 0.9 * Math.sin(2 * ad) * 0.95); };
  const cd = (c, a) => cd0 + k2 * c * c + (Math.abs(a - alpha0) > as - alpha0 ? 1.2 * Math.sin(Math.abs(a - alpha0)) ** 2 : 0);
  return { cl, cd, a0, alpha0, clmax, source: 'thin-aerofoil + stall model' };
}

// ---------- nonlinear lifting line ----------
// geometry: span s, chord(z), alpha(z) [rad] geometric angle of attack incl. twist; V(z) inflow speed
export function liftingLine({ span, chord, alpha, V, rho = 1.225, section, n = 24, mirror = 1, tol = 1e-8, maxIt = 200, alpha0Fn }) {
  const th = linspace(0, Math.PI / 2, n + 1), zE = th.map(t => span * Math.sin(t)); // cosine-type clustering toward the tip
  const zc = range(n).map(k => 0.5 * (zE[k] + zE[k + 1])), dz = range(n).map(k => zE[k + 1] - zE[k]);
  const c = zc.map(chord), Vz = zc.map(V), ag = zc.map(alpha), a0z = zc.map(z => alpha0Fn ? alpha0Fn(z) : 0);
  // influence matrix: downwash at control point i per unit Γ_k
  const W = range(n).map(i => range(n).map(k => { const f = (ze, s) => s / (4 * Math.PI * (zc[i] - ze)); return f(zE[k], 1) + f(zE[k + 1], -1) + mirror * (f(-zE[k], -1) + f(-zE[k + 1], 1)); }));
  // Quasi-Newton (secant-linearised) solution: (I + diag(½ c s) W) Γ = ½ V c [cl(α*) + s (α_g − α*)]
  let G = zc.map((z, i) => 0.5 * Vz[i] * c[i] * section.cl(ag[i] - a0z[i]) * 0.6), it = 0, res = 1; const hist = [];
  for (; it < Math.min(maxIt, 200); it++) {
    const w = W.map(row => dot(row, G)), ae = w.map((v, i) => ag[i] - v / Math.max(Vz[i], 1e-6));
    const sl = ae.map((a, i) => { const h = 1e-4, d = (section.cl(a + h - a0z[i]) - section.cl(a - h - a0z[i])) / (2 * h); return Math.max(d, 0.08 * section.a0); });
    const Am = range(n).map(i => range(n).map(k => (i === k ? 1 : 0) + 0.5 * c[i] * sl[i] * W[i][k]));
    const rhs = range(n).map(i => 0.5 * Vz[i] * c[i] * (section.cl(ae[i] - a0z[i]) + sl[i] * (ag[i] - ae[i])));
    const Gn = luSolve(Am, rhs);
    res = norm(Gn.map((g, i) => g - G[i])) / (norm(Gn) || 1); hist.push(Math.max(res, 1e-16));
    const om = it < 3 ? 1 : 0.7; G = G.map((g, i) => g + om * (Gn[i] - g));
    if (res < tol) break;
  }
  const w = W.map(row => dot(row, G)), ai = w.map((v, i) => v / Math.max(Vz[i], 1e-6));
  const aeff = ag.map((a, i) => a - ai[i]), cl = aeff.map((a, i) => section.cl(a - a0z[i])), cd = cl.map((x, i) => section.cd(x, aeff[i] - a0z[i]));
  const l = G.map((g, i) => rho * Vz[i] * g), di = l.map((x, i) => x * ai[i]), dp = cd.map((x, i) => 0.5 * rho * Vz[i] ** 2 * c[i] * x);
  const L = l.reduce((s, x, i) => s + x * dz[i], 0), Di = di.reduce((s, x, i) => s + x * dz[i], 0), Dp = dp.reduce((s, x, i) => s + x * dz[i], 0);
  const S = c.reduce((s, x, i) => s + x * dz[i], 0), qref = 0.5 * rho * (Vz.reduce((s, v, i) => s + v * v * c[i] * dz[i], 0) / S);
  const Mroot = l.reduce((s, x, i) => s + x * zc[i] * dz[i], 0);
  return { z: zc, dz, chord: c, V: Vz, gamma: G, alphaInd: ai, alphaEff: aeff, cl, cd, l, L, Di, Dp, D: Di + Dp, S, CL: L / (qref * S), CD: (Di + Dp) / (qref * S), CDi: Di / (qref * S), qref, Mroot, iterations: it, residual: res, hist, stalled: aeff.some((a, i) => Math.abs(a - a0z[i]) > section.clmax / section.a0 * 1.05) };
}

// ---------- partitioned static aeroelastic coupling ----------
// model: beamModel; aeroFn(phiNodes) -> {Fw:[nodal forces], Ft:[nodal torques], ...}; returns converged coupled state
export function staticAeroelastic(model, aeroFn, { method = 'aitken', tol = 1e-6, maxIt = 60, omega0 = 0.5 } = {}) {
  const N = model.N; let x = new Array(N).fill(0), omega = omega0, rPrev = null, hist = [], last = null, solid = null;
  const Vs = [], Ws = []; let xPrevTilde = null, rPrevQ = null;
  const S = (phi) => { const a = aeroFn(phi); const F = new Array(model.ndof).fill(0); for (let i = 0; i < N; i++) { F[3 * i] = a.Fw[i]; F[3 * i + 2] = a.Ft[i]; } const sol = beamStatic(model, F); return { a, sol, phi: sol.phi }; };
  for (let k = 0; k < maxIt; k++) {
    const out = S(x); last = out.a; solid = out.sol; const xt = out.phi, r = xt.map((v, i) => v - x[i]), rn = norm(r) / (norm(xt) + 1e-9);
    hist.push(Math.max(rn, 1e-16));
    if (rn < tol) { x = xt; break; }
    if (method === 'iqn' && rPrevQ) {
      Vs.unshift(r.map((v, i) => v - rPrevQ[i])); Ws.unshift(xt.map((v, i) => v - xPrevTilde[i])); if (Vs.length > 8) { Vs.pop(); Ws.pop(); }
      const m = Vs.length, VtV = range(m).map(i => range(m).map(j => dot(Vs[i], Vs[j]) + (i === j ? 1e-14 : 0))), Vtr = range(m).map(i => -dot(Vs[i], r));
      let cvec; try { cvec = luSolve(VtV, Vtr); } catch { cvec = new Array(m).fill(0); }
      const dx = r.map((v, i) => v + Ws.reduce((s, w, j) => s + w[i] * cvec[j], 0));
      rPrevQ = r; xPrevTilde = xt; x = x.map((v, i) => v + dx[i]);
    } else {
      if (method === 'aitken' && rPrev) { const dr = r.map((v, i) => v - rPrev[i]), d2 = dot(dr, dr); if (d2 > 0) omega = Math.max(-2, Math.min(2, -omega * dot(rPrev, dr) / d2)); }
      if (method === 'fixed') omega = omega0;
      if (method === 'iqn') { rPrevQ = r; xPrevTilde = xt; omega = 0.3; }
      rPrev = r; x = x.map((v, i) => v + omega * r[i]);
    }
  }
  return { phi: x, aero: last, sol: solid, history: hist, iterations: hist.length, converged: hist[hist.length - 1] < tol };
}

// Static divergence: smallest dynamic pressure q where K − q·A loses positive definiteness (strip theory)
export function divergenceQ(model, chord, e, a0) {
  const N = model.N, nf = model.free.length, pos = new Map(model.free.map((d, k) => [d, k]));
  // aerodynamic stiffness acting through twist DOFs (strip theory): lift on w_i and torque on φ_i per unit q and φ_i
  const cols = range(N - 1).map(k => k + 1).map(i => { const z = model.z[i], tw = (i === N - 1 ? 0.5 : 1) * model.le, c = chord(z); const col = new Array(nf).fill(0); col[pos.get(3 * i)] = c * a0 * tw; col[pos.get(3 * i + 2)] = c * a0 * e(z) * tw; return col; });
  const sol = cols.map(col => luSolve(model.Kr, col)); // K⁻¹ A, columns for each φ_i
  const B = range(N - 1).map(k => k + 1).map(i => sol.map(x => x[pos.get(3 * i + 2)])); // rows: φ_i response
  const ev = eigGeneral(B.map(r => Array.from(r))).filter(v => v.re > 0 && Math.abs(v.im) <= 1e-6 * Math.abs(v.re)).map(v => v.re);
  const lam = ev.length ? Math.max(...ev) : 0; return lam > 0 ? 1 / lam : Infinity;
}

// ---------- unsteady aerodynamics ----------
const cx = (re, im = 0) => ({ re, im }), cadd = (a, b) => cx(a.re + b.re, a.im + b.im), cmul = (a, b) => cx(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re), cdiv = (a, b) => { const d = b.re * b.re + b.im * b.im; return cx((a.re * b.re + a.im * b.im) / d, (a.im * b.re - a.re * b.im) / d); }, csqrt = (a) => { const r = Math.hypot(a.re, a.im), re = Math.sqrt((r + a.re) / 2), im = Math.sign(a.im || 1) * Math.sqrt(Math.max(0, (r - a.re) / 2)); return cx(re, im); }, cs = (s, a) => cx(s * a.re, s * a.im);
// Theodorsen function via R. T. Jones' exponential approximation of the Wagner function
export const theodorsen = (k) => { const ik = cx(0, k); return cadd(cx(1), cadd(cs(-0.165, cdiv(ik, cadd(ik, cx(0.0455)))), cs(-0.335, cdiv(ik, cadd(ik, cx(0.3)))))); };
// Flutter eigenvector of the typical section at reduced frequency k (same Theodorsen operator as flutterVg):
// returns the plunge/pitch amplitude ratio (h/b per radian of pitch) and the phase of plunge relative to pitch.
export function flutterModeShape(sec, k, fTarget) {
  const { m, Ia, xa, a, b, wh, wa, rho } = sec, Sa = m * xa * b, Kh = m * wh * wh, Ka = Ia * wa * wa;
  const C = theodorsen(k), i = cx(0, 1), P = Math.PI * rho;
  const Lh = cs(P * b * b, cadd(cx(-1), cmul(cs(2 / k, C), i)));
  const La = cs(P * b ** 3, cadd(cadd(cx(a), cmul(i, cx(1 / k))), cadd(cs(2 / (k * k), C), cmul(cs(2 * (0.5 - a) / k, C), i))));
  const Mh = cs(P * b ** 3, cadd(cx(-a), cmul(cs(2 * (a + 0.5) / k, C), i)));
  const Ma = cs(P * b ** 4, cadd(cadd(cx(0.125 + a * a), cmul(i, cx(-(0.5 - a) / k))), cadd(cs(2 * (a + 0.5) / (k * k), C), cmul(cs(2 * (a + 0.5) * (0.5 - a) / k, C), i))));
  const Z11 = cs(1 / Kh, cadd(cx(m), cs(-1, Lh))), Z12 = cs(1 / Kh, cadd(cx(Sa), cs(-1, La))), Z21 = cs(1 / Ka, cadd(cx(Sa), Mh)), Z22 = cs(1 / Ka, cadd(cx(Ia), Ma));
  const tr = cadd(Z11, Z22), dt = cadd(cmul(Z11, Z22), cs(-1, cmul(Z12, Z21))), disc = csqrt(cadd(cmul(tr, tr), cs(-4, dt)));
  const lams = [cs(0.5, cadd(tr, disc)), cs(0.5, cadd(tr, cs(-1, disc)))].filter(L => L.re > 0);
  const f = (L) => 1 / Math.sqrt(L.re) / (2 * Math.PI); const lam = lams.sort((p, q) => Math.abs(f(p) - fTarget) - Math.abs(f(q) - fTarget))[0];
  if (!lam) return { hb: 0, phase: 0 };
  // eigenvector of (Z − λI)v = 0 → v = [Z12, λ − Z11] (h/b scaled by b because Z12 multiplies α with moment-arm units)
  const vh = Z12, va = cadd(lam, cs(-1, Z11));
  const ra = Math.hypot(va.re, va.im) || 1e-12, ratio = { re: (vh.re * va.re + vh.im * va.im) / (ra * ra), im: (vh.im * va.re - vh.re * va.im) / (ra * ra) };
  return { hb: Math.hypot(ratio.re, ratio.im) / b, phase: Math.atan2(ratio.im, ratio.re), f: f(lam) };
}

// Typical-section V–g flutter (k-method). sec: {m (kg/m), Ia (kg·m²/m about EA), xa (CG aft of EA, semichords), a (EA position, semichords aft of mid-chord), b (semichord), wh, wa (rad/s), rho, gs}
export function flutterVg(sec, { kmax = 3, kmin = 0.02, nk = 260, vmax = 60 } = {}) {
  const { m, Ia, xa, a, b, wh, wa, rho } = sec, Sa = m * xa * b, Kh = m * wh * wh, Ka = Ia * wa * wa, gs = sec.gs ?? 0.02;
  // the airspeed sweep must reach vmax (≥ 1.2 × the design speed) for the LOWEST mode, independent of any scenario wind:
  // V = ω·b/k, so k must go down to ω_min·b/vmax (a soft flexure pitch mode needs very low reduced frequencies)
  const kLo = Math.min(kmin, 0.8 * Math.min(wh, wa) * b / Math.max(1, vmax)), ks = linspace(Math.log(kmax), Math.log(Math.max(1e-5, kLo)), nk).map(Math.exp); const br = [[], []]; let prev = null;
  for (const k of ks) {
    const C = theodorsen(k), i = cx(0, 1), P = Math.PI * rho;
    // non-dimensional aerodynamic coefficients (Theodorsen), multiplied by ω²; see method notes
    const Lh = cs(P * b * b, cadd(cx(-1), cmul(cs(2 / k, C), i)));
    const La = cs(P * b ** 3, cadd(cadd(cx(a), cmul(i, cx(1 / k))), cadd(cs(2 / (k * k), C), cmul(cs(2 * (0.5 - a) / k, C), i))));
    const Mh = cs(P * b ** 3, cadd(cx(-a), cmul(cs(2 * (a + 0.5) / k, C), i)));
    const Ma = cs(P * b ** 4, cadd(cadd(cx(0.125 + a * a), cmul(i, cx(-(0.5 - a) / k))), cadd(cs(2 * (a + 0.5) / (k * k), C), cmul(cs(2 * (a + 0.5) * (0.5 - a) / k, C), i))));
    const B11 = cadd(cx(m), cs(-1, Lh)), B12 = cadd(cx(Sa), cs(-1, La)), B21 = cadd(cx(Sa), Mh), B22 = cadd(cx(Ia), Ma);
    const Z11 = cs(1 / Kh, B11), Z12 = cs(1 / Kh, B12), Z21 = cs(1 / Ka, B21), Z22 = cs(1 / Ka, B22);
    const tr = cadd(Z11, Z22), dt = cadd(cmul(Z11, Z22), cs(-1, cmul(Z12, Z21))), disc = csqrt(cadd(cmul(tr, tr), cs(-4, dt)));
    let lams = [cs(0.5, cadd(tr, disc)), cs(0.5, cadd(tr, cs(-1, disc)))];
    if (prev) { const d = (u, v) => Math.hypot(u.re - v.re, u.im - v.im); if (d(lams[0], prev[0]) + d(lams[1], prev[1]) > d(lams[0], prev[1]) + d(lams[1], prev[0])) lams = [lams[1], lams[0]]; }
    prev = lams;
    lams.forEach((L, j) => { if (L.re <= 0) return; const w = 1 / Math.sqrt(L.re), g = L.im / L.re; br[j].push({ k, V: w * b / k, f: w / (2 * Math.PI), g }); });
  }
  let Vf = Infinity, ff = NaN, mode = -1;
  br.forEach((B, j) => { const s = B.slice().sort((p, q) => p.V - q.V); for (let i = 1; i < s.length; i++) if (s[i - 1].g < gs && s[i].g >= gs) { const t = (gs - s[i - 1].g) / (s[i].g - s[i - 1].g), V = s[i - 1].V + t * (s[i].V - s[i - 1].V); if (V < Vf) { Vf = V; ff = s[i - 1].f + t * (s[i].f - s[i - 1].f); mode = j; } break; } });
  const e = (a + 0.5) * b, VD = e > 0 ? Math.sqrt(Ka / (0.5 * rho * 2 * b * 2 * Math.PI * e)) : Infinity;
  // highest airspeed actually reached by every tracked branch: 'no crossing' only means stable up to this speed
  const vSwept = Math.max(...br.map(B => B.length ? Math.max(...B.map(q => q.V)) : 0)); // a branch that ends early has become aerodynamically overdamped (no oscillatory root), not unstable
  return { branches: br.map(B => B.slice().sort((p, q) => p.V - q.V)), Vflutter: Vf, fFlutter: ff, mode, Vdiv: VD, vSwept };
}

// Transient gust / motion response of the wing beam: modal Newmark-β structure + Wagner/Küssner indicial strip aerodynamics.
// opts: {model, nModes, chord(z), V(z), rho, a0, e(z), gust(t) [m/s normal to wing], T, dt, zeta, subIt, alphaStatic(z)}
export function gustResponse(o) {
  const { model, chord, V, rho = 1.225, a0, e, gust, T = 6, dt = 0.004, zeta = 0.02, subIt = 1, nModes = 4 } = o;
  const modes = beamModes(model, nModes), N = model.N, z = model.z, tw = z.map((_, i) => (i === 0 || i === N - 1 ? 0.5 : 1) * model.le);
  const nm = modes.length, q = new Array(nm).fill(0), qd = new Array(nm).fill(0), qdd = new Array(nm).fill(0);
  const XM = z.map(() => [0, 0]), XG = z.map(() => [0, 0]), prevAm = z.map(() => 0), prevAg = z.map(() => 0);
  const W = [0.165, 0.335], Wb = [0.0455, 0.3], Kq = [0.5, 0.5], Kb = [0.13, 1.0];
  const out = { t: [], lift: [], Mroot: [], twistTip: [], wTip: [], gust: [], Paero: [], Estr: [], q: [], Wnet: 0, energyErr: 0 };
  const beta = 0.25, gam = 0.5; let Waero = 0, Wdamp = 0; let E0 = 0;
  const loads = (qq, qqd, commit, t) => {
    const phi = z.map((_, i) => modes.reduce((s, m, j) => s + m.full[3 * i + 2] * qq[j], 0)), wdot = z.map((_, i) => modes.reduce((s, m, j) => s + m.full[3 * i] * qqd[j], 0));
    const Fw = [], Ft = []; let wgRef = 0;
    for (let i = 0; i < N; i++) {
      const Vi = Math.max(V(z[i]), 0.3), c = chord(z[i]), ds = 2 * Vi * dt / c;
      const wg = gust(t, z[i]); if (i === Math.floor(N * 0.75)) wgRef = o.gustRef ? o.gustRef(t) : wg;
      const am = phi[i] - wdot[i] / Vi, ag = wg / Vi;
      let xm = XM[i].map((x, j) => x * Math.exp(-Wb[j] * ds) + (am - prevAm[i]) * Math.exp(-Wb[j] * ds / 2)), xg = XG[i].map((x, j) => x * Math.exp(-Kb[j] * ds) + (ag - prevAg[i]) * Math.exp(-Kb[j] * ds / 2));
      // Wagner: circulatory lift lags motion; Küssner: gust penetration lag
      const aEff = am - W[0] * xm[0] - W[1] * xm[1] + ag - Kq[0] * xg[0] - Kq[1] * xg[1];
      if (commit) { XM[i] = xm; XG[i] = xg; prevAm[i] = am; prevAg[i] = ag; }
      const qc = 0.5 * rho * Vi * Vi * c; let l = qc * a0 * aEff + (o.extraLoad ? o.extraLoad(t, z[i]) : 0);
      if (o.clStatic && o.clmax) { const cs0 = o.clStatic(z[i]), ct = Math.max(-o.clmax, Math.min(o.clmax, cs0 + l / qc)); l = qc * (ct - cs0); } // stall-limited lift increment
      Fw.push(l * tw[i]); Ft.push(l * e(z[i]) * tw[i]);
    }
    const Q = modes.map(m => z.reduce((s, _, i) => s + m.full[3 * i] * Fw[i] + m.full[3 * i + 2] * Ft[i], 0));
    return { Q, Fw, Ft, phi, wg: wgRef };
  };
  // initialise memory with zero motion (perturbation about static equilibrium)
  let L0 = loads(q, qd, true, 0); for (let j = 0; j < nm; j++) qdd[j] = L0.Q[j] - 2 * zeta * modes[j].omega * qd[j] - modes[j].omega ** 2 * q[j];
  const steps = Math.round(T / dt);
  for (let s = 0; s <= steps; s++) {
    const t = s * dt;
    let qn = q.slice(), qdn = qd.slice(), qddn = qdd.slice(), Ld = null;
    for (let it = 0; it < Math.max(1, subIt); it++) {
      Ld = loads(it === 0 ? q : qn, it === 0 ? qd : qdn, false, t + dt);
      for (let j = 0; j < nm; j++) {
        const w = modes[j].omega, c = 2 * zeta * w, keff = w * w + gam * c / (beta * dt) + 1 / (beta * dt * dt);
        const rhs = Ld.Q[j] + (q[j] / (beta * dt * dt) + qd[j] / (beta * dt) + (1 / (2 * beta) - 1) * qdd[j]) + c * (gam * q[j] / (beta * dt) + (gam / beta - 1) * qd[j] + dt * (gam / (2 * beta) - 1) * qdd[j]);
        qn[j] = rhs / keff; qddn[j] = (qn[j] - q[j]) / (beta * dt * dt) - qd[j] / (beta * dt) - (1 / (2 * beta) - 1) * qdd[j]; qdn[j] = qd[j] + dt * ((1 - gam) * qdd[j] + gam * qddn[j]);
      }
    }
    const Lc = loads(qn, qdn, true, t + dt);
    for (let j = 0; j < nm; j++) { Waero += 0.5 * (Lc.Q[j] + (L0.Q[j] || 0)) * (qn[j] - q[j]); const vm = 0.5 * (qdn[j] + qd[j]); Wdamp += 2 * zeta * modes[j].omega * vm * vm * dt; }
    L0 = Lc; for (let j = 0; j < nm; j++) { q[j] = qn[j]; qd[j] = qdn[j]; qdd[j] = qddn[j]; }
    { let k1 = 0, k2 = 0; for (let j = 0; j < nm; j++) { k1 += (modes[j].full[4] - modes[j].full[1]) * q[j]; k2 += (modes[j].full[5] - modes[j].full[2]) * q[j]; } const p0 = o.props(model.le / 2), Mr = (p0.EI * k1 + (p0.K || 0) * k2) / model.le; if (Math.abs(Mr) > Math.abs(out.peakMroot || 0)) out.peakMroot = Mr; }
    if (s % Math.max(1, Math.round(0.02 / dt)) === 0) {
      const u = new Array(model.ndof).fill(0); for (let j = 0; j < nm; j++) for (let d = 0; d < model.ndof; d++) u[d] += modes[j].full[d] * q[j];
      const kap = (u[4] - u[1]) / model.le, dph = (u[5] - u[2]) / model.le, p0 = o.props(model.le / 2);
      const Estr = 0.5 * modes.reduce((s2, m, j) => s2 + qd[j] ** 2 + (m.omega * q[j]) ** 2, 0);
      out.t.push(t); out.lift.push(Lc.Fw.reduce((a, b) => a + b, 0)); out.Mroot.push(p0.EI * kap + (p0.K || 0) * dph); out.twistTip.push(u[3 * (N - 1) + 2] * 180 / Math.PI); out.wTip.push(u[3 * (N - 1)]); out.gust.push(Lc.wg); out.Estr.push(Estr); out.q.push(q.slice());
    }
  }
  const Ef = 0.5 * modes.reduce((s2, m, j) => s2 + qd[j] ** 2 + (m.omega * q[j]) ** 2, 0) - E0;
  out.Waero = Waero; out.Wdamp = Wdamp; out.energyErr = Math.abs(Waero - Wdamp - Ef) / (Math.abs(Waero) + Math.abs(Wdamp) + 1e-12);
  out.modes = modes; return out;
}
export function oneMinusCosGust(t0, dur, amp) { return (t) => (t < t0 || t > t0 + dur) ? 0 : 0.5 * amp * (1 - Math.cos(2 * Math.PI * (t - t0) / dur)); }
export { matvec };
