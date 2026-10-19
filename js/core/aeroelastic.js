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
  const N = model.N, A = model.K.map(r => r.map(() => 0));
  for (let i = 0; i < N; i++) { const z = model.z[i], tw = (i === 0 || i === N - 1 ? 0.5 : 1) * model.le, c = chord(z), ei = e(z); A[3 * i][3 * i + 2] += c * a0 * tw; A[3 * i + 2][3 * i + 2] += c * a0 * ei * tw; }
  const Ar = model.free.map(i => model.free.map(j => A[i][j])), Ki = invert(model.Kr), Z = Ki.map(r => model.free.map((_, j) => Ar.reduce((s, row, k) => s + r[k] * row[j], 0)));
  const ev = eigGeneral(Z.map(r => Array.from(r))).filter(v => Math.abs(v.im) < 1e-9 * (Math.abs(v.re) + 1e-30) && v.re > 0).map(v => v.re);
  const lam = ev.length ? Math.max(...ev) : 0; return lam > 0 ? 1 / lam : Infinity;
}

// ---------- unsteady aerodynamics ----------
const cx = (re, im = 0) => ({ re, im }), cadd = (a, b) => cx(a.re + b.re, a.im + b.im), cmul = (a, b) => cx(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re), cdiv = (a, b) => { const d = b.re * b.re + b.im * b.im; return cx((a.re * b.re + a.im * b.im) / d, (a.im * b.re - a.re * b.im) / d); }, csqrt = (a) => { const r = Math.hypot(a.re, a.im), re = Math.sqrt((r + a.re) / 2), im = Math.sign(a.im || 1) * Math.sqrt(Math.max(0, (r - a.re) / 2)); return cx(re, im); }, cs = (s, a) => cx(s * a.re, s * a.im);
// Theodorsen function via R. T. Jones' exponential approximation of the Wagner function
export const theodorsen = (k) => { const ik = cx(0, k); return cadd(cx(1), cadd(cs(-0.165, cdiv(ik, cadd(ik, cx(0.0455)))), cs(-0.335, cdiv(ik, cadd(ik, cx(0.3)))))); };
// Typical-section V–g flutter (k-method). sec: {m (kg/m), Ia (kg·m²/m about EA), xa (CG aft of EA, semichords), a (EA position, semichords aft of mid-chord), b (semichord), wh, wa (rad/s), rho, gs}
export function flutterVg(sec, { kmax = 3, kmin = 0.02, nk = 220 } = {}) {
  const { m, Ia, xa, a, b, wh, wa, rho } = sec, Sa = m * xa * b, Kh = m * wh * wh, Ka = Ia * wa * wa, gs = sec.gs ?? 0.02;
  const ks = linspace(Math.log(kmax), Math.log(kmin), nk).map(Math.exp); const br = [[], []]; let prev = null;
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
  return { branches: br.map(B => B.slice().sort((p, q) => p.V - q.V)), Vflutter: Vf, fFlutter: ff, mode, Vdiv: VD };
}

// Transient gust / motion response of the wing beam: modal Newmark-β structure + Wagner/Küssner indicial strip aerodynamics.
// opts: {model, nModes, chord(z), V(z), rho, a0, e(z), gust(t) [m/s normal to wing], T, dt, zeta, subIt, alphaStatic(z)}
export function gustResponse(o) {
  const { model, chord, V, rho = 1.225, a0, e, gust, T = 6, dt = 0.004, zeta = 0.02, subIt = 1, nModes = 4 } = o;
  const modes = beamModes(model, nModes), N = model.N, z = model.z, tw = z.map((_, i) => (i === 0 || i === N - 1 ? 0.5 : 1) * model.le);
  const nm = modes.length, q = new Array(nm).fill(0), qd = new Array(nm).fill(0), qdd = new Array(nm).fill(0);
  const XM = z.map(() => [0, 0]), XG = z.map(() => [0, 0]), prevAm = z.map(() => 0), prevAg = z.map(() => 0);
  const W = [0.165, 0.335], Wb = [0.0455, 0.3], Kq = [0.5, 0.5], Kb = [0.13, 1.0];
  const out = { t: [], lift: [], Mroot: [], twistTip: [], wTip: [], gust: [], Paero: [], Estr: [], Wnet: 0, energyErr: 0 };
  const beta = 0.25, gam = 0.5; let Waero = 0, Wdamp = 0; let E0 = 0;
  const loads = (qq, qqd, commit, t) => {
    const phi = z.map((_, i) => modes.reduce((s, m, j) => s + m.full[3 * i + 2] * qq[j], 0)), wdot = z.map((_, i) => modes.reduce((s, m, j) => s + m.full[3 * i] * qqd[j], 0));
    const wg = gust(t), Fw = [], Ft = [];
    for (let i = 0; i < N; i++) {
      const Vi = Math.max(V(z[i]), 0.3), c = chord(z[i]), ds = 2 * Vi * dt / c;
      const am = phi[i] - wdot[i] / Vi, ag = wg / Vi;
      let xm = XM[i].map((x, j) => x * Math.exp(-Wb[j] * ds) + (am - prevAm[i]) * Math.exp(-Wb[j] * ds / 2)), xg = XG[i].map((x, j) => x * Math.exp(-Kb[j] * ds) + (ag - prevAg[i]) * Math.exp(-Kb[j] * ds / 2));
      // Wagner: circulatory lift lags motion; Küssner: gust penetration lag
      const aEff = am - W[0] * xm[0] - W[1] * xm[1] + ag - Kq[0] * xg[0] - Kq[1] * xg[1];
      if (commit) { XM[i] = xm; XG[i] = xg; prevAm[i] = am; prevAg[i] = ag; }
      const l = 0.5 * rho * Vi * Vi * c * a0 * aEff; Fw.push(l * tw[i]); Ft.push(l * e(z[i]) * tw[i]);
    }
    const Q = modes.map(m => z.reduce((s, _, i) => s + m.full[3 * i] * Fw[i] + m.full[3 * i + 2] * Ft[i], 0));
    return { Q, Fw, Ft, phi, wg };
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
    if (s % Math.max(1, Math.round(0.02 / dt)) === 0) {
      const u = new Array(model.ndof).fill(0); for (let j = 0; j < nm; j++) for (let d = 0; d < model.ndof; d++) u[d] += modes[j].full[d] * q[j];
      const kap = (u[4] - u[1]) / model.le, dph = (u[5] - u[2]) / model.le, p0 = o.props(model.le / 2);
      const Estr = 0.5 * modes.reduce((s2, m, j) => s2 + qd[j] ** 2 + (m.omega * q[j]) ** 2, 0);
      out.t.push(t); out.lift.push(Lc.Fw.reduce((a, b) => a + b, 0)); out.Mroot.push(p0.EI * kap + (p0.K || 0) * dph); out.twistTip.push(u[3 * (N - 1) + 2] * 180 / Math.PI); out.wTip.push(u[3 * (N - 1)]); out.gust.push(Lc.wg); out.Estr.push(Estr);
    }
  }
  const Ef = 0.5 * modes.reduce((s2, m, j) => s2 + qd[j] ** 2 + (m.omega * q[j]) ** 2, 0) - E0;
  out.Waero = Waero; out.Wdamp = Wdamp; out.energyErr = Math.abs(Waero - Wdamp - Ef) / (Math.abs(Waero) + Math.abs(Wdamp) + 1e-12);
  out.modes = modes; return out;
}
export function oneMinusCosGust(t0, dur, amp) { return (t) => (t < t0 || t > t0 + dur) ? 0 : 0.5 * amp * (1 - Math.cos(2 * Math.PI * (t - t0) / dur)); }
export { matvec };
