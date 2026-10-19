// Aerodynamics core: Hess–Smith panel method (constant-strength sources + uniform vorticity, Kutta condition),
// integral boundary layer (Thwaites laminar, Michel transition, Head turbulent, Ludwieg–Tillmann skin friction,
// Squire–Young wake drag), Kirchhoff separated-flow correction and flat-plate post-stall extension.
import { luSolve, rad, deg, linspace, clamp } from './num.js';

const TWO_PI = 2 * Math.PI;

// points: closed airfoil, TE(upper) → LE → TE(lower) (cad.naca4 ordering). Re-ordered internally to clockwise
// (TE lower → LE → TE upper) so that the outward normal is the left normal of each panel.
export function buildPanels(points) {
  let P = points.slice();
  let area = 0; for (let i = 0; i < P.length; i++) { const a = P[i], b = P[(i + 1) % P.length]; area += a[0] * b[1] - b[0] * a[1]; }
  if (area > 0) P = P.reverse(); // ensure clockwise
  const n = P.length - 1, pan = [];
  for (let j = 0; j < n; j++) { const [x1, y1] = P[j], [x2, y2] = P[j + 1], dx = x2 - x1, dy = y2 - y1, l = Math.hypot(dx, dy); pan.push({ x1, y1, x2, y2, xm: (x1 + x2) / 2, ym: (y1 + y2) / 2, l, th: Math.atan2(dy, dx), tx: dx / l, ty: dy / l, nx: -dy / l, ny: dx / l }); }
  return { nodes: P, panels: pan, n };
}

// Influence coefficients (normal & tangential velocity at control points from unit source / unit vortex density)
function influence(pan) {
  const n = pan.length, An = [], At = [], Bn = new Float64Array(n), Bt = new Float64Array(n);
  for (let i = 0; i < n; i++) { An.push(new Float64Array(n)); At.push(new Float64Array(n)); }
  for (let i = 0; i < n; i++) {
    const pi = pan[i];
    for (let j = 0; j < n; j++) {
      const pj = pan[j]; let lnr, beta;
      if (i === j) { lnr = 0; beta = Math.PI; }
      else { const r1 = Math.hypot(pi.xm - pj.x1, pi.ym - pj.y1), r2 = Math.hypot(pi.xm - pj.x2, pi.ym - pj.y2); lnr = Math.log(r1 / r2); const ax = pj.x1 - pi.xm, ay = pj.y1 - pi.ym, bx = pj.x2 - pi.xm, by = pj.y2 - pi.ym; beta = Math.atan2(ax * by - ay * bx, ax * bx + ay * by); }
      // local velocities from unit source: (lnr, beta)/2π ; unit vortex (counter-clockwise): (-beta, lnr)/2π
      const us = lnr / TWO_PI, vs = beta / TWO_PI, uv = -beta / TWO_PI, vv = lnr / TWO_PI;
      const c = Math.cos(pj.th), s = Math.sin(pj.th);
      const usx = us * c - vs * s, usy = us * s + vs * c, uvx = uv * c - vv * s, uvy = uv * s + vv * c;
      An[i][j] = usx * pi.nx + usy * pi.ny; At[i][j] = usx * pi.tx + usy * pi.ty;
      Bn[i] += uvx * pi.nx + uvy * pi.ny; Bt[i] += uvx * pi.tx + uvy * pi.ty;
    }
  }
  return { An, At, Bn, Bt };
}

export function panelSolver(points) {
  const geo = buildPanels(points), pan = geo.panels, n = pan.length, I = influence(pan);
  const A = []; for (let i = 0; i < n; i++) { const r = new Float64Array(n + 1); for (let j = 0; j < n; j++) r[j] = I.An[i][j]; r[n] = I.Bn[i]; A.push(r); }
  const k = new Float64Array(n + 1); for (let j = 0; j < n; j++) k[j] = I.At[0][j] + I.At[n - 1][j]; k[n] = I.Bt[0] + I.Bt[n - 1]; A.push(k);
  const chord = Math.max(...geo.nodes.map(p => p[0])) - Math.min(...geo.nodes.map(p => p[0])) || 1;
  const xle = Math.min(...geo.nodes.map(p => p[0]));
  return {
    geo, n, chord,
    solve(alphaDeg) {
      const a = rad(alphaDeg), ca = Math.cos(a), sa = Math.sin(a), b = new Array(n + 1);
      for (let i = 0; i < n; i++) b[i] = -(ca * pan[i].nx + sa * pan[i].ny);
      b[n] = -(ca * (pan[0].tx + pan[n - 1].tx) + sa * (pan[0].ty + pan[n - 1].ty));
      const sol = luSolve(A, b), q = sol.slice(0, n), g = sol[n];
      const Vt = new Float64Array(n), Cp = new Float64Array(n); let Cx = 0, Cy = 0, Cm = 0, srcSum = 0;
      for (let i = 0; i < n; i++) {
        let v = ca * pan[i].tx + sa * pan[i].ty + g * I.Bt[i]; for (let j = 0; j < n; j++) v += q[j] * I.At[i][j];
        Vt[i] = v; Cp[i] = 1 - v * v; srcSum += q[i] * pan[i].l;
        const fx = -Cp[i] * pan[i].nx * pan[i].l, fy = -Cp[i] * pan[i].ny * pan[i].l; Cx += fx; Cy += fy; Cm += -(pan[i].xm - xle - 0.25 * chord) * fy + pan[i].ym * fx;
      }
      Cx /= chord; Cy /= chord; Cm /= chord * chord;
      const CL = Cy * ca - Cx * sa, CDp = Cx * ca + Cy * sa;
      const perim = pan.reduce((s, p) => s + p.l, 0), Gamma = g * perim;
      return { alpha: alphaDeg, CL, CDp, CM: Cm, Vt, Cp, q, gamma: g, Gamma, CLkj: -2 * Gamma / chord, srcSum, panels: pan };
    },
    // non-lifting (source-only, no Kutta condition) solution — reference for potential-flow mesh studies
    solveNonLifting(alphaDeg) {
      const a = rad(alphaDeg), ca = Math.cos(a), sa = Math.sin(a), M = I.An.map(r => Float64Array.from(r)), b = pan.map(p => -(ca * p.nx + sa * p.ny)), q = luSolve(M, b);
      const Vt = pan.map((p, i) => { let v = ca * p.tx + sa * p.ty; for (let j = 0; j < n; j++) v += q[j] * I.At[i][j]; return v; });
      return { q, Vt, VmaxSurface: Math.max(...Vt.map(Math.abs)) };
    },
    // velocity at arbitrary field point (for flow-field contours)
    velocity(sol, x, y, alphaDeg) {
      const a = rad(alphaDeg); let u = Math.cos(a), v = Math.sin(a);
      for (let j = 0; j < n; j++) {
        const pj = pan[j], r1 = Math.hypot(x - pj.x1, y - pj.y1), r2 = Math.hypot(x - pj.x2, y - pj.y2), lnr = Math.log(r1 / r2);
        const ax = pj.x1 - x, ay = pj.y1 - y, bx = pj.x2 - x, by = pj.y2 - y, beta = Math.atan2(ax * by - ay * bx, ax * bx + ay * by);
        const ul = (sol.q[j] * lnr - sol.gamma * beta) / TWO_PI, vl = (sol.q[j] * beta + sol.gamma * lnr) / TWO_PI, c = Math.cos(pj.th), s = Math.sin(pj.th);
        u += ul * c - vl * s; v += ul * s + vl * c;
      }
      return [u, v];
    },
  };
}

// Head's entrainment shape-factor correlations
const H1ofH = (H) => H <= 1.6 ? 3.3 + 0.8234 * (H - 1.1) ** -1.287 : 3.3 + 1.5501 * (H - 0.6778) ** -3.064;
function HofH1(H1) { const d = Math.max(1e-6, H1 - 3.3); return H1 >= 5.3 ? 0.86 * d ** -0.777 + 1.1 : 1.1538 * d ** -0.326 + 0.6778; }

// Integral boundary layer on one surface. s: arc length from stagnation (chord-normalised), ue: edge speed/V∞.
function surfaceBL(s, ue, x, Re, { xtr = 1 } = {}) {
  const nu = 1 / Re, m = s.length, th = new Float64Array(m), H = new Float64Array(m), cf = new Float64Array(m), lam = new Float64Array(m);
  let state = 'lam', itr = -1, isep = -1, laminarSep = false, I5 = 0;
  const due0 = (ue[1] - ue[0]) / Math.max(1e-9, s[1] - s[0]);
  th[0] = Math.sqrt(0.075 * nu / Math.max(1e-6, Math.abs(due0))); H[0] = 2.61;
  let H1 = 0;
  for (let k = 1; k < m; k++) {
    const ds = Math.max(1e-9, s[k] - s[k - 1]), u = Math.max(1e-4, ue[k]), up = Math.max(1e-4, ue[k - 1]), dudx = (u - up) / ds;
    if (state === 'lam') {
      I5 += 0.5 * (u ** 5 + up ** 5) * ds;
      const t2 = 0.45 * nu / u ** 6 * I5 + (th[0] ** 2) * (Math.max(1e-4, ue[0]) / u) ** 6; th[k] = Math.sqrt(t2);
      const l = t2 * dudx / nu; lam[k] = l;
      const L = clamp(l, -0.09, 0.25), Hl = L >= 0 ? 2.61 - 3.75 * L + 5.24 * L * L : 2.088 + 0.0731 / (L + 0.14), ll = L >= 0 ? 0.22 + 1.57 * L - 1.8 * L * L : 0.22 + 1.402 * L + 0.018 * L / (L + 0.107);
      H[k] = Hl; cf[k] = 2 * nu * ll / (u * th[k]);
      const Ret = u * th[k] / nu, Rex = u * s[k] / nu;
      const michel = Ret > 1.174 * (1 + 22400 / Math.max(Rex, 1)) * Rex ** 0.46;
      if (l < -0.09) laminarSep = true;
      if (michel || l < -0.09 || x[k] >= xtr) { state = 'turb'; itr = k; H[k] = 1.4; H1 = H1ofH(1.4); if (laminarSep && !michel) { /* separation-induced transition (short bubble assumed) */ } }
    } else {
      const t = th[k - 1], Hk = H[k - 1], Ret = Math.max(50, up * t / nu);
      const cfk = 0.246 * 10 ** (-0.678 * Hk) * Ret ** -0.268;
      const dth = cfk / 2 - (Hk + 2) * t / up * dudx;
      const F = 0.0306 * Math.max(1e-6, H1 - 3) ** -0.6169;
      const d = up * F - H1 * (up * dth + t * dudx); // d(ue θ H1)/ds = ue F  → dH1/ds
      th[k] = Math.max(1e-7, t + dth * ds); H1 = Math.max(3.31, H1 + d / (up * t) * ds);
      H[k] = clamp(HofH1(H1), 1.1, 4); cf[k] = 0.246 * 10 ** (-0.678 * H[k]) * Math.max(50, u * th[k] / nu) ** -0.268;
      if (H[k] > 2.4 && isep < 0) { isep = k; }
      if (isep >= 0) cf[k] = 0;
    }
  }
  return { th, H, cf, lam, itr, isep, laminarSep: laminarSep && itr >= 0 && lam[itr] < -0.09 };
}

// Viscous analysis at one incidence: panel solution + BL on both surfaces → CL, CD, transition & separation
export function viscous(solver, alphaDeg, Re, { xtrU = 1, xtrL = 1 } = {}) {
  const sol = solver.solve(alphaDeg), pan = sol.panels, n = pan.length;
  // stagnation point: sign change of Vt (clockwise ordering: lower surface first, Vt<0 on lower)
  let ist = 0; for (let i = 0; i < n - 1; i++) if (sol.Vt[i] <= 0 && sol.Vt[i + 1] > 0) { ist = i; break; }
  const chord = solver.chord, xle = Math.min(...solver.geo.nodes.map(p => p[0]));
  const mk = (idx) => { const s = [0], ue = [], x = []; idx.forEach((i, k) => { if (k) s.push(s[k - 1] + 0.5 * (pan[idx[k - 1]].l + pan[i].l) / chord); ue.push(Math.abs(sol.Vt[i])); x.push((pan[i].xm - xle) / chord); }); return { s, ue, x }; };
  const upIdx = []; for (let i = ist + 1; i < n; i++) upIdx.push(i);
  const loIdx = []; for (let i = ist; i >= 0; i--) loIdx.push(i);
  const U = mk(upIdx), Lo = mk(loIdx);
  const bu = surfaceBL(U.s, U.ue, U.x, Re, { xtr: xtrU }), bl = surfaceBL(Lo.s, Lo.ue, Lo.x, Re, { xtr: xtrL });
  const sy = (b, S) => { const k = b.th.length - 1, Hte = clamp(b.H[k], 1, 2.5); return 2 * b.th[k] * Math.max(0.05, S.ue[k]) ** ((Hte + 5) / 2); };
  const CDf = sy(bu, U) + sy(bl, Lo);
  // separated-flow (Kirchhoff) correction using the upper-surface separation location
  const fsep = bu.isep >= 0 ? clamp(U.x[bu.isep], 0, 1) : 1, fsepL = bl.isep >= 0 ? clamp(Lo.x[bl.isep], 0, 1) : 1;
  const f = Math.min(fsep, fsepL), kf = ((1 + Math.sqrt(f)) / 2) ** 2, a = rad(alphaDeg);
  const CN = (sol.CL * Math.cos(a) + sol.CDp * Math.sin(a)) * kf;
  const CL = CN * Math.cos(a), CDsep = Math.max(0, CN * Math.sin(a) - Math.abs(sol.CL) * Math.abs(a) * Math.sqrt(f) * Math.cos(a) * kf);
  const CD = CDf + (f < 1 ? CDsep : 0);
  const at = (b, S) => ({ x: S.x, s: S.s, ue: S.ue, theta: Array.from(b.th), H: Array.from(b.H), cf: Array.from(b.cf), xtr: b.itr >= 0 ? S.x[b.itr] : 1, xsep: b.isep >= 0 ? S.x[b.isep] : null, laminarSep: b.laminarSep });
  return { alpha: alphaDeg, CLinv: sol.CL, CL, CD, CDf, CM: sol.CM, f, sol, upper: at(bu, U), lower: at(bl, Lo) };
}

// Polar with flat-plate post-stall blending so apparent-wind sweeps to 90°+ remain defined
export function polar(points, Re, { alphas = linspace(-6, 22, 29), xtrU = 1, xtrL = 1, panelsPerSide } = {}) {
  const solver = panelSolver(points); const out = { alpha: [], CL: [], CD: [], CM: [], CLinv: [], xtrU: [], xsepU: [], f: [], Re };
  for (const a of alphas) { const v = viscous(solver, a, Re, { xtrU, xtrL }); out.alpha.push(a); out.CL.push(v.CL); out.CD.push(v.CD); out.CM.push(v.CM); out.CLinv.push(v.CLinv); out.xtrU.push(v.upper.xtr); out.xsepU.push(v.upper.xsep); out.f.push(v.f); }
  let im = 0; out.CL.forEach((c, i) => { if (c > out.CL[im]) im = i; }); out.CLmax = out.CL[im]; out.alphaStall = out.alpha[im];
  out.solver = solver; void panelsPerSide; return out;
}
// Interpolated coefficients for any incidence (−180..180): attached polar inside range, flat plate outside
export function coeffs(pol, alphaDeg, CD0 = 0.02) {
  const A = pol.alpha, a0 = A[0], amax = pol.alphaStall;
  const interp = (arr, x) => { for (let i = 0; i < A.length - 1; i++) if (A[i] <= x && x <= A[i + 1]) { const t = (x - A[i]) / (A[i + 1] - A[i]); return arr[i] + t * (arr[i + 1] - arr[i]); } return x < A[0] ? arr[0] : arr[arr.length - 1]; };
  if (alphaDeg < a0) { const c = coeffs(pol, -alphaDeg, CD0); return { CL: -c.CL, CD: c.CD, regime: c.regime + ' (mirrored)' }; }
  if (alphaDeg <= amax) return { CL: interp(pol.CL, alphaDeg), CD: interp(pol.CD, alphaDeg), regime: 'attached' };
  // blend over 8° beyond stall into flat-plate normal-force behaviour (CN ≈ 1.98 sin α)
  const a = Math.min(alphaDeg, 180), t = clamp((a - amax) / 8, 0, 1), fpCN = 1.98 * Math.sin(rad(a)), fpCL = fpCN * Math.cos(rad(a)), fpCD = CD0 + fpCN * Math.sin(rad(a));
  const clS = interp(pol.CL, amax), cdS = interp(pol.CD, amax);
  return { CL: (1 - t) * clS * (1 - 0.3 * t) + t * fpCL, CD: (1 - t) * cdS + t * fpCD, regime: t < 1 ? 'stall transition' : 'flat-plate post-stall' };
}
// Finite-wing (lifting-line) correction with effective aspect ratio and Oswald efficiency
export function finiteWing(CL2D, CD2D, AR, e = 0.85, a0 = TWO_PI) {
  const k = 1 / (1 + a0 / (Math.PI * AR * e)); const CL = CL2D * k; return { CL, CD: CD2D + CL * CL / (Math.PI * AR * e), CDi: CL * CL / (Math.PI * AR * e), k };
}
export { deg };
