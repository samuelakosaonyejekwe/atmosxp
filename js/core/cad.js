// Parametric CAD kernel (procedural B-rep-like surface patches tessellated to watertight triangle shells),
// airfoil parameterisations, whole-vehicle assembly, dimensioned 2D drawings and neutral-format exporters.
import { linspace, rad, deg, range } from './num.js';
import { DEFAULT_VEHICLE } from './physics.js';

// ---------------- Airfoils ----------------
export function naca4(code = '0018', n = 81, closedTE = true) {
  const m = +code[0] / 100, p = +code[1] / 10, t = +code.slice(2) / 100;
  const beta = linspace(0, Math.PI, n), xs = beta.map(b => 0.5 * (1 - Math.cos(b)));
  const a4 = closedTE ? -0.1036 : -0.1015;
  const pt = (x) => {
    const yt = 5 * t * (0.2969 * Math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x ** 3 + a4 * x ** 4);
    let yc = 0, dy = 0; if (m && p) { if (x < p) { yc = m / p ** 2 * (2 * p * x - x * x); dy = 2 * m / p ** 2 * (p - x); } else { yc = m / (1 - p) ** 2 * (1 - 2 * p + 2 * p * x - x * x); dy = 2 * m / (1 - p) ** 2 * (p - x); } }
    const th = Math.atan(dy); return { xu: x - yt * Math.sin(th), yu: yc + yt * Math.cos(th), xl: x + yt * Math.sin(th), yl: yc - yt * Math.cos(th), yc };
  };
  const P = xs.map(pt);
  const upper = P.map(q => [q.xu, q.yu]).reverse(), lower = P.map(q => [q.xl, q.yl]).slice(1);
  return { name: 'NACA ' + code, points: upper.concat(lower), camber: P.map((q, i) => [xs[i], q.yc]), t, m, p }; // TE(upper) → LE → TE(lower)
}
// Kulfan Class–Shape Transformation airfoil; Au, Al Bernstein weights
export function cst(Au = [0.17, 0.16, 0.15, 0.14], Al = [-0.17, -0.12, -0.1, -0.08], n = 81, dzTE = 0) {
  const bern = (k, N, x) => binom(N, k) * x ** k * (1 - x) ** (N - k);
  const S = (A, x) => A.reduce((s, a, k) => s + a * bern(k, A.length - 1, x), 0);
  const xs = linspace(0, Math.PI, n).map(b => 0.5 * (1 - Math.cos(b))), C = (x) => Math.sqrt(x) * (1 - x);
  const up = xs.map(x => [x, C(x) * S(Au, x) + x * dzTE / 2]).reverse(), lo = xs.map(x => [x, C(x) * S(Al, x) - x * dzTE / 2]).slice(1);
  return { name: 'CST', points: up.concat(lo) };
}
const binom = (n, k) => { let r = 1; for (let i = 1; i <= k; i++) r = r * (n - k + i) / i; return r; };
export function airfoilStats(pts) {
  let area = 0, tmax = 0, xt = 0; for (let i = 0; i < pts.length; i++) { const a = pts[i], b = pts[(i + 1) % pts.length]; area += a[0] * b[1] - b[0] * a[1]; }
  const le = pts.reduce((bi, p, i) => p[0] < pts[bi][0] ? i : bi, 0), up = pts.slice(0, le + 1), lo = pts.slice(le);
  for (const x of linspace(0.01, 0.99, 99)) { const yu = interpPoly(up, x), yl = interpPoly(lo, x); if (yu - yl > tmax) { tmax = yu - yl; xt = x; } }
  return { area: Math.abs(area) / 2, tmax, xt };
}
function interpPoly(seg, x) { for (let i = 0; i < seg.length - 1; i++) { const a = seg[i], b = seg[i + 1]; if ((a[0] - x) * (b[0] - x) <= 0 && a[0] !== b[0]) return a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]); } return 0; }

// ---------------- Mesh primitives (all closed, outward normals) ----------------
export class Mesh {
  constructor(name, opts = {}) { this.name = name; this.p = []; this.i = []; Object.assign(this, { color: '#9aa4b2', group: 'misc', component: null, ...opts }); }
  v(x, y, z) { this.p.push(x, y, z); return this.p.length / 3 - 1; }
  t(a, b, c) { this.i.push(a, b, c); }
  q(a, b, c, d) { this.i.push(a, b, c, a, c, d); }
  get positions() { return Float32Array.from(this.p); } get indices() { return Uint32Array.from(this.i); }
  transform(fn) { for (let k = 0; k < this.p.length; k += 3) { const r = fn(this.p[k], this.p[k + 1], this.p[k + 2]); this.p[k] = r[0]; this.p[k + 1] = r[1]; this.p[k + 2] = r[2]; } return this; }
  translate(dx, dy, dz) { return this.transform((x, y, z) => [x + dx, y + dy, z + dz]); }
  rotZ(a) { const c = Math.cos(a), s = Math.sin(a); return this.transform((x, y, z) => [c * x - s * y, s * x + c * y, z]); }
  rotX(a) { const c = Math.cos(a), s = Math.sin(a); return this.transform((x, y, z) => [x, c * y - s * z, s * y + c * z]); }
  rotY(a) { const c = Math.cos(a), s = Math.sin(a); return this.transform((x, y, z) => [c * x + s * z, y, -s * x + c * z]); }
  get triangleCount() { return this.i.length / 3; }
  bbox() { const b = [Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity]; for (let k = 0; k < this.p.length; k += 3) for (let d = 0; d < 3; d++) { b[d] = Math.min(b[d], this.p[k + d]); b[d + 3] = Math.max(b[d + 3], this.p[k + d]); } return b; }
  volume() { let V = 0; const P = this.p; for (let k = 0; k < this.i.length; k += 3) { const a = this.i[k] * 3, b = this.i[k + 1] * 3, c = this.i[k + 2] * 3; V += (P[a] * (P[b + 1] * P[c + 2] - P[b + 2] * P[c + 1]) - P[a + 1] * (P[b] * P[c + 2] - P[b + 2] * P[c]) + P[a + 2] * (P[b] * P[c + 1] - P[b + 1] * P[c])) / 6; } return V; }
  area() { let A = 0; const P = this.p; for (let k = 0; k < this.i.length; k += 3) { const a = this.i[k] * 3, b = this.i[k + 1] * 3, c = this.i[k + 2] * 3; const ux = P[b] - P[a], uy = P[b + 1] - P[a + 1], uz = P[b + 2] - P[a + 2], vx = P[c] - P[a], vy = P[c + 1] - P[a + 1], vz = P[c + 2] - P[a + 2]; A += 0.5 * Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx); } return A; }
  centroid() { let V = 0, cx = 0, cy = 0, cz = 0; const P = this.p; for (let k = 0; k < this.i.length; k += 3) { const a = this.i[k] * 3, b = this.i[k + 1] * 3, c = this.i[k + 2] * 3; const v = (P[a] * (P[b + 1] * P[c + 2] - P[b + 2] * P[c + 1]) - P[a + 1] * (P[b] * P[c + 2] - P[b + 2] * P[c]) + P[a + 2] * (P[b] * P[c + 1] - P[b + 1] * P[c])) / 6; V += v; cx += v * (P[a] + P[b] + P[c]) / 4; cy += v * (P[a + 1] + P[b + 1] + P[c + 1]) / 4; cz += v * (P[a + 2] + P[b + 2] + P[c + 2]) / 4; } return V ? [cx / V, cy / V, cz / V] : [0, 0, 0]; }
  // Manifold check: every edge shared by exactly two triangles with opposite orientation
  checkManifold() { const E = new Map(); for (let k = 0; k < this.i.length; k += 3) for (let e = 0; e < 3; e++) { const a = this.i[k + e], b = this.i[k + (e + 1) % 3]; const key = a < b ? a + '_' + b : b + '_' + a; const s = E.get(key) || { n: 0, dir: 0 }; s.n++; s.dir += a < b ? 1 : -1; E.set(key, s); } let open = 0, nonman = 0, flipped = 0; for (const s of E.values()) { if (s.n === 1) open++; else if (s.n > 2) nonman++; else if (s.dir !== 0) flipped++; } return { edges: E.size, open, nonManifold: nonman, inconsistent: flipped, closed: open === 0 && nonman === 0, oriented: flipped === 0 }; }
}

// Loft through a list of closed sections (each an array of [x,y,z]) with end caps
export function loft(name, sections, opts) {
  const m = new Mesh(name, opts), ns = sections.length, np = sections[0].length;
  const idx = sections.map(sec => sec.map(([x, y, z]) => m.v(x, y, z)));
  for (let s = 0; s < ns - 1; s++) for (let k = 0; k < np; k++) { const k1 = (k + 1) % np; m.q(idx[s][k], idx[s][k1], idx[s + 1][k1], idx[s + 1][k]); }
  const cap = (sec, rev) => { const c = sec.reduce((a, p) => [a[0] + p[0] / np, a[1] + p[1] / np, a[2] + p[2] / np], [0, 0, 0]); const ci = m.v(...c); const ids = rev ? idx[sec === sections[0] ? 0 : ns - 1] : idx[sec === sections[0] ? 0 : ns - 1]; for (let k = 0; k < np; k++) rev ? m.t(ci, ids[(k + 1) % np], ids[k]) : m.t(ci, ids[k], ids[(k + 1) % np]); };
  cap(sections[0], true); cap(sections[ns - 1], false);
  fixOrientation(m); return m;
}
// Ensure outward orientation using signed volume
export function fixOrientation(m) { if (m.volume() < 0) for (let k = 0; k < m.i.length; k += 3) { const t = m.i[k + 1]; m.i[k + 1] = m.i[k + 2]; m.i[k + 2] = t; } return m; }
export function box(name, lx, ly, lz, opts) { const s = (z) => [[-lx / 2, -ly / 2, z], [lx / 2, -ly / 2, z], [lx / 2, ly / 2, z], [-lx / 2, ly / 2, z]]; return loft(name, [s(-lz / 2), s(lz / 2)], opts); }
export function cylinder(name, r, len, n = 40, opts) { const s = (z) => range(n).map(k => [r * Math.cos(2 * Math.PI * k / n), r * Math.sin(2 * Math.PI * k / n), z]); return loft(name, [s(-len / 2), s(len / 2)], opts); }
export function tube(name, ro, ri, len, n = 48, opts) { // annulus extruded along z
  const m = new Mesh(name, opts); const ring = (r, z) => range(n).map(k => m.v(r * Math.cos(2 * Math.PI * k / n), r * Math.sin(2 * Math.PI * k / n), z));
  const o0 = ring(ro, -len / 2), o1 = ring(ro, len / 2), i0 = ring(ri, -len / 2), i1 = ring(ri, len / 2);
  for (let k = 0; k < n; k++) { const k1 = (k + 1) % n; m.q(o0[k], o0[k1], o1[k1], o1[k]); m.q(i0[k1], i0[k], i1[k], i1[k1]); m.q(o1[k], o1[k1], i1[k1], i1[k]); m.q(i0[k], i0[k1], o0[k1], o0[k]); }
  return fixOrientation(m);
}
export function torus(name, R, r, nu = 48, nv = 20, opts) {
  const m = new Mesh(name, opts), id = [];
  for (let i = 0; i < nu; i++) { id.push([]); const u = 2 * Math.PI * i / nu; for (let j = 0; j < nv; j++) { const v = 2 * Math.PI * j / nv; id[i].push(m.v((R + r * Math.cos(v)) * Math.cos(u), (R + r * Math.cos(v)) * Math.sin(u), r * Math.sin(v))); } }
  for (let i = 0; i < nu; i++) for (let j = 0; j < nv; j++) m.q(id[i][j], id[(i + 1) % nu][j], id[(i + 1) % nu][(j + 1) % nv], id[i][(j + 1) % nv]);
  return fixOrientation(m);
}
export function sphere(name, r, nu = 32, nv = 16, opts) { const secs = []; for (let j = 1; j < nv; j++) { const ph = Math.PI * j / nv; secs.push(range(nu).map(i => [r * Math.sin(ph) * Math.cos(2 * Math.PI * i / nu), r * Math.sin(ph) * Math.sin(2 * Math.PI * i / nu), -r * Math.cos(ph)])); } return loft(name, secs, opts); }
// Superellipse closed section in the y–z plane at station x
const superEllipse = (x, a, b, cz, n = 2.5, np = 40) => range(np).map(k => { const t = 2 * Math.PI * k / np, c = Math.cos(t), s = Math.sin(t); return [x, a * Math.sign(c) * Math.abs(c) ** (2 / n), cz + b * Math.sign(s) * Math.abs(s) ** (2 / n)]; });

// Wing built from airfoil sections along its span (vertical, +z), with linear chord taper and twist (deg) distribution
export function wing(name, { airfoil = naca4('0018', 61), span = 3, rootChord = 1.6, tipChord = 0.9, twistTip = -4, sweepLE = 0, nSpan = 14, twistFn, cambDelta, opts, tipRound = true } = {}) {
  const sections = [];
  const sec = (eta, c, z, thick = 1) => { const tw = rad(twistFn ? twistFn(Math.min(1, eta)) : twistTip * Math.min(1, eta)), xle = Math.tan(rad(sweepLE)) * Math.min(z, span), cq = rootChord + (tipChord - rootChord) * Math.min(1, eta); return airfoil.points.map(([x, y]) => { const yy = (y + (cambDelta ? cambDelta(Math.min(1, eta), x) : 0)) * thick; const xc = (x - 0.25) * c, yc = yy * c; return [xle + 0.25 * cq + xc * Math.cos(tw) - yc * Math.sin(tw), xc * Math.sin(tw) + yc * Math.cos(tw), z]; }); };
  for (let s = 0; s <= nSpan; s++) { const eta = s / nSpan; sections.push(sec(eta, rootChord + (tipChord - rootChord) * eta, eta * span)); }
  // rounded (half-elliptic) tip cap: reduces tip-vortex strength and gives a clean finished look
  if (tipRound) for (const f of [0.35, 0.6, 0.8, 0.93]) { const k = Math.sqrt(1 - f * f); sections.push(sec(1, tipChord * (0.25 + 0.75 * k), span + f * 0.09 * tipChord, Math.max(0.15, k))); }
  // drop duplicated closing point if present
  return loft(name, sections, opts);
}

// ---------------- Whole-vehicle assembly ----------------
export const PART_COLORS = { Composite: '#2b2f36', Metal: '#b8c0cc', SMA: '#c58b4b', Polymer: '#5a7fa8', Elastomer: '#1b1b1b', Coating: '#1e293b', Ceramic: '#d6d3d1', Thermal: '#a3a3a3', Core: '#e2c275', Friction: '#7c5c3c', 'Polymer actuator': '#a16207' };
// Signature livery (Revision 2.2 styling): pearl-white lifting body over a graphite aero underbody, satin-silver wings with
// cobalt leading edges, smoked canopy. 'material' mode colours every part by its assigned material family instead.
export const LIVERY = { chassis: '#e9eef5', underbody: '#2a2f38', wingSkin: '#c9d6e6', spar: '#1f2937', rootInsert: '#6b7c93', spat: '#2a2f38', cabin: '#3b6a8f', accent: '#1d4ed8', collector: '#14254a', radiator: '#f8fafc' };
export function vehicleAssembly(veh = DEFAULT_VEHICLE, { wingIncidence, airfoil, twistTip = -4, morph = null, wheelAngle = 0, steer = 0, detail = 1, materials = {}, reefed = false, folded = false, finish = 'livery' } = {}) {
  const L = veh.length, W = veh.width, Hb = veh.height, wb = veh.wheelbase, tr = veh.track, R = veh.wheelRadius, parts = [];
  wingIncidence = wingIncidence ?? ((veh.wingConfig || 'tandem') === 'arch' ? (veh.archIncidence ?? 0) : 10); // Rev 5.1: arch default incidence 0°
  const color = (comp, def) => finish === 'livery' ? (LIVERY[comp] || def) : (PART_COLORS[materials[comp]?.family] || def);
  // monocoque body: x forward from rear (0) to nose (L); bodySec(u) is the analytic section law reused for conformal parts
  const nb = 22 * detail, secs = [];
  // Rev 5.1: with veh.groundClearance the floor is flat at that height (velomobile-style, wheels outboard / in the tail slot);
  // without it the body keeps the earlier placement on the wheel-centre line
  const gClr = veh.groundClearance ?? null;
  // Lifting body: the plan-form is a symmetric NACA-type airfoil (rounded nose at u = 1, sharp Kutta trailing edge at the
  // tail, u = 0) so the body itself produces aerodynamic side force/thrust and acts as an end-plate for the wing roots.
  const tPlan = W / L, yt = (xn) => 5 * tPlan * (0.2969 * Math.sqrt(Math.max(0, xn)) - 0.126 * xn - 0.3516 * xn * xn + 0.2843 * xn ** 3 - 0.1036 * xn ** 4), ytMax = yt(0.3);
  const bodySec = (u) => { const x = u * L - L * 0.42, xn = 1 - u; const halfW = W / 2 * Math.max(0.012, yt(xn) / ytMax); const g = Math.max(u < 0.3 ? 0.45 : 0.06, Math.sin(Math.PI * Math.min(1, 0.06 + 0.92 * u)) ** 0.6); const halfH = Hb * 0.36 * g; return { x, a: Math.max(0.01, halfW * 0.86), b: Math.max(0.02, halfH), cz: gClr != null ? gClr + Math.max(0.02, halfH) : R + 0.02 + halfH * 0.9, ...(gClr != null && (veh.diffuserRise ?? 0) > 0 && u < 0.35 ? (() => { const rz = veh.diffuserRise * ((0.35 - u) / 0.35) ** 1.5, b0 = Math.max(0.02, halfH); return { b: Math.max(0.02, b0 - rz / 2), cz: gClr + b0 + rz / 2 }; })() : {}) }; };
  const uOf = (x) => Math.min(1, Math.max(0, (x + L * 0.42) / L));
  const topZ = (x) => { const q = bodySec(uOf(x)); return q.cz + q.b; };
  for (let s = 0; s <= nb; s++) { const q = bodySec(s / nb); secs.push(superEllipse(q.x, q.a, q.b, q.cz, 2.6, 36)); }
  parts.push(loft('Monocoque body', secs, { color: color('chassis', '#2b2f36'), group: 'Chassis', component: 'chassis' }));
  { const sk = []; for (let s2 = 0; s2 <= nb; s2++) { const q = bodySec(s2 / nb); sk.push(range(16).map(j => { const t = Math.PI + Math.PI * j / 15, c = Math.cos(t), sn = Math.sin(t); return [q.x, q.a * 1.006 * Math.sign(c) * Math.abs(c) ** (2 / 2.6), q.cz + q.b * 1.006 * Math.sign(sn) * Math.abs(sn) ** (2 / 2.6) * 0.98]; }).concat(range(16).map(j => { const t = 2 * Math.PI - Math.PI * j / 15, c = Math.cos(t), sn = Math.sin(t); return [q.x, q.a * 0.99 * Math.sign(c) * Math.abs(c) ** (2 / 2.6), q.cz + q.b * 0.99 * Math.sign(sn) * Math.abs(sn) ** (2 / 2.6)]; }))); }
    parts.push(loft('Aero underbody (graphite)', sk, { color: color('underbody', '#2a2f38'), group: 'Aero fairings', component: 'chassis' })); }
  // canopy
  // cockpit forward between/behind the twin front wheels so the wings sit aft of the driver (clear forward view)
  // teardrop glasshouse: long, low canopy whose tail flows back into the arch root (fuller at the front, tapering aft)
  const cockX = [L * 0.58 - 2.0, L * 0.58 - 0.55], canX = [cockX[0], cockX[1] - (veh.canopyFwdTrim ?? 0)]; // canopy may stop short of the foot well (opaque deck → collector)
  // body/canopy probes for occupant packaging checks (Suite 01): outer surfaces at station x
  const canopyTop = (x) => { const u = (x - canX[0]) / (canX[1] - canX[0]); if (u < 0 || u > 1) return topZ(x); const f = Math.sin(Math.PI * Math.min(1, 0.15 + 0.85 * u) ** 1.35), b = Hb * 0.27 * f ** 0.75 + 0.01; return topZ(x) + 0.45 * b; };
  const bodyProbe = { top: topZ, canopyTop, bottom: (x) => { const q = bodySec(uOf(x)); return q.cz - q.b; }, halfWidth: (x) => bodySec(uOf(x)).a,
    // frontal area: largest body super-ellipse section (n = 2.6 → area ≈ 3.37·a·b) plus the canopy bulge above the body top line
    frontalArea: Math.max(...range(41).map(k => { const q = bodySec(k / 40); return 3.37 * q.a * q.b; })) + Math.max(...range(15).map(k => { const x = canX[0] + k / 14 * (canX[1] - canX[0]); return 0.6 * 2 * (W * 0.27) * (canopyTop(x) - topZ(x)); })) }; const cs = []; for (let s = 0; s <= 14; s++) { const u = s / 14, x = canX[0] + u * (canX[1] - canX[0]), f = Math.sin(Math.PI * Math.min(1, 0.15 + 0.85 * u) ** 1.35); const a = W * 0.27 * f ** 0.55 + 0.01, b = Hb * 0.27 * f ** 0.75 + 0.01, zc = topZ(x) - b * 0.55; cs.push(superEllipse(x, a, b, zc, 2.2, 32)); }
  parts.push(loft('Cabin canopy', cs, { color: color('cabin', '#7dd3fc'), opacity: 0.45, group: 'Cabin', component: 'cabin' }));
  // wheels: tadpole (2 front, 1 rear) or delta (1 front, 2 rear)
  // axleShift moves the whole wheel set along the body (Rev 5.1: places the front axle relative to the CG for SSF vs nose-over)
  const front = wb * 0.58 + (veh.axleShift ?? 0), rear = -wb * 0.42 + (veh.axleShift ?? 0); const tadpole = (veh.layout || 'tadpole') === 'tadpole';
  const wheelPos = tadpole ? [[front, tr / 2, true], [front, -tr / 2, true], [rear, 0, false]] : [[front, 0, true], [rear, tr / 2, false], [rear, -tr / 2, false]];
  wheelPos.forEach(([x, y, steerable], k) => {
    const tr2 = (veh.tyreWidth ?? 50) / 2000; // tyre section radius (Suite 10 auto-sizes the width to the load rating)
    const tyre = torus(`Tyre ${k + 1}`, R - tr2, tr2, 48, 16, { color: color('tyre', '#111'), group: 'Wheels', component: 'tyre' });
    const rim = tube(`Rim ${k + 1}`, R - 2 * tr2 + 0.01, R - 2 * tr2 - 0.02, 0.05, 48, { color: color('wheel', '#c0c6cf'), group: 'Wheels', component: 'wheel' });
    const driven = k < (veh.driveWheels ?? 2), hubR = R * 0.45; const hub = driven ? tube(`Direct-rim hub ring ${k + 1}`, hubR, hubR * 0.62, 0.07, 48, { color: color('actuator', '#c58b4b'), group: 'Hub drive', component: 'actuator' }) : null;
    const disc = tube(`Brake disc ${k + 1} (220 × 4 mm)`, 0.11, 0.065, 0.004, 40, { color: color('brake', '#8b8f96'), group: 'Brakes', component: 'brake' });
    const wparts = [tyre, rim, hub, disc].filter(Boolean);
    for (const m of wparts) { m.rotX(Math.PI / 2); m.rotY(wheelAngle); }
    for (const m of wparts) { m.transform((px, py, pz) => { const a = steerable ? rad(steer) : 0, c = Math.cos(a), s = Math.sin(a); return [c * px - s * py, s * px + c * py, pz]; }); m.translate(x, y + (m === disc ? Math.sign(y || 1) * -0.05 : 0), R); parts.push(m); }
    // spokes as thin boxes
    for (let sp = 0; sp < 6; sp++) { const b = box(`Spoke ${k + 1}.${sp + 1}`, R * 0.5, 0.012, 0.03, { color: color('wheel', '#c0c6cf'), group: 'Wheels', component: 'wheel' }); b.translate(R * 0.5, 0, 0); b.rotY(sp * Math.PI / 3 + wheelAngle); if (steerable && steer) b.rotZ(rad(steer)); b.translate(x, y, R); parts.push(b); }
  });
  // aerodynamic wheel spats (front wheels steer with their spats; rear wheel is enclosed by the tail)
  const axleInfo = [];
  wheelPos.forEach(([x, y, steerable], k) => {
    if (!tadpole && !steerable) return; if (tadpole && !steerable) return;
    const secs2 = []; for (let s2 = 0; s2 <= 12; s2++) { const u = s2 / 12, xx = (u - 0.45) * 2.6 * R, wHalf = 0.075 * Math.max(0.05, Math.sin(Math.PI * u) ** 0.6), hHalf = (R + 0.08) * Math.max(0.12, Math.sin(Math.PI * u) ** 0.45); secs2.push(range(20).map(j => { const t = 2 * Math.PI * j / 20; return [xx, wHalf * Math.cos(t), R * 0.95 + hHalf * Math.sin(t) * (Math.sin(t) > 0 ? 1 : 0.35)]; })); }
    const spat = loft(`Wheel spat ${k + 1}`, secs2, { color: color('spat', '#2a2f38'), group: 'Aero fairings', component: 'chassis' });
    spat.translate(0, 0, -R); spat.transform((px, py, pz) => { const a = steerable ? rad(steer) : 0, c = Math.cos(a), sn = Math.sin(a); return [c * px - sn * py, sn * px + c * py, pz]; }); spat.translate(x, y, R); parts.push(spat);
    // Rev 5.1 blended 'shoulder' wheel pod: the lifting body widens locally at the axle station and a streamlined pod
    // (chord shrinking outboard) carries the axle arm out to within 20 mm of the spat inner face — exposed arm ≈ 0.02 m.
    const yb = bodySec(uOf(x)).a * 0.98, ySpatIn = Math.abs(y) - 0.075, yPodEnd = ySpatIn - 0.02, sa = naca4('0030', 25), pod = [];
    for (let s3 = 0; s3 <= 6; s3++) { const e = s3 / 6, yy = Math.sign(y) * (yb - 0.04 + (yPodEnd - yb + 0.04) * e), ch = 0.75 - 0.35 * e, th = 0.9 - 0.25 * e; pod.push(sa.points.map(([px, py]) => [x + 0.3 * ch - px * ch, yy, R + 0.1 + py * ch * th])); }
    parts.push(fixOrientation(loft(`Front shoulder wheel pod ${k + 1} (blended fairing)`, pod, { color: color('chassis', '#e9eef5'), group: 'Aero fairings', component: 'chassis' })));
    axleInfo.push({ yBody: yb, ySpatInner: ySpatIn, yPodEnd, exposedArm: Math.max(0, ySpatIn - yPodEnd), bodyHalfWidth: bodySec(uOf(x)).a });
  });
  // AeroArch (Revision 5.1): ONE closed box wing — two airfoil legs joined by a top bridge — whose legs stand on a single
  // streamlined CFRP root crossbeam 20–30 mm above the deck. The crossbeam sits on ONE centreline titanium flexure pylon over a
  // transverse root frame tied to the keel: the whole arch yaws elastically ±15° about the pylon axis (no bearings, no sliding,
  // no drives) and self-trims; the bridge top carries a solar collector.
  const arch = (veh.wingConfig || 'tandem') === 'arch', af = airfoil || naca4(arch ? (veh.archNaca ?? '0012') : '0018', 61), pivot = arch ? 0.30 : 0.22;
  const frame = (name, x) => { const q = bodySec(uOf(x)), m = new Mesh(name, { color: color('chassis', '#3b4250'), group: 'Chassis', component: 'chassis' }); const N = 36; const ring = (sc, xx) => range(N).map(k => { const t = 2 * Math.PI * k / N, c = Math.cos(t), sn = Math.sin(t); return m.v(xx, q.a * sc * Math.sign(c) * Math.abs(c) ** (2 / 2.6), q.cz + q.b * sc * Math.sign(sn) * Math.abs(sn) ** (2 / 2.6)); }); const o0 = ring(0.97, x - 0.02), o1 = ring(0.97, x + 0.02), i0 = ring(0.8, x - 0.02), i1 = ring(0.8, x + 0.02); for (let k = 0; k < N; k++) { const k1 = (k + 1) % N; m.q(o0[k], o0[k1], o1[k1], o1[k]); m.q(i0[k1], i0[k], i1[k], i1[k1]); m.q(o1[k], o1[k1], i1[k1], i1[k]); m.q(i0[k], i0[k1], o0[k1], o0[k]); } return fixOrientation(m); };
  let wingX, LE, cr, ct, span, areaFitted, reefSpan, spanDep, archMeta = null;
  if (arch) {
    const zDeck0 = topZ(cockX[0] + 0.05 - (veh.archAft ?? 0.25)), zBeam = zDeck0 + (veh.rootBootH > 0 ? veh.rootBootH : 0.025) + 0.05, cosC = Math.cos(rad(veh.archCant ?? 12)), trA = veh.archTaper ?? 0.6;
    // legs lowered (chord grown to keep the area) so that the overall height incl. crossbeam, bridge camber and bridge thickness ≤ 3.0 m
    // bridge: NACA 00xx with t/c ≥ 0.15 and ≥ 80 mm thick so the CFRP Ø70 roll-over tube fits inside (Suite 09)
    const afTC = 2 * Math.max(...af.points.map(q => Math.abs(q[1]))), bridgeTC = (cb0) => Math.max(0.15, 0.08 / cb0), bridgeHalf = (cb0) => bridgeTC(cb0) * cb0 / 2;
    let hLeg = veh.wingHeight; for (let it = 0; it < 6; it++) { const ctp = (veh.wingArea / hLeg) / (1 + trA) * trA, top = zBeam + hLeg * cosC + 0.16 + bridgeHalf(ctp); if (top <= (veh.maxHeight ?? 2.995)) break; hLeg = Math.min(hLeg, ((veh.maxHeight ?? 2.995) - zBeam - 0.16 - bridgeHalf(ctp)) / cosC); }
    const cm = veh.wingArea / (2 * hLeg), tr = trA, c = cm * 2 / (1 + tr), ctip = c * tr, sweep = rad(veh.archSweep ?? 20), cant = rad(veh.archCant ?? 12), xa = cockX[0] + 0.05 - pivot * cm * 1.2 * 0.2 - (veh.archAft ?? 0.25), q = bodySec(uOf(xa));
    // an explicit archGap may exceed the body (crossbeam overhangs the sides), limited to 1.9 m so the arch stays ≤ 2.0 m wide
    const g = veh.archGap != null ? Math.min(veh.archGap, 1.9) : 2 * q.a * 0.9, ys = g / 2, zRoot = zBeam; // legs stand on the crossbeam top
    const yTip = ys - hLeg * Math.sin(cant), zTop = zRoot + hLeg * Math.cos(cant), fold = rad(folded ? 78 : 0), inc = rad(wingIncidence);
    const A = [];
    for (const sd of [1, -1]) {
      const w = wing(`Arch leg ${sd > 0 ? 'port' : 'starboard'} (skin + lattice)`, { airfoil: af, span: hLeg + 0.06, rootChord: c, tipChord: ctip, sweepLE: deg(sweep), twistTip: twistTip * 0.5, twistFn: morph?.twist, cambDelta: morph?.camber, tipRound: false, opts: { color: color('wingSkin', '#c9d6e6'), group: 'Wings', component: 'wingSkin' } });
      w.translate(-pivot * c, 0, 0); w.transform((px, py, pz) => [-px, py, pz]); fixOrientation(w); w.rotX(sd * cant); w.translate(xa, sd * ys, zRoot); A.push(w);
      const sp = box(`Arch leg ${sd > 0 ? 'port' : 'starboard'} spar`, ctip * 0.14, ctip * 0.09, hLeg * 0.98 / Math.cos(sweep), { color: color('spar', '#1f2937'), group: 'Wings', component: 'spar' }); sp.translate(0, 0, hLeg / 2); sp.rotY(-sweep); sp.translate(-(0.42 - pivot) * c * 0.3, 0, 0); sp.rotX(sd * cant); sp.translate(xa, sd * ys, zRoot); A.push(sp);
      const fr = []; for (let s2 = 0; s2 <= 4; s2++) { const e = s2 / 4, sc = 1.18 - 0.18 * Math.sqrt(e), zz = zRoot - 0.05 + e * 0.13; fr.push(af.points.map(([px, py]) => [xa - (px - pivot) * c * sc, sd * ys + py * c * sc * 1.15, zz])); }
      { const le = []; for (let s2 = 0; s2 <= 10; s2++) { const e = s2 / 10, z = e * hLeg, ch = c + (ctip - c) * e, xle = xa + pivot * c - Math.tan(sweep) * z + 0.004, y0 = sd * (ys - z * Math.sin(cant)); le.push([[xle, y0 - 0.02, zRoot + z * Math.cos(cant)], [xle + 0.012, y0, zRoot + z * Math.cos(cant)], [xle, y0 + 0.02, zRoot + z * Math.cos(cant)], [xle - 0.05 * ch, y0, zRoot + z * Math.cos(cant)]]); } A.push(loft(`Arch leg ${sd > 0 ? 'port' : 'starboard'} leading-edge accent`, le, { color: color('accent', '#1d4ed8'), group: 'Aero fairings', component: 'wingSkin' })); }
      A.push(loft(`Arch leg ${sd > 0 ? 'port' : 'starboard'} root fairing`, fr, { color: color('wingSkin', '#c9d6e6'), group: 'Aero fairings', component: 'wingSkin' }));
    }
    // single streamlined CFRP root crossbeam joining both leg roots, 20–30 mm above the deck, and the centreline flexure pylon
    { const bc = c * 0.55, sb = naca4('0018', 31), cbm = []; for (const yy of [-ys - 0.04, ys + 0.04]) cbm.push(sb.points.map(([px, py]) => [xa + pivot * c * 0.6 - px * bc, yy, zBeam - 0.025 + py * bc * 0.6])); A.push(fixOrientation(loft('Arch root crossbeam (streamlined CFRP)', cbm, { color: color('spar', '#1f2937'), group: 'Wings', component: 'spar' })));
      // root endplate (rootEndplate): thin sandwich plate at the leg roots that yaws with the arch; its brush seal (deckSeal) closes the gap to the deck
      if (veh.rootEndplate) { const ep = box('Arch root endplate (yaws with the arch)', 1.1 * c, 2 * (ys + 0.12), 0.008, { color: color('spar', '#1f2937'), group: 'Wings', component: 'spar' }); ep.translate(xa + pivot * c - 0.55 * c, 0, zBeam - 0.03); A.push(ep); }
      // brush skirt (deckSeal) sealing the crossbeam/endplate-to-deck gap; flexible bristles let the arch yaw
      if ((veh.deckSeal ?? 0) > 0) { const br2 = box('Arch crossbeam brush skirt (deck seal)', bc * 0.7, 2 * (ys + 0.04), 0.022, { color: color('chassis', '#3b4250'), group: 'Wings', component: 'chassis' }); br2.translate(xa + pivot * c * 0.6 - 0.5 * bc, 0, zDeck0 + 0.011); A.push(br2); } }
    // centreline pylon = torsion rod cage (s09 'cage'): 6 × β-21S Ti rods Ø10 × 550 mm on a 105 mm pitch radius, potted into a plate
    // under the crossbeam and hanging down into the body to a base plate on the root frame; a pre-buckled bistable strip pair cancels
    // most of the cage stiffness, and elastomer-faced arc landings under the crossbeam take roll-over moments (1 mm gap)
    { const zPt = zDeck0 + 0.02, Lr = 0.55, zPb = zPt - Lr, rp = 0.105, fx = { color: color('accent', '#1d4ed8'), group: 'Wings', component: 'flexure' }, fxB = { ...fx, group: 'Chassis' };
      const put = (mesh, x0, y0, z0) => { mesh.transform((px, py, pz) => [px + x0, py + y0, pz + z0]); return mesh; };
      A.push(put(cylinder('Arch pylon top plate (potted under the crossbeam)', 0.14, 0.012, 36, fx), xa, 0, zPt));
      for (let k = 0; k < 6; k++) { const t = 2 * Math.PI * k / 6; A.push(put(cylinder(`Arch pylon Ti torsion rod ${k + 1} (Ø10 × 550 mm)`, 0.005, Lr, 12, fx), xa + rp * Math.cos(t), rp * Math.sin(t), zPt - Lr / 2)); }
      parts.push(put(cylinder('Arch pylon base plate (4 Ti inserts on the root frame)', 0.24, 0.015, 36, fxB), xa, 0, zPb));
      parts.push(put(box('Arch pylon bistable compensator strips', 0.04, 0.2, 0.3, fxB), xa, 0, zPb + 0.17));
      for (const sd of [1, -1]) parts.push(put(box(`Arch roll-over arc landing ${sd > 0 ? 'port' : 'starboard'} (elastomer-faced)`, 0.14, 0.09, 0.022, { color: color('chassis', '#3b4250'), group: 'Chassis', component: 'chassis' }), xa, sd * 0.38, zDeck0 + 0.011)); }
    // top bridge: horizontal airfoil-section beam with a gentle upward camber line (closes the box wing: no tip vortices)
    const cb = ctip, xb = xa + pivot * c - Math.tan(sweep) * hLeg * Math.cos(cant) - pivot * cb, br = [], nb2 = 18, rC = 0.22;
    for (let s2 = 0; s2 <= nb2; s2++) { const u = s2 / nb2, y = (-1 + 2 * u) * (yTip + 0.02), e = Math.abs(y) / (yTip + 0.02), drop = e > 1 - rC / yTip ? rC * (1 - Math.cos(Math.PI / 2 * (e - (1 - rC / yTip)) / (rC / yTip))) : 0, rise = 0.16 * (1 - e * e); br.push(af.points.map(([px, py]) => [xb - (px - pivot) * cb, y * (1 - 0.04 * drop / rC), zTop + rise - drop + py * cb * bridgeTC(cb) / afTC])); }
    A.push(loft('Arch top bridge (closes the box wing)', br, { color: color('wingSkin', '#c9d6e6'), group: 'Wings', component: 'wingSkin' }));
    const tmax = bridgeHalf(cb), cs2 = []; const fy = veh.fullBridgeCollector ? 0.95 : 0.72, fxF = veh.fullBridgeCollector ? 0.3 : 0.2, fxA = veh.fullBridgeCollector ? 0.6 : 0.5; // Rev 5.1: absorber over the whole bridge top
    for (let s2 = 0; s2 <= 12; s2++) { const y = -yTip * fy + 2 * fy * yTip * s2 / 12, e = Math.abs(y) / (yTip + 0.02), rise = 0.16 * (1 - e * e), z0 = zTop + rise + tmax + 0.003; cs2.push([[xb + fxF * cb, y, z0], [xb - fxA * cb, y, z0], [xb - fxA * cb, y, z0 + 0.008], [xb + fxF * cb, y, z0 + 0.008]]); }
    A.push(loft('Solar-selective collector (arch bridge top)', cs2, { color: color('collector', '#14254a'), group: 'Thermal', component: 'collector' }));
    for (const m of A) { /* arch, crossbeam and pylon rotate together about the centreline pylon axis */ m.transform((px, py, pz) => { const dx = px - xa, dz = pz - zRoot; return [xa + dx * Math.cos(fold) - dz * Math.sin(fold), py, zRoot + dx * Math.sin(fold) + dz * Math.cos(fold)]; }); m.transform((px, py, pz) => { const dx = px - xa; return [xa + dx * Math.cos(inc) - py * Math.sin(inc), dx * Math.sin(inc) + py * Math.cos(inc), pz]; }); parts.push(m); }
    parts.push(frame('Arch transverse root frame (pylon base, tied to keel)', xa), frame('Arch fore bulkhead', xa + 0.35 * c), frame('Arch aft bulkhead', xa - 0.6 * c));
    wingX = [xa]; LE = [xa + pivot * c]; cr = c; ct = ctip; span = hLeg; spanDep = hLeg; reefSpan = hLeg; areaFitted = 2 * hLeg * (c + ct) / 2;
    archMeta = { gap: g, cant: veh.archCant ?? 6, legHeight: hLeg, chord: c, bridgeSpan: 2 * yTip, bridgeArea: 2 * yTip * cb, collectorArea: 2 * (veh.fullBridgeCollector ? 0.95 : 0.72) * yTip * (veh.fullBridgeCollector ? 0.9 : 0.7) * cb, zRoot, zTop: zTop + 0.16 + tmax, sweep: deg(sweep), rootChord: c, tipChord: ctip, folded, foldedHeight: zRoot + c * Math.sin(rad(78)) + 0.15 * c };
  } else {
    // Revision 2.x tandem layout (kept as an option)
  const nW = veh.nWings || 2; spanDep = veh.wingHeight; reefSpan = Math.min(spanDep, veh.wingReefSpan || spanDep); span = reefed ? reefSpan : spanDep; const cMean = veh.wingArea / nW / spanDep;
  const xStart = cockX[0] - 0.05, xEnd = -L * 0.42 + 0.09 * L, avail = xStart - xEnd;
  cr = Math.min(cMean * 1.1, avail / (nW + 0.3 * (nW - 1))); ct = cr * 0.82; areaFitted = nW * span * (cr + ct) / 2;
  LE = Array.from({ length: nW }, (_, k) => xStart - k * 1.3 * cr); wingX = LE.map(x => x - pivot * cr);
  wingX.forEach((wx, k) => {
    const zr = topZ(wx) - 0.02;
    const fr0 = span / spanDep, w = wing(`Wing ${k + 1} (skin + lattice)`, { airfoil: af, span, rootChord: cr, tipChord: cr + (ct - cr) * fr0, twistTip: twistTip * fr0, twistFn: morph?.twist, cambDelta: morph?.camber, opts: { color: color('wingSkin', '#e5e7eb'), group: 'Wings', component: 'wingSkin' } });
    // wing() builds LE at x = 0 pointing −x; mirror so the leading edge faces forward (+x) and put the pivot at 22 % chord
    w.translate(-pivot * cr, 0, 0); w.transform((px, py, pz) => [-px, py, pz]); fixOrientation(w); w.rotZ(rad(wingIncidence)); w.translate(wx, 0, zr); parts.push(w);
    const zf = bodySec(uOf(wx)).cz - bodySec(uOf(wx)).b * 0.85, sl = span - 0.04 + (zr - zf);
    const spar = box(`Wing ${k + 1} box spar / mast`, cr * 0.14, cr * 0.1, sl, { color: color('spar', '#111827'), group: 'Wings', component: 'spar' }); spar.translate(0, 0, sl / 2); spar.rotZ(rad(wingIncidence)); spar.translate(wx, 0, zf); parts.push(spar);
    const root = cylinder(`Wing ${k + 1} feathering pivot & breakaway lock`, 0.08, 0.16, 24, { color: color('rootInsert', '#9ca3af'), group: 'Wings', component: 'rootInsert' }); root.translate(wx, 0, zr); parts.push(root);
    parts.push(frame(`Wing ${k + 1} mast-step bulkhead`, wx));
    const fr = []; for (let s2 = 0; s2 <= 4; s2++) { const e = s2 / 4, sc = 1.16 - 0.16 * Math.sqrt(e), zz = zr - 0.05 + e * 0.12; fr.push(af.points.map(([px, py]) => { const xc = -(px - pivot) * cr * sc, yc = py * cr * sc * 1.15, a2 = rad(wingIncidence); return [wx + xc * Math.cos(a2) - yc * Math.sin(a2), xc * Math.sin(a2) + yc * Math.cos(a2), zz]; })); }
    parts.push(loft(`Wing ${k + 1} root fairing`, fr, { color: color('wingSkin', '#c9d6e6'), group: 'Aero fairings', component: 'wingSkin' }));
    if (reefSpan < spanDep) { const cj = cr + (ct - cr) * (reefSpan / spanDep), band = []; for (const dz of [0, 0.035]) band.push(af.points.map(([px, py]) => { const tw = -rad(twistTip * reefSpan / spanDep), x0 = -(px - pivot) * cj * 1.03, y0 = py * cj * 1.12, xc = x0 * Math.cos(tw) - y0 * Math.sin(tw), yc = x0 * Math.sin(tw) + y0 * Math.cos(tw), a2 = rad(wingIncidence); return [wx + xc * Math.cos(a2) - yc * Math.sin(a2), xc * Math.sin(a2) + yc * Math.cos(a2), zr + reefSpan - 0.015 + dz]; })); parts.push(loft(`Wing ${k + 1} telescoping reef joint`, band, { color: color('accent', '#1d4ed8'), group: 'Wings', component: 'rootInsert' })); }
  });
  }
  // keel beam along the floor tying both mast steps to the front-axle and rear-wheel hard points
  const rearX = Math.min(...wheelPos.map(w => w[0])), xk0 = Math.min(Math.min(...wingX) - 0.25, rearX + 0.12), xk1 = wheelPos[0][0] - 0.1, zk = bodySec(uOf(0)).cz - bodySec(uOf(0)).b + 0.08;
  const keel = box('Keel beam (root-load path to wheel hard points)', xk1 - xk0, 0.12, 0.08, { color: color('chassis', '#3b4250'), group: 'Chassis', component: 'chassis' }); keel.translate((xk0 + xk1) / 2, 0, zk); parts.push(keel);
  // rear fork: two legs, 7075 rectangular tube 40 × 20 × 2 mm, from the keel end down/back to the rear axle
  if (tadpole) for (const sd of [1, -1]) { const x0 = xk0 + 0.02, z0 = zk, x1 = rearX, z1 = R, Lf = Math.hypot(x0 - x1, z0 - z1), f = box(`Rear fork leg ${sd > 0 ? 'port' : 'starboard'} (7075, 40 × 20 × 2 mm)`, Lf, 0.02, 0.04, { color: color('wheel', '#c0c6cf'), group: 'Wheels', component: 'wheel' }); f.rotY(Math.atan2(z0 - z1, x0 - x1)); f.translate((x0 + x1) / 2, sd * 0.045, (z0 + z1) / 2); parts.push(f); }
  // side sills (lower longitudinal members between the axles) and an Al-honeycomb nose crash cone
  for (const sd of [1, -1]) { const xs0 = rearX + 0.3, xs1 = wheelPos[0][0] - 0.15, qm = bodySec(uOf((xs0 + xs1) / 2)), sill = box(`Side sill ${sd > 0 ? 'port' : 'starboard'} (CFRP)`, xs1 - xs0, 0.05, 0.08, { color: color('chassis', '#3b4250'), group: 'Chassis', component: 'chassis' }); sill.translate((xs0 + xs1) / 2, sd * qm.a * 0.82, qm.cz - qm.b * 0.55); parts.push(sill);
    // underbody side skirt (groundSeal): flexible strip closing groundSeal × the floor-to-ground gap along the body side
    if ((veh.groundSeal ?? 0) > 0 || veh.skirtGap != null) { const zb = qm.cz - qm.b, hs = Math.max(0.01, veh.skirtGap != null ? zb - veh.skirtGap : zb * veh.groundSeal), sk = box(`Underbody side skirt ${sd > 0 ? 'port' : 'starboard'} (flexible rCF/elastomer)`, xs1 - xs0, 0.012, hs, { color: color('chassis', '#3b4250'), group: 'Chassis', component: 'chassis' }); sk.translate((xs0 + xs1) / 2, sd * qm.a * 0.95, zb - hs / 2 + 0.01); parts.push(sk); } }
  { const xn1 = L * 0.58 - 0.03, xn0 = xn1 - 0.28, cc = []; for (let s2 = 0; s2 <= 6; s2++) { const u = s2 / 6, x = xn0 + u * (xn1 - xn0), q = bodySec(uOf(x)), sc = 0.7 * (1 - 0.6 * u); cc.push(superEllipse(x, q.a * sc, q.b * sc, q.cz, 2.2, 20)); } parts.push(loft('Nose crash cone (Al honeycomb)', cc, { color: color('chassisCore', '#e2c275'), group: 'Chassis', component: 'chassisCore' })); }
  // conformal sky-facing thermal surfaces: solar collector on the unshaded nose deck, radiative cooler on the rear deck
  const patch = (name, u0, u1, halfAngle, comp, colorDef) => { const sec = []; const N = 14; for (let s2 = 0; s2 <= 10; s2++) { const u = u0 + (u1 - u0) * s2 / 10, q = bodySec(u); const ring = []; for (let k = 0; k <= N; k++) { const t = Math.PI / 2 - halfAngle + 2 * halfAngle * k / N, c = Math.cos(t), sn = Math.sin(t), y = q.a * Math.sign(c) * Math.abs(c) ** (2 / 2.6), z = q.cz + q.b * Math.sign(sn) * Math.abs(sn) ** (2 / 2.6); ring.push([q.x, y * 1.004, q.cz + (z - q.cz) * 1.004 + 0.006]); } for (let k = N; k >= 0; k--) { const r0 = ring[k]; ring.push([r0[0], r0[1] * 0.996, q.cz + (r0[2] - 0.006 - q.cz) / 1.004 * 0.999 + 0.001]); } sec.push(ring); } return loft(name, sec, { color: color(comp, colorDef), group: 'Thermal', component: comp }); };
  const uC1 = uOf(canX[1]) + 0.01;
  parts.push(patch('Solar-selective collector (nose deck, unshaded)', Math.min(uC1, 0.9), 0.96, veh.noseCollectorHalfAngle ?? 0.95, 'collector', '#1e3a8a')); // Rev 5.1 wraps further down the nose flanks
  parts.push(patch('Radiative-cooling emitter (rear deck)', 0.03, Math.max(0.06, uOf(cockX[0]) - 0.03), 0.95, 'radiator', '#f8fafc'));
  // body section under the (first) wing station and the lowest underside point — used by the stacked lifting-line model
  const bodyStation = (() => { const q = bodySec(uOf(wingX[0])); let zb = Infinity, aM = 0, bM = 0, Amax = 0; for (let s3 = 0; s3 <= 40; s3++) { const qq = bodySec(s3 / 40); zb = Math.min(zb, qq.cz - qq.b); if (qq.a * qq.b > Amax) { Amax = qq.a * qq.b; aM = qq.a; bM = qq.b; } } return { zTop: q.cz + q.b, zBot: q.cz - q.b, zBotMin: zb, halfWidth: q.a, aMax: aM, bMax: bM }; })();
  return { parts, meta: { bodyStation, length: L, width: W, height: archMeta ? (folded ? archMeta.foldedHeight : archMeta.zTop) : Math.max(...wingX.map(x => topZ(x))) + span, wheelPos, arch: archMeta, wingConfig: archMeta ? 'arch' : 'tandem', wingX, LE, cr, ct, span, areaFitted, pivotFrac: pivot, cockpitX: cockX, axle: axleInfo[0] ? { ...axleInfo[0], overallWidth: 2 * (Math.max(...wheelPos.map(w => Math.abs(w[1]))) + 0.075) } : null, reefSpan, deployedSpan: spanDep, reefed, collectorArea: null, body: bodyProbe, overallWidthAll: 2 * Math.max(...parts.map(m => { const b = m.bbox(); return Math.max(Math.abs(b[1]), Math.abs(b[4])); })) } };
}

// Direct-rim hub sub-assembly in its own frame (z = axle axis)
export function hubAssembly({ R = 0.30, nCart = 24, hotSector = 150, camAmp = 0.006, theta = 0 } = {}) {
  const parts = [], rc = R * 0.42;
  parts.push(tube('Rim', R - 0.06, R - 0.09, 0.05, 72, { color: '#c0c6cf', group: 'Wheel' }));
  parts.push(torus('Tyre', R - 0.025, 0.025, 72, 18, { color: '#111', group: 'Wheel' }));
  parts.push(tube('Containment ring', rc + 0.035, rc + 0.028, 0.06, 72, { color: '#9ca3af', group: 'Hub' }));
  // stationary cam: radial profile r(φ) = rc - camAmp*cos(φ) built as extruded closed polygon
  const n = 120, cam = new Mesh('Stationary reaction cam', { color: '#64748b', group: 'Hub' }), ring = (z) => range(n).map(k => { const ph = 2 * Math.PI * k / n, r = rc * 0.62 + camAmp * Math.cos(ph); return cam.v(r * Math.cos(ph), r * Math.sin(ph), z); });
  const a = ring(-0.025), b = ring(0.025), ia = range(n).map(k => cam.v(rc * 0.3 * Math.cos(2 * Math.PI * k / n), rc * 0.3 * Math.sin(2 * Math.PI * k / n), -0.025)), ib = range(n).map(k => cam.v(rc * 0.3 * Math.cos(2 * Math.PI * k / n), rc * 0.3 * Math.sin(2 * Math.PI * k / n), 0.025));
  for (let k = 0; k < n; k++) { const k1 = (k + 1) % n; cam.q(a[k], a[k1], b[k1], b[k]); cam.q(ia[k1], ia[k], ib[k], ib[k1]); cam.q(b[k], b[k1], ib[k1], ib[k]); cam.q(ia[k], ia[k1], a[k1], a[k]); }
  parts.push(fixOrientation(cam));
  // thermal sectors (stationary): hot arc and cold arc as thin annular segments
  const sector = (name, a0, a1, color) => { const m = new Mesh(name, { color, group: 'Thermal sectors', opacity: 0.55 }); const N = 40; const pts = (r, z) => range(N + 1).map(k => { const ph = rad(a0 + (a1 - a0) * k / N); return m.v(r * Math.cos(ph), r * Math.sin(ph), z); }); const o0 = pts(rc * 0.95, -0.03), o1 = pts(rc * 0.95, 0.03), i0 = pts(rc * 0.8, -0.03), i1 = pts(rc * 0.8, 0.03); for (let k = 0; k < N; k++) { m.q(o0[k], o0[k + 1], o1[k + 1], o1[k]); m.q(i0[k + 1], i0[k], i1[k], i1[k + 1]); m.q(o1[k], o1[k + 1], i1[k + 1], i1[k]); m.q(i0[k], i0[k + 1], o0[k + 1], o0[k]); } m.q(o0[0], o1[0], i1[0], i0[0]); m.q(o0[N], i0[N], i1[N], o1[N]); return fixOrientation(m); };
  parts.push(sector('Hot thermal sector', 90 - hotSector / 2, 90 + hotSector / 2, '#ef4444'));
  parts.push(sector('Cold thermal sector', 270 - (360 - hotSector) / 2 + 5, 270 + (360 - hotSector) / 2 - 5, '#3b82f6'));
  for (let k = 0; k < nCart; k++) {
    const ph = theta + 2 * Math.PI * k / nCart, c = box(`NiTi cartridge ${k + 1}`, rc * 0.28, Math.min(0.03, 2 * Math.PI * rc / nCart * 0.55), 0.05, { color: '#c58b4b', group: 'Cartridges', cartridge: k });
    c.translate(rc * 0.78, 0, 0); c.rotZ(ph); parts.push(c);
    const shoe = box(`Flexure shoe ${k + 1}`, rc * 0.06, 0.008, 0.04, { color: '#a3e635', group: 'Flexures' }); shoe.translate(rc * 0.62, 0, 0); shoe.rotZ(ph); parts.push(shoe);
  }
  return { parts };
}

// ---------------- Exporters ----------------
function merged(parts) { const out = []; for (const m of parts) { const P = m.p; for (let k = 0; k < m.i.length; k++) out.push(P[m.i[k] * 3], P[m.i[k] * 3 + 1], P[m.i[k] * 3 + 2]); } return Float32Array.from(out); }
export function toSTL(parts, name = 'atmosxp') {
  const tri = merged(parts), n = tri.length / 9, buf = new ArrayBuffer(84 + n * 50), dv = new DataView(buf);
  const hdr = `PHOENIX-ATMOS-XP ${name} units=m`.slice(0, 79); for (let i = 0; i < hdr.length; i++) dv.setUint8(i, hdr.charCodeAt(i));
  dv.setUint32(80, n, true);
  for (let t = 0; t < n; t++) { const o = 84 + t * 50, a = t * 9; const ux = tri[a + 3] - tri[a], uy = tri[a + 4] - tri[a + 1], uz = tri[a + 5] - tri[a + 2], vx = tri[a + 6] - tri[a], vy = tri[a + 7] - tri[a + 1], vz = tri[a + 8] - tri[a + 2]; let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx; const l = Math.hypot(nx, ny, nz) || 1; dv.setFloat32(o, nx / l, true); dv.setFloat32(o + 4, ny / l, true); dv.setFloat32(o + 8, nz / l, true); for (let k = 0; k < 9; k++) dv.setFloat32(o + 12 + k * 4, tri[a + k], true); }
  return new Blob([buf], { type: 'model/stl' });
}
export function toOBJ(parts) { let s = '# PHOENIX-ATMOS-XP OBJ export (units: m)\n', off = 1; for (const m of parts) { s += `o ${m.name.replace(/\s+/g, '_')}\n`; for (let k = 0; k < m.p.length; k += 3) s += `v ${m.p[k].toFixed(6)} ${m.p[k + 1].toFixed(6)} ${m.p[k + 2].toFixed(6)}\n`; for (let k = 0; k < m.i.length; k += 3) s += `f ${m.i[k] + off} ${m.i[k + 1] + off} ${m.i[k + 2] + off}\n`; off += m.p.length / 3; } return s; }
export function toPLY(parts) { const V = [], F = []; let off = 0; for (const m of parts) { for (let k = 0; k < m.p.length; k += 3) V.push(`${m.p[k]} ${m.p[k + 1]} ${m.p[k + 2]}`); for (let k = 0; k < m.i.length; k += 3) F.push(`3 ${m.i[k] + off} ${m.i[k + 1] + off} ${m.i[k + 2] + off}`); off += m.p.length / 3; } return `ply\nformat ascii 1.0\ncomment PHOENIX-ATMOS-XP units m\nelement vertex ${V.length}\nproperty float x\nproperty float y\nproperty float z\nelement face ${F.length}\nproperty list uchar int vertex_indices\nend_header\n${V.join('\n')}\n${F.join('\n')}\n`; }
export function toVTKPoly(parts, field) { const V = [], F = []; let off = 0; for (const m of parts) { for (let k = 0; k < m.p.length; k += 3) V.push(`${m.p[k]} ${m.p[k + 1]} ${m.p[k + 2]}`); for (let k = 0; k < m.i.length; k += 3) F.push(`3 ${m.i[k] + off} ${m.i[k + 1] + off} ${m.i[k + 2] + off}`); off += m.p.length / 3; } let s = `# vtk DataFile Version 3.0\nPHOENIX-ATMOS-XP\nASCII\nDATASET POLYDATA\nPOINTS ${V.length} float\n${V.join('\n')}\nPOLYGONS ${F.length} ${F.length * 4}\n${F.join('\n')}\n`; if (field) s += `POINT_DATA ${V.length}\nSCALARS ${field.name} float 1\nLOOKUP_TABLE default\n${field.values.join('\n')}\n`; return s; }
// STEP AP214 faceted B-rep (closed shells). Exact NURBS surfaces are not preserved — reported in conversion report.
export function toSTEP(parts, name = 'ATMOSXP') {
  let id = 0; const L = []; const e = (s) => { L.push(`#${++id}=${s};`); return id; };
  const now = new Date().toISOString().slice(0, 19);
  const ctx = e("(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#" + (id + 2) + ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((#" + (id + 3) + ",#" + (id + 4) + ",#" + (id + 5) + ")) REPRESENTATION_CONTEXT('3D',''))");
  e("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06),#" + (id + 2) + ",'distance_accuracy_value','')");
  e("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.))"); e("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))"); e("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())");
  const appc = e("APPLICATION_CONTEXT('automotive design')"); e(`APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#${appc})`);
  const solids = [];
  for (const m of parts) {
    const pts = []; for (let k = 0; k < m.p.length; k += 3) pts.push(e(`CARTESIAN_POINT('',(${m.p[k].toExponential(6)},${m.p[k + 1].toExponential(6)},${m.p[k + 2].toExponential(6)}))`));
    const faces = []; for (let k = 0; k < m.i.length; k += 3) { const loop = e(`POLY_LOOP('',(#${pts[m.i[k]]},#${pts[m.i[k + 1]]},#${pts[m.i[k + 2]]}))`); const bnd = e(`FACE_OUTER_BOUND('',#${loop},.T.)`); faces.push(e(`FACE('',(#${bnd}))`)); }
    const shell = e(`CLOSED_SHELL('${m.name.replace(/'/g, '')}',(${faces.map(f => '#' + f).join(',')}))`); solids.push(e(`FACETED_BREP('${m.name.replace(/'/g, '')}',#${shell})`));
  }
  const pc = e(`PRODUCT_CONTEXT('',#${appc},'mechanical')`), prod = e(`PRODUCT('${name}','${name}','PHOENIX-ATMOS-XP generated assembly',(#${pc}))`);
  const pdf = e(`PRODUCT_DEFINITION_FORMATION('','',#${prod})`), pdc = e(`PRODUCT_DEFINITION_CONTEXT('part definition',#${appc},'design')`), pd = e(`PRODUCT_DEFINITION('design','',#${pdf},#${pdc})`), pds = e(`PRODUCT_DEFINITION_SHAPE('','',#${pd})`);
  const rep = e(`FACETED_BREP_SHAPE_REPRESENTATION('',(${solids.map(s => '#' + s).join(',')}),#${ctx})`); e(`SHAPE_DEFINITION_REPRESENTATION(#${pds},#${rep})`);
  return `ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('PHOENIX-ATMOS-XP faceted B-rep export'),'2;1');\nFILE_NAME('${name}.step','${now}',(''),(''),'PHOENIX-ATMOS-XP','PHOENIX-ATMOS-XP','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n${L.join('\n')}\nENDSEC;\nEND-ISO-10303-21;\n`;
}
export function conversionReport(fmtName, parts) {
  const checks = parts.map(m => ({ name: m.name, tri: m.triangleCount, ...m.checkManifold(), volume: m.volume() }));
  const lost = { STL: ['part names', 'assembly structure', 'materials', 'exact curvature', 'units (header note only)'], OBJ: ['materials', 'exact curvature', 'units'], PLY: ['part names', 'materials', 'exact curvature'], STEP: ['exact NURBS curvature (faceted B-rep)', 'parametric feature tree (kept in native .atmosxp.json)'], glTF: ['exact curvature', 'parametric feature tree'], DXF: ['3D surfaces (2D projection only)'], VTK: ['part names', 'materials'] }[fmtName] || [];
  return { format: fmtName, parts: checks.length, triangles: checks.reduce((s, c) => s + c.tri, 0), allClosed: checks.every(c => c.closed), allOriented: checks.every(c => c.oriented), checks, preserved: ['geometry (tessellated, 1 µm precision)', 'outward normals', 'SI metre units'], lost };
}

// ---------------- 2D engineering drawings (third-angle orthographic, SVG + DXF) ----------------
// Silhouette projection: draws all triangle edges whose adjacent faces flip visibility (feature/silhouette lines)
function projectEdges(parts, view) {
  const proj = { top: (x, y, z) => [x, y], side: (x, y, z) => [x, z], front: (x, y, z) => [y, z] }[view];
  const dir = { top: [0, 0, 1], side: [0, -1, 0], front: [1, 0, 0] }[view]; const segs = [];
  for (const m of parts) {
    const P = m.p, E = new Map(), fn = [];
    for (let k = 0; k < m.i.length; k += 3) { const a = m.i[k] * 3, b = m.i[k + 1] * 3, c = m.i[k + 2] * 3; const ux = P[b] - P[a], uy = P[b + 1] - P[a + 1], uz = P[b + 2] - P[a + 2], vx = P[c] - P[a], vy = P[c + 1] - P[a + 1], vz = P[c + 2] - P[a + 2]; const n = [uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx], l = Math.hypot(...n) || 1; fn.push(n.map(v => v / l)); }
    for (let k = 0; k < m.i.length; k += 3) for (let e2 = 0; e2 < 3; e2++) { const a = m.i[k + e2], b = m.i[k + (e2 + 1) % 3], key = a < b ? a + '_' + b : b + '_' + a; const f = k / 3; if (!E.has(key)) E.set(key, [a, b, f, -1]); else E.get(key)[3] = f; }
    for (const [a, b, f1, f2] of E.values()) {
      const n1 = fn[f1], n2 = f2 >= 0 ? fn[f2] : null; const d1 = n1[0] * dir[0] + n1[1] * dir[1] + n1[2] * dir[2], d2 = n2 ? n2[0] * dir[0] + n2[1] * dir[1] + n2[2] * dir[2] : -d1;
      const crease = n2 ? (n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2]) < 0.82 : true;
      if (d1 * d2 <= 0 || crease) segs.push([...proj(P[a * 3], P[a * 3 + 1], P[a * 3 + 2]), ...proj(P[b * 3], P[b * 3 + 1], P[b * 3 + 2])]);
    }
  }
  return segs;
}
export function engineeringDrawing(parts, { title = 'PHOENIX-ATMOS-XP — General Arrangement', dwgNo = 'AXP-GA-001', rev = 'A', scale = '1:20', dims = [], material = '', author = '' } = {}) {
  const views = ['side', 'top', 'front'].map(v => ({ v, segs: projectEdges(parts, v) }));
  const bb = (segs) => { let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity; for (const s of segs) { x0 = Math.min(x0, s[0], s[2]); x1 = Math.max(x1, s[0], s[2]); y0 = Math.min(y0, s[1], s[3]); y1 = Math.max(y1, s[1], s[3]); } return { x0, y0, x1, y1, w: x1 - x0, h: y1 - y0 }; };
  views.forEach(v => v.bb = bb(v.segs));
  const W = 1189, H = 841, margin = 20, tb = 60; // A0-proportioned sheet in mm units
  const sMax = Math.min((W - 3 * margin - 140) / (views[0].bb.w + views[2].bb.w), (H - 3 * margin - tb - 60) / (views[0].bb.h + views[1].bb.h));
  const k = sMax * 0.9;
  // third-angle: top view above side view, front view to the right of side view
  const place = { side: [margin + 40, H - tb - margin - 40], top: [margin + 40, H - tb - margin - 80 - views[0].bb.h * k], front: [margin + 120 + views[0].bb.w * k, H - tb - margin - 40] };
  let svg = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" font-family="Arial, Helvetica, sans-serif"><rect width="${W}" height="${H}" fill="#fff"/><rect x="10" y="10" width="${W - 20}" height="${H - 20}" fill="none" stroke="#000" stroke-width="1.4"/>`;
  const dxf = []; const dxfLine = (x1, y1, x2, y2, layer) => dxf.push(`0\nLINE\n8\n${layer}\n10\n${x1.toFixed(3)}\n20\n${y1.toFixed(3)}\n11\n${x2.toFixed(3)}\n21\n${y2.toFixed(3)}`);
  const dxfText = (x, y, ht, t, layer = 'TEXT') => dxf.push(`0\nTEXT\n8\n${layer}\n10\n${x.toFixed(3)}\n20\n${y.toFixed(3)}\n40\n${ht}\n1\n${t}`);
  const toSheet = {};
  for (const v of views) {
    const [ox, oy] = place[v.v], b = v.bb, X = (x) => ox + (x - b.x0) * k, Y = (y) => oy - (y - b.y0) * k; toSheet[v.v] = { X, Y, b };
    svg += `<g stroke="#111" stroke-width="0.5" fill="none">` + v.segs.map(s => `<line x1="${X(s[0]).toFixed(2)}" y1="${Y(s[1]).toFixed(2)}" x2="${X(s[2]).toFixed(2)}" y2="${Y(s[3]).toFixed(2)}"/>`).join('') + '</g>';
    for (const s of v.segs) dxfLine(s[0] * 1000, s[1] * 1000 + (v.v === 'top' ? 4000 : 0), s[2] * 1000, s[3] * 1000 + (v.v === 'top' ? 4000 : 0), v.v.toUpperCase());
    svg += `<text x="${X((b.x0 + b.x1) / 2)}" y="${Y(b.y0) + 26}" font-size="12" text-anchor="middle" font-weight="700">${v.v.toUpperCase()} VIEW</text>`;
    // overall dimensions
    const dim = (x1, y1, x2, y2, label, off, vertical) => { const ext = vertical ? `M${x1},${y1}h${off}M${x2},${y2}h${off}` : `M${x1},${y1}v${off}M${x2},${y2}v${off}`; const lx = vertical ? x1 + off * 0.8 : x1, ly = vertical ? y1 : y1 + off * 0.8, mx = vertical ? x2 + off * 0.8 : x2, my = vertical ? y2 : y2 + off * 0.8; svg += `<path d="${ext}" stroke="#2563eb" stroke-width="0.35"/><line x1="${lx}" y1="${ly}" x2="${mx}" y2="${my}" stroke="#2563eb" stroke-width="0.5" marker-start="url(#ar)" marker-end="url(#ar)"/><text x="${(lx + mx) / 2 + (vertical ? 4 : 0)}" y="${(ly + my) / 2 - 3}" font-size="10" fill="#1d4ed8" text-anchor="${vertical ? 'start' : 'middle'}">${label}</text>`; };
    dim(X(b.x0), Y(b.y0), X(b.x1), Y(b.y0), (b.w * 1000).toFixed(0), 16, false);
    dim(X(b.x1), Y(b.y1), X(b.x1), Y(b.y0), (b.h * 1000).toFixed(0), 16, true);
  }
  for (const d of dims) { const t = toSheet[d.view]; if (!t) continue; const x1 = t.X(d.a[0]), y1 = t.Y(d.a[1]), x2 = t.X(d.b[0]), y2 = t.Y(d.b[1]); svg += `<line x1="${x1}" y1="${y1 + (d.off || -12)}" x2="${x2}" y2="${y2 + (d.off || -12)}" stroke="#16a34a" stroke-width="0.5" marker-start="url(#ar)" marker-end="url(#ar)"/><text x="${(x1 + x2) / 2}" y="${(y1 + y2) / 2 + (d.off || -12) - 3}" font-size="10" fill="#15803d" text-anchor="middle">${d.label}</text>`; }
  svg = svg.replace('<rect width', '<defs><marker id="ar" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0,1L10,5L0,9z" fill="#2563eb"/></marker></defs><rect width');
  // title block (ISO 7200 fields)
  const tx = W - 10 - 520, ty = H - 10 - tb;
  svg += `<g font-size="10"><rect x="${tx}" y="${ty}" width="520" height="${tb}" fill="none" stroke="#000"/><line x1="${tx}" y1="${ty + 20}" x2="${tx + 520}" y2="${ty + 20}" stroke="#000"/><line x1="${tx + 300}" y1="${ty}" x2="${tx + 300}" y2="${ty + tb}" stroke="#000"/><line x1="${tx + 410}" y1="${ty}" x2="${tx + 410}" y2="${ty + tb}" stroke="#000"/>
  <text x="${tx + 6}" y="${ty + 14}" font-weight="700" font-size="12">${title}</text><text x="${tx + 306}" y="${ty + 14}">DWG No. ${dwgNo}</text><text x="${tx + 416}" y="${ty + 14}">REV ${rev}</text>
  <text x="${tx + 6}" y="${ty + 34}" font-size="9">Units mm · 3rd-angle · Tol. ISO 2768-mK</text><text x="${tx + 6}" y="${ty + 50}">Material: ${material || 'see BOM / materials register'}</text>
  <text x="${tx + 306}" y="${ty + 34}">Scale ${scale}</text><text x="${tx + 306}" y="${ty + 50}">Date ${new Date().toISOString().slice(0, 10)}</text><text x="${tx + 416}" y="${ty + 34}">Drawn: ${author || '—'}</text><text x="${tx + 416}" y="${ty + 50}">Sheet 1/1</text></g>`;
  // third-angle symbol
  svg += `<g transform="translate(${tx - 70},${ty + 18})" stroke="#000" fill="none" stroke-width="0.8"><circle cx="40" cy="12" r="10"/><circle cx="40" cy="12" r="4"/><path d="M0,4L22,0V24L0,20Z"/></g></svg>`;
  dxfText(0, -600, 120, title); dxfText(0, -800, 80, `DWG ${dwgNo} REV ${rev} units mm`);
  const dxfStr = `0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n${dxf.join('\n')}\n0\nENDSEC\n0\nEOF\n`;
  return { svg, dxf: dxfStr };
}
export function airfoilDAT(af) { return af.name + '\n' + af.points.map(p => ` ${p[0].toFixed(6)}  ${p[1].toFixed(6)}`).join('\n') + '\n'; }
export function airfoilDXF(af, chord = 1) { const L = []; for (let k = 0; k < af.points.length - 1; k++) { const a = af.points[k], b = af.points[k + 1]; L.push(`0\nLINE\n8\nAIRFOIL\n10\n${(a[0] * chord * 1000).toFixed(3)}\n20\n${(a[1] * chord * 1000).toFixed(3)}\n11\n${(b[0] * chord * 1000).toFixed(3)}\n21\n${(b[1] * chord * 1000).toFixed(3)}`); } return `0\nSECTION\n2\nENTITIES\n${L.join('\n')}\n0\nENDSEC\n0\nEOF\n`; }
