// Mesh engine: structured O-grids around airfoils (normal marching + elliptic-type smoothing), 2-D Delaunay
// (Bowyer–Watson) unstructured meshes with boundary-layer node layers, element-quality metrics, a P1
// finite-element potential-flow solver used for mesh-sensitivity studies, GCI-based mesh selection and
// writers for Gmsh MSH 2.2, VTK, SU2, OpenFOAM polyMesh, Abaqus INP and Nastran BDF.
import { SparseBuilder, pcg, gci as gciFn, clamp, linspace, range, mean, max, min } from './num.js';
import { inside } from './lbm-core.js';

// --- resample a closed airfoil contour (TE→upper→LE→lower→TE) with cosine clustering at LE and TE ---
export function resampleAirfoil(pts, n) {
  const P = pts.slice(); if (Math.hypot(P[0][0] - P[P.length - 1][0], P[0][1] - P[P.length - 1][1]) < 1e-9) P.pop();
  const s = [0]; for (let i = 1; i <= P.length; i++) { const a = P[i - 1], b = P[i % P.length]; s.push(s[i - 1] + Math.hypot(b[0] - a[0], b[1] - a[1])); }
  const L = s[s.length - 1], ile = P.reduce((bi, p, i) => p[0] < P[bi][0] ? i : bi, 0), sle = s[ile];
  const out = [];
  for (let k = 0; k < n; k++) {
    const t = k / n; // first half: TE→LE along upper, second half LE→TE along lower, cosine per half
    const tt = t < 0.5 ? sle * 0.5 * (1 - Math.cos(Math.PI * t * 2)) : sle + (L - sle) * 0.5 * (1 - Math.cos(Math.PI * (t - 0.5) * 2));
    let j = 1; while (j < s.length - 1 && s[j] < tt) j++;
    const a = P[(j - 1) % P.length], b = P[j % P.length], f = (tt - s[j - 1]) / ((s[j] - s[j - 1]) || 1);
    out.push([a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])]);
  }
  return out;
}

// --- structured O-grid: ni around (periodic), nj radial ---
// Blunt/round a sharp trailing edge (O-grid topology needs a smooth closed contour): thicken the aft
// surfaces by r·x⁸ and close with a semicircular arc. Returns contour in the same ordering.
export function roundTE(pts, r = 0.004) {
  const P = pts.slice(); if (Math.hypot(P[0][0] - P[P.length - 1][0], P[0][1] - P[P.length - 1][1]) < 1e-9) P.pop();
  const ile = P.reduce((bi, p, i) => p[0] < P[bi][0] ? i : bi, 0), xte = Math.max(...P.map(p => p[0]));
  const base = Math.abs(P[0][1] - P[P.length - 1][1]); if (base >= 2 * r) return { points: pts, rounded: false };
  const add = (r - base / 2);
  const out = P.map(([x, y], i) => [x, y + (i <= ile ? 1 : -1) * add * Math.pow(Math.max(0, x / xte), 8)]);
  const yu = out[0][1], yl = out[out.length - 1][1], yc = (yu + yl) / 2, rr = (yu - yl) / 2, arc = [];
  for (let k = 1; k < 8; k++) { const a = Math.PI / 2 - Math.PI * k / 8; arc.push([xte + rr * Math.cos(a), yc + rr * Math.sin(a)]); }
  // contour starts at the arc tip (angle 0) so the O-grid seam stays on the wake line; arc[3] is the tip
  const upper = out.slice(0, ile + 1), lower = out.slice(ile + 1);
  const res = [arc[3], arc[2], arc[1], arc[0], ...upper, ...lower, arc[6], arc[5], arc[4]];
  return { points: res, rounded: true, radius: rr };
}

export function ogrid(airfoil, { ni = 96, nj = 40, R = 12, h1 = 2e-3, smooth = 60, teRadius = 0.004 } = {}) {
  const rt = roundTE(airfoil, teRadius); const W = resampleAirfoil(rt.points, ni), cx = 0.5;
  // geometric stretching with first spacing h1 reaching R: solve growth ratio g
  let g = 1.2; for (let it = 0; it < 200; it++) { const total = h1 * (g ** (nj - 1) - 1) / (g - 1); g += (total < R ? 1 : -1) * 0.5 ** (it / 8 + 3); g = Math.max(1.0001, g); }
  const d = [0]; for (let j = 1; j < nj; j++) d.push(d[j - 1] + h1 * g ** (j - 1)); const dmax = d[nj - 1];
  const X = range(nj).map(() => new Array(ni)), Y = range(nj).map(() => new Array(ni));
  // outer circle point for each surface point (polar angle about mid-chord)
  const Q = W.map(([x, y]) => { const a = Math.atan2(y, x - cx); return [cx + R * Math.cos(a), R * Math.sin(a)]; });
  // hyperbolic-type marching: each layer advances along the smoothed normals of the previous layer, then the
  // marched family is blended (in position) with straight rays to the far-field circle so the outer row is exact.
  for (let i = 0; i < ni; i++) { X[0][i] = W[i][0]; Y[0][i] = W[i][1]; }
  let prev = W.map(p => p.slice()); const march = [prev];
  for (let j = 1; j < nj; j++) {
    let N = prev.map((_, i) => { const a = prev[(i - 1 + ni) % ni], b = prev[(i + 1) % ni], tx = b[0] - a[0], ty = b[1] - a[1], l = Math.hypot(tx, ty) || 1; return [ty / l, -tx / l]; });
    const passes = Math.min(smooth, 2 + j);
    for (let k = 0; k < passes; k++) N = N.map((n, i) => { const a = N[(i - 1 + ni) % ni], b = N[(i + 1) % ni], x = 0.5 * n[0] + 0.25 * (a[0] + b[0]), y = 0.5 * n[1] + 0.25 * (a[1] + b[1]), l = Math.hypot(x, y) || 1; return [x / l, y / l]; });
    const step = d[j] - d[j - 1];
    prev = prev.map((p, i) => [p[0] + N[i][0] * step, p[1] + N[i][1] * step]); march.push(prev);
  }
  // project the final marched layer radially onto the far-field circle; distribute the correction smoothly
  const last = march[nj - 1], QQ = last.map(([x, y]) => { const a = Math.atan2(y, x - cx); return [cx + R * Math.cos(a), R * Math.sin(a)]; });
  for (let j = 1; j < nj; j++) { const w = (d[j] / dmax) ** 2; for (let i = 0; i < ni; i++) { X[j][i] = march[j][i][0] + w * (QQ[i][0] - last[i][0]); Y[j][i] = march[j][i][1] + w * (QQ[i][1] - last[i][1]); } }
  void Q;
  const nodes = [], id = (i, j) => j * ni + (i % ni);
  for (let j = 0; j < nj; j++) for (let i = 0; i < ni; i++) nodes.push([X[j][i], Y[j][i]]);
  const elems = []; for (let j = 0; j < nj - 1; j++) for (let i = 0; i < ni; i++) elems.push([id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)]);
  const wall = range(ni).map(i => [id(i, 0), id(i + 1, 0)]), farfield = range(ni).map(i => [id(i, nj - 1), id(i + 1, nj - 1)]);
  // orient elements counter-clockwise (positive Jacobian)
  for (const e of elems) if (signedArea(nodes, e) < 0) e.reverse();
  return { type: 'quad', nodes, elems, bnd: { wall, farfield }, ni, nj, growth: g, h1, R, surface: W, structured: true, teRounded: rt.rounded ? rt.radius : 0 };
}

// --- 2-D Delaunay (Bowyer–Watson) ---
export function delaunay(pts) {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity; for (const [x, y] of pts) { x0 = Math.min(x0, x); y0 = Math.min(y0, y); x1 = Math.max(x1, x); y1 = Math.max(y1, y); }
  const dm = Math.max(x1 - x0, y1 - y0) * 20, mx = (x0 + x1) / 2, my = (y0 + y1) / 2;
  const P = pts.concat([[mx - dm, my - dm], [mx, my + dm], [mx + dm, my - dm]]), n = pts.length;
  const circ = (a, b, c) => { const [ax, ay] = P[a], [bx, by] = P[b], [cx, cy] = P[c], d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by)); const ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d, uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d; return [ux, uy, (ax - ux) ** 2 + (ay - uy) ** 2]; };
  let tris = [{ v: [n, n + 1, n + 2], c: circ(n, n + 1, n + 2) }];
  // insert points in spatially coherent order (sorted by grid cell) for speed
  const order = range(n).sort((a, b) => (Math.floor(P[a][1] * 4) - Math.floor(P[b][1] * 4)) || (P[a][0] - P[b][0]));
  for (const p of order) {
    const [px, py] = P[p], bad = [], keep = [];
    for (const t of tris) ((px - t.c[0]) ** 2 + (py - t.c[1]) ** 2 < t.c[2] * (1 + 1e-12) ? bad : keep).push(t);
    const edges = new Map();
    for (const t of bad) for (let e = 0; e < 3; e++) { const a = t.v[e], b = t.v[(e + 1) % 3], k = a < b ? a + ',' + b : b + ',' + a; edges.set(k, edges.has(k) ? null : [a, b]); }
    for (const e of edges.values()) if (e) keep.push({ v: [e[0], e[1], p], c: circ(e[0], e[1], p) });
    tris = keep;
  }
  return tris.filter(t => t.v.every(i => i < n)).map(t => t.v);
}

// Unstructured triangular mesh around an airfoil with boundary-layer node layers and radial sizing
export function unstructured(airfoil, { ns = 120, layers = 8, h1 = 2e-3, growth = 1.3, R = 12, sizeFactor = 0.18 } = {}) {
  const W = resampleAirfoil(airfoil, ns), pts = W.map(p => p.slice());
  const N = W.map((_, i) => { const a = W[(i - 1 + ns) % ns], b = W[(i + 1) % ns], tx = b[0] - a[0], ty = b[1] - a[1], l = Math.hypot(tx, ty) || 1; return [ty / l, -tx / l]; });
  let d = 0, h = h1; for (let k = 0; k < layers; k++) { d += h; h *= growth; for (let i = 0; i < ns; i++) pts.push([W[i][0] + N[i][0] * d, W[i][1] + N[i][1] * d]); }
  const dBL = d, cx = 0.5;
  // rings with spacing growing with distance (target size ∝ distance from body)
  let r = 0.5 + dBL + 0.08; const perimeter = W.reduce((s, p, i) => s + Math.hypot(W[(i + 1) % ns][0] - p[0], W[(i + 1) % ns][1] - p[1]), 0);
  let hs = Math.max(perimeter / ns * 1.5, h * 1.2);
  while (r < R) { const nring = Math.max(12, Math.round(2 * Math.PI * r / hs)); for (let k = 0; k < nring; k++) { const a = 2 * Math.PI * (k + 0.5 * ((Math.round(r * 97)) % 2)) / nring; const px = cx + r * Math.cos(a), py = r * Math.sin(a); if (r < R - 0.6 * hs && !inside(W, px, py)) pts.push([px, py]); } r += hs; hs = Math.max(hs, sizeFactor * (r - 0.5)); }
  const nff = Math.max(24, Math.round(2 * Math.PI * R / hs)), ff0 = pts.length; for (let k = 0; k < nff; k++) { const a = 2 * Math.PI * k / nff; pts.push([cx + R * Math.cos(a), R * Math.sin(a)]); }
  // remove points accidentally inside the body (concave regions) or too close to each other
  let tri = delaunay(pts);
  tri = tri.filter(t => { const c = [(pts[t[0]][0] + pts[t[1]][0] + pts[t[2]][0]) / 3, (pts[t[0]][1] + pts[t[1]][1] + pts[t[2]][1]) / 3]; if (inside(W, c[0], c[1])) return false; for (let e = 0; e < 3; e++) { const a = pts[t[e]], b = pts[t[(e + 1) % 3]]; if (t[e] < ns && t[(e + 1) % 3] < ns && Math.abs(t[e] - t[(e + 1) % 3]) !== 1 && Math.abs(t[e] - t[(e + 1) % 3]) !== ns - 1) { if (inside(W, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2)) return false; } } return true; });
  for (const e of tri) if (signedArea(pts, e) < 0) e.reverse();
  // boundary edges
  const E = new Map(); for (const t of tri) for (let e = 0; e < 3; e++) { const a = t[e], b = t[(e + 1) % 3], k = a < b ? a + ',' + b : b + ',' + a; E.set(k, (E.get(k) || 0) + 1); }
  const wall = [], farfield = []; let recovered = 0;
  for (let i = 0; i < ns; i++) { const a = i, b = (i + 1) % ns, k = a < b ? a + ',' + b : b + ',' + a; if (E.get(k) === 1) { wall.push([a, b]); recovered++; } }
  for (const [k, c] of E) if (c === 1) { const [a, b] = k.split(',').map(Number); if (a >= ff0 && b >= ff0) farfield.push([a, b]); }
  return { type: 'tri', nodes: pts, elems: tri, bnd: { wall, farfield }, ns, layers, h1, R, surface: W, boundaryRecovery: recovered / ns };
}

export function signedArea(nodes, e) { let a = 0; for (let k = 0; k < e.length; k++) { const p = nodes[e[k]], q = nodes[e[(k + 1) % e.length]]; a += p[0] * q[1] - q[0] * p[1]; } return a / 2; }

// --- element quality ---
export function quality(mesh) {
  const { nodes, elems } = mesh, nE = elems.length, minJ = new Float64Array(nE), skew = new Float64Array(nE), ar = new Float64Array(nE), area = new Float64Array(nE), cen = [];
  for (let k = 0; k < nE; k++) {
    const e = elems[k], m = e.length, P = e.map(i => nodes[i]); let jmin = Infinity, amin = 180, amax = 0, lmin = Infinity, lmax = 0;
    for (let c = 0; c < m; c++) {
      const a = P[(c - 1 + m) % m], b = P[c], d = P[(c + 1) % m], e1 = [d[0] - b[0], d[1] - b[1]], e2 = [a[0] - b[0], a[1] - b[1]], l1 = Math.hypot(...e1), l2 = Math.hypot(...e2);
      const cr = (e1[0] * e2[1] - e1[1] * e2[0]) / (l1 * l2 || 1e-30); jmin = Math.min(jmin, cr);
      const ang = Math.acos(clamp((e1[0] * e2[0] + e1[1] * e2[1]) / (l1 * l2 || 1e-30), -1, 1)) * 180 / Math.PI; amin = Math.min(amin, ang); amax = Math.max(amax, ang); lmin = Math.min(lmin, l1); lmax = Math.max(lmax, l1);
    }
    const ideal = m === 3 ? 60 : 90; minJ[k] = m === 3 ? jmin / Math.sin(Math.PI / 3) : jmin;
    skew[k] = Math.max((amax - ideal) / (180 - ideal), (ideal - amin) / ideal);
    if (m === 3) { const [a, b, c] = [0, 1, 2].map(i => Math.hypot(P[(i + 1) % 3][0] - P[i][0], P[(i + 1) % 3][1] - P[i][1])), s = (a + b + c) / 2; ar[k] = a * b * c / (8 * Math.max(1e-30, (s - a) * (s - b) * (s - c))); }
    else ar[k] = lmax / (lmin || 1e-30);
    area[k] = signedArea(nodes, e); cen.push([mean(P.map(p => p[0])), mean(P.map(p => p[1]))]);
  }
  // non-orthogonality: angle between centroid-to-centroid vector and shared-face normal
  const F = new Map(), nonOrth = [];
  elems.forEach((e, k) => { for (let c = 0; c < e.length; c++) { const a = e[c], b = e[(c + 1) % e.length], key = a < b ? a + ',' + b : b + ',' + a; if (F.has(key)) { const o = F.get(key), pa = nodes[a], pb = nodes[b], nx = pb[1] - pa[1], ny = -(pb[0] - pa[0]), dx = cen[k][0] - cen[o][0], dy = cen[k][1] - cen[o][1]; nonOrth.push(Math.acos(clamp(Math.abs(nx * dx + ny * dy) / (Math.hypot(nx, ny) * Math.hypot(dx, dy) || 1e-30), 0, 1)) * 180 / Math.PI); } else F.set(key, k); } });
  const neg = Array.from(area).filter(a => a <= 0).length;
  return { minJ: Array.from(minJ), skew: Array.from(skew), ar: Array.from(ar), area: Array.from(area), nonOrth, summary: { cells: nE, nodes: nodes.length, minJacobian: min(minJ), maxSkew: max(skew), meanSkew: mean(skew), maxAR: max(ar), maxNonOrth: nonOrth.length ? max(nonOrth) : 0, meanNonOrth: nonOrth.length ? mean(nonOrth) : 0, negativeCells: neg, faces: F.size } };
}

// --- P1 finite-element potential-flow solver: ∇²φ = 0, φ = x cosα + y sinα on far field, ∂φ/∂n = 0 on wall ---
export function potentialFlow(mesh, alphaDeg = 0) {
  const tris = mesh.type === 'tri' ? mesh.elems : mesh.elems.flatMap(q => [[q[0], q[1], q[2]], [q[0], q[2], q[3]]]);
  const n = mesh.nodes.length, a = alphaDeg * Math.PI / 180, ca = Math.cos(a), sa = Math.sin(a), X = mesh.nodes;
  const dir = new Uint8Array(n), g = new Float64Array(n); for (const [p, q] of mesh.bnd.farfield) for (const i of [p, q]) { dir[i] = 1; g[i] = X[i][0] * ca + X[i][1] * sa; }
  const K = new SparseBuilder(n), b = new Float64Array(n), grads = [];
  for (const t of tris) {
    const [i, j, k] = t, [x1, y1] = X[i], [x2, y2] = X[j], [x3, y3] = X[k], A2 = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1), A = A2 / 2;
    const bb = [y2 - y3, y3 - y1, y1 - y2], cc = [x3 - x2, x1 - x3, x2 - x1]; grads.push({ t, bb, cc, A2 });
    for (let r = 0; r < 3; r++) for (let s = 0; s < 3; s++) {
      const kv = (bb[r] * bb[s] + cc[r] * cc[s]) / (4 * Math.abs(A)), I = t[r], J = t[s];
      if (dir[I]) continue; if (dir[J]) b[I] -= kv * g[J]; else K.add(I, J, kv);
    }
  }
  for (let i = 0; i < n; i++) if (dir[i]) { K.set(i, i, 1); b[i] = g[i]; }
  const sol = pcg(K.toCSR(), b, { tol: 1e-10, maxIt: 8000 }); const phi = sol.x;
  // nodal velocity by area-weighted averaging of element gradients
  const u = new Float64Array(n), v = new Float64Array(n), w = new Float64Array(n);
  for (const { t, bb, cc, A2 } of grads) { let gx = 0, gy = 0; for (let r = 0; r < 3; r++) { gx += bb[r] * phi[t[r]]; gy += cc[r] * phi[t[r]]; } gx /= A2; gy /= A2; const aw = Math.abs(A2); for (const i of t) { u[i] += gx * aw; v[i] += gy * aw; w[i] += aw; } }
  for (let i = 0; i < n; i++) { u[i] /= w[i] || 1; v[i] /= w[i] || 1; }
  const wallNodes = [...new Set(mesh.bnd.wall.flat())], speed = wallNodes.map(i => Math.hypot(u[i], v[i]));
  return { phi, u, v, wallNodes, VmaxSurface: max(speed), cg: { it: sol.it, res: sol.res, ok: sol.ok, hist: sol.hist } };
}

// --- mesh-sensitivity study & credible selection ---
export function representativeH(mesh) { const q = mesh.elems.reduce((s, e) => s + Math.abs(signedArea(mesh.nodes, e)), 0); return Math.sqrt(q / mesh.elems.length); }
export function sensitivityStudy(gen, quantity, levels) {
  const L = levels.map((lv, k) => { const m = gen(lv, k); const Q = quality(m).summary, val = quantity(m); return { level: k, params: lv, mesh: m, cells: m.elems.length, h: representativeH(m), value: val, quality: Q }; });
  // GCI on the three finest levels (sorted fine→coarse)
  const s = L.slice().sort((a, b) => a.h - b.h), r = s.length >= 3 ? gciFn(s[0].value, s[1].value, s[2].value, s[0].h, s[1].h, s[2].h) : null;
  return { levels: L, gci: r, sorted: s };
}
export function selectMesh(study, { gciMax = 0.02, maxSkew = 0.85, minJ = 0.05, maxNonOrth = 70 } = {}) {
  const s = study.sorted, fext = study.gci?.fext ?? s[0].value;
  const rows = s.map(l => { const err = Math.abs((l.value - fext) / (fext || 1)); const q = l.quality; const qualOk = q.negativeCells === 0 && q.minJacobian > minJ && q.maxSkew <= maxSkew && q.maxNonOrth <= maxNonOrth; return { ...l, errExtrap: err, qualOk, accOk: err <= gciMax, ok: qualOk && err <= gciMax }; });
  // choose the cheapest (coarsest) level meeting accuracy & quality; fall back to finest
  const ok = rows.filter(r2 => r2.ok).sort((a, b) => a.cells - b.cells);
  return { chosen: ok[0] || rows[0], rows, criteria: { gciMax, maxSkew, minJ, maxNonOrth } };
}

// --- writers ---
const tri2 = (m) => m.type === 'tri';
export function toGmsh(m) {
  const L = ['$MeshFormat', '2.2 0 8', '$EndMeshFormat', '$PhysicalNames', '3', '1 1 "wall"', '1 2 "farfield"', '2 3 "fluid"', '$EndPhysicalNames', '$Nodes', String(m.nodes.length)];
  m.nodes.forEach((p, i) => L.push(`${i + 1} ${p[0]} ${p[1]} 0`)); L.push('$EndNodes');
  const el = []; let k = 1; for (const [a, b] of m.bnd.wall) el.push(`${k++} 1 2 1 1 ${a + 1} ${b + 1}`); for (const [a, b] of m.bnd.farfield) el.push(`${k++} 1 2 2 2 ${a + 1} ${b + 1}`);
  for (const e of m.elems) el.push(`${k++} ${e.length === 3 ? 2 : 3} 2 3 3 ${e.map(i => i + 1).join(' ')}`);
  return L.concat(['$Elements', String(el.length), ...el, '$EndElements']).join('\n') + '\n';
}
export function toVTK(m, fields = {}) {
  const L = ['# vtk DataFile Version 3.0', 'PHOENIX-ATMOS-XP mesh', 'ASCII', 'DATASET UNSTRUCTURED_GRID', `POINTS ${m.nodes.length} double`, ...m.nodes.map(p => `${p[0]} ${p[1]} 0`)];
  const sz = m.elems.reduce((s, e) => s + e.length + 1, 0); L.push(`CELLS ${m.elems.length} ${sz}`, ...m.elems.map(e => `${e.length} ${e.join(' ')}`), `CELL_TYPES ${m.elems.length}`, ...m.elems.map(e => e.length === 3 ? '5' : '9'));
  const cf = Object.entries(fields); if (cf.length) { L.push(`CELL_DATA ${m.elems.length}`); for (const [name, vals] of cf) L.push(`SCALARS ${name} double 1`, 'LOOKUP_TABLE default', ...vals.map(v => String(v))); }
  return L.join('\n') + '\n';
}
export function toSU2(m) {
  const L = ['% PHOENIX-ATMOS-XP 2-D mesh', 'NDIME= 2', `NELEM= ${m.elems.length}`, ...m.elems.map((e, k) => `${e.length === 3 ? 5 : 9} ${e.join(' ')} ${k}`), `NPOIN= ${m.nodes.length}`, ...m.nodes.map((p, i) => `${p[0]} ${p[1]} ${i}`), 'NMARK= 2', 'MARKER_TAG= airfoil', `MARKER_ELEMS= ${m.bnd.wall.length}`, ...m.bnd.wall.map(([a, b]) => `3 ${a} ${b}`), 'MARKER_TAG= farfield', `MARKER_ELEMS= ${m.bnd.farfield.length}`, ...m.bnd.farfield.map(([a, b]) => `3 ${a} ${b}`)];
  return L.join('\n') + '\n';
}
export function toAbaqus(m) { const L = ['*Heading', 'PHOENIX-ATMOS-XP mesh (units m)', '*Node']; m.nodes.forEach((p, i) => L.push(`${i + 1}, ${p[0]}, ${p[1]}, 0.`)); const tri = m.elems.filter(e => e.length === 3), quad = m.elems.filter(e => e.length === 4); let k = 1; if (tri.length) { L.push('*Element, type=CPS3, elset=FLUID'); for (const e of tri) L.push(`${k++}, ${e.map(i => i + 1).join(', ')}`); } if (quad.length) { L.push('*Element, type=CPS4, elset=FLUID'); for (const e of quad) L.push(`${k++}, ${e.map(i => i + 1).join(', ')}`); } L.push('*Nset, nset=WALL', ...chunk([...new Set(m.bnd.wall.flat())].map(i => i + 1)), '*Nset, nset=FARFIELD', ...chunk([...new Set(m.bnd.farfield.flat())].map(i => i + 1))); return L.join('\n') + '\n'; }
const chunk = (a) => { const o = []; for (let i = 0; i < a.length; i += 16) o.push(a.slice(i, i + 16).join(', ')); return o; };
export function toNastran(m) { const L = ['$ PHOENIX-ATMOS-XP mesh (units m)', 'BEGIN BULK']; m.nodes.forEach((p, i) => L.push(`GRID,${i + 1},,${p[0].toPrecision(8)},${p[1].toPrecision(8)},0.`)); m.elems.forEach((e, k) => L.push(`${e.length === 3 ? 'CTRIA3' : 'CQUAD4'},${k + 1},1,${e.map(i => i + 1).join(',')}`)); L.push('PSHELL,1,1,0.001', 'ENDDATA'); return L.join('\n') + '\n'; }
// OpenFOAM polyMesh (2-D mesh extruded one cell in z; front/back as "empty" patch)
export function toOpenFOAM(m, dz = 0.1) {
  const n = m.nodes.length, pts = m.nodes.map(p => `(${p[0]} ${p[1]} 0)`).concat(m.nodes.map(p => `(${p[0]} ${p[1]} ${dz})`));
  const F = new Map(); m.elems.forEach((e, c) => { for (let k = 0; k < e.length; k++) { const a = e[k], b = e[(k + 1) % e.length], key = a < b ? a + ',' + b : b + ',' + a; if (F.has(key)) F.get(key).nb = c; else F.set(key, { a, b, own: c, nb: -1 }); } });
  const wallSet = new Set(m.bnd.wall.map(([a, b]) => (a < b ? a + ',' + b : b + ',' + a))), internal = [], wall = [], far = [];
  for (const [k, f] of F) (f.nb >= 0 ? internal : wallSet.has(k) ? wall : far).push(f);
  internal.forEach(f => { if (f.nb < f.own) { const t = f.own; f.own = f.nb; f.nb = t; const tt = f.a; f.a = f.b; f.b = tt; } }); internal.sort((x, y) => x.own - y.own || x.nb - y.nb);
  // edge (a→b) is counter-clockwise for its owner; quad face (a, b, b', a') then has outward normal from owner
  const quadFace = (f) => `4(${f.a} ${f.b} ${f.b + n} ${f.a + n})`;
  const faces = [], owner = [], neigh = [];
  for (const f of internal) { faces.push(quadFace(f)); owner.push(f.own); neigh.push(f.nb); }
  const start = { wall: faces.length }; for (const f of wall) { faces.push(quadFace(f)); owner.push(f.own); }
  start.far = faces.length; for (const f of far) { faces.push(quadFace(f)); owner.push(f.own); }
  start.empty = faces.length; m.elems.forEach((e, c) => { faces.push(`${e.length}(${e.slice().reverse().join(' ')})`); owner.push(c); faces.push(`${e.length}(${e.map(i => i + n).join(' ')})`); owner.push(c); });
  const hdr = (cls, obj) => `FoamFile\n{\n    version     2.0;\n    format      ascii;\n    class       ${cls};\n    object      ${obj};\n}\n`;
  return {
    points: hdr('vectorField', 'points') + `${pts.length}\n(\n${pts.join('\n')}\n)\n`,
    faces: hdr('faceList', 'faces') + `${faces.length}\n(\n${faces.join('\n')}\n)\n`,
    owner: hdr('labelList', 'owner') + `${owner.length}\n(\n${owner.join('\n')}\n)\n`,
    neighbour: hdr('labelList', 'neighbour') + `${neigh.length}\n(\n${neigh.join('\n')}\n)\n`,
    boundary: hdr('polyBoundaryMesh', 'boundary') + `3\n(\n    airfoil { type wall; nFaces ${wall.length}; startFace ${start.wall}; }\n    farfield { type patch; nFaces ${far.length}; startFace ${start.far}; }\n    frontAndBack { type empty; nFaces ${2 * m.elems.length}; startFace ${start.empty}; }\n)\n`,
  };
}
export { linspace };
