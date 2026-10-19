// 3-D vehicle flow-field model for visualisation and probing: boundary-layer freestream (log law) relative to
// the moving vehicle + lifting-line horseshoe-vortex system of each vertical wing (Biot–Savart with a
// Vatistas-type core) + a line of 3-D doublets approximating the monocoque + ground-plane images
// (mirror and negate) for the no-penetration condition at the road. Pre-sampled on a Cartesian grid for
// fast trilinear interpolation by particle tracers and streamlines.

const FOUR_PI = 4 * Math.PI;

// finite vortex segment A→B, strength G, at P; rc core radius
function segVel(out, ax, ay, az, bx, by, bz, G, px, py, pz, rc) {
  const r1x = px - ax, r1y = py - ay, r1z = pz - az, r2x = px - bx, r2y = py - by, r2z = pz - bz;
  const cx = r1y * r2z - r1z * r2y, cy = r1z * r2x - r1x * r2z, cz = r1x * r2y - r1y * r2x;
  const r0x = bx - ax, r0y = by - ay, r0z = bz - az, l0 = r0x * r0x + r0y * r0y + r0z * r0z;
  const c2 = cx * cx + cy * cy + cz * cz + rc * rc * l0; if (c2 < 1e-12) return;
  const n1 = Math.sqrt(r1x * r1x + r1y * r1y + r1z * r1z) || 1e-9, n2 = Math.sqrt(r2x * r2x + r2y * r2y + r2z * r2z) || 1e-9;
  const K = G / (FOUR_PI * c2) * (r0x * (r1x / n1 - r2x / n2) + r0y * (r1y / n1 - r2y / n2) + r0z * (r1z / n1 - r2z / n2));
  out[0] += K * cx; out[1] += K * cy; out[2] += K * cz;
}
// semi-infinite vortex from A along unit d, strength G
function semiVel(out, ax, ay, az, dx, dy, dz, G, px, py, pz, rc) {
  const r1x = px - ax, r1y = py - ay, r1z = pz - az, n1 = Math.sqrt(r1x * r1x + r1y * r1y + r1z * r1z) || 1e-9;
  const cx = dy * r1z - dz * r1y, cy = dz * r1x - dx * r1z, cz = dx * r1y - dy * r1x, h2 = cx * cx + cy * cy + cz * cz;
  const K = G * (1 + (dx * r1x + dy * r1y + dz * r1z) / n1) / (FOUR_PI * (h2 + rc * rc));
  out[0] += K * cx; out[1] += K * cy; out[2] += K * cz;
}
function doubletVel(out, cx, cy, cz, mx, my, mz, px, py, pz, a) {
  let rx = px - cx, ry = py - cy, rz = pz - cz; let r2 = rx * rx + ry * ry + rz * rz; const rmin2 = (0.85 * a) ** 2; if (r2 < rmin2) { const s = Math.sqrt(rmin2 / Math.max(r2, 1e-12)); rx *= s; ry *= s; rz *= s; r2 = rmin2; }
  const r = Math.sqrt(r2), r5 = r2 * r2 * r, md = mx * rx + my * ry + mz * rz;
  // ∇(μ·r / 4πr³) = (μ/r³ − 3(μ·r)r/r⁵)/4π  (with μ = 2πa³U this cancels U at the sphere's stagnation points)
  out[0] += (mx / (r2 * r) - 3 * md * rx / r5) / FOUR_PI; out[1] += (my / (r2 * r) - 3 * md * ry / r5) / FOUR_PI; out[2] += (mz / (r2 * r) - 3 * md * rz / r5) / FOUR_PI;
}

// Build the singularity model.
// opts: air(z) -> [ux, uy] relative air velocity at height z (vehicle frame, x forward, y left),
//       wings: [{x, z0, z1, L, nDir?}] (lift per wing, N, signed; acts ⟂ to local air; nDir picks the lift side,
//       default = positive forward component as in the vehicle thrust resolution), rho, Va,
//       body: [{x, cz, a}] doublet stations, nSeg, rc
export function vehicleFlowModel({ air, wings, body = [], rho = 1.225, Va, nSeg = 10, rc = 0.12, ground = true }) {
  const segs = [], semis = [], doub = [];
  const add = (ax, ay, az, bx, by, bz, G) => { segs.push([ax, ay, az, bx, by, bz, G]); if (ground) segs.push([ax, ay, -az, bx, by, -bz, -G]); };
  for (const w of wings) {
    // lifting line from (x, ya, z0) to (xb, yb, z1); default vertical at y = 0. loading 'box' keeps a finite tip value
    // (closed box wing, Prandtl) and, with `closure` = [x, y, z], the tip circulation continues along the bridge to the
    // closure point and is shed there (two legs → one merged trailing vortex at the bridge centre).
    const xa = w.x, ya = w.ya || 0, xb = w.xb ?? w.x, yb = w.yb || 0, len = Math.hypot(xb - xa, yb - ya, w.z1 - w.z0);
    const zc = 0.5 * (w.z0 + w.z1), [ux, uy] = air(zc), V = Math.hypot(ux, uy) || 1e-6, dx = ux / V, dy = uy / V;
    let nx = -dy, ny = dx; if (w.nDir) { if (nx * w.nDir[0] + ny * w.nDir[1] < 0) { nx = -nx; ny = -ny; } } else if (nx < 0 || (Math.abs(nx) < 1e-9 && ny < 0)) { nx = -nx; ny = -ny; }
    const s = (Math.abs(nx - dy) + Math.abs(ny + dx) < 1e-9) ? 1 : -1; // V×ẑ = [Vy, −Vx]
    const box = w.loading === 'box', shape = (e) => box ? 0.55 + 0.45 * Math.sqrt(Math.max(0, 1 - e * e)) : Math.sqrt(Math.max(0, 1 - e * e)), meanShape = box ? 0.55 + 0.45 * Math.PI / 4 : Math.PI / 4;
    const G0 = w.L / (rho * (Va || V) * len * meanShape);
    const P = (e) => [xa + (xb - xa) * e, ya + (yb - ya) * e, w.z0 + (w.z1 - w.z0) * e];
    for (let i = 0; i < nSeg; i++) {
      const e0 = i / nSeg, e1 = (i + 1) / nSeg, G = s * G0 * shape((e0 + e1) / 2), A = P(e0), B = P(e1), last = i === nSeg - 1;
      add(A[0], A[1], A[2], B[0], B[1], B[2], G);
      semis.push([A[0], A[1], A[2], dx, dy, 0, -G]); if (ground) semis.push([A[0], A[1], -A[2], dx, dy, 0, G]);
      if (last && w.closure) { const C = w.closure; add(B[0], B[1], B[2], C[0], C[1], C[2], G); semis.push([C[0], C[1], C[2], dx, dy, 0, G]); if (ground) semis.push([C[0], C[1], -C[2], dx, dy, 0, -G]); }
      else { semis.push([B[0], B[1], B[2], dx, dy, 0, G]); if (ground) semis.push([B[0], B[1], -B[2], dx, dy, 0, -G]); }
    }
    w.G0 = G0; w.n = [nx, ny]; w.sign = s;
  }
  for (const b of body) { const [ux, uy] = air(b.cz); const k = 2 * Math.PI * b.a ** 3 * b.f; doub.push([b.x, 0, b.cz, k * ux, k * uy, 0, b.a], [b.x, 0, -b.cz, k * ux, k * uy, 0, b.a]); }
  const out = [0, 0, 0];
  const vel = (x, y, z) => {
    const [ux, uy] = air(Math.max(z, 0.02)); out[0] = ux; out[1] = uy; out[2] = 0;
    for (const q of segs) segVel(out, q[0], q[1], q[2], q[3], q[4], q[5], q[6], x, y, z, rc);
    for (const q of semis) semiVel(out, q[0], q[1], q[2], q[3], q[4], q[5], q[6], x, y, z, rc);
    for (const q of doub) doubletVel(out, q[0], q[1], q[2], q[3], q[4], q[5], x, y, z, q[6]);
    const sp = Math.hypot(out[0], out[1], out[2]), lim = 3 * (Va || 5); if (sp > lim) { const f = lim / sp; out[0] *= f; out[1] *= f; out[2] *= f; }
    return [out[0], out[1], z < 0.02 ? 0 : out[2]];
  };
  return { vel, wings, nSegments: segs.length + semis.length, nDoublets: doub.length };
}

// Sample a velocity function on a grid → trilinear sampler (fast for thousands of tracers)
export function sampledField(vel, bounds, dims = [29, 23, 12]) {
  const [nx, ny, nz] = dims, [[x0, x1], [y0, y1], [z0, z1]] = bounds, N = nx * ny * nz;
  const U = new Float32Array(N), V = new Float32Array(N), W = new Float32Array(N), hx = (x1 - x0) / (nx - 1), hy = (y1 - y0) / (ny - 1), hz = (z1 - z0) / (nz - 1);
  let vmax = 0;
  for (let k = 0; k < nz; k++) for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) { const v = vel(x0 + i * hx, y0 + j * hy, z0 + k * hz), id = (k * ny + j) * nx + i; U[id] = v[0]; V[id] = v[1]; W[id] = v[2]; vmax = Math.max(vmax, Math.hypot(v[0], v[1], v[2])); }
  const sample = (x, y, z) => {
    const fx = Math.min(nx - 1.001, Math.max(0, (x - x0) / hx)), fy = Math.min(ny - 1.001, Math.max(0, (y - y0) / hy)), fz = Math.min(nz - 1.001, Math.max(0, (z - z0) / hz));
    const i = fx | 0, j = fy | 0, k = fz | 0, tx = fx - i, ty = fy - j, tz = fz - k, o = [0, 0, 0];
    for (let c = 0; c < 8; c++) { const di = c & 1, dj = (c >> 1) & 1, dk = c >> 2, w = (di ? tx : 1 - tx) * (dj ? ty : 1 - ty) * (dk ? tz : 1 - tz), id = ((k + dk) * ny + (j + dj)) * nx + (i + di); o[0] += w * U[id]; o[1] += w * V[id]; o[2] += w * W[id]; }
    return o;
  };
  return { sample, vmax, dims, bounds };
}

// Monocoque doublet stations from the CAD section law (mirrors cad.vehicleAssembly)
export function bodyStations(veh, n = 7) {
  const L = veh.length, W = veh.width, Hb = veh.height, R = veh.wheelRadius, out = [];
  for (let i = 0; i < n; i++) {
    const u = (i + 0.5) / n, x = u * L - L * 0.42;
    const halfW = W / 2 * Math.max(0.06, Math.sin(Math.PI * Math.min(1, u * 1.05)) ** 0.55) * (u > 0.9 ? 1 - (u - 0.9) * 4 : 1), halfH = Hb * 0.36 * Math.max(0.06, Math.sin(Math.PI * u) ** 0.7);
    const a = Math.sqrt(Math.max(0.02, halfW * 0.82) * Math.max(0.02, halfH)), spacing = L / n;
    out.push({ x, cz: R + 0.02 + halfH * 0.9, a, f: Math.min(1, spacing / (2 * a)) });
  }
  return out;
}
