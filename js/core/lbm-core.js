// D2Q9 lattice-Boltzmann core (BGK collision with Smagorinsky sub-grid eddy viscosity), shared by the
// live CFD worker and the verification benchmarks. Lattice units: dx = dt = 1, cs² = 1/3.
// Fused collide-and-push kernel with a precomputed link table (destination index per population).
export const CX = [0, 1, 0, -1, 0, 1, -1, -1, 1], CY = [0, 0, 1, 0, -1, 1, 1, -1, -1], W = [4 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 36, 1 / 36, 1 / 36, 1 / 36], OPP = [0, 3, 4, 1, 2, 7, 8, 5, 6];
const SPEC = [0, 1, 4, 3, 2, 8, 7, 6, 5]; // specular reflection about a horizontal wall
// link types: 0 stream/reflect (no force), 1 bounce-back on obstacle (momentum exchange), 2 moving-ground bounce-back, 3 dropped (inlet/outlet rebuilt)

export function createLattice(nx, ny, { tau = 0.6, Cs = 0, u0 = 0.08, solid, movingGround = false, inletOutlet = true, slipTopBottom = true, force = [0, 0], periodicX = false, wallsY = false, f32 = false } = {}) {
  const N = nx * ny, Arr = f32 ? Float32Array : Float64Array, f = new Arr(9 * N), ft = new Arr(9 * N), rho = new Float64Array(N).fill(1), ux = new Float64Array(N), uy = new Float64Array(N);
  const S = solid || new Uint8Array(N);
  const L = { nx, ny, N, f, ft, rho, ux, uy, solid: S, tau, Cs, u0, movingGround, inletOutlet, slipTopBottom, force, periodicX, wallsY, Fx: 0, Fy: 0, step: 0, residual: 1 };
  buildLinks(L);
  for (let i = 0; i < N; i++) { const u = S[i] ? 0 : (inletOutlet ? u0 : 0); ux[i] = u; equilibriumAt(f, i, N, 1, u, 0); }
  return L;
}
function buildLinks(L) {
  const { nx, ny, N, solid } = L, dest = new Int32Array(9 * N), type = new Uint8Array(9 * N);
  for (let y = 0; y < ny; y++) for (let x = 0; x < nx; x++) {
    const i = y * nx + x;
    for (let k = 0; k < 9; k++) {
      const q = k * N + i; let xn = x + CX[k], yn = y + CY[k], t = 0, d;
      if (L.periodicX) xn = (xn + nx) % nx;
      if (yn < 0 || yn >= ny) {
        if (L.wallsY) { d = OPP[k] * N + i; t = 0; }
        else if (L.movingGround && yn < 0) { d = OPP[k] * N + i; t = 2; }
        else if (L.slipTopBottom) { const xs = Math.min(nx - 1, Math.max(0, xn)); d = SPEC[k] * N + y * nx + xs; t = 0; }
        else { d = k * N + ((yn + ny) % ny) * nx + Math.min(nx - 1, Math.max(0, xn)); }
      } else if (xn < 0 || xn >= nx) { d = -1; t = 3; }
      else { const j = yn * nx + xn; if (solid[j]) { d = OPP[k] * N + i; t = 1; } else d = k * N + j; }
      dest[q] = d; type[q] = t;
    }
  }
  L.dest = dest; L.type = type;
}
function equilibriumAt(f, i, N, r, u, v) { const uu = 1.5 * (u * u + v * v); for (let k = 0; k < 9; k++) { const cu = 3 * (CX[k] * u + CY[k] * v); f[k * N + i] = W[k] * r * (1 + cu + 0.5 * cu * cu - uu); } }

const fe = new Float64Array(9), fl = new Float64Array(9);
export function step(L) {
  const { nx, ny, N, f, ft, rho, ux, uy, solid, tau, Cs, u0, dest, type } = L; const gx = L.force[0], gy = L.force[1], forced = gx !== 0 || gy !== 0;
  let du = 0, un = 0, Fx = 0, Fy = 0;
  for (let i = 0; i < N; i++) {
    if (solid[i]) continue;
    let r = 0, mx = 0, my = 0;
    for (let k = 0; k < 9; k++) { const v = f[k * N + i]; fl[k] = v; r += v; mx += CX[k] * v; my += CY[k] * v; }
    const u = (mx + 0.5 * gx) / r, v = (my + 0.5 * gy) / r;
    const dux = u - ux[i], duy = v - uy[i]; du += dux * dux + duy * duy; un += u * u + v * v;
    rho[i] = r; ux[i] = u; uy[i] = v;
    const uu = 1.5 * (u * u + v * v);
    for (let k = 0; k < 9; k++) { const cu = 3 * (CX[k] * u + CY[k] * v); fe[k] = W[k] * r * (1 + cu + 0.5 * cu * cu - uu); }
    let te = tau;
    if (Cs > 0) { let pxx = 0, pyy = 0, pxy = 0; for (let k = 0; k < 9; k++) { const n = fl[k] - fe[k]; pxx += CX[k] * CX[k] * n; pyy += CY[k] * CY[k] * n; pxy += CX[k] * CY[k] * n; } const Q = Math.sqrt(pxx * pxx + pyy * pyy + 2 * pxy * pxy); te = 0.5 * (tau + Math.sqrt(tau * tau + 18 * Math.SQRT2 * Cs * Cs * Q / r)); }
    const om = 1 / te, om1 = 1 - om;
    for (let k = 0; k < 9; k++) {
      let fo = fl[k] * om1 + om * fe[k];
      if (forced) fo += (1 - 0.5 * om) * W[k] * (3 * ((CX[k] - u) * gx + (CY[k] - v) * gy) + 9 * (CX[k] * u + CY[k] * v) * (CX[k] * gx + CY[k] * gy));
      const q = k * N + i, d = dest[q], t = type[q];
      if (t === 0) ft[d] = fo;
      else if (t === 1) { ft[d] = fo; Fx += 2 * CX[k] * fo; Fy += 2 * CY[k] * fo; }
      else if (t === 2) ft[d] = fo - 6 * W[k] * CX[k] * u0;
    }
  }
  if (L.inletOutlet) for (let y = 0; y < ny; y++) {
    const i = y * nx; if (!solid[i]) equilibriumAt(ft, i, N, 1, u0, 0);
    const o = y * nx + nx - 1, o1 = o - 1; for (let k = 0; k < 9; k++) ft[k * N + o] = ft[k * N + o1];
  }
  L.f = ft; L.ft = f; L.Fx = Fx; L.Fy = Fy; L.step++; L.residual = Math.sqrt(du / (un || 1e-30));
  return L;
}

// Solid mask from a closed polygon (lattice coordinates)
export function maskPolygon(nx, ny, poly, solid = new Uint8Array(nx * ny)) {
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity; for (const [x, y] of poly) { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); }
  for (let y = Math.max(0, Math.floor(y0)); y <= Math.min(ny - 1, Math.ceil(y1)); y++) for (let x = Math.max(0, Math.floor(x0)); x <= Math.min(nx - 1, Math.ceil(x1)); x++) if (inside(poly, x, y)) solid[y * nx + x] = 1;
  return solid;
}
export function inside(poly, x, y) { let c = false; for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) { const [xi, yi] = poly[i], [xj, yj] = poly[j]; if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) c = !c; } return c; }

// Poiseuille benchmark: body-force-driven channel flow between no-slip walls (half-way bounce-back).
export function poiseuille({ ny = 21, tau = 0.8, g = 1e-6, steps = 6000 } = {}) {
  const nx = 4, L = createLattice(nx, ny, { tau, u0: 0, inletOutlet: false, slipTopBottom: false, wallsY: true, periodicX: true, force: [g, 0] });
  for (let s = 0; s < steps; s++) step(L);
  const nu = (tau - 0.5) / 3, H = ny;
  const yv = [], num = [], exact = [];
  for (let y = 0; y < ny; y++) { const yy = y + 0.5; yv.push(yy); num.push(L.ux[y * nx + 1]); exact.push(g / (2 * nu) * yy * (H - yy)); }
  let e2 = 0, n2 = 0; for (let k = 0; k < ny; k++) { e2 += (num[k] - exact[k]) ** 2; n2 += exact[k] ** 2; }
  return { y: yv, num, exact, relL2: Math.sqrt(e2 / n2), nu, umax: Math.max(...exact) };
}
