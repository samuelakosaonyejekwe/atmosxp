// D2Q9 lattice-Boltzmann core (BGK collision with Smagorinsky sub-grid eddy viscosity), shared by the
// live CFD worker and the verification benchmarks. Lattice units: dx = dt = 1, cs² = 1/3.
export const CX = [0, 1, 0, -1, 0, 1, -1, -1, 1], CY = [0, 0, 1, 0, -1, 1, 1, -1, -1], W = [4 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 36, 1 / 36, 1 / 36, 1 / 36], OPP = [0, 3, 4, 1, 2, 7, 8, 5, 6];

export function createLattice(nx, ny, { tau = 0.6, Cs = 0, u0 = 0.08, solid, movingGround = false, inletOutlet = true, slipTopBottom = true, force = [0, 0], periodicX = false, wallsY = false } = {}) {
  const N = nx * ny, f = new Float64Array(9 * N), ft = new Float64Array(9 * N), rho = new Float64Array(N).fill(1), ux = new Float64Array(N), uy = new Float64Array(N);
  const S = solid || new Uint8Array(N);
  const L = { nx, ny, N, f, ft, rho, ux, uy, solid: S, tau, Cs, u0, movingGround, inletOutlet, slipTopBottom, force, periodicX, wallsY, Fx: 0, Fy: 0, step: 0, residual: 1 };
  for (let i = 0; i < N; i++) { const u = S[i] ? 0 : (inletOutlet ? u0 : 0); ux[i] = u; equilibriumAt(f, i, N, 1, u, 0); }
  return L;
}
function equilibriumAt(f, i, N, r, u, v) { const uu = 1.5 * (u * u + v * v); for (let k = 0; k < 9; k++) { const cu = 3 * (CX[k] * u + CY[k] * v); f[k * N + i] = W[k] * r * (1 + cu + 0.5 * cu * cu - uu); } }

// One collide–stream step. Returns L with updated macroscopic fields and momentum-exchange force on solids.
export function step(L) {
  const { nx, ny, N, f, ft, rho, ux, uy, solid, tau, Cs, u0 } = L; const fx0 = L.force[0], fy0 = L.force[1];
  let du = 0, un = 0;
  // collision (and macroscopic moments)
  for (let i = 0; i < N; i++) {
    if (solid[i]) continue;
    let r = 0, mx = 0, my = 0; for (let k = 0; k < 9; k++) { const v = f[k * N + i]; r += v; mx += CX[k] * v; my += CY[k] * v; }
    const u = mx / r + fx0 * 0.5 / r, v = my / r + fy0 * 0.5 / r; // Guo half-force shift
    du += (u - ux[i]) ** 2 + (v - uy[i]) ** 2; un += u * u + v * v;
    rho[i] = r; ux[i] = u; uy[i] = v;
    const uu = 1.5 * (u * u + v * v); let te = tau;
    if (Cs > 0) { // Smagorinsky: local effective relaxation time from non-equilibrium momentum flux
      let pxx = 0, pyy = 0, pxy = 0;
      for (let k = 0; k < 9; k++) { const cu = 3 * (CX[k] * u + CY[k] * v), fe = W[k] * r * (1 + cu + 0.5 * cu * cu - uu), fn = f[k * N + i] - fe; pxx += CX[k] * CX[k] * fn; pyy += CY[k] * CY[k] * fn; pxy += CX[k] * CY[k] * fn; }
      const Q = Math.sqrt(pxx * pxx + pyy * pyy + 2 * pxy * pxy); te = 0.5 * (tau + Math.sqrt(tau * tau + 18 * Math.SQRT2 * Cs * Cs * Q / r));
    }
    const om = 1 / te;
    for (let k = 0; k < 9; k++) {
      const cu = 3 * (CX[k] * u + CY[k] * v), fe = W[k] * r * (1 + cu + 0.5 * cu * cu - uu);
      let fo = f[k * N + i] * (1 - om) + om * fe;
      if (fx0 || fy0) fo += (1 - 0.5 * om) * W[k] * (3 * ((CX[k] - u) * fx0 + (CY[k] - v) * fy0) + 9 * (CX[k] * u + CY[k] * v) * (CX[k] * fx0 + CY[k] * fy0));
      f[k * N + i] = fo;
    }
  }
  // streaming with half-way bounce-back on solids (momentum exchange) and boundary treatment
  let Fx = 0, Fy = 0;
  for (let y = 0; y < ny; y++) for (let x = 0; x < nx; x++) {
    const i = y * nx + x; if (solid[i]) continue;
    for (let k = 0; k < 9; k++) {
      let xn = x + CX[k], yn = y + CY[k];
      if (L.periodicX) xn = (xn + nx) % nx;
      if (yn < 0 || yn >= ny) {
        if (L.wallsY) { ft[OPP[k] * N + i] = f[k * N + i]; continue; } // no-slip channel wall (bounce-back)
        if (L.movingGround && yn < 0) { ft[OPP[k] * N + i] = f[k * N + i] - 6 * W[k] * 1 * CX[k] * u0; continue; } // moving wall at u0
        if (L.slipTopBottom) { const ks = [0, 1, 4, 3, 2, 8, 7, 6, 5][k]; ft[ks * N + i] = f[k * N + i]; continue; } // specular reflection
        yn = (yn + ny) % ny;
      }
      if (xn < 0 || xn >= nx) continue; // handled by inlet/outlet below
      const j = yn * nx + xn;
      if (solid[j]) { const fk = f[k * N + i]; ft[OPP[k] * N + i] = fk; Fx += 2 * CX[k] * fk; Fy += 2 * CY[k] * fk; }
      else ft[k * N + j] = f[k * N + i];
    }
  }
  if (L.inletOutlet) {
    for (let y = 0; y < ny; y++) { const i = y * nx; if (!solid[i]) { for (let k = 0; k < 9; k++) ft[k * N + i] = 0; equilibriumAt(ft, i, N, 1, u0, 0); } const o = y * nx + nx - 1, o1 = o - 1; for (let k = 0; k < 9; k++) ft[k * N + o] = ft[k * N + o1]; }
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
  const nu = (tau - 0.5) / 3, H = ny; // walls half-way outside first/last nodes
  const yv = [], num = [], exact = [];
  for (let y = 0; y < ny; y++) { const yy = y + 0.5; yv.push(yy); num.push(L.ux[y * nx + 1]); exact.push(g / (2 * nu) * yy * (H - yy)); }
  let e2 = 0, n2 = 0; for (let k = 0; k < ny; k++) { e2 += (num[k] - exact[k]) ** 2; n2 += exact[k] ** 2; }
  return { y: yv, num, exact, relL2: Math.sqrt(e2 / n2), nu, umax: Math.max(...exact) };
}
