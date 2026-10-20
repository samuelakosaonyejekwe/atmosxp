// Live 2-D lattice-Boltzmann (D2Q9, BGK + Smagorinsky LES) flow solver running in a module Web Worker.
import { createLattice, step, maskPolygon } from '../core/lbm-core.js';

let L = null, cfg = null, running = false, hist = { t: [], cd: [], cl: [], res: [] };

function setup(c) {
  cfg = c; const { nx, ny } = c;
  const solid = new Uint8Array(nx * ny);
  maskPolygon(nx, ny, c.poly, solid);
  if (c.ground) for (let x = 0; x < nx; x++) solid[x] = 0; // ground handled as moving wall boundary
  const nu = c.u0 * c.Lref / c.Re, tau = 3 * nu + 0.5;
  L = createLattice(nx, ny, { tau, Cs: c.Cs, u0: c.u0, solid, movingGround: !!c.ground, slipTopBottom: true });
  hist = { t: [], cd: [], cl: [], res: [] };
  postMessage({ type: 'ready', tau, nu, mach: c.u0 * Math.sqrt(3), solidCells: solid.reduce((s, v) => s + v, 0) });
}

function fields() {
  const { nx, ny, N, ux, uy, rho, solid } = L, speed = new Float32Array(N), vort = new Float32Array(N), pres = new Float32Array(N);
  for (let y = 1; y < ny - 1; y++) for (let x = 1; x < nx - 1; x++) {
    const i = y * nx + x; if (solid[i]) { speed[i] = NaN; vort[i] = NaN; pres[i] = NaN; continue; }
    speed[i] = Math.hypot(ux[i], uy[i]) / cfg.u0; pres[i] = (rho[i] - 1) / 3 / (0.5 * cfg.u0 * cfg.u0);
    vort[i] = ((uy[i + 1] - uy[i - 1]) - (ux[i + nx] - ux[i - nx])) * 0.5 * cfg.Lref / cfg.u0;
  }
  return { speed, vort, pres };
}

// passive tracer particles (no feedback on the flow): advected with the lattice velocity each step, re-seeded at the inlet when
// they leave the domain or enter a solid; each keeps a short trail so the frame shows streaklines of the evolving wake
const NP = 1000, TRAIL = 4, P = { x: new Float32Array(NP), y: new Float32Array(NP), tr: new Float32Array(NP * TRAIL * 2), k: 0 };
const seed = (j) => { P.x[j] = 2 + Math.random() * 3; P.y[j] = 2 + Math.random() * (L.ny - 4); for (let t = 0; t < TRAIL; t++) { P.tr[(j * TRAIL + t) * 2] = P.x[j]; P.tr[(j * TRAIL + t) * 2 + 1] = P.y[j]; } };
let seeded = false;
function advect() {
  const { nx, ny, ux, uy, solid } = L; if (!seeded) { for (let j = 0; j < NP; j++) { seed(j); P.x[j] = 2 + Math.random() * (nx - 4); } seeded = true; }
  for (let j = 0; j < NP; j++) { const x = P.x[j], y = P.y[j], i0 = Math.floor(x), j0 = Math.floor(y); if (i0 < 1 || i0 >= nx - 2 || j0 < 1 || j0 >= ny - 2) { seed(j); continue; }
    const fx = x - i0, fy = y - j0, a = j0 * nx + i0, b = a + 1, c = a + nx, d = c + 1; if (solid[a] || solid[b] || solid[c] || solid[d]) { seed(j); continue; }
    P.x[j] = x + (ux[a] * (1 - fx) * (1 - fy) + ux[b] * fx * (1 - fy) + ux[c] * (1 - fx) * fy + ux[d] * fx * fy);
    P.y[j] = y + (uy[a] * (1 - fx) * (1 - fy) + uy[b] * fx * (1 - fy) + uy[c] * (1 - fx) * fy + uy[d] * fx * fy); }
}
function snapTrail() { for (let j = 0; j < NP; j++) { const t = P.k % TRAIL; P.tr[(j * TRAIL + t) * 2] = P.x[j]; P.tr[(j * TRAIL + t) * 2 + 1] = P.y[j]; } P.k++; }
function loop() {
  if (!running) return;
  const t0 = performance.now(); let n = 0;
  while (performance.now() - t0 < 40) { step(L); advect(); n++; if (n % 4 === 0) snapTrail(); }
  const q = 0.5 * cfg.u0 * cfg.u0 * cfg.Lref;
  hist.t.push(L.step); hist.cd.push(L.Fx / q); hist.cl.push(L.Fy / q); hist.res.push(L.residual);
  if (hist.t.length > 600) for (const k in hist) hist[k].shift();
  const f = fields();
  const tr = P.tr.slice(); postMessage({ type: 'frame', step: L.step, mlups: L.N * n / ((performance.now() - t0) * 1e3), CD: L.Fx / q, CL: L.Fy / q, residual: L.residual, hist, ...f, tracers: { tr, n: NP, trail: TRAIL, head: P.k % TRAIL } }, [f.speed.buffer, f.vort.buffer, f.pres.buffer, tr.buffer]);
  setTimeout(loop, 0);
}

onmessage = (e) => {
  const m = e.data;
  if (m.type === 'init') { running = false; setup(m.cfg); }
  else if (m.type === 'run') { if (!running && L) { running = true; loop(); } }
  else if (m.type === 'pause') running = false;
  else if (m.type === 'set') { if (L && m.Cs !== undefined) L.Cs = m.Cs; }
};
