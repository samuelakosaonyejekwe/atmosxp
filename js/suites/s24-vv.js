// Suite 24 — Calibration, verification, validation and scientific experimental methodology.
import { SparseBuilder, pcg, thomas, rk45, eigGen, zeros, levenbergMarquardt, metrics, gci, linspace, range, rng, fmt, mean, max, min, sum, SIGMA, normInv } from '../core/num.js';
import { planck } from '../core/physics.js';
import { poiseuille } from '../core/lbm-core.js';
import { panelSolver } from '../core/aero.js';
import { naca4 } from '../core/cad.js';
import { resampleAirfoil } from '../core/mesh.js';
import { CREDIBILITY } from '../core/state.js';
import { SUITES } from './index.js';
import { h, table } from '../core/ui.js';

// ---------- verification benchmarks ----------
function mmsPoisson(n) { // −∇²u = f on (0,1)², u = sin(πx)sin(2πy) + x²y, 5-point FD, n interior points per side
  const hh = 1 / (n + 1), ue = (x, y) => Math.sin(Math.PI * x) * Math.sin(2 * Math.PI * y) + x * x * y, f = (x, y) => 5 * Math.PI * Math.PI * Math.sin(Math.PI * x) * Math.sin(2 * Math.PI * y) - 2 * y;
  const N = n * n, A = new SparseBuilder(N), b = new Float64Array(N), id = (i, j) => j * n + i;
  for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) {
    const x = (i + 1) * hh, y = (j + 1) * hh, k = id(i, j); A.add(k, k, 4 / hh ** 2); b[k] = f(x, y);
    for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) { const ii = i + di, jj = j + dj; if (ii < 0 || jj < 0 || ii >= n || jj >= n) b[k] += ue((ii + 1) * hh, (jj + 1) * hh) / hh ** 2; else A.add(k, id(ii, jj), -1 / hh ** 2); }
  }
  const s = pcg(A.toCSR(), b, { tol: 1e-12, maxIt: 20000 }); let e2 = 0, einf = 0;
  for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) { const e = Math.abs(s.x[id(i, j)] - ue((i + 1) * hh, (j + 1) * hh)); e2 += e * e; einf = Math.max(einf, e); }
  return { h: hh, l2: Math.sqrt(e2 / N), linf: einf, it: s.it, centre: s.x[id((n + 1) / 4 - 1, (n + 1) / 4 - 1)] }; // probe at (¼,¼), a node for n = 2^k−1
}
function heat1D(scheme, nt, nx = 200, alpha = 0.1, T = 0.5) { // u_t = α u_xx, u = e^{−απ²t} sin πx
  const dx = 1 / (nx + 1), dt = T / nt, r = alpha * dt / dx ** 2, th = scheme === 'CN' ? 0.5 : 1;
  let u = Float64Array.from(range(nx), i => Math.sin(Math.PI * (i + 1) * dx));
  const a = new Float64Array(nx).fill(-th * r), b = new Float64Array(nx).fill(1 + 2 * th * r), c = new Float64Array(nx).fill(-th * r);
  for (let s = 0; s < nt; s++) { const d = Float64Array.from(u, (v, i) => v + (1 - th) * r * ((i ? u[i - 1] : 0) - 2 * v + (i < nx - 1 ? u[i + 1] : 0))); u = thomas(a, b, c, d); }
  // compare with the exact solution of the semi-discrete system (discrete Laplacian eigenvalue) so only temporal error remains
  const lh = 4 / dx ** 2 * Math.sin(Math.PI * dx / 2) ** 2; let e = 0; for (let i = 0; i < nx; i++) e = Math.max(e, Math.abs(u[i] - Math.exp(-alpha * lh * T) * Math.sin(Math.PI * (i + 1) * dx)));
  return { dt, err: e };
}
function cantilever(nel, { E = 70e9, I = 1e-6, rhoA = 5, L = 2, P = 1000 } = {}) { // Euler–Bernoulli Hermite elements
  const le = L / nel, nd = 2 * (nel + 1), K = zeros(nd, nd), M = zeros(nd, nd), F = new Float64Array(nd);
  const k = E * I / le ** 3, ke = [[12, 6 * le, -12, 6 * le], [6 * le, 4 * le * le, -6 * le, 2 * le * le], [-12, -6 * le, 12, -6 * le], [6 * le, 2 * le * le, -6 * le, 4 * le * le]];
  const m = rhoA * le / 420, me = [[156, 22 * le, 54, -13 * le], [22 * le, 4 * le * le, 13 * le, -3 * le * le], [54, 13 * le, 156, -22 * le], [-13 * le, -3 * le * le, -22 * le, 4 * le * le]];
  for (let e = 0; e < nel; e++) { const dofs = [2 * e, 2 * e + 1, 2 * e + 2, 2 * e + 3]; for (let a = 0; a < 4; a++) for (let b = 0; b < 4; b++) { K[dofs[a]][dofs[b]] += k * ke[a][b]; M[dofs[a]][dofs[b]] += m * me[a][b]; } }
  F[nd - 2] = P; const free = range(nd).slice(2), Kf = free.map(i => Float64Array.from(free.map(j => K[i][j]))), Mf = free.map(i => Float64Array.from(free.map(j => M[i][j])));
  // static solve via Cholesky-free Gauss (small)
  const A = Kf.map((r2, i) => [...r2, F[free[i]]]); const n = A.length;
  for (let c = 0; c < n; c++) { for (let r2 = c + 1; r2 < n; r2++) { const f = A[r2][c] / A[c][c]; for (let j = c; j <= n; j++) A[r2][j] -= f * A[c][j]; } }
  const x = new Array(n); for (let i = n - 1; i >= 0; i--) { let s = A[i][n]; for (let j = i + 1; j < n; j++) s -= A[i][j] * x[j]; x[i] = s / A[i][i]; }
  const ev = eigGen(Kf, Mf), w1 = Math.sqrt(ev.values[0]);
  return { tip: x[n - 2], tipExact: P * L ** 3 / (3 * E * I), f1: w1 / (2 * Math.PI), f1Exact: 1.875104 ** 2 * Math.sqrt(E * I / (rhoA * L ** 4)) / (2 * Math.PI), f2: Math.sqrt(ev.values[1]) / (2 * Math.PI), f2Exact: 4.694091 ** 2 * Math.sqrt(E * I / (rhoA * L ** 4)) / (2 * Math.PI) };
}
function oscillatorEnergy(method, dt, periods = 100) {
  const w = 2 * Math.PI, n = Math.round(periods / dt); let x = 1, v = 0, maxErr = 0, early = 0, late = 0; const E0 = 0.5 * (v * v + w * w * x * x), tr = [], er = [];
  for (let s = 0; s < n; s++) {
    if (method === 'Verlet') { v -= 0.5 * dt * w * w * x; x += dt * v; v -= 0.5 * dt * w * w * x; }
    else { const f = (X, V) => [V, -w * w * X]; const k1 = f(x, v), k2 = f(x + dt / 2 * k1[0], v + dt / 2 * k1[1]), k3 = f(x + dt / 2 * k2[0], v + dt / 2 * k2[1]), k4 = f(x + dt * k3[0], v + dt * k3[1]); x += dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]); v += dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]); }
    const e = Math.abs(0.5 * (v * v + w * w * x * x) / E0 - 1); maxErr = Math.max(maxErr, e); if (s < n / 10) early = Math.max(early, e); if (s > 0.9 * n) late = Math.max(late, e); if (s % Math.max(1, Math.floor(n / 200)) === 0) { tr.push(s * dt); er.push(e); }
  }
  return { maxErr, t: tr, e: er, growth: late / (early || 1e-30), steps: n, wh: w * dt };
}
function fresnel(n1, n2, thDeg) { const ti = thDeg * Math.PI / 180, st = n1 / n2 * Math.sin(ti); if (st >= 1) return { Rs: 1, Rp: 1, Ts: 0, Tp: 0 }; const tt = Math.asin(st), ci = Math.cos(ti), ct = Math.cos(tt); const rs = (n1 * ci - n2 * ct) / (n1 * ci + n2 * ct), rp = (n2 * ci - n1 * ct) / (n2 * ci + n1 * ct), ts = 2 * n1 * ci / (n1 * ci + n2 * ct), tp = 2 * n1 * ci / (n2 * ci + n1 * ct); const k = (n2 * ct) / (n1 * ci); return { Rs: rs * rs, Rp: rp * rp, Ts: k * ts * ts, Tp: k * tp * tp }; }

const MODELS = {
  firstOrder: { label: 'First-order step response  y = y∞ + (y₀ − y∞)·e^(−x/τ)', f: (x, p) => p[0] + (p[1] - p[0]) * Math.exp(-x / Math.max(1e-12, p[2])), names: ['y∞', 'y₀', 'τ'], init: (X, Y) => [Y[Y.length - 1], Y[0], (max(X) - min(X)) / 4] },
  linear: { label: 'Linear  y = a + b·x', f: (x, p) => p[0] + p[1] * x, names: ['a', 'b'], init: (X, Y) => [Y[0], (Y[Y.length - 1] - Y[0]) / ((X[X.length - 1] - X[0]) || 1)] },
  quadratic: { label: 'Quadratic  y = a + b·x + c·x²', f: (x, p) => p[0] + p[1] * x + p[2] * x * x, names: ['a', 'b', 'c'], init: (X, Y) => [Y[0], 0, 0] },
  power: { label: 'Power law  y = a·x^b', f: (x, p) => p[0] * Math.pow(Math.max(1e-12, x), p[1]), names: ['a', 'b'], init: (X, Y) => [mean(Y) || 1, 1] },
  exponential: { label: 'Exponential  y = a·e^(b·x)', f: (x, p) => p[0] * Math.exp(p[1] * x), names: ['a', 'b'], init: (X, Y) => [Y[0] || 1, 0] },
};
const CRIT = { s17: 1, s15: 0.95, s13: 0.9, s20: 0.95, s14: 0.8, s06: 0.85, s07: 0.85, s08: 0.75, s18: 0.7, s19: 0.75, s09: 0.7, s12: 0.7, s11: 0.7, s10: 0.6, s23: 0.65, s04: 0.6, s16: 0.5, s05: 0.6, s21: 0.4, s22: 0.45, s26: 0.4, s01: 0.5, s02: 0.4, s03: 0.6, s24: 0.3, s25: 0.4 };

export default {
  description: 'Executes the platform’s numerical verification benchmarks (manufactured solutions, analytical flows, structural, thermal, radiative and time-integration tests) with observed orders of accuracy and grid-convergence indices; calibrates model parameters on imported measurements by weighted nonlinear least squares with identifiability diagnostics; validates on a strictly separate dataset with pre-registered acceptance criteria; and tracks the credibility level of every suite.',
  literature: 'verification validation uncertainty quantification computational model credibility manufactured solutions',
  credibility: 'Numerically verified',
  equations: ['Method of manufactured solutions on the Poisson equation (2-D, second-order finite differences)', 'Transient heat equation with backward-Euler and Crank–Nicolson time integration vs analytical eigen-solution', 'Plane Poiseuille flow (analytical parabola) vs lattice-Boltzmann solution', 'Euler–Bernoulli cantilever: tip deflection PL³/3EI and natural frequencies β₁L = 1.8751, β₂L = 4.6941 (Hermite FE, consistent mass)', 'Lumped-capacitance transient T(t) = T∞ + (T₀ − T∞)e^(−t/τ) vs adaptive Dormand–Prince integration', 'Fresnel equations (energy conservation R + T = 1) and Planck/Stefan–Boltzmann integral', 'Hamiltonian energy conservation: symplectic velocity-Verlet vs fourth-order Runge–Kutta', 'Richardson extrapolation, observed order p, grid-convergence index (Roache; Celik et al. 2008)', 'Weighted nonlinear least squares (Levenberg–Marquardt), parameter covariance, correlation and confidence intervals', 'Validation metrics: bias, RMSE, MAE, normalised RMSE, R², Pearson r, bias confidence interval, prediction intervals, uncertainty-weighted discrepancy'],
  methods: ['Each benchmark is computed live on this device; nothing is pre-stored', 'Calibration and validation subsets are disjoint and recorded', 'Acceptance thresholds are user-set before evaluation (not a universal threshold)'],
  assumptions: ['Verification establishes correct implementation, not physical correctness', 'A simulated positive result is never classified as physical validation without independent measurements'],
  ibc: { initial: ['Baseline simulation configuration and initial parameter estimates for calibration', 'Benchmark reference states (exact solutions)', 'Measurement uncertainty of imported datasets'], boundary: ['Benchmark boundary conditions reproduced exactly (Dirichlet MMS data, no-slip channel walls, clamped beam root, insulated/fixed-temperature heat boundaries)', 'Laboratory test conditions (fixtures, loading rates, temperature histories) reproduced for physical validation cases'] },
  cvv: { calibration: ['Reference datasets with measurement uncertainty', 'Repeatability / reproducibility statistics'], verification: ['All benchmarks in this suite', 'Mesh and time-step studies in Suites 02 and 07'], validation: ['Independent datasets not used for calibration; validation hierarchy Level 1 (material) → Level 6 (real environment)'] },
  formats: { import: ['CSV/TSV/TXT/DAT', 'XLSX/ODS', 'JSON'], export: ['Verification report JSON/CSV', 'Calibration results JSON', 'PNG/SVG plots', 'Evidence package ZIP'] },
  params: [
    { key: 'mmsMax', label: 'MMS finest grid (points/side)', type: 'select', default: 64, options: [32, 64, 96], group: 'Verification benchmarks', expert: true },
    { key: 'dataset', label: 'Calibration dataset', type: 'select', default: 'auto', options: [{ value: 'auto', label: 'First imported dataset (else synthetic demo)' }, { value: 'demo', label: 'Synthetic demonstration dataset' }], group: 'Calibration & validation', help: 'Import measured data with “Import experimental data”.' },
    { key: 'xcol', label: 'x column (blank = auto)', type: 'text', default: '', group: 'Calibration & validation' },
    { key: 'ycol', label: 'y column (blank = auto)', type: 'text', default: '', group: 'Calibration & validation' },
    { key: 'model', label: 'Model form', type: 'select', default: 'firstOrder', options: Object.entries(MODELS).map(([k, v]) => ({ value: k, label: v.label })), group: 'Calibration & validation' },
    { key: 'split', label: 'Calibration fraction', type: 'range', min: 0.3, max: 0.9, step: 0.05, default: 0.6, group: 'Calibration & validation' },
    { key: 'splitMode', label: 'Split method', type: 'select', default: 'interleaved', options: [{ value: 'interleaved', label: 'Interleaved (every k-th point to validation)' }, { value: 'block', label: 'Block (extrapolation test: last part validates)' }], group: 'Calibration & validation' },
    { key: 'sigma', label: 'Measurement standard uncertainty', type: 'number', min: 0, max: 1e6, step: 0.01, default: 0.4, group: 'Calibration & validation', help: 'Used for weights and uncertainty-weighted discrepancy.' },
    { key: 'accept', label: 'Pre-registered acceptance: NRMSE', unit: '%', type: 'number', min: 0.1, max: 50, step: 0.1, default: 5, group: 'Calibration & validation' },
  ],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] }, B = [];
    ctx.progress(0.05, 'MMS');
    // 1. MMS
    const ns = [8, 16, 32, +p.mmsMax].filter((v, i, a) => a.indexOf(v) === i).sort((a, b) => a - b).map(m => m - 1), mm = []; // n = 2^k−1 keeps (½,½) on a node
    for (const n of ns) { mm.push(mmsPoisson(n)); await ctx.yield(); }
    const pObs = mm.slice(1).map((m, i) => Math.log(mm[i].l2 / m.l2) / Math.log(mm[i].h / m.h)), pLast = pObs[pObs.length - 1];
    B.push({ cat: 'Code verification', name: 'MMS Poisson 2-D: observed order (L2)', value: pLast, target: '2 ± 0.1', pass: Math.abs(pLast - 2) < 0.1 });
    const g3 = gci(mm[mm.length - 1].centre, mm[mm.length - 2].centre, mm[mm.length - 3].centre, mm[mm.length - 1].h, mm[mm.length - 2].h, mm[mm.length - 3].h);
    B.push({ cat: 'Solution verification', name: 'GCI of probe value u(¼,¼) (Richardson)', value: g3.gci21, target: '< 1 %, asymptotic ratio ≈ 1', pass: g3.gci21 < 0.01, detail: `p = ${fmt(g3.p, 3)}, ratio ${fmt(g3.asymptoticRatio, 4)}` });
    // 2. transient heat, temporal order
    ctx.progress(0.25, 'heat');
    const nts = [10, 20, 40, 80], be = nts.map(n => heat1D('BE', n)), cn = nts.map(n => heat1D('CN', n));
    const ord = (arr) => Math.log(arr[arr.length - 2].err / arr[arr.length - 1].err) / Math.log(2);
    B.push({ cat: 'Code verification', name: 'Heat equation, backward Euler temporal order', value: ord(be), target: '1 ± 0.15', pass: Math.abs(ord(be) - 1) < 0.15 });
    B.push({ cat: 'Code verification', name: 'Heat equation, Crank–Nicolson temporal order', value: ord(cn), target: '2 ± 0.2', pass: Math.abs(ord(cn) - 2) < 0.2 });
    await ctx.yield();
    // 3. Poiseuille LBM
    ctx.progress(0.4, 'LBM');
    const pz = [11, 21, 41].map(ny => ({ ny, ...poiseuille({ ny, steps: Math.round(ny * ny * 14) }) }));
    const pOrd = Math.log(pz[1].relL2 / pz[2].relL2) / Math.log(41 / 21);
    B.push({ cat: 'Code verification', name: 'LBM Poiseuille relative L2 error (ny = 41)', value: pz[2].relL2, target: '< 0.5 %', pass: pz[2].relL2 < 0.005, detail: `observed order ${fmt(pOrd, 3)}` });
    await ctx.yield();
    // 4. cantilever beam
    ctx.progress(0.55, 'beam');
    const nels = [2, 4, 8, 16], cb = nels.map(n => cantilever(n)), last = cb[cb.length - 1];
    B.push({ cat: 'Code verification', name: 'Cantilever tip deflection vs PL³/3EI', value: last.tip / last.tipExact, target: '1.0000 ± 1e-9 (cubic exact)', pass: Math.abs(last.tip / last.tipExact - 1) < 1e-9 });
    B.push({ cat: 'Code verification', name: 'Cantilever 1st natural frequency', value: last.f1 / last.f1Exact, target: '1 ± 1e-4', pass: Math.abs(last.f1 / last.f1Exact - 1) < 1e-4 });
    B.push({ cat: 'Code verification', name: 'Cantilever 2nd natural frequency', value: last.f2 / last.f2Exact, target: '1 ± 1e-3', pass: Math.abs(last.f2 / last.f2Exact - 1) < 1e-3 });
    // 5. lumped capacitance with adaptive RK45
    const tau = 120, sol = rk45((t, y) => [-(y[0] - 50) / tau], 0, [20], 900, { rtol: 1e-9, atol: 1e-12 }), lcErr = max(sol.t.map((t, i) => Math.abs(sol.y[i][0] - (50 - 30 * Math.exp(-t / tau)))));
    B.push({ cat: 'Code verification', name: 'Lumped-capacitance transient (adaptive RK45) max error', value: lcErr, target: '< 1e-6 K', pass: lcErr < 1e-6, detail: `${sol.steps} accepted, ${sol.rejected} rejected steps` });
    // 6. radiation
    const fr = [0, 30, 60, 80].map(t => fresnel(1, 1.5, t)), frErr = max(fr.flatMap(f => [Math.abs(f.Rs + f.Ts - 1), Math.abs(f.Rp + f.Tp - 1)]));
    B.push({ cat: 'Code verification', name: 'Fresnel energy conservation R + T = 1', value: frErr, target: '< 1e-12', pass: frErr < 1e-12, detail: `R(0°, n=1.5) = ${fmt(fr[0].Rs, 4)} (exact 0.04)` });
    const lam = range(4000).map(i => 1e-7 * Math.pow(1e4, i / 3999)); let Eb = 0; for (let i = 1; i < lam.length; i++) Eb += 0.5 * (planck(lam[i], 300) + planck(lam[i - 1], 300)) * (lam[i] - lam[i - 1]); Eb *= Math.PI;
    B.push({ cat: 'Code verification', name: 'Planck integral vs Stefan–Boltzmann (300 K)', value: Eb / (SIGMA * 300 ** 4), target: '1 ± 0.5 %', pass: Math.abs(Eb / (SIGMA * 300 ** 4) - 1) < 0.005 });
    // 7. energy conservation of integrators
    const ve = oscillatorEnergy('Verlet', 0.02), rk = oscillatorEnergy('RK4', 0.02);
    B.push({ cat: 'Code verification', name: 'Symplectic Verlet: bounded, non-growing energy error', value: ve.maxErr, target: `≤ (ωΔt)²/2 = ${fmt(ve.wh ** 2 / 2, 3)}, late/early ≤ 1.5`, pass: ve.maxErr <= ve.wh ** 2 / 2 && ve.growth <= 1.5, detail: `late/early ratio ${fmt(ve.growth, 3)}` });
    const rkTheory = rk.steps * rk.wh ** 6 / 72;
    B.push({ cat: 'Code verification', name: 'RK4 secular energy drift vs theory N(ωΔt)⁶/72', value: rk.maxErr / rkTheory, target: '1 ± 0.5', pass: Math.abs(rk.maxErr / rkTheory - 1) < 0.5, detail: `drift ${fmt(rk.maxErr, 3)} after ${rk.steps} steps` });
    // 8. aerodynamic cross-check
    const ps = panelSolver((() => { const r = resampleAirfoil(naca4('0012', 161).points, 160); r.push(r[0].slice()); return r; })()), s4 = ps.solve(4);
    B.push({ cat: 'Code verification', name: 'Panel method lift slope NACA 0012 vs 2π(1+0.77t/c)', value: s4.CL / (4 * Math.PI / 180) / (2 * Math.PI * (1 + 0.77 * 0.12)), target: '1 ± 3 %', pass: Math.abs(s4.CL / (4 * Math.PI / 180) / (2 * Math.PI * (1 + 0.77 * 0.12)) - 1) < 0.03 });
    for (const b of B) out.checks.push({ name: `[${b.cat}] ${b.name}`, value: b.value, target: b.target, pass: b.pass, detail: b.detail });
    const passN = B.filter(b => b.pass).length;
    // ---------- calibration & validation ----------
    ctx.progress(0.75, 'calibration');
    const exps = (ctx.experiments || []).filter(d => d.columns);
    let ds = null, synthetic = false;
    if (p.dataset === 'auto' && exps.length) {
      const d = exps[0], numCols = d.header.filter(c => d.columns[c].some(Number.isFinite));
      const xc = p.xcol && d.columns[p.xcol] ? p.xcol : (d.map?.time && numCols.includes(d.map.time) ? d.map.time : numCols[0]), yc = p.ycol && d.columns[p.ycol] ? p.ycol : numCols.find(c => c !== xc);
      const X = [], Y = []; d.columns[xc].forEach((x, i) => { const y = d.columns[yc]?.[i]; if (Number.isFinite(x) && Number.isFinite(y)) { X.push(x); Y.push(y); } });
      if (X.length >= 6) ds = { name: d.name, X, Y, xc, yc };
    }
    if (!ds) { synthetic = true; const R = rng(2024), X = range(91).map(i => i * 10), Y = X.map(t => 48 + (22 - 48) * Math.exp(-t / 180) + R.normal(0, p.sigma || 0.4)); ds = { name: 'Synthetic demonstration (collector warm-up, τ = 180 s, σ = 0.4 K) — NOT a measurement', X, Y, xc: 't [s]', yc: 'T [°C]' }; }
    const idx = range(ds.X.length), k = Math.max(2, Math.round(1 / (1 - p.split)));
    const isVal = (i) => p.splitMode === 'block' ? i >= Math.round(p.split * ds.X.length) : (i % k === k - 1);
    const cal = idx.filter(i => !isVal(i)), val = idx.filter(isVal);
    const M = MODELS[p.model], Xc = cal.map(i => ds.X[i]), Yc = cal.map(i => ds.Y[i]), Xv = val.map(i => ds.X[i]), Yv = val.map(i => ds.Y[i]);
    let fit = null, fitErr = null;
    try { fit = levenbergMarquardt(M.f, Xc, Yc, M.init(Xc, Yc), { weights: Xc.map(() => 1 / Math.max(1e-12, p.sigma || 1) ** 2) }); } catch (e) { fitErr = e.message; }
    let mt = null, piHalf = NaN;
    if (fit) {
      const predV = Xv.map(x => M.f(x, fit.p)); mt = metrics(predV, Yv, Yv.map(() => p.sigma || 1));
      const sres = Math.sqrt(fit.residuals.reduce((s, r) => s + r * r, 0) / Math.max(1, Xc.length - fit.p.length)); piHalf = 1.96 * Math.sqrt(sres ** 2 + (p.sigma || 0) ** 2 * 0);
      const ident = fit.corr ? max(fit.corr.flatMap((r, i) => r.filter((_, j) => j !== i).map(Math.abs))) : NaN;
      out.verdicts.push({ test: 'Parameter identifiability', status: !Number.isFinite(ident) ? 'warn' : ident < 0.95 ? 'pass' : 'warn', detail: `Max |parameter correlation| = ${fmt(ident, 3)}${ident >= 0.95 ? ' — parameters are not independently identifiable from these data' : ''}.`, fix: 'Design an experiment that excites the correlated parameters separately (different loading rates or temperatures).' });
      const accepted = mt.nrmse * 100 <= p.accept;
      out.verdicts.push({ test: 'Validation against independent data', status: synthetic ? 'info' : accepted ? 'pass' : 'fail', detail: `${synthetic ? 'Synthetic demonstration only — not physical validation. ' : ''}Validation NRMSE ${fmt(mt.nrmse * 100, 3)} % vs pre-registered ${p.accept} %; bias ${fmt(mt.bias, 3)} (95 % CI ${fmt(mt.biasCI[0], 3)} … ${fmt(mt.biasCI[1], 3)}).`, fix: 'Investigate model-form error (residual trends) before re-calibrating; do not tune parameters outside defensible ranges.' });
      const xs = linspace(min(ds.X), max(ds.X), 200), ys = xs.map(x => M.f(x, fit.p));
      out.charts.push({ title: `Calibration & validation — ${M.label}`, x: { label: ds.xc }, y: { label: ds.yc }, series: [{ name: 'Calibration data', x: Xc, y: Yc, mode: 'markers', color: '#3b82f6' }, { name: 'Validation data (held out)', x: Xv, y: Yv, mode: 'markers', color: '#f59e0b', r: 4 }, { name: 'Calibrated model ± 95 % prediction band', x: xs, y: ys, color: '#10b981', band: { lo: ys.map(y => y - piHalf), hi: ys.map(y => y + piHalf) } }], wide: true, explain: 'The model is fitted only to the blue points; the orange points were never seen by the fit and test its predictive capability.' });
      out.charts.push({ title: 'Validation residuals (measured − predicted)', type: 'scatter', x: { label: ds.xc }, y: { label: 'Residual' }, series: [{ name: 'validation residual', x: Xv, y: Xv.map((x, i) => Yv[i] - M.f(x, fit.p)) }, { name: 'calibration residual', x: Xc, y: fit.residuals, r: 2 }], annotations: [{ type: 'hline', y: 0, color: '#94a3b8' }, { type: 'band', y0: -2 * (p.sigma || 0), y1: 2 * (p.sigma || 0), color: '#22c55e' }], explain: 'Residuals should scatter randomly inside ±2σ of measurement uncertainty; trends indicate model-form error.' });
      out.tables.push({ title: 'Calibrated parameters', cols: ['Parameter', 'Estimate', 'Std. error', '95 % CI low', '95 % CI high'], rows: M.names.map((n, i) => [n, fit.p[i], fit.se[i], fit.p[i] - 1.96 * fit.se[i], fit.p[i] + 1.96 * fit.se[i]]) });
      if (fit.corr) out.tables.push({ title: 'Parameter correlation matrix', cols: ['', ...M.names], rows: fit.corr.map((r, i) => [M.names[i], ...r]) });
      out.tables.push({ title: 'Validation metrics (independent subset)', cols: ['Metric', 'Value'], rows: [['Calibration points', Xc.length], ['Validation points', Xv.length], ['Bias', mt.bias], ['RMSE', mt.rmse], ['MAE', mt.mae], ['NRMSE', fmt(mt.nrmse * 100, 4) + ' %'], ['R²', mt.r2], ['Pearson r', mt.r], ['Bias 95 % CI', `${fmt(mt.biasCI[0])} … ${fmt(mt.biasCI[1])}`], ['Uncertainty-weighted discrepancy', mt.uwd]] });
    } else out.verdicts.push({ test: 'Calibration', status: 'fail', detail: 'Fit failed: ' + fitErr });
    // ---------- credibility ladder ----------
    const ladder = SUITES.map(s => { const o = ctx.store.outputMeta(s.id), ex = ctx.store.project.experiments[s.id] || [], lvl = o ? Math.max(0, CREDIBILITY.indexOf(o.credibility)) : -1; const nv = ex.filter(d => d.use === 'validation').length, nc = ex.filter(d => d.use === 'calibration').length; return { s, o, lvl, nv, nc, prio: (CRIT[s.id] || 0.5) * (1 - Math.max(0, lvl) / 7) * (nv ? 0.6 : 1) }; });
    out.tables.push({ title: 'Credibility status of every suite', cols: ['Suite', 'Status', 'Credibility level', 'Calibration sets', 'Validation sets', 'Experiment priority'], rows: ladder.map(r => [`${String(r.s.num).padStart(2, '0')} ${r.s.short}`, r.o ? (r.o.stale ? 'stale' : 'current') : 'not run', r.o ? r.o.credibility : '—', r.nc, r.nv, fmt(r.prio, 2)]), explain: 'Experiment priority combines how decisive the suite is for the feasibility question with how far its credibility is from validated; it identifies the measurement that would reduce uncertainty most.' });
    const top = ladder.slice().sort((a, b) => b.prio - a.prio).slice(0, 3);
    // ---------- charts ----------
    out.charts.push({ title: 'MMS spatial convergence (2-D Poisson, 5-point FD)', x: { label: 'h', log: true }, y: { label: 'Error', log: true }, series: [{ name: 'L2 error', x: mm.map(m => m.h), y: mm.map(m => m.l2), mode: 'lines+markers' }, { name: 'L∞ error', x: mm.map(m => m.h), y: mm.map(m => m.linf), mode: 'lines+markers' }, { name: 'slope 2 reference', x: mm.map(m => m.h), y: mm.map(m => mm[0].l2 * (m.h / mm[0].h) ** 2), dash: '5 4' }], explain: `Observed orders ${pObs.map(v => fmt(v, 3)).join(', ')} → second-order accuracy confirmed for the discretisation used by the thermal solvers.` });
    out.charts.push({ title: 'Temporal convergence — transient conduction', x: { label: 'Δt', unit: 's', log: true }, y: { label: 'Max error', log: true }, series: [{ name: 'Backward Euler', x: be.map(b => b.dt), y: be.map(b => b.err), mode: 'lines+markers' }, { name: 'Crank–Nicolson', x: cn.map(b => b.dt), y: cn.map(b => b.err), mode: 'lines+markers' }], explain: 'First-order (BE) and second-order (CN) temporal accuracy reproduced; the spatial grid is fine enough not to pollute the time-step study.' });
    out.charts.push({ title: 'Poiseuille flow: LBM vs analytical', x: { label: 'Velocity (lattice units)' }, y: { label: 'y (cells)' }, series: [{ name: 'Analytical', x: pz[1].exact, y: pz[1].y }, { name: 'LBM ny = 21', x: pz[1].num, y: pz[1].y, mode: 'markers' }], explain: `Relative L2 errors ${pz.map(z => fmt(z.relL2 * 100, 3) + ' %').join(', ')} for ny = 11, 21, 41; observed order ${fmt(pOrd, 3)}.` });
    out.charts.push({ title: 'Cantilever natural frequencies vs elements', x: { label: 'Elements', log: true }, y: { label: 'f_FE / f_exact' }, series: [{ name: 'Mode 1', x: nels, y: cb.map(c => c.f1 / c.f1Exact), mode: 'lines+markers' }, { name: 'Mode 2', x: nels, y: cb.map(c => c.f2 / c.f2Exact), mode: 'lines+markers' }], annotations: [{ type: 'hline', y: 1, color: '#22c55e' }], explain: 'Consistent-mass Hermite elements converge from above to the analytical frequencies (upper-bound property).' });
    out.charts.push({ title: 'Energy error: symplectic Verlet vs RK4 (harmonic oscillator)', x: { label: 'Time (periods)' }, y: { label: '|E/E₀ − 1|', log: true }, series: [{ name: 'Velocity Verlet', x: ve.t, y: ve.e.map(v => Math.max(v, 1e-16)) }, { name: 'RK4', x: rk.t, y: rk.e.map(v => Math.max(v, 1e-16)) }], explain: 'Verlet keeps energy error bounded (oscillating) over long runs; RK4 is more accurate per step but drifts monotonically — relevant for long vehicle-dynamics and hub simulations.' });
    out.charts.push({ title: 'Verification benchmark summary', type: 'bar', horizontal: true, categories: B.map(b => b.name), y: { label: 'Pass (1) / fail (0)' }, series: [{ name: 'status', values: B.map(b => b.pass ? 1 : 0), colors: B.map(b => b.pass ? '#22c55e' : '#ef4444') }], wide: true, explain: 'Each bar is a benchmark executed live in this session.' });
    // ---------- KPIs ----------
    const kk = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    kk('Verification benchmarks passed', `${passN}/${B.length}`, '', 'Executed live; failures would block “Numerically verified” status.', 'calc', passN === B.length ? 'ok' : 'bad');
    kk('MMS observed order', pLast, '', 'Expected 2 for the 5-point scheme.', 'calc');
    kk('Crank–Nicolson temporal order', ord(cn), '', 'Expected 2.', 'calc');
    kk('LBM Poiseuille error', pz[2].relL2 * 100, '%', 'Relative L2 error at 41 lattice cells across the channel.', 'calc');
    kk('Calibration dataset', synthetic ? 'Synthetic demo' : ds.name, '', synthetic ? 'No measured dataset imported — the demo shows the workflow only.' : `${ds.xc} → ${ds.yc}`, synthetic ? 'hypothetical' : 'user', synthetic ? 'warn' : 'ok');
    if (fit) { M.names.forEach((n, i) => kk(`Calibrated ${n}`, `${fmt(fit.p[i], 4)} ± ${fmt(1.96 * fit.se[i], 2)}`, '', '95 % confidence interval from the parameter covariance.', 'calibrated')); kk('Validation NRMSE', mt.nrmse * 100, '%', `Independent subset; acceptance ${p.accept} %.`, 'calc', mt.nrmse * 100 <= p.accept ? 'ok' : 'bad'); kk('Validation R²', mt.r2, '', 'Coefficient of determination on held-out data.', 'calc'); }
    out.verdicts.unshift({ test: 'Code & solution verification', status: passN === B.length ? 'pass' : 'fail', detail: `${passN} of ${B.length} benchmarks met their criteria.` });
    out.verdicts.push({ test: 'Calibration/validation separation', status: 'pass', detail: `${cal.length} calibration and ${val.length} validation points, disjoint (${p.splitMode} split).` });
    out.recs.push(...top.map(t => ({ level: 'warn', text: `Highest-value next experiment: Suite ${String(t.s.num).padStart(2, '0')} ${t.s.short} (credibility: ${t.o?.credibility || 'not run'}). Import independent measurements there and register acceptance criteria before testing.`, basis: 'Experiment-priority score (criticality × credibility gap)' })));
    if (synthetic) out.recs.push({ level: 'critical', text: 'No physical measurements have been imported. Simulated results across the platform remain unvalidated; the direct-rim hub, NiTi cycle and collector heat flux require the instrumented experiments described in the dossier (Experiments A–D).', basis: 'Specification Part B §30–31' });
    out.recs.push({ level: 'good', text: 'Keep calibration and validation datasets in separate files and record instrument calibration, uncertainty, material batch and test conditions with every import (evidence package).', basis: 'Mandatory experimental evidence record' });
    out.explain.push({ title: 'Three different activities', text: 'Verification asks “are the equations solved correctly?” (benchmarks with exact answers, convergence orders). Calibration fits uncertain parameters to measurements. Validation asks “are these the right equations for this purpose?” using data never used for calibration, judged against criteria chosen in advance.' });
    out.views.push({ label: 'Credibility ladder', icon: '🪜', render: (el) => { for (const r of ladder) el.append(h('div', { class: 'card', style: 'padding:8px 12px;margin-bottom:6px' }, h('div', { class: 'small' }, h('b', {}, `${String(r.s.num).padStart(2, '0')} ${r.s.title}`), r.o?.stale ? ' (stale)' : ''), h('div', { class: 'ladder' }, ...CREDIBILITY.map((c, i) => h('span', { class: i <= r.lvl ? 'on' : '' }, c))))); } });
    out.outputs = { passRate: passN / B.length, results: B.map(b => ({ name: b.name, value: b.value, pass: b.pass })), calibration: fit ? { model: p.model, params: fit.p, se: fit.se, dataset: ds.name, synthetic } : null, validation: mt ? { nrmse: mt.nrmse, r2: mt.r2, bias: mt.bias } : null };
    out.headline = `${passN}/${B.length} verification benchmarks passed live on this device. ${synthetic ? 'Calibration shown on a synthetic demonstration dataset — import measurements to validate.' : `Validation NRMSE ${fmt(mt?.nrmse * 100, 3)} % on ${val.length} independent points.`}`;
    out.headlineKind = passN === B.length ? 'ok' : 'warn';
    out.credibility = passN === B.length ? 'Numerically verified' : 'Analytical screening';
    void normInv; void table; void sum;
    return out;
  },
};
