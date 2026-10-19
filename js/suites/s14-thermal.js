// Suite 14 — Thermal metamaterials, heat spreaders, insulation and thermal management.
import * as TH from '../core/thermo.js';
import { SCENARIOS, derive } from '../core/env.js';
import { linspace, fmt, luSolve, rk45, range, KELVIN, argmax, logspace } from '../core/num.js';
import { forUse, get } from '../core/materials.js';

function envCase(env, kind) {
  if (kind === 'current' && env) return { T: env.T, Uz: env.Uz ?? env.U, ghi: env.ghi || 0, eps: env.eps ?? 0.8, isDay: env.isDay, src: 'Suite 05 snapshot', coupled: true };
  const want = kind === 'current' ? 'night' : kind;
  if (env?.series?.length) {
    const s = env.series; if (want === 'day') { const i = argmax(s.map(r => r.ghi || 0)); if (s[i].ghi > 50) return { ...s[i], isDay: true, src: `Suite 05 peak-sun hour ${s[i].time}`, coupled: true }; }
    else { const n = s.filter(r => !r.isDay).sort((a, b) => (a.T - a.Tsky) - (b.T - b.Tsky)); if (n.length) { const r = n[Math.floor(n.length / 2)]; return { ...r, isDay: false, src: `Suite 05 median night hour ${r.time}`, coupled: true }; } }
  }
  const d = derive(want === 'day' ? SCENARIOS.sunnyCalm : SCENARIOS.clearNight, {});
  return { T: d.T, Uz: d.Uz, ghi: d.ghi, eps: d.eps, isDay: want === 'day', src: `scenario “${(want === 'day' ? SCENARIOS.sunnyCalm : SCENARIOS.clearNight).label}”`, coupled: false };
}

export default {
  description: 'Solves the complete heat path from the photonic collector to the actuator hot interface and from the actuator cold interface to the radiator: thermal-resistance network with contact resistances, heat-pipe operating limits, insulation leakage, a 2-D anisotropic finite-volume conduction field and the transient warm-up. It computes the actual interface temperatures seen by the actuator — never assuming they equal ambient or sky temperature.',
  literature: 'anisotropic thermal metamaterial heat flux manipulation thermal contact resistance heat pipe limits',
  sustain: true, credibility: 'Analytical screening',
  equations: [
    'Fourier’s law q = −k∇T with anisotropic conductivity tensor diag(k_x, k_y); transient heat conduction ρc_p ∂T/∂t = ∇·(k∇T)',
    'Thermal resistance networks: conduction L/(kA), contact 1/(h_cA), insulation leakage, series/parallel combination',
    'Interfacial (contact) conductance and harmonic-mean face conductances in the finite-volume discretisation',
    'Newton’s law of cooling; McAdams wind-forced convection; finned-exchanger convective area',
    'Surface radiative balances from Suite 13 (absorber / radiator spectral models)',
    'Heat-pipe limits: capillary (Laplace pressure vs liquid and gravity losses, Darcy wick flow), sonic, entrainment, viscous and boiling limits; water properties vs temperature',
    'Energy conservation at every node; lumped-capacitance transient network for warm-up time constants',
  ],
  methods: ['Newton–Raphson on the 4-node nonlinear network (finite-difference Jacobian, LU solve)', 'Cell-centred finite-volume steady conduction with successive over-relaxation (ω = 1.85)', 'Adaptive Dormand–Prince integration of the 2-node transient network', 'Converter load sweep (impedance matching) for maximum ideal work'],
  assumptions: ['The actuator is represented as an effective thermal conductance G_conv (heat drawn per kelvin of interface ΔT) coupled from Suites 15/17 when available', 'Mechanical work extracted (≤ few % of heat) is neglected in the heat balance', 'Symmetric construction of the hot and cold paths'],
  ibc: {
    initial: ['Uniform initial temperature equal to ambient air', 'Initial contact state and interface pressure (via conductance h_c)', 'Heat-pipe operating state (temperature-dependent properties)'],
    boundary: ['Collector side: spectral radiative + convective balance (day: solar absorber; night: ambient-air finned exchanger)', 'Radiator side: radiative-cooling balance + convection', 'Actuator interfaces: coupled heat flux G_conv(T_hi − T_ci) — not imposed temperatures', 'Adiabatic outer edges of the conduction block; contact resistance at material interfaces'],
  },
  cvv: {
    calibration: ['Thermal conductivity tensor (laser flash, transient plane source)', 'Interface thermal resistance tests vs contact pressure', 'Heat-pipe performance curves (Q vs ΔT, orientation)'],
    verification: ['1-D slab steady conduction vs analytic linear profile', 'Network node energy residuals', '2-D field boundary heat balance', 'Lumped-capacitance exponential solution'],
    validation: ['Instrumented thermal-sector rig: thermocouples at collector, interfaces and radiator; heat-flux sensors; transient histories under outdoor night/day exposure'],
  },
  formats: { import: ['CSV temperature/heat-flux logs', 'XLSX', 'JSON'], export: ['CSV field and network results', 'VTK field (via Suite 02 export manager)', 'PNG/SVG'] },
  params: [
    { key: 'case', label: 'Operating case', type: 'select', group: 'Case', default: 'night', auto: (c) => { const e = c.upstream('s05') || c.env; const sunny = !!(e?.isDay && (e.ghi || 0) > 200) || (e?.series || []).some(r => (r.ghi || 0) > 200); return { value: sunny ? 'day' : 'night', prov: 'coupled' }; }, options: [{ value: 'night', label: 'Night (ambient source → radiator sink)' }, { value: 'day', label: 'Day (solar absorber → radiator/ambient sink)' }, { value: 'current', label: 'Current Suite 05 snapshot' }] },
    { key: 'spreader', label: 'Spreader / thermal metamaterial', type: 'select', group: 'Materials', default: 'al-1050', options: forUse('spreader').map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.store.material('spreader', 'al-1050'), prov: 'library' }) },
    { key: 'insulator', label: 'Hot/cold isolation', type: 'select', group: 'Materials', default: 'aerogel', options: forUse('insulator').map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.store.material('insulator', 'aerogel'), prov: 'library' }) },
    { key: 'anis', label: 'Lattice anisotropy k_in-plane / k_through', type: 'range', min: 1, max: 200, step: 1, group: 'Materials', default: 1, help: 'Architected thermal metamaterial: >1 channels heat along the spreader while limiting through-thickness leakage.' },
    { key: 'nHP', label: 'Number of heat pipes per path', type: 'number', min: 0, max: 40, step: 1, group: 'Heat pipes', default: 6 },
    { key: 'Lhp', label: 'Heat-pipe length', unit: 'm', type: 'number', min: 0.05, max: 2, step: 0.01, group: 'Heat pipes', default: 0.6 },
    { key: 'dHP', label: 'Heat-pipe diameter', unit: 'mm', type: 'number', min: 3, max: 20, step: 0.5, group: 'Heat pipes', default: 8 },
    { key: 'tilt', label: 'Heat-pipe tilt (evaporator above = +)', unit: '°', type: 'range', min: -90, max: 90, step: 1, group: 'Heat pipes', default: 0, expert: true },
    { key: 'Lsp', label: 'Spreader length', unit: 'm', type: 'number', min: 0.02, max: 1, step: 0.01, group: 'Geometry', default: 0.3 },
    { key: 'wsp', label: 'Spreader width', unit: 'm', type: 'number', min: 0.01, max: 1, step: 0.01, group: 'Geometry', default: 0.2 },
    { key: 'tsp', label: 'Spreader thickness', unit: 'mm', type: 'number', min: 0.1, max: 20, step: 0.1, group: 'Geometry', default: 3 },
    { key: 'hc', label: 'Interface contact conductance', unit: 'W/m²K', type: 'number', min: 100, max: 100000, step: 100, group: 'Interfaces', default: 4000, help: 'Bare metal contact ≈ 1–5 kW/m²K; with thermal interface material 10–50 kW/m²K.' },
    { key: 'Ac', label: 'Interface contact area per side', unit: 'cm²', type: 'number', min: 1, max: 5000, step: 1, group: 'Interfaces', default: 300 },
    { key: 'tIns', label: 'Isolation thickness', unit: 'mm', type: 'number', min: 1, max: 100, step: 1, group: 'Interfaces', default: 10 },
    { key: 'Ains', label: 'Isolation (leak path) area', unit: 'cm²', type: 'number', min: 1, max: 5000, step: 1, group: 'Interfaces', default: 400 },
    { key: 'Gconv', label: 'Actuator thermal draw G_conv', unit: 'W/K', type: 'number', min: 0.01, max: 500, step: 0.1, group: 'Converter coupling', default: 4, auto: (c) => { const o = c.upstream('s15'); return o?.Gconv_W_K ? { value: +o.Gconv_W_K.toFixed(3), prov: 'coupled' } : undefined; } },
    { key: 'fin', label: 'Air-side finned area multiplier', type: 'range', min: 1, max: 10, step: 0.5, group: 'Converter coupling', default: 3 },
  ],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const env = ctx.upstream('s05') || ctx.env, s13 = ctx.upstream('s13');
    const c = envCase(env, p.case), TaK = c.T + KELVIN, h = TH.mcAdamsH(c.Uz || 0);
    const colM = get(s13?.collector || ctx.store.material('collector', 'coat-tinox')), radM = get(s13?.radiator || ctx.store.material('radiator', 'coat-pdrc'));
    const Acol = s13?.collectorArea ?? 4, Arad = s13?.radiatorArea ?? 4;
    const epsCol = TH.tabulate(TH.surfaceSpectrum(colM)), epsRad = TH.tabulate(TH.surfaceSpectrum(radM));
    const sky = TH.skySpectralEmissivity(TaK, c.eps ?? 0.8).fn, G = c.isDay ? (c.ghi || 0) : 0;
    const cacheA = new Map(), cacheR = new Map();
    const surf = (tab, T, cache) => { const k = Math.round(T * 20); if (!cache.has(k)) cache.set(k, TH.radiativeTerms(tab, k / 20, TaK, sky, G)); return cache.get(k); };
    // Revision 2 (D5): in the day case the hot side is the Suite 13 glazed collector — use its solved q(T) curve so the
    // collector operating temperature found there is carried through the heat path; otherwise the bare-surface model.
    const curveA = p.case === 'day' && s13?.curves?.absorber?.T?.length ? s13.curves.absorber : null;
    const Qhot = curveA ? (T) => Acol * interpX(curveA.T, curveA.q, T - KELVIN) : (T) => { const r = surf(epsCol, T, cacheA); return Acol * (r.solAbs - (r.emit - r.atm) - h * (T - TaK)) + Acol * (p.fin - 1) * h * (TaK - T); };
    const Qcold = (T) => { const r = surf(epsRad, T, cacheR); return Arad * ((r.emit - r.atm) - r.solAbs - h * (TaK - T)) + Arad * (p.fin - 1) * h * (T - TaK); };
    const sp = get(p.spreader), ins = get(p.insulator), hpM = get(ctx.store.material('heatpipe', 'heatpipe-cuw'));
    const kIn = sp.k;
    const tsp = p.tsp / 1000, Asp = p.wsp * tsp, Ac = p.Ac / 1e4, Ains = p.Ains / 1e4, tIns = p.tIns / 1000;
    const Rsp = p.Lsp / (kIn * Asp), Ahp = Math.PI * (p.dHP / 1000) ** 2 / 4, Rhp = p.nHP > 0 ? p.Lhp / ((hpM?.k || 10000) * Ahp * p.nHP) : Infinity;
    const Rcon = 1 / (p.hc * Ac), Rpath = 1 / (1 / Rsp + (Number.isFinite(Rhp) ? 1 / Rhp : 0)) + Rcon, Rins = tIns / (ins.k * Ains);
    // Newton solve of the 4-node network: x = [Tcol, Thi, Tci, Trad]
    const solve = (Gc) => {
      const F = (x) => { const [Tc_, Th_, Tk_, Tr_] = x, qh = (Tc_ - Th_) / Rpath, qc = (Tk_ - Tr_) / Rpath, qi = Gc * (Th_ - Tk_) + (Th_ - Tk_) / Rins; return [Qhot(Tc_) - qh, qh - qi, qi - qc, qc - Qcold(Tr_)]; };
      let x = [TaK + 1, TaK + 0.5, TaK - 0.5, TaK - 1]; const hist = [];
      for (let it = 0; it < 60; it++) {
        const f = F(x), nrm = Math.hypot(...f); hist.push(nrm); if (nrm < 1e-7) break;
        const J = range(4).map(() => new Float64Array(4)); for (let k = 0; k < 4; k++) { const d = 1e-4, xp = x.slice(); xp[k] += d; const fp = F(xp); for (let i = 0; i < 4; i++) J[i][k] = (fp[i] - f[i]) / d; }
        let dx; try { dx = luSolve(J, f.map(v => -v)); } catch { break; }
        let lam = 1; const n0 = nrm; for (let ls = 0; ls < 12; ls++) { const xn = x.map((v, i) => v + lam * dx[i]); if (Math.hypot(...F(xn)) < n0) { x = xn; break; } lam /= 2; if (ls === 11) x = x.map((v, i) => v + lam * dx[i]); }
      }
      const [Tc_, Th_, Tk_, Tr_] = x, Qconv = Gc * (Th_ - Tk_), Qleak = (Th_ - Tk_) / Rins;
      return { Tcol: Tc_, Thi: Th_, Tci: Tk_, Trad: Tr_, Qconv, Qleak, Qin: (Tc_ - Th_) / Rpath, hist, res: hist[hist.length - 1], Wideal: Math.max(0, Qconv * (1 - Tk_ / Th_)) };
    };
    ctx.progress(0.2, 'network');
    const S = solve(p.Gconv);
    const open = solve(1e-6);
    const Gs = logspace(-2, 2.7, 40), sw = Gs.map(g => solve(g)), iBest = argmax(sw.map(r => r.Wideal));
    await ctx.yield();
    // heat-pipe limits at hot path temperature
    const hpT = (p.case === 'night' ? S.Thi : S.Tcol) - KELVIN, lim = TH.heatPipeLimits({ TC: hpT, dOuter: p.dHP / 1000, L: p.Lhp, tilt: p.tilt });
    const qPerPipe = p.nHP > 0 ? Math.abs(S.Qin) * (1 / Rhp) / (1 / Rsp + 1 / Rhp) / p.nHP : 0;
    ctx.progress(0.45, '2-D field');
    // 2-D anisotropic FV field: hot block | isolation | cold block (x), spreader thickness (y)
    const nH = 18, nI = 6, nx = 2 * nH + nI, ny = 10, Lx = 2 * p.Lsp + Math.max(tIns, 0.005), dxH = p.Lsp / nH, dxI = Math.max(tIns, 0.005) / nI;
    // use uniform dx for the solver; map the isolation gap to nI cells of the same width (geometry scaled, conductance preserved)
    const dx = dxH, dy = tsp / ny, kx = new Float64Array(nx * ny), ky = new Float64Array(nx * ny), rc = new Float64Array(nx * ny);
    const kInsEff = ins.k * dxH / dxI; // preserves the isolation conductance on the uniform grid
    for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) { const k = j * nx + i, isIns = i >= nH && i < nH + nI; kx[k] = isIns ? kInsEff : kIn; ky[k] = isIns ? kInsEff : kIn / (p.anis || 1); rc[k] = isIns ? ins.rho * ins.cp : sp.rho * sp.cp; }
    const qTopHot = S.Qin / (p.Lsp * p.wsp), Tci = S.Tci, Thi = S.Thi;
    const hEffRad = Math.abs(S.Qin) / Math.max(0.05, Math.abs(S.Tci - S.Trad)) / (p.Lsp * p.wsp);
    const field = TH.conduction2D({ nx, ny, dx, dy, kx, ky, rc }, {
      top: { type: 'mixed', fn: (i) => i < nH ? { type: 'q', q: qTopHot } : i >= nH + nI ? { type: 'h', h: hEffRad, Tinf: S.Trad } : null },
      bottom: { type: 'mixed', fn: (i) => i < nH ? { type: 'h', h: p.Gconv / (p.Lsp * p.wsp) + 1e-9, Tinf: Tci } : i >= nH + nI ? { type: 'q', q: S.Qconv / (p.Lsp * p.wsp) } : null },
    }, { T0: TaK, tol: 1e-7, maxIt: 30000 });
    const Z = range(ny).map(j => range(nx).map(i => field.T[j * nx + i] - KELVIN));
    const xsF = range(nx).map(i => (i + 0.5) * dx * 1000), ysF = range(ny).map(j => (j + 0.5) * dy * 1000);
    const U = range(ny).map(j => range(nx).map(i => { const k = j * nx + i, a = field.T[k - (i > 0 ? 1 : 0)], b = field.T[k + (i < nx - 1 ? 1 : 0)]; return -kx[k] * (b - a) / (dx * ((i > 0) + (i < nx - 1))); }));
    const V = range(ny).map(j => range(nx).map(i => { const k = j * nx + i, a = field.T[k - (j > 0 ? nx : 0)], b = field.T[k + (j < ny - 1 ? nx : 0)]; return -ky[k] * (b - a) / (dy * ((j > 0) + (j < ny - 1))); }));
    const faceFlux = (i) => { let q = 0; for (let j = 0; j < ny; j++) { const k = j * nx + i; q += field.Gx[k] * (field.T[k] - field.T[k + 1]) * p.wsp; } return q; };
    const leakA = faceFlux(nH), leakB = faceFlux(nH + nI - 2);
    ctx.progress(0.7, 'transient');
    // transient 2-node network (hot block, cold block) from ambient
    const Chot = sp.rho * sp.cp * Asp * p.Lsp + 2700 * 900 * Acol * 0.001, Ccold = sp.rho * sp.cp * Asp * p.Lsp + 2700 * 900 * Arad * 0.001;
    const tEnd = Math.min(20000, 8 * Math.max(Chot, Ccold) * (Rpath + 0.2)), tr = rk45((t, y) => { const [Th_, Tk_] = y, Tcol = Th_, Trad = Tk_, qi = p.Gconv * (Th_ - Tk_) + (Th_ - Tk_) / Rins; return [(Qhot(Tcol) - qi) / Chot, (qi - Qcold(Trad)) / Ccold]; }, 0, [TaK, TaK], tEnd, { rtol: 1e-6, hmax: tEnd / 200 });
    const dTss = tr.y[tr.y.length - 1][0] - tr.y[tr.y.length - 1][1], i63 = tr.y.findIndex(y => (y[0] - y[1]) >= 0.632 * dTss), tau = i63 > 0 ? tr.t[i63] : NaN;
    // verification
    const slab = TH.conduction2D({ nx: 20, ny: 1, dx: 0.01, dy: 0.01, kx: new Float64Array(20).fill(10), ky: new Float64Array(20).fill(10), rc: new Float64Array(20).fill(1e6) }, { left: { type: 'T', T: 1 }, right: { type: 'T', T: 0 } }, { T0: 0.5, tol: 1e-12, maxIt: 50000 });
    const slabErr = Math.max(...range(20).map(i => Math.abs(slab.T[i] - (1 - (i + 0.5) / 20))));
    out.checks.push({ name: '1-D slab: FV vs analytic linear profile', value: slabErr, target: '< 1e-6 K', pass: slabErr < 1e-6 });
    out.checks.push({ name: 'Network energy residual (Newton)', value: S.res, target: '< 1e-6 W', pass: S.res < 1e-5, detail: `${S.hist.length} iterations` });
    out.checks.push({ name: 'Node balance: Q_in = Q_conv + Q_leak', value: S.Qin - S.Qconv - S.Qleak, target: '≈ 0 W', pass: Math.abs(S.Qin - S.Qconv - S.Qleak) < 1e-4 * Math.max(1, Math.abs(S.Qin)) });
    out.checks.push({ name: '2-D flux conservation across the isolation (entry vs exit face)', value: leakA / (leakB || 1e-30), target: '1 ± 1e-6', pass: Math.abs(leakA / (leakB || 1e-30) - 1) < 1e-6, detail: `${fmt(leakA, 4)} W vs ${fmt(leakB, 4)} W` });
    out.checks.push({ name: '2-D steady solution residual (PCG)', value: field.residual, target: '< 1e-7 K', pass: field.residual < 1e-7, detail: `${field.iterations} CG iterations` });
    const lc = rk45((t, y) => [-(y[0] - 0) / 10], 0, [1], 30, { rtol: 1e-10, atol: 1e-12 }), lcErr = Math.abs(lc.y[lc.y.length - 1][0] - Math.exp(-3)); out.checks.push({ name: 'Lumped capacitance vs exp(−t/τ)', value: lcErr, target: '< 1e-6', pass: lcErr < 1e-6 });
    // KPIs
    const dTi = S.Thi - S.Tci, dTopen = open.Thi - open.Tci;
    const K = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    K('Actuator hot-interface temperature', S.Thi - KELVIN, '°C', 'Solved temperature at the actuator hot face (not ambient, not collector).', 'calc');
    K('Actuator cold-interface temperature', S.Tci - KELVIN, '°C', 'Solved temperature at the actuator cold face (not sky temperature).', 'calc');
    K('Interface ΔT under load', dTi, 'K', `Temperature difference actually available to the actuator while it draws ${fmt(p.Gconv, 3)} W/K.`, 'calc', dTi > 15 ? 'ok' : dTi > 5 ? 'warn' : 'bad');
    K('Open-circuit interface ΔT', dTopen, 'K', 'ΔT with no heat drawn by the actuator — an upper bound that collapses once heat flows.', 'calc');
    K('Heat delivered to actuator', S.Qconv, 'W', 'Heat flow through the converter; the work it can make is bounded by Carnot of the interface temperatures.', 'calc');
    K('Heat leakage through isolation', S.Qleak, 'W', 'Heat bypassing the actuator from the hot to the cold side — wasted exergy.', 'calc', S.Qleak < 0.1 * Math.abs(S.Qconv) ? 'ok' : 'warn');
    K('Ideal work from delivered heat', S.Wideal, 'W', 'Q_conv(1 − T_ci/T_hi): ceiling for any converter at this operating point.', 'calc');
    K('Best ideal work (matched G_conv)', sw[iBest].Wideal, 'W', `Achieved at G_conv ≈ ${fmt(Gs[iBest], 3)} W/K (thermal impedance matching).`, 'calc');
    K('Path thermal resistance (each side)', Rpath, 'K/W', 'Spreader ∥ heat pipes, in series with the contact resistance.', 'calc');
    K('Collector / exchanger temperature', S.Tcol - KELVIN, '°C', p.case === 'day' || c.isDay ? 'Absorber temperature under load.' : 'Air-side exchanger temperature (night).', 'calc');
    K('Radiator temperature', S.Trad - KELVIN, '°C', 'Radiator temperature while rejecting the converter heat.', 'calc');
    K('Heat per pipe / heat-pipe limit', p.nHP ? `${fmt(qPerPipe, 3)} / ${fmt(lim.Qmax, 3)}` : 'no heat pipes', 'W', `Governing limit: ${lim.governing}. ${lim.frozen ? 'Working fluid at/below freezing!' : ''}`, 'calc', p.nHP && qPerPipe > lim.Qmax ? 'bad' : 'ok');
    K('Warm-up time constant', tau, 's', 'Time to reach 63 % of steady interface ΔT from an isothermal start.', 'calc');
    // verdicts
    out.verdicts.push({ test: 'Interface temperatures solved (not assumed)', status: 'pass', detail: `T_hi = ${fmt(S.Thi - KELVIN, 4)} °C vs ambient ${fmt(c.T, 4)} °C; T_ci = ${fmt(S.Tci - KELVIN, 4)} °C (${c.src}).` });
    const actM = get(ctx.store.project.materials?.actuator || 'nitcu') || get('nitcu'), s15o = ctx.upstream('s15'), needDT = s15o?.minDT_K ?? ((actM.Af - actM.Mf) + 6 + 150 / (actM.CA || 7) - 20 / (actM.CM || 6.5));
    K('Achievable ΔT vs actuator requirement', `${fmt(dTi, 3)} / ${fmt(needDT, 3)}`, 'K', `${actM.name}: hysteresis (A_f − M_f = ${fmt(actM.Af - actM.Mf, 3)} K) + stress shift + 2 × 3 K margins. ${s15o?.minDT_K ? 'From Suite 15.' : 'Estimated at σ_H = 150, σ_L = 20 MPa.'}`, s15o ? 'coupled' : 'estimated', dTi >= needDT ? 'ok' : 'bad');
    if (curveA) K('Collector curve source', 'Suite 13 glazed collector', '', `Hot-side heat q(T) taken from Suite 13 (exergy-optimal point ${fmt(s13.Thot_C, 4)} °C); the network solves the actual collector temperature under the actuator draw.`, 'coupled');
    out.verdicts.push({ test: 'Usable actuator ΔT', status: dTi >= needDT ? 'pass' : dTi > 0.5 * needDT ? 'warn' : 'fail', detail: `${fmt(dTi, 3)} K under load vs ${fmt(needDT, 3)} K needed by ${actM.name} (hysteresis + stress shift + margins). Below that the alloy cannot complete a transformation cycle.`, fix: 'Reduce path resistance (more heat pipes, better interface material), enlarge collector/radiator areas, or lower the actuator draw.' });
    out.verdicts.push({ test: 'Heat-pipe operating limits', status: !p.nHP ? 'info' : lim.frozen ? 'fail' : qPerPipe > lim.Qmax ? 'fail' : 'pass', detail: p.nHP ? `${fmt(qPerPipe, 3)} W per pipe vs ${lim.governing} limit ${fmt(lim.Qmax, 3)} W at ${fmt(hpT, 3)} °C.` : 'Heat pipes not used.', fix: 'Add pipes, use larger diameter or finer wick, avoid adverse tilt; water pipes freeze below 0 °C — use another working fluid in cold climates.' });
    out.verdicts.push({ test: 'Isolation effectiveness', status: S.Qleak < 0.1 * Math.abs(S.Qconv) ? 'pass' : 'warn', detail: `Leakage ${fmt(S.Qleak, 3)} W = ${fmt(100 * S.Qleak / Math.max(1e-9, Math.abs(S.Qconv)), 3)} % of converter heat.` });
    // charts
    out.charts.push({ title: '2-D conduction field: hot block | isolation | cold block (°C)', type: 'heatmap', wide: true, x: { label: 'Along spreader', unit: 'mm', values: xsF }, y: { label: 'Thickness', unit: 'mm', values: ysF }, z: Z, cmap: 'coolwarm', isolines: 8, vectors: { u: U, v: V, stride: 2 }, z_label: '°C', explain: 'Finite-volume temperature field with heat-flux vectors (thickness exaggerated). Heat enters at top-left, is drawn into the actuator along the bottom-left, and leaks through the central isolation into the cold block, which rejects it to the radiator at top-right. Anisotropy reduces through-thickness gradients.' });
    out.charts.push({ title: 'Thermal impedance matching: converter draw vs available ΔT and ideal work', type: 'line', x: { label: 'Actuator thermal draw G_conv', unit: 'W/K', log: true }, y: { label: 'Value' }, series: [{ name: 'Interface ΔT (K)', x: Gs, y: sw.map(r => r.Thi - r.Tci) }, { name: 'Heat to actuator (W)', x: Gs, y: sw.map(r => r.Qconv) }, { name: 'Ideal work ×10 (W)', x: Gs, y: sw.map(r => 10 * r.Wideal) }], annotations: [{ type: 'vline', x: p.Gconv, label: 'current' }, { type: 'vline', x: Gs[iBest], label: 'best', color: '#22c55e' }], explain: 'Drawing more heat lowers the interface ΔT. Ideal work peaks where the actuator conductance matches the external path — the thermal equivalent of impedance matching.' });
    out.charts.push({ title: 'Warm-up transient (2-node network)', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Temperature', unit: '°C' }, series: [{ name: 'Hot side', x: tr.t, y: tr.y.map(y => y[0] - KELVIN) }, { name: 'Cold side', x: tr.t, y: tr.y.map(y => y[1] - KELVIN) }], annotations: Number.isFinite(tau) ? [{ type: 'vline', x: tau, label: 'τ (63 %)' }] : [], explain: 'How long the thermal path takes to establish its steady gradient after start-up or a change in weather.' });
    const lims = ['capillary', 'sonic', 'entrainment', 'viscous', 'boiling'];
    const Tsw = linspace(2, 90, 30); out.charts.push({ title: 'Heat-pipe operating envelope (per pipe)', type: 'line', x: { label: 'Vapour temperature', unit: '°C' }, y: { label: 'Limit', unit: 'W', log: true }, series: lims.map(n => ({ name: n, x: Tsw, y: Tsw.map(T => Math.max(1e-3, TH.heatPipeLimits({ TC: T, dOuter: p.dHP / 1000, L: p.Lhp, tilt: p.tilt })[n])) })), annotations: p.nHP ? [{ type: 'point', x: hpT, y: Math.max(1e-3, qPerPipe), label: 'operating' }] : [], explain: 'The lowest curve at the operating temperature governs. Low-temperature operation is usually capillary- or viscous-limited.' });
    out.charts.push({ title: 'Network solution convergence', type: 'line', x: { label: 'Newton iteration' }, y: { label: 'Residual', unit: 'W', log: true }, series: [{ name: '‖F‖', x: range(S.hist.length), y: S.hist.map(v => Math.max(v, 1e-14)) }], explain: 'Quadratic Newton convergence of the nonlinear radiative–conductive network.' });
    out.tables.push({ title: 'Thermal resistance network', cols: ['Element', 'Value', 'Unit'], rows: [['Spreader (in-plane)', Rsp, 'K/W'], ['Heat pipes (parallel)', Number.isFinite(Rhp) ? Rhp : '—', 'K/W'], ['Contact', Rcon, 'K/W'], ['Path total (per side)', Rpath, 'K/W'], ['Isolation leak path', Rins, 'K/W'], ['Converter 1/G', 1 / p.Gconv, 'K/W']] });
    out.tables.push({ title: 'Node temperatures and heat flows', cols: ['Quantity', 'Value', 'Unit'], rows: [['Ambient', c.T, '°C'], ['Collector / exchanger', S.Tcol - KELVIN, '°C'], ['Actuator hot interface', S.Thi - KELVIN, '°C'], ['Actuator cold interface', S.Tci - KELVIN, '°C'], ['Radiator', S.Trad - KELVIN, '°C'], ['Heat in from collector', S.Qin, 'W'], ['Heat to actuator', S.Qconv, 'W'], ['Leakage', S.Qleak, 'W']] });
    // recs
    if (dTi < 10) out.recs.push({ level: 'critical', text: `Only ${fmt(dTi, 3)} K reaches the actuator. ${p.case === 'night' ? 'At night this is an environmental limit (sky radiative sink), not just a design flaw.' : 'Reduce contact resistance and add heat-transport capacity.'}`, basis: 'Coupled network solution' });
    if (Rcon > 0.3 * Rpath) out.recs.push({ level: 'warn', text: `Contact resistance is ${fmt(100 * Rcon / Rpath, 3)} % of the path; specify a compliant thermal interface material and controlled clamping pressure (h_c ≥ 20 kW/m²K).`, basis: 'Resistance breakdown' });
    if (Math.abs(Math.log10(p.Gconv / Gs[iBest])) > 0.3) out.recs.push({ level: 'info', text: `Re-size the actuator heat draw towards ≈ ${fmt(Gs[iBest], 3)} W/K (cartridge count × frequency × thermal mass) to maximise ideal work.`, basis: 'Impedance matching sweep' });
    if (p.anis < 5) out.recs.push({ level: 'info', text: 'An anisotropic thermal metamaterial (graphite-laminate or aligned lattice, k_in/k_through ≥ 20) channels heat to the interfaces while suppressing cross-leakage.', basis: 'Anisotropic conduction' });
    out.recs.push({ level: 'sustain', text: 'Aluminium spreaders and copper/water heat pipes are highly recyclable; aerogel insulation is energy-intensive to produce — size it to the leakage target rather than over-insulating.', basis: 'Embodied energy in materials registry' });
    out.explain.push({ title: 'Why interface temperatures matter', text: 'The actuator does not see ambient air or the sky. It sees the ends of a thermal path whose resistances, leakage and radiative boundaries reduce the available temperature difference — especially once heat actually flows. These solved interface temperatures feed Suites 15 and 17.' });
    out.views.push({ label: '2D animation: transient temperature field', icon: '🎞', render: (el) => anim2dField(el, { tr, Z, nx, ny, nH, nI, TaK, S }) });
    out.views.push({ label: '3D heat path (transient)', icon: '🌡', render: (el) => view3dHeatPath(el, { S, tr, Z, nx, ny, nH, nI, TaK, spName: sp.name, insName: ins.name, nHP: p.nHP }) });
    out.outputs = { Qhot_W: S.Qconv, Qleak_W: S.Qleak, ThotIface_C: S.Thi - KELVIN, TcoldIface_C: S.Tci - KELVIN, Rth_K_W: 2 * Rpath, tauThermal_s: tau, Gconv_W_K: p.Gconv, case: p.case, Wideal_W: S.Wideal, dTopen_K: dTopen, TambC: c.T, bestG_W_K: Gs[iBest] };
    out.convergence = { title: 'Newton residual history', series: [{ name: 'network', x: range(S.hist.length), y: S.hist.map(v => Math.max(v, 1e-14)) }] };
    out.headline = `${p.case === 'day' ? 'Day' : p.case === 'night' ? 'Night' : 'Current'} case: actuator interfaces at ${fmt(S.Thi - KELVIN, 4)} °C / ${fmt(S.Tci - KELVIN, 4)} °C (ΔT ${fmt(dTi, 3)} K) delivering ${fmt(S.Qconv, 3)} W of heat; ideal work ≤ ${fmt(S.Wideal, 3)} W.`;
    out.headlineKind = dTi > 15 ? 'ok' : 'warn';
    return out;
  },
};

// 3D heat-path assembly: collector → hot spreader → actuator → cold spreader → radiator, with the insulation gap.
// Node temperatures vs time are the solved RK45 transient; the in-spreader distribution is the solved steady 2-D
// finite-volume field scaled by the node response at each instant (interpolated between solved states).
async function view3dHeatPath(el, { S, tr, Z, nx, ny, nH, nI, TaK, spName, insName, nHP }) {
  const [{ viewer }, { playback, hud, legend3d, fieldMesh, tracers }, { box, loft }, { cmap }, { h }] = await Promise.all([import('../core/viz3d.js'), import('../core/sim3d.js'), import('../core/cad.js'), import('../core/chart.js'), import('../core/ui.js')]);
  el.append(h('p', { class: 'small muted' }, `Solved: transient hot/cold node temperatures (RK45 network model) and the steady 2-D anisotropic temperature field in the ${spName} spreaders across the ${insName} isolation (finite-volume, PCG). The displayed field at time t is the steady field scaled by each side's solved transient response; geometry is schematic, thicknesses exaggerated ×10. Arrows: heat flows (length ∝ W).`));
  const wrap = h('div'); el.append(wrap);
  const V = await viewer(wrap, { height: 440, grid: false }); const THREE = V.THREE;
  const Lh = 0.8, Lg = 0.8 * nI / nH, T = 0.12; // visual lengths (m) of hot block, insulation gap and slab thickness
  const x0 = -(2 * Lh + Lg) / 2, xH1 = x0 + Lh, xC0 = xH1 + Lg, xC1 = xC0 + Lh;
  const slab = (name, xa, xb, nSeg) => { const secs = []; for (let k = 0; k <= nSeg; k++) { const x = xa + (xb - xa) * k / nSeg; const ring = []; for (let j = 0; j <= 6; j++) ring.push([x, -0.25, j * T / 6]); for (let j = 1; j <= 4; j++) ring.push([x, -0.25 + 0.5 * j / 4, T]); for (let j = 5; j >= 0; j--) ring.push([x, 0.25, j * T / 6]); for (let j = 3; j >= 1; j--) ring.push([x, -0.25 + 0.5 * j / 4, 0]); secs.push(ring); } return loft(name, secs, { color: '#999' }); };
  const ssK = (i, j) => Z[Math.max(0, Math.min(ny - 1, j))][Math.max(0, Math.min(nx - 1, i))] + KELVIN0;
  const KELVIN0 = 273.15;
  const Th_ss = tr.y[tr.y.length - 1][0], Tc_ss = tr.y[tr.y.length - 1][1];
  let fH = 1, fC = 1;
  const valAt = (x, y, z) => { const i = x < xH1 ? Math.floor((x - x0) / Lh * nH) : x < xC0 ? nH + Math.floor((x - xH1) / Lg * nI) : nH + nI + Math.floor((x - xC0) / Lh * nH); const j = Math.floor(z / T * ny); const Tss = ssK(i, j), side = x < (xH1 + xC0) / 2 ? fH : fC; return TaK + (Tss - TaK) * side - KELVIN0; };
  const all = Z.flat(), lo = Math.min(...all, TaK - KELVIN0, Tc_ss - KELVIN0, S.Trad - KELVIN0), hi = Math.max(...all, TaK - KELVIN0, Th_ss - KELVIN0, S.Tcol - KELVIN0);
  const fm = fieldMesh(V, slab('Spreaders + isolation (2-D field)', x0, xC1, 3 * (nH + nI)), valAt, { cmapName: 'coolwarm', range: [lo, hi] });
  const add = (m, color) => { const o = V.add(m); o.material.color.set(color); return o; };
  const col = box('Collector / hot face', Lh * 0.95, 0.6, 0.03, { color: '#f97316' }); col.translate(x0 + Lh / 2, 0, T + 0.45); const oCol = add(col, '#f97316');
  const rad = box('Radiator / cold face', Lh * 0.95, 0.6, 0.03, { color: '#38bdf8' }); rad.translate(xC0 + Lh / 2, 0, T + 0.45); const oRad = add(rad, '#38bdf8');
  const legs = []; for (const [xa, name] of [[x0 + Lh / 2, 'Hot conduction path' + (nHP ? ` (+${nHP} heat pipes)` : '')], [xC0 + Lh / 2, 'Cold conduction path']]) { const b = box(name, 0.08, 0.08, 0.42, { color: '#b8c0cc' }); b.translate(xa, 0, T + 0.22); legs.push(add(b, '#b8c0cc')); }
  const act = box('Actuator (NiTi cartridge) between interfaces', xC0 - xH1 + 0.5, 0.3, 0.1, { color: '#c58b4b' }); act.translate((xH1 + xC0) / 2, 0, -0.07); const oAct = add(act, '#c58b4b');
  const arrow = (from, dir, color) => { const a = new THREE.ArrowHelper(new THREE.Vector3(...dir).normalize(), new THREE.Vector3(...from), 0.3, color, 0.1, 0.06); V.root.add(a); return a; };
  const aIn = arrow([x0 + Lh / 2 + 0.15, 0.32, T + 0.42], [0, 0, -1], 0xf97316), aConv = arrow([xH1 - 0.1, 0.32, -0.02], [1, 0, 0], 0xc58b4b), aLeak = arrow([xH1, 0, T + 0.06], [1, 0, 0], 0xef4444), aOut = arrow([xC0 + Lh / 2 + 0.15, 0.32, T + 0.05], [0, 0, 1], 0x38bdf8);
  const trHot = tracers(V, [[x0 + Lh / 2, 0.1, T + 0.45], [x0 + Lh / 2, 0.1, T], [xH1, 0.1, T / 2], [xH1 + 0.05, 0.1, -0.07], [xC0 - 0.05, 0.1, -0.07], [xC0, 0.1, T / 2], [xC0 + Lh / 2, 0.1, T], [xC0 + Lh / 2, 0.1, T + 0.45]], { n: 14, color: 0xf97316 });
  const trLeak = tracers(V, [[xH1, -0.1, T / 2], [xC0, -0.1, T / 2]], { n: 4, color: 0xef4444, size: 0.018 });
  let lastT = 0;
  V.viewDir(0.35, -1.1, 0.6); V.controls.autoRotate = true; V.controls.autoRotateSpeed = 2; V.renderer.domElement.addEventListener('pointerdown', () => { V.controls.autoRotate = false; });
  const H = hud(wrap); legend3d(wrap, { title: 'Temperature', lo, hi, unit: ' °C', cmapName: 'coolwarm' });
  const tint = (o, Tc) => { const c = cmap((Tc - lo) / ((hi - lo) || 1), 'coolwarm'); o.material.color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255); };
  const tEnd = tr.t[tr.t.length - 1], Qscale = Math.max(1e-6, Math.abs(S.Qin), Math.abs(S.Qconv), Math.abs(S.Qleak));
  playback(wrap, { duration: tEnd, rate: tEnd / 20, label: 'Time from start-up', unit: 's', onFrame: (t) => {
    const Th = interpT(tr.t, tr.y.map(y => y[0]), t), Tc = interpT(tr.t, tr.y.map(y => y[1]), t);
    fH = (Th - TaK) / ((Th_ss - TaK) || 1); fC = (Tc - TaK) / ((Tc_ss - TaK) || 1); if (!Number.isFinite(fH)) fH = 1; if (!Number.isFinite(fC)) fC = 1;
    fm.update(valAt, [lo, hi]); tint(oCol, Th - KELVIN0); tint(oRad, Tc - KELVIN0); tint(legs[0], Th - KELVIN0); tint(legs[1], Tc - KELVIN0); tint(oAct, (Th + Tc) / 2 - KELVIN0);
    const g = Math.max(0, Math.min(1, Math.abs(Th - TaK) / (Math.abs(Th_ss - TaK) || 1))); const len = (q) => 0.12 + 0.5 * Math.abs(q) / Qscale * g;
    { const dt = Math.max(0, Math.min(0.1, (performance.now() - lastT) / 1000)); lastT = performance.now(); trHot.step(dt, 0.08 + 0.35 * Math.abs(S.Qconv) / Qscale * Math.max(0.05, g)); trLeak.step(dt, 0.05 + 0.35 * Math.abs(S.Qleak) / Qscale * Math.max(0.05, g)); }
    aIn.setLength(len(S.Qin), 0.1, 0.06); aConv.setLength(len(S.Qconv), 0.1, 0.06); aLeak.setLength(len(S.Qleak), 0.1, 0.06); aOut.setLength(len(S.Qconv), 0.1, 0.06);
    H.set([['Time', fmt(t, 3) + ' s'], ['Hot node', fmt(Th - KELVIN0, 4) + ' °C', '#f97316'], ['Cold node', fmt(Tc - KELVIN0, 4) + ' °C', '#38bdf8'], ['ΔT hot–cold', fmt(Th - Tc, 3) + ' K'], ['Steady Q_in', fmt(S.Qin, 3) + ' W'], ['Steady Q_conv', fmt(S.Qconv, 3) + ' W'], ['Steady Q_leak', fmt(S.Qleak, 3) + ' W', '#ef4444'], ['Response', fmt(100 * g, 3) + ' % of steady']]);
  } });
}
function interpT(ts, ys, t) { if (t <= ts[0]) return ys[0]; for (let i = 1; i < ts.length; i++) if (ts[i] >= t) { const f = (t - ts[i - 1]) / ((ts[i] - ts[i - 1]) || 1); return ys[i - 1] + f * (ys[i] - ys[i - 1]); } return ys[ys.length - 1]; }

function interpX(xs, ys, x) { const n = xs.length; if (x <= xs[0]) return ys[0] + (ys[1] - ys[0]) * (x - xs[0]) / ((xs[1] - xs[0]) || 1); if (x >= xs[n - 1]) return ys[n - 1] + (ys[n - 1] - ys[n - 2]) * (x - xs[n - 1]) / ((xs[n - 1] - xs[n - 2]) || 1); let i = 1; while (xs[i] < x) i++; const f = (x - xs[i - 1]) / ((xs[i] - xs[i - 1]) || 1); return ys[i - 1] + f * (ys[i] - ys[i - 1]); }

// 2D animated temperature contour of the spreader–isolation–spreader section: solved steady 2-D field scaled at each
// instant by the solved transient node responses (hot and cold side), with the node temperature traces and cursor.
async function anim2dField(el, { tr, Z, nx, ny, nH, nI, TaK, S }) {
  const [{ anim2d }, { h }, { cmap }] = await Promise.all([import('../core/sim3d.js'), import('../core/ui.js'), import('../core/chart.js')]);
  el.append(h('p', { class: 'small muted' }, 'Top: section temperature (x along hot spreader | isolation | cold spreader, y through the thickness ×-stretched). Bottom: solved hot/cold node temperatures from start-up with the time cursor.'));
  const K0 = 273.15, Ths = tr.y[tr.y.length - 1][0], Tcs = tr.y[tr.y.length - 1][1], all = Z.flat(), lo = Math.min(...all, TaK - K0, Tcs - K0), hi = Math.max(...all, TaK - K0, Ths - K0), tEnd = tr.t[tr.t.length - 1];
  const Th = tr.y.map(y => y[0] - K0), Tc = tr.y.map(y => y[1] - K0);
  anim2d(el, { height: 440, duration: tEnd, rate: tEnd / 20, label: 'Time from start-up', unit: 's', onFrame: (t, g) => {
    g.clear(); let j = tr.t.findIndex(x => x >= t); if (j < 0) j = tr.t.length - 1;
    const fH = (Th[j] + K0 - TaK) / ((Ths - TaK) || 1), fC = (Tc[j] + K0 - TaK) / ((Tcs - TaK) || 1), x0 = 60, y0 = 26, w = g.W - 160, hgt = g.H * 0.45;
    for (let jj = 0; jj < ny; jj++) for (let i = 0; i < nx; i++) { const side = i < nH + nI / 2 ? fH : fC, v = TaK - K0 + (Z[jj][i] + K0 - TaK) * (Number.isFinite(side) ? side : 1), c = cmap((v - lo) / ((hi - lo) || 1), 'coolwarm'); g.ctx.fillStyle = `rgb(${c.map(Math.round).join(',')})`; g.ctx.fillRect(x0 + i * w / nx, y0 + (ny - 1 - jj) * hgt / ny, w / nx + 1, hgt / ny + 1); }
    g.ctx.strokeStyle = g.fg; g.ctx.strokeRect(x0, y0, w, hgt); g.ctx.strokeRect(x0 + nH * w / nx, y0, nI * w / nx, hgt);
    { const now = performance.now() / 1000, sp = 0.08 + 0.25 * Math.min(1, Math.abs(S.Qconv) / Math.max(1e-9, Math.abs(S.Qin))); g.ctx.fillStyle = 'rgba(255,255,255,.9)'; for (let q = 0; q < 16; q++) { const f = (now * sp + q / 16) % 1; g.ctx.beginPath(); g.ctx.arc(x0 + f * w, y0 + hgt * (0.2 + 0.3 * (q % 3)), 6, 0, 2 * Math.PI); g.ctx.fill(); } g.text('● heat-flow tracers (speed ∝ solved Q_conv / Q_in)', x0, y0 + hgt + 12, { size: 10, color: g.mute }); }
    g.text('hot spreader', x0 + 4, y0 - 6, { size: 11 }); g.text('isolation', x0 + nH * w / nx + 2, y0 - 6, { size: 11 }); g.text('cold spreader', x0 + (nH + nI) * w / nx + 4, y0 - 6, { size: 11 });
    for (let k = 0; k <= 10; k++) { const c = cmap(k / 10, 'coolwarm'); g.ctx.fillStyle = `rgb(${c.map(Math.round).join(',')})`; g.ctx.fillRect(g.W - 80, y0 + hgt - (k + 1) * hgt / 11, 14, hgt / 11 + 1); }
    g.text(fmt(hi, 3) + ' °C', g.W - 62, y0 + 10, { size: 10 }); g.text(fmt(lo, 3) + ' °C', g.W - 62, y0 + hgt, { size: 10 });
    const P = g.panel(0, y0 + hgt + 10, g.W, g.H - hgt - y0 - 10, { xr: [0, tEnd], yr: [lo - 0.5, hi + 0.5], xlabel: 'Time [s]', ylabel: 'T [°C]', title: `Node temperatures — hot ${fmt(Th[j], 4)} °C, cold ${fmt(Tc[j], 4)} °C, ΔT ${fmt(Th[j] - Tc[j], 3)} K` });
    P.line(tr.t, Th, '#ef4444', 2); P.line(tr.t, Tc, '#3b82f6', 2); P.vline(t); P.point(tr.t[j], Th[j], '#ef4444'); P.point(tr.t[j], Tc[j], '#3b82f6');
  } });
}
