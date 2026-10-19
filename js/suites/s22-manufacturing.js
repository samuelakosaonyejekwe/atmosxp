// Suite 22 — Manufacturing, additive fabrication & quality engineering.
import { h, table } from '../core/ui.js';
import { fmt, linspace, range, mean, std, rng, normCdf, histogram, percentile } from '../core/num.js';
import { bom, avgCost, unitCost } from '../core/econ.js';
import { cachedSiteContext } from '../core/advisor.js';
import { COMPONENTS } from '../core/materials.js';

const AM = {
  'L-PBF Ti-6Al-4V': { minStrut: 0.30, minHole: 0.8, overhang: 45, Ra: 12, density: 99.7, rate: 12, costCm3: 2.2, fatigueKd: 0.45 },
  'EBM Ti-6Al-4V': { minStrut: 0.60, minHole: 1.2, overhang: 50, Ra: 25, density: 99.5, rate: 55, costCm3: 1.8, fatigueKd: 0.35 },
  'L-PBF NiTi': { minStrut: 0.25, minHole: 0.8, overhang: 45, Ra: 10, density: 99.3, rate: 6, costCm3: 4.5, fatigueKd: 0.4 },
  'SLS PA12': { minStrut: 0.80, minHole: 1.5, overhang: 90, Ra: 15, density: 96, rate: 120, costCm3: 0.15, fatigueKd: 0.6 },
};
const R = 8.314;

export default {
  description: 'Manufacturing engineering for every part: dimensional tolerance stacks (worst case, RSS, Monte Carlo) and process capability for the hub cartridge assembly, composite cure kinetics with vitrification and residual stress, additive-manufacturing feasibility of metamaterial lattices, NiTi transformation-temperature variability, bill of materials, manufacturing routes, quality plan and inspection intervals.',
  literature: 'composite cure kinetics residual stress additive manufacturing lattice NiTi transformation temperature variability tolerance analysis',
  credibility: 'Analytical screening', econ: true, sustain: true,
  equations: [
    'Worst-case and root-sum-square tolerance stacks; Monte Carlo tolerance analysis with normal process distributions',
    'Process capability indices C_p = (USL−LSL)/6σ and C_pk = min(USL−μ, μ−LSL)/3σ; expected yield from the normal CDF',
    'Kamal–Sourour autocatalytic cure kinetics dα/dt = (k₁ + k₂αᵐ)(1−α)ⁿ with Arrhenius rate constants',
    'DiBenedetto relation T_g(α) and Cole diffusion-control factor for vitrification',
    'Thermo-elastic residual stress from CTE mismatch between stress-free temperature and room temperature; chemical-shrinkage contribution',
    'Radford spring-in Δθ = θ(α_L − α_T)ΔT/(1 + α_TΔT) for curved laminates',
    'NiTi transformation-temperature sensitivity to Ni content (≈ −90 K per at%) and heat-treatment scatter; Clausius–Clapeyron stress shift',
    'Additive-manufacturing design rules: minimum strut diameter, self-supporting overhang angle, powder-removal aperture, surface-roughness fatigue knock-down',
    'Wright learning curve for production cost',
  ],
  methods: ['Explicit time integration of cure kinetics (Δt = 10 s) along the oven/autoclave cycle', 'Cure-window map: final degree of cure over dwell temperature × dwell time grid', '20 000-sample Monte Carlo tolerance and transformation-temperature simulations (seeded, reproducible)'],
  assumptions: ['Generic 180 °C-class epoxy kinetic parameters (replace by DSC-fitted values)', 'Process distributions normal, σ = tolerance / 3 × process factor', 'Component volumes from the reference design envelopes (first-order estimates)'],
  ibc: {
    initial: ['Raw-material properties, starting geometry, build orientation', 'Material temperature, residual stress, phase state, manufacturing tolerances, process history'],
    boundary: ['Manufacturing heat input, cooling schedules, cure pressure, mould constraints', 'Tool contact, machining supports, AM substrate constraints, thermal exchange, post-processing loads'],
  },
  cvv: {
    calibration: ['Dimensional metrology (CMM, 3D scanning)', 'DSC cure kinetics, rheometry', 'AM distortion scans, batch DSC of NiTi transformation temperatures'],
    verification: ['RSS ≤ worst-case identity', 'Isothermal cure analytic limit (n-th order) vs integrator', 'Cpk from MC vs analytic normal'],
    validation: ['First-article inspection of cartridges & hub', 'Witness-coupon cure (DMA T_g) and laminate warpage measurement'],
  },
  formats: { import: ['CSV metrology logs', 'XLSX supplier data', 'STL/OBJ scanned geometry'], export: ['BOM CSV/JSON', 'Quality plan CSV', 'Plots PNG/SVG'] },
  params: [
    { key: 'quantity', label: 'Production quantity', type: 'number', min: 1, max: 100000, step: 1, group: 'Production', default: 50 },
    { key: 'learning', label: 'Learning-curve progress ratio', type: 'range', min: 0.7, max: 1, step: 0.01, group: 'Production', default: 0.85 },
    { key: 'labourRate', label: 'Labour rate', unit: 'USD/h', type: 'number', min: 2, max: 150, step: 1, group: 'Production', default: 35 },
    { key: 'grid', label: 'Factory grid carbon intensity', unit: 'gCO₂/kWh', type: 'number', min: 0, max: 1200, step: 10, group: 'Production', default: 450, auto: () => { const c = cachedSiteContext(); return c?.gridCarbon?.value ? { value: Math.round(c.gridCarbon.value), prov: 'live' } : undefined; } },
    { key: 'nCart', label: 'Cartridges per driven wheel', type: 'number', min: 4, max: 64, step: 1, group: 'Hub cartridge assembly', default: 24 },
    { key: 'gapNom', label: 'Nominal shoe–cam clearance', unit: 'mm', type: 'number', min: 0.1, max: 2, step: 0.05, group: 'Hub cartridge assembly', default: 0.5 },
    { key: 'gapLSL', label: 'Clearance lower spec limit', unit: 'mm', type: 'number', min: 0, max: 1, step: 0.05, group: 'Hub cartridge assembly', default: 0.3 },
    { key: 'gapUSL', label: 'Clearance upper spec limit', unit: 'mm', type: 'number', min: 0.2, max: 3, step: 0.05, group: 'Hub cartridge assembly', default: 0.7 },
    { key: 'tolScale', label: 'Tolerance grade multiplier', type: 'range', min: 0.25, max: 3, step: 0.05, group: 'Hub cartridge assembly', default: 1, help: '1 = baseline drawing tolerances; <1 tighter (costlier) machining.' },
    { key: 'procFactor', label: 'Process spread factor (σ = tol/3 × f)', type: 'range', min: 0.5, max: 2, step: 0.05, group: 'Hub cartridge assembly', default: 1 },
    { key: 'kFlex', label: 'Flexure preload stiffness', unit: 'N/mm', type: 'number', min: 10, max: 5000, step: 10, group: 'Hub cartridge assembly', default: 600 },
    { key: 'dTop', label: 'Operating temperature excursion', unit: 'K', type: 'number', min: 0, max: 80, step: 1, group: 'Hub cartridge assembly', default: 25 },
    { key: 'niSigma', label: 'Ni content scatter (1σ)', unit: 'at%', type: 'number', min: 0, max: 0.3, step: 0.01, group: 'NiTi variability', default: 0.05 },
    { key: 'htSigma', label: 'Heat-treatment Af scatter (1σ)', unit: 'K', type: 'number', min: 0, max: 15, step: 0.5, group: 'NiTi variability', default: 3 },
    { key: 'actStress', label: 'Cartridge operating stress', unit: 'MPa', type: 'number', min: 0, max: 400, step: 10, group: 'NiTi variability', default: 150, auto: (c) => { const o = c.upstream('s15'); return o?.recoveryStress_MPa ? { value: Math.round(Math.min(400, o.recoveryStress_MPa * 0.6)), prov: 'coupled' } : undefined; } },
    { key: 'Thot', label: 'Hot-interface temperature', unit: '°C', type: 'number', min: 0, max: 200, step: 1, group: 'NiTi variability', default: 60, auto: (c) => { const o = c.upstream('s14'); return o?.ThotIface_C !== undefined ? { value: +o.ThotIface_C.toFixed(1), prov: 'coupled' } : undefined; } },
    { key: 'cureT', label: 'Cure dwell temperature', unit: '°C', type: 'number', min: 80, max: 220, step: 5, group: 'Composite cure', default: 180 },
    { key: 'cureTime', label: 'Cure dwell time', unit: 'min', type: 'number', min: 10, max: 480, step: 5, group: 'Composite cure', default: 120 },
    { key: 'ramp', label: 'Heating ramp rate', unit: 'K/min', type: 'number', min: 0.5, max: 10, step: 0.5, group: 'Composite cure', default: 2 },
    { key: 'E2kin', label: 'Activation energy E₂', unit: 'kJ/mol', type: 'number', min: 40, max: 100, step: 1, group: 'Composite cure', default: 66, expert: true },
    { key: 'A2kin', label: 'Pre-exponential A₂', unit: '1/s', type: 'number', min: 1e3, max: 1e6, step: 1e3, group: 'Composite cure', default: 1e5, expert: true },
    { key: 'shrink', label: 'Resin volumetric cure shrinkage', unit: '%', type: 'number', min: 0, max: 8, step: 0.5, group: 'Composite cure', default: 3, expert: true },
    { key: 'amProc', label: 'Lattice AM process', type: 'select', group: 'Additive manufacturing', default: 'L-PBF Ti-6Al-4V', options: Object.keys(AM) },
    { key: 'strut', label: 'Lattice strut diameter', unit: 'mm', type: 'number', min: 0.1, max: 5, step: 0.05, group: 'Additive manufacturing', default: 0.8 },
    { key: 'cell', label: 'Unit-cell size', unit: 'mm', type: 'number', min: 1, max: 30, step: 0.5, group: 'Additive manufacturing', default: 6 },
    { key: 'incl', label: 'Strut inclination from build plate', unit: '°', type: 'number', min: 0, max: 90, step: 1, group: 'Additive manufacturing', default: 50 },
  ],
  presets: [{ label: 'Prototype (5 off)', values: { quantity: 5 } }, { label: 'Pilot series (500)', values: { quantity: 500 } }, { label: 'Precision hub (tight tol.)', values: { tolScale: 0.5 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const R0 = rng(2201);
    // ---------- tolerance stack ----------
    const t = p.tolScale, act = ctx.material('actuator'), hous = ctx.material('cartridgeHousing'), cam = ctx.material('cam');
    const ring = 86.6 + 4 + 34 + p.gapNom;
    const stack = [
      { name: 'Containment ring inner radius', nom: ring, tol: 0.05 * t, sign: +1 },
      { name: 'Cartridge length', nom: 34, tol: 0.03 * t, sign: -1 },
      { name: 'Flexure shoe thickness', nom: 4, tol: 0.02 * t, sign: -1 },
      { name: 'Cam radius at contact', nom: 86.6, tol: 0.03 * t, sign: -1 },
      { name: 'Bearing radial run-out', nom: 0, tol: 0.02 * t, sign: -1 },
      { name: 'Cam/axle concentricity', nom: 0, tol: 0.025 * t, sign: -1 },
    ];
    // differential thermal growth (housing vs cam+cartridge) at the operating excursion — a deterministic bias
    const thermalBias = (ring * (hous.alpha || 9) - (86.6 * (cam.alpha || 11) + 34 * (act.alpha || 11) + 4 * 9)) * 1e-6 * p.dTop;
    const gapNomCalc = stack.reduce((s, i) => s + i.sign * i.nom, 0);
    const wc = stack.reduce((s, i) => s + i.tol, 0), rss = Math.sqrt(stack.reduce((s, i) => s + i.tol ** 2, 0));
    const N = 20000, gaps = range(N).map(() => stack.reduce((s, i) => s + i.sign * (i.nom + R0.normal(0, i.tol / 3 * p.procFactor)), 0) + thermalBias);
    const mu = mean(gaps), sg = std(gaps), Cp = (p.gapUSL - p.gapLSL) / (6 * sg), Cpk = Math.min(p.gapUSL - mu, mu - p.gapLSL) / (3 * sg);
    const yieldMC = gaps.filter(g => g >= p.gapLSL && g <= p.gapUSL).length / N, yieldAn = normCdf((p.gapUSL - mu) / sg) - normCdf((p.gapLSL - mu) / sg);
    const preloadVar = p.kFlex * 3 * sg, assemblyYield = yieldMC ** (2 * p.nCart);
    // ---------- NiTi transformation-temperature variability ----------
    const AfNom = (act.Af ?? 60) + p.actStress / (act.CA || 7), AfSamples = range(N).map(() => AfNom + R0.normal(0, Math.hypot(90 * p.niSigma, p.htSigma)));
    const margin = 3, fullTransform = AfSamples.filter(a => a + margin <= p.Thot).length / N, AfSig = std(AfSamples);
    // ---------- composite cure ----------
    const chas = ctx.material('chassis'), A1 = 2e3, E1 = 66e3, A2 = p.A2kin, E2 = p.E2kin * 1e3, mK = 1, nK = 1.5, Tg0 = 0, TgInf = 200, lam = 0.4, Ccole = 30;
    const Tg = (a) => Tg0 + (TgInf - Tg0) * lam * a / (1 - (1 - lam) * a);
    const cure = (Td, dwellMin, ramp) => {
      const dt = 10, tRamp = (Td - 20) / ramp * 60, tEnd = tRamp + dwellMin * 60 + (Td - 40) / 3 * 60; let a = 0, T = 20, hist = { t: [], T: [], a: [], Tg: [] }; let tt = 0;
      while (tt < tEnd) {
        T = tt < tRamp ? 20 + ramp * tt / 60 : tt < tRamp + dwellMin * 60 ? Td : Math.max(40, Td - 3 * (tt - tRamp - dwellMin * 60) / 60);
        const TK = T + 273.15, k1 = A1 * Math.exp(-E1 / (R * TK)), k2 = A2 * Math.exp(-E2 / (R * TK));
        const ac = (() => { // α at which Tg reaches the cure temperature (vitrification)
          let lo = 0, hi = 1; for (let i = 0; i < 30; i++) { const m = (lo + hi) / 2; if (Tg(m) < T) lo = m; else hi = m; } return lo; })();
        const fd = 1 / (1 + Math.exp(Ccole * (a - ac - 0.05)));
        a = Math.min(0.999, a + dt * fd * (k1 + k2 * a ** mK) * (1 - a) ** nK);
        if (hist.t.length === 0 || tt % 60 < dt) { hist.t.push(tt / 60); hist.T.push(T); hist.a.push(a); hist.Tg.push(Tg(a)); }
        tt += dt;
      }
      return { a, Tg: Tg(a), hist, total: tEnd / 60 };
    };
    const cr = cure(p.cureT, p.cureTime, p.ramp);
    const Tsf = Math.min(p.cureT, cr.Tg), dTcool = Tsf - 20;
    const isComp = chas.family === 'Composite', a1 = (chas.alpha ?? 2) * 1e-6, a2 = (chas.alpha2 ?? 30) * 1e-6, E1m = (chas.E || 70) * 1e3, E2m = (chas.E2 && chas.E2 < chas.E * 0.5 ? chas.E2 : 9) * 1e3; // through-thickness / transverse matrix-dominated values
    const sigThermal = isComp ? Math.abs(a2 - a1) * dTcool * E1m * E2m / (E1m + E2m) : 0;
    const sigChem = isComp ? (p.shrink / 100 / 3) * Math.max(0, cr.a - 0.6) / 0.4 * E2m * 0.3 : 0;
    const sigRes = sigThermal + sigChem, Yt = chas.Yt ?? chas.su ?? 60;
    const springIn = isComp ? 90 * ((a2 - a1) * dTcool) / (1 + a2 * dTcool) : 0;
    // cure window map
    const Ts = linspace(120, 200, 17), tms = linspace(20, 300, 15);
    const Z = tms.map(tm => Ts.map(Td => cure(Td, tm, p.ramp).a));
    // ---------- additive manufacturing ----------
    const amp = AM[p.amProc], opening = p.cell - p.strut;
    const amChecks = [
      { test: 'Minimum printable strut diameter', status: p.strut >= amp.minStrut * 1.5 ? 'pass' : p.strut >= amp.minStrut ? 'warn' : 'fail', detail: `${p.strut} mm vs process minimum ${amp.minStrut} mm (≥1.5× recommended for fatigue-critical struts).`, fix: 'Increase strut diameter or choose a finer process.' },
      { test: 'Self-supporting overhang', status: p.incl >= 90 - amp.overhang ? 'pass' : 'fail', detail: `Strut inclination ${p.incl}° from the plate; process requires ≥ ${90 - amp.overhang}° without supports.`, fix: 'Re-orient the build or switch to a cell topology with steeper struts (e.g. BCCz, octet at 45°+).' },
      { test: 'Powder removal aperture', status: opening >= amp.minHole * 1.5 ? 'pass' : opening >= amp.minHole ? 'warn' : 'fail', detail: `Cell opening ${fmt(opening, 3)} mm vs minimum ${amp.minHole} mm for depowdering.`, fix: 'Increase cell size, add drain channels or reduce strut diameter.' },
    ];
    // ---------- BOM ----------
    const B = bom(ctx.store, { grid: p.grid, labourRate: p.labourRate });
    const proto = B.reduce((s, r) => s + r.fabCost, 0), mass = B.reduce((s, r) => s + r.mass, 0), matCost = B.reduce((s, r) => s + r.matCost, 0);
    const avgUnit = avgCost(proto, p.learning, p.quantity), lastUnit = unitCost(proto, p.learning, p.quantity);
    // ---------- KPIs ----------
    const k = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('BOM structural mass', mass, 'kg', 'Sum of all component masses from current material assignments × reference volumes (estimate; payload excluded).', 'estimated');
    k('Raw material cost', matCost, 'USD', 'Mass × indicative price for every component.', 'estimated');
    k('First-unit (prototype) cost', proto, 'USD', 'Fabricated cost: material × process factor + labour hours × rate.', 'estimated');
    k(`Average unit cost over ${p.quantity}`, avgUnit, 'USD', `Wright learning curve with ${Math.round(p.learning * 100)} % progress ratio.`, 'calc');
    k('Clearance stack — worst case', wc, '± mm', 'Arithmetic sum of tolerances (100 % interchangeability).', 'calc');
    k('Clearance stack — RSS', rss, '± mm', 'Statistical root-sum-square stack (≈ 3σ if parts are independent and centred).', 'calc');
    k('Mean operating clearance', mu, 'mm', `Monte Carlo mean including differential thermal growth of ${fmt(thermalBias * 1000, 3)} µm at ΔT = ${p.dTop} K.`, 'calc');
    k('Process capability Cpk', Cpk, '', 'C_pk ≥ 1.33 is the usual automotive acceptance for a key characteristic.', 'calc', Cpk >= 1.33 ? 'ok' : Cpk >= 1 ? 'warn' : 'bad');
    k('Clearance yield per cartridge', yieldMC * 100, '%', 'Fraction of Monte Carlo assemblies inside the specification limits.', 'calc');
    k(`Hub assembly first-pass yield (${2 * p.nCart} cartridges)`, assemblyYield * 100, '%', 'Probability that every cartridge in both driven wheels is in specification without rework.', 'calc', assemblyYield > 0.9 ? 'ok' : assemblyYield > 0.5 ? 'warn' : 'bad');
    k('Preload variation (±3σ)', preloadVar, 'N', 'Clearance scatter × flexure stiffness: the spread of cartridge preload that the hub solver must tolerate.', 'calc');
    k('Af under load (mean)', AfNom, '°C', `A_f + σ/C_A for ${act.name} at ${p.actStress} MPa.`, 'calc');
    k('Af scatter (1σ)', AfSig, 'K', 'Combined composition and heat-treatment variability.', 'calc');
    k('Cartridges fully transforming', fullTransform * 100, '%', `Share whose A_f(σ) + ${margin} K margin is below the hot-interface temperature ${p.Thot} °C.`, p.Thot !== 60 ? 'coupled' : 'estimated', fullTransform > 0.95 ? 'ok' : fullTransform > 0.5 ? 'warn' : 'bad');
    k('Final degree of cure', cr.a, '', 'End-of-cycle conversion (≥ 0.95 typical requirement for structural laminates).', 'calc', cr.a >= 0.95 ? 'ok' : cr.a >= 0.9 ? 'warn' : 'bad');
    k('Final glass transition T_g', cr.Tg, '°C', 'DiBenedetto estimate; must exceed maximum service temperature by ≥ 30 K.', 'calc');
    k('Residual transverse stress', sigRes, 'MPa', `Thermal (${fmt(sigThermal, 3)} MPa) + chemical shrinkage (${fmt(sigChem, 3)} MPa) estimate in a cross-ply laminate.`, 'estimated');
    k('Spring-in of a 90° corner', springIn, '°', 'Radford relation — compensate in the mould geometry.', 'estimated');
    // verdicts
    out.verdicts.push({ test: 'Hub clearance capability (Cpk ≥ 1.33)', status: Cpk >= 1.33 ? 'pass' : Cpk >= 1 ? 'warn' : 'fail', detail: `Cpk ${fmt(Cpk, 3)}, Cp ${fmt(Cp, 3)}; mean ${fmt(mu, 4)} mm in [${p.gapLSL}, ${p.gapUSL}] mm.`, fix: 'Tighten the dominant tolerance (see contribution chart), centre the nominal, or add selective assembly / shimming.' });
    out.verdicts.push({ test: 'NiTi transformation within available hot temperature', status: fullTransform > 0.95 ? 'pass' : fullTransform > 0.5 ? 'warn' : 'fail', detail: `${fmt(fullTransform * 100, 3)} % of cartridges complete A→ transformation at ${p.Thot} °C.`, fix: 'Lower A_f by alloy/heat-treatment selection, reduce operating stress, or raise the hot-interface temperature (Suites 13–14). Batch DSC screening of every cartridge is mandatory.' });
    out.verdicts.push({ test: 'Laminate degree of cure', status: cr.a >= 0.95 ? 'pass' : cr.a >= 0.9 ? 'warn' : 'fail', detail: `α = ${fmt(cr.a, 3)}, T_g = ${fmt(cr.Tg, 3)} °C after ${fmt(cr.total, 3)} min cycle.`, fix: 'Extend dwell or raise cure temperature (see cure-window map); add a post-cure.' });
    out.verdicts.push({ test: 'Residual-stress microcracking risk', status: !isComp ? 'info' : sigRes < 0.5 * Yt ? 'pass' : sigRes < 0.8 * Yt ? 'warn' : 'fail', detail: isComp ? `${fmt(sigRes, 3)} MPa vs transverse strength ${Yt} MPa.` : 'Chassis material is not a fibre composite.' });
    out.verdicts.push(...amChecks);
    // checks
    out.checks.push({ name: 'RSS ≤ worst case', value: rss / wc, target: '≤ 1', pass: rss <= wc + 1e-12 });
    out.checks.push({ name: 'Stack nominal = specified nominal clearance', value: gapNomCalc, target: String(p.gapNom), pass: Math.abs(gapNomCalc - p.gapNom) < 1e-9 });
    out.checks.push({ name: 'Monte Carlo yield vs analytic normal yield', value: yieldMC, target: fmt(yieldAn, 4) + ' ± 0.01', pass: Math.abs(yieldMC - yieldAn) < 0.01 });
    const iso = cure(p.cureT, 600, 50).a; out.checks.push({ name: 'Long isothermal cure approaches vitrification-limited α', value: iso, target: '0.9–1.0', pass: iso > 0.9 && iso <= 1 });
    // charts
    const hgap = histogram(gaps, 40);
    out.charts.push({ title: 'Monte Carlo shoe–cam clearance distribution', type: 'line', x: { label: 'Clearance', unit: 'mm' }, y: { label: 'Assemblies' }, series: [{ name: 'MC histogram', x: hgap.centers, y: hgap.counts, area: true }], annotations: [{ type: 'vline', x: p.gapLSL, label: 'LSL', color: '#ef4444' }, { type: 'vline', x: p.gapUSL, label: 'USL', color: '#ef4444' }, { type: 'vline', x: mu, label: 'mean', color: '#22c55e' }], explain: '20 000 virtual assemblies with normally distributed part dimensions. Area outside the red limits is scrap/rework; its share determines Cpk and the first-pass yield of a 48-cartridge hub.' });
    out.charts.push({ title: 'Tolerance contribution to clearance variance', type: 'bar', horizontal: true, categories: stack.map(s => s.name), y: { label: 'Variance share', unit: '%' }, series: [{ name: 'share', values: stack.map(s => 100 * s.tol ** 2 / (rss ** 2)) }], showValues: true, explain: 'Each tolerance’s share of the RSS variance — tighten the top bar first for the cheapest capability gain.' });
    const haf = histogram(AfSamples, 40);
    out.charts.push({ title: 'Cartridge A_f(σ) distribution vs hot-interface temperature', type: 'line', x: { label: 'A_f under load', unit: '°C' }, y: { label: 'Cartridges' }, series: [{ name: 'A_f(σ) samples', x: haf.centers, y: haf.counts, area: true, color: '#c58b4b' }], annotations: [{ type: 'vline', x: p.Thot - margin, label: 'T_hot − margin', color: '#ef4444' }], explain: 'Composition (Ni at%) and heat-treatment scatter spread the austenite-finish temperature; cartridges to the right of the line will not complete their recovery stroke — a direct loss of net work in Suite 17.' });
    out.charts.push({ title: 'Cure cycle: temperature, degree of cure and T_g', type: 'line', x: { label: 'Time', unit: 'min' }, y: { label: 'Temperature / T_g', unit: '°C' }, series: [{ name: 'Part temperature', x: cr.hist.t, y: cr.hist.T, color: '#ef4444' }, { name: 'T_g(α)', x: cr.hist.t, y: cr.hist.Tg, color: '#8b5cf6' }, { name: 'α × 100', x: cr.hist.t, y: cr.hist.a.map(a => a * 100), color: '#22c55e' }], explain: 'Kamal autocatalytic kinetics integrated along the ramp–dwell–cool cycle. When T_g rises to the part temperature the resin vitrifies and cure slows (diffusion control) — the reason post-cures or higher dwell temperatures are needed.' });
    out.charts.push({ title: 'Cure-window map: final degree of cure', type: 'heatmap', x: { label: 'Dwell temperature', unit: '°C', values: Ts }, y: { label: 'Dwell time', unit: 'min', values: tms }, z: Z, cmap: 'viridis', isolines: 6, z_label: 'α', explain: 'Final conversion for every dwell temperature/time pair at the chosen ramp. Choose the lowest-energy point inside the α ≥ 0.95 region to minimise cycle time and energy.' });
    out.charts.push({ title: 'Bill of materials — mass by component', type: 'bar', horizontal: true, categories: B.map(r => r.name), y: { label: 'Mass', unit: 'kg' }, series: [{ name: 'mass', values: B.map(r => r.mass) }], height: 640, explain: 'Component masses from current assignments; change materials in Suite 03 to see the effect.', wide: true });
    out.charts.push({ title: 'Prototype cost by component', type: 'bar', horizontal: true, categories: B.map(r => r.name), y: { label: 'Fabricated cost', unit: 'USD' }, series: [{ name: 'material', values: B.map(r => r.matCost) }, { name: 'fabricated (incl. labour)', values: B.map(r => r.fabCost) }], height: 640, explain: 'Material vs fabricated cost — high ratios flag labour- or process-intensive parts (candidates for design-for-manufacture).', wide: true });
    const qs = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000];
    out.charts.push({ title: 'Learning curve — unit and cumulative-average cost', type: 'line', x: { label: 'Units produced', log: true }, y: { label: 'Cost', unit: 'USD', log: true }, series: [{ name: 'Unit cost', x: qs, y: qs.map(n => unitCost(proto, p.learning, n)) }, { name: 'Cumulative average', x: qs, y: qs.map(n => avgCost(proto, p.learning, n)) }], annotations: [{ type: 'vline', x: p.quantity, label: 'plan' }], explain: 'Cost falls by (1 − progress ratio) at every doubling of output; production cost is passed to Suite 26.' });
    // tables
    out.tables.push({ title: 'Bill of materials', cols: ['Component', 'Material', 'Mass kg', 'Material USD', 'Fabricated USD', 'Labour h', 'Process energy kWh', 'Route'], rows: B.map(r => [r.name, r.material, r.mass, r.matCost, r.fabCost, r.laborH, r.procKWh, r.process]) });
    out.tables.push({ title: 'Hub tolerance stack (radial, mm)', cols: ['Dimension', 'Nominal', '± tol', 'Direction'], rows: [...stack.map(s => [s.name, s.nom, s.tol, s.sign > 0 ? '+' : '−']), ['Thermal growth bias at ΔT', thermalBias, 0, '±'], ['Resulting clearance (nominal)', gapNomCalc, wc, 'WC']] });
    out.tables.push({ title: 'Lattice additive-manufacturing process data', cols: ['Process', 'Min strut mm', 'Min hole mm', 'Overhang °', 'Ra µm', 'Density %', 'Build cm³/h', 'USD/cm³', 'As-built fatigue factor'], rows: Object.entries(AM).map(([n, a]) => [n, a.minStrut, a.minHole, a.overhang, a.Ra, a.density, a.rate, a.costCm3, a.fatigueKd]), explain: 'Typical machine-class values; qualify with the chosen machine and powder lot.' });
    out.tables.push({ title: 'Quality plan (key characteristics)', cols: ['Characteristic', 'Method', 'Frequency', 'Acceptance'], rows: [
      ['Shoe–cam clearance', 'Feeler / CMM at 8 angular positions', '100 % of hubs', `${p.gapLSL}–${p.gapUSL} mm, Cpk ≥ 1.33`],
      ['NiTi A_s/A_f per cartridge lot', 'DSC (ASTM F2004) + bend-free-recovery (ASTM F2082)', 'Every melt lot + 5 % of cartridges', `A_f(σ) + 3 K ≤ ${p.Thot} °C`],
      ['Cartridge recovery force & stroke', 'Thermal-cycling test rig, 50 cycles', '100 %', 'Within ±10 % of qualification'],
      ['Laminate degree of cure / T_g', 'DSC & DMA on witness coupons', 'Every cure cycle', `α ≥ 0.95, T_g ≥ T_service + 30 K`],
      ['Laminate porosity', 'Ultrasonic C-scan', '100 % of primary structure', '< 2 % voids, no delamination > 6 mm'],
      ['Wing-root insert bond', 'Proof load 1.25 × limit, acoustic emission', '100 %', 'No AE burst, no residual set'],
      ['AM lattice geometry', 'CT scan / 3D scan', 'First article + 1 per build', 'Strut ⌀ ± 10 %, no unfused powder'],
      ['Brake & steering', 'Function test, torque audit', '100 %', 'Spec values'],
    ] });
    out.tables.push({ title: 'Inspection intervals (service)', cols: ['Item', 'Interval', 'Basis'], rows: Object.entries(COMPONENTS).map(([kk, c]) => { const r = B.find(b => b.key === kk); const yrs = r.life; return [c.name, yrs <= 3 ? 'every 3 months / 2 500 km' : yrs <= 8 ? 'every 6 months / 5 000 km' : 'annual', `design life ≈ ${yrs} years (estimate; refine with Suite 23 fatigue results)`]; }) });
    out.views.push({ label: 'Manufacturing routes', icon: '🏭', render: (el) => el.append(table(['Component', 'Material', 'Route & conditions'], B.map(r => [r.name, r.material, r.process])), h('p', { class: 'small muted' }, 'Detailed conditions of manufacture, fixtures and inspection per component are in the Component Dossier.'), h('a', { class: 'btn', href: '#/dossier' }, 'Open dossier')) });
    // recs
    const worst = stack.slice().sort((a, b) => b.tol - a.tol)[0];
    if (Cpk < 1.33) out.recs.push({ level: 'warn', text: `Raise hub clearance capability: the largest contributor is “${worst.name}” (±${fmt(worst.tol, 3)} mm). Tightening it or using selective assembly (grade cartridges into length classes) lifts Cpk above 1.33.`, basis: 'Variance-contribution analysis' });
    if (fullTransform < 0.95) out.recs.push({ level: 'critical', text: `Only ${fmt(fullTransform * 100, 3)} % of cartridges reach full transformation. Specify a lower-A_f alloy, a tighter Ni window (±0.02 at%) and per-lot DSC acceptance, or reduce operating stress.`, basis: 'Transformation-temperature Monte Carlo' });
    if (cr.a < 0.95) out.recs.push({ level: 'warn', text: 'Extend cure dwell or add a free-standing post-cure; use the cure-window map to choose the lowest-energy compliant cycle.', basis: 'Kamal kinetics + vitrification' });
    out.recs.push({ level: 'sustain', text: `Processing electricity at ${p.grid} gCO₂/kWh: prefer out-of-autoclave prepregs and near-net-shape AM with powder reuse; schedule energy-intensive cures when grid carbon is low.`, basis: 'BOM process energy × grid intensity' });
    out.recs.push({ level: 'econ', text: `At ${p.quantity} units the average unit cost is ${fmt(avgUnit, 3)} USD (first unit ${fmt(proto, 3)} USD). Labour-dominated parts (high fabricated/material ratio) are the main targets for tooling investment.`, basis: 'Learning curve & BOM' });
    out.explain.push({ title: 'Why tolerances matter for the hub', text: 'The direct-rim drive transmits actuator work through contact between flexure shoes and a stationary cam. Clearance scatter changes preload and therefore the force–displacement path of every cartridge; too small and the cartridge binds (seizure risk), too large and stroke is lost to free play.' });
    out.outputs = { productionCost: avgUnit, prototypeCost: proto, Cpk, toleranceStack_mm: rss, bomMass_kg: mass, transformYield: fullTransform, degreeOfCure: cr.a, bom: B.map(r => ({ key: r.key, name: r.name, material: r.material, mass: r.mass, fabCost: r.fabCost, co2: r.co2, energy: r.energy, life: r.life, recyc: r.recyc, eolCredit: r.eolCredit })) };
    out.headline = `Prototype ${fmt(proto, 3)} USD → ${fmt(avgUnit, 3)} USD average at ${p.quantity} units · hub Cpk ${fmt(Cpk, 3)} · ${fmt(fullTransform * 100, 3)} % of cartridges transform fully · laminate α ${fmt(cr.a, 3)}.`;
    return out;
  },
};
void percentile;
