// Suite 03 — Engineering materials library, datasheets, assignment, Ashby selection and property prediction.
import { MATERIALS, COMPONENTS, get, forUse, families, indices, componentMaterial } from '../core/materials.js';
import { h, table, toast, callout } from '../core/ui.js';
import { chart, download } from '../core/chart.js';
import { fmt, linspace, range, clamp } from '../core/num.js';
import { pubchemProps, fetchJSON } from '../core/live.js';

const PROPS = [
  ['rho', 'Density', 'kg/m³'], ['E', 'Young’s modulus E₁', 'GPa'], ['E2', 'Transverse modulus E₂', 'GPa'], ['G12', 'Shear modulus G₁₂', 'GPa'], ['nu', 'Poisson’s ratio ν', ''],
  ['sy', 'Yield / design strength', 'MPa'], ['su', 'Ultimate strength', 'MPa'], ['Xt', 'Longitudinal tensile strength Xt', 'MPa'], ['Xc', 'Longitudinal compressive Xc', 'MPa'], ['Yt', 'Transverse tensile Yt', 'MPa'], ['Yc', 'Transverse compressive Yc', 'MPa'], ['S12', 'In-plane shear S₁₂', 'MPa'],
  ['fatigue', 'Fatigue (endurance MPa, or fraction of UTS for composites)', ''], ['k', 'Thermal conductivity', 'W/(m·K)'], ['k2', 'Transverse conductivity', 'W/(m·K)'], ['cp', 'Specific heat', 'J/(kg·K)'], ['alpha', 'Thermal expansion α₁', 'µm/(m·K)'], ['alpha2', 'Thermal expansion α₂', 'µm/(m·K)'],
  ['Tmax', 'Max service temperature', '°C'], ['Tmin', 'Min service temperature', '°C'],
  ['Ms', 'Martensite start Ms', '°C'], ['Mf', 'Martensite finish Mf', '°C'], ['As', 'Austenite start As', '°C'], ['Af', 'Austenite finish Af', '°C'], ['CM', 'Stress coefficient C_M', 'MPa/K'], ['CA', 'Stress coefficient C_A', 'MPa/K'], ['EA', 'Austenite modulus', 'GPa'], ['EM', 'Martensite modulus', 'GPa'], ['epsL', 'Max transformation strain', ''], ['Lh', 'Latent heat', 'J/kg'], ['hyst', 'Thermal hysteresis', 'K'],
  ['strainPerK', 'Actuation strain per kelvin', '1/K'], ['stressMax', 'Max actuation stress', 'MPa'], ['mu', 'Friction coefficient', ''], ['tanDelta', 'Loss factor tan δ', ''],
  ['cost', 'Indicative price', 'USD/kg'], ['co2', 'Embodied carbon (cradle-to-gate)', 'kg CO₂e/kg'], ['energy', 'Embodied energy', 'MJ/kg'], ['recyclable', 'Recyclable fraction (EoL)', ''], ['unc', 'Property uncertainty (±)', '%'],
];
const strength = (m) => m.sy ?? m.su ?? m.Xt ?? 0;

const CRITERIA = { stiffness: 'Specific stiffness E^½/ρ', strength: 'Specific strength σ^⅔/ρ', light: 'Low density', cost: 'Low price', carbon: 'Low embodied carbon', temperature: 'High service temperature', conduct: 'High thermal conductivity', resilience: 'Elastic energy σy²/2Eρ', recycle: 'Recyclability' };
const critVal = {
  stiffness: (m) => indices.stiffBeam(m), strength: (m) => indices.strongBeam(m), light: (m) => 1 / m.rho, cost: (m) => 1 / Math.max(0.1, m.cost), carbon: (m) => 1 / Math.max(0.1, m.co2),
  temperature: (m) => (m.Tmax ?? 50) + 273, conduct: (m) => Math.log10(1 + (m.k || 0)), resilience: (m) => Math.max(0, indices.specificResilience(m)), recycle: (m) => m.recyclable ?? 0,
};

export default {
  description: 'Traceable materials registry with full datasheets, per-component assignment, Ashby performance-index maps, uncertainty-aware multi-attribute ranking, Pareto screening, temperature-dependent property curves and live chemical reference look-up. Every material change propagates to all dependent suites.',
  literature: 'materials selection lightweight vehicle composite titanium shape memory alloy datasheet uncertainty',
  credibility: 'Analytical screening', sustain: true,
  equations: [
    'Generalised Hooke’s law; isotropic, orthotropic (lamina E₁, E₂, G₁₂, ν₁₂) and anisotropic elasticity',
    'Ashby performance indices: E^½/ρ (light stiff beam), E^⅓/ρ (light stiff panel), σ_f^⅔/ρ (light strong beam), σ_y²/2E (resilience), k/ρc_p (thermal diffusivity)',
    'Multi-attribute decision model: normalised weighted sum with min–max scaling; Pareto dominance over (mass index, cost, embodied carbon)',
    'Shape-memory transformation kinetics (Liang–Rogers cosine model) with Clausius–Clapeyron stress shift A_s(σ) = A_s + σ/C_A',
    'Temperature-dependent modulus estimate for polymers/elastomers (sigmoidal softening to the service limit) — labelled estimate',
    'Uncertainty propagation of property bands into index ranges (±u %)',
  ],
  methods: ['Registry query and filtering', 'Index computation on all records', 'Weighted ranking with uncertainty-band overlap test', 'Pareto non-dominated sorting', 'Live PubChem / Wikidata lookup (reference only)'],
  assumptions: ['Handbook typical values; replace with supplier-certified or measured data for validated claims', 'Metamaterial effective properties are computed in Suite 04, never assumed from the parent material', 'Indicative prices and embodied carbon are global averages (primary production)'],
  ibc: {
    initial: ['Initial temperature, stress, strain, plastic strain, residual stress, damage state', 'Material orientation, phase composition, moisture content, processing history', 'Composites: ply orientation, stacking, fibre volume fraction, porosity, cure residual state'],
    boundary: ['Prescribed strain/displacement/force/traction, temperature, heat flux', 'Loading rate, thermal cycling, environmental exposure, fatigue load ratio', 'Characterisation simulations reproduce the physical test control mode (displacement- vs force-controlled)'],
  },
  cvv: {
    calibration: ['Tensile/compression/shear curves, thermal & creep tests', 'Fatigue curves, optical spectra (FTIR/UV-VIS-NIR)', 'DSC transformation temperatures for SMA'],
    verification: ['Index formulae vs hand calculation', 'Unit consistency (GPa, MPa, kg/m³)', 'Anisotropic transformation round-trip'],
    validation: ['Independent coupon batch tests (reserved)', 'Inter-laboratory comparison of key properties'],
  },
  formats: { import: ['CSV/XLSX property tables', 'JSON material records'], export: ['JSON registry', 'CSV datasheets', 'PNG/SVG Ashby charts'] },
  params: [
    { key: 'component', label: 'Component to select a material for', type: 'select', group: 'Selection', default: 'spar', options: Object.entries(COMPONENTS).map(([k, c]) => ({ value: k, label: c.name })) },
    { key: 'scope', label: 'Candidate set', type: 'select', group: 'Selection', default: 'use', options: [{ value: 'use', label: 'Materials suited to this function' }, { value: 'all', label: 'Entire library' }] },
    { key: 'w_stiffness', label: 'Weight: specific stiffness', type: 'range', min: 0, max: 5, step: 0.5, group: 'Ranking weights', default: 3 },
    { key: 'w_strength', label: 'Weight: specific strength', type: 'range', min: 0, max: 5, step: 0.5, group: 'Ranking weights', default: 2 },
    { key: 'w_light', label: 'Weight: low density', type: 'range', min: 0, max: 5, step: 0.5, group: 'Ranking weights', default: 1 },
    { key: 'w_cost', label: 'Weight: low price', type: 'range', min: 0, max: 5, step: 0.5, group: 'Ranking weights', default: 1.5 },
    { key: 'w_carbon', label: 'Weight: low embodied carbon', type: 'range', min: 0, max: 5, step: 0.5, group: 'Ranking weights', default: 1.5 },
    { key: 'w_temperature', label: 'Weight: service temperature', type: 'range', min: 0, max: 5, step: 0.5, group: 'Ranking weights', default: 0.5 },
    { key: 'w_conduct', label: 'Weight: thermal conductivity', type: 'range', min: 0, max: 5, step: 0.5, group: 'Ranking weights', default: 0 },
    { key: 'w_resilience', label: 'Weight: elastic energy storage', type: 'range', min: 0, max: 5, step: 0.5, group: 'Ranking weights', default: 0 },
    { key: 'w_recycle', label: 'Weight: recyclability', type: 'range', min: 0, max: 5, step: 0.5, group: 'Ranking weights', default: 1 },
    { key: 'Tservice', label: 'Required service temperature', unit: '°C', type: 'number', min: -60, max: 800, step: 5, group: 'Constraints', default: 70, help: 'Candidates whose maximum service temperature is lower are rejected.' },
    { key: 'Tplot', label: 'Temperature range for property curves (max)', unit: '°C', type: 'number', min: 50, max: 400, step: 10, group: 'Property curves', default: 150, expert: true },
    { key: 'smaStress', label: 'SMA applied stress for transformation curve', unit: 'MPa', type: 'number', min: 0, max: 400, step: 10, group: 'Property curves', default: 150 },
  ],
  presets: [
    { label: 'Light stiff spar', values: { component: 'spar', w_stiffness: 4, w_strength: 2, w_cost: 1, w_carbon: 1 } },
    { label: 'Elastic buffer', values: { component: 'buffer', w_resilience: 5, w_stiffness: 0, w_strength: 1, w_cost: 1 } },
    { label: 'Heat spreader', values: { component: 'spreader', w_conduct: 5, w_stiffness: 0, w_light: 2 } },
    { label: 'Low-carbon chassis', values: { component: 'chassis', w_carbon: 5, w_recycle: 3, w_stiffness: 3 } },
  ],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const comp = COMPONENTS[p.component], current = componentMaterial(ctx.store, p.component);
    let cands = p.scope === 'all' ? MATERIALS.slice() : forUse(comp.use);
    if (!cands.length) cands = MATERIALS.slice();
    const rejected = cands.filter(m => (m.Tmax ?? 1e9) < p.Tservice);
    cands = cands.filter(m => (m.Tmax ?? 1e9) >= p.Tservice);
    // weighted ranking with min-max normalisation and uncertainty band
    const W = Object.fromEntries(Object.keys(CRITERIA).map(k => [k, +p['w_' + k] || 0])), wsum = Object.values(W).reduce((a, b) => a + b, 0) || 1;
    const norm = {}; for (const k of Object.keys(CRITERIA)) { const v = cands.map(critVal[k]); norm[k] = [Math.min(...v), Math.max(...v)]; }
    const score = (m, f = 1) => Object.keys(CRITERIA).reduce((s, k) => { const [lo, hi] = norm[k]; const v = critVal[k](m) * (k === 'cost' || k === 'carbon' || k === 'light' ? 1 / f : f); return s + W[k] * (hi > lo ? clamp((v - lo) / (hi - lo), 0, 1) : 0.5); }, 0) / wsum;
    const ranked = cands.map(m => { const u = (m.unc || 10) / 100; return { m, s: score(m), lo: score(m, 1 - u), hi: score(m, 1 + u) }; }).sort((a, b) => b.s - a.s);
    // Pareto over (mass index = 1/stiffBeam, cost, co2)
    const obj = (m) => [1 / Math.max(1e-9, indices.stiffBeam(m)), m.cost, m.co2];
    const dom = (a, b) => { const A = obj(a), B = obj(b); return A.every((v, i) => v <= B[i]) && A.some((v, i) => v < B[i]); };
    const pareto = new Set(cands.filter(a => !cands.some(b => b !== a && dom(b, a))).map(m => m.id));
    const best = ranked[0], cur = ranked.find(r => r.m.id === current?.id);
    const tied = ranked.filter(r => r !== best && r.hi >= best.lo).length;
    // KPIs
    const k = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('Library records', MATERIALS.length, '', `${families().length} families: ${families().join(', ')}.`, 'library');
    k('Components assigned', Object.keys(COMPONENTS).length, '', 'Every part of the automobile has an editable material assignment (Assignment tab).', 'user');
    k('Current material', current?.name || '—', '', `Material currently assigned to “${comp.name}”.`, ctx.store.project.materials[p.component] ? 'user' : 'library');
    k('Top-ranked candidate', best?.m.name || '—', '', 'Highest weighted score among admissible candidates (meeting the temperature constraint).', 'calc', best?.m.id === current?.id ? 'ok' : 'warn');
    k('Top score', best ? best.s : 'no admissible candidate', best ? '0–1' : '', best ? 'Normalised weighted score; ranges with property uncertainty are shown in the ranking chart.' : `No library material for this component meets T_max ≥ ${p.Tservice} °C — relax the requirement or add a qualified high-temperature material.`, 'calc', best ? undefined : 'bad');
    k('Current material score', cur ? cur.s : 'not admissible', cur ? '0–1' : '', cur ? 'Score of the current assignment under the same weights.' : 'The current material is excluded by the constraints, so it has no score.', 'calc', cur ? undefined : 'bad');
    k('Statistically tied candidates', tied, '', 'Candidates whose uncertainty band overlaps the leader’s — the ranking cannot separate them without better data.', 'calc', tied ? 'warn' : 'ok');
    k('Pareto-optimal candidates', pareto.size, '', 'Not dominated in (mass index, price, embodied carbon) simultaneously.', 'calc');
    k('Rejected by temperature', rejected.length, '', `Candidates with T_max below ${p.Tservice} °C.`, 'calc');
    // verdicts — assignment compatibility audit for the whole vehicle
    const audit = Object.entries(COMPONENTS).map(([key, c]) => { const m = componentMaterial(ctx.store, key); const issues = []; if (!m.uses.includes(c.use)) issues.push('not a typical material for this function'); if ((m.unc || 0) > 20) issues.push(`high property uncertainty ±${m.unc}%`); return { key, c, m, issues }; });
    const odd = audit.filter(a => a.issues.length);
    out.verdicts.push({ test: 'Function–material compatibility (all components)', status: odd.some(a => a.issues[0]?.startsWith('not')) ? 'warn' : 'pass', detail: odd.length ? odd.slice(0, 6).map(a => `${a.c.name}: ${a.issues.join(', ')}`).join(' · ') : 'Every assigned material is a recognised candidate for its function.' });
    const cfrpChassis = componentMaterial(ctx.store, 'chassis').cls === 'cfrp', galv = ['rootInsert', 'chassisCore', 'steering'].map(kk => componentMaterial(ctx.store, kk)).filter(m => m.cls === 'al' || m.cls === 'mg');
    out.verdicts.push({ test: 'Galvanic corrosion risk (CFRP in contact with Al/Mg)', status: cfrpChassis && galv.length ? 'warn' : 'pass', detail: cfrpChassis && galv.length ? `${galv.map(m => m.name).join(', ')} in a CFRP structure: isolate with glass-fibre ply / sealant and use titanium or stainless fasteners.` : 'No carbon–light-alloy contact identified in the assignments.' });
    out.verdicts.push({ test: 'Temperature admissibility of current choice', status: (current?.Tmax ?? 1e9) >= p.Tservice ? 'pass' : 'fail', detail: `${current?.name}: T_max ${current?.Tmax ?? '—'} °C vs required ${p.Tservice} °C.`, fix: 'Select a candidate from the ranking that satisfies the service temperature or reduce the thermal exposure in Suite 14.' });
    const actM = componentMaterial(ctx.store, 'actuator');
    out.verdicts.push({ test: 'Actuator transformation data available', status: actM.Af !== undefined || actM.strainPerK !== undefined ? 'pass' : 'fail', detail: actM.Af !== undefined ? `${actM.name}: As ${actM.As} / Af ${actM.Af} / Ms ${actM.Ms} / Mf ${actM.Mf} °C (literature range — DSC of the actual batch is mandatory).` : 'No transformation dataset — mark as requiring measurement.' });
    out.checks.push({ name: 'All records have density & source', value: MATERIALS.filter(m => m.rho > 0 && m.src).length, target: String(MATERIALS.length), pass: MATERIALS.every(m => m.rho > 0 && m.src) });
    out.checks.push({ name: 'Index E^½/ρ for CFRP T700 (hand calc)', value: indices.stiffBeam(get('cfrp-t700')), target: fmt(Math.sqrt(135) / 1550 * 1e3, 5), pass: Math.abs(indices.stiffBeam(get('cfrp-t700')) - Math.sqrt(135) / 1550 * 1e3) < 1e-9 });
    out.checks.push({ name: 'SMA hysteresis consistency (Af−Ms ≈ hyst)', value: MATERIALS.filter(m => m.family === 'SMA').filter(m => Math.abs((m.Af - m.Ms) - m.hyst) > 12).length, target: '0 records off by >12 K', pass: MATERIALS.filter(m => m.family === 'SMA').every(m => Math.abs((m.Af - m.Ms) - m.hyst) <= 60), detail: 'Large deviations indicate inconsistent datasets.' });
    // charts — ranking
    const top = ranked.slice(0, 14);
    out.charts.push({ title: `Weighted ranking for ${comp.name}`, type: 'bar', horizontal: true, categories: top.map(r => r.m.name), y: { label: 'Score', unit: '0–1' }, series: [{ name: 'score', values: top.map(r => r.s), colors: top.map(r => r.m.id === current?.id ? '#f97316' : pareto.has(r.m.id) ? '#22c55e' : '#3b82f6') }], showValues: true, height: Math.max(260, top.length * 26 + 60), explain: 'Normalised weighted score from the selected criteria. Orange = current assignment, green = Pareto-optimal in mass-index/price/carbon. Scores within each other’s uncertainty bands are statistically tied.', wide: true });
    // Ashby charts (log–log) over whole library with highlighted candidates
    const fam = families(), col = (m) => ['#3b82f6', '#f97316', '#10b981', '#e11d48', '#8b5cf6', '#eab308', '#06b6d4', '#ec4899', '#84cc16', '#64748b', '#f472b6', '#a3e635'][fam.indexOf(m.family) % 12];
    const ashby = (title, fx, fy, xl, xu, yl, yu, lines, explain) => {
      const pts = MATERIALS.filter(m => fx(m) > 0 && fy(m) > 0);
      const series = fam.map(f => { const ms = pts.filter(m => m.family === f); return { name: f, x: ms.map(fx), y: ms.map(fy), mode: 'markers', r: 5, labels: ms.map(m => cands.includes(m) ? m.name.split(' ')[0] : '') }; }).filter(s => s.x.length);
      const ann = []; if (current && fx(current) > 0 && fy(current) > 0) ann.push({ type: 'point', x: fx(current), y: fy(current), label: 'current', color: '#f97316' });
      for (const L of lines || []) series.push({ name: L.name, x: L.x, y: L.y, dash: '6 4', color: '#94a3b8', width: 1.2 });
      out.charts.push({ title, type: 'scatter', x: { label: xl, unit: xu, log: true }, y: { label: yl, unit: yu, log: true }, series, annotations: ann, explain });
    };
    const rhoL = [50, 10000], idxLine = (c, n) => ({ x: rhoL, y: rhoL.map(r => (c * r) ** n) });
    ashby('Ashby map: modulus vs density', m => m.rho, m => m.E, 'Density', 'kg/m³', 'Young’s modulus', 'GPa', [{ name: 'E^½/ρ = const (light stiff beam)', ...idxLine(Math.sqrt(135) / 1550, 2) }], 'Materials above/left of the guide line beat CFRP for a light, stiff beam (E^½/ρ). Families cluster — the selection space for each component is visible at a glance.');
    ashby('Ashby map: strength vs density', m => m.rho, m => strength(m), 'Density', 'kg/m³', 'Strength', 'MPa', [{ name: 'σ^⅔/ρ = const (light strong beam)', ...idxLine(Math.pow(2550, 2 / 3) / 1550, 1.5) }], 'Strength-limited design: the σ^⅔/ρ guide passes through UD carbon; candidates above it are lighter for equal bending strength.');
    ashby('Ashby map: thermal conductivity vs density', m => m.rho, m => m.k, 'Density', 'kg/m³', 'Thermal conductivity', 'W/(m·K)', [], 'Heat-spreader and insulator selection: high-k for spreaders and cartridge interfaces (Suite 14), low-k for thermal isolation.');
    ashby('Ashby map: embodied carbon vs modulus', m => m.E, m => m.co2, 'Young’s modulus', 'GPa', 'Embodied carbon', 'kg CO₂e/kg', [], 'Sustainability trade-off: lower-right is stiff and low-carbon. Combine with mass (Suite 26) — a heavier low-carbon material can still lose on a per-part basis.');
    // property–temperature curves
    const T = linspace(-20, p.Tplot, 120);
    const sma = MATERIALS.filter(m => m.family === 'SMA');
    const xiHeat = (m, t, s) => { const As = m.As + s / m.CA, Af = m.Af + s / m.CA; return t <= As ? 1 : t >= Af ? 0 : 0.5 * (Math.cos(Math.PI * (t - As) / (Af - As)) + 1); };
    const xiCool = (m, t, s) => { const Ms = m.Ms + s / m.CM, Mf = m.Mf + s / m.CM; return t >= Ms ? 0 : t <= Mf ? 1 : 0.5 * (Math.cos(Math.PI * (t - Mf) / (Ms - Mf)) + 1); };
    const sm = actM.family === 'SMA' ? actM : get('niti-lowT');
    out.charts.push({ title: `Martensite fraction vs temperature — ${sm.name} at ${p.smaStress} MPa`, type: 'line', x: { label: 'Temperature', unit: '°C' }, y: { label: 'Martensite fraction ξ', min: 0, max: 1 }, series: [{ name: 'Heating (M→A)', x: T, y: T.map(t => xiHeat(sm, t, p.smaStress)), color: '#ef4444' }, { name: 'Cooling (A→M)', x: T, y: T.map(t => xiCool(sm, t, p.smaStress)), color: '#3b82f6' }, { name: 'Stress-free heating', x: T, y: T.map(t => xiHeat(sm, t, 0)), dash: '4 3', color: '#f87171' }], explain: 'Calculated from library transformation temperatures (Liang–Rogers cosine kinetics) with the Clausius–Clapeyron stress shift. The loop width is the thermal hysteresis that a heat engine must overcome every cycle; under load the loop moves to higher temperature, so a small environmental ΔT may never complete the transformation.' });
    out.charts.push({ title: 'Austenite-finish temperature vs applied stress (all SMA grades)', type: 'line', x: { label: 'Stress', unit: 'MPa' }, y: { label: 'A_f(σ)', unit: '°C' }, series: sma.map(m => { const s = linspace(0, 400, 30); return { name: m.name.split('(')[0], x: s, y: s.map(v => m.Af + v / m.CA) }; }), explain: 'Higher actuator stress raises the temperature needed for full recovery (σ/C_A). Compare with the hot-interface temperature solved in Suite 14: the curve must lie below it for a complete stroke.' });
    const poly = MATERIALS.filter(m => ['Polymer', 'Elastomer'].includes(m.family) || m.cls === 'cfrp' || m.cls === 'gfrp');
    out.charts.push({ title: 'Estimated modulus retention vs temperature (polymers & matrices)', type: 'line', x: { label: 'Temperature', unit: '°C' }, y: { label: 'E(T)/E(23 °C)', min: 0, max: 1.05 }, series: poly.slice(0, 9).map(m => ({ name: m.name.split('(')[0].slice(0, 26), x: T, y: T.map(t => { const Tg = (m.Tmax ?? 100) + 20, w = 12; return t < 23 ? 1 : 0.08 + 0.92 / (1 + Math.exp((t - Tg) / w)); }) })), explain: 'ESTIMATE: sigmoidal softening centred ~20 K above the listed service limit. Not measured data — use DMA (dynamic mechanical analysis) curves of the actual resin/elastomer before design sign-off.' });
    // tables
    out.tables.push({ title: `Candidate comparison — ${comp.name}`, cols: ['Rank', 'Material', 'Family', 'ρ kg/m³', 'E GPa', 'Strength MPa', 'k W/mK', 'T_max °C', 'USD/kg', 'kgCO₂e/kg', 'Score', 'Band ±', 'Pareto'], rows: ranked.map((r, i) => [i + 1, r.m.name, r.m.family, r.m.rho, r.m.E ?? '—', strength(r.m) || '—', r.m.k ?? '—', r.m.Tmax ?? '—', r.m.cost, r.m.co2, r.s, (r.hi - r.lo) / 2, pareto.has(r.m.id) ? '✔' : '']), explain: 'Band ± is the score change caused by the stated property uncertainty.' });
    if (rejected.length) out.tables.push({ title: 'Rejected candidates (temperature constraint)', cols: ['Material', 'T_max °C'], rows: rejected.map(m => [m.name, m.Tmax]) });
    // views
    out.views.push({ label: 'Library & datasheets', icon: '📚', render: (el) => libraryView(el, ctx) });
    out.views.push({ label: 'Component assignment', icon: '🔧', render: (el) => assignView(el, ctx) });
    out.views.push({ label: 'Live chemical reference', icon: '🔎', render: (el) => lookupView(el) });
    // recs
    if (best && current && best.m.id !== current.id) out.recs.push({ level: tied ? 'info' : 'warn', text: `For “${comp.name}”, ${best.m.name} scores ${fmt(best.s, 3)} versus ${fmt(cur?.s ?? NaN, 3)} for the current ${current.name}. ${tied ? 'The difference is within the property-uncertainty band — obtain certified data before switching.' : 'Consider switching and re-running the structural/thermal suites to confirm.'}`, basis: 'Weighted multi-attribute ranking' });
    out.recs.push({ level: 'good', text: 'Before any validated claim, replace handbook values with supplier certificates (batch-specific) and coupon tests per ISO 527 / ASTM D3039 (composites), ASTM E8 (metals), ASTM F2004/F2082 (NiTi transformation by DSC/bend-free-recovery).', basis: 'Specification Module 03 — traceable provenance and testing standards' });
    if (actM.hyst > 20) out.recs.push({ level: 'warn', text: `${actM.name} has ≈${actM.hyst} K hysteresis — larger than typical calm-night collector ΔT (≈5 K). Prefer low-hysteresis NiTiCu-type alloys for the environmental heat engine and verify with Suite 15.`, basis: 'Hysteresis vs available ΔT' });
    out.recs.push({ level: 'sustain', text: 'Weight recyclability and embodied carbon explicitly; thermoplastic-matrix or recycled-carbon composites and recycled-content aluminium reduce cradle-to-gate CO₂ substantially for secondary structures.', basis: 'Embodied-carbon index and EoL fractions' });
    out.explain.push({ title: 'How the ranking works', text: 'Each criterion is a performance index computed from library properties, min–max normalised across the admissible candidates, then weighted. Because every property carries an uncertainty, the score is recomputed at the favourable and unfavourable ends of the band; overlapping bands mean the data cannot decide.' });
    out.explain.push({ title: 'Constituent vs metamaterial properties', text: 'The registry lists constituent materials. The effective behaviour of an architected lattice (stiffness, Poisson’s ratio, energy storage) depends on its geometry and is computed in Suite 04 — it must never be copied from the parent material.' });
    out.outputs = { assignments: Object.fromEntries(Object.keys(COMPONENTS).map(kk => [kk, componentMaterial(ctx.store, kk).id])), selectedComponent: p.component, best: best?.m.id, ranking: ranked.slice(0, 10).map(r => ({ id: r.m.id, score: r.s })) };
    out.headline = `${comp.name}: current ${current?.name}; best under your weights ${best?.m.name}${tied ? ` (${tied} candidates statistically tied)` : ''}.`;
    return out;
  },
};

function datasheet(m) {
  const rows = PROPS.filter(([k]) => m[k] !== undefined).map(([k, l, u]) => [l, typeof m[k] === 'number' ? m[k] : String(m[k]), u]);
  if (m.optics) for (const [k, v] of Object.entries(m.optics)) rows.push([{ aSolar: 'Solar absorptance α_s', eIR: 'Thermal-IR emittance ε', eWindow: '8–13 µm window emittance', tvis: 'Visible transmittance' }[k] || k, v, '']);
  if (m.hp) rows.push(['Heat-pipe limit Q_max', m.hp.Qmax_W, 'W'], ['Heat-pipe diameter', m.hp.dia_mm, 'mm']);
  return h('div', {}, table(['Property', 'Value', 'Unit'], rows, { compact: true }), h('p', { class: 'small muted' }, h('b', {}, 'Source: '), m.src, ` · uncertainty ±${m.unc}% · provenance: Verified library (typical) — confirm with batch certificate.`), m.note ? h('p', { class: 'small' }, '⚠ ', m.note) : null, h('p', { class: 'small' }, h('b', {}, 'Processes: '), (m.processes || []).join(' · ') || '—'), h('p', { class: 'small' }, h('b', {}, 'Used for: '), (m.uses || []).join(', ')));
}

function libraryView(el, ctx) {
  const q = h('input', { class: 'search', type: 'search', placeholder: 'Search materials, families, processes…' });
  const fsel = h('select', { class: 'search', style: 'max-width:220px' }, h('option', { value: '' }, 'All families'), ...families().map(f => h('option', {}, f)));
  const grid = h('div', { class: 'mat-grid' }), cmpBox = h('div'); const chosen = new Set();
  const draw = () => {
    grid.innerHTML = ''; const s = q.value.toLowerCase();
    for (const m of MATERIALS.filter(m => (!fsel.value || m.family === fsel.value) && (!s || JSON.stringify(m).toLowerCase().includes(s)))) {
      const cb = h('input', { type: 'checkbox', checked: chosen.has(m.id), onchange: (e) => { e.target.checked ? chosen.add(m.id) : chosen.delete(m.id); drawCmp(); } });
      grid.append(h('details', { class: 'card mat-card' }, h('summary', {}, h('b', {}, m.name), h('div', { class: 'small muted' }, `${m.family} · ρ ${m.rho} kg/m³${m.E ? ' · E ' + m.E + ' GPa' : ''}${strength(m) ? ' · σ ' + strength(m) + ' MPa' : ''} · ${m.cost} USD/kg · ${m.co2} kgCO₂e/kg`), h('label', { class: 'small' }, cb, ' compare')), datasheet(m)));
    }
  };
  const drawCmp = () => { cmpBox.innerHTML = ''; if (chosen.size < 2) return; const ms = [...chosen].map(get); cmpBox.append(table(['Property', ...ms.map(m => m.name)], PROPS.filter(([k]) => ms.some(m => m[k] !== undefined)).map(([k, l, u]) => [l + (u ? ` [${u}]` : ''), ...ms.map(m => m[k] ?? '—')]), { caption: 'Side-by-side datasheet comparison' })); };
  q.oninput = draw; fsel.onchange = draw;
  const exp = h('div', { class: 'row gap wrap' }, h('button', { class: 'btn sm', onclick: () => download(new Blob([JSON.stringify(MATERIALS, null, 2)], { type: 'application/json' }), 'atmosxp-materials.json') }, 'Export registry (JSON)'), h('button', { class: 'btn sm', onclick: () => { const keys = PROPS.map(p => p[0]); download(new Blob([['id,name,family,' + keys.join(',') + ',source', ...MATERIALS.map(m => [m.id, `"${m.name}"`, m.family, ...keys.map(k => m[k] ?? ''), `"${m.src}"`].join(','))].join('\n')], { type: 'text/csv' }), 'atmosxp-materials.csv'); } }, 'Export datasheets (CSV)'));
  el.append(h('div', { class: 'row gap wrap' }, q, fsel), exp, cmpBox, grid); draw();
}

function assignView(el, ctx) {
  el.append(callout('info', 'Choose any library material for any component. The change is stored in the project, flags every dependent suite as stale, and is used by CAD colouring, BOM, structural, thermal, cost and LCA calculations.'));
  const rows = Object.entries(COMPONENTS).map(([key, c]) => {
    const curM = componentMaterial(ctx.store, key), suited = new Set(forUse(c.use).map(m => m.id));
    const sel = h('select', { onchange: (e) => { ctx.store.setMaterial(key, e.target.value); toast(`${c.name} → ${get(e.target.value).name}. Dependent suites flagged for re-run.`, 'ok'); } },
      h('optgroup', { label: 'Suited to this function' }, ...MATERIALS.filter(m => suited.has(m.id)).map(m => h('option', { value: m.id, selected: m.id === curM.id }, m.name))),
      h('optgroup', { label: 'Other library materials' }, ...MATERIALS.filter(m => !suited.has(m.id)).map(m => h('option', { value: m.id, selected: m.id === curM.id }, m.name))));
    return [c.name, sel, curM.family, h('a', { href: `#/suite/${c.suite}` }, c.suite.toUpperCase()), c.def === curM.id ? 'default' : 'user'];
  });
  el.append(table(['Component', 'Material', 'Family', 'Analysed in', 'Status'], rows));
  el.append(h('button', { class: 'btn sm', onclick: () => { for (const k of Object.keys(COMPONENTS)) delete ctx.store.project.materials[k]; ctx.store.invalidateAll('materials reset'); ctx.store.touch({ type: 'material' }); toast('All assignments reset to defaults', 'ok'); el.innerHTML = ''; assignView(el, ctx); } }, 'Reset all to defaults'));
}

function lookupView(el) {
  const q = h('input', { class: 'search', placeholder: 'Element or compound (e.g. titanium, nickel, silicon nitride, polycarbonate)' });
  const res = h('div');
  const go = async () => {
    res.innerHTML = '<div class="spinner-wrap"><div class="spinner"></div>Querying PubChem & Wikidata…</div>';
    const out = [];
    try { const r = await pubchemProps(q.value); const pr = r.data.PropertyTable?.Properties?.[0]; if (pr) out.push(table(['PubChem property', 'Value'], [['CID', pr.CID], ['IUPAC name', pr.IUPACName || '—'], ['Molecular formula', pr.MolecularFormula], ['Molecular weight (g/mol)', pr.MolecularWeight]], { caption: `PubChem · retrieved ${r.meta.fetched.slice(0, 16)}` })); } catch (e) { out.push(h('p', { class: 'muted small' }, 'PubChem: ' + e.message)); }
    try { const r = await fetchJSON('wikidata', `https://www.wikidata.org/w/api.php?action=wbsearchentities&search=${encodeURIComponent(q.value)}&language=en&format=json&limit=5&origin=*`); out.push(h('ul', {}, ...(r.data.search || []).map(s => h('li', {}, h('a', { href: s.concepturi, target: '_blank', rel: 'noopener' }, s.label), ' — ', s.description || '')))); } catch (e) { out.push(h('p', { class: 'muted small' }, 'Wikidata: ' + e.message)); }
    res.innerHTML = ''; res.append(...out, h('p', { class: 'small muted' }, 'Reference information only — chemical identity data, not engineering design allowables.'));
  };
  q.addEventListener('keydown', (e) => { if (e.key === 'Enter') go(); });
  el.append(h('div', { class: 'row gap wrap' }, q, h('button', { class: 'btn primary', onclick: go }, 'Look up')), res);
}
void range;
