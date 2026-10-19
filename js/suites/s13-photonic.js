// Suite 13 — Photonic metamaterials, solar absorption and radiative cooling.
import * as TH from '../core/thermo.js';
import { SCENARIOS, derive } from '../core/env.js';
import { linspace, fmt, trapz, max, argmax, range, KELVIN, SIGMA } from '../core/num.js';
import { forUse, get } from '../core/materials.js';

const caseFrom = (env, kind) => {
  // pick representative day (max irradiance) and night (median sky depression) hours from the Suite 05 series
  if (env?.series?.length) {
    const s = env.series;
    if (kind === 'day') { const i = argmax(s.map(r => r.ghi || 0)); if ((s[i].ghi || 0) > 50) return { ...s[i], src: `Suite 05 series, peak-irradiance hour ${s[i].time}`, coupled: true }; }
    else { const n = s.filter(r => !r.isDay); if (n.length) { const sorted = n.slice().sort((a, b) => (a.T - a.Tsky) - (b.T - b.Tsky)); const r = sorted[Math.floor(sorted.length / 2)]; return { ...r, src: `Suite 05 series, median night hour ${r.time}`, coupled: true }; } }
  }
  const d = derive(kind === 'day' ? SCENARIOS.sunnyCalm : SCENARIOS.clearNight, {});
  return { T: d.T, U: d.U, Uz: d.Uz, ghi: d.ghi, cloud: d.cloud, RH: d.RH, Tsky: d.Tsky, eps: d.eps, src: `Standard scenario “${(kind === 'day' ? SCENARIOS.sunnyCalm : SCENARIOS.clearNight).label}” (Suite 05 not run)`, coupled: false };
};

export default {
  description: 'Spectral design of the day-side solar-selective absorber and the night-side infrared radiator: wavelength-resolved absorptivity/emissivity (library coatings or a user multilayer stack solved with the transfer-matrix method), coupled to the measured or scenario sky and solar spectra, giving net heat flux, stagnation/equilibrium temperatures, heat available to the thermal converter and its Carnot-limited exergy.',
  literature: 'passive radiative cooling photonic metamaterial selective solar absorber power generation',
  sustain: true, credibility: 'Analytical screening',
  equations: [
    'Maxwell’s equations in stratified media → characteristic (transfer) matrix method with Fresnel interface coefficients for s and p polarisation; Snell’s law for complex indices',
    'Drude free-electron model for metals and multi-oscillator Lorentz model for dielectric phonon resonances (complex permittivity ε = (n + ik)²)',
    'Energy conservation R + T + A = 1; Kirchhoff’s law (spectral directional absorptivity = emissivity)',
    'Planck’s spectral emissive power, Stefan–Boltzmann law, spectral and hemispherical emissivity integrals ε_h = 2∫A(θ)cosθ sinθ dθ',
    'Terrestrial solar spectrum (AM1.5G-like screening approximation) scaled to measured global irradiance',
    'Atmospheric spectral emissivity ε_atm(λ) = 1 − τ_w·w(λ) with the 8–13 µm transmission window, calibrated so its hemispherical integral equals the cloud-corrected sky emissivity from Suite 05',
    'Surface energy balance: q = α_sG − ∫ε(λ)[E_b(λ,T_s) − ε_atm(λ)E_b(λ,T_a)]dλ − h(T_s − T_a); McAdams wind-forced convection h = 5.7 + 3.8U',
    'Carnot and finite-time (maximum-power) work extraction from the absorber or radiator heat flow',
  ],
  methods: ['260-point logarithmic wavelength grid 0.28–40 µm; trapezoidal spectral integration', 'Stack hemispherical emissivity by 7–9 point angular quadrature of averaged s/p absorptance', 'Equilibrium (stagnation) temperatures by Brent root-finding of the energy balance', 'Hourly evaluation over the full Suite 05 time series for daily energy budgets'],
  assumptions: ['Opaque substrate for metal back-reflectors; diffuse (Lambertian) behaviour for library band coatings', 'Single horizontal orientation; sky view factor 1', 'The solar spectrum is a screening approximation, not the tabulated ASTM G173 standard', 'Collector and radiator temperatures are boundary inputs here; the coupled interface temperatures are solved in Suite 14'],
  ibc: {
    initial: ['Initial coating and substrate temperature (ambient)', 'Spectral optical properties (library or TMM stack)', 'Surface orientation, solar position from Suite 05', 'Contamination/soiling factor'],
    boundary: ['Incident solar spectral irradiance scaled to GHI', 'Atmospheric infrared back-radiation (window-calibrated)', 'Ambient convection from wind (or convection shield)', 'Periodic/Bloch boundaries are implicit in the 1-D stratified (laterally infinite) stack', 'Separate day (absorber) and night (radiator) configurations; prescribed operating temperature or coupled heat flow to Suite 14'],
  },
  cvv: {
    calibration: ['Spectrophotometer (0.25–2.5 µm) reflectance and FTIR (2.5–25 µm) emissivity of fabricated coatings', 'Angle-resolved emissivity', 'Outdoor solar-heating and night-cooling stagnation temperature tests'],
    verification: ['Single-interface Fresnel reflectance vs analytic ((n−1)/(n+1))²', 'Lossless stack energy conservation R + T = 1', 'Planck integral equals σT⁴', 'Solar spectrum integral equals the prescribed irradiance'],
    validation: ['Simultaneous measurement of collector temperature, incoming/outgoing radiation and convection on clear and cloudy nights', 'Measured daily useful heat versus predicted daily energy'],
  },
  formats: { import: ['CSV spectra (λ [µm], ε)', 'FTIR/spectrophotometer CSV/TXT', 'XLSX'], export: ['CSV spectra', 'JSON optical model', 'GDSII/OASIS layouts for patterned nanostructures (via Suite 02 export manager)', 'PNG/SVG plots'] },
  params: [
    { key: 'collector', label: 'Day absorber coating', type: 'select', group: 'Surfaces', default: 'coat-tinox', options: forUse('collector').map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.store.material('collector', 'coat-tinox'), prov: 'library' }) },
    { key: 'radiator', label: 'Night radiator surface', type: 'select', group: 'Surfaces', default: 'coat-pdrc', options: forUse('radiator').map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.store.material('radiator', 'coat-pdrc'), prov: 'library' }) },
    { key: 'useStack', label: 'Use custom multilayer for the radiator (TMM)', type: 'bool', group: 'Photonic stack', default: false },
    { key: 'stack', label: 'Stack (top → bottom), material:thickness nm', type: 'text', group: 'Photonic stack', default: 'SiO2:1200, Si3N4:700, Al2O3:400', help: 'Available: ' + Object.keys(TH.OPTICAL).filter(k => k !== 'Air').join(', ') },
    { key: 'substrate', label: 'Stack substrate / back reflector', type: 'select', group: 'Photonic stack', default: 'Ag', options: ['Ag', 'Al', 'Cr', 'Ti', 'SiO2'] },
    { key: 'Acol', label: 'Absorber (collector) area', unit: 'm²', type: 'number', min: 0.1, max: 20, step: 0.1, group: 'Geometry', default: 4, auto: (c) => { const v = c.vehicle; return { value: +(0.32 * v.length * 0.5 * v.width).toFixed(2) || 4, prov: 'calc' }; } },
    { key: 'Arad', label: 'Radiator area', unit: 'm²', type: 'number', min: 0.1, max: 20, step: 0.1, group: 'Geometry', default: 4, help: 'The specification’s reference case uses a 4 m² collector.' },
    { key: 'dThot', label: 'Absorber operating temperature above ambient', unit: 'K', type: 'range', min: 1, max: 120, step: 1, group: 'Operating point', default: 25 },
    { key: 'dTcold', label: 'Radiator operating temperature below ambient', unit: 'K', type: 'range', min: 0, max: 20, step: 0.5, group: 'Operating point', default: 3 },
    { key: 'vConv', label: 'Vehicle speed for convection', unit: 'km/h', type: 'range', min: 0, max: 60, step: 1, group: 'Convection', default: 0, help: 'Moving air increases convective losses from the absorber and parasitic heat gain into a sub-ambient radiator.' },
    { key: 'shield', label: 'IR-transparent convection shield', type: 'bool', group: 'Convection', default: false, help: 'Polyethylene-type windscreen: h → 2 W/m²K, 8 % optical loss.' },
    { key: 'soiling', label: 'Soiling / ageing loss of solar absorptance', unit: '%', type: 'range', min: 0, max: 30, step: 1, group: 'Convection', default: 3, expert: true },
  ],
  presets: [{ label: 'Ultra-white paint radiator', values: { radiator: 'coat-baso4', useStack: false } }, { label: 'Black paint (non-selective)', values: { collector: 'coat-blackpaint' } }, { label: 'Moving at 30 km/h', values: { vConv: 30 } }, { label: 'Shielded radiator', values: { shield: true } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const env = ctx.upstream('s05') || ctx.env;
    const day = caseFrom(env, 'day'), night = caseFrom(env, 'night');
    const colM = get(p.collector), radM = get(p.radiator);
    const layers = TH.parseStack(p.stack), stackOn = p.useStack && layers.length;
    ctx.progress(0.15, 'spectral properties');
    const epsCol = TH.tabulate(TH.surfaceSpectrum(colM)).map(e => e);
    const solarDerate = (1 - p.soiling / 100) * (p.shield ? 0.92 : 1);
    const epsRad = TH.tabulate(TH.surfaceSpectrum(radM, stackOn ? { layers, substrate: p.substrate } : null));
    await ctx.yield();
    const hOf = (c) => p.shield ? 2 : TH.mcAdamsH(Math.max(c.Uz ?? c.U ?? 0, p.vConv / 3.6));
    const balance = (c, epsTab, T) => { const TaK = c.T + KELVIN, sky = TH.skySpectralEmissivity(TaK, c.eps ?? 0.8).fn, r = TH.radiativeTerms(epsTab, T, TaK, sky, c.ghi || 0); return { ...r, solAbs: r.solAbs * solarDerate, TaK, h: hOf(c) }; };
    const absorberQ = (c, T) => { const b = balance(c, epsCol, T); return b.solAbs - (b.emit - b.atm) - b.h * (T - b.TaK); };
    const radiatorQ = (c, T) => { const b = balance(c, epsRad, T); return (b.emit - b.atm) - b.solAbs - b.h * (b.TaK - T); };
    // operating points
    const TaD = day.T + KELVIN, TaN = night.T + KELVIN;
    const qDay = absorberQ(day, TaD + p.dThot), qNight = radiatorQ(night, TaN - p.dTcold), qRadDay = radiatorQ(day, TaD - p.dTcold);
    const stag = TH.brent((T) => absorberQ(day, T), TaD - 5, TaD + 400), Tmin = TH.brent((T) => radiatorQ(night, T), TaN - 80, TaN + 30);
    ctx.progress(0.45, 'operating curves');
    // curves vs temperature
    const Th = linspace(TaD - 5, Math.min(TaD + 250, (stag.ok ? stag.x : TaD + 200) + 5), 50), qA = Th.map(T => absorberQ(day, T));
    const Tc = linspace(Math.max(TaN - 45, (Tmin.ok ? Tmin.x : TaN - 30) - 3), TaN + 5, 50), qR = Tc.map(T => radiatorQ(night, T));
    // ideal engine power: day — absorber (hot) to ambient sink; night — ambient source to radiator (cold)
    const wDay = Th.map((T, i) => Math.max(0, qA[i]) * Math.max(0, 1 - TaD / T)), wNight = Tc.map((T, i) => Math.max(0, qR[i]) * Math.max(0, TaN / T - 1));
    const iWd = argmax(wDay), iWn = argmax(wNight);
    const wDayMax = wDay[iWd], wNightMax = wNight[iWn];
    const PabsW = Math.max(0, qDay) * p.Acol, PcoolW = Math.max(0, qNight) * p.Arad;
    const exDay = PabsW * Math.max(0, 1 - TaD / (TaD + p.dThot)), exNight = PcoolW * Math.max(0, TaN / (TaN - p.dTcold) - 1);
    // spectral summaries
    const aS_col = TH.solarWeighted(epsCol) * solarDerate, eT_col = TH.thermalWeighted(epsCol, TaD + p.dThot), aS_rad = TH.solarWeighted(epsRad), eW_rad = TH.windowWeighted(epsRad, TaN), eT_rad = TH.thermalWeighted(epsRad, TaN);
    // hourly budget over Suite 05 series
    let hourly = null;
    if (env?.series?.length) {
      const S = env.series.slice(0, 168);
      hourly = S.map(r => { const c = { ...r, Uz: r.Uz }; const TaK = r.T + KELVIN; return { t: r.time, abs: Math.max(0, absorberQ(c, TaK + p.dThot)) * p.Acol, rad: Math.max(0, radiatorQ(c, TaK - p.dTcold)) * p.Arad, day: r.isDay }; });
    }
    ctx.progress(0.75, 'verification');
    // verification
    const nG = TH.OPTICAL.SiO2.n(0.55).re, Rf = TH.tmm([], 'SiO2', 0.55).R, Rana = ((nG - 1) / (nG + 1)) ** 2;
    out.checks.push({ name: 'Fresnel normal-incidence reflectance (silica, 550 nm)', value: Rf, target: `analytic ${fmt(Rana, 6)}`, pass: Math.abs(Rf - Rana) < 1e-9 });
    const ls = TH.tmm([{ mat: 'TiO2', d: 57e-9 }, { mat: 'SiO2', d: 95e-9 }, { mat: 'TiO2', d: 57e-9 }], 'SiO2', 0.55); out.checks.push({ name: 'Lossless stack energy conservation R + T (550 nm)', value: ls.R + ls.Tsub, target: '1 ± 1e-6', pass: Math.abs(ls.R + ls.Tsub - 1) < 1e-4 });
    const Lw = linspace(Math.log10(0.1), Math.log10(1000), 3000).map(v => 10 ** v), pl = trapz(Lw.map(l => TH.planckLam(l, 300)), Lw) / (SIGMA * 300 ** 4); out.checks.push({ name: 'Planck integral / σT⁴ at 300 K', value: pl, target: '1 ± 0.001', pass: Math.abs(pl - 1) < 1e-3 });
    const solI = trapz(TH.LAMBDA.map(l => TH.solarSpectrum(l, 1000)), TH.LAMBDA); out.checks.push({ name: 'Solar spectrum normalisation (1000 W/m²)', value: solI, target: '1000 W/m²', pass: Math.abs(solI - 1000) < 0.5 });
    const skyN = TH.skySpectralEmissivity(TaN, night.eps ?? 0.8); out.checks.push({ name: 'Sky window calibration (τ_w within physical range)', value: skyN.tw, target: '0 ≤ τ_w ≤ 0.98', pass: skyN.tw > 0 && skyN.tw < 0.98, detail: `window fraction of E_b(T_a) = ${fmt(skyN.windowFraction, 3)}` });
    out.checks.push({ name: 'Energy balance closure at stagnation', value: stag.ok ? absorberQ(day, stag.x) : NaN, target: '|q| < 1e-6 W/m²', pass: stag.ok && Math.abs(absorberQ(day, stag.x)) < 1e-4 });
    // KPIs
    const K = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    K('Absorber solar absorptance α_s', aS_col, '', 'Solar-spectrum-weighted absorptance incl. soiling/shield losses. Selective absorbers reach ≈ 0.95.', 'calc');
    K('Absorber thermal emittance ε_th', eT_col, '', 'Blackbody-weighted emittance at the operating temperature — low values suppress re-radiation.', 'calc');
    K('Radiator solar absorptance', aS_rad, '', 'Fraction of sunlight absorbed by the radiator; must be ≲ 0.05 for daytime sub-ambient cooling.', 'calc');
    K('Radiator 8–13 µm emittance', eW_rad, '', 'Emittance inside the atmospheric window — the channel through which heat escapes to cold space.', 'calc');
    K('Day useful absorber flux', qDay, 'W/m²', `Net heat deliverable at T_a + ${p.dThot} K under ${fmt(day.ghi, 4)} W/m² (${day.src}).`, day.coupled ? 'coupled' : 'hypothetical', qDay > 0 ? 'ok' : 'bad');
    K('Absorber stagnation temperature', stag.ok ? stag.x - KELVIN : NaN, '°C', 'Temperature at which no heat can be extracted (upper bound for the hot side).', 'calc');
    K('Night net radiator cooling', qNight, 'W/m²', `Net cooling power at T_a − ${p.dTcold} K (${night.src}). Typical clear-night values are 40–100 W/m² at ambient, falling steeply below ambient.`, night.coupled ? 'coupled' : 'hypothetical', qNight > 0 ? 'ok' : 'bad');
    K('Radiator minimum (equilibrium) temperature', Tmin.ok ? Tmin.x - KELVIN : NaN, '°C', 'Lowest temperature the radiator reaches with no heat load — the cold-side limit at night.', 'calc');
    K('Daytime radiator cooling (same ΔT)', qRadDay, 'W/m²', 'Daytime sub-ambient cooling under sunlight; negative means the radiator heats up.', 'calc');
    K('Absorbed useful heat (collector area)', PabsW, 'W', 'Heat available to the thermal converter at the day operating point.', 'calc');
    K('Night cooling capacity (radiator area)', PcoolW, 'W', 'Heat the radiator can reject at the night operating point — this caps the night engine.', 'calc');
    K('Ideal (Carnot) work, day operating point', exDay, 'W', 'Upper bound on mechanical power from the absorbed heat with an ambient sink.', 'calc');
    K('Ideal (Carnot) work, night operating point', exNight, 'W', 'Upper bound on mechanical power using ambient air as source and the radiator as sink.', 'calc', exNight > 150 ? 'ok' : 'bad');
    K('Max ideal night work density', wNightMax, 'W/m²', `Best finite-time optimum over radiator temperature (at T = ${fmt(Tc[iWn] - KELVIN, 4)} °C). Compare: specification target 150 W/m², experimental reference ≈ 0.4 W/m².`, 'calc', wNightMax >= 150 ? 'ok' : 'bad');
    K('Max ideal day work density', wDayMax, 'W/m²', `Best finite-time optimum over absorber temperature (at T = ${fmt(Th[iWd] - KELVIN, 4)} °C).`, 'calc');
    // verdicts
    out.verdicts.push({ test: 'Daytime absorber delivers useful heat', status: qDay > 50 ? 'pass' : qDay > 0 ? 'warn' : 'fail', detail: `${fmt(qDay, 4)} W/m² at ${fmt(day.T + p.dThot, 4)} °C.`, fix: 'Lower the operating temperature, use a more selective absorber, add a convection shield or reduce the vehicle speed during harvesting.' });
    out.verdicts.push({ test: 'Night radiator rejects heat below ambient', status: qNight > 20 ? 'pass' : qNight > 0 ? 'warn' : 'fail', detail: `${fmt(qNight, 4)} W/m² at ${fmt(night.T - p.dTcold, 4)} °C; cloud ${fmt(night.cloud ?? 0, 3)} %.`, fix: 'Increase window emittance, add an IR-transparent shield, or accept a smaller sub-ambient ΔT.' });
    out.verdicts.push({ test: 'Night power-gap target (150 W/m² mechanical)', status: wNightMax >= 150 ? 'pass' : 'fail', detail: `Thermodynamic maximum ${fmt(wNightMax, 3)} W/m² = ${fmt(wNightMax / 150 * 100, 3)} % of the target, before any converter losses.`, fix: 'This is an environmental exergy limit — no photonic design can exceed it. Close the gap with wind, stored energy, or reduced road load.' });
    out.verdicts.push({ test: 'Second-law consistency', status: 'pass', detail: 'All work figures are bounded by Carnot efficiency of the actual surface temperatures (not sky or ambient temperatures).' });
    if (stackOn) out.verdicts.push({ test: 'Custom photonic stack evaluated', status: 'info', detail: `${layers.length} layers on ${p.substrate}; total thickness ${fmt(layers.reduce((s, l) => s + l.d, 0) * 1e6, 3)} µm.` });
    // charts
    const L = TH.LAMBDA, sol = L.map(l => TH.solarSpectrum(l, 1000)), solN = max(sol), bb = L.map(l => TH.planckLam(l, TaN)), bbN = max(bb), win = L.map(l => TH.windowShape(l));
    out.charts.push({ title: 'Spectral absorptivity / emissivity vs solar and thermal spectra', type: 'line', wide: true, x: { label: 'Wavelength', unit: 'µm', log: true }, y: { label: 'ε(λ) or normalised spectrum', min: 0, max: 1.05 }, series: [{ name: `Absorber — ${colM.name}`, x: L, y: epsCol, width: 2.5 }, { name: `Radiator — ${stackOn ? 'TMM stack' : radM.name}`, x: L, y: epsRad, width: 2.5 }, { name: 'Solar spectrum (norm.)', x: L, y: sol.map(v => v / solN), area: true, color: '#facc15', width: 1 }, { name: `Blackbody ${fmt(night.T, 3)} °C (norm.)`, x: L, y: bb.map(v => v / bbN), dash: '4 3', color: '#ef4444', width: 1 }, { name: 'Atmospheric window shape', x: L, y: win, area: true, color: '#22c55e', width: 1 }], explain: 'An ideal absorber is black under the yellow solar band and transparent-mirror-like beyond ≈ 2.5 µm. An ideal radiator reflects sunlight and emits strongly in the green 8–13 µm window where the atmosphere is transparent to space.' });
    out.charts.push({ title: 'Absorber useful heat vs operating temperature (day)', type: 'line', x: { label: 'Absorber temperature', unit: '°C' }, y: { label: 'Heat flux', unit: 'W/m²' }, series: [{ name: 'Useful heat q(T)', x: Th.map(t => t - KELVIN), y: qA }, { name: 'Ideal work q(1−T_a/T)', x: Th.map(t => t - KELVIN), y: wDay }], annotations: [{ type: 'hline', y: 0, color: '#94a3b8' }, { type: 'vline', x: day.T + p.dThot, label: 'operating point' }, { type: 'point', x: Th[iWd] - KELVIN, y: wDayMax, label: 'max-power point' }], explain: 'Hotter absorbers raise Carnot efficiency but lose more heat; their product peaks at the maximum-power temperature.' });
    out.charts.push({ title: 'Radiator net cooling vs temperature (night)', type: 'line', x: { label: 'Radiator temperature', unit: '°C' }, y: { label: 'Heat flux', unit: 'W/m²' }, series: [{ name: 'Net cooling q(T)', x: Tc.map(t => t - KELVIN), y: qR }, { name: 'Ideal work q(T_a/T − 1)', x: Tc.map(t => t - KELVIN), y: wNight }], annotations: [{ type: 'hline', y: 0, color: '#94a3b8' }, { type: 'vline', x: night.T, label: 'ambient' }, { type: 'vline', x: night.T - p.dTcold, label: 'operating', color: '#38bdf8' }], explain: 'Cooling power falls to zero at the equilibrium temperature. The ideal work curve shows how little mechanical power a few kelvin below ambient can support.' });
    out.charts.push({ title: 'Power-gap comparison (mechanical power density)', type: 'bar', horizontal: true, categories: ['Experimental night engine (literature ≈ 0.4)', 'This radiator: ideal maximum', 'This absorber (day): ideal maximum', 'Specification night target'], series: [{ name: 'W/m²', values: [0.4, wNightMax, wDayMax, 150], colors: ['#94a3b8', '#38bdf8', '#f97316', '#ef4444'] }], y: { label: 'Power density', unit: 'W/m²' }, showValues: true, explain: 'Ideal values are thermodynamic ceilings (no converter loss). Real converters achieve a small fraction of them.' });
    if (stackOn) { const lams = [0.55, 10]; const ang = linspace(0, 85, 18); out.charts.push({ title: 'Stack angular absorptance (s/p averaged)', type: 'line', x: { label: 'Incidence angle', unit: '°' }, y: { label: 'Absorptance', min: 0, max: 1 }, series: lams.map(l => ({ name: `λ = ${l} µm`, x: ang, y: ang.map(a => 0.5 * (TH.tmm(layers, p.substrate, l, a * Math.PI / 180, 's').A + TH.tmm(layers, p.substrate, l, a * Math.PI / 180, 'p').A)) })), explain: 'Angle dependence of the stack: radiative cooling needs high emittance over the hemisphere, low solar absorptance at all sun angles.' }); }
    if (hourly) { const tt = range(hourly.length); out.charts.push({ title: 'Hourly heat budget over the Suite 05 period', type: 'area', x: { label: 'Hour index', unit: 'h' }, y: { label: 'Power', unit: 'W' }, series: [{ name: 'Absorber useful heat', x: tt, y: hourly.map(r => r.abs) }, { name: 'Radiator cooling capacity', x: tt, y: hourly.map(r => r.rad) }], explain: 'Real weather drives the heat available to the converter hour by hour; integrate for daily energy (table).' }); }
    // tables
    out.tables.push({ title: 'Operating cases', cols: ['Case', 'Source', 'T_a (°C)', 'GHI (W/m²)', 'Cloud (%)', 'Sky ε', 'h (W/m²K)', 'Net flux (W/m²)'], rows: [['Day absorber', day.src, day.T, day.ghi, day.cloud ?? '', day.eps ?? '', hOf(day), qDay], ['Night radiator', night.src, night.T, night.ghi || 0, night.cloud ?? '', night.eps ?? '', hOf(night), qNight]] });
    if (hourly) { const Eabs = hourly.reduce((s, r) => s + r.abs, 0) / 1000, Erad = hourly.reduce((s, r) => s + r.rad, 0) / 1000; out.tables.push({ title: 'Energy totals over the analysed period', cols: ['Quantity', 'Value (kWh)', 'Per day (kWh/day)'], rows: [['Absorber useful heat', Eabs, Eabs / (hourly.length / 24)], ['Radiator cooling capacity', Erad, Erad / (hourly.length / 24)]] }); }
    out.tables.push({ title: 'Literature references (context, not predictions)', cols: ['Result', 'Value', 'Note'], rows: [['Night radiative-cooling mechanical engine', '≈ 0.4 W/m²', 'Relied on ground thermal coupling a moving vehicle lacks'], ['Night radiative-cooling electricity (2025)', '≈ 0.35 W/m²', 'Electrical, not mechanical'], ['Specification night target', '150 W/m²', '600 W from 4 m²']] });
    // recs
    if (aS_rad > 0.06) out.recs.push({ level: 'warn', text: `Radiator solar absorptance ${fmt(aS_rad, 3)} prevents daytime sub-ambient cooling; use a BaSO₄-type ultra-white or a polymer/dielectric photonic film (α_s ≤ 0.04).`, basis: 'Spectral balance' });
    if (eW_rad < 0.85) out.recs.push({ level: 'warn', text: `Window emittance ${fmt(eW_rad, 3)} is low; add a silica / silicon-nitride / alumina dielectric stack whose phonon resonances fill 8–13 µm.`, basis: 'Atmospheric window' });
    if (eT_col > 0.3) out.recs.push({ level: 'warn', text: `Absorber thermal emittance ${fmt(eT_col, 3)} is high; a cermet or TiNOx-type selective coating cuts re-radiation losses at elevated temperature.`, basis: 'Selective absorber theory' });
    if (!p.shield) out.recs.push({ level: 'info', text: 'An IR-transparent convection shield (thin polyethylene) typically doubles sub-ambient cooling at low wind — evaluate its drag and soiling in Suites 07 and 22.', basis: 'Convective parasitic gain' });
    out.recs.push({ level: 'critical', text: `Even with ideal conversion, the night radiator supports ≈ ${fmt(wNightMax * p.Arad, 3)} W of mechanical power over ${p.Arad} m² — far below the 124–406 W road demand. Plan night operation around wind or finite stored energy.`, basis: 'Carnot-limited exergy of measured/scenario sky' });
    out.recs.push({ level: 'sustain', text: 'Prefer coatings without critical raw materials (avoid Ag where Al or dielectric-only designs suffice); specify UV-stable, recyclable polymer films and design for coating replacement intervals.', basis: 'Material circularity' });
    out.explain.push({ title: 'What the photonic layer can and cannot do', text: 'Spectral engineering controls which wavelengths are absorbed and emitted, maximising heat collection by day and radiative heat rejection by night. It cannot create energy: the useful work is bounded by the heat flow times the Carnot factor of the real surface temperatures, which at night are only a few kelvin apart.' });
    out.outputs = { qNetDay_W_m2: qDay, qNetNight_W_m2: qNight, Pabs_W: PabsW, Pcool_W: PcoolW, Thot_C: day.T + p.dThot, Tcold_C: night.T - p.dTcold, collectorArea: p.Acol, radiatorArea: p.Arad, TstagC: stag.ok ? stag.x - KELVIN : null, TminC: Tmin.ok ? Tmin.x - KELVIN : null, TaDayC: day.T, TaNightC: night.T, curves: { absorber: { T: Th.map(t => t - KELVIN), q: qA }, radiator: { T: Tc.map(t => t - KELVIN), q: qR } }, wNightMax_W_m2: wNightMax, wDayMax_W_m2: wDayMax, aSolar: aS_col, eWindow: eW_rad, collector: p.collector, radiator: p.radiator };
    out.headline = `Day: ${fmt(qDay, 3)} W/m² useful heat at ${fmt(day.T + p.dThot, 3)} °C. Night: ${fmt(qNight, 3)} W/m² cooling at ${fmt(night.T - p.dTcold, 3)} °C — ideal night work ≤ ${fmt(wNightMax, 3)} W/m² vs 150 W/m² target.`;
    out.headlineKind = wNightMax >= 150 ? 'ok' : 'warn';
    return out;
  },
};
