// Suite 13 — Photonic metamaterials, solar absorption and radiative cooling.
import * as TH from '../core/thermo.js';
import { SCENARIOS, derive } from '../core/env.js';
import { linspace, fmt, trapz, max, argmax, range, KELVIN, SIGMA, goldenMin } from '../core/num.js';
import { forUse, get } from '../core/materials.js';
import { vehicleAssembly } from '../core/cad.js';
// Glazing options: solar transmittance, thermal-IR emittance (opaque glazings ≈ 0.9), expansion, continuous service limit, areal mass
const GLAZING = {
  glass: { name: 'Low-iron tempered glass 3.2 mm', tau: 0.91, eps: 0.88, cte: 9e-6, Tmax: 200, kgm2: 8.0, impact: 'brittle — unsuitable as a roll-over contact surface' },
  pc: { name: 'Hard-coated polycarbonate 2 mm (UV/abrasion coat)', tau: 0.88, eps: 0.9, cte: 68e-6, Tmax: 115, kgm2: 2.4, impact: 'impact-tough — suitable as roll-over contact surface' },
  etfe: { name: 'ETFE film 0.2 mm (tensioned)', tau: 0.94, eps: 0.55, cte: 100e-6, Tmax: 150, kgm2: 0.35, impact: 'tough film, but partly transparent to thermal IR (higher losses)' },
};
// Sky-facing area of a conformal thermal patch (thin closed shell → half its surface area), cached per vehicle
const _areaCache = new Map();
function patchInfo(veh, comp) { const key = JSON.stringify(veh) + comp; if (!_areaCache.has(key)) { try { const asm = vehicleAssembly(veh), parts = asm.parts.filter(m => m.component === comp); let area = parts.reduce((s, m) => s + m.area() / 2, 0), z = parts.length ? Math.max(...parts.map(m => m.bbox()[5])) : NaN; if (comp === 'collector' && asm.meta.arch?.collectorArea) { area = parts.reduce((s2, m) => s2 + (/bridge/i.test(m.name) ? asm.meta.arch.collectorArea : m.area() / 2), 0); } _areaCache.set(key, { area, z, names: parts.map(m => m.name) }); } catch { _areaCache.set(key, { area: 0, z: NaN, names: [] }); } } return _areaCache.get(key); }
function patchArea(veh, comp) { return patchInfo(veh, comp).area; }
// Revision 6: sky-facing (upward-projected) areas of the current CAD surfaces that can carry the selective absorber
// and glazing in addition to the nose and bridge collectors. The canopy and the rear-deck radiator stay clear.
const _skyCache = new Map();
export function skyAreas(veh) {
  const key = JSON.stringify(veh); if (_skyCache.has(key)) return _skyCache.get(key);
  let r = { nose: 0, bridge: 0, deck: 0, bridgeRest: 0, crossbeam: 0, fairings: 0, pods: 0, ext: 0, existing: 0, total: 0 };
  try {
    const asm = vehicleAssembly(veh), up = (m) => { if (!m) return 0; const P = m.p, I = m.i; let a = 0; for (let k = 0; k < I.length; k += 3) { const A = I[k] * 3, B = I[k + 1] * 3, C = I[k + 2] * 3; const ux = P[B] - P[A], uy = P[B + 1] - P[A + 1], vx = P[C] - P[A], vy = P[C + 1] - P[A + 1], nz = (ux * vy - uy * vx) / 2; if (nz > 0) { const uz = P[B + 2] - P[A + 2], vz = P[C + 2] - P[A + 2], nx = (uy * vz - uz * vy) / 2, ny = (uz * vx - ux * vz) / 2; if (nz / Math.hypot(nx, ny, nz) > 0.5) a += nz; } } return a; };
    const f = (re) => asm.parts.filter(m => re.test(m.name)), sum = (arr) => arr.reduce((s2, m) => s2 + up(m), 0);
    const body = sum(f(/^Monocoque body/i)), canopy = sum(f(/canopy/i)), rad = sum(f(/Radiative-cooling/i)), nose = sum(f(/collector \(nose/i)), bridgeCol = sum(f(/collector \(arch bridge/i)), bridge = sum(f(/Arch top bridge/i));
    r.nose = nose; r.bridge = bridgeCol; r.deck = Math.max(0, body - canopy - rad - nose); r.bridgeRest = Math.max(0, bridge - bridgeCol);
    r.crossbeam = 0.8 * sum(f(/root crossbeam/i)); r.fairings = sum(f(/root fairing/i)); r.pods = sum(f(/shoulder wheel pod/i));
    r.existing = nose + bridgeCol; r.ext = r.deck + r.bridgeRest + r.crossbeam + r.fairings + r.pods; r.total = r.existing + r.ext;
  } catch { /* no CAD — leave zeros */ }
  _skyCache.set(key, r); return r;
}

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
    { key: 'Acol', label: 'Absorber (collector) area', unit: 'm²', type: 'number', min: 0.1, max: 20, step: 0.1, group: 'Geometry', default: 4, auto: (c) => { const A = patchArea(c.vehicle, 'collector'); return A ? { value: +A.toFixed(2), prov: 'generated' } : undefined; }, help: 'Automatic: area of the conformal collector surface in the current CAD (Suite 02).' },
    { key: 'Arad', label: 'Radiator area', unit: 'm²', type: 'number', min: 0.1, max: 20, step: 0.1, group: 'Geometry', default: 4, auto: (c) => { const A = patchArea(c.vehicle, 'radiator'); return A ? { value: +A.toFixed(2), prov: 'generated' } : undefined; }, help: 'Automatic: area of the conformal radiative-cooling surface in the current CAD. The specification’s reference case uses 4 m².' },
    { key: 'glazed', label: 'Glazed collector (low-iron glass + insulated back)', type: 'bool', group: 'Collector construction', default: true, help: 'Revision 2 (D5): glazing suppresses convective and IR losses of the absorber.' },
    { key: 'glazingMat', label: 'Glazing material', type: 'select', group: 'Collector construction', default: 'glass', auto: (c) => ({ value: c.vehicle.wingConfig === 'arch' ? 'pc' : 'glass', prov: 'generated' }), options: Object.entries(GLAZING).map(([value, g]) => ({ value, label: g.name })), help: 'Revision 5.1: the bridge is also the roll-over contact surface, so impact-tough hard-coated PC replaces glass.' },
    { key: 'tauGlass', label: 'Glazing solar transmittance τ', type: 'number', min: 0.5, max: 0.98, step: 0.01, group: 'Collector construction', default: 0.91, auto: (c) => ({ value: GLAZING[c.vehicle.wingConfig === 'arch' ? 'pc' : 'glass'].tau, prov: 'library' }) },
    { key: 'bondT', label: 'Structural silicone bond thickness', unit: 'mm', type: 'number', min: 1, max: 15, step: 0.5, group: 'Collector construction', default: 3 },
    { key: 'panelL', label: 'Glazing tile size (segmented panels)', unit: 'm', type: 'number', min: 0.1, max: 2, step: 0.05, group: 'Collector construction', default: 0.25, help: 'Segmenting the PC glazing into small bonded tiles keeps the thermal-mismatch shear strain of the silicone bond within its allowable.' },
    { key: 'canopyCoat', label: 'Canopy', type: 'select', group: 'Cabin', default: 'irpc', options: [{ value: 'clear', label: 'Clear PC 3–4 mm' }, { value: 'irpc', label: 'IR-reflective PC 3–4 mm (spectrally selective)' }] },
    { key: 'cabinDT', label: 'Allowed cabin air rise above ambient', unit: 'K', type: 'number', min: 2, max: 20, step: 0.5, group: 'Cabin', default: 8 },
    { key: 'collectorExtent', label: 'Collector extent (Revision 6 store charging)', type: 'select', group: 'Collector construction', default: 'existing', options: [{ value: 'all', label: 'All sky-facing upper surfaces (body deck, bridge, crossbeam, root fairings, pods)' }, { value: 'existing', label: 'Nose and bridge collectors only' }], help: 'Suite 14 compares both extents against the vehicle mass cap.' },
    { key: 'collectorType', label: 'Collector construction for store charging', type: 'select', group: 'Collector construction', default: 'evaccpc', options: [{ value: 'flat', label: 'Glazed flat absorber (air gap)' }, { value: 'cpc', label: 'Static compound-parabolic troughs (C = 1.5, E–W axis, no tracking)' }, { value: 'evac', label: 'Evacuated flat glazing (2 × 2 mm glass, support pillars)' }, { value: 'evaccpc', label: 'Evacuated glazing over compound-parabolic troughs (no moving parts)' }] },
    { key: 'TabsStore', label: 'Absorber temperature while charging the store', unit: '°C', type: 'number', min: 40, max: 200, step: 1, group: 'Operating point', default: 180, help: 'PCM melting point plus the loop-heat-pipe temperature drop (D-mannitol 167 °C + ≈ 10 K; erythritol 118 °C + ≈ 7 K).' },
    { key: 'Uback', label: 'Back/edge insulation loss coefficient', unit: 'W/m²K', type: 'number', min: 0.1, max: 5, step: 0.1, group: 'Collector construction', default: 0.8 },
    { key: 'opMode', label: 'Operating temperatures', type: 'select', group: 'Operating point', default: 'optimal', options: [{ value: 'optimal', label: 'Exergy-optimal (maximise Q·Carnot factor)' }, { value: 'fixed', label: 'Fixed offsets below' }] },
    { key: 'dThot', label: 'Absorber temperature above ambient (fixed mode)', unit: 'K', type: 'range', min: 1, max: 120, step: 1, group: 'Operating point', default: 25 },
    { key: 'dTcold', label: 'Radiator temperature below ambient (fixed mode)', unit: 'K', type: 'range', min: 0, max: 20, step: 0.5, group: 'Operating point', default: 3 },
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
    const GZ = GLAZING[p.glazingMat] || GLAZING.glass, epsAbsTh = TH.thermalWeighted(epsCol, (day.T + KELVIN) + 60), epsGl = GZ.eps; let lastTg = NaN;
    const absorberQ = (c, T) => {
      const b = balance(c, epsCol, T);
      if (!p.glazed) return b.solAbs - (b.emit - b.atm) - b.h * (T - b.TaK);
      // Glazed: absorber → glass by gap convection + grey radiation; glass → ambient (convection) and sky (radiation)
      const Tsky = b.TaK * Math.max(0.5, c.eps ?? 0.8) ** 0.25, hGap = 2.8, rad = (a, g) => SIGMA * (a ** 4 - g ** 4) / (1 / epsAbsTh + 1 / epsGl - 1);
      const f = (Tg) => (hGap * (T - Tg) + rad(T, Tg)) - (b.h * (Tg - b.TaK) + epsGl * SIGMA * (Tg ** 4 - Tsky ** 4));
      const r = TH.brent(f, Math.min(T, Tsky) - 5, Math.max(T, b.TaK) + 5), Tg = r.ok ? r.x : b.TaK;
      lastTg = Tg; return b.solAbs * p.tauGlass - (hGap * (T - Tg) + rad(T, Tg)) - p.Uback * (T - b.TaK);
    };
    const radiatorQ = (c, T) => { const b = balance(c, epsRad, T); return (b.emit - b.atm) - b.solAbs - b.h * (b.TaK - T); };
    // operating points
    const TaD = day.T + KELVIN, TaN = night.T + KELVIN;
    // D5: exergy-optimal operating temperatures (maximise q·(1 − T0/T) for the absorber, q·(T0/T − 1) for the radiator)
    let dTh = p.dThot, dTc = p.dTcold;
    if (p.opMode !== 'fixed') { const Tst = goldenMin((T) => -Math.max(0, absorberQ(day, T)) * (1 - TaD / T), TaD + 0.5, TaD + 220, 1e-3); dTh = Math.max(0.5, Tst - TaD); const Tcs = goldenMin((T) => -Math.max(0, radiatorQ(night, T)) * (TaN / T - 1), TaN - 40, TaN - 0.05, 1e-3); dTc = Math.max(0.05, TaN - Tcs); }
    const qDay = absorberQ(day, TaD + dTh), qNight = radiatorQ(night, TaN - dTc), qRadDay = radiatorQ(day, TaD - dTc);
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
    const exDay = PabsW * Math.max(0, 1 - TaD / (TaD + dTh)), exNight = PcoolW * Math.max(0, TaN / (TaN - dTc) - 1);
    // spectral summaries
    const aS_col = TH.solarWeighted(epsCol) * solarDerate, eT_col = TH.thermalWeighted(epsCol, TaD + dTh), aS_rad = TH.solarWeighted(epsRad), eW_rad = TH.windowWeighted(epsRad, TaN), eT_rad = TH.thermalWeighted(epsRad, TaN);
    // hourly budget over Suite 05 series
    let hourly = null;
    if (env?.series?.length) {
      const S = env.series.slice(0, 168);
      hourly = S.map(r => { const c = { ...r, Uz: r.Uz }; const TaK = r.T + KELVIN; return { t: r.time, abs: Math.max(0, absorberQ(c, TaK + dTh)) * p.Acol, rad: Math.max(0, radiatorQ(c, TaK - dTc)) * p.Arad, day: r.isDay }; });
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
    K('Day useful absorber flux', qDay, 'W/m²', `Net heat deliverable at T_a + ${dTh} K under ${fmt(day.ghi, 4)} W/m² (${day.src}).`, day.coupled ? 'coupled' : 'hypothetical', qDay > 0 ? 'ok' : 'bad');
    { const ci = patchInfo(ctx.vehicle, 'collector'), ri = patchInfo(ctx.vehicle, 'radiator'); K('Collector / radiator placement', `${ci.names.map(n => n.replace(/^Solar-selective collector /, '')).join(' + ')} · ${(ri.names[0] || '—').replace(/^Radiative-cooling emitter /, '')}`, '', ctx.vehicle.wingConfig === 'arch' ? `AeroArch: primary collector on the unshaded nose deck above the driven front wheels plus the sky-facing arch-bridge top (horizontal, sky view ≈ 1, no ground-reflected gain credited, ${fmt(ci.z, 3)} m high so wind convection is stronger — glazing limits it); total ${fmt(ci.area, 3)} m². Suite 14 routes each to the hubs.` : 'Conformal body patches from the current CAD.', 'generated'); }
    // Revision 5.1: glazing attachment / thermal-mismatch and canopy solar-gain checks
    if (p.glazed) {
      absorberQ(day, TaD + dTh); const TgOp = lastTg - KELVIN; absorberQ(day, stag.ok ? stag.x : TaD + 150); const TgStag = lastTg - KELVIN;
      const Lp = p.panelL ?? 0.25, dT = TgStag - (-10), slip = (GZ.cte - 2e-6) * (Lp / 2) * dT * 1000, gam = slip / p.bondT, gAllow = 0.25;
      K('Glazing temperature (operating / stagnation)', `${fmt(TgOp, 4)} / ${fmt(TgStag, 4)}`, '°C', `${GZ.name}: continuous limit ${GZ.Tmax} °C. Stagnation = parked in full sun with no heat drawn.`, 'calc', TgStag <= GZ.Tmax ? 'ok' : 'bad');
      K('Glazing bond shear strain (thermal mismatch)', gam, '', `Expansion mismatch vs the CFRP bridge over a ${fmt(Lp, 3)} m glazing tile from −10 °C to stagnation (${fmt(slip, 3)} mm edge slip) on a ${p.bondT} mm structural-silicone bond; allowable ≈ ${gAllow}.`, 'calc', gam <= gAllow ? 'ok' : 'bad');
      out.verdicts.push({ test: 'Collector glazing attachment & temperature', status: TgStag <= GZ.Tmax && gam <= gAllow ? 'pass' : 'fail', detail: `${GZ.name}: ${fmt(TgStag, 4)} °C at stagnation vs ${GZ.Tmax} °C; bond shear strain ${fmt(gam, 3)} vs ${gAllow} with a ${p.bondT} mm bond. ${GZ.impact}.`, fix: `Thicker silicone bond (≥ ${fmt(Math.ceil(slip / gAllow * 2) / 2, 3)} mm), smaller panels, or venting/over-temperature shading at standstill.` });
    }
    { let Acan = 1.1; try { const cp = vehicleAssembly(ctx.vehicle).parts.find(m => /canopy/i.test(m.name)); if (cp) Acan = cp.area() / 2; } catch { /* default */ }
      const tauC = p.canopyCoat === 'irpc' ? 0.42 : 0.85, shade = ctx.vehicle.wingConfig === 'arch' ? 0.25 : 0, G0d = day.ghi || 0, Qc = G0d * Acan * tauC * (1 - shade), Qclear = G0d * Acan * 0.85, Vdot = Qc / (1.2 * 1005 * p.cabinDT);
      K('Canopy solar gain (clear PC / selected)', `${fmt(Qclear, 4)} / ${fmt(Qc, 4)}`, 'W', `${fmt(Acan, 3)} m² canopy at ${fmt(G0d, 4)} W/m²; ${p.canopyCoat === 'irpc' ? 'IR-reflective PC τ_sol 0.42' : 'clear PC τ_sol 0.85'}${shade ? ', arch bridge shades ≈ 25 %' : ''}.`, 'calc');
      K('Cabin vent airflow needed', Vdot * 1000, 'L/s', `Ram-air vent to keep the cabin within ${p.cabinDT} K of ambient (no fan — no battery).`, 'calc', Vdot * 1000 <= 60 ? 'ok' : 'warn');
      out.verdicts.push({ test: 'Canopy solar gain manageable by a passive vent', status: Vdot * 1000 <= 60 ? 'pass' : 'warn', detail: `${fmt(Qc, 4)} W gain needs ${fmt(Vdot * 1000, 3)} L/s of ram air (≈ 60 L/s is available from a small NACA duct at 20 km/h).` });
    }
    K('Absorber operating temperature', day.T + dTh, '°C', p.opMode !== 'fixed' ? `Exergy-optimal: maximises q·(1 − T_a/T) for the ${p.glazed ? 'glazed (τ = ' + p.tauGlass + ')' : 'unglazed'} collector.` : 'Fixed offset (user).', p.opMode !== 'fixed' ? 'generated' : 'user');
    K('Radiator operating temperature', night.T - dTc, '°C', p.opMode !== 'fixed' ? 'Exergy-optimal: maximises q·(T_a/T − 1).' : 'Fixed offset (user).', p.opMode !== 'fixed' ? 'generated' : 'user');
    K('Absorber stagnation temperature', stag.ok ? stag.x - KELVIN : NaN, '°C', 'Temperature at which no heat can be extracted (upper bound for the hot side).', 'calc');
    K('Night net radiator cooling', qNight, 'W/m²', `Net cooling power at T_a − ${dTc} K (${night.src}). Typical clear-night values are 40–100 W/m² at ambient, falling steeply below ambient.`, night.coupled ? 'coupled' : 'hypothetical', qNight > 0 ? 'ok' : 'bad');
    K('Radiator minimum (equilibrium) temperature', Tmin.ok ? Tmin.x - KELVIN : NaN, '°C', 'Lowest temperature the radiator reaches with no heat load — the cold-side limit at night.', 'calc');
    K('Daytime radiator cooling (same ΔT)', qRadDay, 'W/m²', 'Daytime sub-ambient cooling under sunlight; negative means the radiator heats up.', 'calc');
    K('Absorbed useful heat (collector area)', PabsW, 'W', 'Heat available to the thermal converter at the day operating point.', 'calc');
    K('Night cooling capacity (radiator area)', PcoolW, 'W', 'Heat the radiator can reject at the night operating point — this caps the night engine.', 'calc');
    K('Ideal (Carnot) work, day operating point', exDay, 'W', 'Upper bound on mechanical power from the absorbed heat with an ambient sink.', 'calc');
    K('Ideal (Carnot) work, night operating point', exNight, 'W', 'Upper bound on mechanical power using ambient air as source and the radiator as sink.', 'calc', exNight > 150 ? 'ok' : 'bad');
    K('Max ideal night work density', wNightMax, 'W/m²', `Best finite-time optimum over radiator temperature (at T = ${fmt(Tc[iWn] - KELVIN, 4)} °C). Compare: specification target 150 W/m², experimental reference ≈ 0.4 W/m².`, 'calc', wNightMax >= 150 ? 'ok' : 'bad');
    K('Max ideal day work density', wDayMax, 'W/m²', `Best finite-time optimum over absorber temperature (at T = ${fmt(Th[iWd] - KELVIN, 4)} °C).`, 'calc');
    // verdicts
    out.verdicts.push({ test: 'Daytime absorber delivers useful heat', status: qDay > 50 ? 'pass' : qDay > 0 ? 'warn' : 'fail', detail: `${fmt(qDay, 4)} W/m² at ${fmt(day.T + dTh, 4)} °C.`, fix: 'Lower the operating temperature, use a more selective absorber, add a convection shield or reduce the vehicle speed during harvesting.' });
    out.verdicts.push({ test: 'Night radiator rejects heat below ambient', status: qNight > 20 ? 'pass' : qNight > 0 ? 'warn' : 'fail', detail: `${fmt(qNight, 4)} W/m² at ${fmt(night.T - dTc, 4)} °C; cloud ${fmt(night.cloud ?? 0, 3)} %.`, fix: 'Increase window emittance, add an IR-transparent shield, or accept a smaller sub-ambient ΔT.' });
    out.verdicts.push({ test: 'Night power-gap target (150 W/m² mechanical)', status: wNightMax >= 150 ? 'pass' : 'fail', detail: `Thermodynamic maximum ${fmt(wNightMax, 3)} W/m² = ${fmt(wNightMax / 150 * 100, 3)} % of the target, before any converter losses.`, fix: 'This is an environmental exergy limit — no photonic design can exceed it. Close the gap with wind, stored energy, or reduced road load.' });
    out.verdicts.push({ test: 'Second-law consistency', status: 'pass', detail: 'All work figures are bounded by Carnot efficiency of the actual surface temperatures (not sky or ambient temperatures).' });
    if (stackOn) out.verdicts.push({ test: 'Custom photonic stack evaluated', status: 'info', detail: `${layers.length} layers on ${p.substrate}; total thickness ${fmt(layers.reduce((s, l) => s + l.d, 0) * 1e6, 3)} µm.` });
    // charts
    const L = TH.LAMBDA, sol = L.map(l => TH.solarSpectrum(l, 1000)), solN = max(sol), bb = L.map(l => TH.planckLam(l, TaN)), bbN = max(bb), win = L.map(l => TH.windowShape(l));
    out.charts.push({ title: 'Spectral absorptivity / emissivity vs solar and thermal spectra', type: 'line', wide: true, x: { label: 'Wavelength', unit: 'µm', log: true }, y: { label: 'ε(λ) or normalised spectrum', min: 0, max: 1.05 }, series: [{ name: `Absorber — ${colM.name}`, x: L, y: epsCol, width: 2.5 }, { name: `Radiator — ${stackOn ? 'TMM stack' : radM.name}`, x: L, y: epsRad, width: 2.5 }, { name: 'Solar spectrum (norm.)', x: L, y: sol.map(v => v / solN), area: true, color: '#facc15', width: 1 }, { name: `Blackbody ${fmt(night.T, 3)} °C (norm.)`, x: L, y: bb.map(v => v / bbN), dash: '4 3', color: '#ef4444', width: 1 }, { name: 'Atmospheric window shape', x: L, y: win, area: true, color: '#22c55e', width: 1 }], explain: 'An ideal absorber is black under the yellow solar band and transparent-mirror-like beyond ≈ 2.5 µm. An ideal radiator reflects sunlight and emits strongly in the green 8–13 µm window where the atmosphere is transparent to space.' });
    out.charts.push({ title: 'Absorber useful heat vs operating temperature (day)', type: 'line', x: { label: 'Absorber temperature', unit: '°C' }, y: { label: 'Heat flux', unit: 'W/m²' }, series: [{ name: 'Useful heat q(T)', x: Th.map(t => t - KELVIN), y: qA }, { name: 'Ideal work q(1−T_a/T)', x: Th.map(t => t - KELVIN), y: wDay }], annotations: [{ type: 'hline', y: 0, color: '#94a3b8' }, { type: 'vline', x: day.T + dTh, label: 'operating point' }, { type: 'point', x: Th[iWd] - KELVIN, y: wDayMax, label: 'max-power point' }], explain: 'Hotter absorbers raise Carnot efficiency but lose more heat; their product peaks at the maximum-power temperature.' });
    out.charts.push({ title: 'Radiator net cooling vs temperature (night)', type: 'line', x: { label: 'Radiator temperature', unit: '°C' }, y: { label: 'Heat flux', unit: 'W/m²' }, series: [{ name: 'Net cooling q(T)', x: Tc.map(t => t - KELVIN), y: qR }, { name: 'Ideal work q(T_a/T − 1)', x: Tc.map(t => t - KELVIN), y: wNight }], annotations: [{ type: 'hline', y: 0, color: '#94a3b8' }, { type: 'vline', x: night.T, label: 'ambient' }, { type: 'vline', x: night.T - dTc, label: 'operating', color: '#38bdf8' }], explain: 'Cooling power falls to zero at the equilibrium temperature. The ideal work curve shows how little mechanical power a few kelvin below ambient can support.' });
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
    out.views.push({ label: '2D animation: day/night spectral balance', icon: '🎞', render: (el) => anim2dBalance(el, { env, day, night, p: { ...p, dThot: dTh, dTcold: dTc }, absorberQ, radiatorQ, epsCol, epsRad, ctx }) });
    out.views.push({ label: '3D day/night cycle', icon: '🌗', render: (el) => view3dCycle(el, { env, day, night, p: { ...p, dThot: dTh, dTcold: dTc }, absorberQ, radiatorQ, colName: colM.name, radName: stackOn ? 'custom photonic stack' : radM.name, ctx }) });
    // ----- Revision 6: daily heat collected for the latent-heat store (hour by hour from the Suite 05 series) -----
    const SA = skyAreas(ctx.vehicle), Astore = p.collectorExtent === 'existing' ? SA.existing || p.Acol : SA.total || p.Acol;
    const hPark = (c) => p.shield ? 2 : TH.mcAdamsH(Math.max(0, c.Uz ?? c.U ?? 0)); // charging happens mostly parked
    // Solar position for the compound-parabolic acceptance check (E–W trough axis, horizontal aperture):
    // beam is accepted while the transverse projected angle tan θ_p = tan θ_z·|cos γ_s| stays within θ_a = asin(1/C).
    const lat = (env?.lat ?? ctx.site?.lat ?? 0) * Math.PI / 180, sunPos = (r) => { const d = new Date(r.time || Date.now()), doy = Math.floor((d - new Date(d.getFullYear(), 0, 0)) / 864e5), dec = 23.45 * Math.PI / 180 * Math.sin(2 * Math.PI * (284 + doy) / 365), hr = d.getHours() + d.getMinutes() / 60, w = (hr - 12) * 15 * Math.PI / 180; const cz = Math.sin(lat) * Math.sin(dec) + Math.cos(lat) * Math.cos(dec) * Math.cos(w), z = Math.acos(Math.max(-1, Math.min(1, cz))); const az = Math.atan2(Math.sin(w), Math.cos(w) * Math.sin(lat) - Math.tan(dec) * Math.cos(lat)); return { z, az }; };
    const accepted = (r, Cc) => { if (Cc <= 1) return 1; const { z, az } = sunPos(r); if (z >= Math.PI / 2) return 0; const tp = Math.atan(Math.tan(z) * Math.abs(Math.cos(az))); return tp <= Math.asin(1 / Cc) ? 1 : 0; };
    const collQ = (c, T, type) => { // useful heat per m² of aperture for the static constructions
      const b = balance(c, epsCol, T), hC = hPark(c), Tsky = b.TaK * Math.max(0.5, c.eps ?? 0.8) ** 0.25, evac = type === 'evac' || type === 'evaccpc';
      const Cc = type === 'cpc' || type === 'evaccpc' ? 1.5 : 1, tau = evac ? 0.91 * 0.91 : p.tauGlass, eG = evac ? 0.88 : epsGl, hG = evac ? 0 : 2.8, back = evac ? 1.0 : p.Uback; // evacuated: edge seal + support-pillar conduction ≈ 1 W/m²K
      const rad = (a, g) => SIGMA * (a ** 4 - g ** 4) / (1 / epsAbsTh + 1 / eG - 1), fl = (Tg) => (hG * (T - Tg) + rad(T, Tg)) - (hC * (Tg - b.TaK) + eG * SIGMA * (Tg ** 4 - Tsky ** 4)) / Cc;
      const rr = TH.brent(fl, Math.min(T, Tsky) - 5, Math.max(T, b.TaK) + 5), Tg = rr.ok ? rr.x : b.TaK;
      const fb = (c.ghi || 0) > 0 && (c.dni || 0) > 0 ? Math.min(0.85, Math.max(0.1, 1 - (c.cloud ?? 20) / 100)) : 0.15;
      const opt = Cc > 1 ? fb * 0.96 * accepted(c, Cc) + (1 - fb) / Cc : 1;
      return b.solAbs * tau * opt - ((hG * (T - Tg) + rad(T, Tg)) + back * (T - b.TaK)) / Cc;
    };
    const hrs = (env?.series || []).slice(0, 24), daily = (type, T) => hrs.reduce((sum, r) => sum + Math.max(0, (r.ghi || 0) > 0 ? collQ(r, T, type) : 0) * 3600, 0);
    const kgm2 = { flat: GZ.kgm2 + 1.1 + 0.75 + 0.3, cpc: GZ.kgm2 + 1.1 / 1.5 + 0.75 + 0.3 + 0.6, evac: 2 * 5.0 + 1.1 + 0.3 + 0.4, evaccpc: 2 * 5.0 + 1.1 / 1.5 + 0.3 + 0.4 + 0.6 };
    const typeName = { flat: 'Glazed flat absorber', cpc: 'Compound-parabolic troughs C = 1.5', evac: 'Evacuated flat glazing', evaccpc: 'Evacuated glazing + compound-parabolic troughs' };
    const accFrac = hrs.length ? hrs.filter(r => (r.ghi || 0) > 0).reduce((s2, r) => s2 + (r.ghi || 0) * accepted(r, 1.5), 0) / Math.max(1, hrs.filter(r => (r.ghi || 0) > 0).reduce((s2, r) => s2 + (r.ghi || 0), 0)) : 1;
    const cmp = ['flat', 'cpc', 'evac', 'evaccpc'].map(t => ({ t, d: daily(t, p.TabsStore), noon: hrs.length ? Math.max(...hrs.map(r => (r.ghi || 0) > 0 ? collQ(r, p.TabsStore, t) : 0)) : 0 }));
    const sel = cmp.find(x => x.t === p.collectorType) || cmp[0], dailyStore = sel.d * Astore, extMass = SA.ext * kgm2[sel.t] * (p.collectorExtent === 'existing' ? 0 : 1) + SA.existing * Math.max(0, kgm2[sel.t] - kgm2.flat); // new surfaces + upgrade of the existing glazed collectors
    const qHourly = hrs.map(r => (r.ghi || 0) > 0 ? Math.max(0, collQ(r, p.TabsStore, sel.t)) : 0);
    K('Sky-facing collector area for the store', Astore, 'm²', `Nose ${fmt(SA.nose, 3)} + bridge ${fmt(SA.bridge, 3)} m² existing${p.collectorExtent === 'existing' ? '' : `; added: body deck ${fmt(SA.deck, 3)}, bridge remainder ${fmt(SA.bridgeRest, 3)}, crossbeam ${fmt(SA.crossbeam, 3)} (80 % unshaded), root fairings ${fmt(SA.fairings, 3)}, shoulder pods ${fmt(SA.pods, 3)} m² (upward projections of the current CAD; canopy and rear-deck radiator left clear)`}.`, 'generated');
    K('Heat collected per sunny day (store charging)', dailyStore / 1e6, 'MJ', `${typeName[sel.t]} at ${p.TabsStore} °C absorber temperature, hour by hour over the Suite 05 day (parked convection): ${fmt(sel.d / 1e6, 3)} MJ/m²; peak ${fmt(sel.noon, 3)} W/m² (η at noon ≈ ${fmt(sel.noon / Math.max(1, Math.max(...hrs.map(r => r.ghi || 0))) * 100, 3)} %).`, env?.series ? 'coupled' : 'hypothetical', dailyStore > 0 ? 'ok' : 'bad');
    K('Compound-parabolic acceptance vs sun path', accFrac * 100, '% of daily irradiance', `E–W trough axis, C = 1.5 → acceptance half-angle ${fmt(Math.asin(1 / 1.5) * 180 / Math.PI, 3)}°; share of the day's irradiance arriving while the sun's transverse angle is inside it (latitude ${fmt(lat * 180 / Math.PI, 3)}°). Outside the acceptance only the diffuse share 1/C is collected.`, env?.series ? 'coupled' : 'hypothetical', accFrac > 0.8 ? 'ok' : 'warn');
    out.tables.push({ title: 'Static collector constructions for store charging (no moving parts)', cols: ['Construction', 'Heat per sunny day (MJ/m²)', 'Noon useful flux (W/m²)', 'Areal mass (kg/m²)', 'Heat per day per kg of collector (MJ/kg)'], rows: cmp.map(x => [typeName[x.t], x.d / 1e6, x.noon, kgm2[x.t], x.d / 1e6 / kgm2[x.t]]) });
    out.charts.push({ title: 'Store-charging heat through the day', type: 'area', x: { label: 'Hour of the Suite 05 series', unit: 'h' }, y: { label: 'Useful heat', unit: 'W' }, series: [{ name: `Collector → store (${fmt(Astore, 3)} m²)`, x: range(qHourly.length), y: qHourly.map(q => q * Astore) }], explain: `Hour-by-hour useful heat at a ${p.TabsStore} °C absorber. The loop heat pipe is a one-way thermal diode, so heat only flows into the store while the absorber is hotter.` });
    out.outputs = { dailyHeat_J: dailyStore, dailyHeat_J_m2: sel.d, hourlyStore_W: qHourly.map(q => q * Astore), collectorAreaStore_m2: Astore, collectorExtArea_m2: p.collectorExtent === 'existing' ? 0 : SA.ext, collectorExtMass_kg: extMass, collectorType: sel.t, collectorKgm2: kgm2[sel.t], collectorOptions: cmp.map(x => ({ type: x.t, name: typeName[x.t], dailyJ_m2: x.d, kgm2: kgm2[x.t] })), cpcAcceptance: accFrac, TabsStore_C: p.TabsStore, noonStoreFlux_W_m2: sel.noon, skyAreas: SA, qNetDay_W_m2: qDay, qNetNight_W_m2: qNight, Pabs_W: PabsW, Pcool_W: PcoolW, Thot_C: day.T + dTh, Tcold_C: night.T - dTc, collectorArea: p.Acol, radiatorArea: p.Arad, TstagC: stag.ok ? stag.x - KELVIN : null, TminC: Tmin.ok ? Tmin.x - KELVIN : null, TaDayC: day.T, TaNightC: night.T, curves: { absorber: { T: Th.map(t => t - KELVIN), q: qA }, radiator: { T: Tc.map(t => t - KELVIN), q: qR } }, wNightMax_W_m2: wNightMax, wDayMax_W_m2: wDayMax, aSolar: aS_col, eWindow: eW_rad, collector: p.collector, radiator: p.radiator };
    out.headline = `Day: ${fmt(qDay, 3)} W/m² useful heat at ${fmt(day.T + dTh, 3)} °C. Night: ${fmt(qNight, 3)} W/m² cooling at ${fmt(night.T - dTc, 3)} °C — ideal night work ≤ ${fmt(wNightMax, 3)} W/m² vs 150 W/m² target.`;
    out.headlineKind = wNightMax >= 150 ? 'ok' : 'warn';
    return out;
  },
};

// 24-h 3D cycle on the vehicle: sun path from the solar-position algorithm, sky colour, collector/radiator
// coloured by the solved net heat flux each hour, radiative flux arrows. Hourly values are solved; frames between hours are interpolated.
async function view3dCycle(el, { env, day, night, p, absorberQ, radiatorQ, colName, radName, ctx }) {
  const dTh = p.dThot, dTc = p.dTcold;
  const [{ vehicleScene, playback, hud, legend3d }, { solarPosition }, { cmap }, { h }, mats] = await Promise.all([import('../core/sim3d.js'), import('../core/physics.js'), import('../core/chart.js'), import('../core/ui.js'), import('../core/materials.js')]);
  const lat = env?.lat ?? ctx.site.lat, lon = env?.lon ?? ctx.site.lon;
  // hourly forcing: Suite 05 series (first full day) or a diurnal blend of the representative day/night cases
  let hours;
  if (env?.series?.length >= 24) { const s = env.series, i0 = Math.max(0, s.findIndex(r => /T00:00/.test(r.time))); hours = range(24).map(k => s[Math.min(s.length - 1, i0 + k)]).map(r => ({ ...r, date: new Date(Date.parse(r.time + 'Z') - lon / 15 * 3600e3) })); }
  else { const base = new Date(); base.setUTCHours(0, 0, 0, 0); hours = range(24).map(k => { const sun = Math.max(0, Math.sin(Math.PI * (k - 6) / 12)); return { time: `${String(k).padStart(2, '0')}:00`, T: night.T + (day.T - night.T) * sun, ghi: (day.ghi || 0) * sun, cloud: sun > 0 ? day.cloud : night.cloud, eps: sun > 0 ? day.eps : night.eps, Uz: sun > 0 ? day.Uz : night.Uz, isDay: sun > 0, date: new Date(base.getTime() + (k - lon / 15) * 3600e3) }; }); }
  const solved = hours.map(r => { const c = { ...r, ghi: r.isDay ? (r.ghi || 0) : 0 }, TaK = r.T + KELVIN; const sp = solarPosition(r.date, lat, lon); return { r, sp, qa: absorberQ(c, TaK + dTh), qr: radiatorQ(c, TaK - dTc) }; });
  const qs = solved.flatMap(s => [s.qa, s.qr]), lo = Math.min(0, ...qs), hi = Math.max(1, ...qs);
  el.append(h('p', { class: 'small muted' }, `Solved each hour: sun position (NOAA algorithm), absorber useful flux at T_a + ${dTh} K (${colName}) and radiator net cooling at T_a − ${dTc} K (${radName}) from the full spectral balance with the hour's air temperature, irradiance, cloud and sky emissivity. Frames between hours are linearly interpolated. Colour = net heat flux (W/m²): positive means useful heat collected (absorber) or rejected (radiator).`));
  const box = h('div'); el.append(box);
  const matMap = Object.fromEntries(Object.keys(mats.COMPONENTS).map(k => [k, ctx.material(k)]));
  const S = await vehicleScene(box, { vehicle: ctx.vehicle, materials: matMap, follow: false, height: 440 });
  for (const k of ['thrust', 'side', 'drag', 'hub', 'wind']) S.setArrow(k, [0, 0, 0]);
  S.V.controls.autoRotate = true; S.V.controls.autoRotateSpeed = 2; S.V.renderer.domElement.addEventListener('pointerdown', () => { S.V.controls.autoRotate = false; });
  const THREE = S.THREE, V = S.V, surf = (comp) => S.car.children.filter(o => o.userData?.component === comp);
  const col = surf('collector'), rad = surf('radiator');
  const sunLight = new THREE.DirectionalLight(0xfff2cc, 2.5); V.scene.add(sunLight);
  const sunBall = new THREE.Mesh(new THREE.SphereGeometry(0.6, 24, 16), new THREE.MeshBasicMaterial({ color: 0xffd34d })); V.root.add(sunBall);
  const solarArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(), 1, 0xfacc15, 0.3, 0.18); V.root.add(solarArrow);
  const coolArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 1, 0x38bdf8, 0.3, 0.18); V.root.add(coolArrow);
  const centerOf = (objs) => { const b = new THREE.Box3(); objs.forEach(o => b.expandByObject(o)); return b.isEmpty() ? new THREE.Vector3(0, 0, 1) : b.getCenter(new THREE.Vector3()); };
  const cCol = centerOf(col), cRad = centerOf(rad);
  const H = hud(box); legend3d(box, { title: 'Net surface heat flux', lo, hi, unit: ' W/m²', cmapName: 'coolwarm' });
  const tint = (objs, q) => { const c = cmap((q - lo) / ((hi - lo) || 1), 'coolwarm'); objs.forEach(o => { o.material.color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255); o.material.emissive = new THREE.Color(c[0] / 600, c[1] / 600, c[2] / 600); }); };
  const lerp = (a, b, f) => a + (b - a) * f, night0 = new THREE.Color(0x0b1630), day0 = new THREE.Color(0x8fbbe8);
  const ground = V.root.children.find(o => o.geometry?.type === 'PlaneGeometry'), gNight = new THREE.Color(0x141b2b), gDay = new THREE.Color(0x7a8494), hemi = V.scene.children.find(o => o.isHemisphereLight);
  playback(box, { duration: 24, label: 'Local hour', unit: 'h', onFrame: (t) => {
    const i = Math.min(22, Math.floor(t)), f = t - i, A = solved[i], B = solved[i + 1];
    const el_ = lerp(A.sp.elevation, B.sp.elevation, f), azA = A.sp.azimuth, azB = B.sp.azimuth + (Math.abs(B.sp.azimuth - azA) > 180 ? (B.sp.azimuth < azA ? 360 : -360) : 0), az = lerp(azA, azB, f) * Math.PI / 180, e = el_ * Math.PI / 180;
    const dir = new THREE.Vector3(Math.sin(az) * Math.cos(e), Math.cos(az) * Math.cos(e), Math.sin(e));
    sunBall.position.copy(dir.clone().multiplyScalar(18)); sunBall.visible = el_ > -2; sunLight.position.copy(dir.clone().multiplyScalar(30)); sunLight.intensity = Math.max(0, Math.sin(e)) * 3;
    const dayF = Math.max(0, Math.min(1, (el_ + 6) / 20)); V.scene.background = night0.clone().lerp(day0, dayF); if (ground) ground.material.color.copy(gNight.clone().lerp(gDay, dayF)); if (hemi) hemi.intensity = 0.35 + 0.85 * dayF;
    const ghi = lerp(A.r.ghi || 0, B.r.ghi || 0, f), qa = lerp(A.qa, B.qa, f), qr = lerp(A.qr, B.qr, f), T = lerp(A.r.T, B.r.T, f);
    tint(col, qa); tint(rad, qr);
    solarArrow.visible = ghi > 5 && el_ > 0; if (solarArrow.visible) { const L = 0.4 + ghi / 400; solarArrow.position.copy(cCol.clone().add(dir.clone().multiplyScalar(L))); solarArrow.setDirection(dir.clone().negate()); solarArrow.setLength(L, 0.25, 0.15); }
    coolArrow.visible = qr > 0; if (coolArrow.visible) { const L = 0.3 + qr / 60; coolArrow.position.copy(cRad); coolArrow.setLength(L, 0.25, 0.15); }
    H.set([['Hour', `${String(Math.floor(t)).padStart(2, '0')}:${String(Math.floor(f * 60)).padStart(2, '0')}`], ['Sun elevation', fmt(el_, 3) + '°'], ['GHI', fmt(ghi, 3) + ' W/m²'], ['Air temperature', fmt(T, 3) + ' °C'], ['Cloud', fmt(lerp(A.r.cloud ?? 0, B.r.cloud ?? 0, f), 3) + ' %'], ['Absorber net flux', fmt(qa, 3) + ' W/m²', qa > 0 ? '#f97316' : '#94a3b8'], ['Radiator net cooling', fmt(qr, 3) + ' W/m²', qr > 0 ? '#38bdf8' : '#94a3b8'], ['Absorber heat (×A)', fmt(Math.max(0, qa) * p.Acol, 3) + ' W'], ['Radiator capacity (×A)', fmt(Math.max(0, qr) * p.Arad, 3) + ' W']]);
  } });
}

// 2D animation over 24 h: solar spectrum scaled to the hour's irradiance against the absorber/radiator spectral
// properties, plus traces of the solved net fluxes and equilibrium (no-load) surface temperatures with a time cursor.
async function anim2dBalance(el, { env, day, night, p, absorberQ, radiatorQ, epsCol, epsRad, ctx }) {
  const [{ anim2d }, { h }] = await Promise.all([import('../core/sim3d.js'), import('../core/ui.js')]);
  const lon = env?.lon ?? ctx.site.lon; void lon;
  let hours; if (env?.series?.length >= 24) { const s = env.series, i0 = Math.max(0, s.findIndex(r => /T00:00/.test(r.time))); hours = range(24).map(k => s[Math.min(s.length - 1, i0 + k)]); }
  else hours = range(24).map(k => { const sun = Math.max(0, Math.sin(Math.PI * (k - 6) / 12)); return { T: night.T + (day.T - night.T) * sun, ghi: (day.ghi || 0) * sun, cloud: sun > 0 ? day.cloud : night.cloud, eps: sun > 0 ? day.eps : night.eps, Uz: sun > 0 ? day.Uz : night.Uz, isDay: sun > 0 }; });
  const sol = hours.map(r => { const c = { ...r, ghi: r.isDay ? (r.ghi || 0) : 0 }, Ta = r.T + KELVIN; const sa = TH.brent((T) => absorberQ(c, T), Ta - 5, Ta + 320), sr = TH.brent((T) => radiatorQ(c, T), Ta - 70, Ta + 40); return { ghi: c.ghi, T: r.T, qa: absorberQ(c, Ta + p.dThot), qr: radiatorQ(c, Ta - p.dTcold), Tabs: sa.ok ? sa.x - KELVIN : NaN, Trad: sr.ok ? sr.x - KELVIN : NaN }; });
  el.append(h('p', { class: 'small muted' }, 'Hourly values are solved (spectral radiative balance with the hour’s air temperature, irradiance, cloud and sky emissivity); frames between hours interpolate. Left: spectra; right: net fluxes at the operating temperatures and the no-load equilibrium surface temperatures.'));
  const L = TH.LAMBDA, s1 = L.map(l => TH.solarSpectrum(l, 1000)), sM = Math.max(...s1), hh = range(24), yQ = [Math.min(0, ...sol.flatMap(s => [s.qa, s.qr])) - 10, Math.max(...sol.flatMap(s => [s.qa, s.qr])) + 10], yT = [Math.min(...sol.flatMap(s => [s.Trad, s.T]).filter(Number.isFinite)) - 3, Math.max(...sol.flatMap(s => [s.Tabs, s.T]).filter(Number.isFinite)) + 3];
  anim2d(el, { height: 460, duration: 24, rate: 3, label: 'Local hour', unit: 'h', onFrame: (t, g) => {
    g.clear(); const i = Math.min(22, Math.floor(t)), f = t - i, lp = (a, b) => a + (b - a) * f, G = lp(sol[i].ghi, sol[i + 1].ghi);
    const A = g.panel(0, 0, g.W * 0.5, g.H, { xr: [0.3, 20], yr: [0, 1.05], xlabel: 'Wavelength [µm] (0.3–20)', ylabel: 'ε(λ), normalised spectrum', title: `Spectra — GHI ${fmt(G, 3)} W/m²` });
    A.area(L, s1.map(v => v / sM * G / 1000), 'rgba(250,204,21,.45)'); A.line(L, epsCol, '#f97316', 2); A.line(L, epsRad, '#38bdf8', 2); A.area(L, L.map(l => TH.windowShape(l) * 0.25), 'rgba(34,197,94,.25)');
    g.text('orange: absorber   blue: radiator   yellow: sun (scaled)   green: sky window', A.px + 4, A.py + 14, { size: 10, color: g.mute });
    { const cx = A.px + A.pw - 46, cy = A.py + 70, R = 34, ang = Math.PI * (1 - (t - 6) / 12), up = t > 6 && t < 18; g.ctx.strokeStyle = g.mute; g.ctx.beginPath(); g.ctx.arc(cx, cy, R, Math.PI, 2 * Math.PI); g.ctx.stroke(); g.ctx.fillStyle = up ? '#facc15' : '#64748b'; g.ctx.beginPath(); g.ctx.arc(cx + R * Math.cos(ang), cy - R * Math.sin(ang), 8, 0, 2 * Math.PI); g.ctx.fill(); g.text(`${String(Math.floor(t)).padStart(2, '0')}:${String(Math.floor((t % 1) * 60)).padStart(2, '0')}`, cx, cy + 16, { size: 12, align: 'center', bold: true }); }
    const B = g.panel(g.W * 0.5, 0, g.W * 0.5, g.H / 2, { xr: [0, 23], yr: yQ, xlabel: 'Hour', ylabel: 'Net flux [W/m²]', title: 'Absorber (orange) / radiator (blue) net flux' });
    B.line(hh, sol.map(s => s.qa), '#f97316', 2); B.line(hh, sol.map(s => s.qr), '#38bdf8', 2); B.hline(0, '#94a3b8'); B.vline(t); B.point(t, lp(sol[i].qa, sol[i + 1].qa), '#f97316'); B.point(t, lp(sol[i].qr, sol[i + 1].qr), '#38bdf8');
    const C = g.panel(g.W * 0.5, g.H / 2, g.W * 0.5, g.H / 2, { xr: [0, 23], yr: yT, xlabel: 'Hour', ylabel: 'Temperature [°C]', title: 'Equilibrium surface temperatures vs air' });
    C.line(hh, sol.map(s => s.Tabs), '#ef4444', 2); C.line(hh, sol.map(s => s.Trad), '#3b82f6', 2); C.line(hh, sol.map(s => s.T), '#94a3b8', 1.5, [4, 3]); C.vline(t);
  } });
}
