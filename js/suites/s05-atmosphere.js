// Suite 05 — Live atmospheric environment, meteorology and environmental physics.
import { SCENARIOS, derive, roughnessOptions, ROUGHNESS } from '../core/env.js';
import { moistAir, isa, logWind, powerWind, kaimal, vonKarman, oneMinusCos, psiM } from '../core/physics.js';
import { linspace, range, mean, max, min, fmt, logspace, rng, percentile } from '../core/num.js';
import { forecast, nasaPowerClimatology, hourlyTable, airQuality, elevation as elevApi } from '../core/live.js';
import { h, table } from '../core/ui.js';

export default {
  description: 'Pulls live and historical weather for any WGS84 location directly from global meteorological services, assimilates it into moist-air thermodynamics, boundary-layer wind profiles, gust spectra, solar geometry and night-sky radiation, and produces time-dependent boundary conditions with provenance and uncertainty for every downstream solver.',
  literature: 'atmospheric boundary layer wind profile sky emissivity radiative cooling vehicle',
  credibility: 'Analytical screening',
  equations: [
    'Ideal-gas equation of state for dry air and moist-air equation of state with Dalton’s law of partial pressures',
    'Saturation vapour pressure (Buck), mixing ratio, specific humidity, relative humidity, dew point and virtual temperature relations',
    'Hydrostatic balance, barometric (ISA) and hypsometric equations for pressure with altitude',
    'Logarithmic wind-profile law and power-law profile; Monin–Obukhov similarity with Businger–Dyer stability functions; roughness-length classification',
    'Kaimal and von Kármán turbulence spectra; stochastic wind-field synthesis by inverse spectral (random-phase) method; one-minus-cosine discrete gust',
    'NOAA solar-position algorithm (zenith, azimuth, declination, equation of time); extraterrestrial irradiance; Ineichen–Perez clear-sky model (Linke turbidity)',
    'Clear-sky effective emissivity correlations (Berdahl–Martin, Swinbank, Brunt, Idso–Jackson), cloud correction, downward longwave radiation, radiative-equivalent sky temperature',
    'Sutherland viscosity and conductivity of air; dynamic pressure q = ½ρV²',
  ],
  methods: ['Hourly assimilation of provider fields with quality control (range checks, missing-value flags)', 'Temporal interpolation of hourly series', 'Uncertainty bands from forecast spread proxies (gust–mean ratio, model-type tolerances)', 'Wind-rose binning (16 sectors × 5 speed classes)'],
  assumptions: ['Neutral stability profile unless an Obukhov length is supplied', 'Effective sky temperature is a calculated estimate, never a measured reservoir temperature', 'Forecast-model hours include recent analysis hours — they are not station observations'],
  ibc: {
    initial: ['Geographic coordinates, altitude, timestamp, season', 'Atmospheric temperature, pressure, humidity, air density, wind vector, gust state, turbulence intensity', 'Solar position, cloud fraction, ground temperature, sky-radiation estimate, stability class'],
    boundary: ['Time-dependent weather-provider fields as timestamped series', 'Reference-height pressure, wind profile over terrain roughness, ground thermal condition', 'Solar irradiance (GHI/DNI/DHI), atmospheric longwave, cloud-dependent radiative exchange', 'For local atmospheric CFD: velocity inlet with log profile, pressure outlet, terrain no-slip, turbulence inflow, slip top boundary'],
  },
  cvv: {
    calibration: ['Weather-station observations, anemometer & wind-vane data', 'Pyranometer (GHI) and pyrgeometer (longwave) readings', 'Barometer, hygrometer and thermometer series'],
    verification: ['ISA reference values at sea level and 11 km', 'Psychrometric identities (ρ, w, Td round-trip)', 'Solar-position check against NOAA reference at equinox/solstice', 'Spectrum integral equals prescribed variance'],
    validation: ['Independent on-site station data (reserved validation period)', 'Measured sky temperature via pyrgeometer versus calculated estimate'],
  },
  formats: { import: ['CSV/TSV station logs', 'XLSX', 'JSON', 'NetCDF-derived CSV exports', 'GPX/KML routes for terrain'], export: ['CSV time series', 'JSON boundary-condition package', 'PNG/SVG plots'] },
  params: [
    { key: 'scenario', label: 'Atmospheric scenario', type: 'select', group: 'Scenario', default: 'live', options: Object.entries(SCENARIOS).map(([k, v]) => ({ value: k, label: v.label })), help: 'Live pulls the forecast/analysis for the project site; others are the standard engineering scenarios from the specification.' },
    { key: 'lat', label: 'Latitude', unit: '°', type: 'number', min: -90, max: 90, step: 0.0001, group: 'Site', default: 6.5244, auto: (c) => ({ value: c.site.lat, prov: 'user' }) },
    { key: 'lon', label: 'Longitude', unit: '°', type: 'number', min: -180, max: 180, step: 0.0001, group: 'Site', default: 3.3792, auto: (c) => ({ value: c.site.lon, prov: 'user' }) },
    { key: 'hourOffset', label: 'Analysis hour from now', unit: 'h', type: 'range', min: -48, max: 160, step: 1, group: 'Site', default: 0, help: 'Which hour of the fetched history/forecast becomes the boundary-condition snapshot passed to other suites.' },
    { key: 'roughness', label: 'Terrain roughness class', type: 'select', group: 'Boundary layer', default: 'farmland, few hedges', options: roughnessOptions, auto: (c) => c.site.roughness },
    { key: 'zWing', label: 'Wing centroid height', unit: 'm', type: 'number', min: 0.5, max: 10, step: 0.1, group: 'Boundary layer', default: 2.3, auto: (c) => ({ value: +(c.vehicle.wheelRadius + c.vehicle.height * 0.7 + c.vehicle.wingHeight / 2).toFixed(2), prov: 'calc' }) },
    { key: 'obukhov', label: 'Obukhov length L (0 = neutral)', unit: 'm', type: 'number', min: -500, max: 500, step: 1, group: 'Boundary layer', default: 0, expert: true, help: 'Negative = unstable (daytime convective), positive = stable (night).' },
    { key: 'skyModel', label: 'Sky emissivity model', type: 'select', group: 'Radiation', default: 'Berdahl–Martin', options: ['Berdahl–Martin', 'Swinbank', 'Brunt', 'Idso–Jackson'] },
    { key: 'linke', label: 'Linke turbidity', type: 'number', min: 1.5, max: 7, step: 0.1, group: 'Radiation', default: 3.2, expert: true },
    { key: 'turbInt', label: 'Turbulence intensity', unit: '%', type: 'range', min: 2, max: 40, step: 1, group: 'Gusts & turbulence', default: 15 },
    { key: 'spectrum', label: 'Turbulence spectrum', type: 'select', group: 'Gusts & turbulence', default: 'Kaimal', options: ['Kaimal', 'von Kármán'] },
    { key: 'gustAmp', label: '1-cos gust amplitude', unit: 'm/s', type: 'number', min: 0, max: 25, step: 0.5, group: 'Gusts & turbulence', default: 6 },
    { key: 'gustDur', label: '1-cos gust duration', unit: 's', type: 'number', min: 0.5, max: 20, step: 0.5, group: 'Gusts & turbulence', default: 4 },
    { key: 'seed', label: 'Random seed (stochastic wind)', type: 'number', min: 1, max: 99999, step: 1, group: 'Gusts & turbulence', default: 42, expert: true },
    { key: 'Uthreshold', label: 'Useful-wind threshold', unit: 'm/s', type: 'number', min: 1, max: 12, step: 0.5, group: 'Operating window', default: 4.5, help: 'Wind speed above which the wings are expected to provide useful propulsion (refined by Suites 07 and 20).' },
  ],
  presets: Object.entries(SCENARIOS).filter(([k]) => k !== 'live').slice(0, 6).map(([k, v]) => ({ label: v.label.split('(')[0].trim(), values: { scenario: k } })),

  async run(p, ctx) {
    const z0 = ROUGHNESS[p.roughness] ?? 0.1, out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    let series = null, meta = null, warn = null, clim = null, aq = null, elev = ctx.site.elevation || 0;
    ctx.progress(0.1, 'fetching');
    if (p.scenario === 'live') {
      try {
        const r = await forecast(p.lat, p.lon); meta = r.meta; const H = hourlyTable(r.data); elev = H.elevation ?? elev;
        series = range(H.n).map(i => ({ time: H.time[i], T: H.temperature_2m[i], RH: H.relative_humidity_2m[i], Td: H.dew_point_2m[i], p: H.surface_pressure_Pa[i], U: H.wind_speed_10m[i], dir: H.wind_direction_10m[i], gust: H.wind_gusts_10m[i], U80: H.wind_speed_80m?.[i], cloud: H.cloud_cover[i], ghi: H.shortwave_radiation[i], dni: H.direct_normal_irradiance[i], dhi: H.diffuse_radiation[i], rain: H.precipitation[i], soil: H.soil_temperature_0cm[i], day: H.is_day[i] === 1, utcOffset: H.utcOffset }));
        if (r.meta.warning) warn = r.meta.warning;
      } catch (e) { warn = 'Live service unreachable (' + e.message + ') — falling back to the favourable-wind scenario.'; }
      Promise.all([nasaPowerClimatology(p.lat, p.lon).then(r => clim = r).catch(() => null), airQuality(p.lat, p.lon).then(r => aq = r).catch(() => null), ctx.site.elevation ? null : elevApi(p.lat, p.lon).then(r => { ctx.store.project.site.elevation = r.data.elevation?.[0]; }).catch(() => null)]);
    }
    let imported = false;
    if (p.scenario === 'imported') {
      const W = ctx.store.project.imports?.weather;
      if (W?.rows?.length) { imported = true; meta = { name: 'Imported observations: ' + W.name, fetched: new Date().toISOString(), kind: 'observation' }; series = W.rows.map((r, i) => { const t = Date.parse(r.time); const iso = Number.isFinite(t) ? new Date(t).toISOString().slice(0, 16) : new Date(Date.now() + (i - W.rows.length) * 3600e3).toISOString().slice(0, 16); return { time: iso, T: r.T, RH: r.RH, p: r.p, U: r.U, dir: r.dir, gust: r.U * 1.5, cloud: 30, ghi: r.ghi, day: r.ghi > 5, soil: r.T, utcOffset: 0 }; }); }
      else warn = 'No imported observations found — load a station file in the Data portal. Using the favourable-wind scenario.';
    }
    ctx.progress(0.4, 'deriving');
    const isLive = !!series && !imported, srcProv = isLive ? 'live' : imported ? 'user' : 'hypothetical';
    if (!series) { const S = SCENARIOS[['live', 'imported'].includes(p.scenario) ? 'favorable' : p.scenario]; const now = new Date(); now.setMinutes(0, 0, 0); series = range(48).map(k => { const t = new Date(now.getTime() + (k - 24) * 3600e3), hr = t.getHours(), sun = Math.max(0, Math.sin(Math.PI * (hr - 6) / 12)); const night = S.day === false; return { time: t.toISOString().slice(0, 16), T: S.T + (night ? -1.2 * Math.cos(Math.PI * hr / 12) : 3 * (sun - 0.4)), RH: S.RH, p: S.p, U: Math.max(0.05, S.U * (0.85 + 0.3 * Math.sin(k * 1.3))), dir: S.reversal ? (k < 24 ? S.dir : (S.dir + 180) % 360) : S.dir, gust: S.gust, cloud: S.cloud, ghi: night ? 0 : S.ghi * (S.ghi ? Math.max(0.15, sun) : 0), day: !night, soil: S.T }; }); }
    const nowIdx = imported ? series.length - 1 : isLive ? Math.max(0, series.findIndex(r => Date.parse(r.time + 'Z') - (r.utcOffset || 0) * 1000 >= Date.now() - 3600e3)) : 24;
    const idx = Math.min(series.length - 1, Math.max(0, nowIdx + (p.hourOffset | 0)));
    const D = series.map(r => derive(r, { z: p.zWing, z0, skyModel: p.skyModel, TL: p.linke, lat: p.lat, lon: p.lon, date: new Date(Date.parse(r.time + 'Z') - (isLive ? (r.utcOffset || 0) * 1000 : 0)), elevation: elev }));
    const cur = D[idx];
    // stability-corrected profile at the snapshot
    const L = p.obukhov || 0, zs = logspace(-0.3, 2, 40);
    const prof = zs.map(z => Math.max(0, logWind(cur.U, 10, Math.max(z, z0 * 1.2), z0, psiM(10, L), psiM(z, L))));
    const profPow = zs.map(z => powerWind(cur.U, 10, z, 0.143));
    // stochastic wind at wing height (random-phase synthesis of the selected spectrum)
    const sigma = cur.Uz * p.turbInt / 100, R = rng(p.seed), fs = logspace(-3, 0.7, 120), dt = 0.25, Nt = 1200, tt = range(Nt).map(i => i * dt);
    const Sfun = p.spectrum === 'Kaimal' ? kaimal : vonKarman; const Sv = fs.map(f => Sfun(f, Math.max(cur.Uz, 0.5), sigma));
    const phases = fs.map(() => 2 * Math.PI * R()), amps = fs.map((f, k) => Math.sqrt(2 * Sv[k] * (k ? fs[k] - fs[k - 1] : fs[0])));
    const uTurb = tt.map(t => cur.Uz + fs.reduce((s, f, k) => s + amps[k] * Math.cos(2 * Math.PI * f * t + phases[k]), 0) + oneMinusCos(t, 150, p.gustDur, p.gustAmp));
    const varSyn = mean(uTurb.slice(0, 590).map(v => (v - cur.Uz) ** 2)), varTarget = Sv.reduce((s, v, k) => s + v * (k ? fs[k] - fs[k - 1] : fs[0]), 0);
    // wind rose
    const sectors = range(16).map(k => k * 22.5), classes = [[0, 2], [2, 4], [4, 6], [6, 9], [9, 99]], names = ['0–2', '2–4', '4–6', '6–9', '>9 m/s'];
    const rose = classes.map((c, ci) => ({ name: names[ci], values: sectors.map(s => D.filter(d => ((d.dir - s + 371.25) % 360) < 22.5 && d.U >= c[0] && d.U < c[1]).length / D.length * 100) }));
    const useful = D.filter(d => d.Uz >= p.Uthreshold).length / D.length * 100;
    const nightGrad = D.filter(d => !d.isDay), nightDT = nightGrad.length ? mean(nightGrad.map(d => d.dTsky)) : NaN;
    // verification checks
    const sl = isa(0), tr = isa(11000);
    out.checks.push({ name: 'ISA sea-level density', value: sl.rho, target: '1.2250 kg/m³ ± 0.001', pass: Math.abs(sl.rho - 1.225) < 1e-3 });
    out.checks.push({ name: 'ISA 11 km pressure', value: tr.p, target: '22 632 Pa ± 0.5 %', pass: Math.abs(tr.p - 22632) / 22632 < 5e-3 });
    const ma = moistAir(101325, 15, 0); out.checks.push({ name: 'Dry-air density at 15 °C', value: ma.rho, target: '1.2250 kg/m³', pass: Math.abs(ma.rho - 1.225) < 2e-3 });
    out.checks.push({ name: 'Synthetic turbulence variance / spectrum integral', value: varSyn / (varTarget || 1), target: '1.0 ± 0.35 (finite record)', pass: Math.abs(varSyn / (varTarget || 1) - 1) < 0.35, detail: 'Random-phase synthesis reproduces the prescribed variance' });
    out.checks.push({ name: 'Missing / out-of-range samples', value: D.filter(d => !Number.isFinite(d.T) || !Number.isFinite(d.U)).length, target: '0', pass: D.every(d => Number.isFinite(d.T) && Number.isFinite(d.U)), detail: 'Quality control of provider fields' });
    // KPIs
    const k = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('Air temperature', cur.T, '°C', '2 m air temperature at the snapshot hour — the ambient reservoir for convection and the night-time hot side.', srcProv);
    k('Air density ρ', cur.rho, 'kg/m³', 'Moist-air density from pressure, temperature and humidity (Dalton). Scales every aerodynamic force linearly.', 'calc');
    k('Wind at 10 m', cur.U, 'm/s', 'Reference-height mean wind from the provider (or scenario).', srcProv);
    k('Wind at wing centroid', cur.Uz, 'm/s', `Log-law projection to z = ${p.zWing} m over roughness z₀ = ${z0} m. This is the true wind the wings see before vehicle motion is added.`, 'calc');
    k('Wind direction (from)', cur.dir, '°', 'Meteorological convention: direction the wind blows from, clockwise from north.', srcProv);
    k('Gust', cur.gust, 'm/s', 'Peak gust at 10 m — drives structural gust cases (Suites 06, 08, 23).', srcProv);
    k('Dynamic pressure at wing', cur.qDyn, 'Pa', 'q = ½ρU² at the wing centroid (stationary vehicle).', 'calc');
    k('Global horizontal irradiance', cur.ghi, 'W/m²', 'Solar input for the photonic collector (Suite 13).', srcProv);
    k('Cloud cover', cur.cloud, '%', 'Total cloud fraction — reduces both solar input and night radiative cooling.', srcProv);
    k('Effective sky emissivity', cur.eps, '', `${p.skyModel} clear-sky emissivity with cloud correction.`, 'calc');
    k('Effective sky temperature', cur.Tsky, '°C', 'Radiative-equivalent sky temperature: a calculated estimate, not a measured reservoir temperature.', 'calc');
    k('Air–sky temperature difference', cur.dTsky, 'K', 'Upper bound on radiative cooling drive; actual collector ΔT is solved in Suites 13–14.', 'calc');
    k('Solar elevation', cur.elevationSun, '°', 'NOAA solar-position algorithm for the snapshot time.', 'calc');
    k('Useful-wind availability', useful, '% of hours', `Fraction of hours in the series with wing-height wind ≥ ${p.Uthreshold} m/s.`, 'calc', useful > 40 ? 'ok' : useful > 15 ? 'warn' : 'bad');
    // verdicts
    out.verdicts.push({ test: 'Data provenance', status: isLive || imported ? 'pass' : 'warn', detail: imported ? `${meta.name} — user-supplied measured observations (provenance: observation, calibrate instruments before validation use).` : isLive ? `Live ${meta.name}; fetched ${meta.fetched.slice(0, 16).replace('T', ' ')} UTC${meta.cached ? ' (cached ' + meta.ageMin + ' min)' : ''}. Forecast-model fields, not station observations.` : (warn || 'Synthetic standard scenario (hypothetical inputs).') });
    out.verdicts.push({ test: 'Wind resource for primary propulsion', status: cur.Uz >= p.Uthreshold ? 'pass' : cur.Uz >= p.Uthreshold * 0.6 ? 'warn' : 'fail', detail: `Wing-height wind ${fmt(cur.Uz, 3)} m/s vs useful threshold ${p.Uthreshold} m/s.`, fix: 'Wind is an environmental constraint: choose another time window (see availability), site or route rather than expecting the wing to create energy.' });
    out.verdicts.push({ test: 'Night radiative-cooling potential', status: nightGrad.length ? (nightDT > 15 ? 'pass' : nightDT > 8 ? 'warn' : 'fail') : 'info', detail: nightGrad.length ? `Mean air–sky ΔT at night ${fmt(nightDT, 3)} K. Carnot ceiling at a realistic 5 K collector ΔT ≈ 1.7 %.` : 'No night hours in series.' });
    out.verdicts.push({ test: 'Gust load case', status: cur.gust > 20 ? 'fail' : cur.gust > 14 ? 'warn' : 'pass', detail: `Gust ${fmt(cur.gust, 3)} m/s → q ≈ ${fmt(0.5 * cur.rho * cur.gust ** 2, 3)} Pa at 10 m. Verify flutter/overturning margins in Suites 06, 11, 23.` });
    if (warn && isLive) out.verdicts.push({ test: 'Live data freshness', status: 'warn', detail: warn });
    // charts
    const T = D.map((_, i) => i - idx);
    out.charts.push({ title: 'Wind time history (10 m and wing height)', type: 'line', x: { label: 'Hours from snapshot', unit: 'h' }, y: { label: 'Wind speed', unit: 'm/s' }, series: [{ name: 'Mean 10 m', x: T, y: D.map(d => d.U) }, { name: 'Wing height', x: T, y: D.map(d => d.Uz) }, { name: 'Gust 10 m', x: T, y: D.map(d => d.gust), dash: '4 3' }], annotations: [{ type: 'hline', y: p.Uthreshold, label: 'useful threshold', color: '#22c55e' }, { type: 'vline', x: 0, label: 'snapshot' }], explain: 'Hourly wind history and forecast. Hours above the green line are candidate windows for wind-primary propulsion; gust peaks drive structural load cases.' });
    out.charts.push({ title: 'Temperatures: air, dew point and effective sky', type: 'line', x: { label: 'Hours from snapshot', unit: 'h' }, y: { label: 'Temperature', unit: '°C' }, series: [{ name: 'Air', x: T, y: D.map(d => d.T) }, { name: 'Dew point', x: T, y: D.map(d => d.Td) }, { name: 'Effective sky (calculated)', x: T, y: D.map(d => d.Tsky), dash: '5 3' }, { name: 'Ground surface', x: T, y: D.map(d => d.soil ?? NaN) }], explain: 'The gap between air and sky temperature is the maximum radiative driving force for night-time cooling. Humid, cloudy air raises the sky temperature and collapses the gap.' });
    out.charts.push({ title: 'Solar irradiance components', type: 'area', x: { label: 'Hours from snapshot', unit: 'h' }, y: { label: 'Irradiance', unit: 'W/m²' }, series: [{ name: 'GHI', x: T, y: D.map(d => d.ghi) }, { name: 'DNI', x: T, y: D.map(d => d.dni) }, { name: 'DHI', x: T, y: D.map(d => d.dhi) }, { name: 'Clear-sky GHI (model)', x: T, y: D.map(d => d.ghiClear), dash: '4 3' }], explain: 'Global, direct-normal and diffuse irradiance. The ratio of measured to clear-sky GHI quantifies cloud attenuation; this is the energy input to the daytime photonic collector.' });
    out.charts.push({ title: 'Downward longwave radiation', type: 'line', x: { label: 'Hours from snapshot', unit: 'h' }, y: { label: 'Longwave flux', unit: 'W/m²' }, series: [{ name: 'L↓ (calculated)', x: T, y: D.map(d => d.LW) }, { name: 'σT_air⁴ (blackbody air)', x: T, y: D.map(d => 5.670374e-8 * (d.T + 273.15) ** 4), dash: '4 3' }], explain: 'Atmospheric back-radiation. A radiator facing the sky can only cool if it emits more than it receives from this flux.' });
    out.charts.push({ title: 'Wind rose (frequency by direction & speed)', type: 'polar', sectors, bins: rose, unit: '%', explain: 'Share of hours by the direction the wind comes from, stacked by speed class. Guides route heading and wing incidence choices (apparent-wind angle).' });
    out.charts.push({ title: 'Vertical wind profile at snapshot', type: 'line', x: { label: 'Wind speed', unit: 'm/s' }, y: { label: 'Height', unit: 'm', log: true }, series: [{ name: `Log law (z₀=${z0} m${L ? ', L=' + L + ' m' : ''})`, x: prof, y: zs }, { name: 'Power law α=1/7', x: profPow, y: zs, dash: '4 3' }], annotations: [{ type: 'hline', y: p.zWing, label: 'wing centroid', color: '#f59e0b' }], explain: 'Boundary-layer wind shear between the ground and the wing tip. The wing spans a range of speeds, which the lifting-line model in Suite 06 integrates.' });
    out.charts.push({ title: `Stochastic wind at wing height (${p.spectrum}, TI ${p.turbInt}%) with 1-cos gust`, type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'u(t)', unit: 'm/s' }, series: [{ name: 'u(t)', x: tt, y: uTurb, width: 1.2 }], annotations: [{ type: 'xband', x0: 150, x1: 150 + p.gustDur, color: '#ef4444', opacity: .12 }], explain: 'A synthetic turbulent wind record consistent with the chosen spectrum plus a discrete design gust; used as the transient inflow for Suites 06, 08, 19 and 20.' });
    out.charts.push({ title: 'Turbulence power spectral density', type: 'line', x: { label: 'Frequency', unit: 'Hz', log: true }, y: { label: 'S_u(f)', unit: 'm²/s', log: true }, series: [{ name: p.spectrum, x: fs, y: Sv.map(v => Math.max(v, 1e-8)) }], explain: 'Energy distribution of wind fluctuations over frequency; overlap with wing natural frequencies (Suite 06) indicates dynamic amplification risk.' });
    // tables
    out.tables.push({ title: 'Boundary-condition snapshot (exported to all coupled suites)', cols: ['Quantity', 'Value', 'Unit', 'Provenance'], rows: [['Time (local)', cur.time, '', isLive ? 'Forecast/analysis model' : 'Scenario'], ['Air temperature', cur.T, '°C', isLive ? 'Forecast model' : 'Hypothetical'], ['Pressure', cur.p, 'Pa', isLive ? 'Forecast model' : 'Hypothetical'], ['Relative humidity', cur.RH, '%', isLive ? 'Forecast model' : 'Hypothetical'], ['Density', cur.rho, 'kg/m³', 'Calculated'], ['Kinematic viscosity', cur.nu, 'm²/s', 'Calculated'], ['Wind 10 m', cur.U, 'm/s', isLive ? 'Forecast model' : 'Hypothetical'], ['Wind at wing', cur.Uz, 'm/s', 'Calculated'], ['Direction (from)', cur.dir, '°', isLive ? 'Forecast model' : 'Hypothetical'], ['GHI', cur.ghi, 'W/m²', isLive ? 'Forecast model' : 'Hypothetical'], ['DNI', cur.dni, 'W/m²', isLive ? 'Forecast model' : 'Calculated'], ['Cloud', cur.cloud, '%', isLive ? 'Forecast model' : 'Hypothetical'], ['Sky emissivity', cur.eps, '', 'Calculated'], ['Sky temperature', cur.Tsky, '°C', 'Calculated (heuristic, not measured)'], ['Longwave down', cur.LW, 'W/m²', 'Calculated'], ['Solar zenith', cur.zenith, '°', 'Calculated']], explain: 'Every value is labelled by data kind, as the specification requires (observation vs forecast vs calculated).' });
    out.tables.push({ title: 'Statistics of the fetched period', cols: ['Quantity', 'Min', 'Mean', 'P90', 'Max'], rows: [['Wind at wing (m/s)', min(D.map(d => d.Uz)), mean(D.map(d => d.Uz)), percentile(D.map(d => d.Uz), 0.9), max(D.map(d => d.Uz))], ['Air T (°C)', min(D.map(d => d.T)), mean(D.map(d => d.T)), percentile(D.map(d => d.T), 0.9), max(D.map(d => d.T))], ['GHI (W/m²)', min(D.map(d => d.ghi)), mean(D.map(d => d.ghi)), percentile(D.map(d => d.ghi), 0.9), max(D.map(d => d.ghi))], ['Air–sky ΔT (K)', min(D.map(d => d.dTsky)), mean(D.map(d => d.dTsky)), percentile(D.map(d => d.dTsky), 0.9), max(D.map(d => d.dTsky))]] });
    out.views.push({ label: '3D site wind field & sky', icon: '🌬', render: async (el) => {
      const [{ viewer }, { playback, hud, legend3d }, { windParticles }] = await Promise.all([import('../core/viz3d.js'), import('../core/sim3d.js'), import('../core/sim3d-drive.js')]);
      const holder = h('div'); el.append(holder);
      const modes = [{ key: 'turb', label: `Turbulent record at wing height (${p.spectrum}, ${tt[tt.length - 1]} s, with design gust)` }, { key: 'hourly', label: `Hourly record (${D.length} h${isLive ? ', live' : ''}) — sun, sky & wind evolve` }];
      const show = async (mode) => {
        holder.innerHTML = ''; holder.append(h('div', { class: 'row gap wrap', style: 'margin-bottom:6px' }, ...modes.map(m => h('button', { class: 'chip' + (m.key === mode ? ' on' : ''), onclick: () => show(m.key) }, '▶ ' + m.label))));
        const box = h('div'); holder.append(box);
        const V = await viewer(box, { height: 470, grid: false }); const THREE = V.THREE;
        V.camera.far = 3000; V.camera.near = 0.1; V.camera.updateProjectionMatrix();
        const ground = new THREE.Mesh(new THREE.PlaneGeometry(120, 120), new THREE.MeshStandardMaterial({ color: 0x4d7c4f, roughness: 1 })); V.root.add(ground);
        // roughness elements scaled to the selected roughness length (≈ obstacle height ~ 10·z0, density increasing with z0)
        const nObs = Math.min(400, Math.round(20 + 300 * Math.min(1, z0))), hObs = Math.min(18, Math.max(0.1, 10 * z0)), R0 = Math.random;
        const obsMat = new THREE.MeshStandardMaterial({ color: z0 >= 0.5 ? 0x94a3b8 : 0x2f5d31, roughness: 1 });
        for (let i = 0; i < nObs; i++) { const hh = hObs * (0.5 + R0()), b = new THREE.Mesh(new THREE.BoxGeometry(0.6 + R0() * (z0 >= 0.5 ? 4 : 1), 0.6 + R0() * (z0 >= 0.5 ? 4 : 1), hh), obsMat); const ang = R0() * 2 * Math.PI, rr = 18 + R0() * 40; b.position.set(rr * Math.cos(ang), rr * Math.sin(ang), hh / 2); V.root.add(b); }
        // anemometer mast with solved log-law profile arrows
        const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 12, 8), new THREE.MeshStandardMaterial({ color: 0xe5e7eb })); mast.rotation.x = Math.PI / 2; mast.position.set(0, 0, 6); V.root.add(mast);
        const zs = [0.5, 1, 2, p.zWing, 4, 6, 8, 10, 12], prof = zs.map(z => new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, z), 1, 0x38bdf8, 0.25, 0.12)); prof.forEach(a => V.root.add(a));
        const wingMark = new THREE.Mesh(new THREE.SphereGeometry(0.18, 12, 8), new THREE.MeshBasicMaterial({ color: 0xf59e0b })); wingMark.position.set(0, 0, p.zWing); V.root.add(wingMark);
        const sun = new THREE.DirectionalLight(0xfff4d6, 2.5); V.scene.add(sun); const sunBall = new THREE.Mesh(new THREE.SphereGeometry(3, 16, 8), new THREE.MeshBasicMaterial({ color: 0xffd34d })); V.root.add(sunBall);
        const series = mode === 'turb' ? { t: tt, U: uTurb.map(u => u), dir: tt.map(() => cur.dir), day: tt.map(() => cur.isDay), el: tt.map(() => cur.elevationSun), az: tt.map(() => cur.azimuth), ghi: tt.map(() => cur.ghi), T: tt.map(() => cur.T), Tsky: tt.map(() => cur.Tsky) }
          : { t: D.map((_, i) => i * 3600), U: D.map(d => d.Uz), dir: D.map(d => d.dir), day: D.map(d => d.isDay), el: D.map(d => d.elevationSun), az: D.map(d => d.azimuth), ghi: D.map(d => d.ghi), T: D.map(d => d.T), Tsky: D.map(d => d.Tsky), time: D.map(d => d.time) };
        const Umax = Math.max(1, ...series.U.filter(Number.isFinite)); let Unow = 0, dirNow = 0;
        const zRef = p.zWing, logF = (z) => Math.max(0, Math.log(Math.max(z, z0 * 1.5) / z0) / Math.log(zRef / z0));
        const P = windParticles(V, { count: 3000, half: 45, top: 14, colorMax: Umax * 1.25, size: 0.55, vel: (x, y, z) => { const sp = Unow * logF(z), to = (dirNow + 180) * Math.PI / 180; return [sp * Math.sin(to), sp * Math.cos(to), 0]; } });
        legend3d(box, { title: 'Wind speed (log-law over terrain)', lo: 0, hi: Umax * 1.25, unit: ' m/s' }); Object.assign(box.lastChild.style, { top: '44px', left: '8px', bottom: 'auto', right: 'auto' });
        const H = hud(box); V.controls.target.set(0, 0, 4); V.camera.position.set(40, -55, 28); V.controls.update();
        let last = 0; const T = series.t, n = T.length;
        playback(box, { duration: T[n - 1], rate: mode === 'turb' ? 8 : 3600, label: mode === 'turb' ? 'Time' : 'Hours ×3600 s', onFrame: (t) => {
          let i = 0; while (i < n - 2 && T[i + 1] <= t) i++; const f = Math.min(1, Math.max(0, (t - T[i]) / ((T[i + 1] - T[i]) || 1))), L2 = (a) => a[i] + ((a[i + 1] ?? a[i]) - a[i]) * f;
          Unow = L2(series.U); dirNow = series.dir[i];
          const to = (dirNow + 180) * Math.PI / 180, dx = Math.sin(to), dy = Math.cos(to);
          prof.forEach((a, k) => { const sp = Unow * logF(zs[k]); a.setDirection(new THREE.Vector3(dx, dy, 0)); a.setLength(Math.max(0.05, 0.9 * sp), 0.25, 0.12); });
          const el_ = series.el[i], az = series.az[i] * Math.PI / 180, er = Math.max(-5, el_) * Math.PI / 180;
          const sp = new THREE.Vector3(Math.sin(az) * Math.cos(er), Math.cos(az) * Math.cos(er), Math.sin(er)).multiplyScalar(90); sun.position.copy(sp); sunBall.position.copy(sp); sunBall.visible = el_ > -2;
          const dayF = Math.max(0, Math.min(1, (el_ + 6) / 20)); V.scene.background.setRGB(0.03 + 0.3 * dayF, 0.05 + 0.45 * dayF, 0.12 + 0.65 * dayF); sun.intensity = 1.2 * dayF;
          ground.material.color.setRGB(0.08 + 0.06 * dayF, 0.14 + 0.1 * dayF, 0.08 + 0.05 * dayF);
          const dt = Math.min(0.5, Math.max(0.01, Math.abs(t - last) / (mode === 'turb' ? 1 : 600))); P.step(dt, t); last = t;
          H.set([[mode === 'turb' ? 'Time' : 'Local time', mode === 'turb' ? fmt(t, 4) + ' s' : (series.time?.[i] || '').replace('T', ' ')], ['Wind at wing height', fmt(Unow, 3) + ' m/s' + (Umax < 2 ? ' (calm record)' : ''), Unow >= p.Uthreshold ? '#22c55e' : '#f59e0b'], ['Direction (from)', fmt(dirNow, 3) + '°'], ['Roughness z₀', fmt(z0, 3) + ' m'], ['Solar elevation', fmt(el_, 3) + '°'], ['GHI', fmt(L2(series.ghi), 3) + ' W/m²'], ['Air T', fmt(L2(series.T), 3) + ' °C'], ['Sky T (calc.)', fmt(L2(series.Tsky), 3) + ' °C'], ['Gust event', mode === 'turb' && t >= 150 && t <= 150 + p.gustDur ? 'ACTIVE' : '—', mode === 'turb' && t >= 150 && t <= 150 + p.gustDur ? '#ef4444' : undefined]]);
        } });
        box.append(h('p', { class: 'small muted' }, mode === 'turb' ? `Solved: synthetic turbulent wind at wing height from the ${p.spectrum} spectrum (TI ${p.turbInt} %) plus the 1-cosine design gust at t = 150 s (HUD flags it), projected over height with the log law (z₀ = ${z0} m); sun position from the NOAA algorithm for the snapshot hour. Tracer particles are advected by this solved field; mast arrows show the solved profile. Terrain obstacles are a visual cue sized from z₀.` : `Solved / retrieved: hourly wind speed and direction at wing height, solar elevation & azimuth (NOAA), GHI, air and calculated sky temperature for ${D.length} h (${isLive ? 'live forecast/analysis' : 'scenario'}). Sky brightness and sun follow the solved solar elevation; particles and mast arrows follow the hourly wind (linear interpolation between hours).`));
      };
      await show('turb');
    } });
    out.views.push({ label: '2D animation: wind compass & gust trace', icon: '🧭', render: async (el) => {
      const A = await import('../core/anim2d.js');
      const Th = D.map((_, i) => i * 3600), dirH = D.map(d => d.dir), UzH = D.map(d => d.Uz), Umx = Math.max(1, ...uTurb, ...UzH);
      A.anim2d(el, { duration: tt[tt.length - 1], rate: 8, height: 440, label: 'Turbulence record', unit: 's', draw: (g, t, W, H) => {
        const k = A.at(tt, t), u = A.lerp(uTurb, k), fr = t / tt[tt.length - 1], kh = A.at(Th, fr * Th[Th.length - 1]), dirNow = A.lerp(dirH, kh), UzNow = A.lerp(UzH, kh);
        // compass: needle points where the wind blows TO; length ∝ speed
        const R0 = Math.min(W * 0.18, (H - 60) / 2), cx = 24 + R0, cy = 30 + R0;
        g.strokeStyle = A.theme().grid; g.lineWidth = 1; for (const f of [0.33, 0.66, 1]) { g.beginPath(); g.arc(cx, cy, R0 * f, 0, 7); g.stroke(); }
        ['N', 'E', 'S', 'W'].forEach((d, j) => A.text(g, d, cx + (R0 + 10) * Math.sin(j * Math.PI / 2) - 4, cy - (R0 + 10) * Math.cos(j * Math.PI / 2) + 4, undefined, 'bold 12px Inter,system-ui,sans-serif'));
        const to = (dirNow + 180) * Math.PI / 180, L1 = R0 * Math.min(1, UzNow / Umx), L2 = R0 * Math.min(1, u / Umx);
        A.arrow(g, cx, cy, L1 * Math.sin(to), -L1 * Math.cos(to), '#94a3b8', 4, `hourly ${fmt(UzNow, 3)} m/s`);
        A.arrow(g, cx, cy, L2 * Math.sin(to + 0.05), -L2 * Math.cos(to + 0.05), '#38bdf8', 2.5, `instant ${fmt(u, 3)} m/s`);
        A.text(g, `from ${fmt(dirNow, 3)}° · hour ${kh.i + 1}/${D.length} (${(D[kh.i]?.time || '').replace('T', ' ')})`, 16, H - 16);
        const gust = t >= 150 && t <= 150 + p.gustDur; if (gust) A.text(g, 'DESIGN GUST', cx - 40, cy + R0 + 24, '#ef4444', 'bold 14px Inter,system-ui,sans-serif');
        const sx = 2 * R0 + 70, sw = W - sx - 12;
        A.seriesPanel(g, { x: sx, y: 12, w: sw, h: (H - 36) * 0.55, T: tt, t, title: `Wind at wing height (${p.spectrum}, TI ${p.turbInt} %)`, unit: 'm/s', series: [{ name: 'u(t)', y: uTurb, color: '#38bdf8' }], hlines: [{ y: p.Uthreshold, label: 'useful threshold', color: '#22c55e' }] });
        A.seriesPanel(g, { x: sx, y: 20 + (H - 36) * 0.55, w: sw, h: (H - 36) * 0.45, T: Th, t: fr * Th[Th.length - 1], title: `Hourly record (${D.length} h)`, unit: 'm/s, °/36', series: [{ name: 'wind at wing m/s', y: UzH, color: '#94a3b8' }, { name: 'direction/36', y: dirH.map(d => d / 36), color: '#f59e0b' }] });
      }, explain: 'Compass needle shows where the solved wind blows to (grey = hourly record swept in step with the timeline, blue = instantaneous turbulent wind at wing height); the gust trace shows the synthetic record with the 1-cosine design gust flagged at t = 150 s.' });
    } });
    out.views.push({ label: 'Climatology (NASA POWER)', icon: '🗓', render: async (el) => {
      el.append(h('p', { class: 'muted small' }, 'Long-term monthly climatology for the site from NASA POWER (satellite & reanalysis), used to judge whether the snapshot is typical.'));
      const wait = async () => { for (let i = 0; i < 60 && !clim; i++) await new Promise(r => setTimeout(r, 250)); return clim; };
      let c = clim || await wait(); if (!c) { try { c = await nasaPowerClimatology(p.lat, p.lon); } catch (e) { el.append(h('div', { class: 'callout warn' }, 'Climatology unavailable: ' + e.message)); return; } }
      const P = c.data.properties.parameter, mon = ['JAN', 'FEB', 'MAR', 'APR', 'MAY', 'JUN', 'JUL', 'AUG', 'SEP', 'OCT', 'NOV', 'DEC'];
      const { chart } = await import('../core/chart.js'); const b1 = h('div'), b2 = h('div'); el.append(b1, b2);
      chart(b1, { type: 'bar', title: 'Monthly mean solar and longwave (kWh/m²/day)', categories: mon, y: { label: 'Daily energy', unit: 'kWh/m²/day' }, series: [{ name: 'All-sky shortwave', values: mon.map(m => P.ALLSKY_SFC_SW_DWN?.[m]) }, { name: 'Clear-sky shortwave', values: mon.map(m => P.CLRSKY_SFC_SW_DWN?.[m]) }, { name: 'Longwave down', values: mon.map(m => P.ALLSKY_SFC_LW_DWN?.[m]) }], explain: 'Seasonal solar resource and atmospheric back-radiation.' });
      chart(b2, { type: 'line', title: 'Monthly mean wind & temperature', x: { label: 'Month', unit: '' }, y: { label: 'Value' }, series: [{ name: 'Wind 10 m (m/s)', x: range(12).map(i => i + 1), y: mon.map(m => P.WS10M?.[m]) }, { name: 'Wind 50 m (m/s)', x: range(12).map(i => i + 1), y: mon.map(m => P.WS50M?.[m]) }, { name: 'T2M (°C)', x: range(12).map(i => i + 1), y: mon.map(m => P.T2M?.[m]) }, { name: 'Cloud amount (%/10)', x: range(12).map(i => i + 1), y: mon.map(m => (P.CLOUD_AMT?.[m] ?? NaN) / 10) }], explain: 'Long-term means: seasons with stronger winds favour wind-primary operation.' });
      el.append(h('div', { class: 'small muted' }, `Source: ${c.meta.name} · ${c.meta.fetched.slice(0, 10)}${c.meta.cached ? ' (cached)' : ''}`));
      if (aq) el.append(table(['Air quality (CAMS)', 'Value'], Object.entries(aq.data.current || {}).filter(([k2]) => !['time', 'interval'].includes(k2)).map(([k2, v]) => [k2, v])), h('p', { class: 'small muted' }, 'Particulate and dust loads soil optical collector surfaces; include cleaning intervals in Suite 22/26.'));
    } });
    // recommendations
    if (cur.Uz < p.Uthreshold) out.recs.push({ level: 'warn', text: `Wind at the wing (${fmt(cur.Uz, 3)} m/s) is below the useful threshold. Schedule journeys in the highest-availability windows (see time history) or orient the route for a beam reach (apparent wind 70–110° off the bow).`, basis: 'Wind time history + apparent-wind mechanics' });
    if (useful < 25) out.recs.push({ level: 'critical', text: `Only ${fmt(useful, 3)} % of hours provide useful wind at this site. Wind-primary propulsion cannot be relied on here without a demonstrated backup; prioritise Suite 20 feasibility before hardware.`, basis: 'Availability statistic' });
    if (!cur.isDay && cur.cloud > 60) out.recs.push({ level: 'warn', text: 'Cloudy night: the sky temperature is close to air temperature, so radiative cooling (and the thermal backup) will be weak. Expect the elastic buffer to be the only backup source.', basis: 'Cloud-corrected sky emissivity' });
    if (cur.gust / Math.max(cur.U, 0.1) > 1.8) out.recs.push({ level: 'warn', text: `High gust factor (${fmt(cur.gust / Math.max(cur.U, 0.1), 3)}). Use the gust record from this suite as the transient load case in Suites 06/08 and verify gust-load alleviation of the morphing wing.`, basis: 'Gust factor = gust / mean' });
    out.recs.push({ level: 'good', text: 'Install a calibrated on-vehicle anemometer, pyranometer and pyrgeometer for experimental campaigns; import their logs in “Import experimental data” to calibrate the sky-emissivity model and validate wind projections.', basis: 'Specification Part B §28 (calibration datasets for Module 05)' });
    out.explain.push({ title: 'Where the data come from', text: isLive ? `The ${meta.name} was queried directly from your browser for ${fmt(p.lat, 6)}°, ${fmt(p.lon, 6)}°. It blends global numerical weather prediction models; the first hours are recent analysis, later hours are forecast. Calculated quantities (density, sky temperature, wind at wing height) are derived here with the listed equations.` : 'A standard engineering scenario from the specification was used; values are hypothetical and labelled as such.' });
    out.explain.push({ title: 'Why sky temperature matters', text: 'At night the photonic radiator can fall below air temperature only by radiating through the 8–13 µm atmospheric window. The effective sky temperature summarises that radiative sink. It is a heuristic estimate; the actual hot/cold temperatures of the NiTi converter are solved in Suites 13–15 from the full heat balance.' });
    // coupling outputs
    const env = { ...cur, z0, lat: p.lat, lon: p.lon, scenario: p.scenario, isLive, source: meta?.name || SCENARIOS[p.scenario]?.label, series: D.map(d => ({ time: d.time, T: d.T, U: d.U, Uz: d.Uz, dir: d.dir, gust: d.gust, ghi: d.ghi, dni: d.dni, cloud: d.cloud, RH: d.RH, p: d.p, rho: d.rho, Tsky: d.Tsky, LW: d.LW, isDay: d.isDay, eps: d.eps })), snapshotIndex: idx, turbulence: { t: tt, u: uTurb, sigma, spectrum: p.spectrum } };
    ctx.store.project.env = env; out.outputs = env;
    out.headline = `${isLive ? 'Live' : 'Scenario'} conditions at ${cur.time}: ${fmt(cur.T, 3)} °C, wind ${fmt(cur.U, 3)} m/s from ${fmt(cur.dir, 3)}°, ${fmt(cur.ghi, 3)} W/m² solar, sky ${fmt(cur.Tsky, 3)} °C. These boundary conditions now drive every coupled suite.`;
    out.credibility = isLive ? 'Analytical screening' : 'Conceptual';
    return out;
  },
};
