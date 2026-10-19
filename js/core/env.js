// Environment model: standard operating scenarios + derived atmospheric quantities shared by all suites.
import { moistAir, solarPosition, clearSky, skyEmissivity, cloudyEmissivity, skyTemperature, longwaveDown, logWind, dewPoint, ROUGHNESS } from './physics.js';
import { KELVIN, range } from './num.js';

// Scenario definitions follow the specification's standard simulation scenarios (input/output register §11)
export const SCENARIOS = {
  live: { label: 'Live weather at project site', live: true },
  favorable: { label: 'Stationary, favourable wind (6 m/s crosswind)', T: 24, RH: 55, p: 101325, U: 6, dir: 90, gust: 8, cloud: 30, day: true, ghi: 550 },
  acceleration: { label: 'Wind-powered acceleration (8 m/s beam reach)', T: 22, RH: 55, p: 101325, U: 8, dir: 90, gust: 10.5, cloud: 30, day: true, ghi: 500 },
  crosswind: { label: 'Strong crosswind (14 m/s, gust 20 m/s)', T: 18, RH: 65, p: 100800, U: 14, dir: 90, gust: 20, cloud: 70, day: true, ghi: 250 },
  gust: { label: 'Sudden gust (5 → 15 m/s, 1-cosine 4 s)', T: 20, RH: 60, p: 101325, U: 5, dir: 90, gust: 15, cloud: 40, day: true, ghi: 450, gustEvent: { t0: 20, dur: 4, amp: 10 } },
  reversal: { label: 'Wind reversal (90° → 270° over 60 s)', T: 20, RH: 60, p: 101325, U: 7, dir: 90, gust: 9, cloud: 40, day: true, ghi: 400, reversal: true },
  sunnyCalm: { label: 'Calm sunny day (0.8 m/s, 850 W/m²)', T: 31, RH: 45, p: 101200, U: 0.8, dir: 200, gust: 2, cloud: 5, day: true, ghi: 850 },
  clearNight: { label: 'Calm clear night (0.5 m/s, 20 °C, clear sky)', T: 20, RH: 60, p: 101325, U: 0.5, dir: 200, gust: 1.2, cloud: 0, day: false, ghi: 0 },
  cloudyNight: { label: 'Calm cloudy night (0.5 m/s, 90 % cloud)', T: 20, RH: 85, p: 101000, U: 0.5, dir: 200, gust: 1.2, cloud: 90, day: false, ghi: 0 },
  worstCase: { label: 'Worst-case energy test (no wind, overcast night)', T: 15, RH: 92, p: 100900, U: 0.2, dir: 0, gust: 0.5, cloud: 100, day: false, ghi: 0 },
};

// Compute derived quantities for one environmental state
export function derive(s, { z = 2.2, zref = 10, z0 = 0.1, skyModel = 'Berdahl–Martin', TL = 3, lat = 0, lon = 0, date = new Date(), elevation = 0 } = {}) {
  const air = moistAir(s.p, s.T, s.RH), Td = s.Td ?? dewPoint(s.T, s.RH), TaK = s.T + KELVIN;
  const epsClr = (skyEmissivity[skyModel] || skyEmissivity['Berdahl–Martin'])(TaK, Td), eps = Math.min(1, cloudyEmissivity(epsClr, (s.cloud ?? 0) / 100));
  const Tsky = skyTemperature(TaK, eps), LW = s.lw ?? longwaveDown(TaK, eps);
  const sp = solarPosition(date, lat, lon), cs = clearSky(sp.zenith, elevation, TL);
  const Uz = Math.max(0, logWind(s.U, zref, Math.max(z, z0 * 1.5), z0));
  const ghi = s.ghi ?? (sp.elevation > 0 ? cs.ghi * (1 - 0.75 * ((s.cloud ?? 0) / 100) ** 3.4) : 0);
  return { ...s, Td, rho: air.rho, mu: air.mu, nu: air.nu, kAir: air.k, q: air.q, Tv: air.Tv, epsClr, eps, Tsky: Tsky - KELVIN, TskyK: Tsky, LW, dTsky: TaK - Tsky, zenith: sp.zenith, azimuth: sp.azimuth, elevationSun: sp.elevation, ghiClear: cs.ghi, ghi, dni: s.dni ?? cs.dni * (ghi / (cs.ghi || 1)), dhi: s.dhi ?? Math.max(0, ghi - (s.dni ?? cs.dni) * Math.cos(sp.zenith * Math.PI / 180)), Uz, qDyn: 0.5 * air.rho * Uz * Uz, isDay: s.day ?? sp.elevation > 0 };
}

// Synthetic 24-h time history for a scenario (diurnal shape) so time-dependent suites can run offline
export function scenarioSeries(key, { hours = 24, startHour = 0 } = {}) {
  const S = SCENARIOS[key] || SCENARIOS.favorable;
  return range(hours).map(k => {
    const hr = (startHour + k) % 24, sun = Math.max(0, Math.sin(Math.PI * (hr - 6) / 12));
    const dayFactor = S.day === false ? 0 : 1;
    return { hour: hr, T: S.T + (S.day === false ? -1.5 * Math.cos(Math.PI * hr / 12) : 4 * (sun - 0.5)), RH: S.RH, p: S.p, U: Math.max(0, S.U * (0.85 + 0.3 * Math.sin(k * 1.7))), dir: S.dir, gust: S.gust, cloud: S.cloud, ghi: dayFactor * S.ghi * sun / (S.ghi ? 1 : 1), day: dayFactor ? sun > 0 : false };
  });
}
export const roughnessOptions = Object.keys(ROUGHNESS);
export { ROUGHNESS };

// Pull current environment snapshot from the store or a scenario (used as coupled input by downstream suites)
export function currentEnv(store) {
  const e = store.project.env; if (e) return e;
  return derive(SCENARIOS.favorable, { lat: store.project.site.lat, lon: store.project.site.lon });
}
