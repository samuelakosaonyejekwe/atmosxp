// Decision-support advisor: merges suite-generated recommendations with live economic/sustainability
// context (grid carbon intensity, inflation, lending rates, FX) and fresh literature for best practice.
import { store } from './state.js';
import { siteContext, literature } from './live.js';
import { fmt } from './num.js';

let ctxCache = null, ctxT = 0;
export async function getSiteContext(force = false) {
  const s = store.project.site;
  if (!force && ctxCache && Date.now() - ctxT < 3600e3 && ctxCache.lat === s.lat && ctxCache.lon === s.lon) return ctxCache;
  const c = await siteContext(s.lat, s.lon); c.lat = s.lat; c.lon = s.lon; ctxCache = c; ctxT = Date.now(); return c;
}
export const cachedSiteContext = () => ctxCache;

// Generic sustainability/economics recommendations attached to every suite, conditioned on live context
export function contextRecs(suiteMeta, results, ctx) {
  const recs = [];
  if (!ctx) return recs;
  if (ctx.gridCarbon?.value) {
    const g = ctx.gridCarbon.value;
    recs.push({ level: 'sustain', text: `Fabrication electricity in ${ctx.country || 'this location'} emits ≈ ${fmt(g, 3)} gCO₂/kWh (${ctx.gridCarbon.year}). ${g > 400 ? 'Prefer low-energy processes (out-of-autoclave cure, near-net-shape forming) or source certified renewable power for autoclave and additive-manufacturing steps.' : 'A relatively clean grid favours local manufacture of energy-intensive parts (composite cure, metal additive manufacturing).'}`, basis: 'Our World in Data / Ember electricity carbon intensity (live)' });
  }
  if (ctx.inflation?.value !== undefined && suiteMeta.econ) {
    const inf = ctx.inflation.value; recs.push({ level: 'econ', text: `Consumer-price inflation is ${fmt(inf, 3)} % (${ctx.inflation.year}). ${inf > 10 ? 'Lock material prices with forward contracts and shorten procurement cycles; evaluate costs in real (inflation-adjusted) terms.' : 'Use a real discount rate (nominal minus inflation) when comparing life-cycle costs.'}`, basis: 'World Bank FP.CPI.TOTL.ZG (live)' });
  }
  if (ctx.lendingRate?.value !== undefined && suiteMeta.econ) recs.push({ level: 'econ', text: `Commercial lending rate ≈ ${fmt(ctx.lendingRate.value, 3)} % (${ctx.lendingRate.year}); use it as the upper bound of the nominal discount rate for prototype financing.`, basis: 'World Bank FR.INR.LEND (live)' });
  if (ctx.renewShare?.value !== undefined && suiteMeta.sustain) recs.push({ level: 'sustain', text: `Renewables supply ${fmt(ctx.renewShare.value, 3)} % of final energy consumption here (${ctx.renewShare.year}); include this in the life-cycle inventory of in-country manufacturing and maintenance.`, basis: 'World Bank EG.FEC.RNEW.ZS (live)' });
  return recs;
}

// Literature-backed "fresh best practice" pointers for a suite
export async function bestPractice(suiteMeta, n = 6) {
  if (!suiteMeta.literature) return { items: [], meta: null };
  try { const r = await literature(suiteMeta.literature, { perPage: n }); return { items: r.items, meta: r.meta }; } catch (e) { return { items: [], error: e.message }; }
}

// Universal guard-rail recommendations from verdicts (failures must be explained, never hidden)
export function verdictRecs(verdicts = []) {
  return verdicts.filter(v => (v.status || (v.pass ? 'pass' : 'fail')) === 'fail').map(v => ({ level: 'critical', text: `Failed: ${v.test}. ${v.fix || 'Revise the design variables that drive this check (see the sensitivity tab) before relying on this configuration.'}`, basis: v.detail }));
}
