// Suite 16 — Competing artificial-muscle and thermoresponsive actuator materials (matched-condition comparison).
import * as TH from '../core/thermo.js';
import { linspace, fmt, range, rng, percentile } from '../core/num.js';
import { MATERIALS } from '../core/materials.js';
import { evaluateSMA } from './s15-niti.js';

const CANDIDATES = MATERIALS.filter(m => m.uses.includes('actuator'));

function evalPolymer(m, q) {
  const d = q.t_mm / 1000; // fibre / film thickness
  // a coiled fibre touches a flat interface along a line: effective contact ≈ 15 % of the nominal conductance
  const tim = TH.lamellaTiming(m, { t: d, hc: q.hc * (m.cls === 'tcp' ? 0.15 : 1), Th: q.Th, Tc: q.Tc, Ttarget_h: q.Th - 0.1 * (q.Th - q.Tc), Ttarget_c: q.Tc + 0.1 * (q.Th - q.Tc) });
  const period = (tim.tHeat + tim.tCool) * (1 + (q.dwell ?? 0.1)), f = period > 0 && Number.isFinite(period) ? 1 / period : 0;
  const best = linspace(0.05, 1, 20).map(fr => { const c = TH.polymerCycle(m, { Th: q.Th, Tc: q.Tc, sLoad: (m.stressMax || 10) * fr, fractionReached: 0.8 }); return { c, fr }; }).sort((a, b) => b.c.w_J_kg - a.c.w_J_kg)[0];
  const life = m.cls === 'tcp' ? 1e6 * (0.5 / Math.max(0.05, best.fr)) ** 2 : 1e5;
  return { w_J_kg: best.c.w_J_kg, eta: best.c.eta, f, P_kg: best.c.w_J_kg * f, stress: best.c.sUsed, fatigue: life, stroke: best.c.strokeStrain, net: best.c.w_J_kg > 0.5 };
}
function evalSMA(m, q) {
  // each alloy at its own best recovery stress within its allowable (≤ 0.5 σ_y), bias stress 15 % of σ_H
  let best = null;
  for (const sH of linspace(20, Math.max(40, 0.5 * (m.sy || 400)), 14)) { const r = evaluateSMA(m, { ...q, sH, sL: 0.15 * sH }); if (!best || r.P_kg > best.r.P_kg || (best.r.P_kg === 0 && r.cyc.w_J_kg > best.r.cyc.w_J_kg)) best = { r, sH }; }
  const r = best.r;
  return { w_J_kg: r.cyc.w_J_kg, eta: r.cyc.eta, f: r.f, P_kg: r.P_kg, stress: best.sH, fatigue: r.fatigue, stroke: r.cyc.strokeStrain, net: r.cyc.w_J_kg > 0.5 && r.cyc.dXiHeat > 0.05 };
}
const evaluate = (m, q) => m.cls === 'sma' ? evalSMA(m, q) : evalPolymer(m, q);
function perturb(m, R, scale) { const u = (m.unc || 15) / 100 * scale; const o = { ...m }; for (const k of ['Ms', 'Mf', 'As', 'Af']) if (o[k] !== undefined) o[k] += R.normal(0, 3 * scale); for (const k of ['CM', 'CA', 'epsL', 'strainPerK', 'stressMax', 'EA', 'EM']) if (o[k] !== undefined) o[k] *= Math.max(0.2, 1 + R.normal(0, u)); return o; }

export default {
  description: 'Fair, matched-condition comparison of NiTi grades, Cu-Al-Ni, Fe-Mn-Si, twisted-and-coiled polymer muscles and liquid-crystal elastomers under identical thermal boundary conditions, geometry and cycle-time budget. Ranks work density, continuous power density, efficiency, durability, cost and embodied carbon per watt with Monte Carlo uncertainty — and refuses rankings made under mismatched conditions.',
  literature: 'twisted coiled polymer actuator heat engine comparison shape memory alloy work density',
  sustain: true, econ: true, credibility: 'Analytical screening',
  equations: ['SMA constitutive cycle and Clausius–Clapeyron transformation shift (as Suite 15)', 'Twisted-and-coiled polymer actuation: free coil strain ε₀ = (dε/dT)ΔT from anisotropic fibre thermal expansion and twist–length coupling, linear load line ε = ε₀(1 − σ/σ_block)', 'Helical spring mechanics for coil force–stroke at constant load', 'Transient heat conduction / lumped capacitance for heating–cooling time constants; effective thermal capacitance', 'Work density w = σΔε, power density P = w·f, efficiency η = w/q_in, Carnot bound', 'Monte Carlo propagation of property uncertainty (transformation temperatures, slopes, strains)'],
  methods: ['Common boundary conditions applied to every candidate (same T_h, T_c, thickness, interface conductance, dwell)', 'Per-material optimum load within its allowable stress (declared rule)', '40-sample Monte Carlo per candidate (seeded, reproducible) → 5th/50th/95th percentiles'],
  assumptions: ['Library properties are typical values with stated uncertainty', 'TCP fatigue scaled from reported >16 000 cycles at moderate load (estimate)', 'All actuators cycle through the same nominal interface conductance; coiled fibres achieve only line contact (15 % effective)'],
  ibc: { initial: ['Initial polymer/alloy temperature at T_c, preload, axial and torsional strain, creep state (virgin)', 'Thermal history and fatigue damage reset only by explicit designation'], boundary: ['Identical hot/cold reservoir contact (conductance h_c) and temperatures for every candidate', 'Axial constant load (optimum per material), fixed end attachments', 'Same total cycle-time budget and duration'] },
  cvv: { calibration: ['Contraction/extension vs temperature at several loads', 'Complete-cycle work under matched conditions', 'Creep and fatigue curves'], verification: ['Common-condition enforcement check', 'Monte Carlo convergence (median stability)', 'Second-law check for every candidate'], validation: ['Side-by-side bench test of competing actuators on the same thermal rig and load frame'] },
  formats: { import: ['CSV/XLSX test data per actuator', 'JSON'], export: ['CSV ranking table', 'JSON', 'PNG/SVG'] },
  params: [
    { key: 'basis', label: 'Comparison basis', type: 'select', group: 'Fairness', default: 'matched', options: [{ value: 'matched', label: 'Matched conditions (required for ranking)' }, { value: 'own', label: 'Each at its own favourable ΔT (NOT a fair ranking)' }] },
    { key: 'Th', label: 'Common hot temperature', unit: '°C', type: 'number', min: -10, max: 200, step: 0.5, group: 'Common conditions', default: 45, auto: (c) => { const o = c.upstream('s14'); return o && o.ThotIface_C - o.TcoldIface_C > 10 ? { value: +o.ThotIface_C.toFixed(1), prov: 'coupled' } : undefined; } },
    { key: 'Tc', label: 'Common cold temperature', unit: '°C', type: 'number', min: -40, max: 150, step: 0.5, group: 'Common conditions', default: 15, auto: (c) => { const o = c.upstream('s14'); return o && o.ThotIface_C - o.TcoldIface_C > 10 ? { value: +o.TcoldIface_C.toFixed(1), prov: 'coupled' } : undefined; } },
    { key: 't', label: 'Common element thickness / fibre diameter', unit: 'mm', type: 'number', min: 0.05, max: 3, step: 0.05, group: 'Common conditions', default: 0.3 },
    { key: 'hc', label: 'Common interface conductance', unit: 'W/m²K', type: 'number', min: 50, max: 50000, step: 50, group: 'Common conditions', default: 3000 },
    { key: 'dwell', label: 'Mechanical dwell fraction', type: 'range', min: 0, max: 1, step: 0.05, group: 'Common conditions', default: 0.1, expert: true },
    { key: 'mc', label: 'Monte Carlo samples per candidate', type: 'number', min: 5, max: 200, step: 5, group: 'Uncertainty', default: 40, expert: true },
    { key: 'uscale', label: 'Uncertainty scale', type: 'range', min: 0, max: 2, step: 0.1, group: 'Uncertainty', default: 1 },
  ],
  presets: [{ label: 'Calm night 5 K', values: { Th: 20, Tc: 15 } }, { label: 'Solar 50 K', values: { Th: 70, Tc: 20 } }, { label: 'Unfair: own ΔT', values: { basis: 'own' } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    if (p.Th <= p.Tc) throw new Error('Common hot temperature must exceed the cold temperature.');
    const base = { Th: p.Th, Tc: p.Tc, t_mm: p.t, hc: p.hc, dwell: p.dwell };
    const rows = [];
    for (const [i, m] of CANDIDATES.entries()) {
      ctx.progress(0.05 + 0.85 * i / CANDIDATES.length, m.name);
      const q = p.basis === 'own' ? (m.cls === 'sma' ? { ...base, Th: m.Af + 25, Tc: m.Mf - 10 } : { ...base, Th: Math.min(m.Tmax - 10, p.Tc + 80) }) : base;
      const r = evaluate(m, q);
      const R = rng(1000 + i), mc = range(p.mc).map(() => evaluate(perturb(m, R, p.uscale), q).P_kg);
      rows.push({ m, r, q, p05: percentile(mc, 0.05), p50: percentile(mc, 0.5), p95: percentile(mc, 0.95), mcStable: Math.abs(percentile(mc.slice(0, Math.ceil(mc.length / 2)), 0.5) - percentile(mc, 0.5)) / Math.max(1e-9, percentile(mc, 0.5)) });
      await ctx.yield();
    }
    rows.sort((a, b) => b.r.P_kg - a.r.P_kg);
    const fair = p.basis === 'matched', best = rows[0];
    const sameCond = rows.every(r => r.q.Th === rows[0].q.Th && r.q.Tc === rows[0].q.Tc && r.q.t_mm === rows[0].q.t_mm && r.q.hc === rows[0].q.hc);
    out.checks.push({ name: 'Common boundary conditions enforced for every candidate', value: sameCond ? 'yes' : 'no', target: 'yes', pass: sameCond });
    out.checks.push({ name: 'Second law for every candidate (η ≤ η_C)', value: Math.max(...rows.map(r => r.r.eta)), target: `≤ ${fmt(1 - (p.Tc + 273.15) / (p.Th + 273.15), 4)}`, pass: fair ? rows.every(r => r.r.eta <= 1 - (p.Tc + 273.15) / (p.Th + 273.15) + 1e-9) : undefined });
    out.checks.push({ name: 'Monte Carlo median stability (half vs full sample)', value: Math.max(...rows.filter(r => r.p50 > 0).map(r => r.mcStable), 0), target: '< 25 %', pass: Math.max(...rows.filter(r => r.p50 > 0).map(r => r.mcStable), 0) < 0.25 });
    const K = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    K('Common ΔT', p.Th - p.Tc, 'K', 'All candidates are evaluated between the same hot and cold temperatures.', ctx.upstream('s14') ? 'coupled' : 'user');
    K('Best candidate (matched)', fair ? best.m.name : '— (ranking invalid)', '', fair ? 'Highest median continuous power density under identical conditions.' : 'Mismatched conditions cannot establish superiority.', 'calc', fair ? 'ok' : 'bad');
    K('Best power density', best.r.P_kg, 'W/kg', `5–95 % range ${fmt(best.p05, 3)}–${fmt(best.p95, 3)} W/kg from property uncertainty.`, 'calc');
    K('Candidates with net-positive cycles', rows.filter(r => r.r.net).length + ' / ' + rows.length, '', 'Materials that complete a repeatable net-positive work cycle at this ΔT.', 'calc');
    const niti = rows.find(r => r.m.id.startsWith('niti')), tcp = rows.find(r => r.m.cls === 'tcp');
    if (niti && tcp) K('Best TCP / best NiTi power ratio', niti.r.P_kg ? tcp.r.P_kg / niti.r.P_kg : NaN, '', 'Below 1 means the polymer muscle delivers less power per kg than NiTi under these conditions.', 'calc');
    out.verdicts.push({ test: 'Fair comparison basis', status: fair ? 'pass' : 'fail', detail: fair ? 'Identical T_h, T_c, thickness, interface conductance and cycle budget for all candidates.' : 'Each material evaluated at its own favourable ΔT — the specification forbids using this to rank materials (e.g. 5 K vs 50 K).', fix: 'Select “Matched conditions”.' });
    out.verdicts.push({ test: 'Any actuator viable at this ΔT', status: rows.some(r => r.r.net && r.r.P_kg > 1) ? 'pass' : 'fail', detail: rows.some(r => r.r.net) ? `${rows.filter(r => r.r.net).map(r => r.m.name.split('(')[0].trim()).join(', ')} complete net-positive cycles.` : 'No candidate completes a net-positive cycle under these common conditions.' });
    const lab = rows.map(r => r.m.name.replace(/\(.*\)/, '').trim());
    out.charts.push({ title: 'Continuous power density under matched conditions (median & 5–95 % band)', type: 'line', wide: true, x: { label: 'Candidate rank', min: 0.5, max: rows.length + 0.5 }, y: { label: 'Power density', unit: 'W/kg' }, series: [...rows.map((r, i) => ({ name: `${i + 1}. ${r.m.name.replace(/\(.*\)/, '').trim()} (5–95 %)`, x: [i + 1, i + 1], y: [Math.max(0, r.p05), r.p95], width: 7 })), { name: 'Nominal', x: rows.map((_, i) => i + 1), y: rows.map(r => r.r.P_kg), mode: 'markers', r: 6, color: '#f97316', labels: lab }], explain: 'Bars show the uncertainty band propagated from property scatter; points are nominal values. Overlapping bands mean the ranking is not statistically resolved — measure before choosing.' });
    out.charts.push({ title: 'Work density per cycle', type: 'bar', categories: lab, series: [{ name: 'J/kg', values: rows.map(r => r.r.w_J_kg) }], y: { label: 'Net work', unit: 'J/kg' }, explain: 'Energy per cycle per kg at each material’s optimum load (same ΔT).' });
    out.charts.push({ title: 'Cycle frequency vs work density', type: 'scatter', x: { label: 'Cycle frequency', unit: 'Hz', log: true }, y: { label: 'Net work', unit: 'J/kg' }, series: [{ name: 'candidates', x: rows.map(r => Math.max(1e-3, r.r.f)), y: rows.map(r => r.r.w_J_kg), labels: lab, r: 5 }], explain: 'Polymers cycle slowly (low conductivity) and store little work per cycle; SMAs store more but need ΔT beyond their hysteresis.' });
    out.charts.push({ title: 'Efficiency and durability', type: 'bar', categories: lab, series: [{ name: 'Efficiency (%)', values: rows.map(r => r.r.eta * 100) }, { name: 'log₁₀ fatigue cycles', values: rows.map(r => Math.log10(r.r.fatigue || 1)) }], y: { label: 'Value' }, explain: 'Durability matters for sustainability: a short-lived actuator can lose on life-cycle impact despite higher power.' });
    out.tables.push({ title: 'Matched-condition ranking', cols: ['Rank', 'Material', 'Net work (J/kg)', 'Freq (Hz)', 'Power (W/kg)', 'P 5–95 % (W/kg)', 'η (%)', 'Load (MPa)', 'Fatigue (cycles)', 'Cost (USD/W)', 'CO₂ (kg/W)'], rows: rows.map((r, i) => [i + 1, r.m.name, r.r.w_J_kg, r.r.f, r.r.P_kg, `${fmt(r.p05, 3)}–${fmt(r.p95, 3)}`, r.r.eta * 100, r.r.stress, r.r.fatigue, r.r.P_kg > 0 ? r.m.cost / r.r.P_kg : '∞', r.r.P_kg > 0 ? r.m.co2 / r.r.P_kg : '∞']), explain: 'Cost and embodied carbon per watt of continuous mechanical output (materials only).' });
    out.recs.push(fair ? { level: 'good', text: `Under matched ${fmt(p.Th - p.Tc, 3)} K conditions ${best.m.name} leads (${fmt(best.r.P_kg, 3)} W/kg). Retain the runner-up as a hedge until bench tests resolve the overlapping uncertainty bands.`, basis: 'Matched ranking with Monte Carlo' } : { level: 'critical', text: 'Switch to matched conditions before drawing any conclusion about superiority.', basis: 'Specification fairness rule' });
    if (tcp && niti && tcp.r.fatigue > niti.r.fatigue) out.recs.push({ level: 'info', text: 'Polymer muscles offer longer fatigue life and lower cost; consider them where cycle life dominates power density.', basis: 'Durability vs power trade-off' });
    out.recs.push({ level: 'sustain', text: 'Compare candidates on CO₂ per watt over the service life (power × life), not on peak force or stroke.', basis: 'Life-cycle perspective' });
    out.explain.push({ title: 'Why matched conditions', text: 'Actuator literature reports results at very different temperature differences, loads and timescales. A material tested at 50 K will always look better than one tested at 5 K. Ranking is meaningful only when every candidate faces the same thermal boundary conditions, geometry and time budget.' });
    out.outputs = { ranking: rows.map(r => ({ id: r.m.id, name: r.m.name, P_kg: r.r.P_kg, w_J_kg: r.r.w_J_kg })), best: fair ? best.m.id : null, wNet_J_kg: best.r.w_J_kg, powerDensity_W_kg: best.r.P_kg, fair };
    out.headline = fair ? `Matched ${fmt(p.Th - p.Tc, 3)} K comparison: ${best.m.name} ranks first at ${fmt(best.r.P_kg, 3)} W/kg (5–95 %: ${fmt(best.p05, 3)}–${fmt(best.p95, 3)}).` : 'Ranking refused: candidates were not compared under identical conditions.';
    out.headlineKind = fair ? 'info' : 'fail';
    return out;
  },
};
