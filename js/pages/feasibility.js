// The four mandatory feasibility tests (specification §32) evaluated across environmental scenarios.
import { h, section, callout, progressBar, table, toast } from '../core/ui.js';
import { runChain, FEASIBILITY_CHAIN } from '../core/pipeline.js';
import { store } from '../core/state.js';
import { roadLoad, kmh, carnot } from '../core/physics.js';
import { fmt } from '../core/num.js';
import { SCENARIOS } from '../core/env.js';
import { chart } from '../core/chart.js';
export const title = 'Mandatory feasibility tests';

const SET = [['favorable', 'Windy (favourable)'], ['sunnyCalm', 'Daytime, low wind'], ['clearNight', 'Clear night, calm'], ['cloudyNight', 'Humid / cloudy night'], ['gust', 'Gusting'], ['reversal', 'Changing (wind reversal)'], ['worstCase', 'Worst-case energy'], ['live', 'Live weather at site']];

export async function render(root) {
  root.append(h('h1', {}, 'Mandatory feasibility tests'), h('p', { class: 'muted', style: 'max-width:980px' }, 'Every candidate configuration must pass four distinct tests: aerodynamic, thermal, mechanical and integrated-vehicle feasibility — under daytime, night-time, windy, low-wind, gusting, humid, cloudy and changing conditions. The coupled chain of suites (atmosphere → architecture → tyres → CFD → wing → handling → photonics → thermal → NiTi → hub → buffer → switching → vehicle energy) is executed for each scenario with the current project configuration. A thermomechanical system that exceeds its environmental exergy fails, whatever its recovery force; a wing with high lift but insufficient forward force earns no propulsion credit.'));
  const speed = h('input', { type: 'number', value: 20, min: 5, max: 60, step: 1, class: 'search', style: 'max-width:90px' });
  const pick = h('div', { class: 'row gap wrap' }, ...SET.map(([k, l]) => h('label', { class: 'chip' }, h('input', { type: 'checkbox', checked: k !== 'live', value: k }), ' ' + l)));
  const prog = progressBar(), status = h('div', { class: 'small muted' }), results = h('div');
  const runBtn = h('button', { class: 'btn primary', onclick: () => go() }, '▶ Run all scenarios');
  root.append(section('Configuration', h('div', { class: 'row gap wrap' }, h('label', {}, 'Target cruising speed ', speed, ' km/h'), runBtn), h('div', { class: 'small muted', style: 'margin:8px 0' }, 'Scenarios'), pick, prog, status), results);
  const prevScenario = store.params('s05').scenario;

  async function go() {
    runBtn.disabled = true; results.innerHTML = '';
    const chosen = [...pick.querySelectorAll('input:checked')].map(i => i.value), vT = kmh(+speed.value), rows = [];
    for (const [si, sc] of chosen.entries()) {
      status.textContent = `Scenario ${si + 1}/${chosen.length}: ${SCENARIOS[sc].label}`;
      const S20 = { favorable: 's2', acceleration: 's2', crosswind: 's3', gust: 's4', reversal: 's5', sunnyCalm: 's6', clearNight: 's7', cloudyNight: 's8', worstCase: 's18', live: 's17' };
      const out = await runChain(FEASIBILITY_CHAIN, { s05: { scenario: sc }, s20: { scenario: S20[sc] || 's2', speed: +speed.value } }, { onStep: (id, f, msg) => { prog.set((si + f) / chosen.length); status.textContent = `${SCENARIOS[sc].label} · suite ${id}: ${msg}`; } });
      rows.push({ sc, label: SCENARIOS[sc].label, ...evaluate(out, vT) });
    }
    store.setParam('s05', 'scenario', prevScenario ?? 'live', 'user');
    prog.set(1); status.textContent = 'Completed. The last scenario’s results remain loaded in the suites; re-run Suite 05 to restore your chosen conditions.'; runBtn.disabled = false;
    draw(rows, +speed.value);
    toast('Feasibility matrix complete', 'ok');
  }
  function draw(rows, v) {
    const badge = (t) => h('span', { class: 'badge ' + ({ pass: 'ok', fail: 'fail', warn: 'warn' }[t.status] || ''), title: t.detail }, t.status.toUpperCase());
    results.append(section('Feasibility matrix', table(['Scenario', 'Aerodynamic', 'Thermal', 'Mechanical', 'Integrated vehicle', 'Sustainable speed', 'Wheel power available', 'Required at target'], rows.map(r => [r.label, badge(r.aero), badge(r.thermal), badge(r.mech), badge(r.vehicle), r.vSus !== null ? fmt(r.vSus, 3) + ' km/h' : '—', fmt(r.Pavail, 3) + ' W', fmt(r.Preq, 3) + ' W']))));
    const all = rows.flatMap(r => [r.aero, r.thermal, r.mech, r.vehicle]), nPass = all.filter(t => t.status === 'pass').length;
    const allPassScenarios = rows.filter(r => [r.aero, r.thermal, r.mech, r.vehicle].every(t => t.status === 'pass')).map(r => r.label);
    results.append(callout(allPassScenarios.length === rows.length ? 'ok' : allPassScenarios.length ? 'warn' : 'fail', h('b', {}, 'Conclusion: '), allPassScenarios.length ? `The configuration passes all four tests in: ${allPassScenarios.join(', ')}. ` : 'No scenario passes all four tests. ', `${nPass}/${all.length} individual tests pass at ${v} km/h. Where tests fail, the detail column explains the binding constraint — use Suite 25 to identify the parameters that would have to change and by how much.`));
    for (const r of rows) results.append(section(r.label, h('ul', {}, ...['aero', 'thermal', 'mech', 'vehicle'].map(k => h('li', {}, h('b', {}, { aero: 'Aerodynamic: ', thermal: 'Thermal: ', mech: 'Mechanical: ', vehicle: 'Integrated vehicle: ' }[k]), r[k].detail)))));
    const box = h('div'); results.append(section('Available versus required wheel power', box));
    chart(box, { type: 'bar', title: 'Wheel power by scenario', categories: rows.map(r => r.label), y: { label: 'Power', unit: 'W' }, series: [{ name: 'Wind propulsion power', values: rows.map(r => r.Pwind) }, { name: 'Thermal backup (hub)', values: rows.map(r => r.Phub) }, { name: 'Required at target speed', values: rows.map(r => r.Preq) }], explain: 'Useful forward wind power (thrust × speed), sustained hub power from the thermomechanical backup, and the road-load demand at the target speed. Stored buffer energy is finite and is assessed in the integrated test.' });
  }
}

function evaluate(out, vT) {
  const o = (id) => out[id]?.res?.outputs || {}, err = (id) => out[id]?.error;
  const v = store.project.vehicle, Preq = roadLoad(vT, v).P, Fstart = roadLoad(kmh(2), v).F * 1.4;
  const s07 = o('s07'), s06 = o('s06'), s11 = o('s11'), s13 = o('s13'), s14 = o('s14'), s15 = o('s15'), s17 = o('s17'), s20 = o('s20'), env = o('s05');
  // 1. aerodynamic
  const thrust = s07.thrustN ?? s06.thrustN; let aero;
  if (thrust === undefined) aero = { status: 'warn', detail: `Aerodynamic thrust unavailable${err('s07') ? ' (CFD suite error: ' + err('s07') + ')' : ''}.` };
  else { const stable = s11.rolloverMargin === undefined || s11.rolloverMargin > 0; aero = { status: thrust > Fstart && stable ? 'pass' : thrust > 0 && stable ? 'warn' : 'fail', detail: `Useful forward thrust ${fmt(thrust, 3)} N vs starting resistance ≈ ${fmt(Fstart, 3)} N at wind ${fmt(env.Uz ?? NaN, 3)} m/s${s11.rolloverMargin !== undefined ? `; rollover margin ${fmt(s11.rolloverMargin, 3)}` : ''}.` }; }
  // 2. thermal (exergy bound)
  const Th = (s14.ThotIface_C ?? s13.Thot_C), Tc = (s14.TcoldIface_C ?? s13.Tcold_C), Q = s14.Qhot_W ?? s13.Pabs_W;
  let thermal, Ex = 0;
  if (Th === undefined || Tc === undefined || Q === undefined) thermal = { status: 'warn', detail: 'Thermal interface temperatures / heat flow unavailable — run Suites 13–14.' };
  else { const eta = carnot(Th + 273.15, Tc + 273.15); Ex = Q * eta; thermal = { status: Ex >= Preq ? 'pass' : Ex >= 0.25 * Preq ? 'warn' : 'fail', detail: `Interfaces ${fmt(Th, 3)} °C / ${fmt(Tc, 3)} °C (ΔT ${fmt(Th - Tc, 3)} K), heat ${fmt(Q, 3)} W, Carnot ${fmt(100 * eta, 3)} % → exergy ceiling ${fmt(Ex, 3)} W vs ${fmt(Preq, 3)} W required.` }; }
  // 3. mechanical
  let mech;
  if (s15.netPositive === undefined && s17.Wrev_J === undefined) mech = { status: 'warn', detail: 'Actuator / hub results unavailable — run Suites 15 and 17.' };
  else { const ok = s15.netPositive !== false && (s17.Wrev_J ?? 0) > 0; mech = { status: ok && s17.startupOK !== false ? 'pass' : ok ? 'warn' : 'fail', detail: `NiTi net-positive cycle: ${s15.netPositive === undefined ? 'n/a' : s15.netPositive ? 'yes' : 'NO'} (${fmt(s15.Wcycle_J ?? NaN, 3)} J/cycle); hub net work ${fmt(s17.Wrev_J ?? NaN, 3)} J/rev, mean wheel power ${fmt(s17.Pwheel_W ?? NaN, 3)} W; self-start at all angles: ${s17.startupOK === undefined ? 'n/a' : s17.startupOK ? 'yes' : 'NO'}.` }; }
  // 4. integrated
  const Pwind = thrust !== undefined ? Math.max(0, thrust) * vT : 0, Phub = Math.max(0, Math.min(s17.Pwheel_W ?? 0, Ex || Infinity));
  let vehicle; const vSus = s20.sustainableSpeed_kmh ?? null;
  if (s20.journeySuccess !== undefined) vehicle = { status: s20.journeySuccess && (vSus ?? 0) * 1 >= vT * 3.6 * 0.95 ? 'pass' : s20.journeySuccess ? 'warn' : 'fail', detail: `Journey ${s20.journeySuccess ? 'completed' : 'NOT completed'}; sustainable speed ${fmt(vSus ?? NaN, 3)} km/h; energy deficit ${fmt((s20.energyDeficit_J ?? 0) / 1000, 3)} kJ.` };
  else { const tot = Pwind + Phub; vehicle = { status: tot >= Preq ? 'pass' : tot >= 0.5 * Preq ? 'warn' : 'fail', detail: `Available ${fmt(tot, 3)} W (wind ${fmt(Pwind, 3)} W + hub ${fmt(Phub, 3)} W) vs ${fmt(Preq, 3)} W at ${fmt(vT * 3.6, 3)} km/h → deficit ${fmt(Math.max(0, Preq - tot), 3)} W.` }; }
  return { aero, thermal, mech, vehicle, Preq, Pwind, Phub, Pavail: Pwind + Phub, vSus };
}
