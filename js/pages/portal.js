// Universal input data portal: import case data in common formats and route it to the right suite.
import { h, section, callout, table, toast } from '../core/ui.js';
import { parseFile, parseDelimited, ACCEPT, IMPORT_FORMATS, autoMap } from '../core/io.js';
import { store } from '../core/state.js';
import { SUITES } from '../suites/index.js';
import { fmt } from '../core/num.js';
export const title = 'Data portal';

export async function render(root) {
  store.project.imports ||= {};
  root.append(h('h1', {}, 'Input data portal'), h('p', { class: 'muted', style: 'max-width:980px' }, 'Supply case data in any of the accepted formats. Each dataset is previewed, unit-checked, mapped to physical quantities and routed to the suite that uses it: weather-station observations, routes, airfoil coordinates, geometry, experimental measurements for calibration/validation, or a complete project file. Everything stays on your device.'));
  root.append(section('Accepted formats', table(['Format family', 'Extensions'], IMPORT_FORMATS.map(f => [f.label, f.ext.map(e => '.' + e).join('  ')]))));
  const drop = h('label', { class: 'drop' }, h('input', { type: 'file', accept: ACCEPT, multiple: true, onchange: (e) => handle([...e.target.files]) }), h('div', {}, h('div', { style: 'font-size:28px' }, '⇪'), 'Drop files here or tap to choose (multiple allowed)'));
  drop.addEventListener('dragover', (e) => { e.preventDefault(); drop.classList.add('over'); }); drop.addEventListener('dragleave', () => drop.classList.remove('over'));
  drop.addEventListener('drop', (e) => { e.preventDefault(); drop.classList.remove('over'); handle([...e.dataTransfer.files]); });
  const paste = h('textarea', { rows: 6, class: 'search', style: 'max-width:100%;width:100%;font-family:var(--mono)', placeholder: 'Or paste delimited data here (first row = column names, e.g. time,temperature_C,wind_ms,dir_deg,ghi_Wm2)' });
  const out = h('div');
  root.append(section('Upload', drop, paste, h('button', { class: 'btn', onclick: () => { try { const r = parseDelimited(paste.value); show({ kind: 'table', name: 'pasted-data.csv', ...r }); } catch (e) { toast(e.message, 'fail'); } } }, 'Parse pasted data')), out);
  current(root);

  async function handle(files) { for (const f of files) { try { show(await parseFile(f)); } catch (e) { out.prepend(callout('fail', `${f.name}: ${e.message}`)); } } }
  function show(r) {
    const card = section(`${r.name} — ${r.kind}`);
    if (r.kind === 'table') {
      const map = autoMap(r.header);
      card.append(h('p', { class: 'small muted' }, `${r.nRows} rows × ${r.header.length} columns. Detected quantities: ${Object.entries(map).map(([q, c]) => `${q} ← “${c}”`).join(', ') || 'none (map manually)'}. Units from headers: ${Object.entries(r.units).map(([c, u]) => `${c} [${u}]`).join(', ') || 'none found — SI assumed'}.`));
      card.append(table(r.header, Array.from({ length: Math.min(12, r.nRows) }, (_, i) => r.header.map(c => r.columns[c][i]))));
      const sel = (q) => h('select', { 'data-q': q }, h('option', { value: '' }, '—'), ...r.header.map(c => h('option', { value: c, selected: map[q] === c }, c)));
      const mapBox = h('div', { class: 'row gap wrap' }, ...['time', 'temperature', 'wind', 'direction', 'irradiance', 'humidity', 'pressure'].map(q => h('label', { class: 'small' }, q + ' ', sel(q))));
      card.append(h('h4', {}, 'Use as weather observations (drives Suite 05 → all coupled suites)'), mapBox, h('button', { class: 'btn primary', onclick: () => { const m = Object.fromEntries([...mapBox.querySelectorAll('select')].map(s => [s.dataset.q, s.value])); if (!m.temperature || !m.wind) return toast('Map at least temperature and wind', 'fail'); const n = r.nRows; store.project.imports.weather = { name: r.name, rows: Array.from({ length: n }, (_, i) => ({ time: m.time ? String(r.columns[m.time][i]) : String(i), T: +r.columns[m.temperature][i], U: +r.columns[m.wind][i], dir: m.direction ? +r.columns[m.direction][i] : 90, ghi: m.irradiance ? +r.columns[m.irradiance][i] : 0, RH: m.humidity ? +r.columns[m.humidity][i] : 60, p: m.pressure ? +r.columns[m.pressure][i] * (+r.columns[m.pressure][i] < 2000 ? 100 : 1) : 101325 })) }; store.setParam('s05', 'scenario', 'imported', 'user'); store.touch(); toast('Weather observations loaded — Suite 05 scenario set to “Imported observations”', 'ok'); } }, 'Load as weather observations'));
      card.append(h('h4', {}, 'Or attach as experimental dataset to a suite'), attach(r));
    } else if (r.kind === 'airfoil') {
      card.append(h('p', {}, `${r.points.length} coordinate points (${r.name}).`), airfoilSVG(r.points), h('button', { class: 'btn primary', onclick: () => { store.project.imports.airfoil = { name: r.name, points: r.points }; store.invalidate('s02', 'airfoil imported'); store.touch(); toast('Airfoil available in Suites 02/06/07 as “Imported”', 'ok'); } }, 'Use as wing airfoil'));
    } else if (r.kind === 'route' || r.kind === 'geojson') {
      const pts = r.points || []; let d = 0; for (let i = 1; i < pts.length; i++) d += hav(pts[i - 1], pts[i]);
      card.append(h('p', {}, `${pts.length} route points, length ${fmt(d / 1000, 4)} km, elevation ${fmt(Math.min(...pts.map(p => p.ele || 0)), 4)}–${fmt(Math.max(...pts.map(p => p.ele || 0)), 4)} m.`), h('button', { class: 'btn primary', onclick: () => { store.project.imports.route = pts; if (pts[0]) store.setSite({ lat: pts[0].lat, lon: pts[0].lon, name: r.name + ' (route start)' }); store.touch(); toast('Route loaded for journey simulations (Suite 20)', 'ok'); } }, 'Use as journey route'));
    } else if (r.kind === 'mesh') {
      card.append(h('p', {}, `${r.format}: ${r.triangles} triangles${r.nodes ? ', ' + r.nodes + ' nodes' : ''}.`), h('button', { class: 'btn primary', onclick: () => { store.project.imports.geometry = { name: r.name, positions: r.positions, triangles: r.triangles }; toast('Geometry available in Suite 02 viewer (session only)', 'ok'); } }, 'Use as reference geometry'));
    } else if (r.kind === 'project') {
      card.append(h('p', {}, `ATMOS-XP project “${r.data.name}” revision ${r.data.revision}.`), h('button', { class: 'btn primary', onclick: () => { store.replace(r.data); toast('Project loaded', 'ok'); location.hash = '#/project'; } }, 'Open this project (replaces current)'));
    } else card.append(h('pre', { class: 'small', style: 'max-height:240px;overflow:auto' }, JSON.stringify(r.data, null, 2).slice(0, 4000)), attach(r));
    out.prepend(card);
  }
  function attach(r) { const s = h('select', {}, ...SUITES.map(x => h('option', { value: x.id }, `${String(x.num).padStart(2, '0')} ${x.short}`))), use = h('select', {}, h('option', {}, 'validation'), h('option', {}, 'calibration')); return h('div', { class: 'row gap wrap' }, s, use, h('button', { class: 'btn', onclick: () => { const ex = (store.project.experiments[s.value] ||= []); ex.push({ name: r.name, kind: r.kind, use: use.value, t: new Date().toISOString(), columns: r.columns, header: r.header, nRows: r.nRows, map: r.header ? autoMap(r.header) : undefined }); store.touch(); toast(`Attached to suite ${s.value} (${use.value})`, 'ok'); } }, 'Attach')); }
}
function current(root) {
  const im = store.project.imports || {}, ex = store.project.experiments || {};
  const rows = [];
  if (im.weather) rows.push(['Weather observations', im.weather.name, im.weather.rows.length + ' rows', 'Suite 05']);
  if (im.airfoil) rows.push(['Airfoil', im.airfoil.name, im.airfoil.points.length + ' pts', 'Suites 02/06/07']);
  if (im.route) rows.push(['Route', '—', im.route.length + ' pts', 'Suite 20']);
  for (const [sid, list] of Object.entries(ex)) for (const d of list) rows.push(['Experiment (' + d.use + ')', d.name, (d.nRows ?? '') + ' rows', sid]);
  if (rows.length) root.append(section('Data currently loaded in this project', table(['Type', 'Name', 'Size', 'Used by'], rows)));
}
const hav = (a, b) => { const R = 6371e3, p1 = a.lat * Math.PI / 180, p2 = b.lat * Math.PI / 180, dp = p2 - p1, dl = (b.lon - a.lon) * Math.PI / 180; const x = Math.sin(dp / 2) ** 2 + Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) ** 2; return 2 * R * Math.asin(Math.sqrt(x)); };
function airfoilSVG(pts) { const d = 'M' + pts.map(p => `${(10 + p[0] * 380).toFixed(1)},${(60 - p[1] * 380).toFixed(1)}`).join('L') + 'Z'; return h('div', { html: `<svg viewBox="0 0 400 120" style="width:100%;max-width:520px"><path d="${d}" fill="rgba(56,189,248,.2)" stroke="#38bdf8"/></svg>` }); }
