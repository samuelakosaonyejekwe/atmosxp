// Project record: metadata, configuration, revisions, reproducible project package.
import { h, section, table, toast, modal } from '../core/ui.js';
import { store, CREDIBILITY } from '../core/state.js';
import { SUITES } from '../suites/index.js';
import { zip, saveJSON } from '../core/io.js';
import { download } from '../core/chart.js';
import { fmt } from '../core/num.js';
import * as materials from '../core/materials.js';
export const title = 'Project';

export async function render(root) {
  const P = store.project;
  const inp = (key, label) => h('label', { class: 'param' }, h('span', { class: 'pl' }, label), h('input', { type: 'text', value: P[key] || '', onchange: (e) => { P[key] = e.target.value; store.touch(); } }));
  root.append(h('h1', {}, 'Project & reproducibility'));
  root.append(section('Project record', h('div', { class: 'two' }, h('div', {}, inp('name', 'Project name'), inp('author', 'Author'), inp('objective', 'Objective')), h('div', {}, table(['Field', 'Value'], [['Identifier', P.id], ['Revision', P.revision], ['Created', new Date(P.created).toLocaleString()], ['Modified', new Date(P.modified).toLocaleString()], ['Site', `${P.site.name} (${fmt(P.site.lat, 6)}, ${fmt(P.site.lon, 6)})`], ['Software version', window.AXP_VERSION]])))));
  root.append(section('Vehicle configuration (global, shared by all suites)', table(['Parameter', 'Value'], Object.entries(P.vehicle).map(([k, v]) => [k, v])), h('p', { class: 'small muted' }, 'Edit the architecture in Suite 01; changes invalidate dependent results automatically.')));
  root.append(section('Material assignments', table(['Component', 'Material', 'Family'], Object.entries(materials.COMPONENTS).map(([k, c]) => { const m = materials.componentMaterial(store, k); return [c.name, m?.name || '—', m?.family || '']; }))));
  root.append(section('Suite results status', table(['Suite', 'Status', 'Credibility', 'Computed', 'Inputs hash'], SUITES.map(s => { const o = store.outputMeta(s.id); return [`${String(s.num).padStart(2, '0')} ${s.title}`, o ? (o.stale ? 'stale' : 'current') : 'not run', o?.credibility || '—', o ? new Date(o.t).toLocaleString() : '—', o?.hash || '—']; })), h('p', { class: 'small muted' }, 'Credibility ladder: ' + CREDIBILITY.join(' → '))));
  const note = h('input', { type: 'text', class: 'search', placeholder: 'Revision note (e.g. “switched spar to IM7, 26 cartridges”)' });
  root.append(section('Revisions (immutable)', h('div', { class: 'row gap wrap' }, note, h('button', { class: 'btn', onclick: () => { store.snapshot(note.value); toast('Revision saved', 'ok'); location.reload(); } }, 'Save revision')), P.history.length ? table(['Revision', 'Time', 'Note', 'Suites with results', 'Mass'], P.history.slice().reverse().map(r => [r.revision, new Date(r.t).toLocaleString(), r.note, r.suites, r.vehicle?.mass])) : h('p', { class: 'muted' }, 'No saved revisions yet.')));
  root.append(section('Export / import', h('div', { class: 'row gap wrap' },
    h('button', { class: 'btn primary', onclick: () => projectPackage() }, 'Full project package (ZIP + manifest)'),
    h('button', { class: 'btn', onclick: () => saveJSON(store.export(), `${P.id}-r${P.revision}.atmosxp.json`) }, 'Native project file (.atmosxp.json)'),
    h('label', { class: 'btn' }, 'Open project file…', h('input', { type: 'file', accept: '.json', style: 'display:none', onchange: async (e) => { try { const j = JSON.parse(await e.target.files[0].text()); store.replace(j); toast('Project opened', 'ok'); location.reload(); } catch (err) { toast(err.message, 'fail'); } } })),
    h('button', { class: 'btn', onclick: () => modal('Start a new project?', h('p', {}, 'This clears the current configuration, results and imported data from this device (export first if you need them).'), [{ label: 'Cancel' }, { label: 'Start new project', primary: true, onClick: () => { store.reset(); location.hash = '#/'; location.reload(); } }]) }, 'New project'))));
}

async function projectPackage() {
  const P = store.export(); const files = [];
  const manifest = { software: 'PHOENIX-ATMOS-XP', version: window.AXP_VERSION, project: P.id, revision: P.revision, created: new Date().toISOString(), contents: [] };
  const add = (name, data, desc) => { files.push({ name, data }); manifest.contents.push({ name, description: desc }); };
  add('project.atmosxp.json', JSON.stringify(P, null, 2), 'Native project: configuration, units (SI), provenance, locks, results, experiments, revisions');
  add('configuration/vehicle.json', JSON.stringify(P.vehicle, null, 2), 'Vehicle parameters');
  add('configuration/materials.json', JSON.stringify(Object.fromEntries(Object.keys(materials.COMPONENTS).map(k => [k, materials.componentMaterial(store, k)])), null, 2), 'Assigned material records with sources and uncertainty');
  add('environment/environment.json', JSON.stringify(P.env || {}, null, 2), 'Environmental boundary conditions with data-kind labels');
  for (const [id, o] of Object.entries(P.outputs)) add(`results/${id}.json`, JSON.stringify(o, null, 2), `Suite ${id} outputs`);
  for (const [id, list] of Object.entries(P.experiments || {})) add(`experiments/${id}.json`, JSON.stringify(list, null, 2), `Experimental datasets for ${id}`);
  files.unshift({ name: 'manifest.json', data: JSON.stringify(manifest, null, 2) });
  download(await zip(files), `${P.id}-r${P.revision}-project.zip`); toast('Project package downloaded', 'ok');
}
