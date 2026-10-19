// Project state: typed parameters, provenance, coupled outputs, dependency invalidation, revisions, persistence.
import { DEFAULT_VEHICLE } from './physics.js';

export const PROVENANCE = {
  user: { label: 'User-specified', cls: 'p-user', tip: 'Directly chosen by the operator' },
  library: { label: 'Verified library', cls: 'p-lib', tip: 'Supported by a referenced measurement or assessed source' },
  calc: { label: 'Calculated', cls: 'p-calc', tip: 'Derived from governing equations and other inputs' },
  generated: { label: 'Generated design', cls: 'p-gen', tip: 'Produced by CAD, optimisation or inverse-design algorithms' },
  estimated: { label: 'Estimated', cls: 'p-est', tip: 'Engineering approximation with explicit uncertainty' },
  calibrated: { label: 'Calibrated', cls: 'p-cal', tip: 'Fitted to experimental measurements' },
  hypothetical: { label: 'Hypothetical', cls: 'p-hyp', tip: 'Investigational assumption requiring validation' },
  unsupported: { label: 'Unsupported', cls: 'p-uns', tip: 'Insufficient information; cannot be used for validated claims' },
  live: { label: 'Live data', cls: 'p-live', tip: 'Retrieved from a live global data service (see Data Hub for source & timestamp)' },
  coupled: { label: 'Coupled', cls: 'p-coup', tip: 'Received from an upstream suite result' },
};
export const CREDIBILITY = ['Conceptual', 'Analytical screening', 'Numerically verified', 'Calibrated', 'Component validated', 'Subsystem validated', 'Full-system validated', 'Environmentally validated'];

const KEY = 'atmosxp.project.v1';
const listeners = new Set();
let saveTimer = null;

function freshProject() {
  return {
    id: 'P-' + Date.now().toString(36).toUpperCase(), name: 'ATMOS-XP baseline study', author: '', objective: 'Determine which ATMOS-XP configurations are feasible, which fail, and why.',
    revision: 1, created: new Date().toISOString(), modified: new Date().toISOString(),
    vehicle: { ...DEFAULT_VEHICLE },
    site: { lat: 6.5244, lon: 3.3792, name: 'Lagos, Nigeria', elevation: 41, roughness: 'farmland, few hedges' },
    env: null,             // last environment snapshot (from suite 05 / live data)
    materials: {},         // component -> material id assignment
    params: {},            // suiteId -> {key: value}
    prov: {},              // suiteId -> {key: provenance}
    locks: {},             // suiteId -> {key: true}
    modes: {},             // suiteId -> 'auto'|'guided'|'expert'|'import'
    outputs: {},           // suiteId -> {t, hash, credibility, data:{...}}
    experiments: {},       // suiteId -> imported experimental datasets
    history: [],           // immutable revision summaries
    scenario: 'live',
  };
}

export const store = {
  project: load(),
  subscribe(fn) { listeners.add(fn); return () => listeners.delete(fn); },
  emit(evt) { for (const fn of listeners) { try { fn(evt, this.project); } catch (e) { console.error(e); } } },
  touch(evt = { type: 'change' }) { this.project.modified = new Date().toISOString(); schedule(); this.emit(evt); },
  get vehicle() { return this.project.vehicle; },
  setVehicle(patch, source = 'user') { Object.assign(this.project.vehicle, patch); this.invalidateAll(`vehicle (${source})`); this.touch({ type: 'vehicle' }); },
  setSite(patch) { Object.assign(this.project.site, patch); this.invalidate('s05', 'site changed'); this.touch({ type: 'site' }); },
  params(id) { return (this.project.params[id] ||= {}); },
  prov(id) { return (this.project.prov[id] ||= {}); },
  locks(id) { return (this.project.locks[id] ||= {}); },
  mode(id) { return this.project.modes[id] || 'auto'; },
  setMode(id, m) { this.project.modes[id] = m; this.touch({ type: 'mode', id }); },
  autoKeys(id) { return ((this.project.autoKeys ||= {})[id] ||= {}); },
  setParam(id, key, value, prov = 'user') { this.params(id)[key] = value; this.prov(id)[key] = prov; if (prov === 'user') delete this.autoKeys(id)[key]; this.invalidate(id, `${key} changed`); this.touch({ type: 'param', id, key }); },
  setMaterial(component, matId) { this.project.materials[component] = matId; this.invalidateAll(`material of ${component}`); this.touch({ type: 'material', component }); },
  material(component, fallback) { return this.project.materials[component] || fallback; },
  publish(id, data, { credibility = 'Analytical screening', hash = '' } = {}) {
    this.project.outputs[id] = { t: Date.now(), hash, credibility, data, stale: false };
    for (const d of dependents(id)) this.invalidate(d, `upstream ${id} re-computed`, false);
    this.touch({ type: 'output', id });
  },
  output(id) { return this.project.outputs[id]?.data || null; },
  outputMeta(id) { return this.project.outputs[id] || null; },
  invalidate(id, reason = '', cascade = true) {
    const o = this.project.outputs[id]; if (o && !o.stale) { o.stale = true; o.staleReason = reason; }
    if (cascade) for (const d of dependents(id)) { const od = this.project.outputs[d]; if (od && !od.stale) { od.stale = true; od.staleReason = `upstream ${id}: ${reason}`; } }
  },
  invalidateAll(reason) { for (const id of Object.keys(this.project.outputs)) this.invalidate(id, reason, false); },
  setEnv(env) { this.project.env = env; this.invalidate('s05', 'environment updated'); this.touch({ type: 'env' }); },
  snapshot(note = '') {
    const p = this.project; p.history.push({ revision: p.revision, t: new Date().toISOString(), note, suites: Object.keys(p.outputs).length, vehicle: { ...p.vehicle } });
    p.revision++; this.touch({ type: 'revision' });
  },
  reset() { this.project = freshProject(); this.touch({ type: 'reset' }); },
  replace(obj) { this.project = Object.assign(freshProject(), obj); this.touch({ type: 'reset' }); },
  export() { return JSON.parse(JSON.stringify(this.project)); },
};

// Dependency graph between suites (who consumes whose outputs). Mirrors the five coupling chains in the specification.
export const DEPENDS = {
  s01: [], s02: ['s01'], s03: [], s04: ['s03'], s05: [],
  s06: ['s04', 's05', 's07'], s07: ['s01', 's02', 's05'], s08: ['s06', 's07'], s09: ['s01', 's03', 's06', 's08', 's10', 's12'],
  s10: ['s01', 's03'], s11: ['s01', 's07', 's10'], s12: ['s01', 's07', 's10'],
  s13: ['s05', 's03'], s14: ['s13', 's03'], s15: ['s14', 's03'], s16: ['s14', 's15'], s17: ['s15', 's14', 's10'],
  s18: ['s04', 's03'], s19: ['s07', 's17', 's18'], s20: ['s05', 's07', 's10', 's17', 's18', 's19'], s21: ['s07', 's10', 's17'],
  s22: ['s02', 's03', 's09', 's17'], s23: ['s06', 's08', 's09', 's11', 's12', 's17'], s24: [], s25: ['s20'], s26: ['s03', 's22'],
};
export function dependents(id) { return Object.keys(DEPENDS).filter(k => DEPENDS[k].includes(id)); }

function load() {
  try {
    const raw = localStorage.getItem(KEY);
    if (raw) {
      const p = Object.assign(freshProject(), JSON.parse(raw));
      // Migrate untouched projects from baseline 1 (3 m wings) to the lower, more stable baseline 2
      const v = p.vehicle || {}; if ((!v.baseline && v.wingHeight === 3 && v.wingArea === 8 && v.track === 1.5) || (v.baseline === 2 && v.wingArea === 6.4 && v.length === 4.2) || (v.baseline === 3 && v.wingArea === 3.3) || (v.baseline === 4 && v.wingArea === 5 && v.length === 4.4) || (v.baseline === 5 && v.wingArea === 4.75) || v.baseline === 5 || (v.baseline === 6 && v.wingArea === 4.0)) { p.vehicle = { ...v, ...DEFAULT_VEHICLE }; p.migratedBaseline = true; for (const o of Object.values(p.outputs || {})) { o.stale = true; o.staleReason = 'baseline geometry updated'; } for (const id of Object.keys(p.params || {})) { const lk = p.locks?.[id] || {}; for (const k of Object.keys(p.params[id])) if (!lk[k]) { delete p.params[id][k]; if (p.prov?.[id]) delete p.prov[id][k]; } } }
      return p;
    }
  } catch { /* storage blocked */ }
  return freshProject();
}
function schedule() {
  clearTimeout(saveTimer);
  saveTimer = setTimeout(() => {
    const full = JSON.stringify(store.project, (k, v) => (k === 'geometry' && v?.positions) ? undefined : v);
    try { localStorage.setItem(KEY, full); store.storageWarning = null; return; } catch { /* quota exceeded or storage blocked */ }
    // fall back: keep configuration, results and provenance, drop bulky imported data columns (kept in memory this session)
    try {
      const slim = JSON.stringify(store.project, (k, v) => (k === 'geometry' && v?.positions) ? undefined : (k === 'columns' || k === 'rows' || k === 'points') && (Array.isArray(v) ? v.length > 200 : typeof v === 'object') ? undefined : v);
      localStorage.setItem(KEY, slim);
      if (store.storageWarning !== 'slim') { store.storageWarning = 'slim'; store.emit({ type: 'storage', level: 'warn', msg: 'Device storage is nearly full: the project was saved without the large imported datasets. Export the project (Project page) to keep them.' }); }
    } catch { if (store.storageWarning !== 'none') { store.storageWarning = 'none'; store.emit({ type: 'storage', level: 'fail', msg: 'This browser is not allowing the project to be saved (private mode or storage blocked). Export the project from the Project page before closing.' }); } }
  }, 400);
}

// Simple stable hash for reproducibility records
export function hashObj(o) { const s = JSON.stringify(o, Object.keys(o || {}).sort()); let h = 2166136261; for (let i = 0; i < s.length; i++) { h ^= s.charCodeAt(i); h = Math.imul(h, 16777619); } return (h >>> 0).toString(16).padStart(8, '0'); }
