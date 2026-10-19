// PHOENIX-ATMOS-XP application shell: routing, navigation, site selection, installation, offline support.
import { h, $, modal, toast } from './core/ui.js';
import { store } from './core/state.js';
import { SUITES, GROUPS } from './suites/index.js';
import { geocode, reverseGeocode, onStatus } from './core/live.js';

export const VERSION = '2026.10.19';
window.AXP_VERSION = VERSION;
$('#ver').textContent = 'v' + VERSION;

// ---------- theme ----------
const savedTheme = (() => { try { return localStorage.getItem('axp.theme'); } catch { return null; } })();
if (savedTheme) document.documentElement.dataset.theme = savedTheme;
$('#themeBtn').onclick = () => { const cur = document.documentElement.dataset.theme || (matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark'); const nx = cur === 'light' ? 'dark' : 'light'; document.documentElement.dataset.theme = nx; try { localStorage.setItem('axp.theme', nx); } catch { /* ignore */ } };

// ---------- sidebar ----------
function drawSidebar() {
  const sb = $('#sidebar'); sb.innerHTML = '';
  sb.append(h('a', { href: '#/' }, h('span', { class: 'n' }, '⌂'), 'Engineering console'), h('a', { href: '#/feasibility' }, h('span', { class: 'n' }, '✓'), 'Mandatory feasibility tests'));
  for (const g of GROUPS) {
    sb.append(h('div', { class: 'grp' }, g));
    for (const s of SUITES.filter(x => x.group === g)) { const o = store.outputMeta(s.id); sb.append(h('a', { href: `#/suite/${s.id}`, 'data-id': s.id }, h('span', { class: 'n' }, String(s.num).padStart(2, '0')), h('span', {}, s.icon + ' ' + s.short), h('span', { class: 'st ' + (o ? (o.stale ? 'stale' : 'ok') : ''), title: o ? (o.stale ? 'stale' : 'current') : 'not run' }))); }
  }
}
drawSidebar();
let sbTimer; store.subscribe((e) => { if (['output', 'reset', 'vehicle', 'material', 'site'].includes(e.type)) { clearTimeout(sbTimer); sbTimer = setTimeout(drawSidebar, 200); } if (e.type === 'site' || e.type === 'reset') siteLabel(); });
$('#menuBtn').onclick = () => { $('#sidebar').classList.toggle('open'); $('#topnav').classList.toggle('open'); };

// ---------- router ----------
const PAGES = { '': 'home', feasibility: 'feasibility', scenarios: 'scenarios', portal: 'portal', datahub: 'datahub', dossier: 'dossier', project: 'project', help: 'help', install: 'install' };
let current = null;
async function route() {
  const hash = location.hash.replace(/^#\/?/, ''), [page, arg] = hash.split('/');
  const main = $('#main'); current?.dispose?.(); current = null;
  main.innerHTML = ''; window.scrollTo({ top: 0 });
  $('#sidebar').classList.remove('open'); $('#topnav').classList.remove('open');
  document.querySelectorAll('.sidebar a, .topnav a').forEach(a => a.classList.toggle('on', a.getAttribute('href') === '#/' + hash || (hash === '' && a.getAttribute('href') === '#/')));
  try {
    if (page === 'suite' && arg) { const { renderSuite } = await import('./core/suite.js'); const s = SUITES.find(x => x.id === arg); document.title = s ? `${String(s.num).padStart(2, '0')} ${s.short} · PHOENIX-ATMOS-XP` : 'PHOENIX-ATMOS-XP'; current = await renderSuite(main, arg); }
    else { const name = PAGES[page] ?? 'home'; const mod = await import(`./pages/${name}.js`); document.title = (mod.title ? mod.title + ' · ' : '') + 'PHOENIX-ATMOS-XP'; current = await mod.render(main, arg); }
  } catch (e) { console.error(e); main.append(h('div', { class: 'callout fail' }, 'Could not open this page: ', String(e.message || e))); }
  main.focus({ preventScroll: true });
  updateArrows();
}
window.addEventListener('hashchange', route);
// back / forward arrows (track depth so the buttons can show availability)
let depth = +(sessionStorage.getItem('axp.depth') || 0), maxDepth = +(sessionStorage.getItem('axp.max') || 0);
const navState = () => history.state?.d ?? 0;
window.addEventListener('hashchange', () => { if (history.state?.d === undefined) { depth++; maxDepth = depth; history.replaceState({ d: depth }, ''); } else depth = history.state.d; sessionStorage.setItem('axp.depth', depth); sessionStorage.setItem('axp.max', maxDepth); updateArrows(); });
if (history.state?.d === undefined) history.replaceState({ d: depth }, '');
function updateArrows() { $('#navBack').disabled = navState() <= 0 && location.hash.replace(/^#\/?/, '') === ''; $('#navFwd').disabled = navState() >= maxDepth; }
$('#navBack').onclick = () => { if (navState() > 0) history.back(); else location.hash = '#/'; };
$('#navFwd').onclick = () => history.forward();
window.addEventListener('keydown', (e) => { if (e.altKey && e.key === 'ArrowLeft') { e.preventDefault(); history.back(); } if (e.altKey && e.key === 'ArrowRight') { e.preventDefault(); history.forward(); } });

// ---------- site picker ----------
function siteLabel() { const s = store.project.site; $('#siteName').textContent = s.name || `${s.lat.toFixed(3)}, ${s.lon.toFixed(3)}`; }
siteLabel();
$('#siteBtn').onclick = () => openSitePicker();
export function openSitePicker() {
  const s = store.project.site, results = h('div', { class: 'site-results' });
  const lat = h('input', { type: 'number', step: 'any', value: s.lat, class: 'search', style: 'max-width:160px' }), lon = h('input', { type: 'number', step: 'any', value: s.lon, class: 'search', style: 'max-width:160px' });
  const q = h('input', { type: 'search', class: 'search', placeholder: 'Search any city, town or place worldwide…', onkeydown: (e) => { if (e.key === 'Enter') search(); } });
  const choose = (o) => { store.setSite({ lat: +o.lat, lon: +o.lon, name: o.name, elevation: o.elevation ?? null }); toast(`Site set to ${o.name}`, 'ok'); m.close(); if (location.hash.includes('s05')) route(); };
  async function search() { results.innerHTML = '<div class="spinner-wrap"><div class="spinner"></div>Searching…</div>'; try { const r = await geocode(q.value); results.innerHTML = ''; for (const g of r.data.results || []) results.append(h('button', { class: 'chip', onclick: () => choose({ lat: g.latitude, lon: g.longitude, name: `${g.name}, ${g.admin1 ? g.admin1 + ', ' : ''}${g.country}`, elevation: g.elevation }) }, `${g.name} · ${g.admin1 || ''} ${g.country_code || ''} (${g.latitude.toFixed(2)}, ${g.longitude.toFixed(2)})`)); if (!results.children.length) results.textContent = 'No matches.'; } catch (e) { results.textContent = 'Search unavailable: ' + e.message; } }
  const gps = () => { if (!navigator.geolocation) return toast('Geolocation unsupported on this device', 'fail'); navigator.geolocation.getCurrentPosition(async (pos) => { let name = `GPS ${pos.coords.latitude.toFixed(4)}, ${pos.coords.longitude.toFixed(4)}`; try { const r = await reverseGeocode(pos.coords.latitude, pos.coords.longitude); name = r.data.display_name?.split(',').slice(0, 3).join(',') || name; } catch { /* keep coordinate name */ } choose({ lat: pos.coords.latitude, lon: pos.coords.longitude, name, elevation: pos.coords.altitude }); }, (err) => toast('Location denied: ' + err.message, 'fail'), { enableHighAccuracy: true, timeout: 15000 }); };
  const m = modal('Project site — any location on Earth', h('div', {}, h('p', { class: 'muted' }, 'The site drives live weather, solar, climatology, grid-carbon and economic context for every suite.'), h('div', { class: 'row gap wrap' }, q, h('button', { class: 'btn primary', onclick: search }, 'Search'), h('button', { class: 'btn', onclick: gps }, '◎ Use my location')), results, h('h4', {}, 'Or enter WGS84 coordinates'), h('div', { class: 'row gap wrap' }, h('label', {}, 'Lat ', lat), h('label', {}, 'Lon ', lon), h('button', { class: 'btn', onclick: () => choose({ lat: +lat.value, lon: +lon.value, name: `${(+lat.value).toFixed(4)}, ${(+lon.value).toFixed(4)}` }) }, 'Use coordinates'))));
  setTimeout(() => q.focus(), 50);
}

// ---------- connectivity ----------
const net = () => { const on = navigator.onLine; $('#netState').classList.toggle('off', !on); $('#netState').title = on ? 'Online — live data enabled' : 'Offline — cached data and full local computation available'; };
window.addEventListener('online', () => { net(); toast('Back online — live data refreshing', 'ok'); }); window.addEventListener('offline', () => { net(); toast('Offline: all solvers keep working with cached data', 'info'); }); net();
onStatus((id, s) => { if (!s.ok) $('#netState').title = `Last data error (${id}): ${s.error}`; });

// ---------- install (PWA) ----------
let deferred = null;
window.addEventListener('beforeinstallprompt', (e) => { e.preventDefault(); deferred = e; $('#installBtn').hidden = false; });
window.addEventListener('appinstalled', () => { $('#installBtn').hidden = true; toast('Installed — PHOENIX-ATMOS-XP now runs offline from your home screen', 'ok', 6000); });
const standalone = matchMedia('(display-mode: standalone)').matches || navigator.standalone;
if (!standalone) $('#installBtn').hidden = false;
$('#installBtn').onclick = async () => { if (deferred) { deferred.prompt(); const r = await deferred.userChoice; deferred = null; if (r.outcome === 'accepted') $('#installBtn').hidden = true; } else location.hash = '#/install'; };
export const installPrompt = () => deferred;

// ---------- service worker (offline, background refresh) ----------
if ('serviceWorker' in navigator && location.protocol !== 'file:') {
  navigator.serviceWorker.register('sw.js').then(async (reg) => {
    if (!reg) return;
    reg.addEventListener('updatefound', () => { const w = reg.installing; w?.addEventListener('statechange', () => { if (w.state === 'installed' && navigator.serviceWorker.controller) toast(h('span', {}, 'A new version is ready. ', h('a', { href: '#', onclick: (e) => { e.preventDefault(); w.postMessage('skipWaiting'); } }, 'Reload now')), 'info', 15000); }); });
    try { if ('periodicSync' in reg) { const st = await navigator.permissions.query({ name: 'periodic-background-sync' }); if (st.state === 'granted') await reg.periodicSync.register('axp-refresh', { minInterval: 6 * 3600e3 }); } } catch { /* optional capability */ }
    const s = store.project.site; reg.active?.postMessage({ type: 'site', lat: s.lat, lon: s.lon });
    setInterval(() => reg.update().catch(() => { }), 3600e3);
  }).catch(e => console.warn('SW registration failed', e));
  let reloaded = false; navigator.serviceWorker.addEventListener('controllerchange', () => { if (!reloaded) { reloaded = true; location.reload(); } });
}
// persistent storage so cached datasets survive storage pressure
navigator.storage?.persist?.().catch(() => { });

route();
void reverseGeocode;
