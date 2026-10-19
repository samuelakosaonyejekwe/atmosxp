import { h } from '../core/ui.js';
import { SUITES, GROUPS } from '../suites/index.js';
import { store } from '../core/state.js';
import { roadLoad, kmh } from '../core/physics.js';
import { fmt } from '../core/num.js';
export const title = 'Engineering console';

export async function render(root) {
  const v = store.project.vehicle, p20 = roadLoad(kmh(20), v).P, p40 = roadLoad(kmh(40), v).P;
  const hero3d = h('div', { class: 'hero-3d card' });
  root.append(h('section', { class: 'hero' },
    h('div', {},
      h('div', { class: 'suite-group' }, 'Atmospheric Technology for Mobility, Optimisation and Sustainability — Experimental Propulsion'),
      h('h1', {}, 'PHOENIX-ATMOS-XP engineering console'),
      h('p', {}, 'A virtual engineering laboratory for a wind-primary, metamaterial-assisted automobile with no battery, no fuel and no traction motor. Choose any of the 26 coupled suites, let the input generator build a consistent model, run the governing physics, and get explained results, verdicts and recommendations — with live global data for any location.'),
      h('div', { class: 'row gap wrap' }, h('a', { class: 'btn primary', href: '#/feasibility' }, '▶ Run the four mandatory feasibility tests'), h('a', { class: 'btn', href: '#/suite/s05' }, '🌦 Live atmosphere'), h('a', { class: 'btn', href: '#/suite/s02' }, '📐 CAD & meshing'), h('a', { class: 'btn', href: '#/dossier' }, '📘 Component dossier')),
      h('div', { class: 'stat-row', style: 'margin-top:14px' },
        stat('Vehicle mass', `${v.mass} kg`), stat('Wheel power at 20 km/h', `${fmt(p20, 3)} W`), stat('Wheel power at 40 km/h', `${fmt(p40, 3)} W`), stat('Wing reference area', `${v.wingArea} m²`))),
    hero3d));
  loadHero(hero3d);
  root.append(h('section', { class: 'card' }, h('h3', { class: 'card-title' }, 'The five central coupled engines'), flowDiagram(), h('p', { class: 'small muted' }, 'Each arrow is a live data coupling: when an upstream suite is re-run, downstream results are flagged stale and regenerate with the new state. A single whole-vehicle energy and exergy balance governs all coupled computations.')));
  for (const g of GROUPS) {
    root.append(h('div', { class: 'group-title' }, g));
    root.append(h('div', { class: 'suite-grid' }, ...SUITES.filter(s => s.group === g).map(s => { const o = store.outputMeta(s.id); return h('a', { class: 'suite-card', href: `#/suite/${s.id}` }, h('div', { class: 'top' }, h('span', { class: 'ic' }, s.icon), h('span', { class: 'no' }, 'SUITE ' + String(s.num).padStart(2, '0'))), h('b', {}, s.title), h('span', { class: 'd' }, o ? (o.stale ? '⚠ results stale — re-run' : `✔ ${o.credibility} · ${new Date(o.t).toLocaleString()}`) : 'Not yet run — automatic inputs ready')); })));
  }
  root.append(h('section', { class: 'card', style: 'margin-top:18px' }, h('h3', { class: 'card-title' }, 'How to use the console'), h('ol', {},
    h('li', {}, 'Set the project site (📍 in the top bar) — any place on Earth; live weather, solar, climatology, grid-carbon and economic data follow it.'),
    h('li', {}, 'Open a suite. In Automatic mode inputs are generated from coupled upstream results and the materials registry; switch to Guided or Expert to edit, and lock values you want to keep.'),
    h('li', {}, 'Read the verdicts first: failures are shown, never hidden. Every number has a provenance tag and an explanation; every plot has “What this shows”.'),
    h('li', {}, 'Use Recommendations for engineering, sustainability and economics guidance based on the results, live context and current literature.'),
    h('li', {}, 'Import measured data in any suite to calibrate and validate; export evidence packages, CAD (STEP/STL/OBJ/glTF/DXF) and meshes.'),
    h('li', {}, 'Install the app (⤓) to use it offline, including in airplane mode.'))));
}
const stat = (l, v) => h('div', { class: 'kpi' }, h('div', { class: 'kpi-label' }, l), h('div', { class: 'kpi-val' }, v));

async function loadHero(box) {
  box.append(h('div', { class: 'spinner-wrap' }, h('div', { class: 'spinner' }), 'Generating parametric 3D vehicle…'));
  try {
    const [{ vehicleScene, particles, playback, hud }, mats, { apparentWind, wingForces, roadLoad }] = await Promise.all([import('../core/sim3d.js'), import('../core/materials.js'), import('../core/physics.js')]);
    box.innerHTML = '';
    const v = store.project.vehicle, env = store.project.env, U = Math.max(env?.Uz ?? 6, 3), dirFrom = 90, rho = env?.rho ?? 1.225;
    const matMap = Object.fromEntries(Object.keys(mats.COMPONENTS).map(k => [k, mats.componentMaterial(store, k)]));
    const S = await vehicleScene(box, { vehicle: v, materials: matMap, height: 320 });
    const P = particles(S.V, { count: 1500, vel: (x, y, z) => { const u = U * Math.log(Math.max(z, 0.05) / 0.1) / Math.log(10 / 0.1); return [0, -Math.max(0, u), 0]; }, bounds: [[-14, 14], [-14, 14], [0.1, 6]], colorRange: [0, U * 1.4], size: 0.05 });
    const H = hud(box);
    // quasi-steady wind-propulsion solve each frame: m dv/dt = thrust(apparent wind) − road load (beam reach, wing trimmed CL 0.8)
    let vel = 0, x = 0, last = 0;
    playback(box, { duration: 60, label: 't', onFrame: (t) => {
      const dt = Math.min(0.1, Math.max(0, t - last)); if (t < last) { vel = 0; x = 0; S.resetTrail(); } last = t;
      const aw = apparentWind(U, dirFrom, 0, vel), F = wingForces(aw.Va, Math.abs(aw.beta), 0.8, 0.06, v.wingArea, rho), Fr = roadLoad(vel, v, { rho }).F;
      vel = Math.max(0, vel + (F.thrust - Fr) / v.mass * dt); x += vel * dt;
      S.setPose({ x, y: 0, heading: 0, roll: -Math.min(4, F.side / 120), dist: x }); S.setWingTwist(-Math.min(6, F.q / 8));
      S.setArrow('thrust', [F.thrust, 0, 0]); S.setArrow('side', [0, -F.side, 0]); S.setArrow('wind', [0, -U, 0], 0.25); S.setArrow('drag', [-Fr, 0, 0]);
      P.setCenter(x, 0); P.step(dt || 0.016, t);
      H.set([['Speed', (vel * 3.6).toFixed(1) + ' km/h'], ['True wind', U.toFixed(1) + ' m/s (beam)'], ['Apparent wind', aw.Va.toFixed(1) + ' m/s @ ' + Math.abs(aw.beta).toFixed(0) + '°'], ['Wing thrust', F.thrust.toFixed(0) + ' N', '#22c55e'], ['Side force', F.side.toFixed(0) + ' N', '#ef4444']]);
    } });
    box.append(h('div', { class: 'small muted', style: 'padding:6px 4px 0' }, `Live: wind-powered start from rest in ${U.toFixed(1)} m/s beam wind (site data if Suite 05 has run), solved with apparent-wind aerodynamics and road load each frame. Full journeys: Suite 20.`));
  } catch (e) { box.innerHTML = ''; box.append(h('div', { class: 'callout warn' }, '3D preview unavailable on this device: ' + e.message)); }
}

function flowDiagram() {
  const N = (x, y, w, t, id, c) => `<a href="#/suite/${id}"><rect x="${x}" y="${y}" width="${w}" height="34" rx="9" fill="${c}" fill-opacity=".16" stroke="${c}"/><text x="${x + w / 2}" y="${y + 21}" text-anchor="middle" font-size="12" fill="currentColor">${t}</text></a>`;
  const A = (x1, y1, x2, y2) => `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="currentColor" stroke-opacity=".5" stroke-width="1.5" marker-end="url(#ah)"/>`;
  const rows = [
    ['Atmosphere 05', 's05', 'CFD 07', 's07', 'Aeroelastic wing 06', 's06', 'Vehicle dynamics 11', 's11', '#38bdf8'],
    ['Photonic 13', 's13', 'Thermal 14', 's14', 'NiTi 15', 's15', 'Hub drive 17', 's17', '#f97316'],
    ['NiTi 15', 's15', 'Hub drive 17', 's17', 'Wheels 10', 's10', 'Vehicle energy 20', 's20', '#a78bfa'],
    ['Buffer 18', 's18', 'Switching 19', 's19', 'Hub drive 17', 's17', 'Vehicle energy 20', 's20', '#22c55e'],
    ['Structures 09', 's09', 'Safety 23', 's23', 'Manufacturing 22', 's22', 'Economics 26', 's26', '#eab308'],
  ];
  let s = `<svg class="flow" viewBox="0 0 900 ${rows.length * 52 + 10}" style="color:var(--fg)"><defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0,0L10,5L0,10z" fill="currentColor" fill-opacity=".6"/></marker></defs>`;
  rows.forEach((r, i) => { const y = 8 + i * 52, w = 180, xs = [10, 240, 470, 700]; for (let k = 0; k < 4; k++) s += N(xs[k], y, w, r[k * 2], r[k * 2 + 1], r[8]); for (let k = 0; k < 3; k++) s += A(xs[k] + w + 2, y + 17, xs[k + 1] - 4, y + 17); });
  return h('div', { html: s + '</svg>' });
}
