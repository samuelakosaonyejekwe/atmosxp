// Data-driven 3D driving playback shared by the vehicle-system suites (01, 10, 11, 12, 19, 20).
// The suites pass solved time histories; this module only interpolates between solver output samples.
import { vehicleScene, playback, hud, legend3d } from './sim3d.js';
import { cmap } from './chart.js';
import { fmt, clamp } from './num.js';
import { h } from './ui.js';

const peakOfU = (a) => Math.max(0, ...Array.from(a).filter(Number.isFinite));
const lerpAt = (arr, i, f) => Array.isArray(arr) || ArrayBuffer.isView(arr) ? (arr[i] + ((arr[Math.min(arr.length - 1, i + 1)] ?? arr[i]) - arr[i]) * f) : arr;
export const MODE_COLORS = { WIND: '#22c55e', ASSIST: '#84cc16', BACKUP: '#a855f7', BUFFER: '#f59e0b', COAST: '#94a3b8', BRAKE: '#ef4444', STOP: '#64748b' };

// track: { t[], v[] (m/s), hdg (compass deg, scalar or []), x?[], y?[] (world m, east/north), steer?[], roll?[], pitch?[],
//          U?[] (wind at wing m/s), dirFrom (deg, scalar or []), thrust?[], side?[], drag?[], hub?[] (N), twist?[] (deg tip),
//          discTemp?[] (°C), wheelLoad?: [[N...] per wheel], hud(i,f,t) -> rows, markers:[{x,y,label,color}] }
export async function drivingSim(el, scene, track, { rate, explain, followCam = true, particlesOn = true, discRange, loadRange, legendTitle, presets } = {}) {
  el.innerHTML = '';
  const box = h('div'); el.append(box);
  const S = await vehicleScene(box, { ...scene, follow: followCam });
  const T = track.t, n = T.length;
  // world positions (integrate heading/speed if not supplied)
  let X = track.x, Y = track.y;
  if (!X) { X = [0]; Y = [0]; for (let i = 1; i < n; i++) { const dt = T[i] - T[i - 1], psi = (90 - lerpAt(track.hdg, i - 1, 0)) * Math.PI / 180, v = 0.5 * (track.v[i] + track.v[i - 1]); X.push(X[i - 1] + v * dt * Math.cos(psi)); Y.push(Y[i - 1] + v * dt * Math.sin(psi)); } }
  const D = [0]; for (let i = 1; i < n; i++) D.push(D[i - 1] + Math.hypot(X[i] - X[i - 1], Y[i] - Y[i - 1]));
  const THREE = S.THREE;
  // ground follows the car so long journeys stay on a textured road surface
  const road = new THREE.GridHelper(120, 60, 0xf1f5f9, 0xcbd5e1); road.rotation.x = Math.PI / 2; road.position.z = 0.004; road.material.transparent = true; road.material.opacity = 0.95; S.V.root.add(road);
  const groundObjs = S.V.root.children.filter(o => o.type === 'GridHelper' || o.geometry?.type === 'PlaneGeometry');
  // markers (e.g. stopping-distance line, start line)
  for (const mk of track.markers || []) {
    const geo = new THREE.BoxGeometry(mk.w || 0.15, mk.len || 6, 0.02); const m = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: mk.color || 0xffffff })); m.position.set(mk.x, mk.y || 0, 0.012); if (mk.rot) m.rotation.z = mk.rot; S.V.root.add(m);
  }
  // wind particles (world frame, log-law over ground with solved wind speed/direction)
  let P = null;
  if (particlesOn && track.U && peakOfU(track.U) > 0.05) {
    let curU = 0, curDir = 0; const Umax = Math.max(4, ...track.U.filter(Number.isFinite));
    P = windParticles(S.V, { count: 2400, half: 20, top: 6.5, colorMax: Umax * 2.2, vel: (x, y, z) => { const sp = curU * Math.log(Math.max(z, 0.06) / 0.05) / Math.log(2.3 / 0.05), to = (curDir + 180) * Math.PI / 180; return [sp * Math.sin(to), sp * Math.cos(to), 0]; } });
    P.setWind = (u, d) => { curU = u; curDir = d; };
  }
  // optional colour fields on discs / tyres (fixed range over the whole history so colours evolve)
  if (track.wheelLoad && !loadRange) { const all = track.wheelLoad.flat().filter(Number.isFinite); loadRange = [Math.min(...all), Math.max(...all)]; }
  if (track.discTemp && !discRange) { const all = track.discTemp.filter(Number.isFinite); discRange = [Math.min(...all), Math.max(...all) + 1e-6]; }
  const discs = [], tyres = [];
  S.car.traverse(o => { if (/^Brake disc/.test(o.name)) discs.push(o); if (/^Tyre \d/.test(o.name)) tyres.push(o); });
  if (track.discTemp) legend3d(box, { title: 'Front brake-disc temperature (solved heat balance)', lo: discRange[0], hi: discRange[1], unit: ' °C', cmapName: 'inferno' });
  if (track.wheelLoad) { legend3d(box, { title: legendTitle || 'Tyre vertical load (solved)', lo: loadRange[0], hi: loadRange[1], unit: ' N', cmapName: 'turbo' }); if (track.discTemp) box.lastChild.style.bottom = '120px'; }
  S.V.camera.near = 0.08; S.V.camera.updateProjectionMatrix();
  S.V.root.traverse(o => { if (o.type === 'Line' || o.type === 'Points') o.frustumCulled = false; });
  const fmaxAll = Math.max(1, ...['thrust', 'side', 'drag', 'hub'].flatMap(k => (track[k] || []).map(v => Math.abs(v)).filter(Number.isFinite))), fScale = 3.4 / fmaxAll;
  // auto-magnification of small solved angles (roll, pitch, wing twist) so motion is visible; true scale reported
  const peakOf = (a) => Array.isArray(a) || ArrayBuffer.isView(a) ? Math.max(0, ...Array.from(a).map(v => Math.abs(v)).filter(Number.isFinite)) : 0;
  const peakAng = Math.max(peakOf(track.roll), peakOf(track.pitch), peakOf(track.twist));
  let mag = peakAng > 0 && peakAng < 4 ? clamp(Math.round(8 / peakAng), 1, 200) : 1;
  const magLab = h('span', { class: 'small' }, '');
  const magIn = h('input', { type: 'range', min: 1, max: 200, value: mag, oninput: (e) => { mag = +e.target.value; magLab.textContent = `Angle magnification ×${mag} (true-scale peak = ${fmt(peakAng, 3)}°)`; } });
  magLab.textContent = `Angle magnification ×${mag} (true-scale peak = ${fmt(peakAng, 3)}°)`;
  const maxV = peakOf(track.v), still = maxV < 0.05;
  const H = hud(box);
  let lastT = 0;
  const pb = playback(box, {
    duration: T[n - 1] - T[0], rate: rate || [1, 2, 4, 8, 16, 60].reduce((b, r) => Math.abs(r - (T[n - 1] - T[0]) / 45) < Math.abs(b - (T[n - 1] - T[0]) / 45) ? r : b, 1), onFrame: (tt) => {
      const t = T[0] + tt; let i = 0, lo = 0, hi = n - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (T[m] > t) hi = m; else lo = m; } i = lo; const f = clamp((t - T[i]) / ((T[i + 1] - T[i]) || 1), 0, 1);
      const x = lerpAt(X, i, f), y = lerpAt(Y, i, f), hdg = lerpAt(track.hdg, i, f), psi = 90 - hdg;
      S.setPose({ x, y, heading: psi, roll: track.roll ? mag * lerpAt(track.roll, i, f) : 0, pitch: track.pitch ? mag * lerpAt(track.pitch, i, f) : 0, steer: track.steer ? lerpAt(track.steer, i, f) : 0, dist: lerpAt(D, i, f) });
      for (const g of groundObjs) g.position.set(Math.round(x / 5) * 5, Math.round(y / 5) * 5, g.type === 'GridHelper' ? 0.03 : -0.06);
      if (track.twist || track.reef) S.setWingTwist(track.twist ? mag * lerpAt(track.twist, i, f) : 0);
      if (track.reef) { const sf = track.reef[i] ?? 1; for (const w of S.wingObjs) { const P2 = w.o.geometry.attributes.position.array; let zmin = Infinity; for (let q = 2; q < P2.length; q += 3) zmin = Math.min(zmin, P2[q]); for (let q = 2; q < P2.length; q += 3) P2[q] = zmin + (P2[q] - zmin) * sf; w.o.geometry.attributes.position.needsUpdate = true; } }
      // forces in body frame (car local x forward, y left)
      const sc = track.forceScale || fScale;
      S.setArrow('thrust', [track.thrust ? lerpAt(track.thrust, i, f) : 0, 0, 0], sc);
      S.setArrow('drag', [track.drag ? -Math.abs(lerpAt(track.drag, i, f)) : 0, 0, 0], sc);
      S.setArrow('hub', [track.hub ? lerpAt(track.hub, i, f) : 0, 0, 0], sc);
      const dir = Array.isArray(track.dirFrom) ? lerpAt(track.dirFrom, i, f) : (track.dirFrom ?? 0), U = track.U ? lerpAt(track.U, i, f) : 0;
      // wind "to" vector in world → body frame
      const to = (dir + 180) * Math.PI / 180, wx = Math.sin(to), wy = Math.cos(to), pr = psi * Math.PI / 180, bx = wx * Math.cos(pr) + wy * Math.sin(pr), by = -wx * Math.sin(pr) + wy * Math.cos(pr);
      const side = track.side ? Math.abs(lerpAt(track.side, i, f)) : 0; S.setArrow('side', [0, Math.sign(by || 1) * side, 0], sc);
      S.setArrow('wind', [bx * U, by * U, 0], 0.25);
      if (P) { P.setWind(U, dir); P.setCenter(x, y); const dtv = Math.min(0.25, Math.max(0.005, Math.abs(tt - lastT))); P.step(dtv, t); }
      if (track.discTemp && discs.length) { const Tc = lerpAt(track.discTemp, i, f), c = cmap((Tc - discRange[0]) / ((discRange[1] - discRange[0]) || 1), 'inferno'); for (const d of discs) { d.material.color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255); d.material.emissive?.setRGB(c[0] / 600, c[1] / 900, 0); } }
      if (track.wheelLoad && tyres.length) tyres.forEach((ty, k) => { const L = track.wheelLoad[k]; if (!L) return; const c = cmap((lerpAt(L, i, f) - loadRange[0]) / ((loadRange[1] - loadRange[0]) || 1), 'turbo'); ty.material.color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255); });
      if (track.hud) H.set((still ? [['STANDSTILL', 'solved speed stays 0', '#ef4444']] : []).concat(track.hud(i, f, t)));
      lastT = tt;
    },
  });
  if (peakAng > 0) el.append(h('div', { class: 'row gap wrap', style: 'margin:6px 2px' }, magLab, magIn));
  if (still) el.append(h('div', { class: 'callout warn' }, h('b', {}, 'The solved vehicle does not move in this case'), ' — the available wind/backup force never exceeds starting resistance, so wheels and body stay at rest (wind particles and force arrows still show the solved loads). ', ...(presets || []).map(ps => h('button', { class: 'chip', onclick: ps.onClick }, ps.label))));
  else if (presets?.length) el.append(h('div', { class: 'row gap wrap' }, h('span', { class: 'small muted' }, 'Other cases: '), ...presets.map(ps => h('button', { class: 'chip', onclick: ps.onClick }, ps.label))));
  if (explain) el.append(h('p', { class: 'small muted', style: 'margin:6px 2px' }, explain));
  return { S, pb };
}

// helper: colour for a mode label
export const modeColor = (name) => MODE_COLORS[name] || '#e5e7eb';
export { fmt };

// Wind tracer particles uniformly seeded in a box that follows the vehicle; advected (RK2) by the solved wind field.
export function windParticles(V, { count = 2000, half = 20, top = 6, vel, colorMax = 12, size = 0.12 }) {
  const THREE = V.THREE, pos = new Float32Array(count * 3), col = new Float32Array(count * 3), age = new Float32Array(count), life = new Float32Array(count);
  let cx = 0, cy = 0; const seed = (i) => { pos[i * 3] = cx + (Math.random() * 2 - 1) * half; pos[i * 3 + 1] = cy + (Math.random() * 2 - 1) * half; pos[i * 3 + 2] = 0.08 + Math.random() * top; age[i] = 0; life[i] = 3 + Math.random() * 6; };
  for (let i = 0; i < count; i++) { seed(i); age[i] = Math.random() * life[i]; }
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3)); g.setAttribute('color', new THREE.BufferAttribute(col, 3));
  const pts = new THREE.Points(g, new THREE.PointsMaterial({ size, vertexColors: true, transparent: true, opacity: 0.9, depthWrite: false })); pts.frustumCulled = false; V.root.add(pts);
  const step = (dt, t) => {
    for (let i = 0; i < count; i++) {
      const k = i * 3; let x = pos[k], y = pos[k + 1], z = pos[k + 2];
      const a = vel(x, y, z, t), b = vel(x + a[0] * dt / 2, y + a[1] * dt / 2, z + a[2] * dt / 2, t);
      x += b[0] * dt; y += b[1] * dt; z += b[2] * dt; age[i] += dt;
      pos[k] = x; pos[k + 1] = y; pos[k + 2] = z;
      if (Math.abs(x - cx) > half || Math.abs(y - cy) > half || z < 0 || z > top + 1 || age[i] > life[i]) seed(i);
      const c = cmap(Math.hypot(b[0], b[1], b[2]) / colorMax, 'turbo'); col[k] = c[0] / 255; col[k + 1] = c[1] / 255; col[k + 2] = c[2] / 255;
    }
    g.attributes.position.needsUpdate = true; g.attributes.color.needsUpdate = true;
  };
  return { pts, step, setCenter: (x, y) => { cx = x; cy = y; } };
}
