// 3D dynamic-simulation engine: playback timeline (play/pause/scrub/speed/loop/record video),
// articulated vehicle (body pose, spinning & steering wheels, morphing wings), force arrows,
// wind/flow particle systems advected by solved velocity fields, streamlines and field-coloured meshes.
// Every animation is driven by solved time histories passed in by the suites — never decorative.
import { viewer, loadThree } from './viz3d.js';
import { vehicleAssembly, Mesh } from './cad.js';
import { cmap } from './chart.js';
import { h } from './ui.js';
import { fmt, clamp, interp1 } from './num.js';

// ---------------- playback controller ----------------
export function playback(container, { duration, onFrame, onReset, fps = 60, label = 'Simulation time', unit = 's', autoplay = true, rate = 1 }) {
  let t = 0, playing = autoplay, speed = rate, loop = true, raf = 0, last = performance.now(), recorder = null, chunks = [];
  const play = h('button', { class: 'icon-btn', title: 'Play / pause' }, playing ? '⏸' : '▶');
  const slider = h('input', { type: 'range', min: 0, max: 1000, value: 0, style: 'flex:1', 'aria-label': 'Timeline' });
  const tlab = h('span', { class: 'small mono', style: 'min-width:120px;text-align:right' }, '');
  const sp = h('select', { title: 'Playback speed' }, ...[...new Set([0.1, 0.25, 0.5, 1, 2, 4, 8, 16, 60, speed])].sort((a, b) => a - b).map(v => h('option', { value: v, selected: v === speed }, fmt(v, 3) + '×')));
  const loopB = h('button', { class: 'icon-btn on', title: 'Loop' }, '⟲');
  const rec = h('button', { class: 'icon-btn', title: 'Record the animation as a video file (WebM)' }, '⏺ Rec');
  const rst = h('button', { class: 'icon-btn', title: 'Reset the simulation to t = 0' }, '⏮');
  const bar = h('div', { class: 'sim-bar' }, rst, play, slider, tlab, sp, loopB, rec);
  container.append(bar);
  const set = (tt) => { t = clamp(tt, 0, duration); slider.value = Math.round(t / duration * 1000); tlab.textContent = `${label} ${fmt(t, 4)} ${unit}`; onFrame(t); };
  rst.onclick = () => { playing = false; play.textContent = '▶'; onReset?.(); set(0); };
  play.onclick = () => { playing = !playing; play.textContent = playing ? '⏸' : '▶'; last = performance.now(); };
  slider.oninput = () => { set(+slider.value / 1000 * duration); };
  sp.onchange = () => { speed = +sp.value; };
  loopB.onclick = () => { loop = !loop; loopB.classList.toggle('on', loop); };
  rec.onclick = () => {
    const cv = container.querySelector('canvas'); if (!cv?.captureStream || !window.MediaRecorder) { rec.textContent = 'n/a'; return; }
    if (recorder) { recorder.stop(); return; }
    chunks = []; recorder = new MediaRecorder(cv.captureStream(30), { mimeType: MediaRecorder.isTypeSupported('video/webm;codecs=vp9') ? 'video/webm;codecs=vp9' : 'video/webm' });
    recorder.ondataavailable = (e) => e.data.size && chunks.push(e.data);
    recorder.onstop = () => { const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob(chunks, { type: 'video/webm' })); a.download = 'atmosxp-simulation.webm'; a.click(); recorder = null; rec.textContent = '⏺ Rec'; rec.classList.remove('on'); };
    recorder.start(); rec.textContent = '■ Stop'; rec.classList.add('on'); set(0); playing = true; play.textContent = '⏸';
  };
  const tick = (now) => { raf = requestAnimationFrame(tick); const dt = Math.min(0.1, (now - last) / 1000); last = now; if (!playing) return; let nt = t + dt * speed; if (nt > duration) { if (recorder) { recorder.stop(); } if (loop && !recorder) nt = 0; else { nt = duration; playing = false; play.textContent = '▶'; } } set(nt); };
  set(0); raf = requestAnimationFrame(tick);
  let autoPaused = false; // pause while off-screen (battery), resume automatically when visible again
  const io = new IntersectionObserver((e) => { if (!e[0].isIntersecting) { if (playing) { autoPaused = true; playing = false; play.textContent = '▶'; } } else if (autoPaused) { autoPaused = false; playing = true; last = performance.now(); play.textContent = '⏸'; } }); io.observe(container);
  return { set, get t() { return t; }, play: () => { playing = true; play.textContent = '⏸'; }, pause: () => { playing = false; play.textContent = '▶'; }, dispose: () => { cancelAnimationFrame(raf); io.disconnect(); } };
}
// sample a time history {t:[], ...series} at time tt
export const sampleAt = (T, Y, tt) => interp1(T, Y, tt);

// ---------------- HUD overlay ----------------
export function hud(container) { const el = h('div', { class: 'sim-hud' }); container.append(el); return { set: (rows) => { el.innerHTML = rows.map(([k, v, c]) => `<div><span>${k}</span><b${c ? ` style="color:${c}"` : ''}>${v}</b></div>`).join(''); } }; }

// ---------------- articulated vehicle scene ----------------
// opts: vehicle, materials (component → material record), wingIncidence, airfoil, morphLevels: [{twist, camber}] keyframes
export async function vehicleScene(container, { vehicle, materials = {}, wingIncidence = 10, airfoil, twistTip = -4, ground = 400, follow = true, height } = {}) {
  const THREE = await loadThree();
  const V = await viewer(container, { height: height || 460, grid: false });
  const asm = vehicleAssembly(vehicle, { materials, wingIncidence, airfoil, twistTip });
  const car = new THREE.Group(); V.root.add(car);
  const wheels = asm.meta.wheelPos.map(([x, y, steerable]) => { const steerG = new THREE.Group(); steerG.position.set(x, y, vehicle.wheelRadius); const spinG = new THREE.Group(); steerG.add(spinG); car.add(steerG); return { x, y, steerable, steerG, spinG }; });
  const wingObjs = [];
  const makeMesh = (m) => { const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(m.positions, 3)); g.setIndex(new THREE.BufferAttribute(m.indices, 1)); g.computeVertexNormals(); const op = m.opacity ?? 1; const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(m.color), metalness: 0.3, roughness: 0.5, side: THREE.DoubleSide, transparent: op < 1, opacity: op, depthWrite: op >= 1 }); const o = new THREE.Mesh(g, mat); o.name = m.name; o.userData = { group: m.group, component: m.component }; return o; };
  for (const m of asm.parts) {
    const wm = m.name.match(/^(Tyre|Rim|Direct-rim hub ring|Brake disc|Spoke) (\d+)/);
    if (wm) { const w = wheels[+wm[2] - 1]; const o = makeMesh(m); o.position.set(-w.x, -w.y, -vehicle.wheelRadius); (wm[1] === 'Brake disc' ? w.steerG : w.spinG).add(o); continue; }
    const o = makeMesh(m); car.add(o); if (/^Wing \d+ \(skin/.test(m.name)) wingObjs.push({ o, base: Float32Array.from(m.positions), comp: m });
  }
  // ground with road texture lines
  const gnd = new THREE.Mesh(new THREE.PlaneGeometry(ground, ground), new THREE.MeshStandardMaterial({ color: 0x4b5563, roughness: 0.95 })); gnd.position.z = -0.001; V.root.add(gnd);
  const grid = new THREE.GridHelper(ground, ground / 5, 0x6b7280, 0x56606e); grid.rotation.x = Math.PI / 2; grid.position.z = 0.002; V.root.add(grid);
  // force arrows (body frame, attached to the car)
  const arrows = {};
  const arrow = (key, color, origin) => { const a = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(...origin), 1, color, 0.25, 0.14); car.add(a); arrows[key] = a; return a; };
  const zCE = asm.meta.height - asm.meta.span * 0.55;
  arrow('thrust', 0x22c55e, [0, 0, zCE]); arrow('side', 0xef4444, [0, 0, zCE]); arrow('drag', 0xf59e0b, [0, 0, vehicle.wheelRadius + 0.4]); arrow('hub', 0xa855f7, [asm.meta.wheelPos[0][0], 0, vehicle.wheelRadius]); arrow('wind', 0x38bdf8, [0, 0, asm.meta.height + 0.4]);
  // wing morph: rotate wing vertices about the quarter-chord axis by spanwise twist (deg at tip, linear) — applied relative to base geometry
  const setWingTwist = (tipDeg) => { for (const w of wingObjs) { const P = w.o.geometry.attributes.position.array, B = w.base; let zmin = Infinity, zmax = -Infinity, xc = 0, yc = 0, n = 0; for (let k = 0; k < B.length; k += 3) { zmin = Math.min(zmin, B[k + 2]); zmax = Math.max(zmax, B[k + 2]); xc += B[k]; yc += B[k + 1]; n++; } xc /= n; yc /= n; for (let k = 0; k < B.length; k += 3) { const eta = (B[k + 2] - zmin) / ((zmax - zmin) || 1), a = tipDeg * eta * Math.PI / 180, c = Math.cos(a), s = Math.sin(a), dx = B[k] - xc, dy = B[k + 1] - yc; P[k] = xc + c * dx - s * dy; P[k + 1] = yc + s * dx + c * dy; P[k + 2] = B[k + 2]; } w.o.geometry.attributes.position.needsUpdate = true; w.o.geometry.computeVertexNormals(); } };
  const setArrow = (key, vec, scale = 0.004) => { const a = arrows[key]; if (!a) return; const L = Math.hypot(...vec); a.visible = L * scale > 0.02; if (!a.visible) return; a.setDirection(new THREE.Vector3(...vec).normalize()); a.setLength(Math.min(6, L * scale), 0.22, 0.12); };
  // trail of travelled path
  const trailGeo = new THREE.BufferGeometry(), trailMax = 4000, trailPos = new Float32Array(trailMax * 3); trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPos, 3)); trailGeo.setDrawRange(0, 0);
  const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0xfacc15 })); V.root.add(trail); let trailN = 0;
  let wheelRot = 0, lastX = null;
  // pose: x,y (m), heading (deg, 0 = +x), roll/pitch (deg), steer (deg), distance travelled (m) for wheel spin
  const setPose = ({ x = 0, y = 0, heading = 0, roll = 0, pitch = 0, steer = 0, dist }) => {
    car.position.set(x, y, 0); car.rotation.set(roll * Math.PI / 180, pitch * Math.PI / 180, heading * Math.PI / 180, 'ZYX');
    if (dist !== undefined) wheelRot = -dist / vehicle.wheelRadius;
    for (const w of wheels) { w.spinG.rotation.y = wheelRot; w.steerG.rotation.z = w.steerable ? steer * Math.PI / 180 : 0; }
    if (lastX === null || Math.hypot(x - lastX[0], y - lastX[1]) > 0.5) { if (trailN < trailMax) { trailPos.set([x, y, 0.02], trailN * 3); trailN++; trailGeo.setDrawRange(0, trailN); trailGeo.attributes.position.needsUpdate = true; } lastX = [x, y]; }
    if (follow) { const tgt = new THREE.Vector3(x, y, 1.6); const d = V.camera.position.clone().sub(V.controls.target); V.controls.target.copy(tgt); V.camera.position.copy(tgt.clone().add(d)); }
  };
  const resetTrail = () => { trailN = 0; trailGeo.setDrawRange(0, 0); lastX = null; };
  const frame = (dx = 0.9, dy = -1.25, dz = 0.55) => { const box = new THREE.Box3().setFromObject(car), c = box.getCenter(new THREE.Vector3()), sz = box.getSize(new THREE.Vector3()).length() * 1.15; V.controls.target.copy(c); V.camera.position.set(c.x + dx * sz, c.y + dy * sz, c.z + dz * sz); V.camera.far = 2000; V.camera.updateProjectionMatrix(); V.controls.update(); };
  frame(); V.fit = frame; V.viewDir = (x, y, z) => frame(x, y, z);
  return { V, THREE, car, wheels, asm, setPose, setWingTwist, setArrow, resetTrail, wingObjs, colorWings: (vals, name = 'turbo') => { for (const w of wingObjs) { const g = w.o.geometry, n = g.attributes.position.count, col = new Float32Array(n * 3); for (let k = 0; k < n; k++) { const c = cmap(vals(g.attributes.position.array[k * 3 + 2], k), name); col[k * 3] = c[0] / 255; col[k * 3 + 1] = c[1] / 255; col[k * 3 + 2] = c[2] / 255; } g.setAttribute('color', new THREE.BufferAttribute(col, 3)); w.o.material.vertexColors = true; w.o.material.color.set(0xffffff); w.o.material.needsUpdate = true; } } };
}

// ---------------- particle flow system ----------------
// vel(x,y,z,t) -> [u,v,w] m/s in world frame; particles advected (RK2) and recycled at the inflow
export function particles(V, { count = 3000, bounds = [[-20, 20], [-12, 12], [0, 8]], vel, colorRange = [0, 12], size = 0.06, cmapName = 'turbo' }) {
  const THREE = V.THREE, pos = new Float32Array(count * 3), col = new Float32Array(count * 3), age = new Float32Array(count);
  const rnd = (a, b) => a + Math.random() * (b - a);
  for (let i = 0; i < count; i++) { pos[i * 3] = rnd(...bounds[0]); pos[i * 3 + 1] = rnd(...bounds[1]); pos[i * 3 + 2] = rnd(...bounds[2]); age[i] = Math.random() * 6; }
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3)); g.setAttribute('color', new THREE.BufferAttribute(col, 3));
  const pts = new THREE.Points(g, new THREE.PointsMaterial({ size, vertexColors: true, transparent: true, opacity: 0.85, depthWrite: false })); V.root.add(pts);
  let center = [0, 0];
  const step = (dt, t) => {
    for (let i = 0; i < count; i++) {
      const k = i * 3; let x = pos[k], y = pos[k + 1], z = pos[k + 2];
      const a = vel(x, y, z, t), xm = x + a[0] * dt / 2, ym = y + a[1] * dt / 2, zm = z + a[2] * dt / 2, b = vel(xm, ym, zm, t);
      x += b[0] * dt; y += b[1] * dt; z += b[2] * dt; age[i] += dt;
      const out = x < center[0] + bounds[0][0] || x > center[0] + bounds[0][1] || y < center[1] + bounds[1][0] || y > center[1] + bounds[1][1] || z < bounds[2][0] || z > bounds[2][1] || age[i] > 8;
      if (out) { // re-seed upstream (opposite to local velocity)
        const sp = Math.hypot(b[0], b[1]) || 1, ux = b[0] / sp, uy = b[1] / sp, R = Math.max(bounds[0][1], bounds[1][1]);
        x = center[0] - ux * R * 0.9 + (-uy) * rnd(-R, R) * 0.9; y = center[1] - uy * R * 0.9 + ux * rnd(-R, R) * 0.9; z = rnd(...bounds[2]); age[i] = 0;
      }
      pos[k] = x; pos[k + 1] = y; pos[k + 2] = z;
      const s = Math.hypot(...b), c = cmap((s - colorRange[0]) / (colorRange[1] - colorRange[0]), cmapName); col[k] = c[0] / 255; col[k + 1] = c[1] / 255; col[k + 2] = c[2] / 255;
    }
    g.attributes.position.needsUpdate = true; g.attributes.color.needsUpdate = true;
  };
  return { pts, step, setCenter: (x, y) => { center = [x, y]; }, dispose: () => { V.root.remove(pts); g.dispose(); } };
}

// ---------------- streamlines ----------------
export function streamlines(V, seeds, vel, { steps = 200, ds = 0.05, colorRange = [0, 12], cmapName = 'turbo', t = 0 } = {}) {
  const THREE = V.THREE, group = new THREE.Group(); V.root.add(group);
  for (const s of seeds) {
    const P = [], C = []; let [x, y, z] = s;
    for (let i = 0; i < steps; i++) { const v = vel(x, y, z, t), sp = Math.hypot(...v); if (!sp || !Number.isFinite(sp)) break; P.push(x, y, z); const c = cmap((sp - colorRange[0]) / (colorRange[1] - colorRange[0]), cmapName); C.push(c[0] / 255, c[1] / 255, c[2] / 255); x += v[0] / sp * ds; y += v[1] / sp * ds; z += v[2] / sp * ds; }
    if (P.length < 6) continue;
    const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(P, 3)); g.setAttribute('color', new THREE.Float32BufferAttribute(C, 3));
    group.add(new THREE.Line(g, new THREE.LineBasicMaterial({ vertexColors: true })));
  }
  return group;
}

// ---------------- generic field-coloured mesh from a Mesh + per-vertex value function ----------------
export function fieldMesh(V, mesh, valueFn, { cmapName = 'turbo', range, opacity = 1 } = {}) {
  const THREE = V.THREE, g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(mesh.positions, 3)); g.setIndex(new THREE.BufferAttribute(mesh.indices, 1)); g.computeVertexNormals();
  const n = g.attributes.position.count, col = new Float32Array(n * 3); g.setAttribute('color', new THREE.BufferAttribute(col, 3));
  const o = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ vertexColors: true, side: THREE.DoubleSide, roughness: 0.6, metalness: 0.1, transparent: opacity < 1, opacity })); o.name = mesh.name; V.root.add(o);
  const update = (fn = valueFn, rg = range) => { const P = g.attributes.position.array; const vals = new Float32Array(n); for (let k = 0; k < n; k++) vals[k] = fn(P[k * 3], P[k * 3 + 1], P[k * 3 + 2], k); const lo = rg?.[0] ?? Math.min(...vals), hi = rg?.[1] ?? Math.max(...vals); for (let k = 0; k < n; k++) { const c = cmap((vals[k] - lo) / ((hi - lo) || 1), cmapName); col[k * 3] = c[0] / 255; col[k * 3 + 1] = c[1] / 255; col[k * 3 + 2] = c[2] / 255; } g.attributes.color.needsUpdate = true; return { lo, hi }; };
  update(); return { o, update, mesh };
}
// colour legend overlay for 3D scenes
export function legend3d(container, { title, lo, hi, unit = '', cmapName = 'turbo' }) {
  const stops = Array.from({ length: 11 }, (_, i) => { const c = cmap(i / 10, cmapName); return `rgb(${c.map(Math.round).join(',')}) ${i * 10}%`; }).join(',');
  const el = h('div', { class: 'sim-legend' }, h('div', { class: 'small' }, title), h('div', { style: `height:10px;border-radius:4px;background:linear-gradient(90deg,${stops})` }), h('div', { class: 'row', style: 'justify-content:space-between' }, h('span', { class: 'small' }, fmt(lo, 3) + unit), h('span', { class: 'small' }, fmt(hi, 3) + unit)));
  container.append(el); return { set: (l, hh) => { el.lastChild.firstChild.textContent = fmt(l, 3) + unit; el.lastChild.lastChild.textContent = fmt(hh, 3) + unit; } };
}
export { Mesh };

// Displacement magnifier: slider + label "×N (true-scale peak = …)". auto = suggested factor so the solved
// peak displacement becomes clearly visible; returns { get value } and calls onChange(N).
export function magnifier(container, { auto = 1, truePeak = 0, unit = '', max = 500, onChange } = {}) {
  let N = Math.max(1, Math.min(max, Math.round(auto)));
  const lab = h('span', { class: 'small mono' }), sl = h('input', { type: 'range', min: 0, max: Math.log10(max), step: 0.01, value: Math.log10(N), style: 'width:140px', title: 'Displacement magnification' });
  const upd = () => { lab.textContent = `×${N} (true-scale peak = ${fmt(truePeak, 3)}${unit})`; onChange?.(N); };
  sl.oninput = () => { N = Math.max(1, Math.round(10 ** +sl.value)); upd(); };
  container.append(h('div', { class: 'sim-bar', style: 'border-top:0' }, h('span', { class: 'small' }, 'Magnification'), sl, lab)); upd();
  return { get value() { return N; } };
}
// Slow turntable rotation for scenes whose solved state is static (keeps the view alive; geometry unchanged).
export function turntable(V, rate = 0.15) { let a = 0; V.animate = (dt) => { a += dt * rate; V.root.rotation.z = a; }; }
// Flow tracers: small spheres travelling along a polyline at a speed ∝ solved flow magnitude.
export function tracers(V, pts, { n = 8, color = 0xf97316, size = 0.025 } = {}) {
  const THREE = V.THREE, P = pts.map(p => new THREE.Vector3(...p)), seg = [], tot = P.slice(1).reduce((s, p, i) => { const l = p.distanceTo(P[i]); seg.push(l); return s + l; }, 0);
  const balls = Array.from({ length: n }, () => { const m = new THREE.Mesh(new THREE.SphereGeometry(size, 10, 8), new THREE.MeshBasicMaterial({ color })); V.root.add(m); return m; });
  let s0 = 0;
  return { step(dt, speed) { s0 = (s0 + dt * speed) % 1; balls.forEach((b, k) => { let d = ((s0 + k / n) % 1) * tot, i = 0; while (i < seg.length - 1 && d > seg[i]) { d -= seg[i]; i++; } b.position.copy(P[i]).lerp(P[i + 1], Math.min(1, d / (seg[i] || 1))); b.visible = speed > 1e-6; }); } };
}
