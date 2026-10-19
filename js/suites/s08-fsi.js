// Suite 08 — Coupled fluid–structure interaction and aeroelastic simulation.
import { WING_PARAMS, buildWing, solveStatic, wingLoads, runGust } from './s06-wing.js';
import { beamResultants, beamModes } from '../core/fem.js';
import { flutterVg } from '../core/aeroelastic.js';
import { range, fmt, deg, max, gci, interp1 } from '../core/num.js';
import { h } from '../core/ui.js';

export default {
  description: 'Partitioned fluid–structure interaction of the passive metamaterial wing: the lifting-line aerodynamic solver and the bend–twist beam finite-element solver exchange interface twist, deflection and loads until both satisfy kinematic continuity and traction equilibrium. Compares coupling algorithms (fixed point, Aitken, interface quasi-Newton IQN-ILS), rigid versus coupled loading, and runs transient gust FSI (Newmark-β structure + Wagner/Küssner indicial aerodynamics) with time-step convergence, energy-transfer and added-mass checks.',
  literature: 'partitioned fluid structure interaction Aitken quasi-Newton IQN-ILS aeroelastic gust response',
  credibility: 'Numerically verified',
  equations: ['Structural momentum balance M ü + C u̇ + K(u) u = F_aero(u, V_a, t) with bend–twist coupled stiffness', 'Aerodynamics: nonlinear lifting line (circulation Γ(z), Kutta–Joukowski l = ρVΓ) in sheared apparent wind', 'Interface conditions: kinematic continuity (aerodynamic surface follows the structural twist/deflection) and traction equilibrium (conservative load mapping)', 'Fixed-point, Aitken Δ² dynamic relaxation and IQN-ILS (least-squares interface quasi-Newton) coupling', 'Newmark-β (average acceleration) time integration of modal coordinates; Wagner and Küssner indicial functions (R. T. Jones exponentials) via recursive Duhamel integration', 'Theodorsen V–g flutter boundary; added-mass ratio μ = m/(πρb²)', 'Energy balance: aerodynamic work = Δ structural energy + damping dissipation'],
  methods: ['Interface variable: spanwise twist vector at beam nodes', 'Loose (staggered, 1 sub-iteration) and strong (implicit, k sub-iterations per step) partitioned transient coupling', 'Three-level time-step refinement with Richardson/GCI on peak root moment'],
  assumptions: ['Arbitrary Lagrangian–Eulerian mesh motion is not required: the lifting line is attached to the deforming structural axis', 'Linear structural dynamics about the static equilibrium; stall-limited lift increments in the gust', 'Non-circulatory (apparent-mass) gust forces neglected relative to circulatory lift at the low reduced frequencies of wind gusts'],
  ibc: { initial: ['Converged static aeroelastic state as initial condition for transient runs (avoids artificial pressure impulses)', 'Zero modal velocities; indicial memory states zero'], boundary: ['Clamped root; kinematic continuity and traction equilibrium at the fluid–structure interface', 'Sheared apparent-wind inflow; 1-cos true-wind gust; free tip'] },
  cvv: { calibration: ['Simultaneous pressure and displacement measurements on a wind-tunnel wing section', 'Measured mode shapes and damping (ground vibration test)'], verification: ['Coupled residual convergence to tolerance for every algorithm', 'Identical converged state from different coupling algorithms', 'Time-step convergence (GCI) of transient peaks', 'Aerodynamic work vs structural energy + damping balance'], validation: ['Transient wing response to measured gusts (wind tunnel gust generator / field anemometry)'] },
  formats: { import: ['CSV measured tip deflection/strain/pressure time series'], export: ['CSV time histories & convergence', 'JSON coupled state', 'VTK/glTF deformed shapes'] },
  params: [
    ...WING_PARAMS,
    { key: 'method', label: 'Coupling algorithm (reported solution)', type: 'select', group: 'FSI coupling', default: 'aitken', options: [{ value: 'fixed', label: 'Fixed-point (constant relaxation)' }, { value: 'aitken', label: 'Aitken dynamic relaxation' }, { value: 'iqn', label: 'IQN-ILS interface quasi-Newton' }] },
    { key: 'omega0', label: 'Initial / constant relaxation ω', type: 'range', min: 0.05, max: 1, step: 0.05, group: 'FSI coupling', default: 0.5 },
    { key: 'tolFsi', label: 'Interface tolerance', type: 'number', min: 1e-10, max: 1e-3, step: 1e-9, group: 'FSI coupling', default: 1e-8, expert: true },
    { key: 'subIt', label: 'Transient sub-iterations per step', type: 'number', min: 1, max: 6, step: 1, group: 'Transient FSI', default: 3, help: '1 = loosely coupled (staggered); >1 = strongly coupled implicit partitioned.' },
    { key: 'dtFsi', label: 'Time step', unit: 's', type: 'number', min: 0.0005, max: 0.02, step: 0.0005, group: 'Transient FSI', default: 0.004 },
  ],
  presets: [{ label: 'Gentle breeze', values: { U10: 5, gustAmp: 2 } }, { label: 'Design gust 14 m/s', values: { U10: 14, gustAmp: 6, gustDur: 1.2 } }, { label: 'Compliant spar (thin covers)', values: { layup: '20,20,45,-45', tPly: 0.15, boxW: 0.12 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const W = buildWing(p, ctx), { model, props, span } = W, N = model.N, nW = ctx.vehicle.nWings || 2;
    ctx.progress(0.1, 'static FSI: algorithm comparison');
    const runs = {}; for (const m of ['fixed', 'aitken', 'iqn']) { const t0 = performance.now(); runs[m] = solveStatic(W, p, { method: m, tol: p.tolFsi, maxIt: 150 }); runs[m].ms = performance.now() - t0; }
    const S = runs[p.method] || runs.aitken, rigid = solveStatic(W, p, { rigid: true });
    const ll = S.aero.ll, Lc = wingLoads(W, ll, p.rho), Lr = wingLoads(W, rigid.aero.ll, p.rho), res = beamResultants(model, S.sol.u, props), resR = beamResultants(model, rigid.sol.u, props);
    const tipTw = deg(S.sol.phi[N - 1]), tipW = S.sol.w[N - 1];
    const spread = Math.max(...['fixed', 'iqn'].map(m => Math.abs(deg(runs[m].sol.phi[N - 1]) - deg(runs.aitken.sol.phi[N - 1]))));
    ctx.progress(0.35, 'transient FSI');
    const T = Math.max(3, p.gustDur + 2.5), dt = p.dtFsi;
    const Gs = runGust(W, p, ll, { dt, subIt: 1, T }), Gi = runGust(W, p, ll, { dt, subIt: Math.max(1, p.subIt | 0), T });
    ctx.progress(0.6, 'time-step convergence');
    const G2 = runGust(W, p, ll, { dt: dt / 2, subIt: Math.max(1, p.subIt | 0), T }), G4 = runGust(W, p, ll, { dt: dt / 4, subIt: Math.max(1, p.subIt | 0), T });
    const pk = (G) => Math.abs(G.peakMroot || 0), g = gci(pk(G4), pk(G2), pk(Gi), dt / 4, dt / 2, dt), dtErr = Math.abs(pk(Gi) - g.fext) / (Math.abs(g.fext) || 1);
    const Mstatic = res[0].M, Mpk = pk(G4), looseErr = Math.abs(pk(Gs) - Mpk) / (Mpk || 1);
    // flutter & added mass
    const modes = beamModes(model, 6), fb = modes.find(m => m.type === 'bending'), ft = modes.find(m => m.type === 'torsion') || modes[1];
    const P75 = props(0.75 * span), b = W.chord(0.75 * span) / 2, FL = flutterVg({ m: P75.m, Ia: P75.Ip, xa: P75.xa / b, a: (P75.xEA - 0.5) * 2, b, wh: fb.omega, wa: ft.omega, rho: p.rho, gs: 2 * p.zeta });
    const Vmax = max(range(N).map(i => W.app(model.z[i]).Va)), mu = P75.m / (Math.PI * p.rho * b * b), fm = FL.Vflutter / Vmax;
    // checks
    for (const m of ['fixed', 'aitken', 'iqn']) out.checks.push({ name: `Static FSI convergence — ${m}`, value: runs[m].history[runs[m].history.length - 1], target: `< ${p.tolFsi}`, pass: runs[m].converged, detail: `${runs[m].iterations} iterations, ${fmt(runs[m].ms, 3)} ms` });
    out.checks.push({ name: 'Algorithm independence of the converged tip twist', value: spread, target: '< 1e-4 °', pass: spread < 1e-4 || (!runs.fixed.converged && Math.abs(deg(runs.iqn.sol.phi[N - 1]) - deg(runs.aitken.sol.phi[N - 1])) < 1e-4) });
    const sumF = S.aero.Fw.reduce((a, c) => a + c, 0); out.checks.push({ name: 'Traction equilibrium: Σ structural nodal force / ∫ l dz', value: sumF / ll.L, target: '1 ± 0.05', pass: Math.abs(sumF / ll.L - 1) < 0.05 });
    out.checks.push({ name: 'Transient energy balance |W_aero − ΔE − W_damp| / scale', value: Gi.energyErr, target: '< 0.05', pass: Gi.energyErr < 0.05 });
    out.checks.push({ name: 'Time-step GCI on peak root moment (Δt/4 level)', value: g.gci21 * 100, target: '< 3 %', pass: g.gci21 < 0.03, detail: `observed order p = ${fmt(g.p, 3)}, extrapolated ${fmt(g.fext, 4)} N·m` });
    out.checks.push({ name: 'Selected-Δt error vs Richardson extrapolation', value: dtErr * 100, target: '< 3 %', pass: dtErr < 0.03 });
    out.checks.push({ name: 'Loose vs strong coupling difference (peak root moment)', value: looseErr * 100, target: '< 2 % (otherwise use sub-iterations)', pass: looseErr < 0.02 });
    // KPIs
    const k = (label, value, unit, explain, status, prov = 'calc') => out.kpis.push({ label, value, unit, explain, status, prov });
    k('Coupling iterations (' + p.method + ')', S.iterations, '', 'Number of aerodynamic ↔ structural exchanges needed to reach the interface tolerance.', S.converged ? 'ok' : 'bad');
    k('Aitken / IQN / fixed iterations', `${runs.aitken.iterations} / ${runs.iqn.iterations} / ${runs.fixed.iterations}`, '', 'Acceleration of convergence by dynamic relaxation and quasi-Newton interface Jacobian approximation.');
    k('Coupled tip twist', tipTw, '°', 'Equilibrium twist after FSI (output of the coupled solve).');
    k('Coupled tip deflection', tipW * 1000, 'mm', 'Equilibrium bending deflection at the tip.');
    k('C_L deformed / rigid', `${fmt(ll.CL, 4)} / ${fmt(rigid.aero.ll.CL, 4)}`, '', 'Lift coefficient with and without fluid–structure coupling.');
    k('Root moment coupled / rigid', Mstatic / (resR[0].M || 1), '', 'Ratio < 1 means the deformation relieves the root load.');
    k('Thrust coupled / rigid', Lc.thrust / (Lr.thrust || 1), '', 'Change of useful propulsion caused by the deformation.');
    k('Gust peak root moment (total)', Math.abs(Mstatic) + Mpk, 'N·m', 'Static moment plus the peak dynamic increment from the strongly coupled transient run.');
    k('Gust peak tip twist increment', max(Gi.twistTip.map(Math.abs)), '°', 'Maximum dynamic twist during the gust (passive gust-load alleviation).');
    k('Flutter margin V_F / V_max', fm, '', 'Ratio of flutter speed to the largest apparent wind along the span; ≥ 1.5 required.', fm >= 1.5 ? 'ok' : 'bad');
    k('Added-mass ratio μ', mu, '', 'm/(πρb²) at 75 % span. Low μ (< ≈ 5) makes loosely coupled partitioned schemes unstable — sub-iterations needed.', mu < 5 ? 'warn' : 'ok');
    k('Time-step discretisation error (selected Δt)', dtErr * 100, '%', 'Error of the reported run against the Richardson-extrapolated peak from three time-step levels (Δt, Δt/2, Δt/4).', dtErr < 0.03 ? 'ok' : 'warn');
    // verdicts
    out.verdicts.push({ test: 'Coupled FSI solution converged', status: S.converged ? 'pass' : 'fail', detail: `${p.method}: ${S.iterations} iterations, residual ${fmt(S.history[S.history.length - 1], 2)}.`, fix: 'Use Aitken or IQN-ILS, reduce ω for fixed-point, or check for static divergence.' });
    out.verdicts.push({ test: 'Flutter margin ≥ 1.5', status: fm >= 1.5 ? 'pass' : fm >= 1.15 ? 'warn' : 'fail', detail: `V_F = ${fmt(FL.Vflutter, 3)} m/s at ${fmt(FL.fFlutter, 3)} Hz; V_max = ${fmt(Vmax, 3)} m/s.` });
    out.verdicts.push({ test: 'Transient coupling adequacy', status: looseErr < 0.02 ? 'pass' : 'warn', detail: `Loosely coupled peak differs by ${fmt(looseErr * 100, 3)} % from strongly coupled (${p.subIt} sub-iterations).` });
    out.verdicts.push({ test: 'Temporal convergence', status: dtErr < 0.03 ? 'pass' : 'warn', detail: `Error at Δt = ${dt} s: ${fmt(dtErr * 100, 3)} %; fine-level GCI ${fmt(g.gci21 * 100, 3)} %, order ${fmt(g.p, 3)}.`, fix: 'Halve the time step until the error is < 3 %.' });
    out.verdicts.push({ test: 'Interface energy conservation', status: Gi.energyErr < 0.05 ? 'pass' : 'warn', detail: `Relative energy imbalance ${fmt(Gi.energyErr * 100, 3)} %.` });
    // charts
    out.charts.push({ title: 'Interface residual: coupling algorithm comparison', type: 'line', x: { label: 'Coupling iteration' }, y: { label: 'Relative twist residual', log: true }, series: ['fixed', 'aitken', 'iqn'].map(m => ({ name: { fixed: 'Fixed-point ω=' + p.omega0, aitken: 'Aitken', iqn: 'IQN-ILS' }[m], x: runs[m].history.map((_, i) => i + 1), y: runs[m].history })), annotations: [{ type: 'hline', y: p.tolFsi, label: 'tolerance', color: '#22c55e' }], explain: 'Convergence of the partitioned static FSI loop. Fewer exchanges = cheaper high-fidelity coupling; all algorithms must reach the same equilibrium.', wide: true });
    out.charts.push({ title: 'Rigid vs coupled spanwise lift', type: 'line', x: { label: 'z', unit: 'm' }, y: { label: 'Lift per span', unit: 'N/m' }, series: [{ name: 'Coupled (deformed)', x: ll.z, y: ll.l }, { name: 'Rigid', x: rigid.aero.ll.z, y: rigid.aero.ll.l, dash: '5 3' }], explain: 'Load redistribution caused by the interaction of the flow with the deforming wing.' });
    out.charts.push({ title: 'Root bending moment: rigid vs coupled distribution', type: 'line', x: { label: 'z', unit: 'm' }, y: { label: 'Bending moment', unit: 'N·m' }, series: [{ name: 'Coupled', x: res.map(r => r.z), y: res.map(r => r.M) }, { name: 'Rigid', x: resR.map(r => r.z), y: resR.map(r => r.M), dash: '5 3' }], explain: 'Internal bending moment along the span transferred to the chassis.' });
    out.charts.push({ title: 'Transient gust FSI — root bending moment (total)', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Root moment', unit: 'N·m' }, series: [{ name: `Strong (${p.subIt} sub-it.)`, x: Gi.t, y: Gi.Mroot.map(v => Mstatic + v) }, { name: 'Loose (staggered)', x: Gs.t, y: Gs.Mroot.map(v => Mstatic + v), dash: '4 3' }, { name: 'Δt / 4', x: G4.t, y: G4.Mroot.map(v => Mstatic + v), dash: '2 3' }], annotations: [{ type: 'hline', y: Mstatic, label: 'static', color: '#94a3b8' }], explain: 'Time history of root moment during the gust from the coupled structure–aerodynamics simulation, with a loosely coupled run and a coarser time step for comparison.', wide: true });
    out.charts.push({ title: 'Transient lift and tip twist', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Value' }, series: [{ name: 'Δ lift [N]', x: Gi.t, y: Gi.lift }, { name: 'Δ tip twist [°×100]', x: Gi.t, y: Gi.twistTip.map(v => v * 100) }, { name: 'Δ tip deflection [mm]', x: Gi.t, y: Gi.wTip.map(v => v * 1000) }, { name: 'gust ΔU [m/s]', x: Gi.t, y: Gi.gust, dash: '4 3' }], explain: 'Aerodynamic lift build-up lags the gust (Küssner) while the structure responds through its bending and torsion modes; twist reduces the effective angle of attack when coupling is favourable.' });
    out.charts.push({ title: 'Structural energy in the gust', type: 'area', x: { label: 'Time', unit: 's' }, y: { label: 'Modal energy', unit: 'J (mass-normalised)' }, series: [{ name: 'E_struct', x: Gi.t, y: Gi.Estr }], explain: 'Kinetic + strain energy of the retained modes; it must decay after the gust when the system is aeroelastically stable.' });
    out.charts.push({ title: 'V–g flutter boundary', type: 'line', x: { label: 'Apparent wind', unit: 'm/s', min: 0, max: Math.min(400, (Number.isFinite(FL.Vflutter) ? FL.Vflutter : 80) * 1.3) }, y: { label: 'g', min: -1, max: 0.4 }, series: FL.branches.map((B, j) => ({ name: 'Branch ' + (j + 1), x: B.map(q => q.V), y: B.map(q => q.g) })), annotations: [{ type: 'vline', x: Vmax, label: 'V_max' }], explain: 'Flutter onset where a branch crosses the structural damping level.' });
    out.tables.push({ title: 'Coupling algorithms', cols: ['Algorithm', 'Converged', 'Iterations', 'Final residual', 'Tip twist [°]', 'Wall time [ms]'], rows: ['fixed', 'aitken', 'iqn'].map(m => [m, runs[m].converged ? 'yes' : 'no', runs[m].iterations, runs[m].history[runs[m].history.length - 1], deg(runs[m].sol.phi[N - 1]), runs[m].ms]) });
    out.tables.push({ title: 'Time-step study (peak dynamic root moment)', cols: ['Δt [s]', 'Peak ΔM [N·m]'], rows: [[dt, pk(Gi)], [dt / 2, pk(G2)], [dt / 4, pk(G4)], ['Richardson extrapolated', g.fext], ['Observed order', g.p]] });
    out.views.push({ label: 'Evolving wing shape (3D playback)', icon: '🎞', render: (el) => playback(el, W, p, S, Gi) });
    if (!runs.fixed.converged) out.recs.push({ level: 'info', text: `Fixed-point coupling with ω = ${p.omega0} did not converge in 150 exchanges; use Aitken (${runs.aitken.iterations}) or IQN-ILS (${runs.iqn.iterations}) for high-fidelity CFD–FEA coupling.`, basis: 'Algorithm comparison' });
    if (looseErr >= 0.02 || mu < 5) out.recs.push({ level: 'warn', text: 'Use strongly coupled (sub-iterated) partitioned FSI for transient gusts on this wing; the added-mass ratio and loose-coupling error indicate staggered schemes are not accurate enough.', basis: 'Added-mass ratio & loose/strong comparison' });
    if (dtErr >= 0.03) out.recs.push({ level: 'warn', text: `Reduce the time step: discretisation error ${fmt(dtErr * 100, 3)} % at Δt = ${dt} s.`, basis: 'Richardson extrapolation' });
    out.recs.push({ level: 'good', text: 'Validate with simultaneous pressure-tap and digital-image-correlation measurements on a wind-tunnel section; keep the calibration and validation datasets separate (Suite 24).', basis: 'Specification calibration dataset for Module 08' });
    out.explain.push({ title: 'What “converged” means here', text: 'The aerodynamic model and the structural model are solved separately and exchange interface data. Convergence means the twist used to compute the loads equals the twist produced by those loads — kinematic continuity and traction equilibrium are both satisfied.' });
    out.convergence = { title: 'Static FSI residuals', series: ['fixed', 'aitken', 'iqn'].map(m => ({ name: m, x: runs[m].history.map((_, i) => i + 1), y: runs[m].history })) };
    out.outputs = { converged: S.converged, iterations: S.iterations, twistTip_deg: tipTw, tipDefl_m: tipW, CLdeformed: ll.CL, CLrigid: rigid.aero.ll.CL, flutterMargin: fm, flutterSpeed: FL.Vflutter, gustPeakRootMoment: Math.abs(Mstatic) + Mpk, rootMoment_Nm: Mstatic, thrustN: Lc.thrust * nW, sideN: Lc.side * nW, rootShear_N: ll.L };
    out.headline = `${p.method} FSI converged in ${S.iterations} exchanges; coupled tip twist ${fmt(tipTw, 3)}°, root moment ${fmt(Mstatic, 4)} N·m (${fmt((Mstatic / (resR[0].M || 1) - 1) * 100, 3)} % vs rigid); gust peak ${fmt(Math.abs(Mstatic) + Mpk, 4)} N·m; flutter margin ${fmt(fm, 3)}.`;
    return out;
  },
};

async function playback(el, W, p, S, G) {
  const [{ viewer }, { wing }] = await Promise.all([import('../core/viz3d.js'), import('../core/cad.js')]);
  const V = await viewer(el, { height: 440 }), span = W.span, modes = G.modes, N = W.model.N;
  const mk = () => wing('Wing (transient)', { airfoil: W.af, span, rootChord: W.cr, tipChord: W.ct, twistTip: p.twistBuilt, nSpan: 14, opts: { color: '#e5e7eb' } });
  const base = mk(), basePos = base.positions; const obj = V.add(base, { field: { name: 'Twist increment', unit: '°', values: new Array(basePos.length / 3).fill(0), min: -1, max: 1, cmap: 'coolwarm' } }); V.fit();
  const wStatic = S.sol.w, phiStatic = S.sol.phi, amp = 30;
  const frame = (k) => {
    const q = G.q[k] || G.q[0]; const w = range(N).map(i => wStatic[i] + modes.reduce((s, m, j) => s + m.full[3 * i] * q[j], 0)), ph = range(N).map(i => phiStatic[i] + modes.reduce((s, m, j) => s + m.full[3 * i + 2] * q[j], 0)), dph = range(N).map(i => deg(ph[i] - phiStatic[i]));
    const P = new Float32Array(basePos.length), vals = []; const xr = 0.25 * W.cr;
    for (let v = 0; v < basePos.length; v += 3) { const z = basePos[v + 2], a = interp1(W.model.z, ph, z), wz = interp1(W.model.z, w, z); const x = basePos[v] - xr, y = basePos[v + 1]; P[v] = xr + x * Math.cos(a) - y * Math.sin(a); P[v + 1] = x * Math.sin(a) + y * Math.cos(a) + wz * amp; P[v + 2] = z; vals.push(interp1(W.model.z, dph, z)); }
    V.updatePositions(obj, P); const mx = Math.max(1e-3, ...G.twistTip.map(Math.abs)); V.updateField(obj, { name: 'Twist increment', unit: '°', values: vals, min: -mx, max: mx, cmap: 'coolwarm' });
    lbl.textContent = `t = ${fmt(G.t[k], 3)} s · gust ΔU = ${fmt(G.gust[k], 3)} m/s · Δ root moment ${fmt(G.Mroot[k], 4)} N·m · Δ tip twist ${fmt(G.twistTip[k], 3)}°`;
  };
  const lbl = h('div', { class: 'small' }), sl = h('input', { type: 'range', min: 0, max: G.t.length - 1, value: 0, style: 'width:100%', oninput: () => { playing = false; frame(+sl.value); } });
  let playing = true, k = 0, acc = 0; V.animate = (dt) => { if (!playing) return; acc += dt; if (acc > 0.05) { acc = 0; k = (k + 1) % G.t.length; sl.value = k; frame(k); } };
  el.append(sl, lbl, h('button', { class: 'btn sm', onclick: (e) => { playing = !playing; e.target.textContent = playing ? 'Pause' : 'Play'; } }, 'Pause'), h('p', { class: 'small muted' }, `Solved transient shapes (modal superposition about the static equilibrium). Deflection drawn ×${amp}; twist at true scale; colour = twist increment.`));
  frame(0);
}
