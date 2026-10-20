// Suite 08 — Coupled fluid–structure interaction and aeroelastic simulation.
import { WING_PARAMS, buildWing, solveStatic, wingLoads, runGust, sizeSpar, uncoupledOmegas, featherState, archFrame } from './s06-wing.js';
import { beamResultants, beamModes } from '../core/fem.js';
import { flutterVg } from '../core/aeroelastic.js';
import { range, fmt, deg, rad, max, gci, interp1 } from '../core/num.js';
import { h } from '../core/ui.js';
import * as AA from '../core/archaero.js';

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
    const SZ = sizeSpar(p, ctx); Object.assign(p, SZ.values); // follow the Revision-2 spar sizing of Suite 06
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
    const modes = beamModes(model, 40), fb = modes.find(m => m.type === 'bending'), ft = modes.find(m => m.type === 'torsion') || modes[1];
    const P75 = props(0.75 * span), b = W.chord(0.75 * span) / 2, UO = uncoupledOmegas(W), FL = flutterVg({ m: P75.m, Ia: P75.Ip, xa: P75.xa / b, a: (P75.xEA - 0.5) * 2, b, wh: UO.wh, wa: UO.wa, rho: p.rho, gs: 2 * p.zeta });
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
    k('Flutter margin V_F / V_max', Number.isFinite(fm) ? fm : 'no flutter in range', '', 'Ratio of flutter speed to the largest apparent wind along the span; ≥ 1.5 required.', fm >= 1.5 ? 'ok' : 'bad');
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
    out.views.push({ label: '2D animation: interface-residual convergence & section deformation', icon: '🎞', render: (el) => fsi2D(el, W, p, S, Gi) });
    if (ctx.vehicle.wingConfig === 'arch') out.views.unshift({ label: '3D FSI: AeroArch legs + bridge through the gust (timeline)', icon: '🎞', render: (el) => fsiArch3D(el, W, p, S, Gi, ctx, Lc, nW) });
    out.views.unshift({ label: '3D FSI: evolving wing & gust flow (timeline)', icon: '🎞', render: (el) => fsi3D(el, W, p, S, Gi) });
    if (!runs.fixed.converged) out.recs.push({ level: 'info', text: `Fixed-point coupling with ω = ${p.omega0} did not converge in 150 exchanges; use Aitken (${runs.aitken.iterations}) or IQN-ILS (${runs.iqn.iterations}) for high-fidelity CFD–FEA coupling.`, basis: 'Algorithm comparison' });
    if (looseErr >= 0.02 || mu < 5) out.recs.push({ level: 'warn', text: 'Use strongly coupled (sub-iterated) partitioned FSI for transient gusts on this wing; the added-mass ratio and loose-coupling error indicate staggered schemes are not accurate enough.', basis: 'Added-mass ratio & loose/strong comparison' });
    if (dtErr >= 0.03) out.recs.push({ level: 'warn', text: `Reduce the time step: discretisation error ${fmt(dtErr * 100, 3)} % at Δt = ${dt} s.`, basis: 'Richardson extrapolation' });
    out.recs.push({ level: 'good', text: 'Validate with simultaneous pressure-tap and digital-image-correlation measurements on a wind-tunnel section; keep the calibration and validation datasets separate (Suite 24).', basis: 'Specification calibration dataset for Module 08' });
    out.explain.push({ title: 'What “converged” means here', text: 'The aerodynamic model and the structural model are solved separately and exchange interface data. Convergence means the twist used to compute the loads equals the twist produced by those loads — kinematic continuity and traction equilibrium are both satisfied.' });
    out.convergence = { title: 'Static FSI residuals', series: ['fixed', 'aitken', 'iqn'].map(m => ({ name: m, x: runs[m].history.map((_, i) => i + 1), y: runs[m].history })) };
    // Free-feathering breakaway during the gust (D3): side force scales with the solved lift history
    const FSg = featherState(W, p, ctx, Lc, nW), Lpk = ll.L + Math.max(...Gi.lift), sidePk = FSg.side * Lpk / Math.max(1e-9, ll.L), iRel = Gi.lift.findIndex(dl => FSg.side * (ll.L + dl) / Math.max(1e-9, ll.L) > FSg.FySafe);
    const legacyRoot = (ctx.vehicle.wingConfig || 'arch') !== 'arch' || ['ring', 'feathering'].includes(ctx.vehicle.archRoot); // breakaway exists only on the legacy (non-compliant) roots
    if (legacyRoot) out.kpis.push({ label: 'Gust peak side force / breakaway setting', value: `${fmt(sidePk, 3)} / ${fmt(FSg.FySafe, 3)}`, unit: 'N', explain: `Both wings; ${p.wingState === 'reefed' ? 'reefed' : 'deployed'} configuration. Above the setting the torque-limited lock releases and the wings weathervane.`, prov: 'calc', status: sidePk <= FSg.FySafe ? 'ok' : 'warn' });
    if (legacyRoot) out.verdicts.push({ test: 'Gust response with free-feathering breakaway (D3)', status: sidePk <= FSg.FySafe || p.breakaway ? 'pass' : 'fail', detail: sidePk <= FSg.FySafe ? `Peak side force ${fmt(sidePk, 3)} N stays below the breakaway setting ${fmt(FSg.FySafe, 3)} N — wings remain locked through the gust.` : p.breakaway ? `Breakaway releases at t = ${fmt(Gi.t[iRel], 3)} s (side force reaches ${fmt(FSg.FySafe, 3)} N); feathered residual side force ${fmt(FSg.feathered.side, 3)} N — rollover protected, propulsion paused until re-latching.` : `Peak side force ${fmt(sidePk, 3)} N exceeds the safe limit with the breakaway disabled.` });
    // Revision 5 compliant flexure: gust washout. A gust raises V_a and swings β; a locked root would see α rise by Δβ, the
    // self-trimming flexure re-trims (aero pitch mode ≈ f_aero ≫ 1/T_gust → quasi-static) and holds C_L ≈ C_L,trim.
    const s07 = ctx.upstream('s07'), vehF = ctx.vehicle || {}; let gustWash = null;
    if (s07?.flexure && (vehF.wingConfig || 'arch') === 'arch') {
      const GA = AA.archGeometry(vehF), polA = AA.sectionPolar('7418', s07.Re || 5e5), fx = s07.flexure, b0 = Math.max(5, Math.abs(s07.awa ?? 60)), Va0 = s07.Va || 8, FyS = s07.FySafe ?? AA.safeSide(vehF, GA.hCE);
      const fl = { kTheta: fx.kTheta, xAxis: fx.xAxis, Cm0: fx.Cm0, deltaMax: fx.deltaMax ?? 25 }, T0 = AA.trimState(GA, polA, vehF, b0, Va0, 1.225, { ...fl, FyS: Infinity, scanN: 60 });
      const gV = 1.4, gB = 12, b1 = Math.min(175, b0 + gB), Va1 = gV * Va0;
      const lockF = AA.archForces(GA, polA, vehF, b1, T0.alpha + gB, Va1, 1.225), trimF = AA.trimState(GA, polA, vehF, b1, Va1, 1.225, { ...fl, FyS, scanN: 60 }), trimU = AA.trimState(GA, polA, vehF, b1, Va1, 1.225, { ...fl, FyS: Infinity, scanN: 60 });
      const fA = ctx.upstream('s06')?.flexure?.fAero;
      gustWash = { CL0: T0.CL, CLlocked: lockF.CL, CLtrim: trimU.CL, sideLocked: lockF.side, sideFlex: trimF.side, FyS, fAero: fA };
      out.kpis.push({ label: 'Gust washout (flexure): C_L before / locked root / self-trimmed', value: `${fmt(T0.CL, 3)} / ${fmt(lockF.CL, 3)} / ${fmt(trimU.CL, 3)}`, unit: '', explain: `Gust ×${gV} apparent-wind speed and +${gB}° apparent-wind angle at β = ${fmt(b0, 3)}°. A locked root takes the full angle change as incidence; the closed arch on its single centreline pylon flexure re-trims${fA ? ` (aero pitch mode ${fmt(fA, 3)} Hz, quasi-static for gusts > ${fmt(1 / fA, 2)} s)` : ''}.`, prov: 'calc' });
      out.kpis.push({ label: 'Gust side force: locked root / flexure with washout cap / limit', value: `${fmt(lockF.side, 3)} / ${fmt(trimF.side, 3)} / ${fmt(FyS, 3)}`, unit: 'N', explain: 'The flexure response plus passive de-cambering (lift cap) keeps the gust side force at or below the safe value without any release mechanism.', prov: 'calc', status: trimF.side <= 1.02 * FyS ? 'ok' : 'warn' });
      out.verdicts.push({ test: 'Flexure gust washout keeps side force within the safe limit (no release)', status: trimF.side <= 1.02 * FyS ? 'pass' : trimF.side <= 1.2 * FyS ? 'warn' : 'fail', detail: `Locked root ${fmt(lockF.side, 3)} N vs flexure ${fmt(trimF.side, 3)} N (limit ${fmt(FyS, 3)} N); C_L ${fmt(T0.CL, 3)} → ${fmt(trimU.CL, 3)} self-trimmed vs ${fmt(lockF.CL, 3)} locked.`, fix: 'Move the flexure axis forward or soften the flexure so the arch re-trims; lower the de-camber trigger.' });
    }
    out.outputs = { gustWashout: gustWash, converged: S.converged, iterations: S.iterations, twistTip_deg: tipTw, tipDefl_m: tipW, CLdeformed: ll.CL, CLrigid: rigid.aero.ll.CL, flutterMargin: fm, flutterSpeed: FL.Vflutter, gustPeakRootMoment: Math.abs(Mstatic) + Mpk, rootMomentStatic: Math.abs(Mstatic), rootMoment_Nm: Mstatic, thrustN: Lc.thrust * nW, sideN: Lc.side * nW, rootShear_N: ll.L };
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

// ---------------- 3-D FSI timeline: solved transient wing shape + gust-driven flow field ----------------
// Solved: static coupled equilibrium, modal gust response q(t), gust history ΔU(t), lift increment and root moment.
// Displayed flow: lifting-line vortex system carrying the solved lift L(t) in the (gust-modified) apparent wind,
// with a reflection plane at the wing root; particles advected through it. Bending magnified, twist true scale.
async function fsi3D(el, W, p, S, G) {
  const [{ viewer }, { wing }, { playback, particles, hud, legend3d }, { vehicleFlowModel, sampledField }] = await Promise.all([import('../core/viz3d.js'), import('../core/cad.js'), import('../core/sim3d.js'), import('../core/flow3d.js')]);
  el.append(h('p', { class: 'small muted' }, 'Timeline of the solved partitioned FSI gust response: wing shape from the static equilibrium plus modal response q(t); colour = solved twist increment (fixed range over the whole history). Particles move through a flow field rebuilt every 0.05 s from the gust-modified apparent wind and a lifting-line vortex system that carries exactly the solved lift (static + increment). The default wing is very stiff, so deflection and twist increments are magnified (slider; true-scale values shown).'));
  const box = h('div'); el.append(box);
  const V = await viewer(box, { height: 480 }), span = W.span, modes = G.modes, N = W.model.N, Z = W.model.z;
  const m = wing('Wing (transient FSI)', { airfoil: W.af, span, rootChord: W.cr, tipChord: W.ct, twistTip: p.twistBuilt, nSpan: 18, opts: { color: '#e5e7eb' } });
  const base = Float32Array.from(m.p), obj = V.add(m, { field: { name: 'Twist increment', unit: '°', values: new Array(base.length / 3).fill(0), min: -1, max: 1, cmap: 'coolwarm' } });
  // translucent undeformed reference wing so the solved deformation is always visible against it
  const ref = wing('Undeformed reference', { airfoil: W.af, span, rootChord: W.cr, tipChord: W.ct, twistTip: p.twistBuilt, nSpan: 18, opts: { color: '#94a3b8', opacity: 0.22 } }); V.add(ref, { opacity: 0.22 });
  const liftArrow = V.addArrow([0.3 * W.cr, 0, span * 0.55], [0, 1, 0], 0.5, 0x22c55e);
  const plate = new V.THREE.Mesh(new V.THREE.BoxGeometry(W.cr * 2.4, W.cr * 1.4, 0.04), new V.THREE.MeshStandardMaterial({ color: 0x334155, transparent: true, opacity: 0.8 })); plate.position.set(W.cr * 0.5, 0, -0.02); V.root.add(plate);
  const ll = S.aero.ll, Lstatic = ll.L, dU = oneMinus(p), zMid = span / 2;
  const alphaAt = (z, g) => rad(g.awa - W.setAngle + p.twistBuilt * z / span) + interp1(Z, S.sol.phi, z);
  const H = hud(box), leg = legend3d(box, { title: 'Air speed (m/s)', lo: 0, hi: 1 });
  const wTipMax = Math.max(1e-12, ...G.wTip.map(Math.abs)), wStat = Math.abs(S.sol.w[N - 1]), autoAmp = Math.max(1, Math.round(0.1 * span / wTipMax)), ampS = Math.max(1, Math.round(0.05 * span / Math.max(1e-12, wStat)));
  const ampIn = h('input', { type: 'range', min: 1, max: Math.max(1000, 3 * autoAmp), step: 1, value: autoAmp }), ampLab = h('div', { class: 'small mono' }); let amp = autoAmp;
  const lab = () => { amp = +ampIn.value; ampLab.textContent = `Gust increment ×${amp} (auto ×${autoAmp}; true-scale peak Δ tip deflection = ${fmt(wTipMax * 1000, 4)} mm, peak Δ tip twist = ${fmt(Math.max(...G.twistTip.map(Math.abs)), 3)}°) · static shape ×${ampS} (true tip ${fmt(wStat * 1000, 4)} mm)`; };
  ampIn.oninput = lab; lab(); box.before(h('label', {}, 'Deformation magnification ', ampIn), ampLab);
  const bounds = [[-span * 0.8, span * 1.2], [-span * 0.6, span * 0.6], [0.02, span * 1.3]];
  let field = null, lastBuild = -1, P = null;
  const build = (t) => {
    const du = dU(t), air = (z) => { const g = W.appGust(Math.min(span, Math.max(0, z)), du), a = alphaAt(Math.min(span, Math.max(0, z)), g); return [g.Va * Math.cos(a), g.Va * Math.sin(a)]; };
    const i = Math.min(G.t.length - 1, Math.max(0, Math.round(t / (G.t[1] - G.t[0]))));
    const M = vehicleFlowModel({ air, wings: [{ x: 0.25 * W.cr, z0: 0, z1: span, L: Lstatic + G.lift[i], nDir: [-Math.sin(alphaAt(zMid, W.app(zMid))), Math.cos(alphaAt(zMid, W.app(zMid)))] }], rho: p.rho, Va: W.app(zMid).Va, rc: 0.05 * W.cr });
    field = sampledField(M.vel, bounds, [23, 17, 14]); lastBuild = t;
  };
  build(0); const Vref = W.app(zMid).Va, cr0 = 0.7 * Vref, cr1 = 1.15 * Vref + 1.2 * p.gustAmp; leg.set(cr0, cr1);
  P = particles(V, { count: matchMedia('(max-width:600px)').matches ? 900 : 1800, bounds, vel: (x, y, z) => field.sample(x, y, z), colorRange: [cr0, cr1], size: 0.09 });
  // frame the wing (not the particle cloud), looking along the chord so lateral bending is in the image plane
  V.controls.target.set(0.4 * W.cr, 0, span * 0.5); V.camera.position.set(0.4 * W.cr + span * 2.1, -span * 0.6, span * 0.75); V.controls.update();
  const mxTw = Math.max(1e-3, ...G.twistTip.map(Math.abs)); let lastT = 0;
  const pb = playback(box, { duration: G.t[G.t.length - 1], rate: 1, label: 'FSI time', onFrame: (t) => {
    if (Math.abs(t - lastBuild) > 0.05) build(t);
    const k = Math.min(G.t.length - 1, Math.max(0, Math.round(t / (G.t[1] - G.t[0])))), q = G.q[k] || G.q[0];
    const w = range(N).map(i => S.sol.w[i] + modes.reduce((s, md, j) => s + md.full[3 * i] * q[j], 0)), ph = range(N).map(i => S.sol.phi[i] + modes.reduce((s, md, j) => s + md.full[3 * i + 2] * q[j], 0));
    const Pp = obj.geometry.attributes.position.array, vals = new Array(base.length / 3), xr = 0.25 * W.cr;
    for (let v = 0; v < base.length; v += 3) { const z = base[v + 2], a0 = interp1(Z, S.sol.phi, z), w0 = interp1(Z, S.sol.w, z), a = Math.max(-0.45, Math.min(0.45, a0 * ampS + (interp1(Z, ph, z) - a0) * amp)), wz = w0 * ampS + (interp1(Z, w, z) - w0) * amp, x = base[v] - xr, y = base[v + 1]; Pp[v] = xr + x * Math.cos(a) - y * Math.sin(a); Pp[v + 1] = x * Math.sin(a) + y * Math.cos(a) + wz; Pp[v + 2] = z; vals[v / 3] = deg(interp1(Z, ph, z) - a0); }
    obj.geometry.attributes.position.needsUpdate = true; obj.geometry.computeVertexNormals(); V.updateField(obj, { name: 'Twist increment', unit: '°', values: vals, min: -mxTw, max: mxTw, cmap: 'coolwarm' });
    const dt = Math.min(0.03, Math.max(0, t - lastT)) || 0.01; lastT = t; P.step(dt * 4, t);
    { const Lt = Lstatic + G.lift[k], Lmax = Lstatic + Math.max(...G.lift.map(Math.abs)); liftArrow.setLength(Math.max(0.08, 1.4 * span * 0.5 * Math.abs(Lt) / Math.max(1e-9, Lmax)), 0.12, 0.07); }
    H.set([['Gust ΔU (solved input)', fmt(G.gust?.[k] ?? dU(t), 3) + ' m/s'], ['Lift: static + Δ', fmt(Lstatic, 3) + ' + ' + fmt(G.lift[k], 3) + ' N'], ['Δ root moment', fmt(G.Mroot[k], 4) + ' N·m'], ['Δ tip twist', fmt(G.twistTip[k], 3) + '°'], ['Display scales', `static ×${ampS}, gust ×${amp}`]]);
  } });
  return { dispose: () => pb.dispose() };
}
function oneMinus(p) { return (t) => (t < 0.3 || t > 0.3 + p.gustDur) ? 0 : 0.5 * p.gustAmp * (1 - Math.cos(2 * Math.PI * (t - 0.3) / p.gustDur)); }

// ---------------- 2-D animation: coupling convergence then the 75 %-span section moving through the gust ----------------
async function fsi2D(el, W, p, S, G) {
  const { anim2d, axes2d, polyline } = await import('../core/sim3d.js');
  el.append(h('p', { class: 'small muted' }, 'Left: the solved interface residual of the partitioned (Aitken) static coupling, drawn iteration by iteration. Right: the wing section at 75 % span translating (bending) and rotating (twist) through the solved gust response; grey = static equilibrium, deformation magnified (factor shown).'));
  const Hst = S.history, Z = W.model.z, N = Z.length, z75 = 0.75 * W.span, af = W.af.points, c75 = W.chord(z75), T = G.t[G.t.length - 1];
  const wS = interp1(Z, S.sol.w, z75), phS = interp1(Z, S.sol.phi, z75);
  const dyn = G.t.map((_, k) => { const q = G.q[k] || []; return { w: interp1(Z, Z.map((_, j) => q.reduce((a, qk, m) => a + qk * G.modes[m].w[j], 0)), z75), ph: interp1(Z, Z.map((_, j) => q.reduce((a, qk, m) => a + qk * G.modes[m].phi[j], 0)), z75) }; });
  const dMax = Math.max(1e-12, ...dyn.map(d => Math.abs(d.w))), amp = Math.max(1, Math.round(0.25 * c75 / dMax)), aAmp = Math.max(1, Math.min(200, Math.round(0.15 / Math.max(1e-9, ...dyn.map(d => Math.abs(d.ph))))));
  return anim2d(el, { duration: T, rate: 0.5, label: 'Time', height: 380, draw: (g, t, Wd, H, c) => {
    const hw = Wd / 2 - 20, nIt = Math.min(Hst.length, Math.max(1, Math.round(t / Math.min(2, T) * Hst.length)));
    const A = axes2d(g, { x0: 60, y0: 30, w: hw - 70, h: H - 90, xr: [0, Math.max(2, Hst.length)], yr: [-12, 1], xl: 'Coupling iteration', yl: 'log₁₀ residual', fg: c.fg, mute: c.mute });
    polyline(g, Hst.slice(0, nIt).map((r, i) => [A.X(i + 1), A.Y(Math.log10(Math.max(1e-12, r)))]), '#22c55e', 2.5); g.fillStyle = '#facc15'; const rl = Hst[nIt - 1]; g.beginPath(); g.arc(A.X(nIt), A.Y(Math.log10(Math.max(1e-12, rl))), 5, 0, 7); g.fill();
    g.fillStyle = c.fg; g.fillText(`iteration ${nIt}/${Hst.length} · residual ${fmt(rl, 3)}`, 60, 18);
    const tt = t % T, k = Math.min(G.t.length - 1, Math.round(tt / (G.t[1] - G.t[0]))), d = dyn[k], cx = hw + 60 + hw * 0.3, cy = H / 2 - 10, sc = Math.min(hw * 0.55, H * 0.7) / c75;
    const draw = (w, ph, col, fill) => { g.beginPath(); af.forEach(([x, y], i) => { const xr = (x - 0.25) * c75, yr = y * c75, X = cx + sc * (xr * Math.cos(ph) - yr * Math.sin(ph)), Y = cy - sc * (xr * Math.sin(ph) + yr * Math.cos(ph) + w); i ? g.lineTo(X, Y) : g.moveTo(X, Y); }); g.closePath(); if (fill) { g.fillStyle = col; g.fill(); } else { g.strokeStyle = col; g.lineWidth = 1.5; g.stroke(); } };
    draw(0, phS, c.mute, false); draw(d.w * amp, phS + d.ph * aAmp, '#38bdf8cc', true);
    g.fillStyle = c.fg; g.fillText(`75 % span · t = ${fmt(G.t[k], 3)} s · Δw ${fmt(d.w * 1000, 3)} mm (×${amp}) · Δθ ${fmt(deg(d.ph), 3)}° (×${aAmp})`, hw + 40, 18);
  } });
}

// ---------------- 3-D FSI of the AeroArch (both legs + bridge) ----------------
// Solved: per-leg coupled static aeroelastic lift and the modal gust lift history (Suite-06 model), the closed-frame
// FE deflection for the lateral load (the 48-Hz-class sway mode is far above gust frequencies, so the frame responds
// quasi-statically to L(t)); flow particles in the box-wing vortex system carrying the solved lift.
async function fsiArch3D(el, W, p, S, G, ctx, Lc, nW) {
  const [{ vehicleScene, playback, particles, hud, legend3d }, mats, { vehicleFlowModel, sampledField, bodyStations }] = await Promise.all([import('../core/sim3d.js'), import('../core/materials.js'), import('../core/flow3d.js')]);
  const veh = ctx.vehicle, F0 = archFrame(W, p, ctx, Math.abs(Lc.side * nW)), Lstatic = S.aero.ll.L;
  el.append(h('p', { class: 'small muted' }, 'Both arch legs and the bridge deform with the solved closed-frame deflection, scaled at every instant by the solved lift history L(t) of the gust (quasi-static frame response — the sway mode is far above the gust frequency). Particles move through the box-wing vortex system (legs + bridge closure) carrying the solved lift. Deformation magnified (label below); colours fixed over the whole history.'));
  const box = h('div'); el.append(box);
  const materials = Object.fromEntries(Object.keys(mats.COMPONENTS).map(k => [k, ctx.material(k)]));
  const SC = await vehicleScene(box, { vehicle: veh, materials, follow: false, height: 480 }), { V, THREE } = SC, Hd = hud(box);
  const parts = SC.car.children.filter(o => /^Arch (leg (port|starboard) \(skin|top bridge)/.test(o.name)).map(o => ({ o, base: Float32Array.from(o.geometry.attributes.position.array), port: /port/.test(o.name), bridge: /bridge/.test(o.name) }));
  const AM = F0.AM, inc = rad(10), nL = F0.nLeg, nodes = F0.nodes;
  const lerpU = (idx) => { const i = Math.max(0, Math.min(nodes.length - 2, Math.floor(idx))), f = idx - i; return [0, 1].map(d => F0.u[3 * i + d] * (1 - f) + F0.u[3 * (i + 1) + d] * f); };
  const nodeIdx = (P, part) => { if (part.bridge) { const yy = P[1] / Math.cos(inc); return nL + Math.max(0, Math.min(F0.nBr, (AM.bridgeSpan / 2 - yy) / AM.bridgeSpan * F0.nBr)); } const e = Math.max(0, Math.min(1, (P[2] - AM.zRoot) / AM.legHeight)); return part.port ? e * nL : nodes.length - 1 - e * nL; };
  const umax = Math.max(1e-12, ...nodes.map((_, i) => Math.hypot(F0.u[3 * i], F0.u[3 * i + 1]))), dLmax = Math.max(...G.lift.map(Math.abs)), fMax = (Lstatic + dLmax) / Math.max(1e-9, Lstatic), autoAmp = Math.max(1, Math.round(0.1 * AM.legHeight / (umax * fMax)));
  const ampIn = h('input', { type: 'range', min: 1, max: Math.max(1000, 3 * autoAmp), value: autoAmp, step: 1 }), ampLab = h('div', { class: 'small mono' }); const lab = () => { ampLab.textContent = `Deformation ×${ampIn.value} (auto ×${autoAmp}; true-scale peak arch deflection = ${fmt(umax * fMax * 1000, 4)} mm)`; }; ampIn.oninput = lab; lab(); box.before(h('label', {}, 'Deformation magnification ', ampIn), ampLab);
  // flow: canted legs from the CAD meshes, closure at the bridge centre
  const endOf = (pt, hi) => { const P = pt.base; let zx = hi ? -Infinity : Infinity; for (let k = 2; k < P.length; k += 3) zx = hi ? Math.max(zx, P[k]) : Math.min(zx, P[k]); let sx = 0, sy = 0, n = 0; for (let k = 0; k < P.length; k += 3) if (Math.abs(P[k + 2] - zx) < 0.06) { sx += P[k]; sy += P[k + 1]; n++; } return [sx / n, sy / n, zx]; };
  const legs = parts.filter(q => !q.bridge), br = parts.find(q => q.bridge); let cb = [0, 0, AM.zTop]; if (br) { const P = br.base; let sx = 0, sy = 0, sz = 0; for (let k = 0; k < P.length; k += 3) { sx += P[k]; sy += P[k + 1]; sz += P[k + 2]; } cb = [3 * sx / P.length, 3 * sy / P.length, 3 * sz / P.length]; }
  const app = W.app(W.span / 2), Va = app.Va, awa = rad(app.awa), airV = [-Va * Math.cos(awa), Va * Math.sin(awa)];
  const bounds = [[-6, 6], [-5, 5], [0.02, 4.5]]; let field = null, lastB = -1;
  const build = (Lper) => { const M = vehicleFlowModel({ air: () => airV, wings: legs.map(q => { const a = endOf(q, false), b = endOf(q, true); return { x: a[0], ya: a[1], z0: a[2], xb: b[0], yb: b[1], z1: b[2], L: Lper, loading: 'box', closure: cb }; }), body: bodyStations(veh), rho: p.rho, Va, rc: 0.08 }); field = sampledField(M.vel, bounds, [25, 21, 13]); };
  build(Lstatic); const P = particles(V, { count: matchMedia('(max-width:600px)').matches ? 1000 : 2200, bounds, vel: (x, y, z) => field.sample(x, y, z), colorRange: [0.6 * Va, 1.4 * Va + p.gustAmp], size: 0.07 });
  legend3d(box, { title: 'Air speed (m/s)', lo: 0.6 * Va, hi: 1.4 * Va + p.gustAmp });
  const T = G.t[G.t.length - 1]; let lastT = 0;
  V.controls.target.set(cb[0], 0, AM.zRoot + AM.legHeight * 0.5); V.camera.position.set(cb[0] + 6.5, -4.5, 3.2); V.controls.update();
  const pb = playback(box, { duration: T, rate: 0.5, label: 'FSI time', onFrame: (t) => {
    const k = Math.min(G.t.length - 1, Math.max(0, Math.round(t / (G.t[1] - G.t[0])))), Lt = Lstatic + G.lift[k], fac = Lt / Math.max(1e-9, Lstatic), amp = +ampIn.value;
    if (Math.abs(t - lastB) > 0.08) { build(Lt); lastB = t; }
    for (const part of parts) { const A = part.o.geometry.attributes.position.array, B = part.base; for (let v = 0; v < B.length; v += 3) { const d = lerpU(nodeIdx([B[v], B[v + 1], B[v + 2]], part)), uy = d[0] * fac * amp, uz = d[1] * fac * amp; A[v] = B[v] - uy * Math.sin(inc); A[v + 1] = B[v + 1] + uy * Math.cos(inc); A[v + 2] = B[v + 2] + uz; } part.o.geometry.attributes.position.needsUpdate = true; part.o.geometry.computeVertexNormals(); }
    const dt = Math.min(0.03, Math.max(0, t - lastT)) || 0.01; lastT = t; P.step(dt * 2, t);
    SC.setArrow('side', [0, Math.abs(Lc.side * nW) * fac, 0], 0.004); SC.setArrow('thrust', [Lc.thrust * nW * fac, 0, 0], 0.01); SC.setArrow('hub', [0, 0, 0]); SC.setArrow('drag', [0, 0, 0]); SC.setArrow('wind', [airV[0], airV[1], 0], 0.25);
    Hd.set([['Gust ΔU', fmt(G.gust?.[k] ?? 0, 3) + ' m/s'], ['Leg lift: static + Δ', `${fmt(Lstatic, 3)} + ${fmt(G.lift[k], 3)} N`], ['Frame root moment', fmt(F0.rootM * fac, 3) + ' N·m'], ['Cantilever-leg equivalent', fmt(F0.cantM * fac, 3) + ' N·m'], ['Arch sway (true)', fmt(F0.sway * fac * 1000, 4) + ' mm'], ['Display scale', '×' + amp]]);
  } });
  return { dispose: () => pb.dispose() };
}
