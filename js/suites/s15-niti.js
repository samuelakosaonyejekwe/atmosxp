// Suite 15 — NiTi shape-memory alloy metamaterial and thermomechanical conversion.
import * as TH from '../core/thermo.js';
import { linspace, fmt, range, KELVIN, max } from '../core/num.js';
import { MATERIALS, get } from '../core/materials.js';
import { tornado } from '../core/analysis.js';

const SMAS = MATERIALS.filter(m => m.cls === 'sma');

// Full finite-time evaluation for one parameter set (also used by sweeps, sensitivity and Suites 16/17)
export function evaluateSMA(mat, q) {
  const { Th, Tc, sH, sL, t_mm, hc, dwell = 0.1 } = q;
  const tH = TH.smaCycle(mat, { Th, Tc, sH, sL }).temps; // stress-dependent transformation temperatures
  const span = Math.max(0.01, Th - Tc);
  const ThE = Math.min(Th - 0.02 * span, tH.H.Af + 1), TcE = Math.max(Tc + 0.02 * span, tH.L.Mf - 1);
  const cyc = TH.smaCycle(mat, { Th: Math.max(ThE, TcE + 1e-3), Tc: TcE, sH, sL });
  const ideal = TH.smaCycle(mat, { Th, Tc, sH, sL });
  const tim = TH.lamellaTiming(mat, { t: t_mm / 1000, hc, Th, Tc, Ttarget_h: ThE, Ttarget_c: TcE, latentDxi: cyc.dXiHeat, dT_tr: Math.max(3, mat.Af - mat.As) });
  const period = (tim.tHeat + tim.tCool) * (1 + dwell), f = Number.isFinite(period) && period > 0 ? 1 / period : 0;
  const P_kg = Math.max(0, cyc.w_J_kg) * f;
  const epsTr = mat.epsL * Math.max(cyc.dXiHeat, 0);
  return { cyc, ideal, tim, f, P_kg, ThE, TcE, temps: tH, epsTr, fatigue: TH.smaFatigue(epsTr, sH), etaC: Math.max(0, 1 - (Tc + KELVIN) / (Th + KELVIN)) };
}

export default {
  description: 'Thermomechanical simulation of architected NiTi (and other SMA) actuator elements: stress-dependent phase transformation, complete heating–loading–cooling–unloading cycles at the solved interface temperatures, net work per cycle, thermal-diffusion-limited cycle frequency, continuous power density, efficiency against the Carnot bound, and functional/structural fatigue. A configuration that cannot complete a repeatable net-positive cycle is classified incapable of sustained propulsion.',
  literature: 'NiTi shape memory alloy heat engine elastocaloric cycle work output frequency fatigue',
  sustain: true, credibility: 'Analytical screening',
  equations: [
    'One-dimensional SMA constitutive law σ = E(ξ)(ε − ε_L ξ − αΔT) with Reuss-averaged modulus 1/E = ξ/E_M + (1−ξ)/E_A (Tanaka / Liang–Rogers / Brinson family)',
    'Clausius–Clapeyron stress dependence of transformation temperatures: M_s(σ) = M_s + σ/C_M, A_s(σ) = A_s + σ/C_A (and finish temperatures)',
    'Cosine phase-fraction kinetics for forward (A→M) and reverse (M→A) transformation with segment-start memory (hysteresis, partial cycles)',
    'Complete thermomechanical cycle work W = −∮σ dε (isobaric heating under σ_H, cooling under σ_L)',
    'Energy balance per cycle: q_in = c_pΔT + L_h Δξ + w; thermal efficiency η = w/q_in ≤ η_Carnot = 1 − T_c/T_h',
    'Transient conduction in a lamella with interface conductance: lumped time constant τ = ρc_p,eff t/h_c (Biot-corrected), latent heat via effective heat capacity',
    'Functional/structural fatigue trend N_f ∝ ε_tr⁻³ σ_H⁻² (calibrate to tests); Coffin–Manson for structural life where data exist',
  ],
  methods: ['Path-discretised integration of the constitutive model over three cycles (last cycle reported) with stress-dependent kinetics', 'Finite-time cycle: temperatures reached in the available residence time replace the reservoir temperatures', 'Parameter sweeps over ΔT and thickness; local sensitivity (tornado) on power density'],
  assumptions: ['Uniaxial, rate-independent transformation (kinetics fast compared with heat transfer)', 'Uniform temperature across the lamella when Bi < 0.1 (Biot correction otherwise)', 'Library transformation temperatures are composition-sensitive typical values; replace with DSC data of the actual batch'],
  ibc: {
    initial: ['Martensite fraction ξ₀ consistent with the stress–temperature history (cooled from above A_f under σ_L)', 'Initial stress (preload σ_L), strain, temperature T_c', 'Residual plastic strain and fatigue damage (virgin = 0)'],
    boundary: ['Time-dependent heating/cooling through the hot/cold interface conductance from Suite 14', 'Prescribed stress levels σ_H (recovery stroke) and σ_L (reset), or mixed control by the cam in Suite 17', 'Thermal cycling between the interface temperatures, never the raw ambient/sky temperatures'],
  },
  cvv: {
    calibration: ['DSC transformation temperatures and latent heat of the actual batch', 'Isobaric strain–temperature tests at several stresses (C_M, C_A, ε_L)', 'Isothermal superelastic tests (E_A, E_M)', 'Cyclic actuation fatigue tests'],
    verification: ['Second-law check η ≤ η_Carnot', 'Cycle closure (repeatability) of strain', 'Work-integral convergence with path refinement', 'Stress-free transformation temperatures reproduced'],
    validation: ['Measured complete heating–loading–cooling–unloading cycles: delivered work, reset work, temperatures, frequency, fatigue — high peak force alone is not validation'],
  },
  formats: { import: ['CSV/XLSX DSC curves', 'CSV force–displacement–temperature cycles', 'JSON'], export: ['CSV cycle paths', 'JSON calibrated parameter set', 'Abaqus INP / Nastran BDF material cards via Suite 02 export manager', 'PNG/SVG'] },
  params: [
    { key: 'grade', label: 'Shape-memory alloy grade', type: 'select', group: 'Material', default: 'niti-lowT', options: SMAS.map(m => ({ value: m.id, label: m.name })), auto: (c) => { const id = c.store.material('actuator', 'niti-lowT'); return { value: SMAS.some(m => m.id === id) ? id : 'niti-lowT', prov: 'library' }; } },
    { key: 'source', label: 'Thermal boundary source', type: 'select', group: 'Thermal', default: 'coupled', options: [{ value: 'coupled', label: 'Coupled interface temperatures (Suite 14)' }, { value: 'imposed', label: 'Imposed laboratory ΔT (not atmospheric)' }] },
    { key: 'Th', label: 'Hot-interface temperature', unit: '°C', type: 'number', min: -20, max: 200, step: 0.5, group: 'Thermal', default: 25, auto: (c) => { const o = c.upstream('s14'); return o ? { value: +o.ThotIface_C.toFixed(2), prov: 'coupled' } : { value: +((c.env?.T ?? 20) + 5).toFixed(1), prov: 'hypothetical' }; } },
    { key: 'Tc', label: 'Cold-interface temperature', unit: '°C', type: 'number', min: -40, max: 150, step: 0.5, group: 'Thermal', default: 20, auto: (c) => { const o = c.upstream('s14'); return o ? { value: +o.TcoldIface_C.toFixed(2), prov: 'coupled' } : { value: +(c.env?.T ?? 20).toFixed(1), prov: 'hypothetical' }; } },
    { key: 'hc', label: 'Interface conductance to lamella', unit: 'W/m²K', type: 'number', min: 100, max: 100000, step: 100, group: 'Thermal', default: 5000 },
    { key: 'sH', label: 'Recovery-stroke stress σ_H', unit: 'MPa', type: 'range', min: 20, max: 500, step: 5, group: 'Mechanical', default: 150 },
    { key: 'sL', label: 'Reset (bias) stress σ_L', unit: 'MPa', type: 'range', min: 0, max: 200, step: 5, group: 'Mechanical', default: 20 },
    { key: 't', label: 'Lamella thickness', unit: 'mm', type: 'number', min: 0.05, max: 5, step: 0.05, group: 'Geometry', default: 0.3, help: 'Thin lamellae/microstruts shorten thermal diffusion time but reduce force per element.' },
    { key: 'w', label: 'Lamella width', unit: 'mm', type: 'number', min: 1, max: 50, step: 0.5, group: 'Geometry', default: 12 },
    { key: 'L', label: 'Active length', unit: 'mm', type: 'number', min: 5, max: 500, step: 1, group: 'Geometry', default: 120 },
    { key: 'nElem', label: 'Active elements (system)', type: 'number', min: 1, max: 2000, step: 1, group: 'Geometry', default: 48, help: '24 cartridges × 2 driven wheels in the reference layout.' },
    { key: 'dwell', label: 'Mechanical loading/unloading time fraction', type: 'range', min: 0, max: 1, step: 0.05, group: 'Mechanical', default: 0.1, expert: true },
    { key: 'Ptarget', label: 'System power target', unit: 'W', type: 'select', group: 'Targets', default: 600, options: [150, 200, 350, 600, 1500] },
  ],
  presets: [{ label: 'Calm night 5 K (293/288 K)', values: { source: 'imposed', Th: 20, Tc: 15 } }, { label: 'Lab 60 °C / 15 °C', values: { source: 'imposed', Th: 60, Tc: 15 } }, { label: 'Solar day 70 °C / 30 °C', values: { source: 'imposed', Th: 70, Tc: 30, grade: 'niti-actuator' } }, { label: 'Low-hysteresis NiTiCu', values: { grade: 'nitcu' } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const mat = get(p.grade); if (!mat) throw new Error('Unknown alloy grade');
    const q = { Th: p.Th, Tc: p.Tc, sH: p.sH, sL: p.sL, t_mm: p.t, hc: p.hc, dwell: p.dwell };
    if (p.Th <= p.Tc) throw new Error('Hot-interface temperature must exceed cold-interface temperature (no heat-engine cycle is possible otherwise).');
    const R = evaluateSMA(mat, q), cyc = R.cyc;
    const V = p.t * p.w * p.L * 1e-9, mElem = mat.rho * V, mTot = mElem * p.nElem;
    const Wcycle = cyc.w_J_m3 * V, Psys = R.P_kg * mTot, Fh = p.sH * p.t * p.w, stroke = cyc.strokeStrain * p.L;
    const netPositive = cyc.w_J_kg > 0.5 && R.cyc.dXiHeat > 0.05 && R.f > 0;
    const Gconv = netPositive ? cyc.qIn_J_kg * mTot * R.f / Math.max(0.1, p.Th - p.Tc) : 1e-3 * mTot * mat.cp / 10;
    const lifeH = R.f > 0 ? R.fatigue / R.f / 3600 : Infinity;
    ctx.progress(0.4, 'sweeps');
    // verification
    out.checks.push({ name: 'Second law: η ≤ η_Carnot', value: cyc.eta, target: `≤ ${fmt(cyc.etaCarnot, 4)}`, pass: cyc.eta <= cyc.etaCarnot + 1e-9 });
    out.checks.push({ name: 'Energy: heat input ≥ work output', value: cyc.qIn_J_kg - Math.max(0, cyc.w_J_kg), target: '≥ 0 J/kg', pass: cyc.qIn_J_kg >= cyc.w_J_kg });
    out.checks.push({ name: 'Cycle repeatability (strain closure)', value: cyc.closure, target: '< 2 × 10⁻³', pass: cyc.closure < 2e-3, detail: 'Third-cycle start/end strain difference' });
    const fine = TH.smaCycle(mat, { Th: Math.max(R.ThE, R.TcE + 1e-3), Tc: R.TcE, sH: p.sH, sL: p.sL, nSeg: 320 }); const conv = Math.abs(fine.w_J_kg - cyc.w_J_kg) / Math.max(1, Math.abs(fine.w_J_kg));
    out.checks.push({ name: 'Work integral path-refinement convergence (80 → 320 points)', value: conv, target: '< 5 %', pass: conv < 0.05, detail: `${fmt(cyc.w_J_kg, 4)} vs ${fmt(fine.w_J_kg, 4)} J/kg` });
    const sm = new TH.SMA(mat); sm.xi = 0; sm.update(mat.Ms + 1e-6, 0); const xiMs = sm.xi; sm.update(mat.Mf - 0.01, 0);
    out.checks.push({ name: 'Stress-free kinetics: ξ(M_s) = 0, ξ(M_f) = 1', value: `${fmt(xiMs, 3)} / ${fmt(sm.xi, 3)}`, target: '0 / 1', pass: xiMs < 1e-3 && sm.xi > 0.999 });
    out.checks.push({ name: 'Biot number of lamella', value: R.tim.Bi, target: '< 0.1 for lumped validity', pass: R.tim.Bi < 0.1, detail: R.tim.Bi >= 0.1 ? 'Biot correction applied' : '' });
    // sweeps
    const dTs = linspace(1, 80, 60), wvsDT = dTs.map(d => evaluateSMA(mat, { ...q, Th: p.Tc + d }));
    const thr = dTs.find((d, i) => wvsDT[i].cyc.w_J_kg > 0.5 && wvsDT[i].cyc.dXiHeat > 0.05);
    const ts = linspace(0.05, 2, 40), pvsT = ts.map(t => evaluateSMA(mat, { ...q, t_mm: t }).P_kg);
    await ctx.yield();
    const sens = tornado((x) => evaluateSMA(mat, { ...q, ...x }).P_kg, { Th: p.Th, Tc: p.Tc, sH: p.sH, sL: p.sL, t_mm: p.t, hc: p.hc }, [{ key: 'Th', label: 'Hot temperature ±5 K', lo: p.Th - 5, hi: p.Th + 5 }, { key: 'Tc', label: 'Cold temperature ±5 K', lo: p.Tc - 5, hi: p.Tc + 5 }, { key: 'sH', label: 'σ_H ±30 %', lo: p.sH * 0.7, hi: p.sH * 1.3 }, { key: 'sL', label: 'σ_L ±50 %', lo: p.sL * 0.5, hi: p.sL * 1.5 + 1 }, { key: 't_mm', label: 'Thickness ±50 %', lo: p.t * 0.5, hi: p.t * 1.5 }, { key: 'hc', label: 'Interface conductance ±50 %', lo: p.hc * 0.5, hi: p.hc * 1.5 }]);
    // KPIs
    const K = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    const prov = p.source === 'coupled' ? (ctx.upstream('s14') ? 'coupled' : 'hypothetical') : 'user';
    K('Interface ΔT', p.Th - p.Tc, 'K', p.source === 'imposed' ? 'Imposed laboratory temperature difference — NOT evidence of atmospheric operation.' : 'Solved interface temperature difference from Suite 14 (or a hypothetical 5 K if not yet run).', prov, p.Th - p.Tc > 15 ? 'ok' : 'bad');
    K('A_s / A_f under σ_H', `${fmt(R.temps.H.As, 3)} / ${fmt(R.temps.H.Af, 3)}`, '°C', 'Reverse-transformation window shifted upward by the recovery stress (Clausius–Clapeyron). The hot side must exceed A_s(σ_H).', 'calc');
    K('M_s / M_f under σ_L', `${fmt(R.temps.L.Ms, 3)} / ${fmt(R.temps.L.Mf, 3)}`, '°C', 'Forward-transformation window at the reset stress. The cold side must fall below M_s(σ_L).', 'calc');
    K('Recovered phase fraction per cycle Δξ', cyc.dXiHeat, '', 'Fraction of the material that actually transforms in the available time.', 'calc', cyc.dXiHeat > 0.5 ? 'ok' : cyc.dXiHeat > 0.05 ? 'warn' : 'bad');
    K('Net work per cycle', cyc.w_J_kg, 'J/kg', 'Area enclosed by the σ–ε cycle per kg after reset work and hysteresis.', 'calc', netPositive ? 'ok' : 'bad');
    K('Net work per element per cycle', Wcycle, 'J', `Element ${p.t}×${p.w}×${p.L} mm (${fmt(mElem * 1000, 3)} g). Hub target ≈ 3.54 J per cartridge per revolution.`, 'calc', Wcycle >= 3.54 ? 'ok' : 'warn');
    K('Achievable cycle frequency', R.f, 'Hz', `Heating ${fmt(R.tim.tHeat, 3)} s + cooling ${fmt(R.tim.tCool, 3)} s (τ = ${fmt(R.tim.tau, 3)} s). 40 km/h hub operation needs 3.54 Hz.`, 'calc', R.f >= 3.54 ? 'ok' : 'bad');
    K('Continuous power density', R.P_kg, 'W/kg', 'Net work × frequency. Laboratory NiTi heat engines report ≈ 52 W/kg under strong thermal gradients.', 'calc');
    K('System mechanical power', Psys, 'W', `${p.nElem} elements, ${fmt(mTot, 3)} kg active NiTi. Target ${p.Ptarget} W.`, 'calc', Psys >= p.Ptarget ? 'ok' : 'bad');
    K('Thermal efficiency', cyc.eta * 100, '%', 'Work / heat input per cycle (sensible + latent + work).', 'calc');
    K('Fraction of Carnot', cyc.etaCarnot ? cyc.eta / cyc.etaCarnot * 100 : 0, '%', `Carnot limit at the interface temperatures = ${fmt(cyc.etaCarnot * 100, 3)} %.`, 'calc');
    K('Recovery force / stroke per element', `${fmt(Fh, 4)} N / ${fmt(stroke, 3)} mm`, '', 'High force alone is not propulsion: only the closed-cycle net work counts.', 'calc');
    K('Estimated actuation fatigue life', R.fatigue, 'cycles', `≈ ${fmt(lifeH, 3)} h of continuous cycling at ${fmt(R.f, 3)} Hz (estimate — calibrate with cyclic tests).`, 'estimated', R.fatigue > 1e7 ? 'ok' : R.fatigue > 1e5 ? 'warn' : 'bad');
    K('Minimum ΔT for net-positive cycling', thr ?? NaN, 'K', 'Smallest interface ΔT (cold side fixed) giving a repeatable net-positive cycle at the chosen stresses.', 'calc');
    // verdicts
    out.verdicts.push({ test: 'Repeatable net-positive work cycle', status: netPositive ? 'pass' : 'fail', detail: netPositive ? `${fmt(cyc.w_J_kg, 4)} J/kg per cycle, Δξ = ${fmt(cyc.dXiHeat, 3)}.` : `No repeatable net-positive cycle: ${p.Th < R.temps.H.As ? `hot side ${fmt(p.Th, 3)} °C is below A_s(σ_H) = ${fmt(R.temps.H.As, 3)} °C` : p.Tc > R.temps.L.Ms ? `cold side ${fmt(p.Tc, 3)} °C is above M_s(σ_L) = ${fmt(R.temps.L.Ms, 3)} °C` : 'the transformation and reset work cancel'}. Classified INCAPABLE of sustained thermal propulsion under this condition.`, fix: 'Increase ΔT, choose an alloy whose transformation window and hysteresis fit inside the available ΔT (e.g. low-hysteresis NiTiCu), or reduce σ_H.' });
    out.verdicts.push({ test: 'Cycle frequency compatible with hub speed (3.54 Hz at 40 km/h)', status: R.f >= 3.54 ? 'pass' : R.f >= 1.5 ? 'warn' : 'fail', detail: `${fmt(R.f, 3)} Hz achievable; 0.141 s per sector available at 40 km/h.`, fix: 'Thinner lamellae / microstruts, higher interface conductance (liquid-metal or graphite interfaces), or accept lower speed.' });
    out.verdicts.push({ test: `System power vs ${p.Ptarget} W target`, status: Psys >= p.Ptarget ? 'pass' : 'fail', detail: `${fmt(Psys, 3)} W = ${fmt(Psys / p.Ptarget * 100, 3)} % of target with ${fmt(mTot, 3)} kg active alloy.` });
    out.verdicts.push({ test: 'Second-law compliance', status: cyc.eta <= cyc.etaCarnot + 1e-9 ? 'pass' : 'fail', detail: `η = ${fmt(cyc.eta * 100, 3)} % ≤ η_Carnot = ${fmt(cyc.etaCarnot * 100, 3)} %.` });
    if (p.source === 'imposed') out.verdicts.push({ test: 'Thermal source classification', status: 'warn', detail: 'Imposed laboratory ΔT: this result validates actuator behaviour only, not atmospheric-only feasibility (specification §5, Module 17 experiment modes).' });
    // charts
    const P = cyc.path, Pi = R.ideal.path;
    out.charts.push({ title: 'Stress–strain cycle (finite-time vs quasi-static)', type: 'line', x: { label: 'Strain', unit: '%' }, y: { label: 'Stress', unit: 'MPa' }, series: [{ name: 'Finite-time cycle', x: P.map(r => r.e * 100), y: P.map(r => r.s), width: 2.5 }, { name: 'Quasi-static at full ΔT', x: Pi.map(r => r.e * 100), y: Pi.map(r => r.s), dash: '5 3' }], explain: 'Net work is the enclosed area: contraction (strain decrease) under σ_H while heating, extension under σ_L while cooling. A collapsed loop means no useful work.' });
    out.charts.push({ title: 'Strain–temperature hysteresis', type: 'line', x: { label: 'Temperature', unit: '°C' }, y: { label: 'Strain', unit: '%' }, series: [{ name: 'Cycle path', x: P.map(r => r.T), y: P.map(r => r.e * 100) }], annotations: [{ type: 'vline', x: p.Th, label: 'T_hot' }, { type: 'vline', x: p.Tc, label: 'T_cold', color: '#38bdf8' }], explain: 'Heating under σ_H contracts the element (A transformation); cooling under σ_L elongates it. The horizontal width of the loop is the thermal hysteresis that the ΔT must exceed.' });
    out.charts.push({ title: 'Martensite fraction along the cycle', type: 'line', x: { label: 'Cycle step' }, y: { label: 'ξ (martensite fraction)', min: 0, max: 1.05 }, series: [{ name: 'ξ', x: range(P.length), y: P.map(r => r.xi) }, { name: 'T (normalised)', x: range(P.length), y: P.map(r => (r.T - p.Tc) / ((p.Th - p.Tc) || 1)), dash: '4 3' }], explain: 'Phase-fraction evolution through load → heat → unload → cool. Δξ per cycle determines the transformation strain and work.' });
    const sg = linspace(0, Math.max(p.sH * 1.4, 100), 30), tt = mat;
    out.charts.push({ title: 'Stress–temperature phase diagram with the operating cycle', type: 'line', x: { label: 'Temperature', unit: '°C' }, y: { label: 'Stress', unit: 'MPa' }, series: [{ name: 'M_f(σ)', x: sg.map(s => tt.Mf + s / tt.CM), y: sg, color: '#3b82f6', dash: '3 3' }, { name: 'M_s(σ)', x: sg.map(s => tt.Ms + s / tt.CM), y: sg, color: '#3b82f6' }, { name: 'A_s(σ)', x: sg.map(s => tt.As + s / tt.CA), y: sg, color: '#ef4444' }, { name: 'A_f(σ)', x: sg.map(s => tt.Af + s / tt.CA), y: sg, color: '#ef4444', dash: '3 3' }, { name: 'Cycle', x: P.map(r => r.T), y: P.map(r => r.s), color: '#facc15', width: 3 }], explain: 'The cycle (yellow) must cross the red band (A_s→A_f) while heating and the blue band (M_s→M_f) while cooling. If the interface ΔT is narrower than the gap between the bands, no transformation occurs.' });
    out.charts.push({ title: 'Net work and power density vs interface ΔT', type: 'line', x: { label: 'Interface ΔT', unit: 'K' }, y: { label: 'Value' }, series: [{ name: 'Net work (J/kg)', x: dTs, y: wvsDT.map(r => r.cyc.w_J_kg) }, { name: 'Power density (W/kg)', x: dTs, y: wvsDT.map(r => r.P_kg) }, { name: 'Carnot efficiency ×1000', x: dTs, y: wvsDT.map(r => r.etaC * 1000), dash: '4 3' }], annotations: [{ type: 'vline', x: 5, label: 'calm night ≈ 5 K', color: '#ef4444' }, ...(thr ? [{ type: 'vline', x: thr, label: 'threshold', color: '#22c55e' }] : [])], explain: 'Below the threshold set by hysteresis and stress shift, the alloy cannot cycle at all; above it, work rises then saturates when full transformation is reached.' });
    out.charts.push({ title: 'Power density vs lamella thickness (heat-transfer trade-off)', type: 'line', x: { label: 'Thickness', unit: 'mm' }, y: { label: 'Power density', unit: 'W/kg' }, series: [{ name: 'P/m', x: ts, y: pvsT }], annotations: [{ type: 'vline', x: p.t, label: 'current' }], explain: 'Thinner elements cycle faster (τ ∝ t), raising power per kilogram; force per element falls, so more elements are needed for the same torque.' });
    out.charts.push({ title: 'Sensitivity of power density (local, ± ranges)', type: 'tornado', rows: sens.rows, base: sens.base, x: { label: 'Power density', unit: 'W/kg' }, explain: 'Which inputs most change continuous power density. Temperature inputs usually dominate — the environment, not the alloy, sets the ceiling.' });
    out.charts.push({ title: 'Power density context (W/kg)', type: 'bar', horizontal: true, categories: ['This design', 'Lab NiTi heat engine (literature)', `Needed for ${p.Ptarget} W with ${fmt(mTot, 3)} kg`], series: [{ name: 'W/kg', values: [R.P_kg, 52, p.Ptarget / mTot], colors: ['#38bdf8', '#94a3b8', '#ef4444'] }], y: { label: 'Power density', unit: 'W/kg' }, showValues: true, explain: 'Literature values come from strong laboratory gradients; atmospheric gradients are much weaker.' });
    // tables
    out.tables.push({ title: 'Transformation temperatures (°C)', cols: ['State', 'M_f', 'M_s', 'A_s', 'A_f'], rows: [['Stress-free (library)', mat.Mf, mat.Ms, mat.As, mat.Af], [`σ_L = ${p.sL} MPa`, R.temps.L.Mf, R.temps.L.Ms, R.temps.L.As, R.temps.L.Af], [`σ_H = ${p.sH} MPa`, R.temps.H.Mf, R.temps.H.Ms, R.temps.H.As, R.temps.H.Af]] });
    out.tables.push({ title: 'Cycle energy accounting (per kg)', cols: ['Quantity', 'Value', 'Unit'], rows: [['Net work', cyc.w_J_kg, 'J/kg'], ['Sensible heat c_pΔT', mat.cp * (p.Th - p.Tc), 'J/kg'], ['Latent heat L_hΔξ', mat.Lh * cyc.dXiHeat, 'J/kg'], ['Total heat input', cyc.qIn_J_kg, 'J/kg'], ['Efficiency', cyc.eta * 100, '%'], ['Carnot limit', cyc.etaCarnot * 100, '%'], ['Heat demand at system scale', cyc.qIn_J_kg * mTot * R.f, 'W']] });
    out.tables.push({ title: 'Experimental and numerical references (literature context, not predictions of this design)', cols: ['Source', 'Reported result', 'Conditions'], rows: [['NiTi heat engine (laboratory)', '≈ 52 W/kg mechanical', 'Strong hot/cold reservoirs'], ['NiTi heat engine on geothermal source (2024)', '3.5 / 8.5 / 11.5 W', 'Source at 60 / 80 / 90 °C'], ['Optimised SMA thermoelastic harvesting (2025, numerical)', '8.6 % efficiency', '55 K temperature difference'], ['Calm night reference (this specification)', 'Carnot 1.71 %', '293 K / 288 K']] });
    // recs
    if (!netPositive) out.recs.push({ level: 'critical', text: `No net-positive cycle at ΔT = ${fmt(p.Th - p.Tc, 3)} K. Minimum workable ΔT for this alloy and stresses ≈ ${thr ? fmt(thr, 3) + ' K' : '> 80 K'}. Match transformation temperatures to the actual interface temperatures (composition/heat-treatment) and use low-hysteresis grades.`, basis: 'Phase diagram and ΔT sweep' });
    if (R.f < 3.54) out.recs.push({ level: 'warn', text: `Thermal cycling limits frequency to ${fmt(R.f, 3)} Hz. Halving thickness roughly halves τ; raise interface conductance with conformal thermal interfaces.`, basis: 'Lamella timing τ = ρc_p t/h_c' });
    if (R.fatigue < 1e7) out.recs.push({ level: 'warn', text: `Fatigue life ≈ ${fmt(R.fatigue, 3)} cycles is short for a vehicle (10⁸–10⁹ cycles over its life at Hz rates). Reduce transformation strain below ≈ 2 % and σ_H, and train/stabilise the alloy.`, basis: 'Functional fatigue trend (calibrate)' });
    out.recs.push({ level: 'good', text: 'Before hardware: measure DSC transformation temperatures, isobaric strain–temperature loops at σ_H and σ_L, and a full heat-engine cycle on a single cartridge with all heat flows metered (Gate 2).', basis: 'Specification development gates' });
    out.recs.push({ level: 'sustain', text: 'NiTi has high embodied energy (≈ 900 MJ/kg); maximise fatigue life and design cartridges for replacement and alloy recovery rather than disposal.', basis: 'Materials registry life-cycle data' });
    out.explain.push({ title: 'What “capable of propulsion” means here', text: 'A shape-memory element can produce a large recovery force on a single heating. Sustained propulsion requires repeated complete cycles whose work output exceeds the reset work, at a frequency the heat transfer can sustain, with heat supplied by the environment. All four conditions are evaluated; failing any one classifies the configuration as incapable under the simulated condition.' });
    out.outputs = { wNet_J_kg: cyc.w_J_kg, Wcycle_J: Wcycle, maxFreqHz: R.f, powerDensity_W_kg: R.P_kg, efficiency: cyc.eta, carnotFrac: cyc.etaCarnot ? cyc.eta / cyc.etaCarnot : 0, fatigueCycles: R.fatigue, netPositive, recoveryStress_MPa: p.sH, Gconv_W_K: Gconv, grade: p.grade, Th: p.Th, Tc: p.Tc, sL: p.sL, elem: { t: p.t, w: p.w, L: p.L, mass: mElem }, activeMass_kg: mTot, systemPower_W: Psys, tau_s: R.tim.tau, source: p.source, heatPerCycle_J_kg: cyc.qIn_J_kg };
    out.headline = netPositive ? `Net-positive cycle: ${fmt(cyc.w_J_kg, 4)} J/kg at ${fmt(R.f, 3)} Hz → ${fmt(R.P_kg, 3)} W/kg; system ${fmt(Psys, 3)} W vs ${p.Ptarget} W target.` : `INCAPABLE under this condition: no repeatable net-positive cycle at ΔT = ${fmt(p.Th - p.Tc, 3)} K (threshold ≈ ${thr ? fmt(thr, 3) : '>80'} K).`;
    out.headlineKind = netPositive && Psys >= p.Ptarget ? 'ok' : netPositive ? 'warn' : 'fail';
    void max;
    return out;
  },
};
