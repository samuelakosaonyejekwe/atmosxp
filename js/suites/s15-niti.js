// Suite 15 — NiTi shape-memory alloy metamaterial and thermomechanical conversion.
import * as TH from '../core/thermo.js';
import { measuredBlock } from '../core/measured.js';
import { linspace, fmt, range, KELVIN, max } from '../core/num.js';
import { MATERIALS, get } from '../core/materials.js';
import { tornado } from '../core/analysis.js';
import { regenEffectiveness } from '../core/pcmstore.js';

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

// Design Revision 2 (D5): shift the transformation temperatures by composition / heat treatment so that
// A_f(σ_H) ≤ T_hot − 3 K and M_f(σ_L) ≥ T_cold + 3 K. Admissible shift ±maxShift K (≈ ±0.4 at% Ni or Cu/Hf
// substitution plus ageing). If the interface ΔT is narrower than the alloy's own hysteresis + stress shift,
// no shift can make it cycle — the alloy is then centred on the window and still classified incapable.
export function matchAlloy(mat, { Th, Tc, sH, sL }, maxShift = 40) {
  const hiLim = Th - 3 - (mat.Af + sH / mat.CA), loLim = Tc + 3 - (mat.Mf + sL / mat.CM);
  const need = (mat.Af + sH / mat.CA) - (mat.Mf + sL / mat.CM) + 6; // minimum ΔT for full transformation both ways
  let shift, feasible = loLim <= hiLim;
  if (feasible) shift = Math.max(-maxShift, Math.min(maxShift, 0.5 * (loLim + hiLim)));
  else shift = Math.max(-maxShift, Math.min(maxShift, 0.5 * (Th + Tc) - 0.5 * ((mat.Af + sH / mat.CA) + (mat.Mf + sL / mat.CM))));
  const withinRange = feasible && shift >= loLim - 1e-9 && shift <= hiLim + 1e-9;
  const dNi = -shift / 100; // ≈ −10 K per +0.1 at% Ni (Ni-rich NiTi lowers M_s)
  const note = `${shift >= 0 ? 'Raise' : 'Lower'} all transformation temperatures by ${Math.abs(shift).toFixed(1)} K: ≈ ${dNi >= 0 ? '+' : ''}${dNi.toFixed(2)} at% Ni relative to the library grade, followed by solution treatment (850 °C, 30 min, water quench) and ageing 400–500 °C for 0.5–2 h to trim M_s; confirm by DSC.`;
  return { mat: { ...mat, Ms: mat.Ms + shift, Mf: mat.Mf + shift, As: mat.As + shift, Af: mat.Af + shift, id: mat.id + (shift ? '-matched' : '') }, shift, feasible: withinRange, need, note };
}
// D6: limit the transformation strain amplitude (cam stroke stops) for fatigue life. Revision 6: the stroke-limited
// element transforms only the fraction ε_amp/ε_L, so the latent heat absorbed per cycle scales with the same fraction
// (work and latent heat both scale with the transformed volume — Clausius–Clapeyron keeps their ratio).
export const capStrain = (mat, epsAmp) => { const r = Math.min(1, epsAmp / mat.epsL); return { ...mat, epsL: mat.epsL * r, Lh: (mat.Lh || 0) * r }; };
// Revision 6: strain amplitude that gives the required number of cycles on the fatigue trend N ≈ 1e5 (3 %/ε)³ (200/σ_H)²
export const strainForLife = (Nreq, sH) => Math.min(0.06, 0.03 * Math.cbrt(1e5 * (200 / Math.max(1, sH)) ** 2 / Math.max(1, 1.05 * Nreq))); // 5 % margin on cycles
// Revision 6: counterflow regenerator on the stationary cam — cartridges leaving the hot arc pre-heat those entering it
// through N fixed heat-pipe stations. Heat per kg per cycle = L_hΔξ + (1 − ε)c_p(T_hE − T_cE) + w.
export function regenCycle(mat, R, { Th, Tc, nStations = 8, arcDeg = 120, fRev = 2.95, t_mm = 0.05, hc = 15000 } = {}) {
  const tau = mat.rho * mat.cp * (t_mm / 1000) / hc, RG = regenEffectiveness({ nStations, arcDeg, fRev, tau });
  const swing = Math.max(0, R.ThE - R.TcE), qSens = mat.cp * swing, qLat = (mat.Lh || 0) * Math.max(0, R.cyc.dXiHeat), w = Math.max(0, R.cyc.w_J_kg);
  const qNo = qLat + qSens + w, qReg = qLat + (1 - RG.eps) * qSens + w;
  // hot / cold arcs share what the two regenerator arcs leave; the regenerated cartridge enters the hot arc at T_in
  const arcHC = Math.max(1, (360 - 2 * arcDeg) / 2) / 360, Tin = R.TcE + RG.eps * swing, Tout = R.ThE - RG.eps * swing;
  const lnH = Math.log(Math.max(1.0001, (Th - Tin) / Math.max(1e-3, Th - R.ThE))), lnC = Math.log(Math.max(1.0001, (Tout - Tc) / Math.max(1e-3, R.TcE - Tc)));
  const tHeat = R.tim.tau * lnH, tCool = R.tim.tau * lnC, fMax = Math.min(arcHC / Math.max(1e-6, tHeat), arcHC / Math.max(1e-6, tCool));
  return { ...RG, tau, swing, qSens, qLat, w, qNo, qReg, eta: qReg > 0 ? w / qReg : 0, etaNo: qNo > 0 ? w / qNo : 0, saving: RG.eps * qSens, arcHC, Tin, Tout, tHeat, tCool, fMax };
}

// Revision 6 design search for the store-fed hub: recovery stress, lamella thickness, regenerator station count and arc
// chosen for the highest regenerated efficiency, with the strain amplitude set by the required fatigue life, alloy
// matching feasible within ±40 K and the hot/cold arcs fast enough for 40 km/h (one cycle per wheel revolution).
export function designRegenEngine(mat0, { Th, Tc, Nreq, fRev = 2.95, fMin = 3.54, hc = 15000, sL = 20, grid = null } = {}) {
  const G = grid || { sH: [100, 150, 200, 250, 300, 350, 400, 450, 500], t: [0.03, 0.05, 0.08, 0.1], N: [6, 10, 14], arc: [100, 120, 140] };
  let best = null;
  for (const sH of G.sH) {
    if (sH > 0.6 * (mat0.sy || 500)) continue;
    const eps = strainForLife(Nreq, sH), cap = capStrain(mat0, eps), MA = matchAlloy(cap, { Th, Tc, sH, sL });
    if (!MA.feasible) continue;
    for (const t of G.t) {
      const R = evaluateSMA(MA.mat, { Th, Tc, sH, sL, t_mm: t, hc });
      if (!(R.cyc.w_J_kg > 0.5 && R.cyc.dXiHeat > 0.05)) continue;
      for (const N of G.N) for (const arc of G.arc) {
        const C = regenCycle(MA.mat, R, { Th, Tc, nStations: N, arcDeg: arc, fRev, t_mm: t, hc });
        if (C.fMax < fMin) continue;
        if (!best || C.eta > best.C.eta) best = { sH, sL, t, N, arc, eps, MA, R, C };
      }
    }
  }
  return best;
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
  formats: { import: ['CSV/XLSX DSC curves', 'CSV force–displacement–temperature cycles', 'JSON'], export: ['CSV cycle paths', 'JSON calibrated parameter set', 'INP keyword deck / BDF bulk-data deck material cards via Suite 02 export manager', 'PNG/SVG'] },
  params: [
    { key: 'grade', label: 'Shape-memory alloy grade', type: 'select', group: 'Material', default: 'nitcu', options: SMAS.map(m => ({ value: m.id, label: m.name })), auto: (c) => { const h = c.upstream('s14')?.alloyHint; if (h && SMAS.some(m => m.id === h)) return { value: h, prov: 'coupled' }; const id = c.store.project.materials?.actuator; return { value: id && SMAS.some(m => m.id === id) ? id : 'nitcu', prov: id ? 'user' : 'library' }; }, help: 'Revision 6: follows the latent-heat store temperature (Suite 14) — NiTiHf for the 167 °C store, NiTiCu for a 118 °C store.' },
    { key: 'autoMatch', label: 'Match transformation temperatures to the interfaces', type: 'bool', group: 'Material', default: true, help: 'Composition / heat-treatment tuning within ±40 K so A_f(σ_H) ≤ T_hot − 3 K and M_f(σ_L) ≥ T_cold + 3 K.' },
    { key: 'epsAmp', label: 'Transformation strain amplitude limit', unit: '%', type: 'number', min: 0.2, max: 6, step: 0.1, group: 'Mechanical', default: 1.0, help: 'Revision 2 (D6): 1 % for ≥ 10⁷-cycle life.' },
    { key: 'source', label: 'Thermal boundary source', type: 'select', group: 'Thermal', default: 'coupled', options: [{ value: 'coupled', label: 'Coupled interface temperatures (Suite 14)' }, { value: 'imposed', label: 'Imposed laboratory ΔT (not atmospheric)' }] },
    { key: 'Th', label: 'Hot-interface temperature', unit: '°C', type: 'number', min: -20, max: 200, step: 0.5, group: 'Thermal', default: 25, auto: (c) => { const o = c.upstream('s14'); return o ? { value: +o.ThotIface_C.toFixed(2), prov: 'coupled' } : { value: +((c.env?.T ?? 20) + 5).toFixed(1), prov: 'hypothetical' }; } },
    { key: 'Tc', label: 'Cold-interface temperature', unit: '°C', type: 'number', min: -40, max: 150, step: 0.5, group: 'Thermal', default: 20, auto: (c) => { const o = c.upstream('s14'); return o ? { value: +o.TcoldIface_C.toFixed(2), prov: 'coupled' } : { value: +(c.env?.T ?? 20).toFixed(1), prov: 'hypothetical' }; } },
    { key: 'hc', label: 'Interface conductance to lamella', unit: 'W/m²K', type: 'number', min: 100, max: 100000, step: 100, group: 'Thermal', default: 15000, help: 'Revision 6: heat-pipe saddle shoes on the stationary cam press on the lamella stack through a ≈ 25 µm lubricated sliding film (k ≈ 0.4 W/mK → ≈ 1.6 × 10⁴ W/m²K). Bare dry contact ≈ 2–5 × 10³.' },
    { key: 'regen', label: 'Counterflow regenerator on the stationary cam', type: 'bool', group: 'Regenerator', default: true, help: 'Fixed heat-pipe stations couple each cartridge leaving the hot arc to one entering it; nothing extra rotates.' },
    { key: 'autoDesign', label: 'Optimise stress, strain, thickness and regenerator together', type: 'bool', group: 'Regenerator', default: true, help: 'Highest regenerated efficiency with the strain amplitude set by the required fatigue life and the arcs fast enough for 40 km/h.' },
    { key: 'nRegen', label: 'Regenerator heat-pipe stations', type: 'number', min: 1, max: 30, step: 1, group: 'Regenerator', default: 14 },
    { key: 'regenArc', label: 'Arc of each regenerator sector', unit: '°', type: 'number', min: 20, max: 160, step: 5, group: 'Regenerator', default: 140 },
    { key: 'lifeYears', label: 'Required cartridge life', unit: 'years', type: 'number', min: 1, max: 30, step: 1, group: 'Durability', default: 15 },
    { key: 'kmYear', label: 'Distance per year (one cycle per wheel revolution)', unit: 'km', type: 'number', min: 100, max: 50000, step: 100, group: 'Durability', default: 5000, help: 'Same ownership assumption as Suite 26.' },
    { key: 'sizeRoad', label: 'Size the alloy stack for the road load', type: 'bool', group: 'Geometry', default: true },
    { key: 'vRoad', label: 'Road-load sizing speed', unit: 'km/h', type: 'number', min: 5, max: 60, step: 1, group: 'Geometry', default: 20 },
    { key: 'etaTr', label: 'Hub-to-wheel transmission efficiency', type: 'range', min: 0.5, max: 1, step: 0.01, group: 'Geometry', default: 0.85 },
    { key: 'sH', label: 'Recovery-stroke stress σ_H', unit: 'MPa', type: 'range', min: 20, max: 500, step: 5, group: 'Mechanical', default: 150 },
    { key: 'sL', label: 'Reset (bias) stress σ_L', unit: 'MPa', type: 'range', min: 0, max: 200, step: 5, group: 'Mechanical', default: 20 },
    { key: 'tAuto', label: 'Size thickness from cycle-frequency requirement', type: 'bool', group: 'Geometry', default: true },
    { key: 'fReq', label: 'Required cycle frequency', unit: 'Hz', type: 'number', min: 0.05, max: 20, step: 0.05, group: 'Geometry', default: 2.95, auto: (c) => ({ value: +(20 / 3.6 / (2 * Math.PI * (c.vehicle.wheelRadius || 0.3))).toFixed(2), prov: 'calc' }), help: 'One cycle per wheel revolution at 20 km/h.' },
    { key: 't', label: 'Lamella thickness (manual)', unit: 'mm', type: 'number', min: 0.05, max: 5, step: 0.05, group: 'Geometry', default: 0.3, help: 'Used when automatic sizing is off. Thin lamellae shorten thermal diffusion time but reduce force per element.' },
    { key: 'w', label: 'Lamella width', unit: 'mm', type: 'number', min: 1, max: 50, step: 0.5, group: 'Geometry', default: 12 },
    { key: 'L', label: 'Active length', unit: 'mm', type: 'number', min: 5, max: 500, step: 1, group: 'Geometry', default: 120 },
    { key: 'nElem', label: 'Active elements (system)', type: 'number', min: 1, max: 2000, step: 1, group: 'Geometry', default: 48, help: '24 cartridges × 2 driven wheels in the reference layout.' },
    { key: 'dwell', label: 'Mechanical loading/unloading time fraction', type: 'range', min: 0, max: 1, step: 0.05, group: 'Mechanical', default: 0.1, expert: true },
    { key: 'Ptarget', label: 'System power target (specification research target)', unit: 'W', type: 'select', group: 'Targets', default: 200, options: [150, 200, 350, 600, 1500], help: 'Specification research targets for the metamaterial backup: 200–600 W continuous, 1500 W advanced.' },
  ],
  presets: [{ label: 'Calm night 5 K (293/288 K)', values: { source: 'imposed', Th: 20, Tc: 15 } }, { label: 'Lab 60 °C / 15 °C', values: { source: 'imposed', Th: 60, Tc: 15 } }, { label: 'Solar day 70 °C / 30 °C', values: { source: 'imposed', Th: 70, Tc: 30, grade: 'niti-actuator' } }, { label: 'Low-hysteresis NiTiCu', values: { grade: 'nitcu' } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const mat0 = get(p.grade); if (!mat0) throw new Error('Unknown alloy grade');
    if (p.Th <= p.Tc) return { verdicts: [{ test: 'Physically admissible inputs', status: 'fail', detail: 'Hot-interface temperature must exceed cold-interface temperature (no heat-engine cycle is possible otherwise).', fix: 'Set the hot temperature above the cold temperature — with no temperature difference no heat engine can run.' }], kpis: [], charts: [], tables: [], views: [], explain: ['Hot-interface temperature must exceed cold-interface temperature (no heat-engine cycle is possible otherwise).'], recs: [], checks: [], headline: 'Hot-interface temperature must exceed cold-interface temperature (no heat-engine cycle is possible otherwise).', headlineKind: 'fail', outputs: { infeasible: true, netPositive: false, Wrev_J: 0, Pwheel_W: 0, wNet_J_kg: 0, powerDensity_W_kg: 0 }, credibility: 'Conceptual' };
    const Nreq = p.kmYear * p.lifeYears * 1000 / (2 * Math.PI * (ctx.vehicle?.wheelRadius || 0.3));
    let design = null, designNote = 'Manual stresses, strain amplitude and thickness.';
    if (p.regen !== false && p.autoDesign !== false && p.source !== 'imposed') {
      design = designRegenEngine(mat0, { Th: p.Th, Tc: p.Tc, Nreq, fRev: p.fReq, hc: p.hc, sL: p.sL });
      if (design) { p = { ...p, sH: design.sH, epsAmp: +(design.eps * 100).toFixed(4), t: design.t, tAuto: false, nRegen: design.N, regenArc: design.arc }; designNote = `Generated (Revision 6): σ_H ${design.sH} MPa, strain amplitude ${fmt(design.eps * 100, 3)} % (fatigue life ${p.lifeYears} years = ${fmt(Nreq, 3)} cycles), lamella ${design.t} mm, ${design.N} regenerator stations on ${design.arc}° arcs — highest regenerated efficiency of ${9 * 4 * 9} candidates.`; }
      else { p = { ...p, epsAmp: +(strainForLife(Nreq, p.sH) * 100).toFixed(4) }; designNote = `No candidate cycles under this condition; the strain amplitude stays life-designed (${fmt(p.epsAmp, 3)} % for ${p.lifeYears} years at σ_H ${p.sH} MPa) — it is a property of the cartridge, not of the weather.`; }
    }
    const capped = capStrain(mat0, (p.epsAmp ?? 1) / 100), MA = p.autoMatch !== false ? matchAlloy(capped, { Th: p.Th, Tc: p.Tc, sH: p.sH, sL: p.sL }) : { mat: capped, shift: 0, feasible: null, need: NaN, note: 'Library transformation temperatures used.' };
    const mat = MA.mat;
    // D5: thickness = largest lamella that still reaches the required cycle frequency
    let tUse = p.t, tNote = 'manual';
    if (p.tAuto !== false) { const ts = [2, 1.5, 1.2, 1, 0.8, 0.6, 0.5, 0.4, 0.3, 0.25, 0.2, 0.15, 0.1, 0.07, 0.05]; const ok = ts.find(t => evaluateSMA(mat, { Th: p.Th, Tc: p.Tc, sH: p.sH, sL: p.sL, t_mm: t, hc: p.hc, dwell: p.dwell }).f >= p.fReq); tUse = ok ?? 0.05; tNote = ok ? `largest thickness meeting ${p.fReq} Hz` : `even 0.05 mm cannot reach ${p.fReq} Hz`; }
    p = { ...p, t: tUse };
    const q = { Th: p.Th, Tc: p.Tc, sH: p.sH, sL: p.sL, t_mm: p.t, hc: p.hc, dwell: p.dwell };
    const R = evaluateSMA(mat, q), cyc = R.cyc;
    const RC = p.regen !== false ? regenCycle(mat, R, { Th: p.Th, Tc: p.Tc, nStations: p.nRegen, arcDeg: p.regenArc, fRev: p.fReq, t_mm: p.t, hc: p.hc }) : null;
    const etaUse = RC ? RC.eta : cyc.eta, qUse = RC ? RC.qReg : cyc.qIn_J_kg, fUse = RC ? RC.fMax : R.f, fOp = RC ? Math.min(fUse, p.fReq) : R.f, Pkg = Math.max(0, cyc.w_J_kg) * fOp;
    const vehR = ctx.vehicle || {}, vR = p.vRoad / 3.6, Proad = ((vehR.Crr ?? 0.005) * (vehR.mass ?? 250) * 9.80665 + 0.5 * 1.2 * (vehR.CdA ?? 0.18) * vR * vR) * vR + (vehR.bearingLoss_W ?? 3), Pshaft = Proad / p.etaTr;
    const V = p.t * p.w * p.L * 1e-9, mElem = mat.rho * V;
    const storeFed = ctx.upstream('s14')?.hotSource === 'store'; // stack sized for the road load only when the store supplies the heat
    const nLam = p.sizeRoad !== false && storeFed && RC && Pkg > 0 ? Math.max(1, Math.ceil(Pshaft / (Pkg * mElem * p.nElem))) : 1, mTot = mElem * p.nElem * nLam;
    const Wcycle = cyc.w_J_m3 * V, Psys = Pkg * mTot, Fh = p.sH * p.t * p.w, stroke = cyc.strokeStrain * p.L;
    const netPositive = cyc.w_J_kg > 0.5 && R.cyc.dXiHeat > 0.05 && R.f > 0;
    const Gconv = netPositive ? qUse * mTot * fOp / Math.max(0.1, p.Th - p.Tc) : 1e-3 * mTot * mat.cp / 10;
    const lifeH = fOp > 0 ? R.fatigue / fOp / 3600 : Infinity, cycYear = p.kmYear * 1000 / (2 * Math.PI * (vehR.wheelRadius || 0.3)), lifeYr = R.fatigue / cycYear, Qroad = etaUse > 0 ? Pshaft / etaUse : Infinity;
    ctx.progress(0.4, 'sweeps');
    // verification
    out.checks.push({ name: 'Second law: η ≤ η_Carnot', value: cyc.eta, target: `≤ ${fmt(cyc.etaCarnot, 4)}`, pass: cyc.eta <= cyc.etaCarnot + 1e-9 });
    if (RC) { out.checks.push({ name: 'Second law with regeneration: η_reg ≤ η_Carnot', value: etaUse, target: `≤ ${fmt(cyc.etaCarnot, 4)}`, pass: etaUse <= cyc.etaCarnot + 1e-9 });
      out.checks.push({ name: 'Regenerated efficiency within the literature range (≤ 20 % of Carnot for regenerative elastocaloric / shape-memory cycles)', value: cyc.etaCarnot ? etaUse / cyc.etaCarnot : 0, target: '≤ 0.20', pass: !cyc.etaCarnot || etaUse / cyc.etaCarnot <= 0.2, detail: 'Device-level second-law efficiencies of regenerative elastocaloric heat pumps and shape-memory heat engines are reported at ≈ 5–20 % of Carnot.' }); }
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
    K('Transformation-temperature shift (matching)', MA.shift, 'K', `${MA.note} ${MA.feasible === false ? `Matching is impossible: the alloy needs ΔT ≥ ${fmt(MA.need, 3)} K (hysteresis + stress shift + 6 K margins) but only ${fmt(p.Th - p.Tc, 3)} K is available.` : ''}`, 'generated', MA.feasible === false ? 'bad' : 'ok');
    K('Lamella thickness (sized)', p.t, 'mm', `Sizing: ${tNote}.`, p.tAuto !== false ? 'generated' : 'user');
    K('A_s / A_f under σ_H', `${fmt(R.temps.H.As, 3)} / ${fmt(R.temps.H.Af, 3)}`, '°C', 'Reverse-transformation window shifted upward by the recovery stress (Clausius–Clapeyron). The hot side must exceed A_s(σ_H).', 'calc');
    K('M_s / M_f under σ_L', `${fmt(R.temps.L.Ms, 3)} / ${fmt(R.temps.L.Mf, 3)}`, '°C', 'Forward-transformation window at the reset stress. The cold side must fall below M_s(σ_L).', 'calc');
    K('Recovered phase fraction per cycle Δξ', cyc.dXiHeat, '', 'Fraction of the material that actually transforms in the available time.', 'calc', cyc.dXiHeat > 0.5 ? 'ok' : cyc.dXiHeat > 0.05 ? 'warn' : 'bad');
    K('Net work per cycle', cyc.w_J_kg, 'J/kg', 'Area enclosed by the σ–ε cycle per kg after reset work and hysteresis.', 'calc', netPositive ? 'ok' : 'bad');
    K('Net work per element per cycle', Wcycle, 'J', `Element ${p.t}×${p.w}×${p.L} mm (${fmt(mElem * 1000, 3)} g). Hub target ≈ 3.54 J per cartridge per revolution.`, 'calc', Wcycle >= 3.54 ? 'ok' : 'warn');
    K('Revision 6 engine design', designNote, '', 'Store-fed hub: a hot store at near-constant temperature feeds the hot arc through heat-pipe saddles, so the cycle is no longer limited by collector heat flow.', design ? 'generated' : 'user');
    K('Achievable cycle frequency', fUse, 'Hz', RC ? `Hot / cold arcs ${fmt(RC.arcHC * 360, 3)}° each; regenerated entry at ${fmt(RC.Tin, 4)} °C needs ${fmt(RC.tHeat * 1000, 3)} ms heating and ${fmt(RC.tCool * 1000, 3)} ms cooling (τ = ${fmt(R.tim.tau * 1000, 3)} ms with h_c = ${p.hc} W/m²K). 40 km/h needs 3.54 Hz (one cycle per wheel revolution).` : `Heating ${fmt(R.tim.tHeat, 3)} s + cooling ${fmt(R.tim.tCool, 3)} s (τ = ${fmt(R.tim.tau, 3)} s). 40 km/h hub operation needs 3.54 Hz.`, 'calc', fUse >= 3.54 ? 'ok' : 'bad');
    if (RC) { K('Regenerator effectiveness ε', RC.eps, '', `${p.nRegen} fixed heat-pipe stations on two ${p.regenArc}° arcs: ε = 1/(1 + 1/N + 1/NTU), NTU = t_regen/τ = ${fmt(RC.tRegen * 1000, 3)} ms / ${fmt(RC.tau * 1000, 3)} ms = ${fmt(RC.NTU, 3)} at ${fmt(p.fReq, 3)} Hz.`, 'calc', RC.eps > 0.7 ? 'ok' : 'warn');
      K('Heat per kg per cycle (regenerated / without)', `${fmt(RC.qReg, 4)} / ${fmt(RC.qNo, 4)}`, 'J/kg', `Latent ${fmt(RC.qLat, 4)} + sensible ${fmt(RC.qSens, 4)} × (1 − ε) + work ${fmt(RC.w, 4)}. The regenerator recovers ${fmt(RC.saving, 4)} J/kg of the sensible swing ${fmt(RC.swing, 3)} K.`, 'calc');
      K(storeFed ? 'Alloy stack (sized for the road load)' : 'Alloy stack (heat-limited default — not sized for the road load)', `${nLam} lamellae × ${p.nElem} cartridges`, '', `${storeFed ? '' : 'Without the research store the collector heat, not alloy mass, limits power, so one lamella per cartridge is kept. '}${fmt(mTot, 3)} kg active alloy delivers ${fmt(Psys, 3)} W at ${fmt(fOp, 3)} Hz vs ${fmt(Pshaft, 3)} W shaft power for ${fmt(Proad, 3)} W road load at ${p.vRoad} km/h (transmission ${p.etaTr}).`, 'generated', Psys >= Pshaft ? 'ok' : 'bad');
      K('Heat drawn at the road load', Number.isFinite(Qroad) ? Qroad : 'road load not reachable at these settings', Number.isFinite(Qroad) ? 'W' : '', `Shaft power / regenerated efficiency (${fmt(etaUse * 100, 3)} %). Supplied by the latent-heat store (Suite 14).`, 'calc'); }
    K('Continuous power density', R.P_kg, 'W/kg', 'Net work × frequency. Laboratory NiTi heat engines report ≈ 52 W/kg under strong thermal gradients.', 'calc');
    K('System mechanical power', Psys, 'W', `${p.nElem} elements, ${fmt(mTot, 3)} kg active NiTi. Target ${p.Ptarget} W.`, 'calc', Psys >= p.Ptarget ? 'ok' : 'bad');
    K('Thermal efficiency', etaUse * 100, '%', RC ? `With the counterflow regenerator (without: ${fmt(RC.etaNo * 100, 3)} %).` : 'Work / heat input per cycle (sensible + latent + work).', 'calc');
    K('Fraction of Carnot', cyc.etaCarnot ? etaUse / cyc.etaCarnot * 100 : 0, '%', `Carnot limit at the interface temperatures = ${fmt(cyc.etaCarnot * 100, 3)} %.`, 'calc');
    K('Recovery force / stroke per element', `${fmt(Fh, 4)} N / ${fmt(stroke, 3)} mm`, '', 'High force alone is not propulsion: only the closed-cycle net work counts.', 'calc');
    K('Estimated actuation fatigue life', R.fatigue, 'cycles', `≈ ${fmt(lifeYr, 3)} years at ${p.kmYear} km/year (${fmt(cycYear, 3)} cycles/year; ${fmt(lifeH, 3)} h of continuous cycling) — trend estimate, calibrate with cyclic tests.`, 'estimated', R.fatigue >= Nreq ? 'ok' : R.fatigue > 1e7 ? 'warn' : 'bad');
    K('Minimum ΔT for net-positive cycling', Number.isFinite(thr) ? thr : 'no net-positive cycle in the searched range', Number.isFinite(thr) ? 'K' : '', 'Smallest interface ΔT (cold side fixed) giving a repeatable net-positive cycle at the chosen stresses.', 'calc');
    // verdicts
    out.verdicts.push({ test: 'Repeatable net-positive work cycle', status: netPositive ? 'pass' : 'fail', detail: netPositive ? `${fmt(cyc.w_J_kg, 4)} J/kg per cycle, Δξ = ${fmt(cyc.dXiHeat, 3)}.` : `No repeatable net-positive cycle: ${p.Th < R.temps.H.As ? `hot side ${fmt(p.Th, 3)} °C is below A_s(σ_H) = ${fmt(R.temps.H.As, 3)} °C` : p.Tc > R.temps.L.Ms ? `cold side ${fmt(p.Tc, 3)} °C is above M_s(σ_L) = ${fmt(R.temps.L.Ms, 3)} °C` : 'the transformation and reset work cancel'}. Classified INCAPABLE of sustained thermal propulsion under this condition.`, fix: 'Increase ΔT, choose an alloy whose transformation window and hysteresis fit inside the available ΔT (e.g. low-hysteresis NiTiCu), or reduce σ_H.' });
    out.verdicts.push({ test: 'Cycle frequency compatible with hub speed (3.54 Hz at 40 km/h)', status: fUse >= 3.54 ? 'pass' : fUse >= 1.5 ? 'warn' : 'fail', detail: RC ? `${fmt(fUse, 3)} Hz achievable with heat-pipe saddles (h_c ${p.hc} W/m²K) and regenerated entry temperatures; the hot arc lasts ${fmt(RC.arcHC / 3.54 * 1000, 3)} ms at 40 km/h.` : `${fmt(R.f, 3)} Hz achievable; 0.141 s per sector available at 40 km/h.`, fix: 'Thinner lamellae / microstruts, higher interface conductance (liquid-metal or graphite interfaces), or accept lower speed.' });
    out.verdicts.push({ test: `System power vs ${p.Ptarget} W target`, status: Psys >= p.Ptarget ? 'pass' : 'fail', detail: `${fmt(Psys, 3)} W = ${fmt(Psys / p.Ptarget * 100, 3)} % of target with ${fmt(mTot, 3)} kg active alloy${RC && storeFed ? ` (stack sized for the ${fmt(Pshaft, 3)} W road-load shaft power; ${p.Ptarget} W would draw ${fmt(p.Ptarget / Math.max(1e-9, etaUse), 3)} W of stored heat)` : RC ? ` (heat-limited: ${p.Ptarget} W would need ${fmt(p.Ptarget / Math.max(1e-9, etaUse), 3)} W of heat at the hub)` : ''}. Simulation / research target — not physical validation.` });
    out.verdicts.push({ test: `Cartridge fatigue life ≥ ${p.lifeYears} years`, status: R.fatigue >= Nreq ? 'pass' : 'fail', detail: `${fmt(R.fatigue, 3)} cycles at ${fmt(p.epsAmp, 3)} % strain amplitude and σ_H = ${p.sH} MPa vs ${fmt(Nreq, 3)} needed (${p.kmYear} km/year × ${p.lifeYears} years, one cycle per revolution).`, fix: 'Lower the strain amplitude (more alloy for the same power) or the recovery stress.' });
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
    out.tables.push({ title: 'Transformation temperatures (°C)', cols: ['State', 'M_f', 'M_s', 'A_s', 'A_f'], rows: [['Stress-free (library grade)', mat0.Mf, mat0.Ms, mat0.As, mat0.Af], [`Stress-free (matched, shift ${fmt(MA.shift, 3)} K)`, mat.Mf, mat.Ms, mat.As, mat.Af], [`σ_L = ${p.sL} MPa`, R.temps.L.Mf, R.temps.L.Ms, R.temps.L.As, R.temps.L.Af], [`σ_H = ${p.sH} MPa`, R.temps.H.Mf, R.temps.H.Ms, R.temps.H.As, R.temps.H.Af]] });
    out.tables.push({ title: 'Cycle energy accounting (per kg)', cols: ['Quantity', 'Value', 'Unit'], rows: [['Net work', cyc.w_J_kg, 'J/kg'], ['Sensible heat c_pΔT', mat.cp * (p.Th - p.Tc), 'J/kg'], ['Latent heat L_hΔξ', mat.Lh * cyc.dXiHeat, 'J/kg'], ['Total heat input', cyc.qIn_J_kg, 'J/kg'], ['Efficiency', cyc.eta * 100, '%'], ['Carnot limit', cyc.etaCarnot * 100, '%'], ['Heat demand at system scale', cyc.qIn_J_kg * mTot * R.f, 'W']] });
    out.tables.push({ title: 'Experimental and numerical references (literature context, not predictions of this design)', cols: ['Source', 'Reported result', 'Conditions'], rows: [['NiTi heat engine (laboratory)', '≈ 52 W/kg mechanical', 'Strong hot/cold reservoirs'], ['NiTi heat engine on geothermal source (2024)', '3.5 / 8.5 / 11.5 W', 'Source at 60 / 80 / 90 °C'], ['Optimised SMA thermoelastic harvesting (2025, numerical)', '8.6 % efficiency', '55 K temperature difference'], ['Calm night reference (this specification)', 'Carnot 1.71 %', '293 K / 288 K']] });
    // recs
    if (!netPositive) out.recs.push({ level: 'critical', text: `No net-positive cycle at ΔT = ${fmt(p.Th - p.Tc, 3)} K. Minimum workable ΔT for this alloy and stresses ≈ ${thr ? fmt(thr, 3) + ' K' : '> 80 K'}. Match transformation temperatures to the actual interface temperatures (composition/heat-treatment) and use low-hysteresis grades.`, basis: 'Phase diagram and ΔT sweep' });
    if (R.f < 3.54) out.recs.push({ level: 'warn', text: `Thermal cycling limits frequency to ${fmt(R.f, 3)} Hz. Halving thickness roughly halves τ; raise interface conductance with conformal thermal interfaces.`, basis: 'Lamella timing τ = ρc_p t/h_c' });
    if (R.fatigue < 1e7) out.recs.push({ level: 'warn', text: `Fatigue life ≈ ${fmt(R.fatigue, 3)} cycles is short for a vehicle (10⁸–10⁹ cycles over its life at Hz rates). Reduce transformation strain below ≈ 2 % and σ_H, and train/stabilise the alloy.`, basis: 'Functional fatigue trend (calibrate)' });
    out.recs.push({ level: 'good', text: 'Before hardware: measure DSC transformation temperatures, isobaric strain–temperature loops at σ_H and σ_L, and a full heat-engine cycle on a single cartridge with all heat flows metered (Gate 2).', basis: 'Specification development gates' });
    out.recs.push({ level: 'sustain', text: 'NiTi has high embodied energy (≈ 900 MJ/kg); maximise fatigue life and design cartridges for replacement and alloy recovery rather than disposal.', basis: 'Materials registry life-cycle data' });
    out.explain.push({ title: 'What “capable of propulsion” means here', text: 'A shape-memory element can produce a large recovery force on a single heating. Sustained propulsion requires repeated complete cycles whose work output exceeds the reset work, at a frequency the heat transfer can sustain, with heat supplied by the environment. All four conditions are evaluated; failing any one classifies the configuration as incapable under the simulated condition.' });
    out.views.push({ label: '3D actuator cycle', icon: '🔩', render: (el) => view3dCycle(el, { path: cyc.path, mat, p, f: R.f, netPositive, wNet: cyc.w_J_kg }) });
    out.views.push({ label: '2D animation: σ–ε–T cycle', icon: '🎞', render: (el) => anim2dCycle(el, { path: cyc.path, mat, netPositive, wNet: cyc.w_J_kg, Th: p.Th, Tc: p.Tc }) });
    const MEAS = measuredBlock(out, ctx, 's15', 'Experiment A, cartridge', etaUse, ctx.upstream('s14')?.researchStore?.fracNeeded ?? ctx.upstream('s14')?.fracNeeded);
    out.outputs = { measuredFrac: MEAS ? { mean: MEAS.mean, lo95: MEAS.lo95, n: MEAS.n, validated: MEAS.validated } : null, regen: !!RC, efficiencyNoRegen: RC ? RC.etaNo : cyc.eta, regenEff: RC?.eps ?? 0, regenNTU: RC?.NTU ?? 0, regenStations: p.nRegen, regenArc_deg: p.regenArc, regenSave_J_kg: RC?.saving ?? 0, roadShaft_W: Pshaft, roadLoad_W: Proad, roadHeat_W: Qroad, lifeYearsAchieved: lifeYr, cyclesRequired: Nreq, nLam, designNote, tShift_K: MA.shift, matchFeasible: MA.feasible, minDT_K: MA.need, matchNote: MA.note, tSized_mm: p.t, epsAmp: p.epsAmp, wNet_J_kg: cyc.w_J_kg, Wcycle_J: Wcycle, maxFreqHz: fUse, powerDensity_W_kg: Pkg, efficiency: etaUse, carnotFrac: cyc.etaCarnot ? etaUse / cyc.etaCarnot : 0, fatigueCycles: R.fatigue, netPositive, recoveryStress_MPa: p.sH, Gconv_W_K: Gconv, grade: p.grade, Th: p.Th, Tc: p.Tc, sL: p.sL, elem: { t: p.t, w: p.w, L: p.L, mass: mElem, nLam }, activeMass_kg: mTot, systemPower_W: Psys, tau_s: R.tim.tau, source: p.source, heatPerCycle_J_kg: qUse };
    out.headline = netPositive ? `Net-positive cycle: ${fmt(cyc.w_J_kg, 4)} J/kg at ${fmt(R.f, 3)} Hz → ${fmt(R.P_kg, 3)} W/kg; system ${fmt(Psys, 3)} W vs ${p.Ptarget} W target.` : `INCAPABLE under this condition: no repeatable net-positive cycle at ΔT = ${fmt(p.Th - p.Tc, 3)} K (threshold ≈ ${thr ? fmt(thr, 3) : '>80'} K).`;
    out.headlineKind = netPositive && Psys >= p.Ptarget ? 'ok' : netPositive ? 'warn' : 'fail';
    void max;
    return out;
  },
};

// 3D NiTi cartridge animated through the solved thermomechanical cycle (path from the constitutive integration):
// lamella length follows the solved strain (exaggerated for visibility), colour follows the solved martensite
// fraction or temperature, the bias load moves with the stroke, and an inset tracks the point on the σ–ε loop.
async function view3dCycle(el, { path, mat, p, f, netPositive, wNet }) {
  const [{ viewer }, { playback, hud, legend3d, magnifier }, { box }, { cmap }, { h }] = await Promise.all([import('../core/viz3d.js'), import('../core/sim3d.js'), import('../core/cad.js'), import('../core/chart.js'), import('../core/ui.js')]);
  const period = f > 0 ? 1 / f : NaN, eRange = Math.max(...path.map(r => r.e)) - Math.min(...path.map(r => r.e));
  let colorBy = 'xi';
  const sel = h('select', { onchange: (e) => { colorBy = e.target.value; } }, h('option', { value: 'xi' }, 'Colour: martensite fraction ξ'), h('option', { value: 'T' }, 'Colour: temperature'), h('option', { value: 's' }, 'Colour: stress'));
  el.append(h('p', { class: 'small muted' }, `Solved: the full ${mat.name} heating–loading–cooling–unloading path (${path.length} integration points, stress-dependent transformation, hysteresis). Lamella extension = solved strain × magnification (slider); colour = solved state; inset = σ–ε loop with the current point. Real cycle period ${Number.isFinite(period) ? fmt(period, 3) + ' s' : 'undefined (no repeatable cycle)'}; playback shows one cycle as 0–100 %.`), sel);
  if (!netPositive) el.append(h('div', { class: 'callout fail' }, 'This configuration has no repeatable net-positive cycle: the animation shows the solved (collapsed) loop — the element barely moves and delivers no net work.'));
  const mag = magnifier(el, { auto: Math.min(500, 0.3 / Math.max(eRange, 3e-4)), truePeak: eRange * p.L, unit: ' mm stroke' });
  const wrap = h('div'); el.append(wrap);
  const V = await viewer(wrap, { height: 440, grid: true }); const THREE = V.THREE;
  const Lv = 1.2, nLam = 6, gap = 0.05, tv = 0.025, wv = 0.18; // visual dimensions (m)
  const e0 = Math.min(...path.map(r => r.e));
  const top = box('Fixed end plate', 0.5, 0.4, 0.06, { color: '#64748b' }); top.translate(0, 0, Lv + 0.4); V.add(top);
  const hot = box('Hot interface plate', 0.04, 0.5, Lv * 0.9, { color: '#ef4444' }); hot.translate(-0.25, 0, Lv / 2 + 0.3); const oHot = V.add(hot, { opacity: 0.7 });
  const cold = box('Cold interface plate', 0.04, 0.5, Lv * 0.9, { color: '#3b82f6' }); cold.translate(0.25, 0, Lv / 2 + 0.3); const oCold = V.add(cold, { opacity: 0.7 });
  const lams = []; for (let i = 0; i < nLam; i++) { const m = box(`NiTi lamella ${i + 1}`, tv, wv, 1, { color: '#c58b4b' }); m.translate(-((nLam - 1) * gap) / 2 + i * gap, 0, -0.5); const o = V.add(m); o.position.z = Lv + 0.37; lams.push(o); }
  const mass = box('Bias / load block', 0.42, 0.35, 0.14, { color: '#94a3b8' }); const oMass = V.add(mass);
  const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(), 0.4, 0xfacc15, 0.1, 0.06); V.root.add(arrow);
  V.viewDir(1, -1.4, 0.5);
  const sMax = Math.max(...path.map(r => r.s)), Tmin = Math.min(...path.map(r => r.T)), Tmax = Math.max(...path.map(r => r.T));
  const H = hud(wrap); const leg = legend3d(wrap, { title: 'ξ (0 = austenite, 1 = martensite)', lo: 0, hi: 1, cmapName: 'viridis' });
  // inset σ–ε loop
  const es = path.map(r => r.e * 100), ss = path.map(r => r.s), ex0 = Math.min(...es), ex1 = Math.max(...es) || 1, sy1 = Math.max(...ss) || 1;
  const X = (e) => 10 + 160 * (e - ex0) / ((ex1 - ex0) || 1), Y = (s) => 110 - 95 * s / sy1;
  const inset = h('div', { class: 'sim-legend', style: 'left:8px;right:auto;bottom:60px;width:190px', html: `<div class="small">σ–ε loop (solved)</div><svg viewBox="0 0 180 120" width="180" height="120"><polyline points="${path.map((r, i) => X(es[i]).toFixed(1) + ',' + Y(ss[i]).toFixed(1)).join(' ')}" fill="none" stroke="#38bdf8" stroke-width="1.5"/><circle r="4" fill="#facc15"/></svg><div class="small">ε ${fmt(ex0, 3)}–${fmt(ex1, 3)} %, σ ≤ ${fmt(sy1, 3)} MPa</div>` });
  wrap.append(inset); const dot = inset.querySelector('circle');
  // cumulative work along the path (J/kg): w = −∮σ dε with σ in Pa
  const cum = [0]; for (let i = 1; i < path.length; i++) cum.push(cum[i - 1] - 0.5 * (path[i].s + path[i - 1].s) * 1e6 * (path[i].e - path[i - 1].e) / mat.rho);
  playback(wrap, { duration: 100, rate: 12, label: 'Cycle progress', unit: '%', onFrame: (t) => {
    const x = t / 100 * (path.length - 1), i = Math.min(path.length - 2, Math.floor(x)), fr = x - i, A = path[i], B = path[i + 1];
    const e = A.e + (B.e - A.e) * fr, sg = A.s + (B.s - A.s) * fr, T = A.T + (B.T - A.T) * fr, xi = A.xi + (B.xi - A.xi) * fr;
    const len = Lv * (1 + mag.value * (e - e0));
    const v = colorBy === 'xi' ? xi : colorBy === 'T' ? (T - Tmin) / ((Tmax - Tmin) || 1) : sg / (sMax || 1), c = cmap(v, colorBy === 'xi' ? 'viridis' : colorBy === 'T' ? 'coolwarm' : 'inferno');
    for (const o of lams) { o.scale.z = len; o.material.color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255); }
    oMass.position.set(0, 0, Lv + 0.37 - len - 0.07); arrow.position.set(0.32, 0, Lv + 0.37 - len - 0.07); arrow.setLength(0.15 + 0.5 * sg / (sMax || 1), 0.1, 0.06);
    const heating = B.T > A.T + 1e-9, cooling = B.T < A.T - 1e-9; oHot.material.opacity = heating ? 0.95 : 0.25; oCold.material.opacity = cooling ? 0.95 : 0.25;
    leg.set(colorBy === 'xi' ? 0 : colorBy === 'T' ? Tmin : 0, colorBy === 'xi' ? 1 : colorBy === 'T' ? Tmax : sMax);
    dot.setAttribute('cx', X(e * 100)); dot.setAttribute('cy', Y(sg));
    const w = cum[i] + (cum[i + 1] - cum[i]) * fr;
    H.set([['Stage', heating ? 'Heating (A transformation)' : cooling ? 'Cooling (M transformation)' : 'Load change'], ['Temperature', fmt(T, 4) + ' °C', heating ? '#ef4444' : cooling ? '#38bdf8' : undefined], ['Stress σ', fmt(sg, 4) + ' MPa'], ['Strain ε', fmt(e * 100, 4) + ' %'], ['Martensite ξ', fmt(xi, 3)], ['Work so far', fmt(w, 4) + ' J/kg'], ['Net work / cycle', fmt(wNet, 4) + ' J/kg', netPositive ? '#22c55e' : '#ef4444'], ['Real period', Number.isFinite(period) ? fmt(period, 3) + ' s' : '—']]);
  } });
}

// 2D animation of the solved cycle: σ–ε loop and T–ε hysteresis with a moving state marker, ξ and T traces with a
// cursor, a phase-fraction bar and the running work integral −∫σ dε.
async function anim2dCycle(el, { path, mat, netPositive, wNet, Th, Tc }) {
  const [{ anim2d }, { h }, { cmap }] = await Promise.all([import('../core/sim3d.js'), import('../core/ui.js'), import('../core/chart.js')]);
  el.append(h('p', { class: 'small muted' }, `Every frame is a point of the solved constitutive path (${path.length} points, ${mat.name}). Net work per cycle ${fmt(wNet, 4)} J/kg ${netPositive ? '(net-positive)' : '(NOT net-positive)'}.`));
  const E = path.map(r => r.e * 100), S = path.map(r => r.s), T = path.map(r => r.T), X = path.map(r => r.xi), idx = path.map((_, i) => i);
  const pad = (a) => { const lo = Math.min(...a), hi = Math.max(...a), d = (hi - lo) * 0.1 || 1; return [lo - d, hi + d]; };
  const cum = [0]; for (let i = 1; i < path.length; i++) cum.push(cum[i - 1] - 0.5 * (path[i].s + path[i - 1].s) * 1e6 * (path[i].e - path[i - 1].e) / mat.rho);
  anim2d(el, { height: 420, duration: 100, rate: 12, label: 'Cycle progress', unit: '%', onFrame: (t, g) => {
    g.clear(); const W = g.W, H = g.H, i = Math.min(path.length - 1, Math.round(t / 100 * (path.length - 1)));
    const A = g.panel(0, 0, W / 2, H - 70, { xr: pad(E), yr: pad(S), xlabel: 'Strain ε [%]', ylabel: 'Stress σ [MPa]', title: 'σ–ε loop (enclosed area = net work)' });
    A.line(E, S, '#38bdf8', 2); A.line(E.slice(0, i + 1), S.slice(0, i + 1), '#f97316', 3); A.point(E[i], S[i]);
    const B = g.panel(W / 2, 0, W / 2, H - 70, { xr: [0, path.length - 1], yr: [Math.min(Tc, ...T) - 2, Math.max(Th, ...T) + 2], xlabel: 'Cycle step', ylabel: 'Temperature [°C]', title: 'Temperature (red) and ξ (green, right scale 0–1)' });
    B.line(idx, T, '#ef4444', 2);
    const Tlo = Math.min(Tc, ...T) - 2, Thi = Math.max(Th, ...T) + 2; B.line(idx, X.map(x => Tlo + x * (Thi - Tlo)), '#22c55e', 2, [5, 3]); B.vline(i); B.point(i, T[i], '#ef4444', 4); B.point(i, Tlo + X[i] * (Thi - Tlo), '#22c55e', 4);
    const c = cmap(X[i], 'viridis'); g.gauge(60, H - 40, W * 0.35, X[i], `Martensite fraction ξ = ${fmt(X[i], 3)}`, `rgb(${c.map(Math.round).join(',')})`);
    g.text(`σ = ${fmt(S[i], 4)} MPa   ε = ${fmt(E[i], 4)} %   T = ${fmt(T[i], 4)} °C   work so far ${fmt(cum[i], 4)} J/kg`, W * 0.45, H - 30, { size: 12, color: cum[i] >= 0 ? '#22c55e' : '#ef4444' });
  } });
}
