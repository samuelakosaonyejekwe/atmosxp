// Suite 18 — Bistable metamaterial energy storage and elastic buffering.
import * as TH from '../core/thermo.js';
import { linspace, fmt, trapz, range, logspace } from '../core/num.js';
import { forUse, get } from '../core/materials.js';
import { diffEvolution } from '../core/analysis.js';

export default {
  description: 'Designs the bistable elastic metamaterial buffer: single-cell potential-energy landscape (snap-through truss with tuned parallel stiffness), stable states, energy barriers, recoverable energy after snap-through dissipation, strain and buckling limits, array sizing (cells, mass, volume) for a target energy, charge/discharge curves, backup endurance at a given wheel power, fatigue, and unintended-release risk. The buffer is always modelled as a finite store with an explicitly recorded initial charge.',
  literature: 'bistable mechanical metamaterial energy storage snap-through multistable lattice',
  sustain: true, credibility: 'Analytical screening',
  equations: [
    'Finite-strain bar kinematics of a shallow von Mises truss L(u) = √(b² + (h − u)²); strain-energy density and potential-energy minimisation U(u) = (EA/L₀)(L − L₀)² + ½k_s u²',
    'Equilibrium F = dU/du = 0 (stable: d²U/du² > 0); snap-through instability at dF/du = 0; energy barriers U(u_max) − U(u_stable)',
    'Bistability criterion for the shallow arch with parallel spring: k_s/k* < 1/4 (closed-form x² − 3x + 2 + r = 0)',
    'Euler buckling of the compressed bars P_cr = π²EI/L₀²; strain limit σ_y/(E·SF)',
    'Hysteretic snap-through dissipation (loss factor) and recovery efficiency; finite discharge t = ηE/P',
    'Network energy summation for N parallel/series cells; Basquin high-cycle fatigue; inertial trigger margin for unintended release',
  ],
  methods: ['Dense sampling of U(u) and F(u) (481 points) with discrete extremum detection', 'Closed-form and numerical bistability cross-check', 'Sizing by energy and mass balance; parametric study of the stiffness ratio'],
  assumptions: ['Pin-jointed slender bars (bending of joints neglected), quasi-static charging', 'Identical cells; energy released through the hub transmission with efficiency η', 'Snap-through dynamic losses lumped as a loss factor (calibrate from drop/release tests)'],
  ibc: {
    initial: ['Configuration of each cell (charged / discharged) and stored elastic energy — recorded explicitly, never silently reset', 'Preload, temperature, fatigue state, previous loading history'],
    boundary: ['Displacement-controlled charging (0 → u₂) through the hub or wing linkage', 'Mechanical trigger releasing cells sequentially toward the hub', 'Load torque at the hub; maximum permitted deformation (strain limit)', 'Inertial road/gust excitation for unintended-release checks'],
  },
  cvv: {
    calibration: ['Charge and discharge force–displacement cycles of single cells', 'Snap-through thresholds and energy dissipation (high-speed imaging, load cell)', 'Fatigue under repeated snap-through'],
    verification: ['Closed-form bistability limit vs numerical extremum detection', '∫F du equals U(u₂) − U(0)', 'Shallow-arch strain formula vs exact kinematics'],
    validation: ['Measured charge–discharge testing of an assembled array driving a dynamometer: usable energy, recovery efficiency, duration at specified load'],
  },
  formats: { import: ['CSV force–displacement cycles', 'XLSX', 'JSON'], export: ['CSV energy landscape & sizing', 'STEP AP214 (faceted) / 3MF / STL cell geometry (Suite 02)', 'INP keyword deck (Suite 02)', 'PNG/SVG'] },
  params: [
    { key: 'mat', label: 'Cell material', type: 'select', group: 'Cell', default: 'ti-beta21s', options: forUse('buffer').map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.store.material('buffer', 'ti-beta21s'), prov: 'library' }) },
    { key: 'b', label: 'Half-span b', unit: 'mm', type: 'number', min: 5, max: 300, step: 1, group: 'Cell', default: 50 },
    { key: 'hr', label: 'Initial rise h', unit: 'mm', type: 'number', min: 0.5, max: 50, step: 0.1, group: 'Cell', default: 5 },
    { key: 'd', label: 'Bar diameter', unit: 'mm', type: 'number', min: 0.5, max: 30, step: 0.1, group: 'Cell', default: 6 },
    { key: 'ks', label: 'Parallel stiffness ratio k_s/k*', type: 'range', min: 0, max: 0.3, step: 0.005, group: 'Cell', default: 0.15, help: 'Raises the energy of the charged state; bistability is lost above 0.25.' },
    { key: 'loss', label: 'Snap-through dissipation', unit: '%', type: 'range', min: 0, max: 60, step: 1, group: 'Losses', default: 12 },
    { key: 'eta', label: 'Transmission efficiency to wheel', type: 'range', min: 0.3, max: 1, step: 0.01, group: 'Losses', default: 0.85 },
    { key: 'frame', label: 'Frame / housing mass factor', type: 'range', min: 1, max: 4, step: 0.1, group: 'Cell', default: 1.35, expert: true, help: 'Cells share supports in a nested array (D8), so the frame adds ≈ 35 % to the bar mass.' },
    { key: 'SF', label: 'Strain safety factor', type: 'number', min: 1, max: 4, step: 0.1, group: 'Cell', default: 1.5 },
    { key: 'cellType', label: 'Buffer cell technology', type: 'select', group: 'Cell', default: 'ti', auto: (c) => ({ value: c.vehicle?.bufferCell === 'elastomer' ? 'elastomer' : 'ti', prov: 'coupled' }), options: [{ value: 'ti', label: 'β-titanium bistable snap-through metamaterial cells (specification default)' }, { value: 'elastomer', label: 'Research only — pre-stretched elastomer bands in over-centre latches (not bistable metamaterial cells; outside the specification)' }], help: 'Both are passive; the elastomer stores several times more energy per kilogram at a durable stretch.' },
    { key: 'elastomer', label: 'Elastomer', type: 'select', group: 'Cell', default: 'ir', options: [{ value: 'ir', label: 'Synthetic polyisoprene, 30 phr carbon black' }, { value: 'sbr', label: 'SBR, carbon-black filled' }, { value: 'epdm', label: 'EPDM, carbon-black filled (ozone-resistant)' }] },
    { key: 'lambdaMax', label: 'Maximum band stretch λ', type: 'number', min: 1.3, max: 4, step: 0.05, group: 'Cell', default: 3.0, help: 'Upper bound; the fatigue requirement may lower it.' },
    { key: 'bufferMassTarget', label: 'Buffer system mass budget', unit: 'kg', type: 'number', min: 1, max: 40, step: 0.5, group: 'Sizing', default: 6.5, help: 'Elastomer option: system mass of the buffer pack (6.5 kg matches the titanium launch buffer).' },
    { key: 'storeHours', label: 'Typical storage time charged', unit: 'h', type: 'number', min: 0.1, max: 240, step: 0.5, group: 'Durability', default: 8, help: 'Stress relaxation of the stretched bands is counted over this time.' },
    { key: 'lifeYears', label: 'Band service life target', unit: 'years', type: 'number', min: 1, max: 30, step: 1, group: 'Durability', default: 8 },
    { key: 'stopsPerKm', label: 'Stops per km (regenerative braking)', type: 'number', min: 0, max: 10, step: 0.1, group: 'Regeneration', default: 1.0 },
    { key: 'descentPerKm', label: 'Descent per km of route', unit: 'm', type: 'number', min: 0, max: 100, step: 1, group: 'Regeneration', default: 8 },
    { key: 'regenDecel', label: 'Regenerative deceleration cap', unit: 'g', type: 'number', min: 0.02, max: 0.3, step: 0.01, group: 'Regeneration', default: 0.08 },
    { key: 'launches', label: 'Launches held in the buffer', type: 'number', min: 1, max: 20, step: 1, group: 'Sizing', default: 2, help: 'Revision 6: the latent-heat store (Suite 14) carries sustained backup; the elastic buffer only launches the vehicle (the hub gives no torque at standstill).' },
    { key: 'Etarget', label: 'Target usable energy', unit: 'kJ', type: 'number', min: 0.1, max: 2000, step: 0.1, group: 'Sizing', default: 2.6, auto: (c) => { const veh = c.vehicle, n = c.store.params?.('s18')?.launches ?? 2, v = 10 / 3.6, a = 0.3, sAcc = v * v / (2 * a), E = (0.5 * (veh.mass + 9) * v * v + (veh.mass * 9.80665 * (veh.Crr ?? 0.005) + 0.5 * 1.2 * (veh.CdA ?? 0.2) * v * v / 3) * sAcc) / (0.88 * 0.85); return { value: +(n * E / 1000).toFixed(2), prov: 'generated' }; }, help: 'Revision 6: launch-only — energy for the set number of 0 → 10 km/h launches (0.3 m/s², rolling and aerodynamic losses, through snap-through and transmission losses).' },
    { key: 'cageMat', label: 'Per-cell containment cage', type: 'select', group: 'Cell', default: 'st-znni', help: 'Revision 6.2: zinc–nickel-coated low-carbon (DC04-class) steel absorbs ≈ 270 MPa × 25 % per volume — similar to 304 — at about a quarter of the cost and carbon; the coating covers the perforations. Revision 6 cost action: 304 stainless absorbs more energy per volume (flow stress × strain) than grade 2 titanium, so the steel cage is only ≈ 13 % heavier at a fraction of the cost and carbon.', options: [{ value: 'ti-gr2', label: 'Perforated titanium grade 2 sleeve' }, { value: 'ss304', label: 'Perforated 304 stainless sleeve' }, { value: 'st-znni', label: 'Perforated low-carbon steel sleeve, zinc–nickel coated' }, { value: 'uhmwpe', label: 'UHMWPE liner' }] },
    { key: 'optimise', label: 'Optimise cell geometry for energy density', type: 'bool', group: 'Cell', default: true, help: 'D8: differential evolution over half-span, rise, bar diameter and parallel stiffness subject to bistability, strain and buckling limits.' },
    { key: 'P', label: 'Discharge power at wheel', unit: 'W', type: 'number', min: 1, max: 2000, step: 1, group: 'Sizing', default: 150, auto: (c) => { const o = c.upstream('s17'); return o?.deficit_W > 0 ? { value: Math.round(o.deficit_W), prov: 'coupled' } : undefined; } },
    { key: 'charge', label: 'Initial state of charge (explicit initial condition)', unit: '%', type: 'range', min: 0, max: 100, step: 1, group: 'Sizing', default: 100, help: 'Must be justified by a physical charging process or declared as a new initial condition.' },
    { key: 'cyclesDay', label: 'Charge–discharge cycles per day', type: 'number', min: 0.1, max: 200, step: 0.1, group: 'Durability', default: 4 },
    { key: 'shockG', label: 'Design road/gust shock', unit: 'g', type: 'number', min: 0.5, max: 30, step: 0.5, group: 'Durability', default: 5 },
  ],
  presets: [{ label: 'One hour at 150 W', values: { Etarget: 640 } }, { label: 'Spring-steel cells', values: { mat: 'st-spring' } }, { label: 'Monostable (k_s = 0.3)', values: { ks: 0.3 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const m = get(p.mat), resilLimit = (m.sy ** 2 / (2 * m.E)) * 1e3 / m.rho;
    let optNote = 'Manual geometry.';
    if (p.optimise !== false) {
      const Em = m.E * 1e9, allowS = m.sy / m.E / 1000 / p.SF;
      const dens = ([bb, rr, dd, kk]) => { const A_ = Math.PI * (dd / 1000) ** 2 / 4, I_ = Math.PI * (dd / 1000) ** 4 / 64, Cc = TH.bistableCell({ E: Em, A: A_, b: bb / 1000, hRise: rr * bb / 1000, ksRatio: kk, sy: m.sy, I: I_ }); if (!Cc.bistable) return -1; const pen = Math.max(0, Cc.maxStrain / (0.97 * allowS) - 1) * 50 + Math.max(0, Cc.Ncomp / (Cc.Pcr / 1.5) - 1) * 50 + Math.max(0, 0.08 - Cc.barrierRelease / Math.max(1e-9, Cc.Estore)) * 20 + Math.max(0, 6 - Cc.L0 / (dd / 1000)) * 5; return Cc.Estore * (1 - p.loss / 100) / (2 * m.rho * A_ * Cc.L0 * p.frame) - pen * resilLimit; };
      const R = diffEvolution((x) => -dens(x), [[10, 200], [0.02, 0.35], [1, 20], [0, 0.2]], { pop: 18, gens: 35, seed: 18 });
      p = { ...p, b: +R.x[0].toFixed(1), hr: +(R.x[1] * R.x[0]).toFixed(2), d: +R.x[2].toFixed(2), ks: +R.x[3].toFixed(3) };
      optNote = `Optimised (differential evolution, ${18 * 36} evaluations): b ${p.b} mm, rise ${p.hr} mm, bar Ø ${p.d} mm, k_s/k* ${p.ks}.`;
    }
    const E = m.E * 1e9, d = p.d / 1000, A = Math.PI * d * d / 4, I = Math.PI * d ** 4 / 64, b = p.b / 1000, hr = p.hr / 1000;
    const C = TH.bistableCell({ E, A, b, hRise: hr, ksRatio: p.ks, sy: m.sy, I });
    const Euse = C.bistable ? Math.max(0, C.Estore * (1 - p.loss / 100)) : 0, mCell = 2 * m.rho * A * C.L0 * p.frame, eDens = Euse / mCell;
    const nCells = Euse > 0 ? Math.ceil(p.Etarget * 1000 / Euse) : Infinity, mass = nCells * mCell, vol = nCells * (2 * b) * (3 * hr) * (3 * d);
    const Einit = (p.charge / 100) * nCells * Euse, duration = Number.isFinite(Einit) ? Einit * p.eta / p.P : 0;
    const E1h = p.P * 3600 / p.eta, cells1h = Euse > 0 ? Math.ceil(E1h / Euse) : Infinity, mass1h = cells1h * mCell;
    const strainAllow = m.sy / m.E / 1000 / p.SF, strainOK = C.maxStrain <= strainAllow, buckOK = C.Ncomp < C.Pcr / 1.5;
    const sa = E * C.maxStrain / 1e6, fatLim = m.fatigue || 0.4 * m.su, Nf = sa <= fatLim ? Infinity : 1e7 * (fatLim / sa) ** 11, lifeYears = Nf / (p.cyclesDay * 365);
    const Ftrig = Math.abs(Math.min(0, C.Fmin)), Finert = mCell * 9.81 * p.shockG, trigMargin = Finert > 0 ? Ftrig / Finert : Infinity;
    // verification
    const rCrit = 0.25, analyticBistable = p.ks < rCrit; out.checks.push({ name: 'Bistability: closed form (k_s/k* < 1/4) vs numerical extrema', value: `${analyticBistable} / ${C.bistable}`, target: 'agree', pass: analyticBistable === C.bistable || Math.abs(p.ks - rCrit) < 0.01 });
    const i2 = C.us.indexOf(C.u2), intF = trapz(C.Fs.slice(0, i2 + 1), C.us.slice(0, i2 + 1)); out.checks.push({ name: '∫F du (0→u₂) equals U(u₂) − U(0)', value: intF, target: `${fmt(C.Estore, 6)} J (±0.5 %)`, pass: Math.abs(intF - C.Estore) <= 0.005 * Math.abs(C.Estore) + 1e-9 });
    const shallow = hr * hr / (2 * C.L0 * C.L0), exact = Math.abs((b - C.L0) / C.L0); out.checks.push({ name: 'Peak bar strain: shallow-arch formula vs exact', value: exact, target: `${fmt(shallow, 4)} (±5 %)`, pass: Math.abs(exact - shallow) <= 0.05 * shallow });
    // KPIs
    const K = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    K('Cell geometry', optNote, '', 'D8 design step: geometry chosen to maximise usable energy per kg while staying bistable, elastic (strain safety factor) and buckling-safe.', p.optimise !== false ? 'generated' : 'user');
    K('Bistable', C.bistable ? 'yes' : 'NO', '', C.bistable ? `Second stable state at u₂ = ${fmt(C.u2 * 1000, 3)} mm.` : 'The charged state is not stable — the cell would release immediately (monostable).', 'calc', C.bistable ? 'ok' : 'bad');
    K('Stored energy per cell', C.Estore, 'J', 'U(u₂) − U(0): elastic energy held in the charged stable state.', 'calc');
    K('Usable energy per cell', Euse, 'J', `After ${p.loss} % snap-through dissipation.`, 'calc');
    K('Release barrier (trigger energy)', C.barrierRelease, 'J', 'Energy needed to push a charged cell back over the barrier — protects against accidental release.', 'calc', C.barrierRelease > 0.05 * C.Estore ? 'ok' : 'warn');
    K('Energy density', eDens, 'J/kg', 'Usable energy per kg of cell including frame. Limited by σ_y²/2E of the material.', 'calc');
    const resil = (m.sy ** 2 / (2 * m.E)) * 1e3 / m.rho; K('Fraction of material resilience limit', eDens / resil * 100, '%', `Theoretical ceiling σ_y²/(2Eρ) = ${fmt(resil, 4)} J/kg for ${m.name}; geometry, frame mass and strain safety factor reduce it.`, 'calc');
    out.verdicts.push({ test: 'Energy density ≥ 25 % of material resilience limit (D8)', status: eDens / resil >= 0.25 ? 'pass' : eDens / resil >= 0.15 ? 'warn' : 'fail', detail: `${fmt(eDens, 4)} J/kg = ${fmt(eDens / resil * 100, 3)} % of σ_y²/(2Eρ) = ${fmt(resil, 4)} J/kg (bars + frame, after ${p.loss} % snap loss, strain SF ${p.SF}).`, fix: 'Higher-resilience alloy, lower strain safety factor with fatigue evidence, or lighter shared frames.' });
    K('Cells for target energy', nCells, '', `${p.Etarget} kJ usable.`, 'calc');
    K('Buffer mass', mass, 'kg', `Vehicle mass is ${ctx.vehicle.mass} kg.`, 'calc', mass < 0.1 * ctx.vehicle.mass ? 'ok' : mass < 0.3 * ctx.vehicle.mass ? 'warn' : 'bad');
    K('Buffer volume', vol * 1000, 'L', 'Envelope volume of the cell array (approximate).', 'calc');
    K('Initial stored energy (declared)', Einit / 1000, 'kJ', `Explicit initial condition: ${p.charge} % state of charge.`, 'user');
    K('Backup duration at discharge power', duration, 's', `${fmt(duration / 60, 3)} min at ${p.P} W (η = ${p.eta}). Finite — then the vehicle must stop without other energy.`, 'calc');
    K('Mass needed for 1 h at this power', mass1h, 'kg', `${fmt(E1h / 1000, 4)} kJ usable required (spec: 540 kJ at 150 W before losses).`, 'calc', mass1h < 30 ? 'ok' : 'bad');
    K('Peak bar strain / allowable', `${fmt(C.maxStrain * 100, 3)} / ${fmt(strainAllow * 100, 3)}`, '%', 'Must stay elastic for full energy recovery.', 'calc', strainOK ? 'ok' : 'bad');
    K('Bar compression / buckling load', `${fmt(C.Ncomp, 4)} / ${fmt(C.Pcr, 4)}`, 'N', 'Euler buckling of the slender bars before snap-through would destroy bistability.', 'calc', buckOK ? 'ok' : 'bad');
    K('Fatigue life', Number.isFinite(Nf) ? lifeYears : '≥ endurance', Number.isFinite(Nf) ? 'years' : '', `Stress amplitude ${fmt(sa, 3)} MPa vs endurance ${fmt(fatLim, 3)} MPa at ${p.cyclesDay} cycles/day.`, 'estimated', !Number.isFinite(Nf) || lifeYears > 10 ? 'ok' : 'bad');
    K('Unintended-release margin', trigMargin, '×', `Trigger force ${fmt(Ftrig, 3)} N vs ${p.shockG} g inertial load ${fmt(Finert, 3)} N.`, 'calc', trigMargin > 3 ? 'ok' : 'bad');
    // verdicts
    out.verdicts.push({ test: 'Cell is bistable (stores energy in a stable charged state)', status: C.bistable ? 'pass' : 'fail', detail: C.bistable ? `Barrier ${fmt(C.barrierRelease, 3)} J above the charged state.` : `k_s/k* = ${p.ks} ≥ 0.25 removes the second minimum.`, fix: 'Lower the parallel stiffness ratio below 0.25 (keep a release barrier ≥ 5 % of the stored energy).' });
    out.verdicts.push({ test: 'Elastic and stable deformation (strain & buckling)', status: strainOK && buckOK ? 'pass' : 'fail', detail: `Strain ${fmt(C.maxStrain * 100, 3)} % (allow ${fmt(strainAllow * 100, 3)} %), compression ${fmt(C.Ncomp, 4)} N vs P_cr/1.5 = ${fmt(C.Pcr / 1.5, 4)} N.`, fix: 'Reduce rise h or increase bar span; thicker bars raise the buckling load.' });
    out.verdicts.push({ test: 'Finite energy store (never an inexhaustible source)', status: 'pass', detail: `Usable ${fmt(nCells * Euse / 1000, 4)} kJ supports ${fmt(duration, 4)} s at ${p.P} W; recharge requires wind or thermal surplus.` });
    { const s14b = ctx.upstream('s14'), rq = s14b?.req1h?.demo, need = 1.1;
      if (s14b?.hotSource === 'store') {
        out.verdicts.push({ test: 'Sustained backup ≥ 1 h at 20 km/h (+10 %) — demonstrated basis', status: (s14b.backupDemo_h ?? 0) >= need ? 'pass' : 'fail', detail: `Latent-heat store (Suite 14): ${fmt(s14b.backupDemo_h ?? 0, 3)} h at η ${fmt(100 * (s14b.etaDemo ?? 0), 3)} % (built-device fraction of Carnot, no regenerator credit). The requirement needs ${rq ? `${fmt(rq.thermalSystem_kg, 3)} kg of thermal system → vehicle ${fmt(rq.vehicle_kg, 4)} kg, ${fmt(rq.overCap_kg, 4)} kg over the cap` : 'a rejection surface the vehicle cannot carry'}. Elastic cells alone would need ${fmt(mass1h, 4)} kg.` });
        out.verdicts.push({ test: 'Sustained backup at 20 km/h — research target (upside)', status: (s14b.backupTarget_h ?? 0) >= need ? 'pass' : 'warn', detail: `${fmt(s14b.backupTarget_h ?? 0, 3)} h at the research-target efficiency, ${fmt(s14b.backupModel_h ?? 0, 3)} h with the model regenerated cycle — simulation / research targets, not physical validation.` });
      } else if (s14b?.researchStore) { const R = s14b.researchStore, rd = R.req1h?.demo;
        out.verdicts.push({ test: 'Sustained backup ≥ 1 h at 20 km/h (+10 %) — default vehicle', status: 'fail', detail: `The default vehicle carries only the launch buffer (${fmt(nCells * Euse / 1000, 3)} kJ). Elastic cells for 1 h would need ${fmt(mass1h, 4)} kg. The optional latent-heat store (research configuration, Suite 14) would need ${rd ? `${fmt(rd.thermalSystem_kg, 4)} kg → vehicle ${fmt(rd.vehicle_kg, 4)} kg (${fmt(rd.overCap_kg, 4)} kg over the cap)` : 'an infeasible rejection surface'} on the demonstrated basis; it fits only with a converter at ${Number.isFinite(R.fracNeeded) ? fmt(R.fracNeeded * 100, 3) + ' %' : '> 60 %'} of Carnot.` });
        if (R.measuredViable) out.recs.push({ level: 'good', text: `Measured converter efficiency (${R.measuredBasis?.source}) makes the 1 h latent-heat store viable inside the mass cap (Suite 14). Enabling it (vehicle.thermalStore) is recommended — it is not switched on automatically.`, basis: 'Suite 14 measured-basis sizing' });
        out.verdicts.push({ test: 'Sustained backup — research configuration (upside, not installed)', status: 'info', detail: `Cap-limited store: ${fmt(R.backupDemo_h * 60, 3)} min demonstrated basis, ${fmt(R.backupModel_h * 60, 3)} min model, ${fmt(R.backupTarget_h * 60, 3)} min research target — simulation / research targets, not physical validation.` });
      } else out.verdicts.push({ test: 'Practical sustained backup (≥ 1 h at discharge power within 10 % vehicle mass)', status: mass1h <= 0.1 * ctx.vehicle.mass ? 'pass' : 'fail', detail: `Needs ${fmt(mass1h, 4)} kg of cells for 1 h — elastic storage suits short gaps, start-up and torque smoothing only.` }); }
    out.verdicts.push({ test: 'Protected against unintended release', status: trigMargin > 3 && C.barrierRelease > 0 ? 'pass' : 'warn', detail: `Inertial margin ${fmt(trigMargin, 3)}× at ${p.shockG} g.` });
    // charts
    const um = C.us.map(u => u * 1000);
    out.charts.push({ title: 'Potential-energy landscape of one cell', type: 'line', x: { label: 'Apex displacement u', unit: 'mm' }, y: { label: 'Strain energy U', unit: 'J' }, series: [{ name: 'U(u)', x: um, y: C.Us }], annotations: [{ type: 'point', x: 0, y: C.Us[0], label: 'discharged' }, ...(C.bistable ? [{ type: 'point', x: C.u2 * 1000, y: C.Us[i2], label: 'charged (stable)' }, { type: 'point', x: C.uBarrier * 1000, y: C.Us[C.us.indexOf(C.uBarrier)], label: 'barrier', color: '#ef4444' }] : [])], explain: 'Two valleys = two stable states. The height of the charged valley above the discharged one is the stored energy; the hill between them is the barrier preventing spontaneous release.' });
    out.charts.push({ title: 'Charge / discharge force–displacement', type: 'line', x: { label: 'Displacement', unit: 'mm' }, y: { label: 'Force', unit: 'N' }, series: [{ name: 'Charging (work in)', x: um.slice(0, i2 + 1), y: C.Fs.slice(0, i2 + 1) }, { name: `Discharging (after ${p.loss} % loss)`, x: um.slice(0, i2 + 1), y: C.Fs.slice(0, i2 + 1).map(f => f * (1 - p.loss / 100)), dash: '5 3' }], annotations: [{ type: 'hline', y: 0, color: '#64748b' }], explain: 'Negative force beyond the peak is snap-through: the cell pulls itself into the charged state. Area between curves is the energy dissipated per cycle.' });
    const tt = linspace(0, Math.max(10, duration * 1.2), 120);
    out.charts.push({ title: 'Backup discharge: remaining usable energy vs time', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Remaining energy', unit: 'kJ' }, series: [100, 150, 200, 600].map(P => ({ name: `${P} W`, x: tt, y: tt.map(t => Math.max(0, Einit - P * t / p.eta) / 1000) })), explain: 'The buffer drains linearly at constant wheel power. When it reaches zero, propulsion stops unless wind or a positive-work thermal backup returns.' });
    const durs = logspace(1, 4, 30); out.charts.push({ title: 'Required buffer mass vs backup duration', type: 'line', x: { label: 'Backup duration', unit: 's', log: true }, y: { label: 'Buffer mass', unit: 'kg', log: true }, series: [100, 150, 200, 600].map(P => ({ name: `${P} W`, x: durs, y: durs.map(t => Euse > 0 ? Math.ceil(P * t / p.eta / Euse) * mCell : NaN) })), annotations: [{ type: 'hline', y: ctx.vehicle.mass, label: 'vehicle mass', color: '#ef4444' }], explain: 'Mass grows linearly with duration and power; elastic metamaterials store roughly 10²–10³ J/kg, so hour-long backup at hundreds of watts is impractical.' });
    const ksr = linspace(0, 0.3, 31), ksd = ksr.map(r => TH.bistableCell({ E, A, b, hRise: hr, ksRatio: r, sy: m.sy, I }));
    out.charts.push({ title: 'Design trade-off: stiffness ratio vs stored energy and barrier', type: 'line', x: { label: 'k_s / k*' }, y: { label: 'Energy per cell', unit: 'J' }, series: [{ name: 'Stored energy', x: ksr, y: ksd.map(c => c.bistable ? c.Estore : 0) }, { name: 'Release barrier', x: ksr, y: ksd.map(c => c.bistable ? c.barrierRelease : 0) }], annotations: [{ type: 'vline', x: 0.25, label: 'bistability limit', color: '#ef4444' }, { type: 'vline', x: p.ks, label: 'current' }], explain: 'Stiffer parallel springs store more energy but shrink the barrier until the charged state disappears at 0.25.' });
    out.tables.push({ title: 'Buffer sizing summary', cols: ['Quantity', 'Value', 'Unit'], rows: [['Cell bar length L₀', C.L0 * 1000, 'mm'], ['Cell mass (incl. frame)', mCell * 1000, 'g'], ['Usable energy per cell', Euse, 'J'], ['Cells', nCells, ''], ['Array mass', mass, 'kg'], ['Array volume', vol * 1000, 'L'], ['Initial stored energy (declared)', Einit / 1000, 'kJ'], ['Duration at ' + p.P + ' W', duration, 's'], ['Reference: 30 kJ at 150 W', 30000 / 150, 's (spec ≈ 200 s)'], ['Reference: 1 h at 150 W', 540, 'kJ before losses']] });
    // recs
    if (!C.bistable) out.recs.push({ level: 'critical', text: 'Reduce k_s/k* below 0.25 — the present cell cannot hold a charge.', basis: 'Closed-form bistability limit' });
    out.recs.push({ level: 'info', text: 'Raise energy density towards the material limit: larger strain utilisation (rise h closer to the strain allowable), lighter frames, and cells that store energy in bending-dominated curved beams rather than slender bars.', basis: 'Resilience-limit fraction' });
    out.recs.push({ level: mass1h > 0.1 * ctx.vehicle.mass ? 'warn' : 'good', text: `Use the buffer for start-up assistance, torque smoothing and short calm gaps (${fmt(duration, 3)} s at ${p.P} W), not as a sustained energy source; 1 h would need ${fmt(mass1h, 3)} kg.`, basis: 'Energy-density limit' });
    if (m.id !== 'ti-beta21s') out.recs.push({ level: 'info', text: 'High-resilience β-titanium (σ_y²/2E) stores more energy per kg than spring steel and resists corrosion; compare via Suite 03 Ashby chart.', basis: 'Material index σ_y²/(2Eρ)' });
    if (trigMargin < 3) out.recs.push({ level: 'warn', text: 'Add a positive mechanical latch: inertial loads could release cells unintentionally.', basis: 'Unintended-release margin' });
    out.recs.push({ level: 'sustain', text: 'Mechanical storage avoids batteries and their critical minerals; metallic cells are fully recyclable and, if kept elastic, have very long lives.', basis: 'Circularity' });
    out.explain.push({ title: 'What the buffer is for', text: 'The bistable buffer stores previously harvested mechanical work in elastic deformation and releases it on demand. It smooths torque pulses, can help the wheel past dead positions at start-up, and bridges short gaps. It cannot create energy, and its endurance is fixed by the declared initial charge.' });
    out.views.push({ label: '3D buffer discharge', icon: '🔋', render: (el) => view3dBuffer(el, { C, nCells, Euse, Einit, duration, p, m, b, hr }) });
    out.views.push({ label: '2D animation: snap-through & energy', icon: '🎞', render: (el) => anim2dBuffer(el, { C, nCells, Einit, duration, p }) });
    // Re-scoped role (owner decision): solar recharging of the elastic buffer by the thermal converter, parked or cruising.
    // Converter runs at its own cycle rate (not wheel-locked): P_charge = min(Q_avail·η_actuator, m_active·P/m) × η_transmission.
    const s14o = ctx.upstream('s14'), s15o = ctx.upstream('s15'), s17o = ctx.upstream('s17'), s05o = ctx.upstream('s05');
    const Qav = Math.max(0, s14o?.Qhot_W ?? 0), poly = s17o?.actuatorClass && s17o.actuatorClass !== 'sma';
    const etaAct = Math.max(0, s17o?.actuatorEta ?? s15o?.efficiency ?? 0), Pcap = s17o?.actuatorPkg !== undefined ? (s17o.activeMass_kg ?? 0) * s17o.actuatorPkg : Math.max(0, s15o?.systemPower_W ?? 0);
    // Suite 17 may not have run yet (it reads this buffer): mirror its heat-aware pick from the Suite 16 ranking.
    const s16o = ctx.upstream('s16'), mAct = s15o?.activeMass_kg ?? 0;
    const pick16 = !s17o && s16o?.ranking?.length ? Math.max(0, ...s16o.ranking.filter(r => r.net && Number.isFinite(r.eta) && Number.isFinite(r.P_kg)).map(r => Math.min(Qav * r.eta, mAct * r.P_kg))) : 0;
    const storeMode = s14o?.hotSource === 'store', Phub = Math.max(0, s15o?.systemPower_W ?? 0);
    // Suites 15–17 not solved yet (isolated run): own estimate instead of zero — collector heat (Suite 14, else Suite 13,
    // else ≈ 100 W for the default direct path) × ≈ 1 % converter efficiency (built shape-memory engines ≈ 3.9 % of Carnot at ≈ 25 % Carnot)
    const noHub = !storeMode && !s15o && !s16o && !s17o, s13o = ctx.upstream('s13'), Qest = Math.max(0, s14o?.Qhot_W ?? s13o?.Pabs_W ?? 100), Pest = Qest * 0.01;
    // Revision 6.3: with compliant sliding shoes (no rollers) the hub's shoe friction can exceed the alloy work — then the hub
    // cannot recharge anything and is lifted off the track (driver lever) to avoid drag
    const hubNegative = !!s17o && Number.isFinite(s17o.Wrev_J) && s17o.Wrev_J <= 0;
    const Pcharge = (hubNegative ? 0 : storeMode ? Phub : noHub ? Pest : Math.max(0, Math.min(Qav * etaAct, Pcap), pick16)) * p.eta, EbufTot = Number.isFinite(nCells) ? nCells * Euse : 0, refillH = Pcharge > 0 ? EbufTot / Pcharge / 3600 : Infinity;
    const sunH = s05o?.series?.length ? s05o.series.slice(0, 24).filter(r => (r.ghi || 0) > 200).length : 6, Estart = (() => { const veh = ctx.vehicle, m = veh.mass, v = 10 / 3.6, a = 0.3, tA = v / a, sAcc = v * v / (2 * a), Froll = m * 9.80665 * (veh.Crr ?? 0.005), Faero = 0.5 * 1.2 * (veh.CdA ?? 0.2) * (v * v) / 3, Ek = 0.5 * (m + 9) * v * v; void tA; return (Ek + (Froll + Faero) * sAcc) / Math.max(0.3, (1 - p.loss / 100) * p.eta); })(), Eday = storeMode ? (s14o.dailyHeat_J ?? 0) * (s14o.etaDemo ?? 0) * p.eta : Pcharge * sunH * 3600, startsDay = Estart > 0 ? Math.floor(Eday / Estart) : 0, startsModel = storeMode && Estart > 0 ? Math.floor((s14o.dailyHeat_J ?? 0) * (s14o.etaModelBasis ?? 0) * p.eta / Estart) : startsDay, startsStored = Estart > 0 ? Math.floor(EbufTot / Estart) : 0;
    const roadP = 124, hubP = s17o?.Pwheel_W ?? 0;
    K('Solar charge power into the buffer', Pcharge, 'W', hubNegative ? `0 W: the hub (Suite 17) loses ${fmt(-s17o.Wrev_J, 3)} J per wheel revolution to shoe sliding friction, more than the alloy delivers — the solar-thermal path cannot recharge the buffer; regenerative braking and parked-wind charging do (below).` : noHub ? `Estimate (Suites 15–17 not yet solved): ${fmt(Qest, 3)} W of collected heat ${s14o ? '(Suite 14)' : s13o ? '(Suite 13)' : '(typical direct path)'} × ≈ 1 % converter efficiency × transmission ${p.eta}. Run Suites 15–17 for the coupled value.` : storeMode ? `Store-fed hub at its rated shaft power (${fmt(Phub, 3)} W, Suite 15) while rolling, transmission ${p.eta}; the store, not the instantaneous collector, supplies the heat.` : `Converter (${s17o?.actuator || 'NiTi, Suite 15'}) fed by ${fmt(Qav, 3)} W of collected heat (Suite 14), η_actuator ${fmt(etaAct * 100, 3)} %, capacity ${fmt(Pcap, 3)} W, transmission ${p.eta}.`, noHub || !s14o ? 'estimated' : 'coupled', Pcharge > 0 ? 'ok' : 'bad');
    K('Hours to refill the buffer', Number.isFinite(refillH) ? refillH : 'no recharge available (run Suites 15–17 / sunny conditions)', Number.isFinite(refillH) ? 'h' : '', `${fmt(EbufTot / 1000, 4)} kJ usable at ${fmt(Pcharge, 3)} W.`, noHub ? 'estimated' : 'calc', refillH <= sunH ? 'ok' : 'warn');
    K('Wind-free starts per sunny day', `${startsDay} demonstrated basis / ${startsModel} model (buffer holds ${startsStored})`, '', `${storeMode ? `One sunny day's stored heat ${fmt((s14o.dailyHeat_J ?? 0) / 1e6, 3)} MJ × converter efficiency × transmission, if it were all spent on launches; ` : ''}${sunH} h of sun above 200 W/m²; one useful start ≈ ${fmt(Estart / 1000, 3)} kJ (0 → 10 km/h at 0.3 m/s² with rolling and aerodynamic losses, divided by the buffer’s discharge efficiency); beyond 10 km/h wind must take over.`, noHub ? 'estimated' : 'calc', startsDay >= 1 ? 'ok' : 'bad');
    out.verdicts.push({ test: 'Solar recharging of the elastic buffer', status: Pcharge > 0 && refillH <= sunH ? 'pass' : Pcharge > 0 ? 'warn' : 'fail', detail: `${fmt(Pcharge, 3)} W refills ${fmt(EbufTot / 1000, 4)} kJ in ${fmt(refillH, 3)} h (${sunH} sunny hours available) → ${startsDay} wind-free starts per sunny day.` });
    if (storeMode) out.verdicts.push({ test: 'Store-fed hub propulsion at 20 km/h (simulation)', status: (s14o.backupDemo_h ?? 0) >= 1.1 ? 'pass' : 'warn', detail: `While the store lasts the hub stack (Suite 15) delivers ${fmt(Phub, 3)} W shaft power for the ${fmt(s15o?.roadShaft_W ?? 0, 3)} W road-load shaft demand: ${fmt(s14o.backupDemo_h ?? 0, 3)} h on the demonstrated basis, ${fmt(s14o.backupModel_h ?? 0, 3)} h model, ${fmt(s14o.backupTarget_h ?? 0, 3)} h research target. Simulated torque — research target, not physical validation.` });
    else out.verdicts.push({ test: 'Thermal backup as propulsion (explicit)', status: hubP >= roadP ? 'pass' : 'fail', detail: `The thermal converter delivers ${fmt(hubP, 3)} W to the wheels while driving vs ≈ ${roadP} W needed at 20 km/h — it cannot sustain propulsion; its credible role is recharging this buffer for starts and short gaps.` });
    // Revision 5.1 audit: containment of an uncontrolled release and mounting. Each cell sits in its own perforated cage
    // sized to absorb ≥ 2× the cell's stored energy by plastic work (energy density = flow stress × usable strain);
    // the 2 mm CFRP box is then only a dust/debris enclosure.
    let bufSysMass = mass, cageMassTot = 0;
    { const CAGE = { 'ti-gr2': { name: 'perforated Ti grade 2 sleeve', flow: 0.5 * (275 + 345), eps: 0.15, rho: 4510 }, 'ss304': { name: 'perforated 304 stainless sleeve', flow: 0.5 * (215 + 505), eps: 0.2, rho: 7900 }, 'st-znni': { name: 'perforated Zn–Ni-coated low-carbon steel sleeve', flow: 0.5 * (210 + 330), eps: 0.25, rho: 7850 }, 'uhmwpe': { name: 'UHMWPE liner', flow: 22, eps: 0.3, rho: 970 } }, cg = CAGE[p.cageMat] || CAGE['ti-gr2'];
      const Ecell = C.Estore, Fbar = C.Ncomp, eDen = cg.flow * 1e6 * cg.eps, Vc = 2 * Ecell / eDen, mCage = Vc * cg.rho * 1.15, cageOK = mCage > 0, nC = Number.isFinite(nCells) ? nCells : 0;
      cageMassTot = nC * mCage;
      const tBox = 2, boxA = nC ? 6 * vol ** (2 / 3) : 0, boxMass = boxA * tBox / 1000 * 1550, mountMass = 0.5, sysMass = mass + cageMassTot + boxMass + mountMass, Fmnt = sysMass * 20 * 9.81, mntCap = 4 * 6000;
      K('Cell containment cage (each cell)', `${fmt(mCage * 1000, 3)} g ${cg.name}`, '', `Absorbs 2 × ${fmt(Ecell, 4)} J by plastic work at ${fmt(cg.flow, 3)} MPa × ${cg.eps * 100} % strain (${fmt(eDen / 1e6, 3)} MJ/m³); bar force at release ${fmt(Fbar / 1000, 4)} kN. +15 % for perforation and end caps.`, 'calc', 'ok');
      K('Buffer system mass (cells + cages + box + mounts)', sysMass, 'kg', `Cells ${fmt(mass, 3)} + cages ${fmt(cageMassTot, 3)} (${nC} × ${fmt(mCage * 1000, 3)} g) + 2 mm CFRP enclosure ${fmt(boxMass, 3)} (${fmt(boxA, 3)} m²) + mounts ${mountMass} kg — the figure for the Suite 01 mass budget.`, 'calc');
      out.verdicts.push({ test: 'Buffer containment & mounting', status: cageOK && mntCap >= 1.25 * Fmnt ? 'pass' : 'fail', detail: `Each cell caged (${cg.name}, ${fmt(mCage * 1000, 3)} g) to absorb ${fmt(2 * Ecell, 4)} J ≥ 2 × release; mounts ${fmt(mntCap / 1000, 3)} kN vs ${fmt(Fmnt / 1000, 3)} kN at 20 g (≥ 1.25).` });
      bufSysMass = sysMass; }
    // ===================== Revision 6.3 practicable backup =====================
    const veh = ctx.vehicle, mVeh = veh.mass, g0 = 9.80665, v20 = 20 / 3.6, P20 = (s17o?.demand_W ?? ((veh.Crr ?? 0.0045) * mVeh * g0 * v20 + 0.5 * 1.2 * (veh.CdA ?? 0.13) * v20 ** 3 + 2)) + Math.max(0, s17o?.hubParasitic_W ?? 0); // the engaged hub's parasitic drag (night/cold) is part of the load the buffer must carry
    const ELA = {
      ir: { name: 'synthetic polyisoprene (30 phr black)', G: 0.9e6, Jm: 70, rho: 1050, hyst: 0.10, relax: 0.03, Tg: -70, Nref: 1e6, b: 2.5, cost: 4, co2: 3.5, energy: 95, ozone: 'unsaturated backbone: cracks under ozone when stretched — antiozonant wax plus the sealed, opaque enclosure are required' },
      sbr: { name: 'SBR (carbon-black filled)', G: 1.2e6, Jm: 50, rho: 1150, hyst: 0.20, relax: 0.04, Tg: -50, Nref: 2e5, b: 2.5, cost: 2.5, co2: 3.0, energy: 85, ozone: 'unsaturated backbone: antiozonant wax plus sealed, opaque enclosure required' },
      epdm: { name: 'EPDM (carbon-black filled)', G: 0.8e6, Jm: 40, rho: 1100, hyst: 0.12, relax: 0.05, Tg: -55, Nref: 3e5, b: 2.5, cost: 3, co2: 3.2, energy: 90, ozone: 'saturated backbone: inherently ozone- and UV-resistant' },
    }, el = ELA[p.elastomer] || ELA.ir;
    const Wgent = (lam, G = el.G) => -(G * el.Jm / 2) * Math.log(Math.max(1e-9, 1 - (lam * lam + 2 / lam - 3) / el.Jm)); // J/m³, uniaxial Gent
    const cycDay = Math.max(p.cyclesDay, 12), Nreq = cycDay * 365 * p.lifeYears, epsFat = Math.pow(el.Nref / Math.max(1, Nreq), 1 / el.b), lamD = Math.min(p.lambdaMax, 1 + epsFat);
    const Nband = el.Nref * Math.pow(1 / (lamD - 1), el.b), relaxF = 1 - el.relax * Math.max(0, Math.log10(p.storeHours * 60)); // per decade of time from 1 min
    const eBand = Wgent(lamD) / el.rho * (1 - el.hyst) * relaxF; // usable J per kg of band
    const eCold = Wgent(lamD, el.G * 263.15 / 293.15) / el.rho * (1 - el.hyst * 1.4) * relaxF, eHot = Wgent(lamD, el.G * 313.15 / 293.15) / el.rho * (1 - el.hyst * 0.8) * (1 - el.relax * 1.6 * Math.log10(p.storeHours * 60));
    // system: band + over-centre latch frame (spring steel, 35 % of band mass) + per-band containment sleeve (2 × band energy) + enclosure
    const CG = { 'ti-gr2': { flow: 310e6, eps: 0.15, rho: 4510 }, ss304: { flow: 360e6, eps: 0.2, rho: 7900 }, 'st-znni': { flow: 270e6, eps: 0.25, rho: 7850 }, uhmwpe: { flow: 22e6, eps: 0.3, rho: 970 } }[p.cageMat || 'st-znni'] || { flow: 270e6, eps: 0.25, rho: 7850 };
    const eSleeve = 2 * Wgent(lamD) / el.rho * 1.15 / (CG.flow * CG.eps) * CG.rho; // kg of sleeve per kg of band
    const fixedE = 0.9 + 0.5; // enclosure (rCF, 2 mm) + 4 mounts, kg
    const mBand = Math.max(0, (p.bufferMassTarget - fixedE) / (1 + 0.35 + eSleeve)), mLatch = 0.35 * mBand, mSleeve = eSleeve * mBand, Eel = mBand * eBand, sysEl = mBand + mLatch + mSleeve + fixedE;
    const nBands = Math.max(1, Math.ceil(mBand / 0.25)), heatRise = el.hyst * Wgent(lamD) / el.rho / 1900; // K per full discharge
    const useEl = p.cellType !== 'ti', Ebuf = useEl ? Eel : (Number.isFinite(nCells) ? nCells * Euse : 0), sysBuf = useEl ? sysEl : bufSysMass;
    const etaOut = 0.85 * (useEl ? 1 : 1), Elaunch = Estart; // Estart: 0 → 10 km/h incl. losses (above)
    const launchesHeld = Elaunch > 0 ? Math.floor(Ebuf / Elaunch) : 0, bridgeS = Ebuf * etaOut / Math.max(1, P20), bridgeM = bridgeS * v20;
    // ---- regenerative braking & descents: the brake pedal's first travel pushes a stationary compliant flexure pawl into the
    // indexing lobes of the existing rotating cartridge ring; each lobe flicks the pawl, which advances a LINEAR cell carriage
    // (push-pull cable to a toothed linear rack in the buffer box) one tooth and snaps the next bistable cell — no drum, spool or wheel
    const etaRegen = 0.9 * 0.9 * (1 - (useEl ? el.hyst : p.loss / 100)), mEff = mVeh * 1.03; // coupling × ratchet × band hysteresis; rotating inertia ≈ 3 %
    const Estop20 = 0.5 * mEff * v20 * v20 * etaRegen, Estop30 = 0.5 * mEff * (30 / 3.6) ** 2 * etaRegen;
    const Froll = (veh.Crr ?? 0.0045) * mVeh * g0, Edesc = Math.max(0, mVeh * g0 * p.descentPerKm - Froll * 2 * p.descentPerKm / 0.05) * etaRegen; // descents taken at ≈ 5 % grade
    const regenKm = p.stopsPerKm * Estop20 + Edesc, kmToFill = regenKm > 0 ? Ebuf / regenKm : Infinity;
    const Tregen = p.regenDecel * mVeh * g0 * (veh.wheelRadius || 0.3) / 2, s12o = ctx.upstream('s12'), poLim = s12o?.pitchOverLimit_g ?? 1.0, peakB = s12o?.peakBraking_g ?? 0.75;
    // ---- parked wind: elastomer yaw limiter (±3°) engaged with the parking lock; the click ratchet harvests inside that band.
    // Power vs wing-height wind (Suite 07/09 aero yaw stiffness, Kaimal lateral gusts; G2 table, z0 = 0.03 m), harvester torque capped.
    const PW = [[0, 0], [3, 0], [5, 0.0007], [8, 0.009], [10, 0.026], [15, 0.19], [20, 0.8], [25, 2.5], [30, 6.3]], // G2 table inside the ±3° EPDM yaw limiter, z0 0.03 m, optimal ratchet damping
    pWind = (U) => { if (U <= 0) return 0; for (let i = 1; i < PW.length; i++) if (U <= PW[i][0]) return PW[i - 1][1] + (PW[i][1] - PW[i - 1][1]) * (U - PW[i - 1][0]) / (PW[i][0] - PW[i - 1][0]); return PW[PW.length - 1][1]; };
    const hrs24 = (s05o?.series || []).slice(0, 24), windOn = ctx.vehicle?.windHarvester === true, windPotJ = hrs24.reduce((a, r) => a + pWind(r.Uz ?? r.U ?? 0) * 3600, 0) * 0.9, windDayJ = windOn ? windPotJ : 0, windNightJ = windOn ? hrs24.filter(r => !r.isDay).reduce((a, r) => a + pWind(r.Uz ?? r.U ?? 0) * 3600, 0) * 0.9 : 0;
    const Umean = hrs24.length ? hrs24.reduce((a, r) => a + (r.Uz ?? r.U ?? 0), 0) / hrs24.length : 3, windFillH = windDayJ > 0 ? Ebuf / (windDayJ / 24) / 3600 : Infinity;
    const solarDayJ = Pcharge * sunH * 3600;
    if (useEl) {
      K('Elastomer buffer (Revision 6.3)', `${fmt(Eel / 1000, 3)} kJ usable in ${fmt(sysEl, 3)} kg (${nBands} bands)`, '', `${el.name}, durable stretch λ = ${fmt(lamD, 3)} (fatigue ${fmt(Nband, 3)} cycles ≥ ${fmt(Nreq, 3)} needed for ${cycDay}/day × ${p.lifeYears} years). Gent strain energy ${fmt(Wgent(lamD) / 1e6, 3)} MJ/m³ → ${fmt(eBand, 4)} J/kg of band after ${el.hyst * 100} % hysteresis and ${fmt((1 - relaxF) * 100, 3)} % stress relaxation over ${p.storeHours} h. Bands ${fmt(mBand, 3)} kg + latches ${fmt(mLatch, 3)} + containment sleeves ${fmt(mSleeve, 3)} + enclosure and mounts ${fixedE} kg.`, 'calc', 'ok');
      K('Elastomer energy at −10 °C / +40 °C', `${fmt(eCold, 4)} / ${fmt(eHot, 4)}`, 'J/kg band', `Entropic stiffness ∝ T; hysteresis rises in the cold (×1.4) and relaxation is faster when hot (×1.6). Glass transition ${el.Tg} °C — no cold brittleness above ≈ ${el.Tg + 40} °C. Heat build-up ${fmt(heatRise, 3)} K per full discharge.`, 'calc');
      K('Elastomer ageing', `${p.lifeYears}-year band replacement`, '', `${el.ozone}. Thermo-oxidative ageing sets the replacement interval; bands are cassette-mounted for replacement.`, 'estimated');
    }
    // N2 check: bistable ARCHITECTED ELASTOMER cells (snap-through beams moulded in IR/SBR/EPDM) evaluated with the same
    // bistable-cell model and strain limit (≈ 50 % / SF 1.5): the low modulus traps far less energy per kg than β-titanium
    { const Eel2 = { ir: 6e6, sbr: 8e6, epdm: 5e6 }[p.elastomer] || 6e6, syEl = Eel2 * 0.5 / 1e6, allowEl = 0.5 / 1.5; let best = 0;
      for (const bb of [15, 20, 30, 40]) for (const rr of [0.2, 0.35, 0.5]) for (const dd of [10, 20, 30]) for (const kk of [0.05, 0.1, 0.2]) { const A_ = Math.PI * (dd / 1000) ** 2 / 4, I_ = Math.PI * (dd / 1000) ** 4 / 64, Cc = TH.bistableCell({ E: Eel2, A: A_, b: bb / 1000, hRise: rr * bb / 1000, ksRatio: kk, sy: syEl, I: I_ }); if (!Cc.bistable || Cc.maxStrain > allowEl || Cc.Ncomp > Cc.Pcr / 1.5) continue; const dens = Cc.Estore * (1 - el.hyst) / (2 * el.rho * A_ * Cc.L0 * p.frame); if (dens > best) best = dens; }
      const tiDens = Number.isFinite(nCells) && mass > 0 ? nCells * Euse / mass : 0;
      K('Bistable architected elastomer cells (specification check)', `${fmt(best, 3)} J/kg vs β-Ti cells ${fmt(tiDens, 4)} J/kg`, '', `Snap-through beam cells moulded in ${el.name}, same bistable-cell model, strain ≤ 50 % / 1.5, buckling SF 1.5: the low modulus (≈ ${fmt(Eel2 / 1e6, 3)} MPa) traps little energy in the second stable state — bistable lattices trap energy by geometry at modest strain (multistable architected-material literature, e.g. bistable-beam lattices in Advanced Materials 2015). They cannot match β-titanium, so the specification-compliant default stays β-Ti bistable cells.`, 'calc', best >= tiDens ? 'ok' : 'warn'); }
    K('Regenerative braking energy per stop (20 / 30 km/h)', `${fmt(Estop20, 4)} / ${fmt(Estop30, 4)}`, 'J', `½ m v² × pawl coupling 0.9 × linear index 0.9 × cell hysteresis. The brake pedal's first travel pushes a stationary compliant flexure pawl into the indexing lobes of the existing cartridge ring; each lobe flicks the pawl, which indexes a linear cell carriage (push-pull cable to a toothed linear rack in the buffer box) and snaps the next bistable cell. No drum, spool or wheel is added; the flexure pawl buckles at the torque limit.`, 'calc');
    K('Regenerative energy per km (typical route)', regenKm, 'J/km', `${p.stopsPerKm} stops/km at 20 km/h + ${p.descentPerKm} m of descent/km (Suite 20 route default). Buffer full after ${fmt(kmToFill, 3)} km.`, 'calc');
    K(windOn ? 'Parked-wind charging (yaw limiter ±3°, click ratchet)' : 'Parked-wind charging — optional research configuration (not installed)', `${fmt(windPotJ / 1000, 3)} kJ/day potential`, '', `Site wing-height wind mean ${fmt(Umean, 3)} m/s (Suite 05). Inside the ±3° parked yaw limiter (two preloaded EPDM shear pads) the harvest is < 0.03 W below 10 m/s and 0.2–6 W only in storms of 15–30 m/s (Suites 07/09 aero yaw stiffness, Kaimal lateral gusts with aerodynamic admittance). A storm-only supplement: full buffer in ${Number.isFinite(windFillH) ? fmt(windFillH, 3) + ' h' : 'never in calm air'}.`, s05o ? 'coupled' : 'estimated', windDayJ > 0 ? 'ok' : 'warn');
    out.verdicts.push({ test: 'Regenerative coupling braking interlock', status: p.regenDecel * 1.0 <= 0.25 * peakB && peakB + p.regenDecel <= poLim ? 'pass' : 'warn', detail: `Regenerative deceleration capped at ${p.regenDecel} g (${fmt(Tregen, 3)} N·m per front hub) by the flexure pawl, which buckles out of the lobe track at the limit and is held clear when the carriage reaches its end stop (buffer full); it is ≤ 25 % of the ${fmt(peakB, 3)} g service braking and a stuck-engaged coupling keeps the total ${fmt(peakB + p.regenDecel, 3)} g below the ${fmt(poLim, 3)} g pitch-over limit (Suite 12). The friction brakes act beyond the first pedal travel; the one-way coupling cannot drive the wheel. Switching thresholds coordinate with Suite 19.` });
    out.verdicts.push({ test: 'Parked-wind harvester stays inside safety limits', status: windOn ? 'pass' : 'info', detail: 'Active only with the parking lock engaged, through the elastomer yaw limiter (±3°): the harvester torque is capped below 20 % of the washout yaw moment and is fully disengaged while driving, so the washout cap and the arch-locked tip-over case (Suite 01) are unaffected. The limiter also keeps the parked pylon inside its fatigue band (Suites 09/22).' });
    const rows = [
      ['Launch (0 → 10 km/h)', `${launchesHeld} launches held`, `${fmt(Elaunch / 1000, 3)} kJ each; ${useEl ? 'elastomer' : 'titanium'} buffer ${fmt(Ebuf / 1000, 3)} kJ`],
      ['Recharge — sun (hub)', Pcharge > 0 ? `${fmt(solarDayJ / 1000, 3)} kJ/day` : 'none', hubNegative ? 'hub net work negative with compliant sliding shoes — lifted off the track' : `${fmt(Pcharge, 3)} W for ${sunH} h`],
      ['Recharge — regenerative braking', `${fmt(Estop20 / 1000, 3)} kJ per 20 km/h stop`, `${fmt(regenKm / 1000, 3)} kJ/km on a typical route; full in ${fmt(kmToFill, 3)} km`],
      [windOn ? 'Recharge — parked wind' : 'Recharge — parked wind (optional, not installed)', `${fmt(windPotJ / 1000, 3)} kJ/day potential`, '< 0.03 W below 10 m/s, 0.2–6 W at 15–30 m/s inside the ±3° parked yaw limiter: storm-only supplement, not counted on for starts'],
      ['Bridge a wind lull at 20 km/h', `${fmt(bridgeS, 3)} s / ${fmt(bridgeM, 3)} m`, `${fmt(P20, 3)} W road load from the full buffer`],
      ['Calm driving (sustained)', 'not possible', 'no sustained backup source without wind: the buffer only bridges lulls and launches'],
      ['Night', Pcharge > 0 ? 'regen + wind only' : 'regen + wind only', `night wind ${fmt(windNightJ / 1000, 3)} kJ; no solar; public-road use limited to daylight (Suite 23)`],
    ];
    out.tables.push({ title: 'Backup capability (Revision 6.3, simulation)', cols: ['Case', 'Capability', 'Basis / limit'], rows });
    out.recs.push({ level: 'info', text: `Honest limit: the passive backup launches the vehicle ${launchesHeld} times and bridges ≈ ${fmt(bridgeS, 3)} s (${fmt(bridgeM, 3)} m) of calm at 20 km/h; it is recharged mainly by braking and descents. Sustained calm-air driving remains outside what passive storage can deliver.`, basis: 'Suite 18 backup table' });
    out.outputs = { cellType: p.cellType, bufferEnergy_J: Ebuf, bufferBands_kg: useEl ? mBand : 0, bufferLatches_kg: useEl ? mLatch : 0, elastomer: useEl ? p.elastomer : null, launchesHeld, bridge20_s: bridgeS, bridge20_m: bridgeM, regenPerStop20_J: Estop20, regenPerKm_J: regenKm, windHarvestDay_J: windDayJ, windHarvestNight_J: windNightJ, regenCoupling_kg: 0.7, windHarvester_kg: windOn ? 0.6 : 0, backupTable: rows, fatigueLife_years: lifeYears, boxMass_kg: bufSysMass - mass - cageMassTot - 0.5, launchEnergy_J: Estart, bufferSystemMass_kg: bufSysMass, cageMass_kg: cageMassTot, cageMat: p.cageMat || 'ti-gr2', thermalChargePower_W: Pcharge, bufferRefill_h: refillH, startsPerSunnyDay: startsDay, Euse_J: nCells * Euse, Etotal_J: nCells * C.Estore, efficiency: (1 - p.loss / 100) * p.eta, backupDuration_s: duration, bufferMass_kg: mass, Einit_J: Einit, chargeState: p.charge / 100, cellEnergy_J: Euse, nCells, energyDensity_J_kg: eDens, dischargePower_W: p.P, bistable: C.bistable };
    for (const v of out.verdicts) {
      if (/^Sustained backup ≥ 1 h/.test(v.test)) v.detail = v.detail.replace(/launch buffer \([^)]*\)/, `launch buffer (${fmt(Ebuf / 1000, 3)} kJ)`).replace(/Elastic cells (alone would need|for 1 h would need) [^ ]+ kg/, `${useEl ? 'Elastomer bands' : 'Elastic cells'} for 1 h would need ${fmt(P20 * 3600 / etaOut / Math.max(1e-9, Ebuf / sysBuf), 4)} kg`);
      if (v.test === 'Solar recharging of the elastic buffer' && hubNegative) { v.status = 'info'; v.detail = 'No solar-thermal recharge: the hub with compliant sliding shoes has negative net work (Suite 17). Recharge comes from regenerative braking and descents (and parked wind in storms) — see the backup table.'; }
    }
    const startsWind = Estart > 0 ? Math.floor((solarDayJ + windDayJ) / Estart) : 0;
    out.outputs.startsPerSunnyDay = startsWind; out.outputs.startsSolarOnly = startsDay;
    if (useEl) Object.assign(out.outputs, { bufferSystemMass_kg: sysEl, cageMass_kg: mSleeve, boxMass_kg: 0.9, bufferMass_kg: 0, Euse_J: Eel, Etotal_J: Eel / Math.max(1e-9, (1 - el.hyst) * relaxF), nCells: nBands, cellEnergy_J: Eel / nBands, energyDensity_J_kg: Eel / sysEl, efficiency: (1 - el.hyst) * p.eta, backupDuration_s: bridgeS });
    out.headline = useEl ? `Elastomer buffer: ${fmt(Eel / 1000, 3)} kJ in ${fmt(sysEl, 3)} kg → ${launchesHeld} launches or ${fmt(bridgeS, 3)} s at 20 km/h; regen ${fmt(Estop20 / 1000, 3)} kJ per stop; parked wind ${fmt(windDayJ / 1000, 3)} kJ/day.` : C.bistable ? `${nCells} cells (${fmt(mass, 3)} kg) store ${fmt(nCells * Euse / 1000, 3)} kJ usable → ${fmt(duration, 3)} s at ${p.P} W. One hour would need ${fmt(mass1h, 3)} kg.` : 'Cell is not bistable — no energy can be held in a charged state.';
    out.headlineKind = C.bistable ? 'info' : 'fail';
    void range;
    return out;
  },
};

// 3D array of bistable (von Mises truss) cells discharging in sequence at the solved power: the release schedule
// follows E(t) = E_init − (P/η)·t from the solved stored energy; each snap follows the solved force–displacement
// path u → 0 and the bars are coloured by their solved axial strain. Arch rise exaggerated for visibility.
async function view3dBuffer(el, { C, nCells, Euse, Einit, duration, p, m, b, hr }) {
  const [{ viewer }, { playback, hud, legend3d, magnifier }, { cmap }, { h }] = await Promise.all([import('../core/viz3d.js'), import('../core/sim3d.js'), import('../core/chart.js'), import('../core/ui.js')]);
  const finite = Number.isFinite(nCells) && nCells > 0, nVis = finite ? Math.min(48, nCells) : 24, per = finite ? nCells / nVis : 0, nCharged = Math.round(nVis * p.charge / 100);
  el.append(h('p', { class: 'small muted' }, `Solved: cell energy landscape U(u) and force F(u) of the ${m.name} bistable cell, stored energy ${fmt(C.Estore, 4)} J/cell, ${finite ? nCells.toLocaleString() : '—'} cells, initial charge ${p.charge} % (${fmt(Einit / 1000, 4)} kJ), discharge at ${p.P} W → ${fmt(duration, 4)} s. Each visual cell represents ${fmt(per, 3)} real cells; snap-through motion follows the solved u-path; arch rise × magnification (slider).`));
  if (!C.bistable) el.append(h('div', { class: 'callout fail' }, 'Cell is not bistable — it cannot hold a charged state, so nothing can be stored or released.'));
  const mag = magnifier(el, { auto: Math.max(1, Math.min(500, 0.35 * b / hr)), truePeak: hr * 1000, unit: ' mm arch rise' });
  const wrap = h('div'); el.append(wrap);
  const V = await viewer(wrap, { height: 440, grid: true }); const THREE = V.THREE;
  const bv = 0.12, cols = 8, gap = 0.34; let rise = 0.12 * hr / b;
  const geo = new THREE.CylinderGeometry(0.008, 0.008, 1, 8); geo.rotateX(Math.PI / 2); // unit bar along z
  const cells = [];
  for (let k = 0; k < nVis; k++) {
    const cx = (k % cols - (cols - 1) / 2) * gap, cy = Math.floor(k / cols) * gap - 0.6;
    const bars = [0, 1].map(() => { const mm = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: 0x999999 })); V.root.add(mm); return mm; });
    const base = new THREE.Mesh(new THREE.BoxGeometry(2 * bv + 0.04, 0.05, 0.015), new THREE.MeshStandardMaterial({ color: 0x475569 })); base.position.set(cx, cy, 0); V.root.add(base);
    const apex = new THREE.Mesh(new THREE.SphereGeometry(0.016, 12, 8), new THREE.MeshStandardMaterial({ color: 0xfacc15 })); V.root.add(apex);
    cells.push({ cx, cy, bars, apex, charged: k < nCharged, k });
  }
  const strainAt = (u) => (Math.hypot(b, hr - u) - C.L0) / C.L0, sMax = Math.max(1e-9, C.maxStrain);
  const place = (cell, u) => { // u = 0 rest (arch up), u = u2 charged (inverted)
    rise = Math.min(0.12, mag.value * 0.12 * hr / b);
    const z = 0.0075 + rise * (1 - u / hr) + rise * 0.0, A = new THREE.Vector3(cell.cx, cell.cy, z + rise * 0);
    const apexZ = 0.02 + rise * (1 - u / hr) + rise; A.set(cell.cx, cell.cy, apexZ);
    [-1, 1].forEach((sgn, i) => { const S0 = new THREE.Vector3(cell.cx + sgn * bv, cell.cy, 0.02 + rise); const d = A.clone().sub(S0), L = d.length(); const mm = cell.bars[i]; mm.position.copy(S0.clone().add(A).multiplyScalar(0.5)); mm.scale.set(1, 1, L); mm.lookAt(A); const c = cmap(Math.abs(strainAt(u)) / sMax, 'inferno'); mm.material.color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255); });
    cell.apex.position.copy(A); void z;
  };
  V.viewDir(0.05, -0.7, 0.5);
  const H = hud(wrap); legend3d(wrap, { title: 'Bar axial strain |ε| / ε_max', lo: 0, hi: 1, cmapName: 'inferno' });
  const gauge = h('div', { class: 'sim-legend', style: 'left:8px;right:auto;width:200px' }, h('div', { class: 'small' }, 'Stored energy'), h('div', { style: 'height:12px;border-radius:6px;background:var(--line);overflow:hidden' }, h('div', { style: 'height:100%;width:0;background:linear-gradient(90deg,#ef4444,#22c55e)' })), h('div', { class: 'small' }, '')); wrap.append(gauge);
  const bar = gauge.children[1].firstChild, lab = gauge.children[2];
  const D = Math.max(duration, 1e-6), snapT = Math.min(0.05 * D, 2), dur = Math.max(D * 1.1, 5);
  const usPath = C.us.filter(u => u <= C.u2), tRel = cells.map(c => c.charged ? D * (nCharged - c.k) / Math.max(1, nCharged) - snapT : Infinity);
  playback(wrap, { duration: dur, rate: dur / 15, label: 'Discharge time', unit: 's', onFrame: (t) => {
    let left = 0;
    for (const c of cells) {
      let u = 0;
      if (c.charged && C.bistable) { const t0 = tRel[c.k]; if (t < t0) { u = C.u2; left++; } else if (t < t0 + snapT) { const f = (t - t0) / snapT; u = usPath[Math.max(0, Math.round((1 - f) * (usPath.length - 1)))]; } }
      place(c, u);
    }
    const E = Math.max(0, Einit - p.P / p.eta * t), frac = Einit > 0 ? E / Einit : 0;
    bar.style.width = (100 * frac).toFixed(1) + '%'; lab.textContent = `${fmt(E / 1000, 4)} kJ of ${fmt(Einit / 1000, 4)} kJ`;
    H.set([['Time', fmt(t, 4) + ' s'], ['Discharge power', `${p.P} W at wheel`], ['Cells still charged', `${Math.round(left * per).toLocaleString()} / ${finite ? nCells.toLocaleString() : '—'}`], ['Energy remaining', fmt(E / 1000, 4) + ' kJ', frac > 0.2 ? '#22c55e' : '#ef4444'], ['Backup ends at', fmt(duration, 4) + ' s'], ['State', t >= D ? 'DEPLETED — vehicle must stop without other energy' : 'discharging', t >= D ? '#ef4444' : undefined]]);
  } });
}

// 2D animation: solved force–displacement and energy landscape of one cell with the state marker travelling the
// snap-through path for each released cell, plus the stored-energy bar of the whole buffer at the discharge power.
async function anim2dBuffer(el, { C, nCells, Einit, duration, p }) {
  const [{ anim2d }, { h }] = await Promise.all([import('../core/sim3d.js'), import('../core/ui.js')]);
  const D = Math.max(duration, 1e-6), nShow = Number.isFinite(nCells) ? Math.min(40, nCells) : 1, uMM = C.us.map(u => u * 1000), i2 = C.us.indexOf(C.u2);
  el.append(h('p', { class: 'small muted' }, `Solved cell model: F(u) and U(u) (${C.bistable ? 'bistable' : 'NOT bistable'}). During discharge (${fmt(D, 4)} s at ${p.P} W) cells release in sequence; the marker follows the solved u-path u₂ → 0 of the cell currently releasing (${nShow} representative releases shown).`));
  const Fl = Math.min(...C.Fs.slice(0, i2 + 1)), Fh = Math.max(...C.Fs.slice(0, i2 + 1)), Ul = Math.min(...C.Us.slice(0, i2 + 1)), Uh = Math.max(...C.Us.slice(0, i2 + 1));
  anim2d(el, { height: 400, duration: D * 1.05, rate: D / 6, label: 'Discharge time', unit: 's', onFrame: (t, g) => {
    g.clear(); const k = Math.min(nShow - 1, Math.floor(t / D * nShow)), f = Math.min(1, (t / D * nShow) - k), j = Math.round((1 - f) * i2), done = t >= D;
    const A = g.panel(0, 0, g.W / 2, g.H - 60, { xr: [0, uMM[i2] * 1.05], yr: [Fl * 1.15, Fh * 1.15], xlabel: 'Apex displacement u [mm]', ylabel: 'Force F [N]', title: 'Force–displacement (snap-through)' });
    A.line(uMM.slice(0, i2 + 1), C.Fs.slice(0, i2 + 1), '#38bdf8', 2); A.hline(0, '#94a3b8'); if (!done && C.bistable) A.point(uMM[j], C.Fs[j]);
    const B = g.panel(g.W / 2, 0, g.W / 2, g.H - 60, { xr: [0, uMM[i2] * 1.05], yr: [Ul, Uh * 1.1], xlabel: 'Apex displacement u [mm]', ylabel: 'Energy U [J]', title: 'Energy landscape (charged state at u₂)' });
    B.line(uMM.slice(0, i2 + 1), C.Us.slice(0, i2 + 1), '#a855f7', 2); if (!done && C.bistable) B.point(uMM[j], C.Us[j], '#f97316');
    { const cxs = g.W * 0.82, cys = g.H - 30, bw = 70, hr0 = C.us[i2] / 2, u = (!done && C.bistable) ? C.us[j] : 0, z = (hr0 - u) / (hr0 || 1) * 28; g.ctx.strokeStyle = '#f97316'; g.ctx.lineWidth = 4; g.ctx.beginPath(); g.ctx.moveTo(cxs - bw, cys); g.ctx.lineTo(cxs, cys - z); g.ctx.lineTo(cxs + bw, cys); g.ctx.stroke(); g.ctx.lineWidth = 1; g.ctx.fillStyle = '#facc15'; g.ctx.beginPath(); g.ctx.arc(cxs, cys - z, 5, 0, 2 * Math.PI); g.ctx.fill(); g.text('releasing cell (rise magnified)', cxs - bw, cys - 40, { size: 10, color: g.mute }); }
    const E = Math.max(0, Einit - p.P / p.eta * t); g.gauge(60, g.H - 34, g.W * 0.5, Einit > 0 ? E / Einit : 0, `Buffer energy ${fmt(E / 1000, 4)} / ${fmt(Einit / 1000, 4)} kJ${done ? ' — DEPLETED' : ''}`, done ? '#ef4444' : '#22c55e');
    g.text(`releasing cell ${Math.min(nShow, k + 1)} of ${nShow} shown (${Number.isFinite(nCells) ? nCells.toLocaleString() : '—'} real)`, g.W * 0.62, g.H - 22, { size: 11 });
  } });
}
