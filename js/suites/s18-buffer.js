// Suite 18 — Bistable metamaterial energy storage and elastic buffering.
import * as TH from '../core/thermo.js';
import { linspace, fmt, trapz, range, logspace } from '../core/num.js';
import { forUse, get } from '../core/materials.js';

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
  formats: { import: ['CSV force–displacement cycles', 'XLSX', 'JSON'], export: ['CSV energy landscape & sizing', 'STEP/3MF/STL cell geometry (Suite 02)', 'Abaqus INP (Suite 02)', 'PNG/SVG'] },
  params: [
    { key: 'mat', label: 'Cell material', type: 'select', group: 'Cell', default: 'ti-beta21s', options: forUse('buffer').map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.store.material('buffer', 'ti-beta21s'), prov: 'library' }) },
    { key: 'b', label: 'Half-span b', unit: 'mm', type: 'number', min: 5, max: 300, step: 1, group: 'Cell', default: 50 },
    { key: 'hr', label: 'Initial rise h', unit: 'mm', type: 'number', min: 0.5, max: 50, step: 0.1, group: 'Cell', default: 5 },
    { key: 'd', label: 'Bar diameter', unit: 'mm', type: 'number', min: 0.5, max: 30, step: 0.1, group: 'Cell', default: 6 },
    { key: 'ks', label: 'Parallel stiffness ratio k_s/k*', type: 'range', min: 0, max: 0.3, step: 0.005, group: 'Cell', default: 0.15, help: 'Raises the energy of the charged state; bistability is lost above 0.25.' },
    { key: 'loss', label: 'Snap-through dissipation', unit: '%', type: 'range', min: 0, max: 60, step: 1, group: 'Losses', default: 12 },
    { key: 'eta', label: 'Transmission efficiency to wheel', type: 'range', min: 0.3, max: 1, step: 0.01, group: 'Losses', default: 0.85 },
    { key: 'frame', label: 'Frame / housing mass factor', type: 'range', min: 1, max: 4, step: 0.1, group: 'Cell', default: 1.8, expert: true },
    { key: 'SF', label: 'Strain safety factor', type: 'number', min: 1, max: 4, step: 0.1, group: 'Cell', default: 1.5 },
    { key: 'Etarget', label: 'Target usable energy', unit: 'kJ', type: 'number', min: 0.1, max: 2000, step: 0.1, group: 'Sizing', default: 30 },
    { key: 'P', label: 'Discharge power at wheel', unit: 'W', type: 'number', min: 1, max: 2000, step: 1, group: 'Sizing', default: 150, auto: (c) => { const o = c.upstream('s17'); return o?.deficit_W > 0 ? { value: Math.round(o.deficit_W), prov: 'coupled' } : undefined; } },
    { key: 'charge', label: 'Initial state of charge (explicit initial condition)', unit: '%', type: 'range', min: 0, max: 100, step: 1, group: 'Sizing', default: 100, help: 'Must be justified by a physical charging process or declared as a new initial condition.' },
    { key: 'cyclesDay', label: 'Charge–discharge cycles per day', type: 'number', min: 0.1, max: 200, step: 0.1, group: 'Durability', default: 4 },
    { key: 'shockG', label: 'Design road/gust shock', unit: 'g', type: 'number', min: 0.5, max: 30, step: 0.5, group: 'Durability', default: 5 },
  ],
  presets: [{ label: 'One hour at 150 W', values: { Etarget: 640 } }, { label: 'Spring-steel cells', values: { mat: 'st-spring' } }, { label: 'Monostable (k_s = 0.3)', values: { ks: 0.3 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const m = get(p.mat), E = m.E * 1e9, d = p.d / 1000, A = Math.PI * d * d / 4, I = Math.PI * d ** 4 / 64, b = p.b / 1000, hr = p.hr / 1000;
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
    K('Bistable', C.bistable ? 'yes' : 'NO', '', C.bistable ? `Second stable state at u₂ = ${fmt(C.u2 * 1000, 3)} mm.` : 'The charged state is not stable — the cell would release immediately (monostable).', 'calc', C.bistable ? 'ok' : 'bad');
    K('Stored energy per cell', C.Estore, 'J', 'U(u₂) − U(0): elastic energy held in the charged stable state.', 'calc');
    K('Usable energy per cell', Euse, 'J', `After ${p.loss} % snap-through dissipation.`, 'calc');
    K('Release barrier (trigger energy)', C.barrierRelease, 'J', 'Energy needed to push a charged cell back over the barrier — protects against accidental release.', 'calc', C.barrierRelease > 0.05 * C.Estore ? 'ok' : 'warn');
    K('Energy density', eDens, 'J/kg', 'Usable energy per kg of cell including frame. Limited by σ_y²/2E of the material.', 'calc');
    const resil = (m.sy ** 2 / (2 * m.E)) * 1e3 / m.rho; K('Fraction of material resilience limit', eDens / resil * 100, '%', `Theoretical ceiling σ_y²/(2Eρ) = ${fmt(resil, 4)} J/kg for ${m.name}; geometry, frame mass and strain safety factor reduce it.`, 'calc');
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
    out.verdicts.push({ test: 'Practical sustained backup (≥ 1 h at discharge power within 10 % vehicle mass)', status: mass1h <= 0.1 * ctx.vehicle.mass ? 'pass' : 'fail', detail: `Needs ${fmt(mass1h, 4)} kg of cells for 1 h — elastic storage suits short gaps, start-up and torque smoothing only.` });
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
    out.outputs = { Euse_J: nCells * Euse, Etotal_J: nCells * C.Estore, efficiency: (1 - p.loss / 100) * p.eta, backupDuration_s: duration, bufferMass_kg: mass, Einit_J: Einit, chargeState: p.charge / 100, cellEnergy_J: Euse, nCells, energyDensity_J_kg: eDens, dischargePower_W: p.P, bistable: C.bistable };
    out.headline = C.bistable ? `${nCells} cells (${fmt(mass, 3)} kg) store ${fmt(nCells * Euse / 1000, 3)} kJ usable → ${fmt(duration, 3)} s at ${p.P} W. One hour would need ${fmt(mass1h, 3)} kg.` : 'Cell is not bistable — no energy can be held in a charged state.';
    out.headlineKind = C.bistable ? 'info' : 'fail';
    void range;
    return out;
  },
};
