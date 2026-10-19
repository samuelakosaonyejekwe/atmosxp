// Suite 01 — Complete automobile architecture & system-level engineering.
import { G0, fmt, range, sum } from '../core/num.js';
import { roadLoad, kmh } from '../core/physics.js';
import { wheelLayout, threeWheelLoads, trikeSSF, rolloverAy, tipOverWind } from '../core/vehicle.js';
import { vehicleAssembly } from '../core/cad.js';
import { indices } from '../core/materials.js';
import { DEPENDS } from '../core/state.js';
import { SUITES } from './index.js';
import { h } from '../core/ui.js';

const V = (k, d) => (c) => ({ value: c.vehicle[k] ?? d, prov: 'user' });

export default {
  description: 'Defines the whole-vehicle architecture: dimensions, three-wheel layout, a component-level mass budget built from the materials registry, centre of gravity, inertia tensor, static and dynamic wheel loads, stability, design-rule and requirements compliance (no battery, no fuel, no electric traction, no engine) and the design-structure matrix that couples all 26 suites. Edits here become the global vehicle used by every suite.',
  literature: 'three-wheeled vehicle stability lightweight vehicle architecture mass budget land yacht',
  credibility: 'Analytical screening', sustain: true,
  equations: ['Newton–Euler rigid-body equations; longitudinal, lateral and yaw equilibrium', 'Mass and volume integration, centre-of-mass equations, inertia tensor with parallel-axis (Huygens–Steiner) theorem including products of inertia', 'Statically determinate three-support normal-load equilibrium (ΣF_z, ΣM_x, ΣM_y) with d’Alembert inertial and aerodynamic loads', 'Trike static stability factor t·b/(2hL) (tadpole) or t·a/(2hL) (delta): tipping about the line through outer front and rear contacts', 'Road-load and propulsion power balance P = (m g C_rr + ½ ρ C_dA v²) v', 'Design-structure matrix (DSM) and requirements-traceability graph; geometric feasibility and design-rule checks'],
  methods: ['Component masses from geometry × constituent density (sandwich skins + core, box spar, ribs, lattice relative density, rims, cartridges, discs, glazing)', 'Energy-buffer mass from target usable energy ÷ specific resilience σy²/(2Eρ) ÷ packaging efficiency', 'Bisection for wheel-lift lateral acceleration and tip-over crosswind speed'],
  assumptions: ['Point-mass + box self-inertia representation of each subsystem', 'Rigid chassis and rigid tyres for load distribution (compliance in Suites 09/10)', 'Default positions follow the parametric assembly of Suite 02'],
  ibc: { initial: ['Initial configuration: wheel number and placement, CG, component masses, inertia, position, heading, speed, steering, brake, wing configuration, payload, buffer state, propulsion mode'], boundary: ['Road support (three contacts), gravity, dimensional limits, payload, permitted wheel-contact locations, wing-to-chassis mounting, wind-load envelope, steering travel and braking requirements', 'Hard requirements: no fossil fuel, no battery, no electric traction, no conventional engine; disclosure of internal moving parts'] },
  cvv: { calibration: ['Physical component dimensions and weighed masses', 'Measured CG (axle-load scales, tilt test) and inertia (pendulum / bifilar tests)'], verification: ['Mass integration equals sum of components', 'Normal loads sum to weight and satisfy moment balance', 'SSF formula versus numerical wheel-lift bisection'], validation: ['Corner-weight measurements of the built prototype', 'Tilt-table rollover threshold test'] },
  formats: { import: ['CSV/XLSX mass lists', 'JSON project files'], export: ['CSV mass budget', 'JSON architecture', 'PNG/SVG plots', 'glTF/GLB assembly (3D view)'] },
  params: [
    { key: 'layout', label: 'Wheel layout', type: 'select', group: 'Architecture', default: 'tadpole', options: [{ value: 'tadpole', label: 'Tadpole (2 front, 1 rear)' }, { value: 'delta', label: 'Delta (1 front, 2 rear)' }], auto: V('layout', 'tadpole') },
    { key: 'wingConfig', label: 'Wing configuration', type: 'select', group: 'Architecture', default: 'strict', options: [{ value: 'strict', label: 'Strict: fixed root, passive morphing only' }, { value: 'practical', label: 'Practical: adjustable root incidence + unloading' }] },
    { key: 'length', label: 'Overall length', unit: 'm', type: 'number', min: 2.5, max: 6, step: 0.05, group: 'Dimensions', default: 3.6, auto: V('length') },
    { key: 'width', label: 'Body width', unit: 'm', type: 'number', min: 1, max: 2.55, step: 0.05, group: 'Dimensions', default: 1.6, auto: V('width') },
    { key: 'height', label: 'Body height (excl. wings)', unit: 'm', type: 'number', min: 0.7, max: 1.8, step: 0.05, group: 'Dimensions', default: 1.1, auto: V('height') },
    { key: 'wheelbase', label: 'Wheelbase', unit: 'm', type: 'number', min: 1.5, max: 4, step: 0.05, group: 'Dimensions', default: 2.4, auto: V('wheelbase') },
    { key: 'track', label: 'Track (twin-wheel axle)', unit: 'm', type: 'number', min: 0.8, max: 2.4, step: 0.05, group: 'Dimensions', default: 1.5, auto: V('track') },
    { key: 'wheelRadius', label: 'Wheel effective radius', unit: 'm', type: 'number', min: 0.2, max: 0.45, step: 0.01, group: 'Dimensions', default: 0.3, auto: V('wheelRadius') },
    { key: 'wingArea', label: 'Combined wing reference area', unit: 'm²', type: 'number', min: 1, max: 20, step: 0.1, group: 'Wings', default: 8, auto: V('wingArea') },
    { key: 'wingHeight', label: 'Wing height (span)', unit: 'm', type: 'number', min: 1, max: 6, step: 0.1, group: 'Wings', default: 3, auto: V('wingHeight') },
    { key: 'nWings', label: 'Number of wings', type: 'select', group: 'Wings', default: 2, options: [1, 2], auto: V('nWings') },
    { key: 'massBasis', label: 'Operating-mass basis', type: 'select', group: 'Mass', default: 'target', options: [{ value: 'target', label: 'Design target mass (below)' }, { value: 'computed', label: 'Computed mass budget' }], help: 'Which mass becomes the global vehicle mass for every suite.' },
    { key: 'targetMass', label: 'Design target operating mass', unit: 'kg', type: 'number', min: 120, max: 800, step: 5, group: 'Mass', default: 300, auto: V('mass') },
    { key: 'payload', label: 'Occupant + payload', unit: 'kg', type: 'number', min: 0, max: 250, step: 5, group: 'Mass', default: 90, auto: V('payload') },
    { key: 'skin', label: 'Monocoque skin thickness (each face)', unit: 'mm', type: 'number', min: 0.5, max: 4, step: 0.1, group: 'Construction', default: 1.2 },
    { key: 'core', label: 'Sandwich core thickness', unit: 'mm', type: 'number', min: 5, max: 40, step: 1, group: 'Construction', default: 15 },
    { key: 'cartPerWheel', label: 'NiTi cartridges per driven wheel', type: 'number', min: 0, max: 64, step: 1, group: 'Propulsion hardware', default: 24 },
    { key: 'cartMass', label: 'Active NiTi mass per cartridge', unit: 'kg', type: 'number', min: 0.005, max: 0.5, step: 0.005, group: 'Propulsion hardware', default: 0.06 },
    { key: 'driveWheels', label: 'Driven (hub-drive) wheels', type: 'select', group: 'Propulsion hardware', default: 2, options: [0, 1, 2, 3], auto: V('driveWheels') },
    { key: 'bufferE', label: 'Elastic-buffer usable energy target', unit: 'kJ', type: 'number', min: 0, max: 600, step: 5, group: 'Propulsion hardware', default: 30 },
    { key: 'Crr', label: 'Rolling-resistance coefficient', type: 'number', min: 0.002, max: 0.03, step: 0.0005, group: 'Road load', default: 0.006, auto: V('Crr') },
    { key: 'CdA', label: 'Body drag area C_dA', unit: 'm²', type: 'number', min: 0.08, max: 1, step: 0.01, group: 'Road load', default: 0.25, auto: V('CdA') },
    { key: 'designGust', label: 'Design gust for tip-over check', unit: 'm/s', type: 'number', min: 5, max: 40, step: 0.5, group: 'Road load', default: 20 },
  ],
  presets: [{ label: 'Baseline tadpole 300 kg', values: { layout: 'tadpole', targetMass: 300, track: 1.5, wheelbase: 2.4 } }, { label: 'Delta layout', values: { layout: 'delta' } }, { label: 'Wide-track stability', values: { track: 1.9, height: 1.0 } }, { label: 'Single large wing', values: { nWings: 1, wingArea: 8, wingHeight: 3.6 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const veh0 = { ...ctx.vehicle, layout: p.layout, length: p.length, width: p.width, height: p.height, wheelbase: p.wheelbase, track: p.track, wheelRadius: p.wheelRadius, wingArea: p.wingArea, wingHeight: p.wingHeight, nWings: +p.nWings };
    const M = (k) => ctx.material(k);
    const asm = vehicleAssembly(veh0, {}); const bodyArea = asm.parts[0].area(), meta = asm.meta, R = p.wheelRadius, Hb = p.height;
    // ---------- component mass budget ----------
    const items = [];
    const add = (name, comp, mass, x, y, z, dims = [0.2, 0.2, 0.2], note = '') => items.push({ name, comp, mat: comp ? M(comp)?.name : '—', m: mass, x, y, z, dims, note });
    const ch = M('chassis'), core = M('chassisCore');
    add('Monocoque sandwich shell', 'chassis', bodyArea * (2 * p.skin / 1000 * ch.rho + p.core / 1000 * core.rho) * 1.12, 0.08 * p.length, 0, R + Hb * 0.36, [p.length, p.width * 0.8, Hb * 0.7], `${fmt(bodyArea, 3)} m² surface; +12 % joints, resin & inserts`);
    const nW = +p.nWings, S1 = p.wingArea / nW, span = p.wingHeight, cMean = S1 / span;
    for (let k = 0; k < nW; k++) {
      const wx = meta.wingX[k], zc = R + Hb * 0.7 + span * 0.45;
      const skinA = 2.05 * S1, sk = M('wingSkin'), sp = M('spar'), rb = M('rib'), lt = M('lattice');
      add(`Wing ${k + 1} morphing skin`, 'wingSkin', skinA * 0.001 * sk.rho, wx, 0, zc, [cMean, 0.15 * cMean, span]);
      add(`Wing ${k + 1} box spar`, 'spar', span * 2 * (0.12 + 0.1) * cMean * 0.003 * sp.rho * 1.15, wx, 0, R + Hb * 0.7 + span * 0.4, [0.12 * cMean, 0.1 * cMean, span]);
      add(`Wing ${k + 1} flexible ribs (×8, 3 mm, 50 % lightening)`, 'rib', 8 * 0.1 * cMean * cMean * 0.003 * 0.5 * rb.rho, wx, 0, zc, [cMean, 0.1, span]);
      add(`Wing ${k + 1} metamaterial core`, 'lattice', S1 * 0.085 * cMean * 0.3 * 0.03 * lt.rho, wx, 0, zc, [cMean * 0.6, 0.1 * cMean, span], 'morphing regions = 30 % of wing volume, relative density 3 %');
      add(`Wing ${k + 1} root fitting`, 'rootInsert', 2.2 * (M('rootInsert').rho / 4430), wx, 0, R + Hb * 0.66, [0.2, 0.2, 0.2]);
    }
    meta.wheelPos.forEach(([x, y], k) => {
      const wm = M('wheel'), ty = M('tyre');
      add(`Wheel ${k + 1} rim + spokes`, 'wheel', (2 * Math.PI * (R - 0.075) * 0.05 * 0.003 + 6 * 0.5 * R * 0.012 * 0.03) * wm.rho, x, y, R, [2 * R, 0.05, 2 * R]);
      add(`Wheel ${k + 1} tyre`, 'tyre', 2 * Math.PI * (R - 0.035) * 2 * Math.PI * 0.035 * 0.0035 * ty.rho * 1.6, x, y, R, [2 * R, 0.07, 2 * R]);
      add(`Wheel ${k + 1} brake disc`, 'brake', Math.PI * ((R * 0.55) ** 2 - (R * 0.3) ** 2) * 0.005 * M('brake').rho, x, y, R, [R, 0.01, R]);
      add(`Wheel ${k + 1} bearings & axle`, 'bearing', 0.9, x, y, R, [0.1, 0.1, 0.1]);
    });
    const driven = meta.wheelPos.slice(0, +p.driveWheels);
    driven.forEach(([x, y], k) => {
      add(`Hub ${k + 1} NiTi cartridges (×${p.cartPerWheel})`, 'actuator', p.cartPerWheel * p.cartMass, x, y, R, [0.25, 0.06, 0.25]);
      add(`Hub ${k + 1} housings, flexures & containment`, 'cartridgeHousing', p.cartPerWheel * p.cartMass * 1.2 + 0.6, x, y, R, [0.27, 0.07, 0.27]);
      add(`Hub ${k + 1} stationary cam & thermal sectors`, 'cam', 1.2 + 0.8, x, y, R, [0.2, 0.06, 0.2]);
    });
    const buf = M('buffer'), sres = indices.specificResilience(buf); const bufMass = p.bufferE > 0 ? p.bufferE * 1000 / (sres * 0.3) : 0;
    add('Bistable elastic buffer', 'buffer', bufMass, -0.1 * p.wheelbase, 0, R + 0.15, [0.6, 0.4, 0.2], `${fmt(sres, 3)} J/kg material resilience × 30 % packaging`);
    add('Solar-selective collector deck', 'collector', (p.length * 0.32) * (p.width * 0.5) * 0.0012 * 2700, -p.length * 0.2, 0, R + Hb * 0.67, [p.length * 0.32, p.width * 0.5, 0.01]);
    add('Radiative-cooling panels (2)', 'radiator', 2 * (p.length * 0.28) * (Hb * 0.22) * 0.0012 * 2700, -p.length * 0.18, 0, R + Hb * 0.35, [p.length * 0.28, p.width, Hb * 0.22]);
    add('Heat spreaders, heat pipes, insulation', 'spreader', 4.5, -p.length * 0.1, 0, R + Hb * 0.5, [0.8, 0.6, 0.3]);
    add('Steering system (mechanical)', 'steering', 6.5, meta.wheelPos[0][0], 0, R + 0.1, [0.5, p.track * 0.8, 0.2]);
    add('Cabin canopy / glazing', 'cabin', 1.2 * 0.004 * M('cabin').rho, 0.35, 0, R + Hb * 0.62, [1.1, p.width * 0.5, 0.5]);
    add('Seat, restraints & controls', null, 9, 0.15, 0, R + 0.25, [0.6, 0.5, 0.6]);
    add('Wing unloading / incidence mechanism', null, p.wingConfig === 'practical' ? 7 : 2.5, meta.wingX[0], 0, R + Hb * 0.62, [0.3, 0.3, 0.2]);
    const struct = sum(items.map(i => i.m)); add('Fasteners, sealing & miscellaneous (6 %)', null, 0.06 * struct, 0, 0, R + Hb * 0.4, [p.length, p.width, Hb]);
    const empty = sum(items.map(i => i.m));
    add('Occupant + payload', null, p.payload, 0.1, 0, R + 0.45, [0.5, 0.45, 0.9]);
    const total = sum(items.map(i => i.m)), mOp = p.massBasis === 'computed' ? total : p.targetMass;
    // ---------- CG and inertia (about CG) ----------
    const cg = [0, 1, 2].map(d => sum(items.map(i => i.m * [i.x, i.y, i.z][d])) / total);
    let Ixx = 0, Iyy = 0, Izz = 0, Ixy = 0, Ixz = 0, Iyz = 0;
    for (const i of items) { const dx = i.x - cg[0], dy = i.y - cg[1], dz = i.z - cg[2], [a, b, c] = i.dims, m = i.m; Ixx += m * (dy * dy + dz * dz) + m * (b * b + c * c) / 12; Iyy += m * (dx * dx + dz * dz) + m * (a * a + c * c) / 12; Izz += m * (dx * dx + dy * dy) + m * (a * a + b * b) / 12; Ixy -= m * dx * dy; Ixz -= m * dx * dz; Iyz -= m * dy * dz; }
    const scale = mOp / total; // inertia scaled consistently when the target mass is adopted
    const front = meta.wheelPos.filter(w => w[0] > 0)[0][0], rear = meta.wheelPos.filter(w => w[0] < 0)[0][0];
    const frontFrac = (cg[0] - rear) / (front - rear), cgHeight = cg[2];
    const veh = { ...veh0, mass: mOp, cgHeight, payload: p.payload, Crr: p.Crr, CdA: p.CdA, driveWheels: +p.driveWheels };
    const lay = wheelLayout(veh, frontFrac), W = mOp * G0, loads = threeWheelLoads(lay.pts, W);
    const ssf = trikeSSF(veh, frontFrac), ayLift = rolloverAy(veh, mOp, { frontFrac }), hs = R + Hb * 0.7 + span * 0.45;
    const tipU = tipOverWind(veh, mOp, { S: p.wingArea, Cs: 1.2, hs, frontFrac });
    const P20 = roadLoad(kmh(20), veh).P, P40 = roadLoad(kmh(40), veh).P;
    // ---------- requirements compliance ----------
    const reqs = [
      ['No fossil-fuel propulsion', 'pass', 'No combustion system or fuel store in the bill of materials.'],
      ['No battery', 'pass', 'No electrochemical storage; finite mechanical buffer only (bistable cells).'],
      ['No operational electric traction', 'pass', 'Wheel torque only from wind (via tyres) or the thermomechanical hub.'],
      ['No conventional engine, turbine, gearbox or flywheel', 'pass', 'Direct aerodynamic force + wheel-integrated actuators.'],
      ['Only wheels, steering and brakes move', 'warn', `Disclosed internal motion: ${+p.driveWheels * p.cartPerWheel} NiTi cartridges cycle and contact a stationary cam; buffer trigger; passive switch flexures${p.wingConfig === 'practical' ? '; adjustable wing-root incidence & unloading mechanism' : ''}.`],
      ['Metamaterials essential to function', 'pass', 'Wing morphing core, photonic collectors, thermal metamaterials, NiTi lattice, bistable buffer.'],
      ['Completely noiseless', 'warn', 'Not achievable: tyre–road and airflow noise remain (Suite 21).'],
      ['Robust automatic backup in calm air', 'fail', 'Not demonstrated — must be proven by Suites 15–20 and physical Gates 2–4.'],
    ];
    // ---------- dimension / design-rule checks ----------
    const overallH = R + Hb * 0.7 + span, dims = [
      ['Overall width ≤ 2.55 m (common road-vehicle limit)', p.width, 2.55, p.width <= 2.55],
      ['Track ≤ body width + 0.2 m', p.track, p.width + 0.2, p.track <= p.width + 0.2],
      ['Overall height with wings ≤ 4.0 m', overallH, 4.0, overallH <= 4.0],
      ['Wheelbase ≤ 0.85 × length', p.wheelbase, 0.85 * p.length, p.wheelbase <= 0.85 * p.length],
      ['Wing root chord fits body length (twin wings)', meta.cr * nW, p.length * 0.95, meta.cr * nW <= p.length * 0.95],
      ['Ground clearance under body ≥ 0.12 m', R - 0.02 - 0.036 * Hb, 0.12, R - 0.02 - 0.036 * Hb >= 0.12],
    ];
    // ---------- checks ----------
    out.checks.push({ name: 'Σ normal loads = weight', value: sum(loads), target: fmt(W, 5) + ' N', pass: Math.abs(sum(loads) - W) < 1e-6 * W });
    out.checks.push({ name: 'Moment balance Σ x_i F_i (static)', value: sum(loads.map((F, i) => F * lay.pts[i][0])), target: '0 N·m', pass: Math.abs(sum(loads.map((F, i) => F * lay.pts[i][0]))) < 1e-6 * W });
    out.checks.push({ name: 'SSF closed form vs numerical wheel-lift', value: ssf, target: fmt(ayLift, 4) + ' g (bisection)', pass: Math.abs(ssf - ayLift) < 0.01 * Math.max(ssf, 0.1) });
    out.checks.push({ name: 'Inertia tensor positive definite (triangle inequality)', value: Ixx + Iyy - Izz, target: '> 0', pass: Ixx + Iyy > Izz && Iyy + Izz > Ixx && Ixx + Izz > Iyy });
    // ---------- KPIs ----------
    const k = (label, value, unit, explain, prov = 'calc', status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('Computed mass (incl. payload)', total, 'kg', 'Sum of every component in the budget (constituent density × geometry) plus occupant/payload.', 'calc', total <= p.targetMass ? 'ok' : total <= p.targetMass * 1.1 ? 'warn' : 'bad');
    k('Empty mass', empty, 'kg', 'Vehicle without occupant/payload.');
    k('Operating mass used globally', mOp, 'kg', p.massBasis === 'computed' ? 'Computed budget adopted as the global vehicle mass.' : 'Design target adopted; the budget shows whether it is achievable.', p.massBasis === 'computed' ? 'calc' : 'user');
    k('CG ahead of rear axle', cg[0] - rear, 'm', 'Longitudinal CG position; sets static axle loads.');
    k('CG height', cgHeight, 'm', 'Height of the combined CG above ground — tall wings raise it and reduce rollover resistance.', 'calc', cgHeight < 0.6 ? 'ok' : 'warn');
    k('Front-axle load fraction', frontFrac * 100, '%', 'Share of weight on the front axle.');
    k('I_xx (roll)', Ixx * scale, 'kg·m²', 'Roll inertia about the CG (parallel-axis sum).');
    k('I_yy (pitch)', Iyy * scale, 'kg·m²', 'Pitch inertia about the CG.');
    k('I_zz (yaw)', Izz * scale, 'kg·m²', 'Yaw inertia about the CG — governs yaw response to gusts and steering (Suite 11).');
    k('Static stability factor', ssf, 'g', 'Lateral acceleration at which an inside wheel lifts (trike tipping axis).', 'calc', ssf >= 1 ? 'ok' : ssf >= 0.75 ? 'warn' : 'bad');
    k('Tip-over crosswind (stationary)', tipU, 'm/s', 'Crosswind speed whose wing side force (C_side 1.2) alone lifts the windward wheel(s).', 'calc', tipU >= p.designGust ? 'ok' : 'bad');
    k('Wheel power at 20 km/h', P20, 'W', 'Ideal level-road wheel power; specification reference 124 W for the 300 kg baseline.');
    k('Wheel power at 40 km/h', P40, 'W', 'Specification reference 406 W.');
    k('Wing loading', mOp / p.wingArea, 'kg/m²', 'Mass per square metre of sail: lower is better for light-wind acceleration.');
    // ---------- verdicts ----------
    out.verdicts.push({ test: 'Mass budget vs target', status: total <= p.targetMass ? 'pass' : total <= 1.1 * p.targetMass ? 'warn' : 'fail', detail: `Computed ${fmt(total, 4)} kg vs target ${p.targetMass} kg.`, fix: 'Reduce sandwich skin/core thickness, switch heavy parts to lighter materials (Suite 03) or reduce buffer energy.' });
    out.verdicts.push({ test: 'Static stability (trike)', status: ssf >= 1 ? 'pass' : ssf >= 0.75 ? 'warn' : 'fail', detail: `SSF ${fmt(ssf, 3)} g with CG height ${fmt(cgHeight, 3)} m and track ${p.track} m (${p.layout}).`, fix: 'Widen the track, lower the CG, move mass toward the twin-wheel axle.' });
    out.verdicts.push({ test: 'Wing overturning under design gust', status: tipU >= p.designGust ? 'pass' : 'fail', detail: `Tip-over at ${fmt(tipU, 3)} m/s crosswind vs design gust ${p.designGust} m/s.`, fix: 'Reduce wing height/area, add a wing-unloading device, widen track or add ballast low; never park with an exposed wing.' });
    out.verdicts.push({ test: 'Hard requirements', status: 'pass', detail: 'No battery, no fossil fuel, no electric traction, no conventional engine in the architecture.' });
    out.verdicts.push({ test: 'Moving-part restriction', status: 'warn', detail: reqs[4][2] });
    out.verdicts.push({ test: 'Dimensional design rules', status: dims.every(d => d[3]) ? 'pass' : 'fail', detail: dims.filter(d => !d[3]).map(d => d[0]).join('; ') || 'All rules satisfied.' });
    // ---------- charts ----------
    const sorted = items.slice().sort((a, b) => b.m - a.m);
    out.charts.push({ title: 'Mass budget by component', type: 'bar', horizontal: true, categories: sorted.map(i => i.name), y: { label: 'Mass', unit: 'kg' }, series: [{ name: 'mass', values: sorted.map(i => i.m) }], height: Math.max(300, sorted.length * 16), wide: true, explain: 'Every subsystem’s mass from geometry × material density. The largest bars are where material substitution (Suite 03) pays off most.' });
    const groupMass = {}; for (const i of items) { const g = i.name.replace(/ \d+.*| \(.*\)/g, '').split(' ').slice(0, 2).join(' '); groupMass[g] = (groupMass[g] || 0) + i.m; }
    out.charts.push({ title: 'Static wheel loads', type: 'bar', categories: lay.pts.map(q => q[2]), y: { label: 'Normal load', unit: 'N' }, series: [{ name: 'Fz', values: loads }], showValues: true, explain: 'Normal force at each contact patch at rest; the basis for tyre forces (Suite 10), braking (Suite 12) and chassis loads (Suite 09).' });
    const ays = range(41).map(i => i * 0.04);
    out.charts.push({ title: 'Wheel loads vs steady lateral acceleration', type: 'line', x: { label: 'Lateral acceleration', unit: 'g' }, y: { label: 'Normal load', unit: 'N' }, series: lay.pts.map((q, j) => ({ name: q[2], x: ays, y: ays.map(a => threeWheelLoads(lay.pts, W, 0, [{ Fy: -mOp * a * G0, z: cgHeight }])[j]) })), annotations: [{ type: 'hline', y: 0, label: 'wheel lift', color: '#ef4444' }, { type: 'vline', x: ayLift, label: 'SSF' }], explain: 'As cornering acceleration grows, load transfers to the outside contacts. Where a line crosses zero the inside wheel lifts — the rollover threshold.' });
    const vs = range(61).map(i => i * 1); const Pv = vs.map(v => roadLoad(kmh(v), veh).P);
    out.charts.push({ title: 'Level-road wheel power demand', type: 'line', x: { label: 'Speed', unit: 'km/h' }, y: { label: 'Power', unit: 'W' }, series: [{ name: 'Total', x: vs, y: Pv }, { name: 'Rolling', x: vs, y: vs.map(v => roadLoad(kmh(v), veh).Pr) }, { name: 'Aerodynamic drag', x: vs, y: vs.map(v => roadLoad(kmh(v), veh).Pa) }], annotations: [{ type: 'hline', y: 200, label: '200 W target', color: '#22c55e' }, { type: 'hline', y: 600, label: '600 W target', color: '#f59e0b' }], explain: 'P = (m g C_rr + ½ρC_dA v²)v. At low speed rolling dominates; above ≈ 35 km/h aerodynamic drag dominates. These are the power gaps the backup must close.' });
    // DSM heat-map
    const ids = SUITES.map(s => s.id), Z = ids.map(r => ids.map(c => (DEPENDS[c] || []).includes(r) ? 1 : (r === c ? 0.5 : 0)));
    out.charts.push({ title: 'Design-structure matrix (row → column: output feeds input)', type: 'heatmap', x: { label: 'Consuming suite', values: SUITES.map(s => s.num) }, y: { label: 'Providing suite', values: SUITES.map(s => s.num) }, z: Z, cmap: 'inferno', z_label: 'coupled', wide: true, explain: 'Each bright cell means the row suite’s outputs are inputs to the column suite. Clusters reveal the five coupled engines (atmosphere→aero→wing→dynamics; radiation→thermal→NiTi→hub; buffer→switching→energy; structures→safety→manufacturing→economics).' });
    // tables
    out.tables.push({ title: 'Mass budget (bill of materials)', cols: ['Component', 'Material', 'Mass (kg)', 'x (m)', 'z (m)', 'Note'], rows: items.map(i => [i.name, i.mat, i.m, i.x, i.z, i.note]), explain: 'x measured forward from the body reference; z height above ground.' });
    out.tables.push({ title: 'Inertia tensor about CG (kg·m²)', cols: ['', 'x', 'y', 'z'], rows: [['x', Ixx * scale, Ixy * scale, Ixz * scale], ['y', Ixy * scale, Iyy * scale, Iyz * scale], ['z', Ixz * scale, Iyz * scale, Izz * scale]] });
    out.tables.push({ title: 'Requirements compliance & disclosure', cols: ['Requirement', 'Status', 'Evidence / disclosure'], rows: reqs.map(r => [r[0], h('span', { class: 'badge ' + ({ pass: 'ok', warn: 'warn', fail: 'fail' }[r[1]]) }, r[1]), r[2]]) });
    out.tables.push({ title: 'Dimensional design-rule checks', cols: ['Rule', 'Value', 'Limit', 'Status'], rows: dims.map(d => [d[0], d[1], d[2], d[3] ? 'pass' : 'FAIL']) });
    // 3D view
    out.views.push({ label: '3D architecture', icon: '🧊', render: async (el) => {
      const { viewer } = await import('../core/viz3d.js'); const box = h('div'); el.append(box);
      const mats = await import('../core/materials.js'); const matMap = Object.fromEntries(Object.keys(mats.COMPONENTS).map(c => [c, ctx.material(c)]));
      const V3 = await viewer(box); for (const m of vehicleAssembly(veh, { materials: matMap }).parts) V3.add(m);
      const { sphere } = await import('../core/cad.js'); const s = sphere('Centre of gravity', 0.06, 16, 8, { color: '#facc15' }); s.translate(...cg); V3.add(s);
      V3.addArrow([cg[0], cg[1], cg[2]], [0, 0, -1], 0.8, 0xfacc15); V3.fit();
      el.append(h('p', { class: 'small muted' }, `Yellow sphere: computed CG (${fmt(cg[0], 3)}, ${fmt(cg[1], 3)}, ${fmt(cg[2], 3)}) m. Colours follow the assigned material families.`));
    } });
    // recs
    if (total > p.targetMass) out.recs.push({ level: 'warn', text: `Budget exceeds target by ${fmt(total - p.targetMass, 3)} kg. Largest contributors: ${sorted.slice(0, 3).map(i => i.name).join(', ')}.`, basis: 'Mass budget' });
    if (tipU < p.designGust) out.recs.push({ level: 'critical', text: `The wing tips the stationary vehicle at ${fmt(tipU, 3)} m/s. A mechanical wing-unloading/feathering device and a parking procedure that secures the wing are mandatory before any occupied test.`, basis: 'Three-support moment equilibrium with wing side force' });
    if (ssf < 1) out.recs.push({ level: 'warn', text: `SSF ${fmt(ssf, 3)} g is below the ≈1.0 g typical of passenger cars. Prefer ${p.layout === 'tadpole' ? 'moving mass forward toward the twin front wheels' : 'switching to a tadpole layout'} and widening the track.`, basis: 'Trike tipping-axis geometry' });
    out.recs.push({ level: 'sustain', text: `Bistable buffer of ${p.bufferE} kJ requires ≈ ${fmt(bufMass, 3)} kg of ${buf.name}. Size the buffer to bridge short gaps only — each extra kilogram costs ≈ ${fmt(roadLoad(kmh(20), { ...veh, mass: 1 }).Pr, 2)} W of rolling power at 20 km/h for its whole life.`, basis: 'Specific resilience σy²/(2Eρ) and rolling resistance' });
    out.recs.push({ level: 'good', text: 'Freeze the architecture around the four fabrication gates (wing section → single cartridge → hub rig → integrated prototype) instead of repeatedly replacing it; weigh and corner-weight every built assembly to calibrate this budget.', basis: 'ATMOS X fabrication & verification gates' });
    out.explain.push({ title: 'Why the architecture matters', text: 'The vehicle is wind-primary: tall wings make propulsion possible but raise the CG and add large side forces. The three-wheel support triangle, mass distribution and wing height therefore jointly decide whether the car can accelerate, corner and stand safely in gusts.' });
    out.explain.push({ title: 'Global effect of this suite', text: 'When this suite runs, its dimensions, layout, CG height and operating mass become the global vehicle for every other suite; those suites are flagged stale and regenerate with the new geometry.' });
    // write back global vehicle
    const patch = { layout: p.layout, length: p.length, width: p.width, height: p.height, wheelbase: p.wheelbase, track: p.track, wheelRadius: p.wheelRadius, wingArea: p.wingArea, wingHeight: p.wingHeight, nWings: +p.nWings, mass: +mOp.toFixed(2), payload: p.payload, Crr: p.Crr, CdA: p.CdA, cgHeight: +cgHeight.toFixed(4), driveWheels: +p.driveWheels, frontFrac: +frontFrac.toFixed(4) };
    const changed = Object.keys(patch).some(kk => (typeof patch[kk] === 'number' ? Math.abs((ctx.vehicle[kk] ?? NaN) - patch[kk]) > 1e-6 : ctx.vehicle[kk] !== patch[kk]));
    if (changed) ctx.store.setVehicle(patch, 'Suite 01');
    out.headline = `${p.layout === 'tadpole' ? 'Tadpole' : 'Delta'} trike: computed ${fmt(total, 4)} kg (target ${p.targetMass} kg), CG ${fmt(cgHeight, 3)} m high, SSF ${fmt(ssf, 3)} g, tip-over crosswind ${fmt(tipU, 3)} m/s.${changed ? ' Global vehicle updated — dependent suites marked stale.' : ''}`;
    out.headlineKind = tipU < p.designGust || ssf < 0.75 ? 'warn' : 'info';
    out.outputs = { mass: mOp, massComputed: total, cg, inertia: { Ixx: Ixx * scale, Iyy: Iyy * scale, Izz: Izz * scale, Ixy: Ixy * scale, Ixz: Ixz * scale, Iyz: Iyz * scale }, wheelLoads: loads, wheelLabels: lay.pts.map(q => q[2]), wheelbase: p.wheelbase, track: p.track, cgHeight, ssf, frontFrac, tipOverWind: tipU, massBreakdown: Object.fromEntries(items.map(i => [i.name, i.m])), layout: p.layout };
    void groupMass;
    return out;
  },
};
