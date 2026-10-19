// Suite 01 — Complete automobile architecture & system-level engineering.
import { G0, fmt, range, sum, linspace } from '../core/num.js';
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
    { key: 'rig', label: 'Wing rig', type: 'select', group: 'Architecture', default: 'arch', options: [{ value: 'arch', label: 'AeroArch: one closed box-wing arch on a ring track (Revision 4)' }, { value: 'tandem', label: 'Separate tandem wings (Revisions 1–3)' }], auto: V('wingConfig', 'arch') },
    { key: 'archCant', label: 'Arch leg inward cant', unit: '°', type: 'number', min: 0, max: 30, step: 1, group: 'Wings', default: 12, auto: V('archCant', 12) },
    { key: 'archSweep', label: 'Arch leg sweep', unit: '°', type: 'number', min: 0, max: 40, step: 1, group: 'Wings', default: 20, auto: V('archSweep', 20) },
    { key: 'length', label: 'Overall length', unit: 'm', type: 'number', min: 2.5, max: 6, step: 0.05, group: 'Dimensions', default: 4.6, auto: V('length') },
    { key: 'width', label: 'Body width', unit: 'm', type: 'number', min: 1, max: 2.55, step: 0.05, group: 'Dimensions', default: 1.6, auto: V('width') },
    { key: 'height', label: 'Body height (excl. wings)', unit: 'm', type: 'number', min: 0.7, max: 1.8, step: 0.05, group: 'Dimensions', default: 1.0, auto: V('height') },
    { key: 'wheelbase', label: 'Wheelbase', unit: 'm', type: 'number', min: 1.5, max: 4, step: 0.05, group: 'Dimensions', default: 2.9, auto: V('wheelbase') },
    { key: 'track', label: 'Track (twin-wheel axle)', unit: 'm', type: 'number', min: 0.8, max: 2.4, step: 0.05, group: 'Dimensions', default: 1.9, auto: V('track') },
    { key: 'wheelRadius', label: 'Wheel effective radius', unit: 'm', type: 'number', min: 0.2, max: 0.45, step: 0.01, group: 'Dimensions', default: 0.3, auto: V('wheelRadius') },
    { key: 'wingArea', label: 'Wing reference area (arch: both legs; tandem: all wings deployed)', unit: 'm²', type: 'number', min: 1, max: 20, step: 0.1, group: 'Wings', default: 4.0, auto: V('wingArea') },
    { key: 'wingHeight', label: 'Arch leg height / wing span (deployed)', unit: 'm', type: 'number', min: 1, max: 6, step: 0.05, group: 'Wings', default: 1.75, auto: V('wingHeight') },
    { key: 'wingReefSpan', label: 'Wing span, reefed (telescoped)', unit: 'm', type: 'number', min: 0.8, max: 6, step: 0.1, group: 'Wings', default: 1.75, auto: V('wingReefSpan', 1.75), help: 'Tandem rigs only (Revision 2.2): the upper section telescopes into the lower one for transit and strong wind.' },
    { key: 'wingState', label: 'Wing state', type: 'select', group: 'Wings', default: 'auto', options: [{ value: 'auto', label: 'Auto (deploy in light wind, reef above the reef threshold)' }, { value: 'deployed', label: 'Deployed (full span)' }, { value: 'reefed', label: 'Reefed (telescoped)' }], auto: V('wingReef', 'auto') },
    { key: 'nWings', label: 'Number of wings', type: 'select', group: 'Wings', default: 2, options: [1, 2, 3], auto: V('nWings') },
    { key: 'wingRoot', label: 'Wing root / gust protection', type: 'select', group: 'Wings', default: 'ring', auto: (c) => ({ value: (c.vehicle.wingConfig || 'arch') === 'arch' ? 'ring' : 'feathering', prov: 'calc' }), options: [{ value: 'ring', label: 'Arch ring track: incidence + torque-limited breakaway to feathered (Revision 4)' }, { value: 'feathering', label: 'Free-feathering with torque-limited breakaway (Revision 2)' }, { value: 'fixed', label: 'Fixed root (specification strict)' }], help: 'Feathering pivot at ≈ 21 % chord; the incidence lock releases at the safe side force and the wing weathervanes (C_L ≈ 0, C_D ≈ 0.02).' },
    { key: 'stowage', label: 'Wing stowage', type: 'select', group: 'Wings', default: 'fixed', auto: (c) => ({ value: (c.vehicle.wingConfig || 'arch') === 'arch' ? 'fixed' : 'tilt', prov: 'calc' }), options: [{ value: 'fixed', label: 'Fixed upright — park feathered on the ring track (default for the arch: folding saves only ≈ 0.3 m)' }, { value: 'fold', label: 'Arch folds back over the body (parking)' }, { value: 'tilt', label: 'Tilt-down for parking / transit' }, { value: 'fixed', label: 'Fixed upright' }] },
    { key: 'wSkin', label: 'Wing skin thickness', unit: 'mm', type: 'number', min: 0.2, max: 3, step: 0.1, group: 'Wing construction', default: 0.6 },
    { key: 'coreRel', label: 'Wing lattice core relative density', unit: '%', type: 'number', min: 0.5, max: 30, step: 0.5, group: 'Wing construction', default: 3 },
    { key: 'nRibs', label: 'Flexible ribs per wing', type: 'number', min: 2, max: 20, step: 1, group: 'Wing construction', default: 6 },
    { key: 'ribT', label: 'Rib thickness', unit: 'mm', type: 'number', min: 0.5, max: 6, step: 0.5, group: 'Wing construction', default: 2 },
    { key: 'sparT', label: 'Box-spar wall thickness', unit: 'mm', type: 'number', min: 0.5, max: 6, step: 0.1, group: 'Wing construction', default: 1.2, help: 'Revision 2 sizes the CFRP spar for strength SF ≈ 2.5 (verified in Suites 06/09).' },
    { key: 'bodyShape', label: 'Body aerodynamic form', type: 'select', group: 'Body', default: 'lifting', options: [{ value: 'lifting', label: 'Lifting body: symmetric airfoil plan-form + wing-root endplate (Revision 2.3)' }, { value: 'conventional', label: 'Conventional streamlined body' }], auto: V('bodyShape', 'lifting') },
    { key: 'spats', label: 'Wheel spats / fairings', type: 'bool', group: 'Body', default: true, auto: V('spats', true), help: 'Thin CFRP fairings around the exposed wheels (≈ 0.8 kg each) cut wheel drag and side-force noise.' },
    { key: 'massBasis', label: 'Operating-mass basis', type: 'select', group: 'Mass', default: 'computed', options: [{ value: 'target', label: 'Design target mass (below)' }, { value: 'computed', label: 'Computed mass budget' }], help: 'Which mass becomes the global vehicle mass for every suite.' },
    { key: 'targetMass', label: 'Design target operating mass', unit: 'kg', type: 'number', min: 120, max: 800, step: 5, group: 'Mass', default: 300, auto: V('mass') },
    { key: 'payload', label: 'Occupant + payload', unit: 'kg', type: 'number', min: 0, max: 250, step: 5, group: 'Mass', default: 90, auto: V('payload') },
    { key: 'skin', label: 'Monocoque skin thickness (each face)', unit: 'mm', type: 'number', min: 0.3, max: 4, step: 0.05, group: 'Construction', default: 0.5, help: 'Minimum-gauge design: 2 plies of 200 g/m² woven CFRP (≈ 0.5 mm) for impact/handling damage tolerance; Suite 09 strength SF and panel buckling remain far above 2.5 / 3 at this gauge.' },
    { key: 'core', label: 'Sandwich core thickness', unit: 'mm', type: 'number', min: 3, max: 40, step: 1, group: 'Construction', default: 6, help: 'Minimum for potted inserts and local crush; set by gauge, not strength (Suite 09).' },
    { key: 'cartPerWheel', label: 'NiTi cartridges per driven wheel', type: 'number', min: 0, max: 64, step: 1, group: 'Propulsion hardware', default: 24 },
    { key: 'cartMass', label: 'Active NiTi mass per cartridge', unit: 'kg', type: 'number', min: 0.005, max: 0.5, step: 0.005, group: 'Propulsion hardware', default: 0.06 },
    { key: 'driveWheels', label: 'Driven (hub-drive) wheels', type: 'select', group: 'Propulsion hardware', default: 2, options: [0, 1, 2, 3], auto: V('driveWheels') },
    { key: 'bufferE', label: 'Elastic-buffer usable energy target', unit: 'kJ', type: 'number', min: 0, max: 600, step: 5, group: 'Propulsion hardware', default: 10 },
    { key: 'Crr', label: 'Rolling-resistance coefficient', type: 'number', min: 0.002, max: 0.03, step: 0.0005, group: 'Road load', default: 0.006, auto: V('Crr') },
    { key: 'CdA', label: 'Body drag area C_dA', unit: 'm²', type: 'number', min: 0.08, max: 1, step: 0.01, group: 'Road load', default: 0.25, auto: V('CdA') },
    { key: 'designGust', label: 'Design gust for tip-over check', unit: 'm/s', type: 'number', min: 5, max: 40, step: 0.5, group: 'Road load', default: 20 },
  ],
  presets: [{ label: 'Revision 4.1 AeroArch (default: box-wing arch, 5.2 m² legs, 2.0 m, 3.0 m overall, 0.9 m lifting body)', values: { rig: 'arch', layout: 'tadpole', length: 4.6, width: 1.6, height: 0.9, wheelbase: 2.6, track: 1.9, wingArea: 5.2, wingHeight: 2.0, wingReefSpan: 2.0, archCant: 6, archSweep: 8, nWings: 2, wingRoot: 'ring', stowage: 'fixed', Crr: 0.004, CdA: 0.18 } }, { label: 'Revision 3 baseline (3 × 2.0 m fixed wings, 3.0 m overall, 5.4 m lifting body)', values: { layout: 'tadpole', length: 5.4, width: 1.6, height: 1.0, wheelbase: 3.3, track: 1.9, wingArea: 4.34, wingHeight: 2.0, wingReefSpan: 2.0, nWings: 3, rig: 'tandem', wingRoot: 'feathering', stowage: 'tilt' } }, { label: 'Revision 2.2 baseline (reefable 3.0/2.0 m wings, 5.0 m², track 1.8 m)', values: { layout: 'tadpole', length: 4.4, width: 1.6, height: 1.0, wheelbase: 2.8, track: 1.8, wingArea: 5.0, wingHeight: 3.0, wingReefSpan: 2.0, wingState: 'auto', nWings: 2, rig: 'tandem', wingRoot: 'feathering', stowage: 'tilt' } }, { label: 'Specification starting model (2 × 3 m wings, 8 m², track 1.5 m)', values: { layout: 'tadpole', length: 3.6, width: 1.6, height: 1.1, wheelbase: 2.4, track: 1.5, wingArea: 8, wingHeight: 3, wingReefSpan: 3, wingState: 'deployed', nWings: 2, rig: 'tandem', wingRoot: 'fixed', stowage: 'fixed' } }, { label: 'Delta layout', values: { layout: 'delta' } }, { label: 'Wide-track stability', values: { track: 1.9, height: 1.0 } }, { label: 'Single large wing', values: { nWings: 1, wingArea: 8, wingHeight: 3.6 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const veh0 = { ...ctx.vehicle, layout: p.layout, length: p.length, width: p.width, height: p.height, wheelbase: p.wheelbase, track: p.track, wheelRadius: p.wheelRadius, wingArea: p.wingArea, wingHeight: p.wingHeight, wingReefSpan: Math.min(p.wingHeight, p.wingReefSpan), nWings: +p.nWings, wingConfig: p.rig, archCant: p.archCant, archSweep: p.archSweep };
    const M = (k) => ctx.material(k);
    const asm = vehicleAssembly(veh0, {}), asmR = vehicleAssembly(veh0, { reefed: true }), asmF = vehicleAssembly(veh0, { folded: true }); const bodyArea = asm.parts[0].area(), meta = asm.meta, R = p.wheelRadius, Hb = p.height;
    // ---------- component mass budget ----------
    const items = [];
    const add = (name, comp, mass, x, y, z, dims = [0.2, 0.2, 0.2], note = '') => items.push({ name, comp, mat: comp ? M(comp)?.name : '—', m: mass, x, y, z, dims, note });
    const ch = M('chassis'), core = M('chassisCore');
    add('Monocoque sandwich shell', 'chassis', bodyArea * (2 * p.skin / 1000 * ch.rho + p.core / 1000 * core.rho) * 1.12, 0.08 * p.length, 0, R + Hb * 0.36, [p.length, p.width * 0.8, Hb * 0.7], `${fmt(bodyArea, 3)} m² surface; +12 % joints, resin & inserts`);
    const isArch = meta.wingConfig === 'arch' && !!meta.arch, AR4 = meta.arch;
    const nW = isArch ? 2 : meta.wingX.length, span = isArch ? AR4.legHeight : p.wingHeight, areaW = isArch ? p.wingArea : (meta.areaFitted ?? p.wingArea), S1 = areaW / nW, cMean = isArch ? (AR4.rootChord + AR4.tipChord) / 2 : S1 / span, zRoot = isArch ? AR4.zRoot : meta.height - span;
    const wingStruct = [], sk = M('wingSkin'), sp = M('spar'), rb = M('rib'), lt = M('lattice');
    const wingPart = (lab, x, y, z, S, c, len, nRib) => { const m0 = items.length;
      add(`${lab} morphing skin (${p.wSkin} mm)`, 'wingSkin', 2.05 * S * p.wSkin / 1000 * sk.rho, x, y, z, [c, 0.15 * c, len]);
      add(`${lab} box spar (${p.sparT} mm wall)`, 'spar', len * 2 * (0.12 + 0.1) * c * p.sparT / 1000 * sp.rho * 1.15, x, y, z, [0.12 * c, 0.1 * c, len]);
      add(`${lab} flexible ribs (×${nRib}, ${p.ribT} mm, 50 % lightening)`, 'rib', nRib * 0.1 * c * c * p.ribT / 1000 * 0.5 * rb.rho, x, y, z, [c, 0.1, len]);
      add(`${lab} metamaterial core (${p.coreRel} % rel. density)`, 'lattice', S * 0.085 * c * 0.3 * (p.coreRel / 100) * lt.rho, x, y, z, [c * 0.6, 0.1 * c, len], `${lt.name}; morphing regions = 30 % of wing volume`);
      wingStruct.push(items.slice(m0).reduce((a2, i) => a2 + i.m, 0)); };
    if (isArch) {
      const xa = meta.wingX[0], legLen = span / Math.cos(AR4.sweep * Math.PI / 180), zc = zRoot + span * 0.45;
      for (const sd of [1, -1]) { wingPart(`Arch leg ${sd > 0 ? 'port' : 'starboard'}`, xa - 0.3 * span * Math.tan(AR4.sweep * Math.PI / 180), sd * (AR4.gap / 2 - 0.2 * span * Math.tan(AR4.cant * Math.PI / 180)), zc, S1, cMean, legLen, p.nRibs); add(`Arch leg ${sd > 0 ? 'port' : 'starboard'} root fitting`, 'rootInsert', 1.2 * (M('rootInsert').rho / 4430), xa, sd * AR4.gap / 2, zRoot, [0.15, 0.15, 0.15]); }
      const cb = AR4.bridgeArea / Math.max(0.1, AR4.bridgeSpan); wingPart('Arch top bridge', xa - span * Math.tan(AR4.sweep * Math.PI / 180), 0, AR4.zTop - 0.12, AR4.bridgeArea, cb, AR4.bridgeSpan, Math.max(2, Math.round(p.nRibs / 2)));
    } else for (let k = 0; k < nW; k++) {
      const wx = meta.wingX[k], m1 = items.length; wingPart(`Wing ${k + 1}`, wx, 0, zRoot + span * 0.45, S1, cMean, span, p.nRibs); void m1;
      add(`Wing ${k + 1} root fitting${p.wingRoot === 'feathering' ? ' + feathering pivot & breakaway lock' : ''}`, 'rootInsert', (p.wingRoot === 'feathering' ? 3.0 : 2.2) * (M('rootInsert').rho / 4430), wx, 0, zRoot, [0.2, 0.2, 0.2]);
    }
    const wingAreal = sum(wingStruct) / (areaW + (isArch ? AR4.bridgeArea : 0));
    if (isArch) {
      // ring-track bulkheads (fore/aft) + ring track, incidence drive & breakaway lock; keel beam to the wheel hard points
      const xa = meta.wingX[0], c0 = AR4.chord;
      add('Arch ring-track bulkhead (fore, CFRP)', 'chassis', 1.8 * (ch.rho / 1550), xa + 0.35 * c0, 0, R + Hb * 0.35, [0.06, p.width * 0.8, Hb * 0.7]);
      add('Arch ring-track bulkhead (aft, CFRP)', 'chassis', 1.8 * (ch.rho / 1550), xa - 0.6 * c0, 0, R + Hb * 0.35, [0.06, p.width * 0.8, Hb * 0.7]);
      add('Arch ring track, incidence drive & breakaway lock', 'rootInsert', 4.0, xa, 0, zRoot, [0.3, AR4.gap, 0.08], 'single mechanism: incidence for any wind direction + torque-limited breakaway to the feathered position');
      add('Keel beam (ring-track loads → wheel hard points)', 'chassis', 4.0 * (ch.rho / 1550), (xa + (meta.wheelPos.find(w => w[0] > 0)?.[0] ?? 1)) / 2, 0, R + 0.06, [Math.abs((meta.wheelPos.find(w => w[0] > 0)?.[0] ?? 1) - xa), 0.12, 0.08]);
    } else {
      // mast-step bulkheads (one per wing) and keel beam carrying root loads to the wheel hard points (Revision 2.1)
      for (let k = 0; k < nW; k++) add(`Wing ${k + 1} mast-step bulkhead (CFRP)`, 'chassis', 1.8 * (ch.rho / 1550), meta.wingX[k], 0, R + Hb * 0.35, [0.06, p.width * 0.8, Hb * 0.7]);
      add('Keel beam (root loads → wheel hard points)', 'chassis', 4.0 * (ch.rho / 1550), (meta.wingX[nW - 1] + (meta.wheelPos.find(w => w[0] > 0)?.[0] ?? 1)) / 2, 0, R + 0.06, [Math.abs((meta.wheelPos.find(w => w[0] > 0)?.[0] ?? 1) - meta.wingX[nW - 1]), 0.12, 0.08]);
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
    const xFront = meta.wheelPos.filter(w => w[0] > 0)[0]?.[0] ?? p.wheelbase * 0.58;
    add('Bistable elastic buffer', 'buffer', bufMass, xFront - 0.35, 0, R + 0.1, [0.6, 0.4, 0.2], `${fmt(sres, 3)} J/kg material resilience × 30 % packaging`);
    // conformal sky-facing thermal surfaces following the body's top deck (same section law as the CAD assembly)
    const bodySec = (u) => { const halfW = p.width / 2 * Math.max(0.06, Math.sin(Math.PI * Math.min(1, u * 1.05)) ** 0.55) * (u > 0.9 ? 1 - (u - 0.9) * 4 : 1); const halfH = Hb * 0.36 * Math.max(0.06, Math.sin(Math.PI * u) ** 0.7); return { x: u * p.length - p.length * 0.42, a: Math.max(0.02, halfW * 0.82), top: R + 0.02 + halfH * 1.9 }; };
    const deck = (u0, u1) => { let A = 0, xm = 0, zm = 0; const n = 10; for (let i = 0; i < n; i++) { const q = bodySec(u0 + (u1 - u0) * (i + 0.5) / n), dA = (u1 - u0) * p.length / n * 2 * q.a * 0.9; A += dA; xm += q.x * dA; zm += q.top * dA; } return { A, x: xm / A, z: zm / A }; };
    const uOf = (x) => Math.min(1, Math.max(0, (x + p.length * 0.42) / p.length)), cock = meta.cockpitX || [p.length * 0.58 - 2.0, p.length * 0.58 - 0.8];
    const dCol = deck(Math.min(uOf(cock[1]), 0.9), 0.96), dRad = deck(0.03, Math.max(0.06, uOf(cock[0]) - 0.03));
    if (isArch) add('Glazed solar-selective collector (arch bridge top, sky-facing)', 'collector', AR4.collectorArea * (0.0012 * 2700 + 0.0032 * 2500 + 0.02 * 40), meta.wingX[0] - span * Math.tan(AR4.sweep * Math.PI / 180), 0, AR4.zTop, [0.8, AR4.bridgeSpan, 0.03], `${fmt(AR4.collectorArea, 3)} m² on the bridge: Al absorber 1.2 mm + low-iron glazing 3.2 mm + insulation; also shades the cockpit`);
    else add('Glazed solar-selective collector (nose deck, unshaded)', 'collector', dCol.A * (0.0012 * 2700 + 0.0032 * 2500 + 0.02 * 40), dCol.x, 0, dCol.z, [0.14 * p.length, p.width * 0.6, 0.03], `${fmt(dCol.A, 3)} m² conformal patch: Al absorber 1.2 mm + low-iron glazing 3.2 mm + insulation`);
    add('Radiative-cooling emitter (rear deck)', 'radiator', dRad.A * (0.0012 * 2700 + 0.3), dRad.x, 0, dRad.z, [0.16 * p.length, p.width * 0.6, 0.01], `${fmt(dRad.A, 3)} m² conformal patch`);
    add('Heat spreaders, heat pipes, insulation', 'spreader', 4.5, -p.length * 0.1, 0, R + Hb * 0.5, [0.8, 0.6, 0.3]);
    add('Steering system (mechanical)', 'steering', 6.5, meta.wheelPos[0][0], 0, R + 0.1, [0.5, p.track * 0.8, 0.2]);
    add('Cabin canopy / glazing', 'cabin', 1.2 * 0.004 * M('cabin').rho, 0.35, 0, R + Hb * 0.62, [1.1, p.width * 0.5, 0.5]);
    const xOcc = (cock[0] + cock[1]) / 2; // occupant at the forward cockpit centre (Revision 2.1)
    add('Recumbent seat, restraints & controls', null, 9, xOcc, 0, R + 0.12, [0.6, 0.5, 0.3]);
    if (!isArch) add('Wing unloading / incidence mechanism', null, p.wingConfig === 'practical' ? 7 : 2.5, meta.wingX[0], 0, zRoot - 0.1, [0.3, 0.3, 0.2]);
    const reefable = !isArch && p.wingReefSpan < p.wingHeight - 1e-6;
    if (reefable) add('Wing telescoping reef mechanism (2 wings)', null, 3.5, meta.wingX[0], 0, zRoot + p.wingReefSpan, [0.3, 0.2, 0.1], 'joint collars, guides & mechanical latch');
    if (isArch && p.stowage === 'fold') add('Arch fold hinge & parking latch', null, 3.0, meta.wingX[0], 0, zRoot, [0.3, AR4.gap, 0.1], 'the arch folds back over the body for parking');
    if (!isArch && p.stowage === 'tilt') add('Wing tilt-down stowage mechanism', null, 6, meta.wingX[0], 0, zRoot - 0.15, [0.4, 0.3, 0.2], 'hinge, latch & gas-strut-free mechanical linkage');
    if (p.spats) meta.wheelPos.forEach(([x, y], k) => add(`Wheel ${k + 1} spat / fairing (CFRP)`, 'chassis', 0.45 * 0.001 * ch.rho * 1.1, x, y, R, [2.1 * R, 0.12, 1.6 * R]));
    const struct = sum(items.map(i => i.m)); add('Fasteners, sealing & miscellaneous (6 %)', null, 0.06 * struct, 0, 0, R + Hb * 0.4, [p.length, p.width, Hb]);
    const empty = sum(items.map(i => i.m));
    add('Occupant + payload (recumbent)', null, p.payload, xOcc, 0, R + 0.22, [1.1, 0.45, 0.35]);
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
    const ssf = trikeSSF(veh, frontFrac), ayLift = rolloverAy(veh, mOp, { frontFrac }), hs = zRoot + span * (isArch ? 0.5 : 0.45);
    const Csd = isArch ? 1.2 * 0.85 : 1.2; // arch legs act as a laterally separated biplane in crosswind: mutual interference ≈ 0.85
    const tipScan = (fn) => { for (let U = 0.5; U <= 80; U += 0.1) if (Math.min(...fn(U)) <= 0) return U; return 80; };
    const spanR = Math.min(p.wingHeight, p.wingReefSpan), areaR = p.wingArea * spanR / p.wingHeight, hsR = zRoot + spanR * 0.45;
    const BF1 = ctx.upstream('s07')?.bodyForces, rr0 = ctx.env?.rho ?? 1.225, AbT = BF1?.Aref ?? 0.8 * p.length * Hb * 0.75, CYT = BF1?.CYvsAWA?.CY ? Math.max(...BF1.CYvsAWA.CY.map(Math.abs)) : 0.8, hbT = BF1?.hCE_m ?? R + Hb * 0.45;
    const bodyL = (U) => ({ Fy: -0.5 * rr0 * U * U * AbT * CYT, z: hbT });
    const tipU = tipScan((U) => threeWheelLoads(lay.pts, W, 0, [{ Fy: -0.5 * rr0 * U * U * p.wingArea * Csd, z: hs }, bodyL(U)]));
    const tipUR = tipScan((U) => threeWheelLoads(lay.pts, W, 0, [{ Fy: -0.5 * rr0 * U * U * areaR * 1.2, z: hsR }, bodyL(U)]));
    void tipOverWind;
    // Revision 2 gust protection: free-feathering root releases at the safe side force; above release the wing weathervanes
    const rhoA = ctx.env?.rho ?? 1.225, mu0 = 0.95, FySafe = Math.min(0.7 * mu0 * mOp * G0, 0.8 * ssf * mOp * G0 * cgHeight / hs);
    const Urelease = Math.sqrt(FySafe / (0.5 * rhoA * p.wingArea * Csd));
    const FySafeR = Math.min(0.7 * mu0 * mOp * G0, 0.8 * ssf * mOp * G0 * cgHeight / hsR), UreleaseR = Math.sqrt(FySafeR / (0.5 * rhoA * areaR * 1.2));
    const loadedOKR = Math.min(...threeWheelLoads(lay.pts, W, 0, [{ Fy: -FySafeR, z: hsR }])) > 0;
    const Abody = 0.8 * p.length * Hb * 0.75, hBody = R + Hb * 0.45;
    const featheredLoads = (U) => threeWheelLoads(lay.pts, W, 0, [{ Fy: -0.5 * rhoA * U * U * p.wingArea * 0.02, z: hs }, { Fy: -0.5 * rhoA * U * U * Abody * 0.8, z: hBody }]);
    const stowedLoads = (U) => threeWheelLoads(lay.pts, W, 0, [{ Fy: -0.5 * rhoA * U * U * (Abody * 0.8 + p.wingArea * 0.35), z: hBody }]);
    const tipFeather = (() => { for (let U = 1; U <= 80; U += 0.25) if (Math.min(...featheredLoads(U)) <= 0) return U; return 80; })();
    const tipStowed = (() => { for (let U = 1; U <= 80; U += 0.25) if (Math.min(...stowedLoads(U)) <= 0) return U; return 80; })();
    const feather = p.wingRoot === 'feathering' || p.wingRoot === 'ring', loadedOK = feather ? Math.min(...threeWheelLoads(lay.pts, W, 0, [{ Fy: -FySafe, z: hs }])) > 0 : tipU >= p.designGust;
    const gustOK = feather ? tipFeather >= p.designGust && loadedOKR : tipU >= p.designGust;
    const overallH = meta.height, reefedH = asmR.meta.height, stowedH = isArch ? asmF.meta.height : meta.height - span + 0.15, gapTandem = !isArch && nW > 1 ? Math.abs(meta.wingX[1] - meta.wingX[0]) : Infinity;
    const P20 = roadLoad(kmh(20), veh).P, P40 = roadLoad(kmh(40), veh).P;
    // ---------- requirements compliance ----------
    const reqs = [
      ['No fossil-fuel propulsion', 'pass', 'No combustion system or fuel store in the bill of materials.'],
      ['No battery', 'pass', 'No electrochemical storage; finite mechanical buffer only (bistable cells).'],
      ['No operational electric traction', 'pass', 'Wheel torque only from wind (via tyres) or the thermomechanical hub.'],
      ['No conventional engine, turbine, gearbox or flywheel', 'pass', 'Direct aerodynamic force + wheel-integrated actuators.'],
      ['Only wheels, steering and brakes move', 'warn', `Disclosed internal motion: ${+p.driveWheels * p.cartPerWheel} NiTi cartridges cycle and contact a stationary cam; buffer trigger; passive switch flexures${p.wingConfig === 'practical' ? '; adjustable wing-root incidence & unloading mechanism' : ''}${isArch ? '; arch ring track (incidence for any wind direction, torque-limited breakaway to feathered)' + (p.stowage === 'fold' ? ' and arch fold hinge & latch — these replace per-wing pivots and reefing' : '') : (p.wingRoot === 'feathering' ? '; wing-root feathering pivot with torque-limited breakaway lock' : '') + (p.stowage === 'tilt' ? '; wing tilt-down stowage hinge & latch' : '') + (p.wingReefSpan < p.wingHeight ? '; telescoping wing reef joint & latch' : '')}.`],
      ['Metamaterials essential to function', 'pass', 'Wing morphing core, photonic collectors, thermal metamaterials, NiTi lattice, bistable buffer.'],
      ['Completely noiseless', 'warn', 'Not achievable: tyre–road and airflow noise remain (Suite 21).'],
      ['Robust automatic backup in calm air', 'fail', 'Not demonstrated — must be proven by Suites 15–20 and physical Gates 2–4.'],
    ];
    // ---------- dimension / design-rule checks ----------
    const dims = [
      ['Overall width ≤ 2.55 m (common road-vehicle limit)', p.width, 2.55, p.width <= 2.55],
      ['Overall width (track + tyre/spat width) ≤ 2.0 m', p.track + (p.spats ? 0.10 : 0.08), 2.0, p.track + (p.spats ? 0.10 : 0.08) <= 2.0 + 1e-9],
      ['Overall height with wings ≤ 4.0 m (legal maximum; 1 cm modelling tolerance)', overallH, 4.0, overallH <= 4.01],
      ['Tandem spacing ≥ 1 root chord (wake/downwash interference)', !isArch && nW > 1 ? gapTandem : 0, !isArch && nW > 1 ? meta.cr : 0, isArch || nW <= 1 ? true : gapTandem >= meta.cr],
      ['Wheelbase ≤ 0.85 × length', p.wheelbase, 0.85 * p.length, p.wheelbase <= 0.85 * p.length],
      ['Wing root chords fit body length', meta.cr * nW, p.length * 0.95, meta.cr * nW <= p.length * 0.95],
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
    k('CG height', cgHeight, 'm', 'Height of the combined CG above ground — tall wings raise it and reduce rollover resistance.', 'calc', cgHeight < 0.6 ? 'ok' : cgHeight < 0.75 ? 'warn' : 'bad');
    k('Front-axle load fraction', frontFrac * 100, '%', 'Share of weight on the front axle (Revision 2 target ≥ 60 % on the twin front wheels).', 'calc', p.layout !== 'tadpole' || frontFrac >= 0.6 ? 'ok' : 'warn');
    k('I_xx (roll)', Ixx * scale, 'kg·m²', 'Roll inertia about the CG (parallel-axis sum).');
    k('I_yy (pitch)', Iyy * scale, 'kg·m²', 'Pitch inertia about the CG.');
    k('I_zz (yaw)', Izz * scale, 'kg·m²', 'Yaw inertia about the CG — governs yaw response to gusts and steering (Suite 11).');
    k('Static stability factor', ssf, 'g', 'Lateral acceleration at which an inside wheel lifts (trike tipping axis).', 'calc', ssf >= 1 ? 'ok' : ssf >= 0.75 ? 'warn' : 'bad');
    k('Tip-over crosswind, wing loaded & locked', tipU, 'm/s', 'Crosswind whose fully loaded wing side force (C_side 1.2) plus the body side force lifts the windward wheel(s).', 'calc', tipU >= p.designGust ? 'ok' : feather ? 'warn' : 'bad');
    if (feather) { k('Breakaway (feathering) side force', FySafe, 'N', `Release setting = min(0.7 μmg, 0.8·SSF·mg·h_cg/h_CE); reached at ≈ ${fmt(Urelease, 3)} m/s crosswind.`, 'calc'); k('Tip-over crosswind, wing feathered', tipFeather, 'm/s', 'After release the wing weathervanes (C_D ≈ 0.02); body side force (C_y 0.8) remains.', 'calc', tipFeather >= p.designGust ? 'ok' : 'bad'); }
    if (p.stowage === 'tilt' || p.stowage === 'fold') k(isArch ? 'Tip-over crosswind, arch folded' : 'Tip-over crosswind, wings stowed', tipStowed, 'm/s', 'Parked/transit configuration with wings tilted down.', 'calc', tipStowed >= p.designGust ? 'ok' : 'bad');
    if (reefable) { k('Overall height, wings reefed', reefedH, 'm', 'Telescoped wings for transit and strong wind (≈ 3.0 m target).', 'calc', reefedH <= 3.01 ? 'ok' : reefedH <= 4.0 ? 'warn' : 'bad'); k('Tip-over crosswind, reefed & locked', tipUR, 'm/s', `Reefed area ${fmt(areaR, 3)} m², centre of effort ${fmt(hsR, 3)} m.`, 'calc', tipUR >= p.designGust ? 'ok' : feather ? 'warn' : 'bad'); }
    k('Overall height (wings deployed)', overallH, 'm', '≤ 4.0 m legal; ≤ 3.0 m recommended for urban overhead structures, trees and sightlines.', 'calc', overallH <= 3.01 ? 'ok' : overallH <= 4.01 ? 'warn' : 'bad');
    if (p.stowage === 'tilt' || p.stowage === 'fold') k(isArch ? 'Parked height (arch folded)' : 'Parked / transit height (wings down)', stowedH, 'm', isArch ? 'Arch folded back over the body.' : 'Body height plus stowed wing thickness (≈ 0.15 m).', 'calc');
    k('Wing structure areal mass', wingAreal, 'kg/m²', 'Wing skin, spar, ribs and lattice core ÷ wing area (root fittings belong to the chassis load path); rigid land-yacht wingsails achieve ≈ 3–5 kg/m².', 'calc', wingAreal <= 5 ? 'ok' : wingAreal <= 6.5 ? 'warn' : 'bad');
    if (!isArch && nW > 1) k('Tandem spacing / root chord', gapTandem / meta.cr, '–', 'The rear wing works in the front wing’s downwash and wake; ≥ 1 chord keeps interference moderate (thrust of the rear wing still reduced — resolved in Suite 07).', 'calc', gapTandem >= meta.cr ? 'ok' : 'warn');
    k('Wheel power at 20 km/h', P20, 'W', 'Ideal level-road wheel power; specification reference 124 W for the 300 kg baseline.');
    k('Wheel power at 40 km/h', P40, 'W', 'Specification reference 406 W.');
    k('Body plan-form thickness ratio (width / length)', p.width / p.length, '–', p.bodyShape === 'lifting' ? 'Lifting body: the plan-form acts as a symmetric airfoil of this t/c in crosswind and as an endplate for the wing roots; its forces are solved in Suite 07.' : 'Conventional body: treated as drag area only.', 'calc');
    k('Wing loading', mOp / p.wingArea, 'kg/m²', 'Mass per square metre of sail: lower is better for light-wind acceleration.');
    // ---------- verdicts ----------
    out.verdicts.push({ test: 'Mass budget vs target', status: total <= p.targetMass ? 'pass' : total <= 1.1 * p.targetMass ? 'warn' : 'fail', detail: `Computed ${fmt(total, 4)} kg vs target ${p.targetMass} kg.`, fix: 'Reduce sandwich skin/core thickness, switch heavy parts to lighter materials (Suite 03) or reduce buffer energy.' });
    out.verdicts.push({ test: 'Static stability (trike)', status: ssf >= 1 ? 'pass' : ssf >= 0.75 ? 'warn' : 'fail', detail: `SSF ${fmt(ssf, 3)} g with CG height ${fmt(cgHeight, 3)} m and track ${p.track} m (${p.layout}).`, fix: 'Widen the track, lower the CG, move mass toward the twin-wheel axle.' });
    out.verdicts.push({ test: 'Wing overturning under design gust', status: gustOK && loadedOK ? 'pass' : 'fail', detail: feather ? `Feathering root: lock releases at ${fmt(FySafe, 4)} N (≈ ${fmt(Urelease, 3)} m/s crosswind) with all wheels still loaded (${loadedOK ? 'yes' : 'NO'}); feathered, the car tips only at ${fmt(tipFeather, 3)} m/s vs design gust ${p.designGust} m/s. A locked (non-feathering) wing would tip at ${fmt(tipU, 3)} m/s deployed${reefable ? ` and ${fmt(tipUR, 3)} m/s reefed; reefed lock release at ${fmt(FySafeR, 4)} N with all wheels loaded (${loadedOKR ? 'yes' : 'NO'})` : ''}.` : `Fixed root: tip-over at ${fmt(tipU, 3)} m/s crosswind vs design gust ${p.designGust} m/s.`, fix: 'Use the free-feathering root with breakaway (Revision 2), lower the wing, widen the track; never park with an exposed wing.' });
    if (p.layout === 'tadpole') out.verdicts.push({ test: 'Forward mass bias (≥ 60 % front)', status: frontFrac >= 0.6 ? 'pass' : 'warn', detail: `${fmt(frontFrac * 100, 3)} % on the twin front wheels.`, fix: 'Move the cockpit and buffer toward the front axle (Suite 02 geometry).' });
    const Hroad = reefable ? reefedH : overallH; out.verdicts.push({ test: 'Overall height vs overhead clearance', status: overallH > 4.01 ? 'fail' : Hroad <= 3.01 ? 'pass' : 'warn', detail: `${fmt(overallH, 3)} m deployed${reefable ? `, ${fmt(reefedH, 3)} m reefed (reef for urban transit and strong wind)` : ''} (1 cm tolerance on the 3.0 m recommendation)${p.stowage === 'tilt' || p.stowage === 'fold' ? `; ${fmt(stowedH, 3)} m parked (${isArch ? 'arch folded' : 'wings tilted down'})` : ''}. ≤ 4.0 m legal; ≤ 3.0 m recommended for overhead structures, trees and sightlines.` });
    if (isArch) out.verdicts.push({ test: 'Arch box-wing configuration', status: 'info', detail: `Closed box wing: legs ${fmt(span, 3)} m, gap ${fmt(AR4.gap, 3)} m, cant ${AR4.cant}°, sweep ${fmt(AR4.sweep, 3)}°, bridge ${fmt(AR4.bridgeArea, 3)} m² carrying a ${fmt(AR4.collectorArea, 3)} m² collector. No free tips; in crosswind the legs act as a laterally separated biplane (interference factor 0.85 applied). The ring track rotates the whole arch for incidence and feathering; the frame doubles as a roll-over hoop.` });
    if (!isArch && nW > 1) out.verdicts.push({ test: 'Tandem-wing interference', status: gapTandem >= meta.cr ? 'info' : 'warn', detail: `Pivot spacing ${fmt(gapTandem, 3)} m = ${fmt(gapTandem / meta.cr, 3)} root chords (clear gap ${fmt(gapTandem - meta.cr, 3)} m). The rear wing operates in the front wing’s downwash and wake, so its effective angle and dynamic pressure are reduced — evaluate the pair in Suite 07 rather than doubling single-wing thrust.` });
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
    out.views.push({ label: '3D load path, mass map & tip-over (animated)', icon: '🎬', render: async (el) => {
      const [{ vehicleScene, playback, hud, legend3d }, { cmap }, mats] = await Promise.all([import('../core/sim3d.js'), import('../core/chart.js'), import('../core/materials.js')]);
      const matMap = Object.fromEntries(Object.keys(mats.COMPONENTS).map(c => [c, ctx.material(c)]));
      const box = h('div'); el.append(box);
      const vehNow = { ...veh, layout: p.layout, length: p.length, width: p.width, height: p.height, wheelbase: p.wheelbase, track: p.track, wheelRadius: p.wheelRadius, wingArea: p.wingArea, wingHeight: p.wingHeight, nWings: +p.nWings };
      const S = await vehicleScene(box, { vehicle: vehNow, materials: matMap, follow: false, height: 480 }); const THREE = S.THREE;
      // mass map: colour every part by the solved mass of its component group
      const compMass = {}; for (const it of items) if (it.comp) compMass[it.comp] = (compMass[it.comp] || 0) + it.m;
      const mVals = Object.values(compMass), mLo = 0, mHi = Math.max(...mVals);
      const meshes = []; S.car.traverse(o => { if (o.isMesh) meshes.push(o); });
      for (const o of meshes) { const cm = compMass[o.userData.component]; if (cm !== undefined) { const c = cmap(cm / mHi, 'turbo'); o.material.color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255); } }
      legend3d(box, { title: 'Component-group mass (solved budget)', lo: mLo, hi: mHi, unit: ' kg' });
      // explode: top-level children of the car move radially away from the solved CG
      const kids = S.car.children.filter(o => o.type !== 'ArrowHelper'), base = kids.map(o => o.position.clone()), ctr = kids.map(o => { const b = new THREE.Box3().setFromObject(o); return b.getCenter(new THREE.Vector3()); });
      const cgV = new THREE.Vector3(...cg);
      const cgBall = new THREE.Mesh(new THREE.SphereGeometry(0.07, 20, 12), new THREE.MeshBasicMaterial({ color: 0xfacc15, depthTest: false })); cgBall.renderOrder = 10; cgBall.position.copy(cgV); S.car.add(cgBall);
      const gArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), cgV, 0.8, 0xfacc15, 0.15, 0.08); S.car.add(gArrow);
      const asmW = S.asm.meta.wheelPos, loadArrows = asmW.map(([x, y]) => { const a = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(x, y + (y >= 0 ? 0.22 : -0.22) * (Math.abs(y) > 0.01 ? 1 : 0) + (Math.abs(y) <= 0.01 ? 0.25 : 0), 0.01), 0.5, 0x22c55e, 0.12, 0.07); S.car.add(a); return a; });
      const W0 = mOp * G0, Lmax = W0 * 0.8, kT = 150000, Ugust = feather ? p.designGust * 1.1 : Math.max(tipU * 1.15, 1);
      const Fs = (U) => 0.5 * rhoA * U * U * p.wingArea * 1.2;
      const H = hud(box); let mag = 1, magSet = false;
      const magLab = h('span', { class: 'small' }, ''), magIn = h('input', { type: 'range', min: 1, max: 150, value: 1, oninput: (e) => { mag = +e.target.value; } });
      playback(box, { duration: 20, rate: 1, label: 'Sequence', onFrame: (t) => {
        const ex = t < 4 ? t / 4 : t < 6 ? 1 - (t - 4) / 2 : 0;
        kids.forEach((o, i) => { const d = ctr[i].clone().sub(cgV); o.position.copy(base[i]).add(d.multiplyScalar(0.9 * ex)); });
        const U = t < 6 ? 0 : Math.min(Ugust, Ugust * (t - 6) / 12), F = Fs(U);
        const fe = feather && F > FySafe, L = fe ? featheredLoads(U).map(x => x) : threeWheelLoads(lay.pts, W0, 0, [{ Fy: F, z: hs }]); if (fe) S.setWingTwist(0);
        const tw = p.layout === 'delta' ? [1, 2] : [0, 1], rollTrue = Math.atan(((L[tw[1]] - L[tw[0]]) / kT) / p.track) * 180 / Math.PI;
        if (!magSet && t >= 6) { magSet = true; const Lg = threeWheelLoads(lay.pts, W0, 0, [{ Fy: Math.min(Fs(Ugust), feather ? FySafe : Infinity), z: hs }]), rg = Math.abs(Math.atan(((Lg[tw[1]] - Lg[tw[0]]) / kT) / p.track) * 180 / Math.PI); mag = Math.min(150, Math.max(1, Math.round(6 / Math.max(1e-3, rg)))); magIn.value = mag; }
        S.setPose({ x: 0, y: 0, heading: 0, roll: rollTrue * mag });
        magLab.textContent = `Roll magnification ×${mag} (true-scale roll = ${fmt(rollTrue, 3)}°, from tyre radial compliance)`;
        L.forEach((Fz, k2) => { const a = loadArrows[k2]; a.visible = Fz > 1; if (Fz > 1) a.setLength(Math.max(0.05, 2.4 * Fz / Lmax), 0.14, 0.08); a.setColor(Fz < 0.15 * W0 / 3 ? 0xef4444 : 0x22c55e); });
        const Fside = fe ? 0.5 * rhoA * U * U * (p.wingArea * 0.02 + Abody * 0.8) : F; S.setArrow('side', [0, Fside, 0], 1.5 / Math.max(1, feather ? FySafe : Fs(Ugust)));
        S.setArrow('wind', [0, U, 0], 0.15);
        const lift = L.some(x => x <= 0);
        H.set([['Phase', t < 6 ? 'Exploded assembly & mass map' : 'Stationary crosswind sweep'], ['Operating mass', fmt(mOp, 4) + ' kg'], ['CG height', fmt(cgHeight, 3) + ' m'], ['Crosswind', fmt(U, 3) + ' m/s'], ['Wing state', fe ? 'FEATHERED (lock released)' : 'locked, loaded', fe ? '#f59e0b' : undefined], ['Side force on vehicle', fmt(Fside, 4) + ' N'], ...lay.pts.map((q, k2) => [q[2] + ' load', fmt(L[k2], 4) + ' N', L[k2] <= 0 ? '#ef4444' : undefined]), ['Tip-over wind (solved)', feather ? fmt(tipFeather, 3) + ' m/s feathered' : fmt(tipU, 3) + ' m/s'], ['Status', lift ? 'WHEEL LIFT — tips over' : 'all wheels loaded', lift ? '#ef4444' : '#22c55e']]);
      } });
      el.append(h('div', { class: 'row gap wrap', style: 'margin:6px 2px' }, magLab, magIn), h('p', { class: 'small muted' }, `Solved: component masses and CG from the mass budget (part colours, yellow CG), statically determinate three-wheel loads under the wing side force at each crosswind speed (green arrows, red when a wheel unloads), tip-over wind ${fmt(tipU, 3)} m/s. Calculated: body roll from tyre radial compliance (150 kN/m), magnified for visibility. The exploded phase is a presentation of the assembly (not a physical motion).`));
    } });
    out.views.push({ label: '2D animation: load-transfer triangle in crosswind', icon: '🔺', render: async (el) => {
      const A = await import('../core/anim2d.js');
      const Umax2 = Math.max(p.designGust * 1.1, 5), sideAt = (U) => { const F = 0.5 * rhoA * U * U * p.wingArea * 1.2; if (feather && F > FySafe) { const Fw = 0.5 * rhoA * U * U * p.wingArea * 0.02, Fb = 0.5 * rhoA * U * U * Abody * 0.8; return { F: Fw + Fb, z: (Fw * hs + Fb * hBody) / (Fw + Fb), fe: true }; } return { F, z: hs, fe: false }; };
      const Us = linspace(0, Umax2, 200), Ls = Us.map(U => { const sd = sideAt(U); return { ...sd, L: threeWheelLoads(lay.pts, W, 0, [{ Fy: -sd.F, z: sd.z }]) }; });
      A.anim2d(el, { duration: 20, rate: 1, height: 440, label: 'Sweep', unit: 's', draw: (g, t, Wd, H) => {
        const U = Umax2 * Math.min(1, t / 18), k = Math.min(199, Math.round(U / Umax2 * 199)), st = Ls[k];
        const pts = lay.pts, xs = pts.map(q => q[0]), ys = pts.map(q => q[1]), sc = Math.min((Wd * 0.55 - 60) / (Math.max(...xs) - Math.min(...xs) || 1), (H - 80) / (p.track * 1.6));
        const ox = 40 + (0 - Math.min(...xs)) * sc, oy = H / 2, P = (x, y) => [ox + x * sc, oy - y * sc];
        g.fillStyle = A.theme().bg2; g.fillRect(12, 12, Wd * 0.55, H - 24);
        g.strokeStyle = '#94a3b8'; g.lineWidth = 2; g.beginPath(); pts.forEach((q, j) => { const [a1, b1] = P(q[0], q[1]); j ? g.lineTo(a1, b1) : g.moveTo(a1, b1); }); g.closePath(); g.stroke(); g.lineWidth = 1;
        // tipping axis (windward wheel lifts first: wind from −y pushes toward +y)
        const Wmax = W * 0.8;
        pts.forEach((q, j) => { const [a1, b1] = P(q[0], q[1]), Lj = st.L[j], r = 6 + 28 * Math.max(0, Lj) / Wmax; g.fillStyle = Lj <= 0 ? '#ef4444' : '#22c55e'; g.globalAlpha = 0.75; g.beginPath(); g.arc(a1, b1, r, 0, 7); g.fill(); g.globalAlpha = 1; const lab = `${q[2]} ${fmt(Lj, 4)} N`; A.text(g, lab, a1 + r + 4 + g.measureText(lab).width > Wd * 0.55 ? a1 - r - 4 - g.measureText(lab).width : a1 + r + 4, b1 + 4, Lj <= 0 ? '#ef4444' : undefined); });
        // CG and ground resultant (weight + side-force moment)
        const [gx, gy] = P(0, 0); g.fillStyle = '#facc15'; g.beginPath(); g.arc(gx, gy, 6, 0, 7); g.fill(); A.text(g, 'CG', gx + 8, gy - 6, '#facc15');
        const yr = -st.F * st.z / W, [rx, ry] = P(0, yr); A.arrow(g, gx, gy, rx - gx, ry - gy, '#ef4444', 2.5); g.strokeStyle = '#ef4444'; g.beginPath(); g.arc(rx, ry, 5, 0, 7); g.stroke(); A.text(g, 'resultant', rx + 8, ry - 6, '#ef4444');
        A.arrow(g, gx - 60, gy - 110, 0, Math.min(100, st.F / 8), '#38bdf8', 3, `side force ${fmt(st.F, 4)} N (toward −y)`);
        // right: bars and status
        const bx = 24 + Wd * 0.55, bw = Wd - bx - 12;
        A.text(g, `Crosswind ${fmt(U, 3)} m/s`, bx, 30, undefined, 'bold 14px Inter,system-ui,sans-serif');
        A.text(g, feather ? (st.fe ? 'Wing FEATHERED (lock released)' : 'Wing locked, loaded') : 'Fixed wing root', bx, 50, st.fe ? '#f59e0b' : '#22c55e');
        pts.forEach((q, j) => A.bar(g, { x: bx, y: 64 + j * 22, w: bw, value: Math.max(0, st.L[j]), max: W * 0.8, label: q[2], unit: ' N', color: st.L[j] <= 0.1 * W / 3 ? '#ef4444' : '#22c55e' }));
        A.seriesPanel(g, { x: bx, y: 140, w: bw, h: H - 160, T: Us, t: U, title: 'Wheel loads vs crosswind', unit: 'N', series: pts.map((q, j) => ({ name: q[2], y: Ls.map(o => o.L[j]), color: ['#22c55e', '#38bdf8', '#f59e0b'][j] })), hlines: [{ y: 0, label: 'wheel lift', color: '#ef4444' }] });
      }, explain: `Top view of the three-wheel support triangle as the stationary car is swept through crosswinds up to ${fmt(Umax2, 3)} m/s: circle size = solved normal load (red = lifted), red arrow = lateral shift of the ground resultant (side force × height ÷ weight). ${feather ? `At ${fmt(Urelease, 3)} m/s the breakaway releases and the feathered wing's side force collapses.` : ''}` });
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
    const patch = { layout: p.layout, length: p.length, width: p.width, height: p.height, wheelbase: p.wheelbase, track: p.track, wheelRadius: p.wheelRadius, wingArea: p.wingArea, wingHeight: p.wingHeight, wingReefSpan: Math.min(p.wingHeight, p.wingReefSpan), wingReef: isArch ? 'none' : p.wingState, wingConfig: p.rig, archCant: p.archCant, archSweep: p.archSweep, bodyShape: p.bodyShape, spats: !!p.spats, nWings: +p.nWings, mass: +mOp.toFixed(2), payload: p.payload, Crr: p.Crr, CdA: p.CdA, cgHeight: +cgHeight.toFixed(4), driveWheels: +p.driveWheels, frontFrac: +frontFrac.toFixed(4) };
    const changed = Object.keys(patch).some(kk => (typeof patch[kk] === 'number' ? Math.abs((ctx.vehicle[kk] ?? NaN) - patch[kk]) > 1e-6 : ctx.vehicle[kk] !== patch[kk]));
    if (changed) ctx.store.setVehicle(patch, 'Suite 01');
    out.headline = `${p.layout === 'tadpole' ? 'Tadpole' : 'Delta'} trike: computed ${fmt(total, 4)} kg (target ${p.targetMass} kg), CG ${fmt(cgHeight, 3)} m high, SSF ${fmt(ssf, 3)} g, tip-over crosswind ${fmt(tipU, 3)} m/s.${changed ? ' Global vehicle updated — dependent suites marked stale.' : ''}`;
    out.headlineKind = tipU < p.designGust || ssf < 0.75 ? 'warn' : 'info';
    const massByComponent = {}; for (const it of items) if (it.comp) massByComponent[it.comp] = (massByComponent[it.comp] || 0) + it.m;
    out.outputs = { massByComponent, mass: mOp, massComputed: total, cg, inertia: { Ixx: Ixx * scale, Iyy: Iyy * scale, Izz: Izz * scale, Ixy: Ixy * scale, Ixz: Ixz * scale, Iyz: Iyz * scale }, wheelLoads: loads, wheelLabels: lay.pts.map(q => q[2]), wheelbase: p.wheelbase, track: p.track, cgHeight, ssf, frontFrac, tipOverWind: tipU, massBreakdown: Object.fromEntries(items.map(i => [i.name, i.m])), layout: p.layout, wingRoot: p.wingRoot, FySafe_N: FySafe, releaseWind: Urelease, tipOverLocked: tipU, tipOverFeathered: tipFeather, tipOverStowed: tipStowed, bodySideArea: Abody, hCE: hs, overallHeight: overallH, reefedHeight: reefedH, stowedHeight: stowedH, tipOverReefed: tipUR, FySafeReefed_N: FySafeR, hCEReefed: hsR, areaReefed: areaR, wingState: p.wingState, rig: isArch ? 'arch' : 'tandem', arch: isArch ? { ...AR4, sideCoeff: Csd, hCE: hs } : null, skin_mm: p.skin, core_mm: p.core, foldedHeight: isArch ? stowedH : null, stowage: p.stowage, wingArealMass: wingAreal };
    void groupMass;
    return out;
  },
};
