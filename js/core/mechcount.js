// Moving-part and mechanism count under ONE written rule, applied identically to every design (Suite 01).
export const COUNT_RULE = 'Moving part: every discrete part that moves relative to the body, or relative to the part that carries it, in normal use (driving, braking, parking, selecting a lever position); each cable or tendon strand is one part. Mechanism: one actuation chain — all the moving parts driven by one input (a wheel turning, the tiller, the brake pedal, a driver lever, the buffer release, the wind on a pivot). Not counted: elastic deformation of fixed structure (flexures, pad flexure arms, skins, kirigami panels, the pylon rod cage, detent leaves, bimetal strips) and fixed stops. The same rule is applied to the earlier designs’ reef joints, feathering roots, tilt hinges and ring track.';

// f: feature set of one design (see featuresOf)
export function mechInventory(f) {
  const L = [], add = (item, m, n) => L.push([item, m, n]);
  add(`Road wheels (${f.wheels} wheels, bearings)`, f.wheels, f.wheels);
  if (f.tiller) add('Direct tiller steering: tiller, 2 kingpin uprights, 2 tie rods (no bellcrank)', 1, 5); else add('Steering linkage: tiller, bellcrank, 2 kingpin uprights, 2 tie rods', 1, 6);
  add(`Service brakes: pedal, master cylinder, front-circuit pressure limiter, 3 calipers${f.parkLockOnPedal ? ', parking lock valve on the master cylinder' : ''}`, 1, f.parkLockOnPedal ? 7 : 6);
  if (!f.parkLockOnPedal) add('Parking brake: lever + cable (elastic detent gate)', 1, 2);
  for (let i = 0; i < f.driveHubs; i++) add(`Drive hub ${i + 1}: ${f.cpw} NiTi cartridges + ${2 * f.cpw} rocking cam pads (flexure arms elastic, not counted)`, 1, 3 * f.cpw);
  const sailParts = f.sail ? 2 * f.driveHubs : 0; // per hub: one bistable lifter ring (snap-through; holds the pads off in its second state) + cable branch
  if (f.lever) add(`Driver lever chain, one 4-position gate drive / sail / depower / park: lever, main cable${f.sail ? `, ${f.driveHubs} hub branches + ${f.driveHubs} bistable lifter rings` : ''}${f.tendons ? `, ${f.tendons} tendon tail${f.tendons > 1 ? 's' : ''}` : ''}${f.returnCord ? ', series-strand return cord' : ''}${f.wedges ? `, ${f.wedges} park wedge${f.wedges > 1 ? 's' : ''}` : ''}`, 1, 2 + sailParts + f.tendons + (f.returnCord ? 1 : 0) + f.wedges);
  else if (f.sail) add(`Hub sail lever chain (drive / sail): lever, main cable, ${f.driveHubs} hub branches + ${f.driveHubs} bistable lifter rings`, 1, 2 + sailParts);
  if (f.buffer) add('Elastic-buffer trigger', 1, 1);
  if (f.regen) add('Regenerative linear-index coupling: 2 flexure pawls (switched by the pedal), 2 push-pull cables, linear carriage (rack fixed)', 1, 5);
  if (f.thermalStore) { add('Thermal-store discharge: driver lever + evaporator saddle clamp', 1, 2); add('Collector glazing vent flap (bimetal-driven)', 1, 1); }
  if (f.autoLatch) add('Optional automatic bimetal hub latches (research option)', 2, 2);
  if (f.windHarvester) add('Parked-wind click harvester (research option)', 1, 2);
  if (f.root === 'ring') add('Arch ring track + breakaway lock (wind input)', 1, 2);
  if (f.root === 'feather') for (let w = 0; w < f.nW; w++) add(`Wing ${w + 1} feathering pivot + breakaway lock (wind input)`, 1, 2);
  if (f.fold) add('Arch fold hinge + latch', 1, 2);
  if (f.tilt) for (let w = 0; w < f.nW; w++) add(`Wing ${w + 1} tilt hinge + latch (driver input)`, 1, 2);
  if (f.reef) for (let w = 0; w < f.nW; w++) add(`Wing ${w + 1} telescoping reef section + latch (driver input)`, 1, 2);
  if (f.practical) for (let w = 0; w < f.nW; w++) add(`Wing ${w + 1} adjustable incidence + unloading`, 1, 1);
  if (f.root === 'flexure') add(f.arch ? 'Arch flexure pylon (elastic rod cage; fixed EPDM end stops)' : 'Wing flexure roots (elastic)', 0, 0);
  if (f.vents) add('Kirigami relief panels (elastic skin parts)', 0, 0);
  return { list: L, mechanisms: L.reduce((a, r) => a + r[1], 0), movingParts: L.reduce((a, r) => a + r[2], 0) };
}

// features of a design from its vehicle and Suite 01 parameters (+ Suite 17 / 18 state)
export function featuresOf(v, p = {}, o = {}) {
  const arch = (v.wingConfig || 'arch') === 'arch', lever = !!(arch && v.vents && v.unloadingLever);
  const root = arch ? ((p.wingRoot ?? ((v.archRoot || 'flexure') === 'flexure' ? 'washout' : 'ring')) === 'washout' ? 'flexure' : 'ring') : (p.wingRoot === 'feathering' ? 'feather' : 'flexure');
  return { arch, wheels: 3, tiller: !!v.directTiller, parkLockOnPedal: !!v.parkLockOnPedal, driveHubs: +(p.driveWheels ?? v.driveWheels ?? 2), cpw: +(p.cartPerWheel ?? 24), sail: o.sail ?? true, lever,
    tendons: lever ? (v.tendonSeries ? 0 : 3) : 0, returnCord: lever && !!v.tendonSeries && (v.strandReturn ?? (v.returnInPanel ? 'panel' : 'cord')) === 'cord', /* a separate cord counts; moulded into the far panel edge rib it is elastic panel structure */ wedges: lever ? (v.parkWedge === 'forked' ? 1 : v.parkWedge === 'none' ? 0 : 2) : 0, buffer: (p.bufferE ?? 1) > 0, regen: o.regen ?? true, thermalStore: !!v.thermalStore,
    autoLatch: !!v.hubAutoLatch, windHarvester: !!v.windHarvester, root, fold: p.stowage === 'fold', tilt: p.stowage === 'tilt', reef: !arch && (p.wingReefSpan ?? v.wingReefSpan ?? 1) < (p.wingHeight ?? v.wingHeight ?? 1) - 1e-6,
    practical: p.wingConfig === 'practical' || v.wingConfig === 'practical', nW: arch ? 1 : (v.nWings || 2), vents: !!(arch && v.vents) };
}

// the head-to-head designs (same definitions as the comparison runs); Revision 5.1 is evaluated live from the current vehicle
export const EARLIER_DESIGNS = [
  ['Revision 5 (flexure arch, L 4.6)', { wingConfig: 'arch', archRoot: 'flexure' }, { wingRoot: 'washout' }],
  ['Revision 4.1 AeroArch (ring track)', { wingConfig: 'arch', archRoot: 'ring' }, { wingRoot: 'ring' }],
  ['Revision 3 tandem (3 wings)', { wingConfig: 'tandem', nWings: 3, wingReefSpan: 2, wingHeight: 2 }, { wingRoot: 'feathering', stowage: 'tilt' }],
  ['Revision 2.2 reefable', { wingConfig: 'tandem', nWings: 2, wingReefSpan: 2, wingHeight: 3 }, { wingRoot: 'feathering', stowage: 'tilt' }],
  ['Specification original (2 × 3 m wings)', { wingConfig: 'tandem', nWings: 2, wingReefSpan: 3, wingHeight: 3 }, { wingRoot: 'feathering', stowage: 'tilt' }]];
