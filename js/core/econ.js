// Engineering economics & life-cycle sustainability: cost estimating relationships, learning curves,
// discounted cash flow (NPV/IRR/payback), life-cycle cost, cradle-to-gate LCA inventory from the
// materials registry, end-of-life credits and a simplified Material Circularity Indicator.
import { COMPONENTS, componentMaterial, get as getMat } from './materials.js';
import { brent } from './num.js';
import { pcmById } from './pcmstore.js';

// Reference component envelopes (volume of material at default design, processing energy, labour, life).
// Volumes are sized so the default material gives a plausible prototype mass; changing material keeps the
// volume (first-order estimate) — stiffness-equivalent resizing is done in the structural suites.
export const COMPONENT_REF = {
  chassis: { vol: 0.0258, qty: 1, kWhPerKg: 9, laborH: 120, factor: 2.9, life: 20, process: 'Out-of-autoclave vacuum-bag-only prepreg (recycled-fibre plies), oven cure 120–135 °C — no autoclave', basis: 'Rev 6.2: OOA vacuum-bag-only prepregs reach < 1 % voids at oven pressure; ≈ 60 % less cure energy and no autoclave tooling vs 180 °C / 6 bar autoclave' },
  chassisCore: { vol: 0.040, qty: 1, kWhPerKg: 4, laborH: 12, factor: 1.8, life: 20, process: 'CNC-contoured foam core, bonded in co-cure' },
  spar: { vol: 0.0077, qty: 1, kWhPerKg: 9, laborH: 40, factor: 2.9, life: 15, process: 'Tailored-angle out-of-autoclave prepreg box spar, matched-die oven cure', basis: 'Rev 6.2: same vacuum-bag-only out-of-autoclave route as the monocoque (oven cure, no autoclave)' },
  rib: { vol: 0.0030, qty: 1, kWhPerKg: 12, laborH: 24, factor: 3.0, life: 15, process: 'Compliant ribs: GFRP press-moulding or polypropylene powder-bed printing' },
  wingSkin: { vol: 0.016, qty: 1, kWhPerKg: 6, laborH: 30, factor: 2.2, life: 10, process: 'Reinforced elastomer skin laminated & thermoformed, fluoropolymer UV top film', basis: 'Rev 6.2: fluoropolymer-faced elastomer skins keep ≥ 10 years outdoor UV exposure; the morphing strain is far below the elastomer fatigue limit' },
  leadingEdge: { vol: 0.0019, qty: 1, kWhPerKg: 22, laborH: 12, factor: 3.5, life: 15, process: 'Thin CFRP shell, female-mould cure' },
  lattice: { vol: 0.0018, qty: 1, kWhPerKg: 15, laborH: 12, factor: 3.5, life: 12, process: 'Polypropylene cellular core: powder-bed fused (prototype) or injection-moulded cell sheets (series), heat-staked', basis: 'Rev 6.2: polymer (PP) lattice constituent — the former metal powder-bed + HIP route (55 kWh/kg, ×6) does not apply; PP fusion/moulding ≈ 15 kWh/kg' },
  rootInsert: { vol: 0.00068, qty: 1, kWhPerKg: 8, laborH: 10, factor: 2.8, life: 20, process: '5-axis CNC machining of forged billet' },
  collector: { vol: 0.004, qty: 1, kWhPerKg: 12, laborH: 10, factor: 1.6, life: 10, process: 'PVD selective coating on aluminium substrate' },
  radiator: { vol: 0.0012, qty: 1, kWhPerKg: 10, laborH: 6, factor: 1.6, life: 10, process: 'Photonic film coating / lamination under the same fluoropolymer top film', basis: 'Rev 6.2: same UV top film as the collector (10-year class)' },
  spreader: { vol: 0.0022, qty: 1, kWhPerKg: 3, laborH: 6, factor: 2.0, life: 20, process: 'Sheet stamping, brazed thermal joints' },
  heatpipe: { vol: 0.00067, qty: 1, kWhPerKg: 8, laborH: 4, factor: 2.5, life: 15, process: 'Sintered wick, evacuation, charging, sealing' },
  insulator: { vol: 0.0067, qty: 1, kWhPerKg: 15, laborH: 3, factor: 1.4, life: 12, process: 'Die-cut blanket, encapsulated' },
  actuator: { vol: 0.00037, qty: 1, kWhPerKg: 40, laborH: 30, factor: 2.2, life: 3, process: 'VIM melting, drawing/L-PBF, shape setting 500 °C, training cycles' },
  cartridgeHousing: { vol: 0.0009, qty: 1, kWhPerKg: 50, laborH: 16, factor: 4.0, life: 12, process: 'L-PBF titanium housings, machined datums' },
  cam: { vol: 0.00064, qty: 1, kWhPerKg: 8, laborH: 12, factor: 3.0, life: 20, process: 'CNC profile milling, H1025 ageing, profile grinding', basis: 'Rev 6.2: Hertz contact ≈ 0.12–0.35 GPa vs 1.8 GPa allowable (Suite 17) — well inside rolling-contact endurance for the 20-year life' },
  flexure: { vol: 0.0003, qty: 1, kWhPerKg: 6, laborH: 4, factor: 1.6, life: 12, process: 'Arch pylon torsion-rod cage from bar stock: rods cut to length, centreless-ground, aged and shot-peened, ends terminated per Suite 09 (default cold-swaged Ti ferrule sockets); laser-cut end plates — no machining from billet', basis: 'Rev 6.5 cost basis (same rates as every bar-stock part): bar-stock rods with rolled threads need ≈ 4 h (cut, grind, roll, age, peen, assemble) and a ×1.6 material-to-part factor, vs ×3 / 10 h for parts machined from billet. Life:  the arch pylon rods are designed to SF ≥ 2 in torsional fatigue (Suites 09 / 22) — inspected, not replaced, within 12 years' },
  buffer: { vol: 0.0012, qty: 1, kWhPerKg: 10, laborH: 14, factor: 3.0, life: 8, process: 'Wire-EDM bistable cells, ageing' },
  wheel: { vol: 0.0032, qty: 1, kWhPerKg: 6, laborH: 9, factor: 2.6, life: 15, process: 'Forged & machined rims' },
  tyre: { vol: 0.0052, qty: 1, kWhPerKg: 3, laborH: 0.6, factor: 4.0, life: 3, process: 'Purchased low-rolling-resistance tyres' },
  bearing: { vol: 0.000154, qty: 1, kWhPerKg: 10, laborH: 0.5, factor: 12, life: 15, process: 'Purchased sealed bearings', basis: 'Rev 6.2: L10 life at ≈ 0.4–0.8 kN wheel load and < 300 rpm exceeds 10⁴ h ≫ ≈ 250 h/year of driving; sealed-for-life grease' },
  brake: { vol: 0.0005, qty: 1, kWhPerKg: 3, laborH: 3, factor: 2.5, life: 15, process: 'Cast & machined discs (recycled-iron melt)', basis: 'Rev 6.2: a 225 kg vehicle at ≤ 40 km/h puts ≈ 14 kJ per stop into 220 mm discs — disc wear is negligible; pads remain the consumable' },
  brakePad: { vol: 0.00026, qty: 1, kWhPerKg: 4, laborH: 0.5, factor: 5, life: 2, process: 'Purchased friction pads' },
  steering: { vol: 0.00064, qty: 1, kWhPerKg: 4, laborH: 8, factor: 2.2, life: 20, process: 'Direct tiller: welded 4130 tube, two rod ends, no bellcrank (Rev 6.2)', basis: 'Rev 6.2: direct tiller removes the bellcrank, its pivot and two rod ends' },
  cabin: { vol: 0.0042, qty: 1, kWhPerKg: 5, laborH: 10, factor: 2.0, life: 12, process: 'Thermoformed polycarbonate canopy, hard-coated' },
};

// Bill of materials with mass, cost and cradle-to-gate impacts. grid: gCO2/kWh for processing electricity.
export function bom(store, { grid = 450, labourRate = 35 } = {}) {
  return Object.entries(COMPONENTS).map(([key, c]) => {
    // the arch pylon cage ('flexure') is priced in the rod material chosen in Suite 09 (pylonMat), the same basis for every design
    const m = key === 'flexure' && store.params?.('s09')?.pylonMat ? (getMat(store.params('s09').pylonMat) || componentMaterial(store, key)) : componentMaterial(store, key), ref = COMPONENT_REF[key] || { vol: 0.001, kWhPerKg: 5, laborH: 2, factor: 2, life: 10, process: '—' };
    // masses come from the Suite 01 mass budget of the design actually configured; the reference volume is only a fallback
    // 'flexure' is only the arch pylon cage (hub flexures are booked in cartridgeHousing), so it is zero when Suite 01 has none
    // Rev 6.2: Suite 01 books the whole buffer system (cells + cages + box + mounts) as 'buffer'; cages and box have their own rows, so only the cells are priced here
    const mb = store.output('s01')?.massByComponent, massRef = (m?.rho || 1000) * ref.vol, o18b = key === 'buffer' && mb ? store.output?.('s18') : null, cells18 = o18b?.bufferMass_kg, mass = o18b && o18b.cellType === 'elastomer' ? 0 : cells18 ? Math.min(mb.buffer ?? cells18, cells18) : mb ? (mb[key] ?? (key in mb || key === 'flexure' ? 0 : massRef)) : massRef, matCost = mass * (m?.cost || 5), procKWh = mass * ref.kWhPerKg;
    const rf = recycledFactor(store, key, m), matCostR = matCost * rf.cost;
    // pylon cage labour scales with the rod count from Suite 09 (≈ 0.3 h per bar-stock rod + 1.5 h assembly)
    // rib labour scales with the rib count Suite 06 publishes for the configured wing (≈ 2.5 h per rib + 2 h set-up; + 2 h for the
    // arch root plate, which is booked with the ribs) — the same rate for every design; without the count the reference 24 h is kept
    const p09 = key === 'flexure' ? store.params?.('s09') ?? {} : null, nRods = p09 ? (p09.cageN ?? store.project?.vehicle?.pylonRodN ?? 8) * (p09.cageStages ?? store.project?.vehicle?.pylonStages ?? 1) : 0;
    // rod preparation ≈ 0.2 h each (cut, grind, age, peen) + terminations (2 per rod): cold-swaged Ti ferrule ($3 + 0.05 h), machined clamp socket (0.25 h), or upset end + rolled M14 thread ($4 + nut $0.5 + 0.05 h)
    const rodEnd = p09 ? (p09.rodEnd ?? store.output?.('s09')?.rodEnd ?? 'swaged') : null, TERM = { swaged: [3, 0.05], machined: [0, 0.25], rolled: [4.5, 0.05] }[rodEnd] || [3, 0.05];
    const nRib = key === 'rib' ? store.output?.('s06')?.ribCount : null;
    const labH = p09 ? 1.5 + 0.2 * nRods + 2 * nRods * TERM[1] : Number.isFinite(nRib) ? 2 + 2.5 * nRib + (store.project?.vehicle?.wingConfig === 'arch' ? 2 : 0) : ref.laborH, termCost = p09 ? 2 * nRods * TERM[0] : 0;
    const fabCost = mass > 0 ? matCostR * ref.factor + labH * labourRate + termCost : 0; // a component the design does not have (zero mass) carries no labour either
    const co2Mat = mass * (m?.co2 || 5) * rf.co2, co2Proc = procKWh * grid / 1000, energyMat = mass * (m?.energy || 80) * rf.energy;
    const recyc = m?.recyclable ?? 0.3, eolCredit = recyc * co2Mat * 0.6;
    // Revision 6 owner-cost actions: NiTi cartridge life from the Suite 15 fatigue design (strain amplitude set for the
    // required life) or Suite 23; buffer-cell life from the Suite 18 Basquin estimate — both capped at the 20-year structure life.
    const s23l = key === 'actuator' ? Math.max(store.output?.('s23')?.cartridgeLife_years || 0, store.output?.('s15')?.lifeYearsAchieved || 0) : key === 'buffer' ? store.output?.('s18')?.fatigueLife_years : null, life = s23l && Number.isFinite(s23l) && s23l > ref.life ? Math.min(20, Math.floor(s23l)) : s23l === Infinity ? 20 : ref.life; // NiTi life extended only when Suite 23 fatigue (at the 1 % strain amplitude) justifies it
    return { lifeBasis: ref.basis, key, name: c.name, material: (m?.name || '—') + (rf.note ? ` (${rf.note})` : ''), matId: m?.id, family: m?.family, mass, massSource: mb?.[key] !== undefined ? 'Suite 01 mass budget' : 'reference volume', matCost: matCostR, fabCost, laborH: mass > 0 ? labH : 0, procKWh, co2Mat, co2Proc, co2: co2Mat + co2Proc, energyMat, energy: energyMat + procKWh * 3.6, recyc, eolCredit, life, lifeSource: s23l && life !== ref.life ? 'Suite 23 fatigue' : 'reference', process: ref.process, unc: m?.unc ?? 20 };
  }).concat(reefMechanismRow(store, { grid, labourRate }), aeroFairingRows(store, { grid, labourRate }), rev51Rows(store, { grid, labourRate }), rev6Rows(store, { grid, labourRate }))
    // Rev 6.2 audit: these parts are already booked by Suite 01 inside its component lines (spats, pods, sills → chassis;
    // crossbeam → spar; pylon rod cage → flexure; nose cone → chassisCore). Counting them again here double-charged them.
    .filter(r => !(store.output?.('s01')?.massByComponent && ['spats', 'fairings', 'sideSills', 'archCrossbeam', 'archPylon', 'noseCone'].includes(r.key)));
}
// Revision 2.2: telescoping reef joints + torque-limited breakaway locks for both wings (≈ 3.5 kg per vehicle):
// machined Ti-6Al-4V sleeves and latch, PTFE-lined bushings, stainless detent springs.
function reefMechanismRow(store, { grid, labourRate }) {
  if ((store.project?.vehicle?.wingConfig || 'tandem') === 'arch') return archMechanismRows(store, { grid, labourRate });
  const m = componentMaterial(store, 'rootInsert') || { name: 'Ti-6Al-4V', rho: 4430, cost: 30, co2: 40, energy: 650, recyclable: 0.6, unc: 15 };
  const v = store.project.vehicle, nW = v.nWings || 2, reefable = (v.wingReefSpan ?? v.wingHeight) < v.wingHeight - 1e-6;
  const mass = reefable ? 3.5 : 0.7 * nW, matCost = mass * (m.cost || 30), procKWh = mass * 35, fabCost = matCost * 4 + (reefable ? 8 : 2 * nW) * labourRate;
  const co2Mat = mass * (m.co2 || 40), co2Proc = procKWh * grid / 1000, energyMat = mass * (m.energy || 650), recyc = m.recyclable ?? 0.6;
  return [{ key: 'reefJoint', name: reefable ? `Wing telescoping reef joints & breakaway locks (${nW} wings)` : `Wing feathering pivots & breakaway locks (${nW} wings)`, material: reefable ? `${m.name} sleeves, PTFE bushings` : `${m.name} pivot housings, friction discs`, matId: m.id, family: m.family, mass, matCost, fabCost, laborH: 8, procKWh, co2Mat, co2Proc, co2: co2Mat + co2Proc, energyMat, energy: energyMat + procKWh * 3.6, recyc, eolCredit: recyc * co2Mat * 0.6, life: 8, process: 'CNC machining, hard anodise / PTFE liner bonding, assembly & proof test', unc: 25 }];
}

// Discounted cash flow
export const npv = (rate, flows) => flows.reduce((s, f, t) => s + f / (1 + rate) ** t, 0);
export function irr(flows) { const f = (r) => npv(r, flows); for (const [a, b] of [[-0.9, 0], [0, 0.5], [0.5, 3], [3, 20]]) { if (f(a) * f(b) <= 0) { const r = brent(f, a, b, 1e-10); if (r.ok) return r.x; } } return NaN; }
export function payback(flows) { let c = 0; for (let t = 0; t < flows.length; t++) { const p = c; c += flows[t]; if (c >= 0 && t > 0) return t - 1 + (-p) / (flows[t] || 1); } return Infinity; }
export function discountedPayback(rate, flows) { return payback(flows.map((f, t) => f / (1 + rate) ** t)); }
export const realRate = (nominal, inflation) => (1 + nominal) / (1 + inflation) - 1;
// Wright learning curve: cost of unit n given first-unit cost T1 and progress ratio pr (e.g. 0.85)
export const unitCost = (T1, pr, n) => T1 * n ** (Math.log(pr) / Math.LN2);
export const avgCost = (T1, pr, N) => { let s = 0; const b = Math.log(pr) / Math.LN2; if (N <= 2000) { for (let n = 1; n <= N; n++) s += n ** b; return T1 * s / N; } return T1 * ((N + 0.5) ** (b + 1) - 0.5 ** (b + 1)) / ((b + 1) * N); };
// Life-cycle cost (present value) for an owner
export function lcc({ purchase, years, rate, annualOps = 0, replacements = [] }) {
  let pv = purchase; const rows = [{ year: 0, cost: purchase, pv: purchase }];
  for (let y = 1; y <= years; y++) { let c = annualOps; for (const r of replacements) if (r.interval > 0 && y % Math.round(r.interval) === 0 && y < years) c += r.cost; const d = c / (1 + rate) ** y; pv += d; rows.push({ year: y, cost: c, pv: d }); }
  return { pv, rows, annualised: pv * rate / (1 - (1 + rate) ** -years) };
}
// Simplified Ellen-MacArthur Material Circularity Indicator for one product
export function mci({ mass, recycledIn = 0.1, collectedOut = 0.5, efficiency = 0.8, lifeRatio = 1 }) {
  // V: virgin feedstock, W: unrecoverable waste, LFI: linear flow index, F(X)=0.9/X utility factor
  const V = mass * (1 - recycledIn), W = mass * (1 - collectedOut * efficiency);
  const LFI = (V + W) / (2 * mass), F = 0.9 / Math.max(0.1, lifeRatio);
  return Math.max(0, Math.min(1, 1 - LFI * F));
}

// Lifting-body revision: aerodynamic wheel spats (3) and wing-root fairings (2) in the chassis skin material.
function aeroFairingRows(store, { grid, labourRate }) {
  const m = componentMaterial(store, 'chassis') || { name: 'CFRP woven', rho: 1550, cost: 45, co2: 24, energy: 290, recyclable: 0.25, unc: 15 };
  const arch = (store.project?.vehicle?.wingConfig || 'tandem') === 'arch'; // Rev 6.2: pods/fairings co-moulded with the crossbeam and endplate (one tool, one cure; trim only)
  return [['spats', 'Aerodynamic wheel spats (3)', 4.2, 10], ['fairings', arch ? 'Shoulder pods / root fairings (co-moulded with the crossbeam and endplate)' : 'Wing-root fairings (2)', 1.6, arch ? 1 : 5]].map(([key, name, mass, laborH]) => {
    const rf = recycledFactor(store, key, m), matCost = mass * (m.cost || 40) * rf.cost, procKWh = mass * 12, fabCost = matCost * 2.5 + laborH * labourRate, co2Mat = mass * (m.co2 || 20) * rf.co2, co2Proc = procKWh * grid / 1000, energyMat = mass * (m.energy || 250) * rf.energy, recyc = m.recyclable ?? 0.25;
    return { key, name, material: m.name + (rf.note ? ` (${rf.note})` : ''), matId: m.id, family: m.family, mass, matCost, fabCost, laborH, procKWh, co2Mat, co2Proc, co2: co2Mat + co2Proc, energyMat, energy: energyMat + procKWh * 3.6, recyc, eolCredit: recyc * co2Mat * 0.6, life: 12, process: 'Vacuum infusion / prepreg moulding on the body tooling', unc: 20 };
  });
}

// Revision 4 AeroArch: ring track with incidence drive and torque-limited breakaway lock, plus the fold hinge and latch.
function archMechanismRows(store, { grid, labourRate }) {
  const veh = store.project?.vehicle || {};
  if (veh.archRoot !== 'ring') { // Revision 5.1: one centreline titanium flexure pylon (±vehicle.yawLimit elastic yaw) under a CFRP root crossbeam — no rotating parts
    const fm = componentMaterial(store, 'flexure') || { name: 'Titanium β-21S', rho: 4940, cost: 45, co2: 42, energy: 680, recyclable: 0.5, unc: 15 }, cf = componentMaterial(store, 'spar') || { name: 'CFRP', cost: 38, co2: 24, energy: 290, recyclable: 0.25 };
    const row = (key, name, mat, mass, laborH, kWhkg, factor, process, rfac = { co2: 1, energy: 1, cost: 1, note: '' }) => { const matCost = mass * (mat.cost || 30) * rfac.cost, procKWh = mass * kWhkg, co2Mat = mass * (mat.co2 || 20) * rfac.co2, co2Proc = procKWh * grid / 1000, energyMat = mass * (mat.energy || 300) * rfac.energy, recyc = mat.recyclable ?? 0.4; return { key, name, material: mat.name + (rfac.note ? ` (${rfac.note})` : ''), matId: mat.id, family: mat.family, mass, matCost, fabCost: matCost * factor + laborH * labourRate, laborH, procKWh, co2Mat, co2Proc, co2: co2Mat + co2Proc, energyMat, energy: energyMat + procKWh * 3.6, recyc, eolCredit: recyc * co2Mat * 0.6, life: 12, process, unc: 25 }; };
    return [row('archPylon', `Arch centreline flexure pylon (titanium rod cage, ±${veh.yawLimit ?? 35}° yaw)`, fm, 1.9, 10, 45, 5, 'Wire-EDM from β-21S plate, chemical milling, shot peening, ageing; machined interfaces', recycledFactor(store, 'flexure', fm)),
            row('archCrossbeam', 'Arch root crossbeam (CFRP box) with co-moulded endplate and pod skins', cf, 1.4, 7, 9, 2.9, 'One out-of-autoclave moulding: box beam, root endplate and pod skins co-cured with Ti inserts (Rev 6.2 consolidation: one tool, one cure, no bonding or fasteners between them)')];
  }
  const m = componentMaterial(store, 'rootInsert') || { name: 'Ti-6Al-4V', rho: 4430, cost: 30, co2: 40, energy: 650, recyclable: 0.6, unc: 15 };
  return [['archRing', 'Arch ring track, slewing bearing, incidence drive & breakaway lock — legacy (non-compliant: rotating bearing/drive)', 6.0, 14, 40, 'Ring rolling / CNC, induction-hardened raceway, crossed-roller bearing assembly, breakaway torque calibration', 10],
          ['archHinge', 'Arch fold hinge & latch (parking fold)', 1.4, 5, 35, 'CNC machined clevis, PTFE-lined pins, positive latch, proof test', 10]].map(([key, name, mass, laborH, kWhkg, process, life]) => {
    const matCost = mass * (m.cost || 30), procKWh = mass * kWhkg, fabCost = matCost * 4 + laborH * labourRate, co2Mat = mass * (m.co2 || 40), co2Proc = procKWh * grid / 1000, energyMat = mass * (m.energy || 650), recyc = m.recyclable ?? 0.6;
    return { key, name, material: m.name, matId: m.id, family: m.family, mass, matCost, fabCost, laborH, procKWh, co2Mat, co2Proc, co2: co2Mat + co2Proc, energyMat, energy: energyMat + procKWh * 3.6, recyc, eolCredit: recyc * co2Mat * 0.6, life, process, unc: 25 };
  });
}

// Recycled feedstock (owner decision, Revision 5.1): recycled carbon fibre (pyrolysis-recovered, chopped/aligned) for the
// monocoque and spats; recycled titanium (scrap remelt) for buffer cells, cartridge housings and flexures. Factors are
// cradle-to-gate reductions relative to virgin material (literature ranges; mechanical knock-down handled in Suite 09).
export const RECYCLED = { cf: { co2: 0.35, energy: 0.3, cost: 0.6, note: 'recycled carbon fibre' }, ti: { co2: 0.3, energy: 0.35, cost: 0.75, note: 'recycled titanium' }, tow: { co2: 0.9, energy: 0.9, cost: 0.6, note: 'large-tow 50k fibre' } };
// Rev 6.2: large-tow (50k) carbon fibre for the spars — ≈ 60 % of the 12k-tow price at equal modulus and ≈ 10 % lower strength;
// after the ×0.9 knock-down the strength margins stay far above SF 2 (Suite 06 root Tsai–Wu ratio ≈ 474, arch strength
// factor ≈ 26; Suite 09 roll-over bridge ≈ 5.4; Suite 23 roll-over capacity ≈ 11, wing-root SF ≈ 134); buckling (32.2) is modulus-governed and unchanged.
export function recycledFactor(store, key, m) {
  const veh = store.project?.vehicle || {}, none = { co2: 1, energy: 1, cost: 1, note: '' };
  if (!m) return none;
  if (veh.recycledCF !== false && ['chassis', 'spats', 'fairings', 'bufferBox'].includes(key) && m.cls === 'cfrp') return RECYCLED.cf; // Rev 6.2: non-structural mouldings also in recycled fibre
  if (veh.recycledTi !== false && ['buffer', 'cartridgeHousing', 'flexure'].includes(key) && m.cls === 'ti') return RECYCLED.ti;
  if (veh.largeTowSpar !== false && key === 'spar' && m.cls === 'cfrp') return RECYCLED.tow;
  return none;
}
// Revision 5.1 part-audit additions: crash nose cone, side sills, buffer containment box, heat-pipe pigtail, passive conspicuity kit (Revision 6: no electric lighting)
function rev51Rows(store, { grid, labourRate }) {
  // Rev 6.2 fairness: the buffer enclosure and cages, conspicuity kit and ballast belong to every design (any buffer needs
  // containment; every design is unlit by specification); only the nose cone, sills and flexure pigtail are arch-specific
  const veh = store.project?.vehicle || {}, archOnly = ['noseCone', 'sideSills', 'hpPigtail', 'windHarvester', 'parkLimiter', 'hpThermalJoint'];
  const al = { name: 'Aluminium 5052 honeycomb + 6061 skins', cost: 12, co2: 9, energy: 155, recyclable: 0.85 }, cf = componentMaterial(store, 'chassis') || { name: 'CFRP', cost: 40, co2: 24, energy: 290, recyclable: 0.25 }, rcf = recycledFactor(store, 'chassis', cf);
  const R = (key, name, material, mass, matCostKg, co2Kg, energyKg, kWhkg, laborH, factor, life, process, recyc, fixedCost) => { const matCost = fixedCost ?? mass * matCostKg, procKWh = mass * kWhkg, co2Mat = mass * co2Kg, co2Proc = procKWh * grid / 1000, energyMat = mass * energyKg; return { key, name, material, mass, matCost, fabCost: fixedCost ?? (matCost * factor + laborH * labourRate), laborH, procKWh, co2Mat, co2Proc, co2: co2Mat + co2Proc, energyMat, energy: energyMat + procKWh * 3.6, recyc, eolCredit: recyc * co2Mat * 0.6, life, process, unc: 25 }; };
  return [
    R('noseCone', 'Crash nose cone (aluminium honeycomb crush element)', al.name, store.output?.('s23')?.noseConeMass_kg ?? 1.5, al.cost, al.co2, al.energy, 8, 3, 2, 20, 'Expanded 5052 honeycomb, bonded 6061 skins, pre-crushed trigger', al.recyclable),
    R('sideSills', 'Side-impact sills (CFRP crush tubes)', cf.name + (rcf.note ? ` (${rcf.note})` : ''), 2.2, (cf.cost || 40) * rcf.cost, (cf.co2 || 24) * rcf.co2, (cf.energy || 290) * rcf.energy, 22, 6, 3, 20, 'Braided / roll-wrapped CFRP tubes bonded into the monocoque sill', cf.recyclable ?? 0.25),
    R('bufferBox', 'Energy-buffer enclosure (2 mm recycled-fibre CFRP) & 4 mounts', cf.name + (rcf.note ? ` (${rcf.note})` : ''), store.output?.('s18')?.boxMass_kg !== undefined ? store.output('s18').boxMass_kg + 0.5 : 1.8, (cf.cost || 40) * rcf.cost, (cf.co2 || 24) * rcf.co2, (cf.energy || 290) * rcf.energy, 9, 4, 2.9, 20, 'Out-of-autoclave recycled-fibre enclosure, bonded Ti mount inserts (≥ 6 kN each)', cf.recyclable ?? 0.25),
    (() => { const o18 = store.output?.('s18'), mc = o18?.cageMass_kg ?? 2.2, cm = o18?.cageMat || 'ti-gr2', ti = { co2: 38, energy: 600, cost: 22 }, rT = veh.recycledTi !== false && cm === 'ti-gr2' ? RECYCLED.ti : { co2: 1, energy: 1, cost: 1, note: '' }, props = cm === 'ss304' ? { n: 'Stainless 304', co2: 4.8, energy: 60, cost: 4 } : cm === 'st-znni' ? { n: 'Low-carbon steel, Zn–Ni coated (recycled EAF feedstock)', co2: 1.1, energy: 14, cost: 1.4 } : cm === 'uhmwpe' ? { n: 'UHMWPE', co2: 2.5, energy: 90, cost: 8 } : { n: 'Titanium grade 2' + (rT.note ? ' (recycled titanium)' : ''), co2: ti.co2 * rT.co2, energy: ti.energy * rT.energy, cost: ti.cost * rT.cost }; return R('bufferCages', 'Per-cell containment cages (energy buffer)', props.n, mc, props.cost, props.co2, props.energy, 30, 6, 3, 20, 'Laser-perforated sleeves, rolled and welded, end caps', 0.8); })(),
    // Revision 6.5 demountable-arch dry thermal joint: two Al 6061 blocks (pad area from Suite 14, 8 mm thick) with brazed internal
    // condenser / evaporator passages, graphite pad, 2 × M6; arch-only
    // integrated variant (default, Suite 09 checked): two Ø114 × 6 mm Al 6061 inserts (0.01 m²) — arch side in a pocket milled in the β-Ti
    // flange (clearance fit on a centre dowel, PTFE-glass slip film: no press fit across the Al/Ti expansion mismatch), hub side bonded
    // into a moulded recess with a ply doubler in the CFRP root frame; 0.5 mm flexible-graphite pad; existing 12 × M6 at 2.8 kN each.
    ...(() => { const A = (store.output?.('s14')?.jointPad_cm2 ?? 100) / 1e4, integ = veh.jointIntegrated !== false; if (A <= 0) return [];
      return integ ? [R('hpThermalJoint', 'Demountable-arch dry thermal joint integrated in the pylon flange (two Ø114 mm Al 6061 inserts with heat-pipe passages: dowel-located in the Ti flange, bonded in the CFRP root frame; graphite pad; existing 12 × M6)', 'Al 6061 inserts, flexible graphite pad', 2 * A * 0.006 * 2700 + A * 0.0005 * 700 + 0.01, 6, 9, 160, 8, 1.0, 2.5, 15, 'Pocket milled in the Ti flange (0.25 h), recess moulded with a ply doubler in the root frame (0.1 h), inserts brazed to the loop passages (0.4 h), dowel + slip film + bond (0.15 h), pad die-cut (0.1 h)', 0.8)]
        : [R('hpThermalJoint', 'Demountable-arch dry thermal joint (sealed condenser block + sealed evaporator block, graphite pad, 2 × M6)', 'Al 6061 blocks, flexible graphite pad', 2 * A * 0.008 * 2700 + A * 0.0002 * 1100 + 0.04, 6, 9, 160, 8, 1.5, 2.5, 15, 'CNC-pocketed Al 6061 blocks with brazed passages joined into the two sealed loops at assembly; die-cut graphite pad', 0.8)]; })(),
    R('hpPigtail', 'Loop-heat-pipe pigtail across the flexure pylon (316L coil)', 'Stainless 316L tube', 0.25 * (store.output?.('s14')?.pigtailTurns ?? 4) / 4, 6, 5, 60, 10, 1.3, 3, 12, 'Mandrel-coiled 6 mm tube (0.2 h), two orbital welds (0.6 h), helium leak test (0.5 h)', 0.9),
    ...((veh.floorBallast ?? 0) > 0 ? [R('ballast', 'Floor ballast plates (recycled-steel offcuts, bonded)', 'Recycled steel (EAF scrap route)', veh.floorBallast, 0.8, 0.45, 9, 0.5, 1, 1.2, 20, 'Laser-cut plate offcuts bonded into the floor pan', 0.95)] : []),
    // Revision 6.3 practicable backup: elastomer bands + over-centre latches (Suite 18), regenerative one-way coupling, parked-wind click harvester
    ...(() => { const o = store.output?.('s18'); if (!o || o.cellType !== 'elastomer') return []; const E = { ir: ['Synthetic polyisoprene bands (30 phr black, antiozonant wax)', 4, 3.5, 95], sbr: ['SBR bands (black-filled)', 2.5, 3.0, 85], epdm: ['EPDM bands (black-filled)', 3, 3.2, 90] }[o.elastomer] || ['Synthetic polyisoprene bands', 4, 3.5, 95];
      return [R('bufferBands', `Buffer elastomer bands (${E[0]})`, E[0], o.bufferBands_kg || 0, E[1], E[2], E[3], 3, 3, 2.5, 8, 'Compression-moulded cured bands with moulded anchor eyes; cassette-mounted for replacement', 0.3),
              R('bufferLatches', 'Buffer over-centre latches and trigger (spring steel, no rotation beyond the toggle)', 'Spring steel 51CrV4', o.bufferLatches_kg || 0, 2.5, 2.4, 30, 4, 5, 3, 20, 'Laser-cut and formed toggles, flexure hinges, sequential trigger', 0.9)]; })(),
    ...(store.output?.('s18')?.regenCoupling_kg ? [R('regenCoupling', 'Regenerative linear-index coupling (2 front hubs: stationary flexure pawls on the existing cartridge-ring lobes, push-pull cables, linear toothed cell carriage; pedal-engaged, buckling torque limit — no drum, spool or wheel)', 'Spring steel / 17-4PH', store.output('s18').regenCoupling_kg, 4, 3, 40, 6, 6, 3, 15, 'Laser-cut spring-steel flexure pawls, two push-pull cables, a stamped toothed linear rack and carriage in the buffer box; cable from the brake pedal', 0.9),
       ...(veh.windHarvester === true && store.output('s18').windHarvester_kg > 0 ? [R('windHarvester', 'Parked-wind click harvester (pylon, parking-lock linked; optional research configuration)', 'Spring steel', store.output('s18').windHarvester_kg, 4, 3, 40, 5, 5, 3, 15, 'Compliant linear bistable click ratchet with linkage to the parking lock', 0.8)] : [])] : []),
    // parked ±3° yaw limiter (two preloaded EPDM shear pads + parking-lever linkage), mass from Suite 09
    ...(veh.parkLimiter === false || store.output?.('s09')?.parkLimiter?.required === false ? [] : [(store.output?.('s09')?.parkMode ?? store.params?.('s09')?.parkMode ?? 'merged') === 'merged'
      ? R('parkLimiter', veh.parkWedge === 'forked' ? 'Parked yaw restraint merged into the yaw end stops (2 enlarged EPDM end-stop blocks 70 × 70 × 30 mm + forked Al park wedge on the existing lever cable)' : 'Parked yaw restraint merged into the yaw end stops (2 enlarged EPDM end-stop blocks 70 × 70 × 30 mm + 2 Al wedges on the existing parking-lever cable)', 'EPDM blocks, Al wedges', store.output('s09').parkLimiter?.stopMass_kg ?? 0.32, 3, 3.2, 60, 4, veh.parkWedge === 'forked' ? 0.35 : 0.5, 1.5, 12, veh.parkWedge === 'forked' ? 'Moulded EPDM end-stop blocks (enlarged) and one forked 6061-T6 park wedge with PTFE-faced 4° tips, machined from one blank (≈ 0.35 h)' : 'Moulded EPDM end-stop blocks (enlarged), two machined Al wedges clamped on the existing cable (≈ 0.5 h)', 0.6)
      : R('parkLimiter', 'Parked yaw limiter ±3° (two EPDM shear pads on a pressed-steel wedge, parking-lever linked)', 'EPDM pads, pressed steel', store.output?.('s09')?.parkLimiter?.stopMass_kg ?? 1.08, 3, 3.2, 60, 4, 1, 1.5, 12, 'Purchased moulded EPDM pads bonded to a pressed-steel wedge (≈ 1 h: press, bond, fit cable); same pressed-part rates as the other stampings', 0.6)]),
    ...(veh.hubAutoLatch === true ? [R('hubAutoLatch', 'Optional automatic hub latches (2 bimetal snap latches on the hot-sector saddles; research option, 2 discrete moving parts)', 'Bimetal strip, stainless brackets', 0.05, 20, 8, 120, 5, 1, 2.5, 10, 'Purchased thermostatic bimetal discs in stamped brackets', 0.5)] : []),
    // hub sail hardware (Suite 17 sail position): bistable hold-off spring + pad lifter ring + cable branch per drive hub, the hot-sector
    // pivot-less bimetal strip indicator, and — for designs without the four-position lever — a separate drive/sail lever and cable
    // depower path (designs with the four-position lever): one stainless 7×19 Ø1.5 mm series strand through PTFE-lined rib guides,
    // a pre-stretched silicone return cord; the forked park wedge is in the parked-restraint row
    ...(veh.unloadingLever && veh.vents && veh.wingConfig === 'arch' ? (() => { const legH = store.output?.('s01')?.archLegHeight_m ?? 2.6, span = store.output?.('s01')?.bridgeSpan_m ?? 1.6, L = 3.0 + 0.97 * (2 * legH + span) + 0.6, mS = store.output?.('s01')?.depowerStrand_kg ?? (veh.tendonSeries !== false ? L * 0.009 + 3.6 * 0.03 : 0.149);
      return [R('depowerStrand', veh.tendonSeries !== false ? (veh.strandReturn === 'loop' || veh.strandReturn === undefined ? `Depower series strand, closed loop (stainless 7×19 Ø1.5 mm, ≈ ${(L + 3.6).toFixed(1)} m incl. 3.6 m return run to the lever's lost-motion slot, PTFE-lined guides, 3 crimps + slot nipple)` : `Depower series strand (stainless 7×19 Ø1.5 mm, ≈ ${L.toFixed(1)} m, PTFE-lined rib guides, 2 crimps)`) : 'Depower cable extension and 3 tendon tails', 'Stainless 316 strand, PTFE guides', mS, 15, 6, 60, 5, veh.tendonSeries !== false ? ((veh.strandReturn ?? 'loop') === 'loop' ? 0.6 : 0.5) : 0.6, 2, 15, veh.tendonSeries !== false ? 'Route one strand through the guides, crimp both ends (≈ 0.5 h)' : 'Three tails with three crimps (≈ 0.6 h)', 0.9),
              ...(veh.tendonSeries !== false && (veh.strandReturn ?? 'loop') !== 'loop' && (store.output?.('s01')?.returnCord_kg ?? 0.054) > 0 ? [R('returnCord', 'Depower return cord (pre-stretched silicone, 2 swaged end fittings; sized in Suite 01 for −20/+60 °C and long parking)', 'Silicone, stainless fittings', store.output?.('s01')?.returnCord_kg ?? 0.054, 12, 7, 120, 3, 0.1, 2, 8, 'Cut, pre-stretch, swage two end fittings (≈ 0.1 h)', 0.3)] : [])]; })() : []),
    R('hubSail', 'Hub sail hardware × 2 drive hubs (one bistable 17-7PH CH900 conical lifter ring Ø133/71 × 0.5 mm per hub + cable branch)', '17-7PH stainless, steel cable', 0.124, 6, 5, 70, 5, 0.6, 2.5, 15, 'One stamped and CH900 heat-treated conical ring per hub (replaces separate spring + ring); cable branches', 0.9),
    R('sailIndicator', 'Hot-sector bimetal strip indicator (pivot-less, marked at the net-work threshold)', 'Bimetal strip, PC window', 0.05, 15, 6, 100, 3, 0.3, 2, 15, 'Purchased bimetal strip in a printed scale housing', 0.5),
    ...(veh.unloadingLever && veh.vents && veh.wingConfig === 'arch' ? [] : [R('sailLever', 'Separate drive/sail lever and main cable (designs without the four-position lever)', 'Al lever, steel cable', 0.15, 5, 8, 120, 5, 0.7, 2.5, 15, 'Machined lever, purchased cable and housing', 0.8)]),
    R('lighting', 'Passive conspicuity kit (retroreflectors front/rear/side, retroreflective & photoluminescent tape, mirrors, flag) — no electric lighting', 'PMMA reflectors, prismatic film, glass mirrors, Al brackets', 0.6, 0, 6, 120, 0, 0, 1, 8, 'Purchased approved retroreflectors and tape; bonded mirrors', 0.4, 60),
  ].filter(r => veh.wingConfig === 'arch' || !archOnly.includes(r.key));
}

// Revision 6: solar-charged latent-heat store system (masses from Suite 14; PCM properties from the PCM library).
function rev6Rows(store, { grid, labourRate }) {
  const o = store.output?.('s14'); if (!o || o.hotSource !== 'store') return [];
  const pcm = pcmById(o.pcm), row = (key, name, material, mass, costKg, co2Kg, energyKg, kWhkg, laborH, factor, life, process, recyc) => { const matCost = mass * costKg, procKWh = mass * kWhkg, co2Mat = mass * co2Kg, co2Proc = procKWh * grid / 1000, energyMat = mass * energyKg; return { key, name, material, mass, matCost, fabCost: matCost * factor + laborH * labourRate, laborH, procKWh, co2Mat, co2Proc, co2: co2Mat + co2Proc, energyMat, energy: energyMat + procKWh * 3.6, recyc, eolCredit: recyc * co2Mat * 0.6, life, process, unc: 25, massSource: 'Suite 14 store design' }; };
  return [
    row('pcmFill', `Latent-heat store fill (${pcm.name})`, pcm.name, o.storePcm_kg || 0, pcm.cost, pcm.co2, pcm.energy, 0.5, 1, 1.2, 5, 'Melt-filled under dry nitrogen with nucleating additive; refill interval set by cycling-stability tests', pcm.recyclable),
    row('pcmVessel', 'Store vessel: 316L inner can, evacuated 5083 shell, reflective foil, Al heat-exchange fins', '316L / Al 5083 / Al foil', o.storeVessel_kg || 0, 7, 9, 120, 15, 8, 2.5, 20, 'Deep-drawn and orbital-welded cans, brazed fin pack, bake-out and evacuation, getter', 0.85),
    row('heatPipes6', 'Loop heat pipes & saddles (collector → store, store → hot arc, cold arc → spreader)', 'Copper / water, 316L lines', o.heatPipes_kg || 0, 12, 4.5, 60, 12, 6, 3, 15, 'Sintered-wick evaporators, charging, sealing, leak test', 0.9),
    row('rejectFins', 'Cold-side rejection fins in wheel pods / spats', 'Aluminium 1050 fin stock', o.rejectFin_kg || 0, 4, 9, 155, 8, 3, 2, 20, 'Stamped louvre-free fin plates brazed to heat-pipe condensers', 0.9),
    row('collectorUpgrade', 'Collector upgrade (evacuated glazing / reflector troughs / added absorber area)', 'Tempered glass, Al reflector, selective-coated Al', o.collectorUpgrade_kg || 0, 6, 2.5, 30, 10, 4, 2, 15, 'Pillar-supported evacuated glazing, edge seal, reflector forming', 0.6),
  ].filter(r => r.mass > 0);
}
