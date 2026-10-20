// Engineering economics & life-cycle sustainability: cost estimating relationships, learning curves,
// discounted cash flow (NPV/IRR/payback), life-cycle cost, cradle-to-gate LCA inventory from the
// materials registry, end-of-life credits and a simplified Material Circularity Indicator.
import { COMPONENTS, componentMaterial } from './materials.js';
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
  flexure: { vol: 0.0003, qty: 1, kWhPerKg: 10, laborH: 10, factor: 3.0, life: 12, process: 'Arch pylon torsion-rod cage: centreless-ground β-21S rods, aged, shot-peened; machined end plates', basis: 'Rev 6.2: the arch pylon rods are designed to SF ≥ 2 in torsional fatigue (Suites 09 / 22) — inspected, not replaced, within 12 years (was 5 years for the former hub flexures)' },
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
    const m = componentMaterial(store, key), ref = COMPONENT_REF[key] || { vol: 0.001, kWhPerKg: 5, laborH: 2, factor: 2, life: 10, process: '—' };
    // masses come from the Suite 01 mass budget of the design actually configured; the reference volume is only a fallback
    // 'flexure' is only the arch pylon cage (hub flexures are booked in cartridgeHousing), so it is zero when Suite 01 has none
    // Rev 6.2: Suite 01 books the whole buffer system (cells + cages + box + mounts) as 'buffer'; cages and box have their own rows, so only the cells are priced here
    const mb = store.output('s01')?.massByComponent, massRef = (m?.rho || 1000) * ref.vol, cells18 = key === 'buffer' && mb && store.output?.('s18')?.bufferMass_kg, mass = cells18 ? Math.min(mb.buffer ?? cells18, cells18) : mb ? (mb[key] ?? (key in mb || key === 'flexure' ? 0 : massRef)) : massRef, matCost = mass * (m?.cost || 5), procKWh = mass * ref.kWhPerKg;
    const rf = recycledFactor(store, key, m), matCostR = matCost * rf.cost;
    const fabCost = matCostR * ref.factor + ref.laborH * labourRate;
    const co2Mat = mass * (m?.co2 || 5) * rf.co2, co2Proc = procKWh * grid / 1000, energyMat = mass * (m?.energy || 80) * rf.energy;
    const recyc = m?.recyclable ?? 0.3, eolCredit = recyc * co2Mat * 0.6;
    // Revision 6 owner-cost actions: NiTi cartridge life from the Suite 15 fatigue design (strain amplitude set for the
    // required life) or Suite 23; buffer-cell life from the Suite 18 Basquin estimate — both capped at the 20-year structure life.
    const s23l = key === 'actuator' ? Math.max(store.output?.('s23')?.cartridgeLife_years || 0, store.output?.('s15')?.lifeYearsAchieved || 0) : key === 'buffer' ? store.output?.('s18')?.fatigueLife_years : null, life = s23l && Number.isFinite(s23l) && s23l > ref.life ? Math.min(20, Math.floor(s23l)) : s23l === Infinity ? 20 : ref.life; // NiTi life extended only when Suite 23 fatigue (at the 1 % strain amplitude) justifies it
    return { lifeBasis: ref.basis, key, name: c.name, material: (m?.name || '—') + (rf.note ? ` (${rf.note})` : ''), matId: m?.id, family: m?.family, mass, massSource: mb?.[key] !== undefined ? 'Suite 01 mass budget' : 'reference volume', matCost: matCostR, fabCost, laborH: ref.laborH, procKWh, co2Mat, co2Proc, co2: co2Mat + co2Proc, energyMat, energy: energyMat + procKWh * 3.6, recyc, eolCredit, life, lifeSource: s23l && life !== ref.life ? 'Suite 23 fatigue' : 'reference', process: ref.process, unc: m?.unc ?? 20 };
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
  if (veh.archRoot !== 'ring') { // Revision 5.1: one centreline titanium flexure pylon (±15° elastic yaw) under a CFRP root crossbeam — no rotating parts
    const fm = componentMaterial(store, 'flexure') || { name: 'Titanium β-21S', rho: 4940, cost: 45, co2: 42, energy: 680, recyclable: 0.5, unc: 15 }, cf = componentMaterial(store, 'spar') || { name: 'CFRP', cost: 38, co2: 24, energy: 290, recyclable: 0.25 };
    const row = (key, name, mat, mass, laborH, kWhkg, factor, process, rfac = { co2: 1, energy: 1, cost: 1, note: '' }) => { const matCost = mass * (mat.cost || 30) * rfac.cost, procKWh = mass * kWhkg, co2Mat = mass * (mat.co2 || 20) * rfac.co2, co2Proc = procKWh * grid / 1000, energyMat = mass * (mat.energy || 300) * rfac.energy, recyc = mat.recyclable ?? 0.4; return { key, name, material: mat.name + (rfac.note ? ` (${rfac.note})` : ''), matId: mat.id, family: mat.family, mass, matCost, fabCost: matCost * factor + laborH * labourRate, laborH, procKWh, co2Mat, co2Proc, co2: co2Mat + co2Proc, energyMat, energy: energyMat + procKWh * 3.6, recyc, eolCredit: recyc * co2Mat * 0.6, life: 12, process, unc: 25 }; };
    return [row('archPylon', 'Arch centreline flexure pylon (titanium leaf-spring lattice, ±15° yaw)', fm, 1.9, 10, 45, 5, 'Wire-EDM from β-21S plate, chemical milling, shot peening, ageing; machined interfaces', recycledFactor(store, 'flexure', fm)),
            row('archCrossbeam', 'Arch root crossbeam (CFRP box) with co-moulded endplate and pod skins', cf, 1.4, 7, 9, 2.9, 'One out-of-autoclave moulding: box beam, root endplate and pod skins co-cured with Ti inserts (Rev 6.2 consolidation: one tool, one cure, no bonding or fasteners between them)')];
  }
  const m = componentMaterial(store, 'rootInsert') || { name: 'Ti-6Al-4V', rho: 4430, cost: 30, co2: 40, energy: 650, recyclable: 0.6, unc: 15 };
  return [['archRing', 'Arch ring track, slewing bearing, incidence drive & breakaway lock', 6.0, 14, 40, 'Ring rolling / CNC, induction-hardened raceway, crossed-roller bearing assembly, breakaway torque calibration', 10],
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
  const veh = store.project?.vehicle || {}, archOnly = ['noseCone', 'sideSills', 'hpPigtail'];
  const al = { name: 'Aluminium 5052 honeycomb + 6061 skins', cost: 12, co2: 9, energy: 155, recyclable: 0.85 }, cf = componentMaterial(store, 'chassis') || { name: 'CFRP', cost: 40, co2: 24, energy: 290, recyclable: 0.25 }, rcf = recycledFactor(store, 'chassis', cf);
  const R = (key, name, material, mass, matCostKg, co2Kg, energyKg, kWhkg, laborH, factor, life, process, recyc, fixedCost) => { const matCost = fixedCost ?? mass * matCostKg, procKWh = mass * kWhkg, co2Mat = mass * co2Kg, co2Proc = procKWh * grid / 1000, energyMat = mass * energyKg; return { key, name, material, mass, matCost, fabCost: fixedCost ?? (matCost * factor + laborH * labourRate), laborH, procKWh, co2Mat, co2Proc, co2: co2Mat + co2Proc, energyMat, energy: energyMat + procKWh * 3.6, recyc, eolCredit: recyc * co2Mat * 0.6, life, process, unc: 25 }; };
  return [
    R('noseCone', 'Crash nose cone (aluminium honeycomb crush element)', al.name, store.output?.('s23')?.noseConeMass_kg ?? 1.5, al.cost, al.co2, al.energy, 8, 3, 2, 20, 'Expanded 5052 honeycomb, bonded 6061 skins, pre-crushed trigger', al.recyclable),
    R('sideSills', 'Side-impact sills (CFRP crush tubes)', cf.name + (rcf.note ? ` (${rcf.note})` : ''), 2.2, (cf.cost || 40) * rcf.cost, (cf.co2 || 24) * rcf.co2, (cf.energy || 290) * rcf.energy, 22, 6, 3, 20, 'Braided / roll-wrapped CFRP tubes bonded into the monocoque sill', cf.recyclable ?? 0.25),
    R('bufferBox', 'Energy-buffer enclosure (2 mm recycled-fibre CFRP) & 4 mounts', cf.name + (rcf.note ? ` (${rcf.note})` : ''), store.output?.('s18')?.boxMass_kg !== undefined ? store.output('s18').boxMass_kg + 0.5 : 1.8, (cf.cost || 40) * rcf.cost, (cf.co2 || 24) * rcf.co2, (cf.energy || 290) * rcf.energy, 9, 4, 2.9, 20, 'Out-of-autoclave recycled-fibre enclosure, bonded Ti mount inserts (≥ 6 kN each)', cf.recyclable ?? 0.25),
    (() => { const o18 = store.output?.('s18'), mc = o18?.cageMass_kg ?? 2.2, cm = o18?.cageMat || 'ti-gr2', ti = { co2: 38, energy: 600, cost: 22 }, rT = veh.recycledTi !== false && cm === 'ti-gr2' ? RECYCLED.ti : { co2: 1, energy: 1, cost: 1, note: '' }, props = cm === 'ss304' ? { n: 'Stainless 304', co2: 4.8, energy: 60, cost: 4 } : cm === 'st-znni' ? { n: 'Low-carbon steel, Zn–Ni coated (recycled EAF feedstock)', co2: 1.1, energy: 14, cost: 1.4 } : cm === 'uhmwpe' ? { n: 'UHMWPE', co2: 2.5, energy: 90, cost: 8 } : { n: 'Titanium grade 2' + (rT.note ? ' (recycled titanium)' : ''), co2: ti.co2 * rT.co2, energy: ti.energy * rT.energy, cost: ti.cost * rT.cost }; return R('bufferCages', 'Per-cell containment cages (energy buffer)', props.n, mc, props.cost, props.co2, props.energy, 30, 6, 3, 20, 'Laser-perforated sleeves, rolled and welded, end caps', 0.8); })(),
    R('hpPigtail', 'Loop-heat-pipe pigtail across the flexure pylon (316L coil)', 'Stainless 316L tube', 0.25, 6, 5, 60, 10, 2, 3, 12, 'Mandrel-coiled 6 mm tube, orbital-welded, helium leak test', 0.9),
    ...((veh.floorBallast ?? 0) > 0 ? [R('ballast', 'Floor ballast plates (recycled-steel offcuts, bonded)', 'Recycled steel (EAF scrap route)', veh.floorBallast, 0.8, 0.45, 9, 0.5, 1, 1.2, 20, 'Laser-cut plate offcuts bonded into the floor pan', 0.95)] : []),
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
