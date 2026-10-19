// Engineering economics & life-cycle sustainability: cost estimating relationships, learning curves,
// discounted cash flow (NPV/IRR/payback), life-cycle cost, cradle-to-gate LCA inventory from the
// materials registry, end-of-life credits and a simplified Material Circularity Indicator.
import { COMPONENTS, componentMaterial } from './materials.js';
import { brent } from './num.js';

// Reference component envelopes (volume of material at default design, processing energy, labour, life).
// Volumes are sized so the default material gives a plausible prototype mass; changing material keeps the
// volume (first-order estimate) — stiffness-equivalent resizing is done in the structural suites.
export const COMPONENT_REF = {
  chassis: { vol: 0.0258, qty: 1, kWhPerKg: 22, laborH: 120, factor: 3.5, life: 20, process: 'Prepreg lay-up, vacuum bag, oven/autoclave cure 180 °C, 6 bar' },
  chassisCore: { vol: 0.040, qty: 1, kWhPerKg: 4, laborH: 12, factor: 1.8, life: 20, process: 'CNC-contoured foam core, bonded in co-cure' },
  spar: { vol: 0.0077, qty: 1, kWhPerKg: 22, laborH: 40, factor: 3.5, life: 15, process: 'Tailored-angle prepreg box spar, matched-die cure' },
  rib: { vol: 0.0030, qty: 1, kWhPerKg: 12, laborH: 24, factor: 3.0, life: 15, process: 'Compliant ribs: GFRP press-moulding or PA12 powder-bed printing' },
  wingSkin: { vol: 0.016, qty: 1, kWhPerKg: 6, laborH: 30, factor: 2.2, life: 6, process: 'Reinforced elastomer skin laminated & thermoformed' },
  leadingEdge: { vol: 0.0019, qty: 1, kWhPerKg: 22, laborH: 12, factor: 3.5, life: 15, process: 'Thin CFRP shell, female-mould cure' },
  lattice: { vol: 0.0018, qty: 1, kWhPerKg: 55, laborH: 20, factor: 6.0, life: 12, process: 'Laser powder-bed fusion lattice + HIP + stress relief' },
  rootInsert: { vol: 0.00068, qty: 1, kWhPerKg: 8, laborH: 10, factor: 2.8, life: 20, process: '5-axis CNC machining of forged billet' },
  collector: { vol: 0.004, qty: 1, kWhPerKg: 12, laborH: 10, factor: 1.6, life: 10, process: 'PVD selective coating on aluminium substrate' },
  radiator: { vol: 0.0012, qty: 1, kWhPerKg: 10, laborH: 6, factor: 1.6, life: 8, process: 'Photonic film coating / lamination' },
  spreader: { vol: 0.0022, qty: 1, kWhPerKg: 3, laborH: 6, factor: 2.0, life: 20, process: 'Sheet stamping, brazed thermal joints' },
  heatpipe: { vol: 0.00067, qty: 1, kWhPerKg: 8, laborH: 4, factor: 2.5, life: 15, process: 'Sintered wick, evacuation, charging, sealing' },
  insulator: { vol: 0.0067, qty: 1, kWhPerKg: 15, laborH: 3, factor: 1.4, life: 12, process: 'Die-cut blanket, encapsulated' },
  actuator: { vol: 0.00037, qty: 1, kWhPerKg: 40, laborH: 30, factor: 2.2, life: 3, process: 'VIM melting, drawing/L-PBF, shape setting 500 °C, training cycles' },
  cartridgeHousing: { vol: 0.0009, qty: 1, kWhPerKg: 50, laborH: 16, factor: 4.0, life: 12, process: 'L-PBF titanium housings, machined datums' },
  cam: { vol: 0.00064, qty: 1, kWhPerKg: 8, laborH: 12, factor: 3.0, life: 10, process: 'CNC profile milling, H1025 ageing, profile grinding' },
  flexure: { vol: 0.0003, qty: 1, kWhPerKg: 10, laborH: 10, factor: 3.0, life: 5, process: 'Wire-EDM flexures, ageing, shot-peen' },
  buffer: { vol: 0.0012, qty: 1, kWhPerKg: 10, laborH: 14, factor: 3.0, life: 8, process: 'Wire-EDM bistable cells, ageing' },
  wheel: { vol: 0.0032, qty: 1, kWhPerKg: 6, laborH: 9, factor: 2.6, life: 15, process: 'Forged & machined rims' },
  tyre: { vol: 0.0052, qty: 1, kWhPerKg: 3, laborH: 0.6, factor: 4.0, life: 3, process: 'Purchased low-rolling-resistance tyres' },
  bearing: { vol: 0.000154, qty: 1, kWhPerKg: 10, laborH: 0.5, factor: 12, life: 8, process: 'Purchased sealed bearings' },
  brake: { vol: 0.0005, qty: 1, kWhPerKg: 3, laborH: 3, factor: 2.5, life: 8, process: 'Cast & machined discs' },
  brakePad: { vol: 0.00026, qty: 1, kWhPerKg: 4, laborH: 0.5, factor: 5, life: 2, process: 'Purchased friction pads' },
  steering: { vol: 0.00064, qty: 1, kWhPerKg: 4, laborH: 14, factor: 2.5, life: 20, process: 'Welded 4130 tube linkage, machined clevises' },
  cabin: { vol: 0.0042, qty: 1, kWhPerKg: 5, laborH: 10, factor: 2.0, life: 12, process: 'Thermoformed polycarbonate canopy, hard-coated' },
};

// Bill of materials with mass, cost and cradle-to-gate impacts. grid: gCO2/kWh for processing electricity.
export function bom(store, { grid = 450, labourRate = 35 } = {}) {
  return Object.entries(COMPONENTS).map(([key, c]) => {
    const m = componentMaterial(store, key), ref = COMPONENT_REF[key] || { vol: 0.001, kWhPerKg: 5, laborH: 2, factor: 2, life: 10, process: '—' };
    const mass = (m?.rho || 1000) * ref.vol, matCost = mass * (m?.cost || 5), procKWh = mass * ref.kWhPerKg;
    const fabCost = matCost * ref.factor + ref.laborH * labourRate;
    const co2Mat = mass * (m?.co2 || 5), co2Proc = procKWh * grid / 1000, energyMat = mass * (m?.energy || 80);
    const recyc = m?.recyclable ?? 0.3, eolCredit = recyc * co2Mat * 0.6;
    return { key, name: c.name, material: m?.name || '—', matId: m?.id, family: m?.family, mass, matCost, fabCost, laborH: ref.laborH, procKWh, co2Mat, co2Proc, co2: co2Mat + co2Proc, energyMat, energy: energyMat + procKWh * 3.6, recyc, eolCredit, life: ref.life, process: ref.process, unc: m?.unc ?? 20 };
  });
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
