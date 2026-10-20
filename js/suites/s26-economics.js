// Suite 26 — Economics, life-cycle sustainability & manufacturability assessment.
import { h, table } from '../core/ui.js';
import { fmt, linspace, range } from '../core/num.js';
import { bom, npv, irr, payback, discountedPayback, realRate, avgCost, unitCost, lcc, mci } from '../core/econ.js';
import { getSiteContext } from '../core/advisor.js';
import { tornado } from '../core/analysis.js';

const CUR = ['USD', 'EUR', 'GBP', 'JPY', 'CNY', 'INR', 'ZAR', 'BRL', 'CAD', 'AUD', 'CHF', 'KRW', 'MXN', 'TRY', 'IDR', 'PLN', 'SEK', 'NOK', 'DKK', 'SGD', 'HKD', 'NZD', 'THB', 'PHP', 'MYR', 'ILS', 'CZK', 'HUF', 'RON', 'ISK'];

export default {
  description: 'Prototype and production cost from the live bill of materials, learning curves, investment appraisal (NPV, IRR, payback) with live inflation, lending-rate and exchange-rate data for the site country, owner life-cycle cost, cradle-to-grave carbon and energy with live grid intensity, replacement intervals, end-of-life recycling credits, circularity, and a transparent comparison with a light battery-electric reference vehicle.',
  literature: 'life cycle assessment light electric vehicle carbon fibre composite recycling cost model learning curve',
  credibility: 'Analytical screening', econ: true, sustain: true,
  equations: [
    'Component cost-estimating relationship: C = m·p_material·f_process + h_labour·r_labour; Wright learning curve C_n = C₁ n^{log₂ PR}',
    'Net present value NPV = Σ CF_t/(1+r)^t, internal rate of return (NPV = 0, Brent root), simple and discounted payback; Fisher real rate (1+i)/(1+π) − 1',
    'Owner life-cycle cost LCC = purchase + Σ (operation + replacements)/(1+r)^t; equivalent annual cost',
    'Life-cycle inventory: Σ mᵢ·(CO₂e/kg)ᵢ (cradle-to-gate) + processing energy × grid intensity; use-phase energy × grid intensity; replacement multiplicity ⌈L/Lᵢ⌉ − 1',
    'End-of-life avoided-burden credit = recyclable fraction × material CO₂e × substitution factor (0.6)',
    'Material Circularity Indicator MCI = 1 − LFI·(0.9/X) with linear-flow index LFI = (V + W)/2M',
  ],
  methods: ['BOM regenerated from the current material assignments every run', 'Live economic & grid-carbon context fetched for the project site (World Bank, Our World in Data / Ember, ECB reference FX)', 'Tornado sensitivity of NPV'],
  assumptions: ['Reference battery-electric light vehicle figures are illustrative literature-typical assumptions (editable), not a specific product', 'Atmospheric propulsion assumed to have zero purchased energy; its feasibility is assessed in Suites 17/20', 'Indicative global material prices'],
  ibc: {
    initial: ['Baseline material inventory, process selection, initial component cost', 'Target production quantity, economic assumptions, expected service life, initial environmental inventory'],
    boundary: ['Geographic electricity & manufacturing energy mix (live grid intensity)', 'Material supply boundaries, transport, recycling assumptions, service-life boundary, maintenance intervals, end-of-life treatment', 'Functional unit: 1 vehicle-km over the service life; system boundary cradle-to-grave'],
  },
  cvv: {
    calibration: ['Supplier quotes, manufacturing records, maintenance data', 'Life-cycle inventory databases, material replacement costs, actual resource consumption'],
    verification: ['NPV/IRR consistency (NPV at IRR = 0)', 'Learning-curve unit vs average identity', 'Mass balance of the inventory = BOM mass'],
    validation: ['Post-prototype actual cost reconciliation', 'Third-party LCA review (ISO 14040/14044)'],
  },
  formats: { import: ['CSV/XLSX cost & inventory data'], export: ['Cost model CSV', 'LCA inventory CSV/JSON', 'Plots PNG/SVG'] },
  params: [
    { key: 'useLive', label: 'Use live economic & grid data for the site', type: 'bool', group: 'Data', default: true },
    { key: 'currency', label: 'Display currency', type: 'select', group: 'Data', default: 'USD', options: CUR },
    { key: 'grid', label: 'Grid carbon intensity (fallback)', unit: 'gCO₂/kWh', type: 'number', min: 0, max: 1200, step: 10, group: 'Data', default: 450 },
    { key: 'nominal', label: 'Nominal discount rate (fallback)', unit: '%', type: 'number', min: 0, max: 60, step: 0.5, group: 'Data', default: 8 },
    { key: 'inflation', label: 'Inflation (fallback)', unit: '%', type: 'number', min: -5, max: 100, step: 0.5, group: 'Data', default: 3 },
    { key: 'quantity', label: 'Production volume per year', type: 'number', min: 1, max: 100000, step: 1, group: 'Business case', default: 200 },
    { key: 'years', label: 'Business horizon', unit: 'years', type: 'number', min: 2, max: 30, step: 1, group: 'Business case', default: 10 },
    { key: 'learning', label: 'Learning-curve progress ratio', type: 'range', min: 0.7, max: 1, step: 0.01, group: 'Business case', default: 0.85 },
    { key: 'labourRate', label: 'Labour rate', unit: 'USD/h', type: 'number', min: 2, max: 150, step: 1, group: 'Business case', default: 35 },
    { key: 'capex', label: 'Tooling & facility investment', unit: 'USD', type: 'number', min: 0, max: 5e7, step: 1e4, group: 'Business case', default: 1.5e6 },
    { key: 'overhead', label: 'Fixed overhead per year', unit: 'USD', type: 'number', min: 0, max: 1e7, step: 1e4, group: 'Business case', default: 3e5 },
    { key: 'price', label: 'Vehicle sale price (manual mode)', unit: 'USD', type: 'number', min: 1000, max: 500000, step: 500, group: 'Business case', default: 26000 },
    { key: 'priceMode', label: 'Price setting', type: 'select', group: 'Business case', default: 'costplus', options: [{ value: 'costplus', label: 'Series unit cost + a fixed per-vehicle margin, capped at EV parity less the buyer discount' }, { value: 'parity', label: 'Owner-cost parity with the reference EV (willingness to pay) less a buyer discount' }, { value: 'manual', label: 'Manual sale price' }], help: 'Revision 6: price follows the willingness-to-pay logic; the unit margin at series cost is reported and must be positive.' },
    { key: 'unitMargin', label: 'Fixed margin per vehicle (cost-plus)', unit: 'USD', type: 'number', min: 0, max: 50000, step: 50, group: 'Business case', default: 10700, help: 'Price = series unit cost + this margin, rounded to the nearest $1, never above parity less the buyer discount. The same margin for every design keeps producer NPV design-neutral (it then differs only through tooling, prototype and learning), and the cheaper-to-build design passes its whole saving to the owner.' },
    { key: 'parityDisc', label: 'Buyer discount below parity', unit: '%', type: 'number', min: 0, max: 40, step: 1, group: 'Business case', default: 5, help: 'A new, unfamiliar vehicle type must undercut the incumbent on owner cost to sell.' },
    { key: 'designLife', label: 'ATMOS-XP structural design life', unit: 'years', type: 'number', min: 5, max: 40, step: 1, group: 'Ownership', default: 20, help: 'Basis: CFRP/titanium structure sized for fatigue (Suites 09, 22, 23); no traction battery to age.' },
    { key: 'evDesignLife', label: 'Reference EV design life', unit: 'years', type: 'number', min: 5, max: 40, step: 1, group: 'Reference light EV (assumptions)', default: 15 },
    { key: 'residHaircut', label: 'Market haircut on remaining-life value', unit: '%', type: 'number', min: 0, max: 100, step: 5, group: 'Ownership', default: 50, help: 'Residual value = purchase × remaining design-life fraction × (1 − haircut), discounted; the same rule for both vehicles.' },
    { key: 'insurance', label: 'Insurance + registration (both vehicles)', unit: 'USD/year', type: 'number', min: 0, max: 5000, step: 10, group: 'Ownership', default: 180, help: 'Basis: both are light-vehicle category (three-wheel / light quadricycle class, low power), so the same annual amount is used — no differential is claimed.' },
    { key: 'wtpBasis', label: 'Willingness-to-pay basis', type: 'select', group: 'Business case', default: 'evParity', options: [{ value: 'evParity', label: 'Reference-EV total-cost-of-ownership parity (default)' }, { value: 'assumed', label: 'Accept the sale price as given (user override)' }], help: 'Buyers will not pay more than the price at which owning ATMOS-XP costs the same as the reference EV over its life.' },
    { key: 'demandInfeasible', label: 'Demand share if simulated journeys fail', unit: '%', type: 'number', min: 0, max: 100, step: 1, group: 'Business case', default: 10, help: 'Research / enthusiast niche when Suite 20 shows journeys failing.' },
    { key: 'life', label: 'Vehicle service life', unit: 'years', type: 'number', min: 3, max: 30, step: 1, group: 'Ownership', default: 12 },
    { key: 'kmYear', label: 'Distance per year', unit: 'km', type: 'number', min: 100, max: 50000, step: 100, group: 'Ownership', default: 5000 },
    { key: 'maint', label: 'Routine maintenance', unit: 'USD/year', type: 'number', min: 0, max: 10000, step: 50, group: 'Ownership', default: 450 },
    { key: 'repMarkup', label: 'Spare-part price / fabricated cost', type: 'number', min: 1, max: 4, step: 0.1, group: 'Ownership', default: 1.6 },
    { key: 'recycledIn', label: 'Recycled content of materials', unit: '%', type: 'range', min: 0, max: 90, step: 1, group: 'Circularity', default: 10 },
    { key: 'collect', label: 'End-of-life collection rate', unit: '%', type: 'range', min: 0, max: 100, step: 1, group: 'Circularity', default: 70 },
    { key: 'evPrice', label: 'Reference light EV price', unit: 'USD', type: 'number', min: 2000, max: 80000, step: 500, group: 'Reference light EV (assumptions)', default: 15000 },
    { key: 'evCons', label: 'Reference EV consumption', unit: 'kWh/100 km', type: 'number', min: 3, max: 25, step: 0.5, group: 'Reference light EV (assumptions)', default: 8 },
    { key: 'evBattery', label: 'Reference EV battery', unit: 'kWh', type: 'number', min: 2, max: 60, step: 1, group: 'Reference light EV (assumptions)', default: 10 },
    { key: 'evBattCO2', label: 'Battery production footprint', unit: 'kgCO₂e/kWh', type: 'number', min: 30, max: 200, step: 5, group: 'Reference light EV (assumptions)', default: 85 },
    { key: 'evGlider', label: 'EV glider production footprint', unit: 'kgCO₂e', type: 'number', min: 500, max: 10000, step: 100, group: 'Reference light EV (assumptions)', default: 2500 },
    { key: 'evBattLife', label: 'Battery life', unit: 'years', type: 'number', min: 3, max: 20, step: 1, group: 'Reference light EV (assumptions)', default: 8 },
    { key: 'elecPrice', label: 'Electricity price', unit: 'USD/kWh', type: 'number', min: 0.01, max: 1, step: 0.01, group: 'Reference light EV (assumptions)', default: 0.18 },
  ],
  presets: [{ label: 'Pilot (50/yr)', values: { quantity: 50, price: 40000 } }, { label: 'Series (2 000/yr)', values: { quantity: 2000, price: 18000, capex: 8e6 } }, { label: 'Heavy use (15 000 km/yr)', values: { kmYear: 15000 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    let live = null, liveNote = 'Live context not requested — fallback values used.';
    if (p.useLive) { try { live = await Promise.race([getSiteContext(), new Promise((_, rej) => setTimeout(() => rej(new Error('timeout after 9 s')), 9000))]); liveNote = `Live context for ${live.country || 'site'}${live.errors?.length ? ' (partial: ' + live.errors.length + ' sources unavailable)' : ''}.`; } catch (e) { liveNote = 'Live context unavailable (' + e.message + ') — fallback values used.'; } }
    const grid = live?.gridCarbon?.value ?? p.grid, gridProv = live?.gridCarbon ? 'live' : 'estimated';
    const infl = (live?.inflation?.value ?? p.inflation) / 100, nominal = live?.lendingRate?.value !== undefined ? live.lendingRate.value / 100 : Math.max(p.nominal / 100, infl + 0.03), rReal = realRate(nominal, infl); // without a live lending rate, nominal ≥ inflation + 3 % keeps the real rate positive
    const rUse = Math.max(0.005, rReal), fx = p.currency === 'USD' ? 1 : (live?.fx?.[p.currency] ?? null), fxOK = fx !== null, X = (v) => fxOK ? v * fx : v, cur = fxOK ? p.currency : 'USD';
    const B = bom(ctx.store, { grid, labourRate: p.labourRate });
    const mass = B.reduce((s, r) => s + r.mass, 0), proto = B.reduce((s, r) => s + r.fabCost, 0), co2Cradle = B.reduce((s, r) => s + r.co2, 0), energy = B.reduce((s, r) => s + r.energy, 0), credit = B.reduce((s, r) => s + r.eolCredit, 0);
    // replacements over the vehicle life (priced at the learning-curve unit cost of the production programme)
    const seriesFactor = Math.min(1, avgCost(proto, p.learning, Math.max(1, p.quantity * p.years)) / Math.max(1, proto));
    const reps = B.map(r => ({ ...r, n: Math.max(0, Math.ceil(p.life / r.life) - 1) }));
    const repCO2 = reps.reduce((s, r) => s + r.n * r.co2, 0), repCostYr = reps.filter(r => r.n).map(r => ({ interval: r.life, cost: r.fabCost * seriesFactor * p.repMarkup })); // spares at series-production cost, not one-off prototype cost
    // Revision 6 owner-cost benefits (each with a basis): consumables common to both light vehicles (tyres, pads) are
    // charged to both; residual value on remaining design life uses the same rule for both; insurance/registration equal.
    const common = reps.filter(r => r.n && ['tyre', 'brakePad'].includes(r.key)).map(r => ({ interval: r.life, cost: r.fabCost * seriesFactor * p.repMarkup }));
    const disc = (1 + rUse) ** -p.life, rf1 = Math.max(0, (p.designLife - p.life) / p.designLife) * (1 - p.residHaircut / 100) * disc, rf2 = Math.max(0, (p.evDesignLife - p.life) / p.evDesignLife) * (1 - p.residHaircut / 100) * disc;
    const evEnergyCost0 = p.evCons / 100 * p.kmYear * p.elecPrice;
    const L2pre = lcc({ purchase: p.evPrice, years: p.life, rate: rUse, annualOps: p.maint * 0.8 + evEnergyCost0 + p.insurance, replacements: [{ interval: p.evBattLife, cost: p.evBattery * 150 }, ...common] }), L2net = L2pre.pv - p.evPrice * rf2;
    const L1unit = lcc({ purchase: 0, years: p.life, rate: rUse, annualOps: p.maint + p.insurance, replacements: repCostYr }).pv; // owner PV without the purchase
    const wtpParity = (L2net - L1unit) / Math.max(1e-6, 1 - rf1), unitPre = avgCost(proto, p.learning, Math.max(1, p.quantity * p.years)), capParity = Math.max(0, Math.floor(wtpParity * (1 - p.parityDisc / 100)));
    const priceSet = p.priceMode === 'manual' ? p.price : p.priceMode === 'parity' ? capParity : Math.min(capParity, Math.round(unitPre + p.unitMargin));
    const priceManual = p.price; p = { ...p, price: priceSet };
    // production business case
    const flows = [-p.capex - proto * 3]; let cum = 0; const unitHist = [];
    for (let y = 1; y <= p.years; y++) { const n0 = cum + 1, n1 = cum + p.quantity; const avg = (avgCost(proto, p.learning, n1) * n1 - (cum ? avgCost(proto, p.learning, cum) * cum : 0)) / p.quantity; unitHist.push(avg); flows.push(p.quantity * (p.price - avg) - p.overhead); cum = n1; void n0; }
    const NPV = npv(nominal, flows), IRR = irr(flows), PB = payback(flows), DPB = discountedPayback(nominal, flows);
    // owner LCC (real terms)
    const L1g = lcc({ purchase: p.price, years: p.life, rate: rUse, annualOps: p.maint + p.insurance, replacements: repCostYr }), L1 = { ...L1g, pv: L1g.pv - p.price * rf1, resid: p.price * rf1 };
    const evEnergyCost = evEnergyCost0, L2 = { ...L2pre, pv: L2net, resid: p.evPrice * rf2 };
    // Demand / price realism: price capped by willingness to pay (EV TCO parity unless overridden); volume scaled when
    // Suite 20 does not demonstrate successful journeys. Owner PV is linear in purchase price, so parity price = price − ΔPV.
    const s20r = ctx.upstream('s20'), wtp = p.wtpBasis === 'assumed' ? p.price : p.price - (L1.pv - L2.pv);
    const unitSeries = avgCost(proto, p.learning, Math.max(1, p.quantity * p.years)), unitMargin = p.price - unitSeries;
    // demand realism (Revision 6.1): demand scales with the share of trips feasible in the site's wind climate (Suite 20),
    // with the hub's help only when the research store is installed; never below the enthusiast niche share
    const storeOn = ctx.upstream('s14')?.hotSource === 'store', tripShare = s20r ? (storeOn ? s20r.tripFeasibleShareWithHub : s20r.tripFeasibleShareWind) : undefined;
    const adoption = Number.isFinite(tripShare) ? Math.min(1, Math.max(p.demandInfeasible / 100, tripShare)) : s20r && s20r.journeySuccess === false ? p.demandInfeasible / 100 : 1, priceR = Math.min(p.price, wtp), qtyR = Math.max(0, Math.round(p.quantity * adoption));
    const flowsR = [-p.capex - proto * 3]; { let cR = 0; for (let y = 1; y <= p.years; y++) { if (!qtyR) { flowsR.push(-p.overhead); continue; } const n1 = cR + qtyR, avg = (avgCost(proto, p.learning, n1) * n1 - (cR ? avgCost(proto, p.learning, cR) * cR : 0)) / qtyR; flowsR.push(qtyR * (priceR - avg) - p.overhead); cR = n1; } }
    const NPVr = npv(nominal, flowsR), IRRr = irr(flowsR), realistic = p.price <= wtp && adoption >= 1, demandNote = Number.isFinite(tripShare) ? `${fmt(100 * tripShare, 3)} % of trips feasible in the site's wind climate (Suite 20, ${storeOn ? 'wind + store-fed hub' : 'wind alone'}${s20r?.climateHours ? `, ${s20r.climateHours} hourly samples` : ''})` : 'Suite 20 journeys fail';
    // life-cycle carbon (functional unit vehicle-km)
    const kmLife = p.kmYear * p.life, atmosLC = co2Cradle + repCO2 - credit * (p.collect / 100);
    const evProd = p.evGlider + p.evBattery * p.evBattCO2, evRep = Math.max(0, Math.ceil(p.life / p.evBattLife) - 1) * p.evBattery * p.evBattCO2, evUseYr = p.evCons / 100 * p.kmYear * grid / 1000;
    const evLC = evProd + evRep + evUseYr * p.life - p.evGlider * 0.15;
    const recycW = B.reduce((s, r) => s + r.mass * r.recyc, 0) / mass;
    const MCI = mci({ mass, recycledIn: p.recycledIn / 100, collectedOut: p.collect / 100 * recycW, efficiency: 0.8, lifeRatio: p.life / 12 });
    const unitAtVol = avgCost(proto, p.learning, p.quantity * p.years);
    // KPIs
    const k = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('Prototype cost (one-off)', X(proto), cur, 'Sum of fabricated component costs from the live BOM (materials × process factor + labour).', 'estimated');
    k(`Average unit cost (${(p.quantity * p.years).toLocaleString()} units)`, X(unitAtVol), cur, `Learning curve PR ${p.learning}.`, 'calc');
    k('NPV of production programme', X(NPV), cur, `Nominal discount ${fmt(nominal * 100, 3)} %, ${p.years} years, ${p.quantity}/yr at ${fmt(X(p.price), 4)} ${cur}.`, 'calc', NPV > 0 ? 'ok' : 'bad');
    k('Willingness-to-pay ceiling', X(wtp), cur, p.wtpBasis === 'assumed' ? 'User override: sale price accepted as given.' : `Price at which the owner's life-cycle cost equals the reference EV (${fmt(X(L2.pv), 4)} ${cur} PV).`, p.wtpBasis === 'assumed' ? 'user' : 'calc', p.price <= wtp ? 'ok' : 'bad');
    k('Realism-adjusted NPV', X(NPVr), cur, `Price ${fmt(X(priceR), 4)} ${cur} (≤ willingness to pay), volume ${qtyR}/yr${adoption < 1 ? ` (demand × ${fmt(adoption, 3)}: ${demandNote}; floor ${p.demandInfeasible} % niche)` : ''}. IRR ${Number.isFinite(IRRr) ? fmt(IRRr * 100, 3) + ' %' : 'not defined — the cash flow never changes sign'}.`, 'calc', NPVr > 0 ? 'ok' : 'bad');
    k('IRR', Number.isFinite(IRR) ? IRR * 100 : 'not defined (cash flow has no sign change)', Number.isFinite(IRR) ? '%' : '', 'Discount rate at which NPV = 0.', 'calc', IRR > nominal ? 'ok' : 'warn');
    k('Payback (simple / discounted)', `${Number.isFinite(PB) ? fmt(PB, 3) : '∞'} / ${Number.isFinite(DPB) ? fmt(DPB, 3) : '∞'}`, 'years', 'Years until cumulative (discounted) cash flow turns positive.', 'calc');
    k('Sale price (Revision 6 pricing)', X(p.price), cur, p.priceMode === 'manual' ? 'Manual price.' : p.priceMode === 'parity' ? `Owner-cost parity with the reference EV ${fmt(X(wtpParity), 4)} ${cur} less a ${p.parityDisc} % buyer discount, rounded down to the nearest $1.` : `Cost-plus: series unit cost ${fmt(X(unitPre), 4)} + fixed margin ${fmt(X(p.unitMargin), 4)} ${cur}, rounded to the nearest $1${p.price >= capParity ? ' (capped at parity less the buyer discount)' : ''}; parity ceiling ${fmt(X(capParity), 4)} ${cur} (EV owner-cost parity ${fmt(X(wtpParity), 4)} less ${p.parityDisc} %).`, p.priceMode === 'manual' ? 'user' : 'generated');
    k('Unit margin at series cost', X(unitMargin), cur, `Price − average unit cost of ${(p.quantity * p.years).toLocaleString()} units (${fmt(X(unitSeries), 4)} ${cur}); ${fmt(100 * unitMargin / Math.max(1, p.price), 3)} % of price.`, 'calc', unitMargin > 0 ? 'ok' : 'bad');
    k('Owner life-cycle cost (PV)', X(L1.pv), cur, `Purchase + maintenance + insurance/registration ${p.insurance}/yr + scheduled replacements (NiTi cartridges ${B.find(r => r.key === 'actuator')?.life ?? '—'} y, buffer ${B.find(r => r.key === 'buffer')?.life ?? '—'} y) − residual value ${fmt(X(L1.resid), 4)} (${p.designLife}-year design life, ${p.residHaircut} % haircut) over ${p.life} years at a real rate of ${fmt(rUse * 100, 3)} %. No purchased energy.`, 'calc');
    k('Ownership running cost (PV, excluding purchase)', X(L1unit), cur, `Maintenance + insurance/registration + scheduled replacements only — independent of the sale price, so it compares designs directly. The residual value (${fmt(X(L1.resid), 4)} ${cur}, proportional to price) is counted in the total owner cost, not here.`, 'calc');
    k('Reference EV life-cycle cost (PV)', X(L2.pv), cur, `Same method: charging ${fmt(X(evEnergyCost), 4)}/yr, battery ${p.evBattery} kWh replaced every ${p.evBattLife} years at 150 USD/kWh, the same tyres/pads and insurance, residual ${fmt(X(L2.resid), 4)} (${p.evDesignLife}-year design life).`, 'hypothetical');
    k('Cradle-to-gate carbon', co2Cradle, 'kg CO₂e', `Materials ${fmt(B.reduce((s, r) => s + r.co2Mat, 0), 4)} + processing ${fmt(B.reduce((s, r) => s + r.co2Proc, 0), 4)} at ${fmt(grid, 3)} gCO₂/kWh.`, gridProv);
    k('Life-cycle carbon (with replacements & EoL credit)', atmosLC, 'kg CO₂e', 'Cradle-to-grave inventory; no purchased propulsion energy.', 'calc');
    k('Carbon per km — ATMOS-XP', atmosLC / kmLife * 1000, 'g CO₂e/km', `${kmLife.toLocaleString()} km functional unit.`, 'calc', atmosLC < evLC ? 'ok' : 'warn');
    k('Carbon per km — reference EV', evLC / kmLife * 1000, 'g CO₂e/km', `Includes charging at ${fmt(grid, 3)} gCO₂/kWh.`, 'hypothetical');
    k('Embodied energy', energy / 1000, 'GJ', 'Primary energy of materials + processing electricity (×3.6 MJ/kWh).', 'estimated');
    k('Material Circularity Indicator', MCI, '0–1', 'Ellen MacArthur-style indicator (0 = linear, 1 = fully circular).', 'calc', MCI > 0.5 ? 'ok' : 'warn');
    k('Real discount rate', rReal * 100, '%', 'Fisher real rate from nominal lending rate and inflation.', live?.lendingRate ? 'live' : 'estimated');
    // verdicts
    out.verdicts.push({ test: 'Data provenance', status: live && !live.errors?.length ? 'pass' : 'warn', detail: `${liveNote} Grid ${fmt(grid, 3)} gCO₂/kWh${live?.gridCarbon ? ' (' + live.gridCarbon.year + ')' : ''}; lending ${fmt(nominal * 100, 3)} %; inflation ${fmt(infl * 100, 3)} %${fxOK ? '' : `; ${p.currency} not in ECB reference set — shown in USD`}.` });
    out.verdicts.push({ test: 'Demand / price realism', status: realistic ? 'pass' : p.price <= wtp && adoption >= 0.5 ? 'warn' : 'fail', detail: realistic ? `Sale price ${fmt(X(p.price), 4)} ${cur} ≤ willingness to pay ${fmt(X(wtp), 4)} ${cur}; simulated journeys feasible (simulation, not road validation).` : `${p.price > wtp ? `Sale price ${fmt(X(p.price), 4)} exceeds willingness to pay ${fmt(X(wtp), 4)} ${cur} (owner cost ${fmt(L1.pv / L2.pv, 3)}× the reference EV). ` : ''}${adoption < 1 ? `Demand scaled to ${fmt(100 * adoption, 3)} % (${demandNote}). ` : ''}The assumed-demand NPV of ${fmt(X(NPV), 4)} ${cur} is not credible; realism-adjusted NPV ${fmt(X(NPVr), 4)} ${cur}.`, fix: 'Lower owner cost (maintenance, replacement intervals, unit cost) to reach EV parity and demonstrate journeys in Suite 20 before relying on volume forecasts.' });
    out.verdicts.push({ test: 'Production programme NPV > 0 (realism-adjusted)', status: NPVr > 0 ? 'pass' : 'fail', detail: `Adjusted NPV ${fmt(X(NPVr), 4)} ${cur} (assumed-demand NPV ${fmt(X(NPV), 4)} ${cur}, IRR ${Number.isFinite(IRR) ? fmt(IRR * 100, 3) + ' %' : 'not defined — the cash flow never changes sign'}).`, fix: 'Raise volume, cut unit cost (labour-heavy parts, see Suite 22), lower capex, or reposition the price.' });
    out.verdicts.push({ test: 'Life-cycle carbon below reference EV', status: atmosLC < evLC ? 'pass' : 'fail', detail: `${fmt(atmosLC / kmLife * 1000, 3)} vs ${fmt(evLC / kmLife * 1000, 3)} g CO₂e/km. Valid only if atmospheric propulsion proves feasible in tests — Suites 17 and 20 are simulations; otherwise the comparison is hypothetical.` });
    out.verdicts.push({ test: 'Positive unit margin at series cost', status: unitMargin > 0 ? 'pass' : 'fail', detail: `Price ${fmt(X(p.price), 4)} − series unit cost ${fmt(X(unitSeries), 4)} = ${fmt(X(unitMargin), 4)} ${cur} per vehicle.` });
    out.verdicts.push({ test: 'Owner cost competitiveness', status: L1.pv <= L2.pv ? 'pass' : 'warn', detail: `ATMOS-XP ${fmt(X(L1.pv), 4)} vs EV ${fmt(X(L2.pv), 4)} ${cur} present value.` });
    const f17 = ctx.upstream('s17'), f20 = ctx.upstream('s20');
    if (f17 || f20) out.verdicts.push({ test: 'Propulsion feasibility underpinning the comparison', status: (f20?.journeySuccess ?? f17?.feasible) ? 'pass' : 'warn', detail: f20 ? `Suite 20 journey success: ${f20.journeySuccess}.` : `Suite 17 hub feasible: ${f17.feasible}.` });
    out.tables.push({ title: 'Replacement intervals and their basis (owner cost)', cols: ['Part', 'Life (years)', 'Replacements in ownership', 'Basis'], rows: reps.map(r => [r.name, r.life, r.n, r.lifeBasis || (r.key === 'actuator' ? 'Suite 15 fatigue design (strain amplitude set for the required life)' : r.key === 'buffer' ? 'Suite 18 Basquin estimate' : ['tyre', 'brakePad'].includes(r.key) ? 'Consumable — charged to the reference EV as well' : 'Reference service life')]) });
    // checks
    out.checks.push({ name: 'NPV evaluated at IRR', value: Number.isFinite(IRR) ? npv(IRR, flows) : NaN, target: '≈ 0', pass: !Number.isFinite(IRR) || Math.abs(npv(IRR, flows)) < 1e-3 * Math.abs(flows[0]) });
    out.checks.push({ name: 'Inventory mass = BOM mass', value: reps.reduce((s, r) => s + r.mass, 0), target: fmt(mass, 5) + ' kg', pass: Math.abs(reps.reduce((s, r) => s + r.mass, 0) - mass) < 1e-9 });
    out.checks.push({ name: 'Learning curve: avg cost of 1 unit = unit cost', value: avgCost(proto, p.learning, 1), target: fmt(unitCost(proto, p.learning, 1), 5), pass: Math.abs(avgCost(proto, p.learning, 1) - proto) < 1e-6 });
    // charts
    const sorted = B.slice().sort((a, b) => b.co2 - a.co2);
    out.charts.push({ title: 'Cradle-to-gate carbon by component', type: 'bar', horizontal: true, categories: sorted.map(r => r.name), y: { label: 'CO₂e', unit: 'kg' }, series: [{ name: 'materials', values: sorted.map(r => r.co2Mat) }, { name: 'processing electricity', values: sorted.map(r => r.co2Proc) }], height: 640, wide: true, explain: 'Hotspots of embodied carbon. Composite and titanium parts dominate; processing share grows with grid carbon intensity at the factory location.' });
    const kms = linspace(0, kmLife, 60), stepRep = (km) => reps.reduce((s, r) => s + r.co2 * Math.min(r.n, Math.floor(km / (r.life * p.kmYear))), 0);
    out.charts.push({ title: 'Cumulative life-cycle carbon vs distance', type: 'line', x: { label: 'Distance', unit: 'km' }, y: { label: 'Cumulative CO₂e', unit: 'kg' }, series: [{ name: 'ATMOS-XP (replacements, no fuel/electricity)', x: kms, y: kms.map(km => co2Cradle + stepRep(km)) }, { name: 'Reference light EV', x: kms, y: kms.map(km => evProd + Math.min(Math.max(0, Math.ceil(p.life / p.evBattLife) - 1), Math.floor(km / (p.evBattLife * p.kmYear))) * p.evBattery * p.evBattCO2 + km * p.evCons / 100 * grid / 1000), dash: '6 4' }], explain: 'Production footprint (intercept), scheduled replacements (steps) and use-phase emissions (slope). A low-carbon grid flattens the EV slope; frequent cartridge/skin replacement steepens ATMOS-XP.' });
    const yrs = range(p.life + 1), cumPV = (L) => yrs.map(y => L.rows.slice(0, y + 1).reduce((s, r) => s + r.pv, 0));
    out.charts.push({ title: 'Owner cumulative cost (present value)', type: 'line', x: { label: 'Year' }, y: { label: 'Cumulative PV', unit: cur }, series: [{ name: 'ATMOS-XP', x: yrs, y: cumPV(L1).map(X) }, { name: 'Reference EV', x: yrs, y: cumPV(L2).map(X), dash: '6 4' }], explain: 'Discounted ownership cost; the higher purchase price must be recovered through zero energy cost against replacement costs.' });
    out.charts.push({ title: 'Production cash flow', type: 'bar', categories: flows.map((_, i) => 'Y' + i), y: { label: 'Cash flow', unit: cur }, series: [{ name: 'net cash flow', values: flows.map(X), colors: flows.map(f => f >= 0 ? '#22c55e' : '#ef4444') }], explain: 'Initial investment then annual margin (price − learning-curve unit cost) minus fixed overhead.' });
    const rates = linspace(0, 0.4, 41);
    out.charts.push({ title: 'NPV vs discount rate', type: 'line', x: { label: 'Discount rate', unit: '%' }, y: { label: 'NPV', unit: cur }, series: [{ name: 'NPV', x: rates.map(r => r * 100), y: rates.map(r => X(npv(r, flows))) }], annotations: [{ type: 'hline', y: 0, color: '#94a3b8' }, { type: 'vline', x: nominal * 100, label: 'site rate' }], explain: 'Where the curve crosses zero is the IRR. High-interest economies penalise capital-intensive production.' });
    const npvF = (x) => { const fl = [-x.capex - proto * 3]; let c2 = 0; for (let y = 1; y <= p.years; y++) { const n1 = c2 + x.quantity; const avg = (avgCost(proto * x.cost, p.learning, n1) * n1 - (c2 ? avgCost(proto * x.cost, p.learning, c2) * c2 : 0)) / x.quantity; fl.push(x.quantity * (x.price - avg) - p.overhead); c2 = n1; } return npv(x.rate, fl); };
    const tb = { capex: p.capex, quantity: p.quantity, price: p.price, cost: 1, rate: nominal };
    const tor = tornado(npvF, tb, [{ key: 'price', label: 'Sale price', lo: p.price * 0.8, hi: p.price * 1.2 }, { key: 'quantity', label: 'Annual volume', lo: Math.max(1, p.quantity * 0.5), hi: p.quantity * 1.5 }, { key: 'cost', label: 'Unit cost factor', lo: 0.8, hi: 1.2 }, { key: 'capex', label: 'Investment', lo: p.capex * 0.7, hi: p.capex * 1.3 }, { key: 'rate', label: 'Discount rate', lo: nominal * 0.5, hi: nominal * 1.5 }]);
    out.charts.push({ title: 'NPV sensitivity (tornado)', type: 'tornado', rows: tor.rows.map(r => ({ ...r, lo: X(r.lo), hi: X(r.hi) })), base: X(tor.base), x: { label: 'NPV', unit: cur }, explain: 'Which commercial assumptions decide the business case.' });
    // tables
    out.tables.push({ title: 'Life-cycle inventory & replacement schedule', cols: ['Component', 'Material', 'Mass kg', 'CO₂e kg', 'Energy MJ', 'Life y', 'Replacements', 'EoL recyclable', 'EoL credit kg CO₂e'], rows: reps.map(r => [r.name, r.material, r.mass, r.co2, r.energy, r.life, r.n, r.recyc, r.eolCredit * p.collect / 100]) });
    out.tables.push({ title: 'Reference light EV assumptions (illustrative, editable)', cols: ['Item', 'Value'], rows: [['Purchase price (USD)', p.evPrice], ['Consumption (kWh/100 km)', p.evCons], ['Battery (kWh)', p.evBattery], ['Battery footprint (kg CO₂e/kWh)', p.evBattCO2], ['Glider footprint (kg CO₂e)', p.evGlider], ['Battery life (years)', p.evBattLife], ['Electricity price (USD/kWh)', p.elecPrice], ['Grid intensity used (g/kWh)', grid]] });
    if (live?.fx) out.tables.push({ title: `Live reference exchange rates (ECB, ${live.fxDate})`, cols: ['Currency', 'per USD'], rows: Object.entries(live.fx).slice(0, 30).map(([c2, v]) => [c2, v]) });
    out.tables.push({ title: 'Live site economics', cols: ['Indicator', 'Value', 'Year', 'Source'], rows: [['Inflation, consumer prices (%)', live?.inflation?.value ?? '—', live?.inflation?.year ?? '', 'World Bank'], ['Lending interest rate (%)', live?.lendingRate?.value ?? '—', live?.lendingRate?.year ?? '', 'World Bank'], ['Renewables share of final energy (%)', live?.renewShare?.value ?? '—', live?.renewShare?.year ?? '', 'World Bank'], ['Electricity carbon intensity (g/kWh)', live?.gridCarbon?.value ?? '—', live?.gridCarbon?.year ?? '', 'Our World in Data / Ember']] });
    // recs
    const hot = sorted[0]; out.recs.push({ level: 'sustain', text: `Largest carbon hotspot: ${hot.name} (${fmt(hot.co2, 3)} kg CO₂e, ${hot.material}). Evaluate recycled-carbon or natural-fibre alternatives in Suite 03 and re-run Suites 09/26.`, basis: 'Component inventory' });
    const repHot = reps.filter(r => r.n).sort((a, b) => b.n * b.co2 - a.n * a.co2)[0]; if (repHot) out.recs.push({ level: 'sustain', text: `Replacement burden is led by ${repHot.name} (${repHot.n}× over life). Durability improvements here cut both life-cycle carbon and owner cost — a light part that must be replaced often can lose to a heavier durable one.`, basis: 'Replacement multiplicity' });
    if (NPV < 0) out.recs.push({ level: 'econ', text: `The programme does not pay back at ${p.quantity}/yr. Break-even needs higher volume or price; the tornado shows ${tor.rows[0].label.toLowerCase()} is the strongest lever.`, basis: 'DCF analysis' });
    if (nominal > 0.15) out.recs.push({ level: 'econ', text: `High local cost of capital (${fmt(nominal * 100, 3)} %) favours low-capex flexible manufacturing (out-of-autoclave, modular tooling) and staged investment.`, basis: 'Live lending rate' });
    out.recs.push({ level: 'good', text: 'Commission an ISO 14040/14044 LCA with primary supplier data and a critical review before publishing environmental claims; keep the functional unit (vehicle-km) and system boundary explicit.', basis: 'Specification Module 26' });
    out.explain.push({ title: 'What the comparison means', text: 'ATMOS-XP purchases no propulsion energy, so its footprint is production plus replacements minus end-of-life credits. The reference EV adds charging emissions that depend on the live grid intensity of the site. The comparison is only meaningful if the atmospheric propulsion is feasible for the intended duty — check Suites 17 and 20.' });
    out.views.push({ label: '2D animation: cash flow & carbon over life', icon: '🎞', render: (el) => anim2dLife(el, { flows: flowsR, nominal, B, life: p.life, evProd, evUseYr, evBattLife: p.evBattLife, evBattCO2: p.evBattery * p.evBattCO2, cur, X }) });
    out.views.push({ label: '3D life-cycle footprint', icon: '♻', render: (el) => view3dLCA(el, { B, life: p.life, cur, X, ctx }) });
    out.outputs = { price: p.price, priceMode: p.priceMode, unitMargin: unitMargin, runningPV: L1unit, residual: L1.resid, adoption, tripShare: tripShare ?? null, npvAdjusted: NPVr, wtp, priceRealistic: realistic, prototypeCost: proto, unitCost: unitAtVol, lifecycleCost: L1.pv, embodiedCO2_kg: co2Cradle, lifecycleCO2_kg: atmosLC, gPerKm: atmosLC / kmLife * 1000, npv: NPV, irr: IRR, mci: MCI, grid };
    out.headline = `Prototype ${fmt(X(proto), 4)} ${cur}; NPV ${fmt(X(NPV), 4)} ${cur}; ${fmt(atmosLC / kmLife * 1000, 3)} g CO₂e/km vs ${fmt(evLC / kmLife * 1000, 3)} g for the reference EV (grid ${fmt(grid, 3)} g/kWh).`;
    return out;
  },
};

// 3D vehicle coloured by each component's solved cradle-to-gate CO₂e (or fabricated cost) from the live BOM, with a
// service-life playback: parts flash when their solved replacement interval falls due and the running totals
// accumulate. Components not drawn in the 3D assembly are listed in the HUD.
async function view3dLCA(el, { B, life, cur, X, ctx }) {
  const [{ viewer }, { playback, hud, legend3d }, { vehicleAssembly }, { cmap }, { h }, mats] = await Promise.all([import('../core/viz3d.js'), import('../core/sim3d.js'), import('../core/cad.js'), import('../core/chart.js'), import('../core/ui.js'), import('../core/materials.js')]);
  const row = Object.fromEntries(B.map(r => [r.key, r]));
  let metric = 'co2';
  const sel = h('select', { onchange: (e) => { metric = e.target.value; } }, h('option', { value: 'co2' }, 'Colour: embodied CO₂e per component'), h('option', { value: 'cost' }, 'Colour: fabricated cost per component'));
  el.append(h('p', { class: 'small muted' }, `Solved from the BOM: mass × material emission factor + processing energy × grid intensity, and fabricated cost. Playback runs over the ${life}-year service life; a part flashes when its replacement interval is reached and its footprint is added again. Use Explode / X-ray in the toolbar; hover a part for its values.`), sel);
  const wrap = h('div'); el.append(wrap);
  const V = await viewer(wrap, { height: 460, grid: true });
  const matMap = Object.fromEntries(Object.keys(mats.COMPONENTS).map(k => [k, ctx.material(k)]));
  const asm = vehicleAssembly(ctx.vehicle, { materials: matMap });
  const drawn = new Set(); const objs = asm.parts.map(m => { const r = row[/reef joint/i.test(m.name) ? 'reefJoint' : /ring track/i.test(m.name) ? 'archRing' : /pylon/i.test(m.name) ? 'archPylon' : /crossbeam/i.test(m.name) ? 'archCrossbeam' : /nose cone|crash/i.test(m.name) ? 'noseCone' : /sill/i.test(m.name) ? 'sideSills' : /flexure/i.test(m.name) && /arch|root/i.test(m.name) ? 'archFlex' : /fold hinge/i.test(m.name) ? 'archHinge' : /spat/i.test(m.name) ? 'spats' : /fairing/i.test(m.name) ? 'fairings' : m.component]; if (r) { drawn.add(r.key); m.materialName = `${r.material} · ${fmt(r.mass, 3)} kg · ${fmt(r.co2, 3)} kg CO₂e · ${fmt(X(r.fabCost), 4)} ${cur} · replaced every ${r.life} y`; } return { r, o: V.add(m) }; });
  V.viewDir(0.9, -1.2, 0.6);
  const vmax = { co2: Math.max(...B.map(r => r.co2)), cost: Math.max(...B.map(r => r.fabCost)) };
  const leg = legend3d(wrap, { title: 'Embodied CO₂e (kg)', lo: 0, hi: vmax.co2, cmapName: 'inferno' }); const H = hud(wrap);
  const notDrawn = B.filter(r => !drawn.has(r.key));
  let spin = 0; V.animate = (dt) => { spin += dt * 0.2; V.root.rotation.z = spin; };
  playback(wrap, { duration: life, rate: life / 20, label: 'Service year', unit: 'y', onFrame: (t) => {
    let co2 = 0, cost = 0; const flashing = new Set();
    for (const r of B) { const n = Math.floor(t / r.life); co2 += r.co2 * (1 + n); cost += r.fabCost * (1 + n); if (n > 0 && t - n * r.life < life * 0.03) flashing.add(r.key); }
    leg.set(0, vmax[metric]);
    for (const { r, o } of objs) { if (!r) { o.material.color.setRGB(0.4, 0.42, 0.46); continue; } const c = flashing.has(r.key) ? [80, 255, 120] : cmap((metric === 'co2' ? r.co2 : r.fabCost) / (vmax[metric] || 1), 'inferno'); o.material.color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255); }
    const topR = B.slice().sort((a, b) => (metric === 'co2' ? b.co2 - a.co2 : b.fabCost - a.fabCost))[0];
    H.set([['Year', fmt(t, 3)], ['Cumulative CO₂e (parts)', fmt(co2, 4) + ' kg'], ['Cumulative parts cost', fmt(X(cost), 4) + ' ' + cur], ['Replacing now', flashing.size ? [...flashing].map(k => row[k].name).join(', ').slice(0, 40) : '—', flashing.size ? '#22c55e' : undefined], ['Largest contributor', topR.name.slice(0, 30)], ['Not drawn in 3D', notDrawn.length + ' parts (e.g. ' + (notDrawn[0]?.name || '—').slice(0, 22) + ')']]);
  } });
}

// 2D animation: cumulative (discounted) programme cash flow year by year, and cumulative life-cycle carbon of
// ATMOS-XP (parts + scheduled replacements) versus the reference battery EV (production + grid electricity + battery replacement).
async function anim2dLife(el, { flows, nominal, B, life, evProd, evUseYr, evBattLife, evBattCO2, cur, X }) {
  const [{ anim2d }, { h }] = await Promise.all([import('../core/sim3d.js'), import('../core/ui.js')]);
  const yrsF = flows.map((_, i) => i), cumF = [], cumD = []; flows.reduce((s, f, i) => { cumF.push(s + f); return s + f; }, 0); flows.reduce((s, f, i) => { const v = s + f / (1 + nominal) ** i; cumD.push(v); return v; }, 0);
  const ys = linspace(0, life, 121), atm = ys.map(y => B.reduce((s, r) => s + r.co2 * (1 + Math.floor(y / r.life)), 0)), ev = ys.map(y => evProd + evUseYr * y + Math.floor(y / evBattLife) * evBattCO2);
  el.append(h('p', { class: 'small muted' }, 'Solved economics and inventory of this suite, revealed year by year. Left: cumulative nominal and discounted cash flow of the production programme at the realism-adjusted price and volume. Right: cumulative CO₂e of one vehicle.'));
  const T = Math.max(life, flows.length - 1);
  anim2d(el, { height: 420, duration: T, rate: T / 8, label: 'Year', unit: 'y', onFrame: (t, g) => {
    g.clear(); const k = Math.min(flows.length - 1, Math.floor(t)), lo = Math.min(...cumF, ...cumD), hi = Math.max(...cumF, ...cumD, 0);
    const A = g.panel(0, 0, g.W / 2, g.H, { xr: [0, flows.length - 1], yr: [X(lo) * 1.1, X(hi) * 1.1 + 1], xlabel: 'Programme year', ylabel: `Cumulative cash [${cur}]`, title: `Cash (realism-adjusted): ${fmt(X(cumF[k]), 4)} (disc. ${fmt(X(cumD[k]), 4)}) ${cur}` });
    A.hline(0, '#94a3b8'); A.line(yrsF.slice(0, k + 1), cumF.slice(0, k + 1).map(X), '#38bdf8', 2.5); A.line(yrsF.slice(0, k + 1), cumD.slice(0, k + 1).map(X), '#a855f7', 2.5, [5, 3]); A.point(k, X(cumF[k]));
    const j = Math.min(ys.length - 1, Math.round(Math.min(t, life) / life * (ys.length - 1)));
    const Bp = g.panel(g.W / 2, 0, g.W / 2, g.H, { xr: [0, life], yr: [0, Math.max(...atm, ...ev) * 1.1], xlabel: 'Vehicle age [years]', ylabel: 'Cumulative CO₂e [kg]', title: `Carbon: ATMOS-XP ${fmt(atm[j], 4)} vs reference EV ${fmt(ev[j], 4)} kg` });
    Bp.line(ys.slice(0, j + 1), atm.slice(0, j + 1), '#22c55e', 2.5); Bp.line(ys.slice(0, j + 1), ev.slice(0, j + 1), '#ef4444', 2.5); Bp.point(ys[j], atm[j], '#22c55e'); Bp.point(ys[j], ev[j], '#ef4444');
  } });
}
