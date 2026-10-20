// Suite 14 — Thermal metamaterials, heat spreaders, insulation and thermal management.
import * as TH from '../core/thermo.js';
import { SCENARIOS, derive } from '../core/env.js';
import { linspace, fmt, luSolve, rk45, range, KELVIN, argmax, logspace, goldenMin } from '../core/num.js';
import { forUse, get } from '../core/materials.js';
import { vehicleAssembly } from '../core/cad.js';
import * as PS from '../core/pcmstore.js';
import { roadLoad } from '../core/physics.js';
import { designRegenEngine } from './s15-niti.js';
const archMeta = (veh) => { try { return veh.wingConfig === 'arch' ? vehicleAssembly(veh).meta.arch : null; } catch { return null; } };
// Thermal-path geometry from the current CAD: collector patches, driven front hubs, radiator (distances in m)
const _geo = new Map();
export function thermalGeometry(veh) {
  const key = JSON.stringify(veh); if (_geo.has(key)) return _geo.get(key);
  let g = null;
  try {
    const asm = vehicleAssembly(veh), P = asm.parts, find = (re) => P.find(m => re.test(m.name)), R = veh.wheelRadius;
    const fronts = asm.meta.wheelPos.filter(w => w[2]).map(([x, y]) => [x, y, R]), near = (c) => fronts.reduce((b, w) => { const d = Math.hypot(c[0] - w[0], c[1] - w[1], c[2] - w[2]); return d < b.d ? { d, w } : b; }, { d: Infinity, w: fronts[0] });
    const nose = find(/nose deck/i), bridge = find(/arch bridge top/i), rad = find(/Radiative-cooling/i);
    const nearestVertex = (m) => { let best = Infinity; if (!m) return best; for (let k = 0; k < m.p.length; k += 3) for (const [x, y] of fronts) { const d = Math.hypot(m.p[k] - x, m.p[k + 1] - (y - Math.sign(y) * 0.12), m.p[k + 2] - (R + 0.12)); if (d < best) best = d; } return best; };
    const nC = nose?.centroid(), bC = bridge?.centroid(), rC = rad?.centroid();
    g = { arch: veh.wingConfig === 'arch', noseArea: nose ? nose.area() / 2 : 0, bridgeArea: bridge ? (asm.meta.arch?.collectorArea ?? bridge.area() / 2) : 0,
      strapL: nose ? Math.max(0.15, nearestVertex(nose)) : 0.5, strapToCentroid: nC ? near(nC).d : 0.5, lhpL: bC ? 1.2 * near(bC).d : 0, lhpDz: bC ? bC[2] - R : 0, radL: rC ? near(rC).d : 0.6, radDz: rC ? rC[2] - R : 0 };
  } catch { g = null; }
  _geo.set(key, g); return g;
}
// Loop heat pipe (capillary-pumped two-phase loop, fine-pore sintered-nickel primary wick). Water properties vs T (°C);
// ammonia only below ≈ 90 °C (critical point 132 °C). Pressure budget: 2σ/r_p ≥ ΔP_liquid + ΔP_vapour + ΔP_wick + ρ_l g Δz.
export function loopHeatPipe({ fluid = 'water', Tsat = 100, dz = 2.4, L = 3.7, dLiq = 0.004, dVap = 0.006, rPore = 1.2e-6, K = 1e-14, tWick = 0.003, Aw = 0.0094, hEvap = 12000, Aevap = 0.0094, hCond = 8000, Acond = 0.02 } = {}) {
  let sig, rl, mul, hfg, rv, muv, ok = true, note = '';
  if (fluid === 'ammonia') { sig = Math.max(0.002, 0.0234 - 1.9e-4 * Tsat); rl = 638 - 1.6 * Tsat; mul = 1.7e-4 * Math.exp(-0.012 * Tsat); hfg = 1.26e6 - 4.5e3 * Tsat; const pv = 429.6e3 * Math.exp(0.035 * Tsat); rv = pv / (488 * (Tsat + 273.15)); muv = 1e-5; if (Tsat > 90) { ok = false; note = `Ammonia unsuitable at ${Tsat.toFixed(0)} °C (vapour pressure ≈ ${(pv / 1e5).toFixed(0)} bar, near the critical point).`; } }
  else { sig = 0.0756 - 1.67e-4 * Tsat; rl = 1000 - 0.42 * Tsat; mul = 2.414e-5 * 10 ** (247.8 / (Tsat + 133.15)); hfg = 2.501e6 - 2370 * Tsat; const pv = 611.21 * Math.exp((18.678 - Tsat / 234.5) * (Tsat / (257.14 + Tsat))); rv = pv / (461.5 * (Tsat + 273.15)); muv = 1.2e-5; if (Tsat < 40) note = 'Low vapour density below 40 °C raises vapour-line losses; water LHPs prefer ≥ 60 °C.'; }
  const dPcap = 2 * sig / rPore, dPhyd = rl * 9.81 * Math.max(0, dz);
  const cL = 128 * mul * L / (Math.PI * rl * hfg * dLiq ** 4), cV = 128 * muv * L / (Math.PI * rv * hfg * dVap ** 4), cW = mul * tWick / (rl * hfg * K * Aw), perW = cL + cV + cW;
  const Qmax = Math.max(0, (dPcap - dPhyd) / perW), R = 1 / (hEvap * Aevap) + 1 / (hCond * Acond);
  return { ok: ok && dPcap > dPhyd, Qmax: ok ? Qmax : 0, dPcap, dPhyd, perW, cL, cV, cW, R, sig, note, startup: `Start-up needs ≈ 3–8 K evaporator superheat and a liquid-flooded compensation chamber; adverse elevation ${dz.toFixed(2)} m uses ${(100 * dPhyd / dPcap).toFixed(0)} % of the capillary head.` };
}

function envCase(env, kind) {
  if (kind === 'current' && env) return { T: env.T, Uz: env.Uz ?? env.U, ghi: env.ghi || 0, eps: env.eps ?? 0.8, isDay: env.isDay, src: 'Suite 05 snapshot', coupled: true };
  const want = kind === 'current' ? 'night' : kind;
  if (env?.series?.length) {
    const s = env.series; if (want === 'day') { const i = argmax(s.map(r => r.ghi || 0)); if (s[i].ghi > 50) return { ...s[i], isDay: true, src: `Suite 05 peak-sun hour ${s[i].time}`, coupled: true }; }
    else { const n = s.filter(r => !r.isDay).sort((a, b) => (a.T - a.Tsky) - (b.T - b.Tsky)); if (n.length) { const r = n[Math.floor(n.length / 2)]; return { ...r, isDay: false, src: `Suite 05 median night hour ${r.time}`, coupled: true }; } }
  }
  const d = derive(want === 'day' ? SCENARIOS.sunnyCalm : SCENARIOS.clearNight, {});
  return { T: d.T, Uz: d.Uz, ghi: d.ghi, eps: d.eps, isDay: want === 'day', src: `scenario “${(want === 'day' ? SCENARIOS.sunnyCalm : SCENARIOS.clearNight).label}”`, coupled: false };
}

export default {
  description: 'Solves the complete heat path from the photonic collector to the actuator hot interface and from the actuator cold interface to the radiator: thermal-resistance network with contact resistances, heat-pipe operating limits, insulation leakage, a 2-D anisotropic finite-volume conduction field and the transient warm-up. It computes the actual interface temperatures seen by the actuator — never assuming they equal ambient or sky temperature.',
  literature: 'anisotropic thermal metamaterial heat flux manipulation thermal contact resistance heat pipe limits',
  sustain: true, credibility: 'Analytical screening',
  equations: [
    'Fourier’s law q = −k∇T with anisotropic conductivity tensor diag(k_x, k_y); transient heat conduction ρc_p ∂T/∂t = ∇·(k∇T)',
    'Thermal resistance networks: conduction L/(kA), contact 1/(h_cA), insulation leakage, series/parallel combination',
    'Interfacial (contact) conductance and harmonic-mean face conductances in the finite-volume discretisation',
    'Newton’s law of cooling; McAdams wind-forced convection; finned-exchanger convective area',
    'Surface radiative balances from Suite 13 (absorber / radiator spectral models)',
    'Heat-pipe limits: capillary (Laplace pressure vs liquid and gravity losses, Darcy wick flow), sonic, entrainment, viscous and boiling limits; water properties vs temperature',
    'Energy conservation at every node; lumped-capacitance transient network for warm-up time constants',
  ],
  methods: ['Newton–Raphson on the 4-node nonlinear network (finite-difference Jacobian, LU solve)', 'Cell-centred finite-volume steady conduction with successive over-relaxation (ω = 1.85)', 'Adaptive Dormand–Prince integration of the 2-node transient network', 'Converter load sweep (impedance matching) for maximum ideal work'],
  assumptions: ['The actuator is represented as an effective thermal conductance G_conv (heat drawn per kelvin of interface ΔT) coupled from Suites 15/17 when available', 'Mechanical work extracted (≤ few % of heat) is neglected in the heat balance', 'Symmetric construction of the hot and cold paths'],
  ibc: {
    initial: ['Uniform initial temperature equal to ambient air', 'Initial contact state and interface pressure (via conductance h_c)', 'Heat-pipe operating state (temperature-dependent properties)'],
    boundary: ['Collector side: spectral radiative + convective balance (day: solar absorber; night: ambient-air finned exchanger)', 'Radiator side: radiative-cooling balance + convection', 'Actuator interfaces: coupled heat flux G_conv(T_hi − T_ci) — not imposed temperatures', 'Adiabatic outer edges of the conduction block; contact resistance at material interfaces'],
  },
  cvv: {
    calibration: ['Thermal conductivity tensor (laser flash, transient plane source)', 'Interface thermal resistance tests vs contact pressure', 'Heat-pipe performance curves (Q vs ΔT, orientation)'],
    verification: ['1-D slab steady conduction vs analytic linear profile', 'Network node energy residuals', '2-D field boundary heat balance', 'Lumped-capacitance exponential solution'],
    validation: ['Instrumented thermal-sector rig: thermocouples at collector, interfaces and radiator; heat-flux sensors; transient histories under outdoor night/day exposure'],
  },
  formats: { import: ['CSV temperature/heat-flux logs', 'XLSX', 'JSON'], export: ['CSV field and network results', 'VTK field (via Suite 02 export manager)', 'PNG/SVG'] },
  params: [
    { key: 'case', label: 'Operating case', type: 'select', group: 'Case', default: 'night', auto: (c) => { const e = c.upstream('s05') || c.env; const sunny = !!(e?.isDay && (e.ghi || 0) > 200) || (e?.series || []).some(r => (r.ghi || 0) > 200); return { value: sunny ? 'day' : 'night', prov: 'coupled' }; }, options: [{ value: 'night', label: 'Night (ambient source → radiator sink)' }, { value: 'day', label: 'Day (solar absorber → radiator/ambient sink)' }, { value: 'current', label: 'Current Suite 05 snapshot' }] },
    { key: 'spreader', label: 'Spreader / thermal metamaterial', type: 'select', group: 'Materials', default: 'al-1050', options: forUse('spreader').map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.store.material('spreader', 'al-1050'), prov: 'library' }) },
    { key: 'insulator', label: 'Hot/cold isolation', type: 'select', group: 'Materials', default: 'aerogel', options: forUse('insulator').map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.store.material('insulator', 'aerogel'), prov: 'library' }) },
    { key: 'anis', label: 'Lattice anisotropy k_in-plane / k_through', type: 'range', min: 1, max: 200, step: 1, group: 'Materials', default: 1, help: 'Architected thermal metamaterial: >1 channels heat along the spreader while limiting through-thickness leakage.' },
    { key: 'nHP', label: 'Number of heat pipes per path', type: 'number', min: 0, max: 40, step: 1, group: 'Heat pipes', default: 6 },
    { key: 'Lhp', label: 'Heat-pipe length (cold path, radiator ↔ hubs)', unit: 'm', type: 'number', min: 0.05, max: 6, step: 0.01, group: 'Heat pipes', default: 0.6, auto: (c) => { const g = thermalGeometry(c.vehicle); return g?.arch ? { value: +g.radL.toFixed(2), prov: 'generated' } : undefined; }, help: 'From the CAD: rear-deck radiator to the driven front hubs.' },
    { key: 'strapL', label: 'Graphite strap length (nose collector → hub hot interface)', unit: 'm', type: 'number', min: 0.1, max: 2, step: 0.01, group: 'Hot path (AeroArch)', default: 0.3, auto: (c) => { const g = thermalGeometry(c.vehicle); return g?.arch ? { value: +Math.min(g.strapL, 2).toFixed(2), prov: 'generated' } : undefined; }, help: 'Hot interface relocated into the nose bay beside the driven hubs: shortest collector-edge-to-hub distance from the CAD.' },
    { key: 'strapMass', label: 'Graphite strap mass budget (both hubs)', unit: 'kg', type: 'number', min: 0.02, max: 3, step: 0.01, group: 'Hot path (AeroArch)', default: 0.5, help: 'For fixed mass the lowest-resistance strap has R = L²ρ/(k·m), independent of how the section is split.' },
    { key: 'matchG', label: 'Match converter draw for maximum power (Curzon–Ahlborn)', type: 'bool', group: 'Converter coupling', default: true },
    { key: 'lhpFluid', label: 'Loop-heat-pipe working fluid (bridge → hubs)', type: 'select', group: 'Hot path (AeroArch)', default: 'water', options: ['water', 'ammonia'] },
    { key: 'lhpPore', label: 'Primary wick pore radius (sintered nickel)', unit: 'µm', type: 'number', min: 0.3, max: 20, step: 0.1, group: 'Hot path (AeroArch)', default: 1.2 },
    { key: 'dHP', label: 'Heat-pipe diameter', unit: 'mm', type: 'number', min: 3, max: 20, step: 0.5, group: 'Heat pipes', default: 8 },
    { key: 'tilt', label: 'Heat-pipe tilt (evaporator above = +)', unit: '°', type: 'range', min: -90, max: 90, step: 1, group: 'Heat pipes', default: 0, expert: true, auto: (c) => { const g = thermalGeometry(c.vehicle); return g?.arch ? { value: +(-Math.asin(Math.min(1, Math.max(0, g.radDz) / (g.radL || 1))) * 180 / Math.PI).toFixed(1), prov: 'generated' } : undefined; }, help: 'Cold path: hub (evaporator) below the rear-deck radiator (condenser) → negative tilt = gravity-assisted.' },
    { key: 'Lsp', label: 'Spreader length', unit: 'm', type: 'number', min: 0.02, max: 1, step: 0.01, group: 'Geometry', default: 0.3 },
    { key: 'wsp', label: 'Spreader width', unit: 'm', type: 'number', min: 0.01, max: 1, step: 0.01, group: 'Geometry', default: 0.2 },
    { key: 'tsp', label: 'Spreader thickness', unit: 'mm', type: 'number', min: 0.1, max: 20, step: 0.1, group: 'Geometry', default: 3 },
    { key: 'hc', label: 'Interface contact conductance', unit: 'W/m²K', type: 'number', min: 100, max: 100000, step: 100, group: 'Interfaces', default: 4000, help: 'Bare metal contact ≈ 1–5 kW/m²K; with thermal interface material 10–50 kW/m²K.' },
    { key: 'Ac', label: 'Interface contact area per side', unit: 'cm²', type: 'number', min: 1, max: 5000, step: 1, group: 'Interfaces', default: 300 },
    { key: 'tIns', label: 'Isolation thickness', unit: 'mm', type: 'number', min: 1, max: 100, step: 1, group: 'Interfaces', default: 10 },
    { key: 'Ains', label: 'Isolation (leak path) area', unit: 'cm²', type: 'number', min: 1, max: 5000, step: 1, group: 'Interfaces', default: 400 },
    { key: 'Gconv', label: 'Actuator thermal draw G_conv', unit: 'W/K', type: 'number', min: 0.01, max: 500, step: 0.1, group: 'Converter coupling', default: 4, auto: (c) => { const o = c.upstream('s15'); return o?.Gconv_W_K ? { value: +o.Gconv_W_K.toFixed(3), prov: 'coupled' } : undefined; } },
    { key: 'fin', label: 'Air-side finned area multiplier', type: 'range', min: 1, max: 10, step: 0.5, group: 'Converter coupling', default: 3 },
    { key: 'hotSource', label: 'Hub hot source', type: 'select', group: 'Latent-heat store', default: 'direct', auto: (c) => ({ value: c.vehicle?.thermalStore ? 'store' : 'direct', prov: 'coupled' }), options: [{ value: 'direct', label: 'Direct collector path (default vehicle)' }, { value: 'store', label: 'Research configuration: solar-charged latent-heat store installed' }], help: 'Revision 6.1 decision: the store system gives ≈ 1 min of backup for ≈ 30 kg on the demonstrated basis, so it is an optional research configuration, OFF by default. Its sizing is still shown for comparison.' },
    { key: 'pcm', label: 'Phase-change material', type: 'select', group: 'Latent-heat store', default: 'hdpe', options: PS.PCMS.map(m => ({ value: m.id, label: `${m.name} — ${m.Tm} °C` })) },
    { key: 'storeSizing', label: 'Store sizing', type: 'select', group: 'Latent-heat store', default: 'budget', options: [{ value: 'budget', label: 'Largest store inside the vehicle mass cap' }, { value: 'demo', label: 'Backup requirement on the demonstrated basis' }, { value: 'model', label: 'Backup requirement on the model basis' }, { value: 'target', label: 'Backup requirement on the research target' }] },
    { key: 'massCap', label: 'Vehicle mass cap', unit: 'kg', type: 'number', min: 100, max: 1000, step: 1, group: 'Latent-heat store', default: 258 },
    { key: 'backupH', label: 'Required backup duration', unit: 'h', type: 'number', min: 0.1, max: 10, step: 0.1, group: 'Latent-heat store', default: 1 },
    { key: 'margin', label: 'Required margin on the backup duration', unit: '%', type: 'number', min: 0, max: 100, step: 1, group: 'Latent-heat store', default: 10 },
    { key: 'vSize', label: 'Backup road speed', unit: 'km/h', type: 'number', min: 5, max: 60, step: 1, group: 'Latent-heat store', default: 20 },
    { key: 'dTface', label: 'Fin-face drop below the melting point at end of discharge', unit: 'K', type: 'number', min: 1, max: 30, step: 0.5, group: 'Latent-heat store', default: 6 },
    { key: 'dThp', label: 'Hot heat-pipe temperature drop (store → hot arc)', unit: 'K', type: 'number', min: 1, max: 30, step: 0.5, group: 'Latent-heat store', default: 6 },
    { key: 'dTcp', label: 'Cold heat-pipe temperature drop (cold arc → spreader)', unit: 'K', type: 'number', min: 1, max: 30, step: 0.5, group: 'Latent-heat store', default: 4 },
    { key: 'epsMLI', label: 'Effective emittance of the evacuated gap (multilayer foil)', type: 'number', min: 0.002, max: 0.2, step: 0.001, group: 'Latent-heat store', default: 0.01 },
  ],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const env = ctx.upstream('s05') || ctx.env, s13 = ctx.upstream('s13');
    const c = envCase(env, p.case), TaK = c.T + KELVIN, h = TH.mcAdamsH(c.Uz || 0);
    const colM = get(s13?.collector || ctx.store.material('collector', 'coat-tinox')), radM = get(s13?.radiator || ctx.store.material('radiator', 'coat-pdrc'));
    const Acol = s13?.collectorArea ?? 4, Arad = s13?.radiatorArea ?? 4;
    const epsCol = TH.tabulate(TH.surfaceSpectrum(colM)), epsRad = TH.tabulate(TH.surfaceSpectrum(radM));
    const sky = TH.skySpectralEmissivity(TaK, c.eps ?? 0.8).fn, G = c.isDay ? (c.ghi || 0) : 0;
    const cacheA = new Map(), cacheR = new Map();
    const surf = (tab, T, cache) => { const k = Math.round(T * 20); if (!cache.has(k)) cache.set(k, TH.radiativeTerms(tab, k / 20, TaK, sky, G)); return cache.get(k); };
    // Revision 2 (D5): in the day case the hot side is the Suite 13 glazed collector — use its solved q(T) curve so the
    // collector operating temperature found there is carried through the heat path; otherwise the bare-surface model.
    const curveA = p.case === 'day' && s13?.curves?.absorber?.T?.length ? s13.curves.absorber : null;
    const geo = thermalGeometry(ctx.vehicle), archDay = !!(geo?.arch && curveA && geo.noseArea > 0);
    const qA = (T) => interpX(curveA.T, curveA.q, T - KELVIN); // useful collector flux per m² (Suite 13 glazed collector)
    const pgs = get('pgs'), Rstrap = archDay ? p.strapL ** 2 * (pgs?.rho || 2100) / ((pgs?.k || 1500) * p.strapMass) + 1 / (p.hc * Math.max(1e-4, p.Ac / 1e4)) : Infinity, strapSection = archDay ? p.strapMass / ((pgs?.rho || 2100) * p.strapL) : 0;
    // Revision 5.1: the bridge lines cross the yaw flexure pylon (elastic range vehicle.yawLimit, ±25° in the final design)
    // through a coiled 316L pigtail; the number of turns is sized so the bending strain amplitude stays ≤ 0.1 %
    const pigR = 0.04, pigD = 0.006, yawLim = ctx.vehicle.yawLimit ?? 15, pigYaw = yawLim * Math.PI / 180, pigAllow0 = 0.001, pigN = Math.max(4, Math.ceil((pigD / 2) * pigYaw / (2 * Math.PI * pigR * 0.95 * pigAllow0))), pigL = 2 * Math.PI * pigR * pigN, pigStrain = (pigD / 2) * pigYaw / pigL, pigAllow = 0.001; // 316L endurance strain amplitude ≈ 0.1 %
    const lhp = archDay && geo.bridgeArea > 0 ? loopHeatPipe({ fluid: p.lhpFluid, Tsat: Math.max(20, (s13?.Thot_C ?? 100) - 10), dz: geo.lhpDz, L: geo.lhpL + (ctx.vehicle.archRoot !== 'ring' ? 2 * pigL : 0), rPore: p.lhpPore * 1e-6 }) : null;
    // bridge branch: collector temperature Tb such that A_b q(Tb) = min((Tb − T_hi)/R_LHP, Q_max); diode-like (no reverse flow)
    const qBridge = (Thi) => { if (!lhp || !lhp.ok || geo.bridgeArea * qA(Thi) <= 0) return 0; const f = (Tb) => geo.bridgeArea * qA(Tb) - Math.min((Tb - Thi) / lhp.R, lhp.Qmax); const r = TH.brent(f, Thi, Thi + 400); return r.ok ? Math.max(0, Math.min((r.x - Thi) / lhp.R, lhp.Qmax)) : 0; };
    const Qhot = archDay ? (T) => geo.noseArea * qA(T) : curveA ? (T) => Acol * interpX(curveA.T, curveA.q, T - KELVIN) : (T) => { const r = surf(epsCol, T, cacheA); return Acol * (r.solAbs - (r.emit - r.atm) - h * (T - TaK)) + Acol * (p.fin - 1) * h * (TaK - T); };
    const Qcold = (T) => { const r = surf(epsRad, T, cacheR); return Arad * ((r.emit - r.atm) - r.solAbs - h * (TaK - T)) + Arad * (p.fin - 1) * h * (T - TaK); };
    const sp = get(p.spreader), ins = get(p.insulator), hpM = get(ctx.store.material('heatpipe', 'heatpipe-cuw'));
    const kIn = sp.k;
    const tsp = p.tsp / 1000, Asp = p.wsp * tsp, Ac = p.Ac / 1e4, Ains = p.Ains / 1e4, tIns = p.tIns / 1000;
    const Rsp = p.Lsp / (kIn * Asp), Ahp = Math.PI * (p.dHP / 1000) ** 2 / 4; let Rhp = p.nHP > 0 ? p.Lhp / ((hpM?.k || 10000) * Ahp * p.nHP) : Infinity;
    const Rcon = 1 / (p.hc * Ac), Rins = tIns / (ins.k * Ains); let Rpath = 1 / (1 / Rsp + (Number.isFinite(Rhp) ? 1 / Rhp : 0)) + Rcon;
    // Newton solve of the 4-node network: x = [Tcol, Thi, Tci, Trad]
    const solve = (Gc) => {
      const Rh = archDay ? Rstrap : Rpath, F = (x) => { const [Tc_, Th_, Tk_, Tr_] = x, qh = (Tc_ - Th_) / Rh + (archDay ? qBridge(Th_) : 0), qc = (Tk_ - Tr_) / Rpath, qi = Gc * (Th_ - Tk_) + (Th_ - Tk_) / Rins; return [Qhot(Tc_) - (Tc_ - Th_) / Rh, qh - qi, qi - qc, qc - Qcold(Tr_)]; };
      let x = [TaK + 1, TaK + 0.5, TaK - 0.5, TaK - 1]; const hist = [];
      for (let it = 0; it < 60; it++) {
        const f = F(x), nrm = Math.hypot(...f); hist.push(nrm); if (nrm < 1e-7) break;
        const J = range(4).map(() => new Float64Array(4)); for (let k = 0; k < 4; k++) { const d = 1e-4, xp = x.slice(); xp[k] += d; const fp = F(xp); for (let i = 0; i < 4; i++) J[i][k] = (fp[i] - f[i]) / d; }
        let dx; try { dx = luSolve(J, f.map(v => -v)); } catch { break; }
        let lam = 1; const n0 = nrm; for (let ls = 0; ls < 12; ls++) { const xn = x.map((v, i) => v + lam * dx[i]); if (Math.hypot(...F(xn)) < n0) { x = xn; break; } lam /= 2; if (ls === 11) x = x.map((v, i) => v + lam * dx[i]); }
      }
      const [Tc_, Th_, Tk_, Tr_] = x, Qconv = Gc * (Th_ - Tk_), Qleak = (Th_ - Tk_) / Rins;
      const Qn = (Tc_ - Th_) / Rh, Qb = archDay ? qBridge(Th_) : 0;
      return { Tcol: Tc_, Thi: Th_, Tci: Tk_, Trad: Tr_, Qconv, Qleak, Qin: Qn + Qb, Qnose: Qn, Qbridge: Qb, hist, res: hist[hist.length - 1], Wideal: Math.max(0, Qconv * (1 - Tk_ / Th_)) };
    };
    ctx.progress(0.2, 'network');
    let S = solve(p.Gconv);
    let open = solve(1e-6);
    const Gs = logspace(-2, 2.7, 40); let sw = Gs.map(g => solve(g)), iBest = argmax(sw.map(r => r.Wideal));
    // Endoreversible (Curzon–Ahlborn) matching: converter heat draw chosen for maximum work from the delivered heat
    let Guse = p.Gconv; const refineG = () => { const lo = Gs[Math.max(0, iBest - 1)], hi = Gs[Math.min(Gs.length - 1, iBest + 1)]; return Math.exp(goldenMin((x) => -solve(Math.exp(x)).Wideal, Math.log(lo), Math.log(hi), 1e-3)); };
    if (p.matchG !== false) { Guse = refineG(); S = solve(Guse); }
    await ctx.yield();
    // heat-pipe limits at hot path temperature
    const hpT = (archDay ? S.Tci : p.case === 'night' ? S.Thi : S.Tcol) - KELVIN, lim = TH.heatPipeLimits({ TC: hpT, dOuter: p.dHP / 1000, L: p.Lhp, tilt: p.tilt });
    const pathQ = () => archDay ? Math.abs(S.Qconv + S.Qleak) : Math.abs(S.Qin);
    let qPerPipe = p.nHP > 0 ? pathQ() * (1 / Rhp) / (1 / Rsp + 1 / Rhp) / p.nHP : 0, dryOut = false;
    // A heat pipe asked to carry more than its governing limit dries out and conducts only through its copper wall
    // (0.5 mm): re-solve the network with that conductance so no heat is credited beyond the limit.
    if (p.nHP > 0 && qPerPipe > lim.Qmax) { dryOut = true; Rhp = p.Lhp / (390 * Math.PI * (p.dHP / 1000) * 0.0005 * p.nHP); Rpath = 1 / (1 / Rsp + 1 / Rhp) + Rcon; S = solve(Guse); open = solve(1e-6); sw = Gs.map(g => solve(g)); iBest = argmax(sw.map(r => r.Wideal)); qPerPipe = pathQ() * (1 / Rhp) / (1 / Rsp + 1 / Rhp) / p.nHP; }
    ctx.progress(0.45, '2-D field');
    // 2-D anisotropic FV field: hot block | isolation | cold block (x), spreader thickness (y)
    const nH = 18, nI = 6, nx = 2 * nH + nI, ny = 10, Lx = 2 * p.Lsp + Math.max(tIns, 0.005), dxH = p.Lsp / nH, dxI = Math.max(tIns, 0.005) / nI;
    // use uniform dx for the solver; map the isolation gap to nI cells of the same width (geometry scaled, conductance preserved)
    const dx = dxH, dy = tsp / ny, kx = new Float64Array(nx * ny), ky = new Float64Array(nx * ny), rc = new Float64Array(nx * ny);
    const kInsEff = ins.k * dxH / dxI; // preserves the isolation conductance on the uniform grid
    for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) { const k = j * nx + i, isIns = i >= nH && i < nH + nI; kx[k] = isIns ? kInsEff : kIn; ky[k] = isIns ? kInsEff : kIn / (p.anis || 1); rc[k] = isIns ? ins.rho * ins.cp : sp.rho * sp.cp; }
    const qTopHot = S.Qin / (p.Lsp * p.wsp), Tci = S.Tci, Thi = S.Thi;
    const hEffRad = Math.abs(S.Qin) / Math.max(0.05, Math.abs(S.Tci - S.Trad)) / (p.Lsp * p.wsp);
    const field = TH.conduction2D({ nx, ny, dx, dy, kx, ky, rc }, {
      top: { type: 'mixed', fn: (i) => i < nH ? { type: 'q', q: qTopHot } : i >= nH + nI ? { type: 'h', h: hEffRad, Tinf: S.Trad } : null },
      bottom: { type: 'mixed', fn: (i) => i < nH ? { type: 'h', h: Guse / (p.Lsp * p.wsp) + 1e-9, Tinf: Tci } : i >= nH + nI ? { type: 'q', q: S.Qconv / (p.Lsp * p.wsp) } : null },
    }, { T0: TaK, tol: 1e-7, maxIt: 30000 });
    const Z = range(ny).map(j => range(nx).map(i => field.T[j * nx + i] - KELVIN));
    const xsF = range(nx).map(i => (i + 0.5) * dx * 1000), ysF = range(ny).map(j => (j + 0.5) * dy * 1000);
    const U = range(ny).map(j => range(nx).map(i => { const k = j * nx + i, a = field.T[k - (i > 0 ? 1 : 0)], b = field.T[k + (i < nx - 1 ? 1 : 0)]; return -kx[k] * (b - a) / (dx * ((i > 0) + (i < nx - 1))); }));
    const V = range(ny).map(j => range(nx).map(i => { const k = j * nx + i, a = field.T[k - (j > 0 ? nx : 0)], b = field.T[k + (j < ny - 1 ? nx : 0)]; return -ky[k] * (b - a) / (dy * ((j > 0) + (j < ny - 1))); }));
    const faceFlux = (i) => { let q = 0; for (let j = 0; j < ny; j++) { const k = j * nx + i; q += field.Gx[k] * (field.T[k] - field.T[k + 1]) * p.wsp; } return q; };
    const leakA = faceFlux(nH), leakB = faceFlux(nH + nI - 2);
    ctx.progress(0.7, 'transient');
    // transient 2-node network (hot block, cold block) from ambient
    const Chot = sp.rho * sp.cp * Asp * p.Lsp + 2700 * 900 * Acol * 0.001, Ccold = sp.rho * sp.cp * Asp * p.Lsp + 2700 * 900 * Arad * 0.001;
    const tEnd = Math.min(20000, 8 * Math.max(Chot, Ccold) * (Rpath + 0.2)), tr = rk45((t, y) => { const [Th_, Tk_] = y, Tcol = Th_, Trad = Tk_, qi = Guse * (Th_ - Tk_) + (Th_ - Tk_) / Rins; return [(Qhot(Tcol) - qi) / Chot, (qi - Qcold(Trad)) / Ccold]; }, 0, [TaK, TaK], tEnd, { rtol: 1e-6, hmax: tEnd / 200 });
    const dTss = tr.y[tr.y.length - 1][0] - tr.y[tr.y.length - 1][1], i63 = tr.y.findIndex(y => (y[0] - y[1]) >= 0.632 * dTss), tau = i63 > 0 ? tr.t[i63] : NaN;
    // verification
    const slab = TH.conduction2D({ nx: 20, ny: 1, dx: 0.01, dy: 0.01, kx: new Float64Array(20).fill(10), ky: new Float64Array(20).fill(10), rc: new Float64Array(20).fill(1e6) }, { left: { type: 'T', T: 1 }, right: { type: 'T', T: 0 } }, { T0: 0.5, tol: 1e-12, maxIt: 50000 });
    const slabErr = Math.max(...range(20).map(i => Math.abs(slab.T[i] - (1 - (i + 0.5) / 20))));
    out.checks.push({ name: '1-D slab: FV vs analytic linear profile', value: slabErr, target: '< 1e-6 K', pass: slabErr < 1e-6 });
    out.checks.push({ name: 'Network energy residual (Newton)', value: S.res, target: '< 1e-6 W', pass: S.res < 1e-5, detail: `${S.hist.length} iterations` });
    out.checks.push({ name: 'Node balance: Q_in = Q_conv + Q_leak', value: S.Qin - S.Qconv - S.Qleak, target: '≈ 0 W', pass: Math.abs(S.Qin - S.Qconv - S.Qleak) < 1e-4 * Math.max(1, Math.abs(S.Qin)) });
    out.checks.push({ name: '2-D flux conservation across the isolation (entry vs exit face)', value: leakA / (leakB || 1e-30), target: '1 ± 1e-6', pass: Math.abs(leakA / (leakB || 1e-30) - 1) < 1e-6, detail: `${fmt(leakA, 4)} W vs ${fmt(leakB, 4)} W` });
    out.checks.push({ name: '2-D steady solution residual (PCG)', value: field.residual, target: '< 1e-7 K', pass: field.residual < 1e-7, detail: `${field.iterations} CG iterations` });
    const lc = rk45((t, y) => [-(y[0] - 0) / 10], 0, [1], 30, { rtol: 1e-10, atol: 1e-12 }), lcErr = Math.abs(lc.y[lc.y.length - 1][0] - Math.exp(-3)); out.checks.push({ name: 'Lumped capacitance vs exp(−t/τ)', value: lcErr, target: '< 1e-6', pass: lcErr < 1e-6 });
    // KPIs
    const dTi = S.Thi - S.Tci, dTopen = open.Thi - open.Tci;
    const K = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    K('Actuator hot-interface temperature', S.Thi - KELVIN, '°C', 'Solved temperature at the actuator hot face (not ambient, not collector).', 'calc');
    K('Actuator cold-interface temperature', S.Tci - KELVIN, '°C', 'Solved temperature at the actuator cold face (not sky temperature).', 'calc');
    K('Converter heat draw G_conv (matched)', Guse, 'W/K', p.matchG !== false ? `Curzon–Ahlborn matching: G_conv that maximises ideal work Q·(1 − T_c,i/T_h,i) for this collector/strap/radiator network (sweep + golden-section refinement). The actuator array (Suites 15–17) is sized to draw this.` : 'User / Suite 15 value.', p.matchG !== false ? 'generated' : 'user');
    K('Interface ΔT under load', dTi, 'K', `Temperature difference actually available to the actuator while it draws ${fmt(Guse, 3)} W/K.`, 'calc', dTi > 15 ? 'ok' : dTi > 5 ? 'warn' : 'bad');
    K('Open-circuit interface ΔT', dTopen, 'K', 'ΔT with no heat drawn by the actuator — an upper bound that collapses once heat flows.', 'calc');
    K('Heat delivered to actuator', S.Qconv, 'W', 'Heat flow through the converter; the work it can make is bounded by Carnot of the interface temperatures.', 'calc');
    K('Heat leakage through isolation', S.Qleak, 'W', 'Heat bypassing the actuator from the hot to the cold side — wasted exergy.', 'calc', S.Qleak < 0.1 * Math.abs(S.Qconv) ? 'ok' : 'warn');
    K('Ideal work from delivered heat', S.Wideal, 'W', 'Q_conv(1 − T_ci/T_hi): ceiling for any converter at this operating point.', 'calc');
    K('Best ideal work (matched G_conv)', sw[iBest].Wideal, 'W', `Achieved at G_conv ≈ ${fmt(Gs[iBest], 3)} W/K (thermal impedance matching).`, 'calc');
    K('Path thermal resistance (each side)', Rpath, 'K/W', 'Spreader ∥ heat pipes, in series with the contact resistance.', 'calc');
    K('Collector / exchanger temperature', S.Tcol - KELVIN, '°C', p.case === 'day' || c.isDay ? 'Absorber temperature under load.' : 'Air-side exchanger temperature (night).', 'calc');
    K('Radiator temperature', S.Trad - KELVIN, '°C', 'Radiator temperature while rejecting the converter heat.', 'calc');
    K('Heat per pipe / heat-pipe limit', p.nHP ? `${fmt(qPerPipe, 3)} / ${fmt(lim.Qmax, 3)}` : 'no heat pipes', 'W', `Governing limit: ${lim.governing}. ${lim.frozen ? 'Working fluid at/below freezing!' : ''}`, 'calc', p.nHP && qPerPipe > lim.Qmax ? 'bad' : 'ok');
    K('Warm-up time constant', tau, 's', 'Time to reach 63 % of steady interface ΔT from an isothermal start.', 'calc');
    // verdicts
    out.verdicts.push({ test: 'Interface temperatures solved (not assumed)', status: 'pass', detail: `T_hi = ${fmt(S.Thi - KELVIN, 4)} °C vs ambient ${fmt(c.T, 4)} °C; T_ci = ${fmt(S.Tci - KELVIN, 4)} °C (${c.src}).` });
    const actM = get(ctx.store.project.materials?.actuator || 'nitcu') || get('nitcu'), s15o = ctx.upstream('s15'), needDT = s15o?.minDT_K ?? ((actM.Af - actM.Mf) + 6 + 150 / (actM.CA || 7) - 20 / (actM.CM || 6.5));
    K('Achievable ΔT vs actuator requirement', `${fmt(dTi, 3)} / ${fmt(needDT, 3)}`, 'K', `${actM.name}: hysteresis (A_f − M_f = ${fmt(actM.Af - actM.Mf, 3)} K) + stress shift + 2 × 3 K margins. ${s15o?.minDT_K ? 'From Suite 15.' : 'Estimated at σ_H = 150, σ_L = 20 MPa.'}`, s15o ? 'coupled' : 'estimated', dTi >= needDT ? 'ok' : 'bad');
    if (curveA) K('Collector curve source', 'Suite 13 glazed collector', '', `Hot-side heat q(T) taken from Suite 13 (exergy-optimal point ${fmt(s13.Thot_C, 4)} °C); the network solves the actual collector temperature under the actuator draw.`, 'coupled');
    out.verdicts.push({ test: 'Usable actuator ΔT', status: dTi >= needDT ? 'pass' : dTi > 0.5 * needDT ? 'warn' : 'fail', detail: `${fmt(dTi, 3)} K under load vs ${fmt(needDT, 3)} K needed by ${actM.name} (hysteresis + stress shift + margins). Below that the alloy cannot complete a transformation cycle.`, fix: 'Reduce path resistance (more heat pipes, better interface material), enlarge collector/radiator areas, or lower the actuator draw.' });
    if (archDay) {
      K('Hot sources in parallel (nose strap / bridge LHP)', `${fmt(S.Qnose, 4)} / ${fmt(S.Qbridge, 4)}`, 'W', `Nose collector ${fmt(geo.noseArea, 3)} m² over a ${fmt(p.strapL, 3)} m pyrolytic-graphite strap (${fmt(p.strapMass * 1000, 3)} g, section ${fmt(strapSection * 1e4, 3)} cm², R = ${fmt(Rstrap, 3)} K/W, no pumping); bridge collector ${fmt(geo.bridgeArea, 3)} m² through a loop heat pipe (${fmt(geo.lhpL, 3)} m line, ${fmt(geo.lhpDz, 3)} m adverse elevation). Distances from the current CAD.`, 'calc');
      if (lhp) K('Loop-heat-pipe capillary budget', `${fmt(lhp.dPcap / 1000, 3)} kPa head − ${fmt(lhp.dPhyd / 1000, 3)} kPa hydrostatic → Q_max ${fmt(lhp.Qmax, 4)} W`, '', `${p.lhpFluid}, r_p = ${p.lhpPore} µm; flow losses ${fmt(lhp.perW, 3)} Pa/W (liquid ${fmt(lhp.cL, 3)}, vapour ${fmt(lhp.cV, 3)}, wick ${fmt(lhp.cW, 3)}); thermal resistance ${fmt(lhp.R, 3)} K/W. ${lhp.note} ${lhp.startup}`, 'calc', lhp.ok && S.Qbridge < lhp.Qmax ? 'ok' : 'bad');
      out.verdicts.push({ test: 'Bridge loop heat pipe operates against gravity', status: !lhp ? 'info' : lhp.ok && S.Qbridge <= lhp.Qmax ? 'pass' : 'fail', detail: lhp ? `${fmt(S.Qbridge, 4)} W transported vs Q_max ${fmt(lhp.Qmax, 4)} W; hydrostatic head uses ${fmt(100 * lhp.dPhyd / lhp.dPcap, 3)} % of the ${fmt(lhp.dPcap / 1000, 3)} kPa capillary pressure. ${lhp.note}` : 'No bridge collector.', fix: 'Finer primary wick, larger liquid line, water above 60 °C (ammonia only below 90 °C).' });
    }
    if (archDay && lhp && ctx.vehicle.archRoot !== 'ring') { K('LHP pigtail across the flexure pylon', `${fmt(pigStrain * 100, 3)} % strain amplitude (allow ${fmt(pigAllow * 100, 3)} %)`, '', `${pigN}-turn 316L coil (turns sized for ±${fmt(yawLim, 3)}° yaw), Ø${pigD * 1000} mm tube on R ${pigR * 1000} mm, the elastic yaw distributed over ${fmt(pigL, 3)} m of tube (vapour and liquid lines, both counted in the loop pressure budget).`, 'calc', pigStrain <= pigAllow ? 'ok' : 'bad');
      out.verdicts.push({ test: 'Heat-pipe pigtail fatigue across the flexure', status: pigStrain <= pigAllow ? 'pass' : 'fail', detail: `Bending strain amplitude ${fmt(pigStrain * 100, 3)} % vs ≈ 0.1 % endurance for annealed 316L (≥ 10⁷ yaw cycles).`, fix: 'More turns or a larger coil radius.' }); }
    if (dryOut) K('Heat pipes dried out', 'yes — wall conduction only', '', `Demand exceeded the ${lim.governing} limit (${fmt(lim.Qmax, 3)} W per pipe at ${p.tilt}° adverse tilt over ${p.Lhp} m); the network was re-solved with copper-wall conduction only. `, 'calc', 'bad');
    out.verdicts.push({ test: 'Heat-pipe operating limits', status: !p.nHP ? 'info' : lim.frozen ? 'fail' : qPerPipe > lim.Qmax ? 'fail' : 'pass', detail: p.nHP ? `${fmt(qPerPipe, 3)} W per pipe vs ${lim.governing} limit ${fmt(lim.Qmax, 3)} W at ${fmt(hpT, 3)} °C.` : 'Heat pipes not used.', fix: 'Add pipes, use larger diameter or finer wick, avoid adverse tilt; water pipes freeze below 0 °C — use another working fluid in cold climates.' });
    out.verdicts.push({ test: 'Isolation effectiveness', status: S.Qleak < 0.1 * Math.abs(S.Qconv) ? 'pass' : 'warn', detail: `Leakage ${fmt(S.Qleak, 3)} W = ${fmt(100 * S.Qleak / Math.max(1e-9, Math.abs(S.Qconv)), 3)} % of converter heat.` });
    // charts
    out.charts.push({ title: '2-D conduction field: hot block | isolation | cold block (°C)', type: 'heatmap', wide: true, x: { label: 'Along spreader', unit: 'mm', values: xsF }, y: { label: 'Thickness', unit: 'mm', values: ysF }, z: Z, cmap: 'coolwarm', isolines: 8, vectors: { u: U, v: V, stride: 2 }, z_label: '°C', explain: 'Finite-volume temperature field with heat-flux vectors (thickness exaggerated). Heat enters at top-left, is drawn into the actuator along the bottom-left, and leaks through the central isolation into the cold block, which rejects it to the radiator at top-right. Anisotropy reduces through-thickness gradients.' });
    out.charts.push({ title: 'Thermal impedance matching: converter draw vs available ΔT and ideal work', type: 'line', x: { label: 'Actuator thermal draw G_conv', unit: 'W/K', log: true }, y: { label: 'Value' }, series: [{ name: 'Interface ΔT (K)', x: Gs, y: sw.map(r => r.Thi - r.Tci) }, { name: 'Heat to actuator (W)', x: Gs, y: sw.map(r => r.Qconv) }, { name: 'Ideal work ×10 (W)', x: Gs, y: sw.map(r => 10 * r.Wideal) }], annotations: [{ type: 'vline', x: Guse, label: 'current' }, { type: 'vline', x: Gs[iBest], label: 'best', color: '#22c55e' }], explain: 'Drawing more heat lowers the interface ΔT. Ideal work peaks where the actuator conductance matches the external path — the thermal equivalent of impedance matching.' });
    out.charts.push({ title: 'Warm-up transient (2-node network)', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Temperature', unit: '°C' }, series: [{ name: 'Hot side', x: tr.t, y: tr.y.map(y => y[0] - KELVIN) }, { name: 'Cold side', x: tr.t, y: tr.y.map(y => y[1] - KELVIN) }], annotations: Number.isFinite(tau) ? [{ type: 'vline', x: tau, label: 'τ (63 %)' }] : [], explain: 'How long the thermal path takes to establish its steady gradient after start-up or a change in weather.' });
    const lims = ['capillary', 'sonic', 'entrainment', 'viscous', 'boiling'];
    const Tsw = linspace(2, 90, 30); out.charts.push({ title: 'Heat-pipe operating envelope (per pipe)', type: 'line', x: { label: 'Vapour temperature', unit: '°C' }, y: { label: 'Limit', unit: 'W', log: true }, series: lims.map(n => ({ name: n, x: Tsw, y: Tsw.map(T => Math.max(1e-3, TH.heatPipeLimits({ TC: T, dOuter: p.dHP / 1000, L: p.Lhp, tilt: p.tilt })[n])) })), annotations: p.nHP ? [{ type: 'point', x: hpT, y: Math.max(1e-3, qPerPipe), label: 'operating' }] : [], explain: 'The lowest curve at the operating temperature governs. Low-temperature operation is usually capillary- or viscous-limited.' });
    out.charts.push({ title: 'Network solution convergence', type: 'line', x: { label: 'Newton iteration' }, y: { label: 'Residual', unit: 'W', log: true }, series: [{ name: '‖F‖', x: range(S.hist.length), y: S.hist.map(v => Math.max(v, 1e-14)) }], explain: 'Quadratic Newton convergence of the nonlinear radiative–conductive network.' });
    out.tables.push({ title: 'Thermal resistance network', cols: ['Element', 'Value', 'Unit'], rows: [['Spreader (in-plane)', Rsp, 'K/W'], ['Heat pipes (parallel)', Number.isFinite(Rhp) ? Rhp : '—', 'K/W'], ['Contact', Rcon, 'K/W'], ['Path total (per side)', Rpath, 'K/W'], ['Isolation leak path', Rins, 'K/W'], ['Converter 1/G', 1 / Guse, 'K/W']] });
    out.tables.push({ title: 'Node temperatures and heat flows', cols: ['Quantity', 'Value', 'Unit'], rows: [['Ambient', c.T, '°C'], ['Collector / exchanger', S.Tcol - KELVIN, '°C'], ['Actuator hot interface', S.Thi - KELVIN, '°C'], ['Actuator cold interface', S.Tci - KELVIN, '°C'], ['Radiator', S.Trad - KELVIN, '°C'], ['Heat in from collector', S.Qin, 'W'], ['Heat to actuator', S.Qconv, 'W'], ['Leakage', S.Qleak, 'W']] });
    // recs
    if (dTi < 10) out.recs.push({ level: 'critical', text: `Only ${fmt(dTi, 3)} K reaches the actuator. ${p.case === 'night' ? 'At night this is an environmental limit (sky radiative sink), not just a design flaw.' : 'Reduce contact resistance and add heat-transport capacity.'}`, basis: 'Coupled network solution' });
    if (Rcon > 0.3 * Rpath) out.recs.push({ level: 'warn', text: `Contact resistance is ${fmt(100 * Rcon / Rpath, 3)} % of the path; specify a compliant thermal interface material and controlled clamping pressure (h_c ≥ 20 kW/m²K).`, basis: 'Resistance breakdown' });
    if (Math.abs(Math.log10(Guse / Gs[iBest])) > 0.3) out.recs.push({ level: 'info', text: `Re-size the actuator heat draw towards ≈ ${fmt(Gs[iBest], 3)} W/K (cartridge count × frequency × thermal mass) to maximise ideal work.`, basis: 'Impedance matching sweep' });
    if (p.anis < 5) out.recs.push({ level: 'info', text: 'An anisotropic thermal metamaterial (graphite-laminate or aligned lattice, k_in/k_through ≥ 20) channels heat to the interfaces while suppressing cross-leakage.', basis: 'Anisotropic conduction' });
    out.recs.push({ level: 'sustain', text: 'Aluminium spreaders and copper/water heat pipes are highly recyclable; aerogel insulation is energy-intensive to produce — size it to the leakage target rather than over-insulating.', basis: 'Embodied energy in materials registry' });
    out.explain.push({ title: 'Why interface temperatures matter', text: 'The actuator does not see ambient air or the sky. It sees the ends of a thermal path whose resistances, leakage and radiative boundaries reduce the available temperature difference — especially once heat actually flows. These solved interface temperatures feed Suites 15 and 17.' });
    out.views.push({ label: '2D animation: transient temperature field', icon: '🎞', render: (el) => anim2dField(el, { tr, Z, nx, ny, nH, nI, TaK, S }) });
    out.views.push({ label: '3D heat path (transient)', icon: '🌡', render: (el) => view3dHeatPath(el, { S, tr, Z, nx, ny, nH, nI, TaK, spName: sp.name, insName: ins.name, nHP: p.nHP }) });
    const SB = await storeBlock(p, ctx, { Ta: c.T, s13 }), storeOn = !!SB && p.hotSource === 'store';
    if (SB) renderStore(out, SB, p, K);
    if (storeOn) {
      const v = out.verdicts.find(x => x.test === 'Usable actuator ΔT'), dTs = SB.opM.feasible ? SB.opM.Th - SB.opM.Tc : 0;
      if (v) { v.test = 'Usable actuator ΔT (store-fed hub)'; v.status = dTs >= needDT ? 'pass' : dTs > 0.5 * needDT ? 'warn' : 'fail'; v.detail = `${fmt(dTs, 3)} K between the store-fed hot arc (${fmt(SB.opM.Th, 4)} °C) and the cold arc (${fmt(SB.opM.Tc, 4)} °C) vs ${fmt(needDT, 3)} K needed. The direct collector path alone gives ${fmt(dTi, 3)} K in this case.`; }
      for (const x of out.verdicts) if (/Heat-pipe operating limits|Isolation effectiveness|Bridge loop heat pipe|pigtail/.test(x.test)) x.test += ' (direct collector path)';
    }
    out.outputs = { pigtailTurns: pigN, Qhot_W: S.Qconv, Qleak_W: S.Qleak, ThotIface_C: S.Thi - KELVIN, TcoldIface_C: S.Tci - KELVIN, Rth_K_W: 2 * Rpath, tauThermal_s: tau, Gconv_W_K: Guse, case: p.case, Wideal_W: S.Wideal, dTopen_K: dTopen, TambC: c.T, bestG_W_K: Gs[iBest] };
    if (!storeOn) Object.assign(out.outputs, { finDrag_N: 0, finDragCdA_m2: 0, hotSource: 'direct', alloyHint: S.Thi - KELVIN > 125 ? 'nitihf' : undefined });
    if (SB && !storeOn) Object.assign(out.outputs, { hotSource: 'direct', thermalSystemMass_kg: 0, researchStore: { backupDemo_h: SB.dur.demo, backupModel_h: SB.dur.model, backupTarget_h: SB.dur.target, fracNeeded: SB.fracNeeded, etaDemo: SB.ops.demo.eta ?? 0, req1h: Object.fromEntries(Object.entries(SB.req).map(([k, r]) => [k, r ? { thermalSystem_kg: r.tot, vehicle_kg: SB.base + r.tot, overCap_kg: SB.base + r.tot - p.massCap } : null])), installedMass_kg: SB.sysMass } });
    if (storeOn) Object.assign(out.outputs, { hotSource: 'store', directQhot_W: S.Qconv, Qhot_W: SB.opM.feasible ? 1.2 * SB.inst.Qhp : 0, ThotIface_C: SB.opM.feasible ? SB.opM.Th : S.Thi - KELVIN, TcoldIface_C: SB.opM.feasible ? SB.opM.Tc : S.Tci - KELVIN, alloyHint: SB.alloyHint, pcm: SB.pcm.id,
      storePcm_kg: SB.inst.mPcm, storeVessel_kg: SB.inst.mPcm > 0 ? SB.inst.V.mass : 0, rejectFin_m2: SB.inst.Af, rejectFin_kg: SB.inst.rej.finMass, finDrag_W: SB.inst.rej.Pdrag, finDrag_N: SB.inst.rej.drag, finDragCdA_m2: SB.inst.rej.drag / (0.5 * 1.18 * (p.vSize / 3.6) ** 2), finDragRef_kmh: p.vSize, finDragExp: 1.5, heatPipes_kg: SB.inst.hp, collectorUpgrade_kg: SB.collExt, thermalSystemMass_kg: SB.sysMass, storeEnergy_J: SB.Einst, storeLoss_W: SB.inst.V.qLeak, dailyHeat_J: SB.dailyJ, chargeFull_h: SB.tFull,
      backupDemo_h: SB.dur.demo, backupModel_h: SB.dur.model, backupTarget_h: SB.dur.target, backupDemo24_h: SB.dur24.demo, heatDemo_W: SB.ops.demo.Q, heatModel_W: SB.ops.model.Q, heatTarget_W: SB.ops.target.Q, etaDemo: SB.ops.demo.eta ?? 0, etaModelBasis: SB.ops.model.eta ?? 0, etaTarget: SB.ops.target.eta ?? 0, fracModel: SB.fracModel, demoMaxWheel_W: SB.demoMax.W * 0.85, demoMaxSpeed_kmh: SB.demoMax.vKmh, demoMax_h: SB.demoMax.hours,
      req1h: Object.fromEntries(Object.entries(SB.req).map(([k, r]) => [k, r ? { thermalSystem_kg: r.tot, pcm_kg: r.st.mPcm, vehicle_kg: SB.base + r.tot, overCap_kg: SB.base + r.tot - p.massCap, heat_W: r.op.Q, fin_m2: r.Af, drag_W: r.rej.Pdrag } : null])), vehicleBase_kg: SB.base, storePosition: SB.position, roadLoad_W: SB.Proad });
    out.convergence = { title: 'Newton residual history', series: [{ name: 'network', x: range(S.hist.length), y: S.hist.map(v => Math.max(v, 1e-14)) }] };
    out.headline = `${p.case === 'day' ? 'Day' : p.case === 'night' ? 'Night' : 'Current'} case: actuator interfaces at ${fmt(S.Thi - KELVIN, 4)} °C / ${fmt(S.Tci - KELVIN, 4)} °C (ΔT ${fmt(dTi, 3)} K) delivering ${fmt(S.Qconv, 3)} W of heat; ideal work ≤ ${fmt(S.Wideal, 3)} W.`;
    out.headlineKind = dTi > 15 ? 'ok' : 'warn';
    if (storeOn) { out.headline = `Store-fed hub: ${fmt(SB.inst.mPcm, 3)} kg ${SB.pcm.name.split(' (')[0]} (${fmt(SB.Einst / 1e6, 3)} MJ) → ${fmt(SB.dur.demo, 3)} h at ${p.vSize} km/h on the demonstrated basis (${fmt(SB.dur.model, 3)} h model, ${fmt(SB.dur.target, 3)} h research target); 1 h + ${p.margin} % on the demonstrated basis needs ${SB.req.demo ? fmt(SB.req.demo.tot, 3) + ' kg' : 'infeasible rejection'}.`; out.headlineKind = SB.dur.demo >= p.backupH * (1 + p.margin / 100) ? 'ok' : 'warn'; }
    return out;
  },
};

// 3D heat-path assembly: collector → hot spreader → actuator → cold spreader → radiator, with the insulation gap.
// Node temperatures vs time are the solved RK45 transient; the in-spreader distribution is the solved steady 2-D
// finite-volume field scaled by the node response at each instant (interpolated between solved states).
async function view3dHeatPath(el, { S, tr, Z, nx, ny, nH, nI, TaK, spName, insName, nHP }) {
  const [{ viewer }, { playback, hud, legend3d, fieldMesh, tracers }, { box, loft }, { cmap }, { h }] = await Promise.all([import('../core/viz3d.js'), import('../core/sim3d.js'), import('../core/cad.js'), import('../core/chart.js'), import('../core/ui.js')]);
  el.append(h('p', { class: 'small muted' }, `Solved: transient hot/cold node temperatures (RK45 network model) and the steady 2-D anisotropic temperature field in the ${spName} spreaders across the ${insName} isolation (finite-volume, PCG). The displayed field at time t is the steady field scaled by each side's solved transient response; geometry is schematic, thicknesses exaggerated ×10. Arrows: heat flows (length ∝ W).`));
  const wrap = h('div'); el.append(wrap);
  const V = await viewer(wrap, { height: 440, grid: false }); const THREE = V.THREE;
  const Lh = 0.8, Lg = 0.8 * nI / nH, T = 0.12; // visual lengths (m) of hot block, insulation gap and slab thickness
  const x0 = -(2 * Lh + Lg) / 2, xH1 = x0 + Lh, xC0 = xH1 + Lg, xC1 = xC0 + Lh;
  const slab = (name, xa, xb, nSeg) => { const secs = []; for (let k = 0; k <= nSeg; k++) { const x = xa + (xb - xa) * k / nSeg; const ring = []; for (let j = 0; j <= 6; j++) ring.push([x, -0.25, j * T / 6]); for (let j = 1; j <= 4; j++) ring.push([x, -0.25 + 0.5 * j / 4, T]); for (let j = 5; j >= 0; j--) ring.push([x, 0.25, j * T / 6]); for (let j = 3; j >= 1; j--) ring.push([x, -0.25 + 0.5 * j / 4, 0]); secs.push(ring); } return loft(name, secs, { color: '#999' }); };
  const ssK = (i, j) => Z[Math.max(0, Math.min(ny - 1, j))][Math.max(0, Math.min(nx - 1, i))] + KELVIN0;
  const KELVIN0 = 273.15;
  const Th_ss = tr.y[tr.y.length - 1][0], Tc_ss = tr.y[tr.y.length - 1][1];
  let fH = 1, fC = 1;
  const valAt = (x, y, z) => { const i = x < xH1 ? Math.floor((x - x0) / Lh * nH) : x < xC0 ? nH + Math.floor((x - xH1) / Lg * nI) : nH + nI + Math.floor((x - xC0) / Lh * nH); const j = Math.floor(z / T * ny); const Tss = ssK(i, j), side = x < (xH1 + xC0) / 2 ? fH : fC; return TaK + (Tss - TaK) * side - KELVIN0; };
  const all = Z.flat(), lo = Math.min(...all, TaK - KELVIN0, Tc_ss - KELVIN0, S.Trad - KELVIN0), hi = Math.max(...all, TaK - KELVIN0, Th_ss - KELVIN0, S.Tcol - KELVIN0);
  const fm = fieldMesh(V, slab('Spreaders + isolation (2-D field)', x0, xC1, 3 * (nH + nI)), valAt, { cmapName: 'coolwarm', range: [lo, hi] });
  const add = (m, color) => { const o = V.add(m); o.material.color.set(color); return o; };
  const col = box('Collector / hot face', Lh * 0.95, 0.6, 0.03, { color: '#f97316' }); col.translate(x0 + Lh / 2, 0, T + 0.45); const oCol = add(col, '#f97316');
  const rad = box('Radiator / cold face', Lh * 0.95, 0.6, 0.03, { color: '#38bdf8' }); rad.translate(xC0 + Lh / 2, 0, T + 0.45); const oRad = add(rad, '#38bdf8');
  const legs = []; for (const [xa, name] of [[x0 + Lh / 2, 'Hot conduction path' + (nHP ? ` (+${nHP} heat pipes)` : '')], [xC0 + Lh / 2, 'Cold conduction path']]) { const b = box(name, 0.08, 0.08, 0.42, { color: '#b8c0cc' }); b.translate(xa, 0, T + 0.22); legs.push(add(b, '#b8c0cc')); }
  const act = box('Actuator (NiTi cartridge) between interfaces', xC0 - xH1 + 0.5, 0.3, 0.1, { color: '#c58b4b' }); act.translate((xH1 + xC0) / 2, 0, -0.07); const oAct = add(act, '#c58b4b');
  const arrow = (from, dir, color) => { const a = new THREE.ArrowHelper(new THREE.Vector3(...dir).normalize(), new THREE.Vector3(...from), 0.3, color, 0.1, 0.06); V.root.add(a); return a; };
  const aIn = arrow([x0 + Lh / 2 + 0.15, 0.32, T + 0.42], [0, 0, -1], 0xf97316), aConv = arrow([xH1 - 0.1, 0.32, -0.02], [1, 0, 0], 0xc58b4b), aLeak = arrow([xH1, 0, T + 0.06], [1, 0, 0], 0xef4444), aOut = arrow([xC0 + Lh / 2 + 0.15, 0.32, T + 0.05], [0, 0, 1], 0x38bdf8);
  const trHot = tracers(V, [[x0 + Lh / 2, 0.1, T + 0.45], [x0 + Lh / 2, 0.1, T], [xH1, 0.1, T / 2], [xH1 + 0.05, 0.1, -0.07], [xC0 - 0.05, 0.1, -0.07], [xC0, 0.1, T / 2], [xC0 + Lh / 2, 0.1, T], [xC0 + Lh / 2, 0.1, T + 0.45]], { n: 14, color: 0xf97316 });
  const trLeak = tracers(V, [[xH1, -0.1, T / 2], [xC0, -0.1, T / 2]], { n: 4, color: 0xef4444, size: 0.018 });
  let lastT = 0;
  V.viewDir(0.35, -1.1, 0.6); V.controls.autoRotate = true; V.controls.autoRotateSpeed = 2; V.renderer.domElement.addEventListener('pointerdown', () => { V.controls.autoRotate = false; });
  const H = hud(wrap); legend3d(wrap, { title: 'Temperature', lo, hi, unit: ' °C', cmapName: 'coolwarm' });
  const tint = (o, Tc) => { const c = cmap((Tc - lo) / ((hi - lo) || 1), 'coolwarm'); o.material.color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255); };
  const tEnd = tr.t[tr.t.length - 1], Qscale = Math.max(1e-6, Math.abs(S.Qin), Math.abs(S.Qconv), Math.abs(S.Qleak));
  playback(wrap, { duration: tEnd, rate: tEnd / 20, label: 'Time from start-up', unit: 's', onFrame: (t) => {
    const Th = interpT(tr.t, tr.y.map(y => y[0]), t), Tc = interpT(tr.t, tr.y.map(y => y[1]), t);
    fH = (Th - TaK) / ((Th_ss - TaK) || 1); fC = (Tc - TaK) / ((Tc_ss - TaK) || 1); if (!Number.isFinite(fH)) fH = 1; if (!Number.isFinite(fC)) fC = 1;
    fm.update(valAt, [lo, hi]); tint(oCol, Th - KELVIN0); tint(oRad, Tc - KELVIN0); tint(legs[0], Th - KELVIN0); tint(legs[1], Tc - KELVIN0); tint(oAct, (Th + Tc) / 2 - KELVIN0);
    const g = Math.max(0, Math.min(1, Math.abs(Th - TaK) / (Math.abs(Th_ss - TaK) || 1))); const len = (q) => 0.12 + 0.5 * Math.abs(q) / Qscale * g;
    { const dt = Math.max(0, Math.min(0.1, (performance.now() - lastT) / 1000)); lastT = performance.now(); trHot.step(dt, 0.08 + 0.35 * Math.abs(S.Qconv) / Qscale * Math.max(0.05, g)); trLeak.step(dt, 0.05 + 0.35 * Math.abs(S.Qleak) / Qscale * Math.max(0.05, g)); }
    aIn.setLength(len(S.Qin), 0.1, 0.06); aConv.setLength(len(S.Qconv), 0.1, 0.06); aLeak.setLength(len(S.Qleak), 0.1, 0.06); aOut.setLength(len(S.Qconv), 0.1, 0.06);
    H.set([['Time', fmt(t, 3) + ' s'], ['Hot node', fmt(Th - KELVIN0, 4) + ' °C', '#f97316'], ['Cold node', fmt(Tc - KELVIN0, 4) + ' °C', '#38bdf8'], ['ΔT hot–cold', fmt(Th - Tc, 3) + ' K'], ['Steady Q_in', fmt(S.Qin, 3) + ' W'], ['Steady Q_conv', fmt(S.Qconv, 3) + ' W'], ['Steady Q_leak', fmt(S.Qleak, 3) + ' W', '#ef4444'], ['Response', fmt(100 * g, 3) + ' % of steady']]);
  } });
}
function interpT(ts, ys, t) { if (t <= ts[0]) return ys[0]; for (let i = 1; i < ts.length; i++) if (ts[i] >= t) { const f = (t - ts[i - 1]) / ((ts[i] - ts[i - 1]) || 1); return ys[i - 1] + f * (ys[i] - ys[i - 1]); } return ys[ys.length - 1]; }

function interpX(xs, ys, x) { const n = xs.length; if (x <= xs[0]) return ys[0] + (ys[1] - ys[0]) * (x - xs[0]) / ((xs[1] - xs[0]) || 1); if (x >= xs[n - 1]) return ys[n - 1] + (ys[n - 1] - ys[n - 2]) * (x - xs[n - 1]) / ((xs[n - 1] - xs[n - 2]) || 1); let i = 1; while (xs[i] < x) i++; const f = (x - xs[i - 1]) / ((xs[i] - xs[i - 1]) || 1); return ys[i - 1] + f * (ys[i] - ys[i - 1]); }

// 2D animated temperature contour of the spreader–isolation–spreader section: solved steady 2-D field scaled at each
// instant by the solved transient node responses (hot and cold side), with the node temperature traces and cursor.
async function anim2dField(el, { tr, Z, nx, ny, nH, nI, TaK, S }) {
  const [{ anim2d }, { h }, { cmap }] = await Promise.all([import('../core/sim3d.js'), import('../core/ui.js'), import('../core/chart.js')]);
  el.append(h('p', { class: 'small muted' }, 'Top: section temperature (x along hot spreader | isolation | cold spreader, y through the thickness ×-stretched). Bottom: solved hot/cold node temperatures from start-up with the time cursor.'));
  const K0 = 273.15, Ths = tr.y[tr.y.length - 1][0], Tcs = tr.y[tr.y.length - 1][1], all = Z.flat(), lo = Math.min(...all, TaK - K0, Tcs - K0), hi = Math.max(...all, TaK - K0, Ths - K0), tEnd = tr.t[tr.t.length - 1];
  const Th = tr.y.map(y => y[0] - K0), Tc = tr.y.map(y => y[1] - K0);
  anim2d(el, { height: 440, duration: tEnd, rate: tEnd / 20, label: 'Time from start-up', unit: 's', onFrame: (t, g) => {
    g.clear(); let j = tr.t.findIndex(x => x >= t); if (j < 0) j = tr.t.length - 1;
    const fH = (Th[j] + K0 - TaK) / ((Ths - TaK) || 1), fC = (Tc[j] + K0 - TaK) / ((Tcs - TaK) || 1), x0 = 60, y0 = 26, w = g.W - 160, hgt = g.H * 0.45;
    for (let jj = 0; jj < ny; jj++) for (let i = 0; i < nx; i++) { const side = i < nH + nI / 2 ? fH : fC, v = TaK - K0 + (Z[jj][i] + K0 - TaK) * (Number.isFinite(side) ? side : 1), c = cmap((v - lo) / ((hi - lo) || 1), 'coolwarm'); g.ctx.fillStyle = `rgb(${c.map(Math.round).join(',')})`; g.ctx.fillRect(x0 + i * w / nx, y0 + (ny - 1 - jj) * hgt / ny, w / nx + 1, hgt / ny + 1); }
    g.ctx.strokeStyle = g.fg; g.ctx.strokeRect(x0, y0, w, hgt); g.ctx.strokeRect(x0 + nH * w / nx, y0, nI * w / nx, hgt);
    { const now = performance.now() / 1000, sp = 0.08 + 0.25 * Math.min(1, Math.abs(S.Qconv) / Math.max(1e-9, Math.abs(S.Qin))); g.ctx.fillStyle = 'rgba(255,255,255,.9)'; for (let q = 0; q < 16; q++) { const f = (now * sp + q / 16) % 1; g.ctx.beginPath(); g.ctx.arc(x0 + f * w, y0 + hgt * (0.2 + 0.3 * (q % 3)), 6, 0, 2 * Math.PI); g.ctx.fill(); } g.text('● heat-flow tracers (speed ∝ solved Q_conv / Q_in)', x0, y0 + hgt + 12, { size: 10, color: g.mute }); }
    g.text('hot spreader', x0 + 4, y0 - 6, { size: 11 }); g.text('isolation', x0 + nH * w / nx + 2, y0 - 6, { size: 11 }); g.text('cold spreader', x0 + (nH + nI) * w / nx + 4, y0 - 6, { size: 11 });
    for (let k = 0; k <= 10; k++) { const c = cmap(k / 10, 'coolwarm'); g.ctx.fillStyle = `rgb(${c.map(Math.round).join(',')})`; g.ctx.fillRect(g.W - 80, y0 + hgt - (k + 1) * hgt / 11, 14, hgt / 11 + 1); }
    g.text(fmt(hi, 3) + ' °C', g.W - 62, y0 + 10, { size: 10 }); g.text(fmt(lo, 3) + ' °C', g.W - 62, y0 + hgt, { size: 10 });
    const P = g.panel(0, y0 + hgt + 10, g.W, g.H - hgt - y0 - 10, { xr: [0, tEnd], yr: [lo - 0.5, hi + 0.5], xlabel: 'Time [s]', ylabel: 'T [°C]', title: `Node temperatures — hot ${fmt(Th[j], 4)} °C, cold ${fmt(Tc[j], 4)} °C, ΔT ${fmt(Th[j] - Tc[j], 3)} K` });
    P.line(tr.t, Th, '#ef4444', 2); P.line(tr.t, Tc, '#3b82f6', 2); P.vline(t); P.point(tr.t[j], Th[j], '#ef4444'); P.point(tr.t[j], Tc[j], '#3b82f6');
  } });
}

// ---------------------------------------------------------------------------------------------------------------
// Design Revision 6 — solar-charged latent-heat store feeding the hub (passive / driver-mechanical control only).
// Charging: collector → loop heat pipe (one-way thermal diode) → PCM fins; a bimetal vent opens the collector glazing
// gap when the store is full. Discharge: a driver lever clamps the hot heat-pipe evaporator saddle onto the store's
// discharge boss (bellows-sealed, nothing rotates); its condenser saddles sit on the hot arc of the fixed cam.
// Rejection: cold-arc heat pipes into the graphite underbody spreader and short fins inside the wheel pods / spats.
async function storeBlock(p, ctx, { Ta, s13 }) {
  const pcm = PS.pcmById(p.pcm), veh = ctx.vehicle, v = p.vSize / 3.6, Proad = roadLoad(v, veh).P + (veh.bearingLoss_W ?? 3);
  // Suite 01 already carries the store line (this suite's previous solve, or its own estimate): take it out to get the base
  // Suite 01 tags its store lines 'thermalStore' (present only when the research configuration is installed)
  const s01 = ctx.upstream('s01'), base = s01 ? s01.mass - (s01.massByComponent?.thermalStore ?? 0) : veh.mass;
  const collExt = s13?.collectorExtMass_kg ?? 0, dailyJ = s13?.dailyHeat_J ?? 0, hourly = s13?.hourlyStore_W ?? [];
  const alloyHint = pcm.Tm > 125 ? 'nitihf' : 'nitcu', ThD = pcm.Tm - p.dTface - p.dThp, Nreq = 5000 * 15 * 1000 / (2 * Math.PI * (veh.wheelRadius || 0.3));
  const dsg = designRegenEngine(get(alloyHint), { Th: ThD, Tc: Ta + 15, Nreq, grid: { sH: [100, 150, 200, 250, 300, 350, 400, 450, 500], t: [0.03, 0.05, 0.08], N: [10, 14], arc: [120, 140] } });
  const cModel = 1 - (Ta + 15 + KELVIN) / (ThD + KELVIN), frac15 = dsg ? dsg.C.eta / cModel : 0;
  // the hub simulation (Suite 17, previous solve) can be less efficient than the material cycle: use the lower of the two
  const s17o = ctx.upstream('s17'), eta17 = s17o?.heatDemand_W > 0 && s17o?.PwheelGross_W > 0 ? s17o.PwheelGross_W / s17o.heatDemand_W : null, c17 = s17o?.Th !== undefined ? 1 - (s17o.Tc + KELVIN) / (s17o.Th + KELVIN) : null;
  const frac17 = p.hotSource === 'store' && eta17 && c17 > 0 ? eta17 / c17 : null, fracModel = frac17 !== null ? Math.min(frac15, frac17) : frac15; // the hub simulation is store-fed only in the research configuration
  const bases = { demo: { frac: PS.BASIS.demo.frac, label: 'Demonstrated basis' }, model: { frac: fracModel, label: 'Model (regenerated cycle, Suite 15)' }, target: { frac: PS.BASIS.target.frac, label: 'Research target' } };
  const Tlow = pcm.Tm - 25, Ttop = Math.min(pcm.Tmax, pcm.Tm + 8), eKg = PS.usableEnergy(pcm, 1, Tlow, Ttop), hpKgPerkW = 0.25;
  const hpMass = (Q) => 2 * hpKgPerkW * Q / 1000 + 0.8; // hot + cold loop heat-pipe hardware (≈ 0.25 kg/kW each side) + saddles
  const sizeFor = (frac, hours) => { let best = null; for (let Af = 0; Af <= 30; Af += 0.5) { const rej = PS.rejection({ v, Afin: Af }), op = PS.storeOperatingPoint({ pcm, Ta, Proad, frac, rej, dTface: p.dTface, dThot: p.dThp, dTcold: p.dTcp }); if (!op.feasible) continue; const st = PS.storeForEnergy(pcm, op.Q * 3600 * hours * (1 + p.margin / 100), { Tlow, Ttop, Tamb: Ta, epsEff: p.epsMLI }), tot = st.total + rej.finMass + hpMass(op.Q) + collExt; if (!best || tot < best.tot) best = { Af, rej, op, st, tot }; } return best; };
  const req = {}; for (const k of Object.keys(bases)) req[k] = bases[k].frac > 0 ? sizeFor(bases[k].frac, p.backupH) : null;
  // converter efficiency (fraction of Carnot) at which the requirement would just fit the vehicle mass cap
  const fracNeeded = (() => { const fits = (f) => { const r = sizeFor(f, p.backupH); return r && base + r.tot <= p.massCap; }; if (!fits(0.6)) return Infinity; let lo = 0.01, hi = 0.6; for (let i = 0; i < 18; i++) { const m = 0.5 * (lo + hi); if (fits(m)) hi = m; else lo = m; } return hi; })();
  // installed design: largest store inside the vehicle mass cap with the model-basis rejection hardware (budget mode),
  // or the store required by one of the bases
  const budget = p.massCap - base, refK = req.model ? 'model' : 'target', ref = req[refK];
  let inst = null;
  if (p.storeSizing !== 'budget' && req[p.storeSizing]) { const r = req[p.storeSizing]; inst = { Af: r.Af, rej: r.rej, mPcm: r.st.mPcm, V: r.st.V, hp: hpMass(r.op.Q), Qhp: r.op.Q }; }
  else if (ref) { const fixed = ref.rej.finMass + hpMass(ref.op.Q) + collExt, room = budget - fixed; let m = 0; if (room > 0.5) { let lo = 0, hi = room; for (let i = 0; i < 40; i++) { const mid = 0.5 * (lo + hi); if (mid + PS.vessel(pcm, mid, { Tamb: Ta, Tstore: Ttop, epsEff: p.epsMLI }).mass <= room) lo = mid; else hi = mid; } m = lo; } inst = { Af: ref.Af, rej: ref.rej, mPcm: m, V: PS.vessel(pcm, Math.max(m, 1e-3), { Tamb: Ta, Tstore: Ttop, epsEff: p.epsMLI }), hp: hpMass(ref.op.Q), Qhp: ref.op.Q }; }
  if (!inst) return null;
  const Einst = inst.mPcm * eKg, loss24 = inst.V.qLeak * 24 * 3600, sysMass = inst.mPcm + (inst.mPcm > 0 ? inst.V.mass : 0) + inst.rej.finMass + inst.hp + collExt;
  const ops = {}, dur = {}, dur24 = {}; for (const k of Object.keys(bases)) { ops[k] = bases[k].frac > 0 ? PS.storeOperatingPoint({ pcm, Ta, Proad, frac: bases[k].frac, rej: inst.rej, dTface: p.dTface, dThot: p.dThp, dTcold: p.dTcp }) : { feasible: false, Q: Infinity }; dur[k] = ops[k].feasible ? Einst / ops[k].Q / 3600 : 0; dur24[k] = ops[k].feasible ? Math.max(0, Einst - loss24) / ops[k].Q / 3600 : 0; }
  const opM = ops[refK];
  // demonstrated basis with the installed rejection: highest shaft power it can reach, the speed that sustains and for how long
  const demoMax = (() => { let best = { W: 0, Q: 0 }; const Th = pcm.Tm - p.dTface - p.dThp; for (let k = 0; k <= 300; k++) { const Q = 10 * Math.pow(2000, k / 300), Tc = Ta + Q / Math.max(1e-6, inst.rej.UA) + p.dTcp, W = Q * PS.BASIS.demo.frac * Math.max(0, 1 - (Tc + KELVIN) / (Th + KELVIN)); if (W > best.W) best = { W, Q, Tc }; } let lo = 0.5, hi = 60; for (let i = 0; i < 40; i++) { const m = 0.5 * (lo + hi), P = roadLoad(m / 3.6, veh).P + (veh.bearingLoss_W ?? 3) + PS.rejection({ v: m / 3.6, Afin: inst.Af }).Pdrag; if (P <= best.W * 0.85) lo = m; else hi = m; } return { ...best, vKmh: best.W > 0 ? lo : 0, hours: best.Q > 0 ? Einst / best.Q / 3600 : 0 }; })();
  // enthalpy-method charge (sunny day from the end-of-discharge state) and discharge (model basis) of one fin cell
  const delta = 0.004, Aface = inst.V.Aface || 1, lossPerA = inst.V.qLeak / Aface;
  const chg = inst.mPcm > 0.05 ? PS.enthalpySlab(pcm, { delta, n: 12, T0: Tlow, tEnd: 24 * 3600, record: 144, schedule: (t, Tf) => { const hr = Math.floor(t / 3600), q = (hourly[hr] || 0) / Aface; return (Tf < Ttop ? q : 0) - lossPerA; } }) : null;
  const tDis = Number.isFinite(opM.Q) && opM.Q > 0 ? Math.max(600, Math.min(6 * 3600, 1.6 * Einst / opM.Q)) : 3600;
  const TfaceMin = pcm.Tm - p.dTface - 6;
  const dis = inst.mPcm > 0.05 && opM.feasible ? PS.enthalpySlab(pcm, { delta, n: 16, T0: Ttop, tEnd: tDis, record: 160, schedule: (t, Tf) => Tf > TfaceMin - 15 ? -opM.Q / Aface - lossPerA : -lossPerA }) : null;
  const tUseful = dis ? (() => { const i = dis.rec.Tface.findIndex(T => T < TfaceMin); return i > 0 ? dis.rec.t[i] / 3600 : dis.rec.t[dis.rec.t.length - 1] / 3600; })() : 0;
  const tFull = chg ? (() => { const i = chg.rec.melt.findIndex(m => m > 0.98); return i >= 0 ? chg.rec.t[i] / 3600 : Infinity; })() : Infinity;
  const fa = (veh.wheelbase || 2) * 0.58 + (veh.axleShift ?? 0);
  return { fracNeeded, demoMax, frac15, frac17, pcm, Proad, base, budget, collExt, dailyJ, hourly, alloyHint, dsg, fracModel, bases, req, refK, inst, Einst, loss24, sysMass, ops, dur, dur24, opM, chg, dis, tUseful, tFull, Tlow, Ttop, eKg, delta, Aface, TfaceMin,
    position: { x: fa - 0.38, y: 0, z: 0.3, note: 'centreline, just aft of the driven front axle, low (heat-pipe runs ≤ 0.6 m to both hubs)' } };
}

function renderStore(out, SB, p, K0) {
  const installed = p.hotSource === 'store', { pcm, inst, ops, dur, dur24, req, bases, opM } = SB, cap = p.massCap, f3 = (x) => fmt(x, 3);
  const K = installed ? K0 : (label, value, unit, explain, prov, status) => K0(`Research configuration — ${label}`, value, unit, `NOT installed on the default vehicle. ${explain}`, prov, status === 'bad' ? 'warn' : status);
  const V0 = out.verdicts.length;
  const veh1h = (r) => r ? SB.base + r.tot : Infinity;
  K('Hot source (Revision 6)', `${pcm.name}: T_m ${pcm.Tm} °C, ${f3(pcm.L / 1000)} kJ/kg design latent heat`, '', `${pcm.note} Basis: ${pcm.src}.`, 'library');
  K('Installed latent-heat store', `${f3(inst.mPcm)} kg PCM + ${f3(inst.mPcm > 0 ? inst.V.mass : 0)} kg vessel and fins`, '', `${p.storeSizing === 'budget' ? `Largest store inside the ${cap} kg vehicle cap: ${f3(SB.budget)} kg available above the ${f3(SB.base)} kg vehicle (Suite 01, store excluded) for store, fins, heat pipes and collector upgrade.` : `Sized for ${p.backupH} h + ${p.margin} % on the ${p.storeSizing} basis.`} Vessel: 316L inner can (dry nitrogen fill) inside an evacuated 5083 shell with multilayer reflective foil (ε_eff ${p.epsMLI}); Ø${f3(inst.V.D * 1000)} × ${f3(inst.V.L * 1000)} mm, outer-shell buckling SF ${f3(inst.V.buckSF)}.`, p.storeSizing === 'budget' ? 'generated' : 'calc', inst.mPcm > 0 ? 'ok' : 'bad');
  K('Thermal-system mass (store + fins + heat pipes + collector upgrade)', SB.sysMass, 'kg', `Store ${f3(inst.mPcm + (inst.mPcm > 0 ? inst.V.mass : 0))} kg, rejection fins ${f3(inst.rej.finMass)} kg (${f3(inst.Af)} m²), loop heat pipes and saddles ${f3(inst.hp)} kg (sized for ${f3(inst.Qhp)} W), collector upgrade ${f3(SB.collExt)} kg (Suite 13). Vehicle with store ${f3(SB.base + SB.sysMass)} kg vs cap ${cap} kg.`, 'calc', SB.base + SB.sysMass <= cap + 1e-6 ? 'ok' : 'bad');
  K('Usable stored heat', SB.Einst / 1e6, 'MJ', `${f3(SB.eKg / 1000)} kJ/kg between ${f3(SB.Tlow)} °C (solid, sensible) and ${f3(SB.Ttop)} °C (liquid, sensible) including the latent heat.`, 'calc');
  K('Standby loss of the vacuum vessel', inst.V.qLeak, 'W', `Radiation across the evacuated gap ${f3(inst.V.qRad)} W + four Ti-alloy straps ${f3(inst.V.qSup)} W + six thin-wall penetrations ${f3(inst.V.qPen)} W → ${f3(SB.loss24 / 1e6)} MJ per 24 h (${f3(100 * SB.loss24 / Math.max(1, SB.Einst))} % of the store). The disengaged discharge saddle sits in a bellows-sealed vacuum pocket.`, 'calc', SB.loss24 < 0.15 * SB.Einst ? 'ok' : 'warn');
  K('Heat collected per sunny day vs store + 24 h loss', `${f3(SB.dailyJ / 1e6)} / ${f3((SB.Einst + SB.loss24) / 1e6)}`, 'MJ', 'Suite 13 hour-by-hour collection at the charging absorber temperature through the loop heat pipe (one-way thermal diode).', 'coupled', SB.dailyJ >= SB.Einst + SB.loss24 ? 'ok' : 'warn');
  K('Time to full charge on a sunny day (enthalpy model)', Number.isFinite(SB.tFull) ? SB.tFull : 'does not fill in one sunny day', Number.isFinite(SB.tFull) ? 'h' : '', `Melt front driven across the ${SB.delta * 1000} mm half-pitch between heat-pipe fins (${f3(SB.Aface)} m² of fin face) by the hourly collector heat, standby loss included.`, 'calc', Number.isFinite(SB.tFull) ? 'ok' : 'warn');
  K('Operating point on the reference basis', opM.feasible ? `T_hot ${f3(opM.Th)} °C / T_cold ${f3(opM.Tc)} °C, η ${f3(opM.eta * 100)} %, heat ${f3(opM.Q)} W` : 'infeasible', '', `${bases[SB.refK].label}: ${f3(bases[SB.refK].frac * 100)} % of Carnot. Shaft demand ${f3(opM.Pshaft)} W = (road ${f3(SB.Proad)} W + fin drag ${f3(inst.rej.Pdrag)} W) / 0.85. Rejection UA ${f3(inst.rej.UA)} W/K (underbody h ${f3(inst.rej.hU)} W/m²K, pod/spat fins h ${f3(inst.rej.hF)} W/m²K).`, 'calc', opM.feasible ? 'ok' : 'bad');
  for (const k of ['demo', 'model', 'target']) K(`Backup at ${p.vSize} km/h road load — ${bases[k].label}`, dur[k], 'h', ops[k].feasible ? `η = ${f3(ops[k].eta * 100)} % (${f3(bases[k].frac * 100)} % of Carnot ${f3(ops[k].etaC * 100)} %) draws ${f3(ops[k].Q)} W of stored heat; ${f3(dur24[k])} h after a 24 h standby. ${k === 'demo' ? 'Regenerator given no credit.' : k === 'model' ? 'Simulation / research target — not physical validation.' : 'Upside only.'}` : 'The installed rejection cannot carry the heat this basis needs.', k === 'demo' ? 'literature' : 'calc', dur[k] >= p.backupH * (1 + p.margin / 100) ? 'ok' : 'bad');
  K('Demonstrated basis with the installed hardware', `${fmt(SB.demoMax.W * 0.85, 3)} W at the wheel max → ${fmt(SB.demoMax.vKmh, 3)} km/h for ${fmt(SB.demoMax.hours, 3)} h`, '', `Highest converter output when the installed rejection (${fmt(inst.Af, 3)} m² fins + underbody) carries the demonstrated-basis heat ${fmt(SB.demoMax.Q, 3)} W (cold arc ${fmt(SB.demoMax.Tc, 4)} °C). Not enough for ${p.vSize} km/h.`, 'calc', SB.demoMax.vKmh >= p.vSize ? 'ok' : 'bad');
  for (const k of ['demo', 'model', 'target']) { const r = req[k]; K(`Mass for ${p.backupH} h + ${p.margin} % — ${bases[k].label}`, r ? r.tot : NaN, 'kg', r ? `PCM ${f3(r.st.mPcm)} kg + vessel ${f3(r.st.V.mass)} + fins ${f3(r.rej.finMass)} (${f3(r.Af)} m², drag ${f3(r.rej.Pdrag)} W) + heat pipes ${f3(r.tot - r.st.total - r.rej.finMass - SB.collExt)} + collector upgrade ${f3(SB.collExt)} kg; heat draw ${f3(r.op.Q)} W at ${f3(r.op.Th)} / ${f3(r.op.Tc)} °C. Vehicle ${f3(veh1h(r))} kg → ${veh1h(r) > cap ? `${f3(veh1h(r) - cap)} kg OVER` : `${f3(cap - veh1h(r))} kg under`} the ${cap} kg cap.` : 'No rejection surface up to 30 m² of fins makes this basis feasible.', 'calc', r && veh1h(r) <= cap ? 'ok' : 'bad'); }
  const dDemo = dur.demo, okDemo = dDemo >= p.backupH * (1 + p.margin / 100);
  out.verdicts.push({ test: `Backup ≥ ${p.backupH} h at ${p.vSize} km/h with ≥ ${p.margin} % margin — demonstrated basis`, status: okDemo ? 'pass' : 'fail', detail: `${fmt(dDemo, 3)} h from the installed ${fmt(inst.mPcm, 3)} kg store at η ${fmt(ops.demo.eta * 100, 3)} % (${fmt(bases.demo.frac * 100, 3)} % of Carnot, built devices). Meeting it needs ${req.demo ? `${fmt(req.demo.tot, 3)} kg of thermal system → vehicle ${fmt(veh1h(req.demo), 4)} kg, ${fmt(veh1h(req.demo) - cap, 4)} kg over the ${cap} kg cap` : 'more rejection than 30 m² of fins can give'}.`, fix: 'Only a higher demonstrated converter efficiency closes this: on this basis heat storage cannot fit the mass cap.' });
  out.verdicts.push({ test: `Backup at ${p.vSize} km/h — research target (upside)`, status: dur.target >= p.backupH * (1 + p.margin / 100) ? 'pass' : 'warn', detail: `${fmt(dur.target, 3)} h at ${fmt(bases.target.frac * 100, 3)} % of Carnot; model (regenerated cycle) ${fmt(dur.model, 3)} h. Simulation / research targets — not physical validation.` });
  out.verdicts.push({ test: 'Store fully recharged on a sunny day', status: SB.dailyJ >= SB.Einst + SB.loss24 && Number.isFinite(SB.tFull) ? 'pass' : 'warn', detail: `${fmt(SB.dailyJ / 1e6, 3)} MJ collected vs ${fmt((SB.Einst + SB.loss24) / 1e6, 3)} MJ store + 24 h loss; full after ${fmt(SB.tFull, 3)} h (enthalpy model).` });
  out.verdicts.push({ test: 'Charge / discharge control is passive or driver-mechanical', status: 'pass', detail: 'Loop heat pipe collector → store is a one-way thermal diode (no flow at night); a bimetal strip opens the collector glazing vent when the store is full; a driver lever clamps the discharge heat-pipe saddle (bellows feed-through, no rotation). No valves, pumps, heaters or electronics.' });
  out.verdicts.push({ test: 'PCM long-term stability in the sealed store', status: 'warn', detail: `${pcm.name}: ${pcm.note}` });
  out.tables.push({ title: `Store sizing for ${p.backupH} h at ${p.vSize} km/h + ${p.margin} % margin by efficiency basis`, cols: ['Basis', '% of Carnot', 'η (%)', 'Heat draw (W)', 'T_hot / T_cold (°C)', 'Fin area (m²)', 'Fin drag (W)', 'PCM (kg)', 'Thermal system (kg)', 'Vehicle (kg)', 'vs cap (kg)'], rows: ['demo', 'model', 'target'].map(k => { const r = req[k]; return r ? [bases[k].label, bases[k].frac * 100, r.op.eta * 100, r.op.Q, `${fmt(r.op.Th, 4)} / ${fmt(r.op.Tc, 4)}`, r.Af, r.rej.Pdrag, r.st.mPcm, r.tot, veh1h(r), veh1h(r) - cap] : [bases[k].label, bases[k].frac * 100, '—', '—', '—', '—', '—', '—', 'infeasible', '—', '—']; }) });
  out.tables.push({ title: 'Efficiency bases (literature)', cols: ['Basis', 'Value', 'Source'], rows: [['Demonstrated (built device)', '1.4 % measured vs 35.6 % Carnot ≈ 3.9 % of Carnot', 'Shape-memory heat engine, Ni-45 at% Ti Stirling/Ericsson cycles (Springer, doi:10.1007/BF00550253)'], ['Simulated device', '≈ 1.2 % vs ≈ 20 % Carnot (18–90 °C) ≈ 6 %', 'Energy conversion in SMA heat engine, part II: simulation'], ['Model (this chain)', `${fmt(SB.fracModel * 100, 3)} % of Carnot`, `Lower of the Suite 15 regenerated material cycle (${fmt(SB.frac15 * 100, 3)} %) and the Suite 17 hub simulation (${SB.frac17 !== null ? fmt(SB.frac17 * 100, 3) + ' %' : 'not yet solved'})`], ['Research target', '≈ 1/5 – 1/3 of Carnot', 'Thermodynamic estimates for SMA engines (1975 analysis cited in SMA engine patents); theoretical, not built']] });
  if (SB.chg) out.charts.push({ title: 'Store charge on a sunny day (enthalpy method)', type: 'line', x: { label: 'Time', unit: 'h' }, y: { label: 'Value' }, series: [{ name: 'Fin-face temperature (°C)', x: SB.chg.rec.t.map(t => t / 3600), y: SB.chg.rec.Tface }, { name: 'Melt fraction × 100', x: SB.chg.rec.t.map(t => t / 3600), y: SB.chg.rec.melt.map(m => 100 * m), dash: '4 3' }, { name: 'Melt-front position (mm × 10)', x: SB.chg.rec.t.map(t => t / 3600), y: SB.chg.rec.front.map(f => 10 * f), dash: '2 2' }], explain: 'One fin cell of the store: the melting front moves from the heat-pipe fin across the half-pitch as collector heat arrives hour by hour; the temperature plateau is the phase change.' });
  if (SB.dis) out.charts.push({ title: `Store discharge at the ${p.vSize} km/h road load (reference basis)`, type: 'line', x: { label: 'Time', unit: 'h' }, y: { label: 'Value' }, series: [{ name: 'Fin-face temperature (°C)', x: SB.dis.rec.t.map(t => t / 3600), y: SB.dis.rec.Tface }, { name: 'Liquid fraction × 100', x: SB.dis.rec.t.map(t => t / 3600), y: SB.dis.rec.melt.map(m => 100 * m), dash: '4 3' }], annotations: [{ type: 'hline', y: SB.TfaceMin, label: 'lowest useful face T' }, { type: 'vline', x: SB.tUseful, label: 'useful end' }], explain: 'The solidification front grows from the fin; the face temperature drops as the solid layer thickens. Below the dashed line the hot arc can no longer reach the alloy\'s transformation temperature.' });
  out.charts.push({ title: 'Backup duration at the road load by efficiency basis', type: 'bar', categories: ['Demonstrated', 'Model (regenerated)', 'Research target', `Requirement (${p.backupH} h + ${p.margin} %)`], series: [{ name: 'h', values: [dur.demo, dur.model, dur.target, p.backupH * (1 + p.margin / 100)], colors: ['#ef4444', '#f59e0b', '#22c55e', '#94a3b8'] }], y: { label: 'Duration', unit: 'h' }, showValues: true, explain: 'Same installed store, different converter efficiencies: only the demonstrated basis may be used to claim the requirement.' });
  out.views.push({ label: '2D animation: PCM melting / solidification fronts', icon: '🎞', render: (el) => anim2dStore(el, SB) });
  const fn = Number.isFinite(SB.fracNeeded) ? `${fmt(SB.fracNeeded * 100, 3)} % of Carnot` : 'not reachable below 60 % of Carnot';
  if (!installed) { const added = out.verdicts.splice(V0); out.verdicts.push({ test: 'Latent-heat store (research configuration, not installed)', status: 'info', detail: `Demonstrated basis: 1 h + ${p.margin} % at ${p.vSize} km/h needs ${req.demo ? `${fmt(req.demo.tot, 4)} kg of thermal system → vehicle ${fmt(SB.base + req.demo.tot, 4)} kg, ${fmt(SB.base + req.demo.tot - cap, 4)} kg over the ${cap} kg cap` : 'more rejection than 30 m² of fins'}; a cap-limited store gives ${fmt(dur.demo * 60, 3)} min. The requirement fits the cap only with a demonstrated converter efficiency of ${fn}. ${added.map(v => v.test).join('; ')} — see KPIs and tables.` }); }
  out.recs.push({ level: installed ? 'warn' : 'info', text: `Latent-heat store: worthwhile only if a shape-memory converter reaching ≥ ${fn} (built-device evidence today ≈ ${fmt(bases.demo.frac * 100, 3)} %) is demonstrated by test; on the demonstrated basis the 1 h requirement is ${req.demo ? fmt(SB.base + req.demo.tot - cap, 4) + ' kg' : 'far'} over the mass cap. ${installed ? 'Currently installed as a research configuration.' : 'Kept as an optional research configuration, off by default.'}`, basis: 'Suite 14 store sizing on demonstrated / model / research-target bases' });
}

async function anim2dStore(el, SB) {
  const [{ anim2d }, { h }, { cmap }] = await Promise.all([import('../core/sim3d.js'), import('../core/ui.js'), import('../core/chart.js')]);
  const C = SB.chg?.rec, D = SB.dis?.rec; if (!C && !D) { el.append(h('p', { class: 'small muted' }, 'No store installed.')); return; }
  el.append(h('p', { class: 'small muted' }, 'Solved enthalpy-method histories of one fin cell: sunny-day charge (left) then discharge at the road load (right). Shading = liquid fraction across the half-pitch from the heat-pipe fin; traces = fin-face temperature.'));
  const tc = C ? C.t[C.t.length - 1] : 0, td = D ? D.t[D.t.length - 1] : 0, T = tc + td;
  anim2d(el, { height: 400, duration: T / 3600, rate: Math.max(0.5, T / 3600 / 3), label: 'Time', unit: 'h', onFrame: (th, g) => {
    g.clear(); const t = th * 3600, W = g.W, Hh = g.H, inC = t <= tc, R = inC ? C : D, tl = inC ? t : t - tc;
    const i = R ? Math.max(0, Math.min(R.t.length - 1, R.t.findIndex(x => x >= tl))) : 0, melt = R ? R.melt[i] : 0, front = R ? R.front[i] : 0, Tf = R ? R.Tface[i] : 0;
    const x0 = 40, y0 = 30, w = W * 0.38, hh = Hh - 90, dmm = SB.delta * 1000;
    g.text(inC ? 'Charging (collector → loop heat pipe → fin)' : 'Discharging (fin → hot arc of the cam)', x0, 20, { bold: true });
    g.ctx.fillStyle = '#64748b'; g.ctx.fillRect(x0, y0, 10, hh);
    const fx = Math.max(0, Math.min(1, (inC ? front : dmm - front) / dmm));
    for (let k = 0; k < 40; k++) { const xf = k / 40, liq = inC ? (xf < fx ? 1 : 0) : (xf < 1 - fx ? 0 : 1), c = cmap(liq ? 0.85 : 0.2, 'coolwarm'); g.ctx.fillStyle = `rgb(${c.map(Math.round).join(',')})`; g.ctx.fillRect(x0 + 10 + xf * w, y0, w / 40 + 1, hh); }
    g.text(`liquid fraction ${fmt(melt, 3)}   front ${fmt(front, 3)} mm   face ${fmt(Tf, 4)} °C`, x0, y0 + hh + 20, { size: 12 });
    const P = g.panel(W * 0.48, 0, W * 0.52, Hh - 40, { xr: [0, T / 3600], yr: [SB.Tlow - 10, SB.Ttop + 15], xlabel: 'Time [h]', ylabel: 'Fin-face T [°C]', title: 'Charge then discharge' });
    if (C) P.line(C.t.map(x => x / 3600), C.Tface, '#ef4444', 2); if (D) P.line(D.t.map(x => (x + tc) / 3600), D.Tface, '#38bdf8', 2);
    P.hline(SB.pcm.Tm, '#a855f7'); P.vline(th); P.point(th, Tf);
  } });
}
