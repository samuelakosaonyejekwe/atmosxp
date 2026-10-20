// Solar-charged latent-heat store (Design Revision 6): phase-change material data, a one-dimensional enthalpy-method
// model of the PCM between heat-exchange fins (melting / solidification fronts, charge and discharge curves), the
// vacuum-insulated vessel (mass and standing loss) and the passive charge / driver-engaged discharge heat paths.
// All control is passive or driver-mechanical: the collector loop heat pipe is a one-way thermal diode, a bimetal vent
// opens the collector glazing gap when the store is full, and a driver lever clamps the discharge heat-pipe saddle.
import { SIGMA, KELVIN } from './num.js';

// Literature property sets (typical values; confirm by DSC / laser-flash on the supplied batch). Only polyolefin
// (paraffin-class) PCMs are offered: inert, insoluble hydrocarbons.
export const PCMS = [
  { id: 'pe-wax', name: 'Polyethylene wax (high-melting)', Tm: 108, dTm: 6, L: 210e3, cpS: 2200, cpL: 2600, rhoS: 940, rhoL: 800, kS: 0.35, kL: 0.25, Tmax: 200, expand: 0.15, cost: 3, co2: 1.9, energy: 75, recyclable: 0.8,
    note: 'No supercooling, chemically inert; lower latent heat and conductivity, broad melting range.', src: 'Polyolefin-wax PCM literature (DSC 180–240 kJ/kg, T_m 100–115 °C)' },
  { id: 'hdpe', name: 'High-density polyethylene (form-stable)', Tm: 130, dTm: 5, L: 190e3, cpS: 2100, cpL: 2500, rhoS: 955, rhoL: 790, kS: 0.45, kL: 0.26, Tmax: 220, expand: 0.17, cost: 2, co2: 1.8, energy: 76, recyclable: 0.9,
    note: 'Form-stable (cross-linked) grades keep their shape when molten; melting point above the polycarbonate glazing limit, so collector stagnation must be vented.', src: 'Polyolefin PCM literature (crystallinity-dependent 160–220 kJ/kg)' },
];
export const pcmById = (id) => PCMS.find(m => m.id === id) || PCMS.find(m => m.id === 'hdpe') || PCMS[0];

// Specific enthalpy h(T) (J/kg, zero at 0 °C solid) with a linear mushy zone of width dTm, and its inverse.
export function enthalpyOf(m) {
  const T1 = m.Tm - m.dTm / 2, T2 = m.Tm + m.dTm / 2, h1 = m.cpS * T1, h2 = h1 + m.L + 0.5 * (m.cpS + m.cpL) * m.dTm;
  const h = (T) => T <= T1 ? m.cpS * T : T >= T2 ? h2 + m.cpL * (T - T2) : h1 + (T - T1) / m.dTm * (m.L + 0.5 * (m.cpS + m.cpL) * m.dTm);
  const T = (H) => H <= h1 ? H / m.cpS : H >= h2 ? T2 + (H - h2) / m.cpL : T1 + (H - h1) / (h2 - h1) * m.dTm;
  const f = (H) => Math.max(0, Math.min(1, (H - h1) / (h2 - h1))); // liquid fraction
  return { h, T, f, h1, h2, T1, T2 };
}

// 1-D enthalpy method across the half-pitch δ between a fin face (x = 0, heat flux in/out) and the symmetry plane.
// schedule(t, Tface, state) → face heat flux into the PCM (W/m², negative = discharge). Explicit, stability-limited step.
export function enthalpySlab(m, { delta, n = 24, T0, tEnd, schedule, record = 120 }) {
  const E = enthalpyOf(m), dx = delta / n, H = new Float64Array(n).fill(E.h(T0));
  const kMax = Math.max(m.kS, m.kL), rho = 0.5 * (m.rhoS + m.rhoL), cpMin = Math.min(m.cpS, m.cpL);
  const dt = Math.min(5, 0.4 * rho * cpMin * dx * dx / kMax);
  const rec = { t: [], Tface: [], Tmid: [], melt: [], front: [], q: [] };
  const every = Math.max(1, Math.floor(tEnd / dt / record));
  let t = 0, step = 0, Ein = 0, Eout = 0;
  const Tn = new Float64Array(n), kf = new Float64Array(n);
  while (t < tEnd) {
    for (let i = 0; i < n; i++) { Tn[i] = E.T(H[i]); const f = E.f(H[i]); kf[i] = m.kS * (1 - f) + m.kL * f; }
    // face temperature extrapolated from the first node (half-cell) using the flux of the previous step
    const qf = schedule(t, Tn[0], { melt: meanF(H, E), H });
    const Tface = Tn[0] + qf * dx / 2 / kf[0];
    for (let i = 0; i < n; i++) {
      const kw = i > 0 ? 2 * kf[i] * kf[i - 1] / (kf[i] + kf[i - 1]) : 0, ke = i < n - 1 ? 2 * kf[i] * kf[i + 1] / (kf[i] + kf[i + 1]) : 0;
      const qW = i === 0 ? qf : kw * (Tn[i - 1] - Tn[i]) / dx, qE = i < n - 1 ? ke * (Tn[i] - Tn[i + 1]) / dx : 0;
      H[i] += dt * (qW - qE) / (rho * dx);
    }
    if (qf > 0) Ein += qf * dt; else Eout -= qf * dt;
    if (step % every === 0) { const mf = meanF(H, E); let fr = 0; for (let i = 0; i < n; i++) if (E.f(H[i]) > 0.5) fr = (i + 1) * dx; rec.t.push(t); rec.Tface.push(Tface); rec.Tmid.push(Tn[n - 1]); rec.melt.push(mf); rec.front.push(fr * 1000); rec.q.push(qf); }
    t += dt; step++;
  }
  return { rec, H, dt, Ein, Eout, final: { melt: meanF(H, E), Tmean: Array.from(H).reduce((s, x) => s + E.T(x), 0) / n } };
}
const meanF = (H, E) => { let s = 0; for (let i = 0; i < H.length; i++) s += E.f(H[i]); return s / H.length; };

// Vacuum-insulated cylindrical vessel (L/D = 2) holding PCM mass mPcm with aluminium fins at half-pitch delta:
// 316L inner can, 5083 aluminium outer shell sized against external-pressure buckling (SF ≥ 2.5).
export function vessel(m, mPcm, { delta = 0.005, finT = 0.0004, wallIn = 0.0005, wallOut = 0.0015, gap = 0.012, epsEff = 0.01, Tamb = 25, Tstore = null, nPen = 6 } = {}) {
  const Vp = mPcm / m.rhoL * (1 + 0.02), Aface = Vp / delta;       // each fin face serves δ of PCM
  const finMass = (Aface / 2) * finT * 2700, Vin = (Vp + Aface / 2 * finT) * (1 + m.expand * 0.3); // ullage for melt expansion
  const D = Math.cbrt(4 * Vin / (2 * Math.PI)), Lc = 2 * D, Ain = Math.PI * D * Lc + 2 * Math.PI * D * D / 4;
  const Do = D + 2 * gap, Lo = Lc + 2 * gap, Aout = Math.PI * Do * Lo + 2 * Math.PI * Do * Do / 4;
  // short-cylinder external-pressure buckling p_cr ≈ 0.92 E (t/R)^2.5 / (L/R) (Windenburg–Trilling)
  const Ro = Do / 2, pcr = 0.92 * 70e9 * (wallOut / Ro) ** 2.5 / (Lo / Ro), buckSF = pcr / 101325;
  const shellMass = Ain * wallIn * 8000 + Aout * wallOut * 2660, mli = Ain * 0.12, supports = 0.08, mass = shellMass + mli + supports + finMass;
  const Ts = (Tstore ?? m.Tm) + KELVIN, Ta = Tamb + KELVIN;
  const qRad = SIGMA * epsEff * Ain * (Ts ** 4 - Ta ** 4);
  const qSup = 4 * 7 * 1.2e-5 / 0.08 * (Ts - Ta);                           // four Ti-alloy tension straps 12 mm², 80 mm
  const qPen = nPen * 16 * (Math.PI * 0.006 * 0.0004) / 0.15 * (Ts - Ta);   // thin-wall 316L penetrations (heat-pipe and loop lines)
  const qLeak = qRad + qSup + qPen;
  return { D, L: Lc, Do, Lo, Vin, Aface, finMass, shellMass, mass, pcr, buckSF, Ain, Aout, qRad, qSup, qPen, qLeak };
}

// Usable store energy between the lowest useful discharge temperature TlowC and a full charge at TtopC (J).
export function usableEnergy(m, mPcm, TlowC, TtopC) { const E = enthalpyOf(m); return mPcm * Math.max(0, E.h(TtopC) - E.h(TlowC)); }

// Regenerated, partially transforming NiTi cycle on a stationary-cam counterflow regenerator:
// N heat-pipe stations each couple an outbound (cooling) cartridge position to an inbound (heating) one.
// Effectiveness: ε = 1 / (1 + 1/N + 1/NTU), NTU = t_regen / τ (t_regen = residence time in one regenerator arc).
export function regenEffectiveness({ nStations, arcDeg, fRev, tau }) {
  if (!(nStations > 0) || !(fRev > 0) || !(tau > 0)) return { eps: 0, NTU: 0, tRegen: 0 };
  const tRegen = arcDeg / 360 / fRev, NTU = tRegen / tau;
  return { eps: 1 / (1 + 1 / nStations + 1 / NTU), NTU, tRegen };
}

// ---------------------------------------------------------------------------------------------------------------
// Revision 6 store-fed hub system sizing. Bases for the converter efficiency (fraction of the Carnot efficiency at the
// solved actuator interface temperatures):
//   demonstrated — built shape-memory heat engines: 1.4 % measured against a 35.6 % Carnot limit (≈ 3.9 % of Carnot,
//                  Ni-45Ti Stirling/Ericsson-cycle engine, Springer doi:10.1007/BF00550253); no regenerator credit.
//   model        — this chain's regenerated cycle (Suite 15 constitutive model + stationary-cam regenerator).
//   target       — thermodynamic estimates of ≈ 1/5–1/3 of Carnot for SMA engines (research target, lower bound 20 %).
export const BASIS = { demo: { frac: 0.039, label: 'Demonstrated basis (built devices, ≈ 3.9 % of Carnot, no regenerator credit)' }, target: { frac: 0.20, label: 'Research target (≈ 20 % of Carnot, theoretical estimates)' } };

// Cold-side rejection: graphite underbody as a heat-pipe-fed spreader (turbulent flat plate, no added drag) plus
// short aluminium fins in the wheel pods / spats (laminar plates; skin-friction and leading-edge drag counted).
export function rejection({ v, Aunder = 3.0, Lunder = 3.0, etaU = 0.6, Afin = 0, Lfin = 0.1, etaF = 0.85 }) {
  const nu = 1.6e-5, k = 0.027, Pr = 0.71, rho = 1.18, q = 0.5 * rho * v * v;
  const ReU = Math.max(1, v * Lunder / nu), hU = 0.037 * ReU ** 0.8 * Pr ** (1 / 3) * k / Lunder;
  const ReF = Math.max(1, v * Lfin / nu), hF = 0.664 * ReF ** 0.5 * Pr ** (1 / 3) * k / Lfin, Cf = 1.328 / Math.sqrt(ReF);
  const UA = hU * Aunder * etaU + hF * Afin * etaF, drag = 1.2 * Cf * q * Afin, finMass = Afin / 2 * 0.0005 * 2700 + Afin * 0.05;
  return { hU, hF, UA, drag, Pdrag: drag * v, finMass, Cf };
}

// Solve the hot/cold interface temperatures and heat draw for a given efficiency basis. The hot and cold heat-pipe
// hardware is sized for fixed temperature drops (mass ∝ heat flow), so Th is fixed and Tc rises with the rejected heat;
// the smallest Q with Q·η(Q) = P_shaft is returned (Infinity when the rejection surface cannot reach it).
export function storeOperatingPoint({ pcm, Ta, Proad, etaTr = 0.85, frac, rej, dTface = 6, dThot = 6, dTcold = 4, etaFn = null }) {
  const Pshaft = (Proad + rej.Pdrag) / etaTr, Th = pcm.Tm - dTface - dThot;
  const at = (Q) => { const Qrej = Math.max(0, Q - Pshaft), Tc = Ta + Qrej / Math.max(1e-6, rej.UA) + dTcold, etaC = Math.max(0, 1 - (Tc + KELVIN) / (Th + KELVIN)), eta = etaFn ? etaFn(Th, Tc) : frac * etaC; return { Q, Th, Tc, eta, etaC, Pshaft, Qrej, W: Q * eta }; };
  let prev = at(Pshaft * 1.0001);
  for (let k = 1; k <= 400; k++) { const Q = Pshaft * 1.0001 * Math.pow(1e4, k / 400), cur = at(Q); if (cur.W >= Pshaft) { let lo = prev.Q, hi = Q; for (let i = 0; i < 50; i++) { const m = 0.5 * (lo + hi); if (at(m).W >= Pshaft) hi = m; else lo = m; } return { ...at(hi), feasible: true }; } prev = cur; }
  return { ...at(Infinity), Q: Infinity, feasible: false };
}

// Store mass for a required stored heat E (J), including the vessel, fins and the 24 h standby loss (fixed point).
export function storeForEnergy(pcm, E, { Tlow, Ttop, Tamb = 25, delta = 0.004, epsEff = 0.01, standby_h = 24 } = {}) {
  const e = usableEnergy(pcm, 1, Tlow, Ttop); let m = E / Math.max(1, e), V = null;
  for (let it = 0; it < 12; it++) { V = vessel(pcm, m, { delta, Tamb, Tstore: Ttop, epsEff }); const mn = (E + V.qLeak * standby_h * 3600) / e; if (Math.abs(mn - m) < 1e-3) { m = mn; break; } m = mn; }
  V = vessel(pcm, m, { delta, Tamb, Tstore: Ttop, epsEff });
  return { mPcm: m, V, e, total: m + V.mass, standbyJ: V.qLeak * standby_h * 3600 };
}
