// 24 h parking-hold check (7th audit), the same rule for every design's parking system: clamp left after the hot discs and pads cool
// from the last stop to ambient, brake-fluid contraction, seal leak-down over 24 h, and a brake-circuit failure while parked.
// Capacity after 24 h must be ≥ 1.5 × the slope + gust demand. Modes: 'hydraulicHold' (service brake held by a spring/strand on the
// pedal), 'cable' (mechanical lever + cable on the calipers), 'pawl' (positive mechanical lock on the rear wheel).
export function parkHold24h({ mode, demand, cap0, grip, Tdisc = 300, Tamb = 20, tDisc = 0.004, tPad = 0.008, nCal = 3, areaRatio = 4, pedalRatio = 4,
  dMC = 0.0159, Vcal = 4e-6, beta = 0.9e-3, leak_mm3_h = 0.5, Fpush0 = 204, strandEA = 110e3, strandL = 1.5, cableEA = 690e3, cableL = 2.5, cableF0 = 1200, mechRatio = 5.1,
  wheelR = 0.24, pawlR = 0.08, teeth = 20, pawlW = 0.012, pawlH = 0.006, sy = 600e6, circuitShare = 0.5 } = {}) {
  const dT = Math.max(0, Tdisc - Tamb), dGap = 12e-6 * tDisc * dT + 2 * 15e-6 * tPad * 0.6 * dT /* disc + both pads contract */;
  if (mode === 'hydraulicHold') {
    const Amc = Math.PI / 4 * dMC * dMC, Acal = areaRatio * Amc, dV = nCal * (Acal * dGap + Vcal * beta * 0.25 * dT) + leak_mm3_h * 24 * 1e-9, xMC = dV / Amc;
    const kMC = strandEA / strandL * pedalRatio * pedalRatio, F24 = Math.max(0, Fpush0 - kMC * xMC), cap24 = Math.min(cap0 * F24 / Fpush0, grip), capFail = Math.min(cap24 * circuitShare, grip);
    return { mode, dGap_mm: dGap * 1000, dV_cm3: dV * 1e6, xMC_mm: xMC * 1000, F24_N: F24, cap24, SF24: cap24 / demand, capFail, SFfail: capFail / demand, ok: cap24 >= 1.5 * demand && capFail >= 1.5 * demand,
      note: 'held hydraulic pressure: the pedal link is stiff, so the cooling volume loss removes the clamp; a failed circuit halves it' };
  }
  if (mode === 'cable') {
    const kC = cableEA / cableL, slack = 2 * dGap * mechRatio /* both caliper faces through the caliper lever */, F24 = Math.max(0, cableF0 - kC * slack), cap24 = Math.min(cap0 * F24 / cableF0, grip);
    return { mode, dGap_mm: dGap * 1000, cableLoss_N: cableF0 - F24, cap24, SF24: cap24 / demand, capFail: cap24, SFfail: cap24 / demand, ok: cap24 >= 1.5 * demand, note: 'mechanical: no hydraulics, the cable stretch compensates the cooling' };
  }
  // positive mechanical lock: tooth in bending at the slope + gust torque; no time dependence, no hydraulics
  const T = demand * wheelR, Ft = T / pawlR, sig = 6 * Ft * pawlH / (pawlW * (2 * Math.PI * pawlR / teeth / 2) ** 2), SFtooth = sy / sig, roll = 2 * Math.PI * wheelR / teeth;
  const capLock = Math.min(grip, sy / 1.0 * (pawlW * (2 * Math.PI * pawlR / teeth / 2) ** 2) / (6 * pawlH) * pawlR / wheelR);
  return { mode, toothLoad_N: Ft, SFtooth, maxRoll_mm: roll * 1000, cap24: capLock, SF24: capLock / demand, capFail: capLock, SFfail: capLock / demand, ok: capLock >= 1.5 * demand && SFtooth >= 2, note: 'positive lock on the rear wheel: independent of temperature, leak-down and the hydraulic circuits' };
}
