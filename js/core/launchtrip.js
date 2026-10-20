// Launch-buffer trip from the BRAKE PEDAL (vehicle.launchTrip 'pedal'): merges the separate buffer trigger into the brake mechanism.
// At standstill, with the lever in drive, a deliberate pedal over-travel past a preloaded pushrod spring (threshold above normal
// holding force) cocks a bistable β-titanium trip leaf in the buffer box; lifting off the brake lets it snap and push the charged cell
// over its release barrier. Interlocks with no added part: (1) standstill — while the wheels turn with the brake pressed, the regen
// carriage indexes and a fixed ramp on it knocks a cocked leaf back (disarm); (2) lever — in depower/park a bead on the lever strand
// sits in the leaf's path. A broken leaf cannot release (the cell catch holds), so the failure mode is "no launch".
export function pedalLaunchTrip({ Ebarrier = 9.8, Fcatch = 2773, Fthreshold = 160, Fmax = 190, normalStroke = 0.060, pedalBox = 0.150, eta = 0.85,
  launchesPerDay = 50, years = 15, leaf = { E: 100e9, rho: 4940, sigFat: 600e6 /* β-21S at ~3·10⁵ cycles */, util: 1 / 3 }, lobesPerRev = 24, wheelCirc = 2 * Math.PI * 0.24, pedalRatio = 4, pawlEngage = 0.008, pawlDwell = 0.130, preTol = 0.05, Fhold = 150, Fmc = 40, padFric = 1.0, sealFric = 5, hookDrag = 0.5, padWeight = 1.0, tauMax = 480e6, tauE = 350e6, tauU = 1200e6, boxLen = 0.200 } = {}) {
  const Fav = (Fthreshold + Fmax) / 2, over = Ebarrier / eta / Fav, stroke = normalStroke + over, Ncyc = launchesPerDay * 365 * years;
  const sigA = leaf.sigFat / 2, wDens = sigA * sigA / (2 * leaf.E) * leaf.util, V = Ebarrier / eta / wDens, mass = V * leaf.rho, SFfat = leaf.sigFat / sigA;
  // disarm window: a cocked leaf survives at most one carriage index; at walking pace (0.5 m/s) that is
  const window05 = wheelCirc / lobesPerRev / 0.5, window2 = wheelCirc / lobesPerRev / 2;
  // preloaded over-travel spring in the pushrod line (a counted moving part: its end moves with the pedal): pushrod-level threshold and rate
  // preloaded over-travel CARTRIDGE in the pushrod line (rigid below its preload): it also carries the pedal return — there is no separate
  // pedal return spring; the master cylinder's own return spring pushes the pushrod, the rigid cartridge and the pedal home
  const Fp0 = Fthreshold * pedalRatio, Fp1 = Fmax * pedalRatio, xp = over / pedalRatio, kS = (Fp1 - Fp0) / xp, xPre = Fp0 / kS, Espring = 0.5 * kS * (xPre + xp) ** 2;
  const tauA = tauMax * (xp / 2) / (xPre + xp), tauM = tauMax - tauA, SFspring = 1 / (tauA / tauE + tauM / tauU) /* Goodman, Cr-Si shot-peened; braking below the preload does not cycle it */;
  const springMass = Espring / (tauMax * tauMax / (4 * 79e9) * 0.5) * 7850 /* coil energy density τ²/4G × 0.5 utilisation */, lenCart = xPre + xp + 0.040 + 0.010, boxOK = lenCart <= boxLen;
  const thLo = Fthreshold * (1 - preTol), thHi = Fmax * (1 + preTol), repeatOK = thLo > Fhold && thHi <= 200;
  const retAvail = Fmc / pedalRatio, retNeed = padFric + sealFric / pedalRatio + hookDrag + padWeight, SFreturn = retAvail / retNeed, portSF = Fmc / sealFric;
  // regen pawl engaged over the whole pedal stroke (assumption made explicit): the pedal-side pawl actuator is a dwell ramp — engages at
  // pawlEngage, then constant radius for pawlDwell — so the disarm interlock works at any pedal position
  const pawlCover = pawlEngage + pawlDwell, pawlOK = pawlCover >= stroke;
  return { SFspring, cartridgeLen_mm: lenCart * 1000, boxLen_mm: boxLen * 1000, boxOK, firmPoint_N: [thLo, Fthreshold, thHi], repeatOK, SFreturn, portSF, retAvail_N: retAvail, pushrodThreshold_N: Fp0, springRate_Nmm: kS / 1000, springPreload_mm: xPre * 1000, springMass_kg: springMass, pawlEngage_mm: pawlEngage * 1000, pawlCover_mm: pawlCover * 1000, pawlOK, over_mm: over * 1000, stroke_mm: stroke * 1000, pedalBox_mm: pedalBox * 1000, footMax_N: Fmax, threshold_N: Fthreshold, leafMass_kg: mass, leafVol_cm3: V * 1e6, SFfatigue: SFfat, cycles: Ncyc,
    disarmWindow_s: { at0_5ms: window05, at2ms: window2 }, ok: Fmax <= 200 && stroke <= pedalBox && SFfat >= 2 && Fthreshold >= 150 && pawlOK && SFspring >= 2 && boxOK && repeatOK && SFreturn >= 2 && portSF >= 2 };
}
