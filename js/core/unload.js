// Wing-unloading hardware of the Revision 5.1 arch (no rotating parts, no electricity; its moving parts are counted in Suite 01):
//  (1) passive pressure relief — one monolithic KIRIGAMI RELIEF PANEL per leg and one in the bridge: a laser-cut pattern of
//      bistable flaps in the glass-reinforced TPU skin (curved ligaments). The flaps pop out of plane together when the mean
//      pressure difference reaches Δp_open and fold back elastically below Δp_close; one elastic part per panel, no hinges;
//  (2) the driver lever is the EXISTING parking-lock lever with ONE 3-position gate, in order of cable stroke — DRIVE / DEPOWER / PARK
//      (there is no hub sail position: no design can hold the pads clear of the rotating cartridges, Suite 01). One closed-loop
//      strand runs through the three panels in series: depower stretches the panels past their pop-up strain so all flaps open at
//      any load (crosswind, braking, Suite 12); park adds the forked wedge on the strand (yaw end stops) and the strand bead that holds
//      the brake pedal (parking brake, Suite 12) through a 4:1 park-step profile. Park is reachable only through depower (interlock).
// Ligament strain ε = t·θ/(2·L_lig); flap opening moment M = Δp·b·a²/2; pop-up work by energy (barrier + ligament bending).
import { G0 } from './num.js';

export function unloadingDesign({ S, legHeight, bridgeSpan = 0.9, chord, vents, FyS, skin, tSkin = 0.35, flapA = 0.07, flapB = 0.06,
  thetaMax = 85, ligW = 0.004, SFtarget = 2, ratio = 6, strokeHand = 0.15, eta = 0.7, series = false, cantDeg = 15, nGuides = 8, muGuide = 0.10, etaLever = 0.9, Fresid = 3, cordL = 1.0, cordD = 0.007, cordE = 2.0, cordPre = 0.15, returnInPanel = false, returnMode = returnInPanel ? 'panel' : 'cord' }) {
  const Es = (skin?.E ?? 0.8) * 1e9, fs = (skin?.fatigue ?? 15) * 1e6, t = tSkin / 1000, th = thetaMax * Math.PI / 180;
  const dpO = vents.dpOpen ?? vents.dpOpenK * FyS / S, dpC = (vents.dpCloseK ?? 0.5) * dpO;
  // ligament length for the fatigue target at full opening
  const epsAllow = fs / Es / SFtarget, Llig = Math.max(0.01, t * th / (2 * epsAllow)), eps = t * th / (2 * Llig), SF = fs / (Es * eps);
  // panel layout (two legs + bridge); vented area = zone × arch area
  const Aven = vents.zone * S, flapCell = flapA * flapB * 1.15, nFlap = Math.round(Aven / flapCell), spans = [legHeight * 0.97, legHeight * 0.97, bridgeSpan * 0.97];
  const panels = spans.map((L, i) => ({ name: i < 2 ? `Leg kirigami relief panel (${i ? 'starboard' : 'port'})` : 'Bridge kirigami relief panel', span: L, chordLE: 0.10 * chord, chordAft: 0.68 * chord, flaps: Math.round(nFlap * L / spans.reduce((a, b) => a + b, 0)) }));
  // pressure that would hold a flap at full opening on its ligaments alone (the bistable barrier sets Δp_open above it)
  const kLig = 2 * Es * ligW * t ** 3 / (12 * Llig), dpHold = 2 * kLig * th / (flapB * flapA ** 2), Mopen = dpO * flapB * flapA ** 2 / 2;
  // lever: work to pop every flap over its barrier (≈ M_open × 0.2 rad) and hold it open on its ligaments
  const Wpop = nFlap * (Mopen * 0.2 + 0.5 * kLig * th ** 2);
  // SERIES strand (one cable continuing through the three panel edge ribs: port leg → bridge → starboard leg, anchored at the far
  // leg root, PTFE-lined guides): capstan loss over the routed wrap — the root turn and the two leg/bridge corners
  // (each 90° + leg splay) plus a small deviation at every rib guide; conservatively the whole pop work is taken at the far end.
  const wrap = 3 * (Math.PI / 2 + Math.abs(cantDeg) * Math.PI / 180) + nGuides * 0.05, cap = Math.exp(muGuide * wrap);
  const strokePanel = strokeHand / ratio, ratioUsed = series ? strokeHand / (3 * strokePanel) : ratio; // series: the three panel strokes add, so the lever ratio drops to keep the same hand travel
  // series return element. Option A (default): a pre-stretched silicone cord (non-absorbing) from the strand end to the far leg-root
  // fitting. Option B (returnInPanel): a meander spring moulded into the far panel's glass-reinforced TPU edge rib. Both are taken
  // conservatively as deflecting by the whole strand travel; their work is added to the lever and their tension to the return.
  // Temperature / time factors on stiffness (−20 °C, +60 °C) and stress retained after 1000 h held stretched at +60 °C (long
  // parking in depower): TPU-glass 1.6 / 0.55 / 0.60; silicone 1.10 / 0.95 / 0.90. The kirigami flaps are the same TPU-glass,
  // so their pop work and hold-back follow the TPU factors. Duty: 20 lever cycles a day for 15 years.
  const sTot = 3 * strokePanel, Ncyc = 20 * 365 * 15, TPU = { cold: 1.6, hot: 0.55, ret: 0.60 }, SIL = { cold: 1.10, hot: 0.95, ret: 0.90 };
  const Tflap = nFlap / 3 * 0.5 * kLig * th ** 2 / Math.max(1e-6, strokePanel), Tret = Fresid * cap;
  let leafPre = 0, FleafMax = 0, SFleaf = Infinity, epsMax = 0, RET = SIL, inPanel = null, Acord = Math.PI / 4 * cordD ** 2;
  const loop = series && returnMode === 'loop'; // option C: the strand's far end runs back to the lever (closed loop) and engages it through a lost-motion slot
  if (series && returnMode === 'cord') { leafPre = cordE * 1e6 * Acord * cordPre; epsMax = cordPre + sTot / cordL; FleafMax = cordE * 1e6 * Acord * epsMax; SFleaf = 0.5 / epsMax; /* silicone: ≤ 50 % working strain for long cyclic life */ }
  if (series && returnMode === 'panel') { RET = TPU; // size the moulded spring for the hot, relaxed return, then check the cold lever force, fatigue and fit
    const d0 = 0.15, need = Math.max(0, 2 * Tret - Tflap * TPU.hot), Fpre = need / (TPU.hot * TPU.ret), Fmx = Fpre * (1 + sTot / d0), epsFull = 0.012, epsA = epsFull * (sTot / 2) / (d0 + sTot);
    const sigN = (skin?.fatigue ?? 15) * Math.pow(1e7 / Ncyc, 1 / 10), SFfat = sigN / ((skin?.E ?? 0.8) * 1e3) / epsA, Ldev = (d0 + sTot) / epsFull, Lfit = Ldev * 0.006 / 0.05, Lrib = spans[1];
    leafPre = Fpre; FleafMax = Fmx; SFleaf = SFfat; epsMax = epsFull; inPanel = { Fpre_N: Fpre, Fmax_N: Fmx, SFfatigue: SFfat, meanderLength_m: Lfit, edgeRib_m: Lrib, fits: Lfit <= 0.5 * Lrib, cycles: Ncyc }; }
  const Wret = (k) => series ? (leafPre + FleafMax) / 2 * sTot * cap * k : 0;
  const Fhand = series ? (Wpop * cap + Wret(1)) / (etaLever * strokeHand) : Wpop / (eta * strokeHand), Fcable = Fhand * ratioUsed, tendonStroke = strokePanel;
  const FhandCold = series ? (Wpop * cap * TPU.cold + Wret(RET.cold)) / (etaLever * strokeHand) : Wpop * TPU.cold / (eta * strokeHand);
  // full return: with the lever back in drive the strand is free at the lever end (gate residual F_resid); the hold-back tension of the
  // flaps + return element must draw the strand back through the guides: T_hold ≥ 2·F_resid·e^{μθ}, at 23 °C and hot after long parking
  let Thold = Tflap + leafPre, TholdHot = Tflap * TPU.hot + leafPre * RET.hot * RET.ret, retSF = Thold / Math.max(1e-9, Tret), retSFhot = TholdHot / Math.max(1e-9, Tret), LOOP = null;
  if (loop) { // positive return: in depower the return-run nipple rests on a fixed stop in the gate plate (it is the anchor); back toward drive
    // the lever's slot end picks it up and draws the strand back. Steel strand vs composite arch over −20…+60 °C changes the slack.
    const lRet = 0.6 + 3.0, wrapRet = Math.PI / 2 + Math.abs(cantDeg) * Math.PI / 180 + 4 * 0.05, capLoop = Math.exp(muGuide * (wrap + wrapRet)), Tback = Fresid * capLoop,
      FhandRet = Tback / ratioUsed / etaLever, lLoop = 3.0 + spans.reduce((x, y) => x + y, 0) + 0.6 + lRet, slackT = 80 * (17.3e-6 - 3e-6) * lLoop, slot = sTot + slackT;
    Thold = 60 * etaLever * ratioUsed; TholdHot = Thold; retSF = Thold / Math.max(1e-9, Tback); retSFhot = retSF;
    LOOP = { returnRun_m: lRet, loopLength_m: lLoop, capLoop, Tback_N: Tback, FhandReturn_N: FhandRet, slackThermal_mm: slackT * 1000, slot_mm: slot * 1000, handStroke_mm: (strokeHand + slackT * ratioUsed) * 1000 }; }
  const seriesCheck = series ? { wrap_rad: wrap, mu: muGuide, capstan: cap, Fhand_N: Fhand, FhandCold_N: FhandCold, Fcable_N: Fcable, ratio: ratioUsed, strokeCable_m: sTot, Thold_N: Thold, TholdHot_N: TholdHot, Treturn_N: Tret, returnSF: retSF, returnSFhot: retSFhot, cycles: Ncyc,
    returnElement: returnMode, inPanel, loop: LOOP, leaf: { L: cordL, d: cordD, pre_N: leafPre, max_N: FleafMax, epsMax, SF: SFleaf, mass_kg: returnMode === 'panel' ? 0.02 : returnMode === 'loop' ? 0 : cordL * Acord * 1150 + 0.01 },
    ok: Fhand <= 60 && FhandCold <= 60 && retSF >= 2 && retSFhot >= 2 && (loop || SFleaf >= 2) && (!inPanel || inPanel.fits) } : null;
  const openArea = vents.phiMax, kerf = vents.phiClosed ?? 0;
  // added mass: flap-edge reinforcement + panel edge ribs (the panels replace skin area) + cable extension and 3 tendon tails
  const lStrand = 3.0 + (series ? spans.reduce((x, y) => x + y, 0) + 0.6 + (returnMode === 'loop' ? 3.6 : 0) : 3 * 1.2), mCordX = series && returnMode === 'cord' ? cordL * Math.PI / 4 * cordD ** 2 * 1150 + 0.01 : 0, mTendon = lStrand * 0.009 + (series ? 3.6 + (returnMode === 'loop' ? 1.2 : 0) : 3.0) * 0.03 /* PTFE liner & guides */ + mCordX /* return cord + end fittings */, mEdge = 0.12 * panels.length, mRib = 0.08 * panels.length, mass = mEdge + mRib + mTendon;
  const parts = [
    ...panels.map(pn => [pn.name, 1, `${pn.span.toFixed(2)} m span; slit zones ${(pn.chordLE * 1000).toFixed(0)} mm (2–12 % c) + ${(pn.chordAft * 1000).toFixed(0)} mm (30–98 % c); ${pn.flaps} flaps ${(flapA * 1000).toFixed(0)} × ${(flapB * 1000).toFixed(0)} mm, ligaments ${(ligW * 1000).toFixed(0)} × ${(Llig * 1000).toFixed(0)} mm`, (mEdge + mRib) / panels.length, 'replaces skin area (elastic, 0 moving parts)']),
    ['Parking-lock lever, 3-position gate (drive / depower / park; 4:1 park-step profile)', 1, `ratio ${ratio}:1, stroke ${(strokeHand * 1000).toFixed(0)} mm`, 0, 'replaces the 2-position parking-lock lever; gate detent counted as a moving part (Suite 01)'],
    ...(series ? [['Parking-lock cable continued as ONE strand through the three panel edge ribs in series (stainless 7×19 Ø1.5 mm, PTFE-lined guides), anchored at the far leg root', 1, `${lStrand.toFixed(1)} m; wrap ${(wrap * 180 / Math.PI).toFixed(0)}°, capstan ×${cap.toFixed(2)}${returnMode === 'loop' ? '; closed loop back to the lever (lost-motion slot), positive return' : ''}`, mTendon - mCordX, 'one strand (Suite 01 counts it once)'], ...(returnMode === 'cord' ? [['Series-strand return cord (pre-stretched silicone, strand end → far leg-root fitting)', 1, `Ø${(cordD * 1000).toFixed(0)} × ${(cordL * 1000).toFixed(0)} mm, ${leafPre.toFixed(1)} → ${FleafMax.toFixed(1)} N`, cordL * Math.PI / 4 * cordD ** 2 * 1150 + 0.01, 'its end moves with the strand: counted as a moving part (Suite 01)']] : [])] : [['Parking-lock cable extended to the pylon root + 3 tendon tails (stainless 7×19 Ø1.5 mm, PTFE-lined)', 1, '3.0 m extension + 3 × 1.2 m tails', mTendon, 'one cable with branches; every moving strand is counted in Suite 01']])];
  return { dpOpen: dpO, dpClose: dpC, nFlap, panels, Llig, epsLig: eps, SFlig: SF, dpHold, Mopen, Wpop, Fhand, Fcable, tendonStroke, ratio: ratioUsed, series: seriesCheck, openArea, kerf, mass, parts, weightN: mass * G0,
    newMechanisms: 0, newMovingParts: series ? (returnMode === 'cord' ? 1 : 0) : 3 }; // the lever chain replaces the parking-brake lever chain; separate tendon tails are new moving strands, the series strand is the existing cable
}
