// Wing-unloading hardware of the Revision 5.1 arch (no rotating parts, no electricity, no added mechanism):
//  (1) passive pressure relief — one monolithic KIRIGAMI RELIEF PANEL per leg and one in the bridge: a laser-cut pattern of
//      bistable flaps in the glass-reinforced TPU skin (curved ligaments). The flaps pop out of plane together when the mean
//      pressure difference reaches Δp_open and fold back elastically below Δp_close; one elastic part per panel, no hinges;
//  (2) the driver lever is the EXISTING parking-lock lever with a three-position gate — drive / depower / park-and-limit. Its
//      existing cable is extended past the pylon root with three tendon tails, one per panel edge rib: in depower the tendons
//      stretch each panel spanwise past its pop-up strain so all flaps open at any load; park adds the yaw-limiter pads and the
//      parking lock. The gate passes through depower to reach park, so the parking lock can only be set with the arch unloaded.
// Ligament strain ε = t·θ/(2·L_lig); flap opening moment M = Δp·b·a²/2; pop-up work by energy (barrier + ligament bending).
import { G0 } from './num.js';

export function unloadingDesign({ S, legHeight, bridgeSpan = 0.9, chord, vents, FyS, skin, tSkin = 0.35, flapA = 0.07, flapB = 0.06,
  thetaMax = 75, ligW = 0.004, SFtarget = 2, ratio = 6, strokeHand = 0.15, eta = 0.7 }) {
  const Es = (skin?.E ?? 0.8) * 1e9, fs = (skin?.fatigue ?? 15) * 1e6, t = tSkin / 1000, th = thetaMax * Math.PI / 180;
  const dpO = vents.dpOpen ?? vents.dpOpenK * FyS / S, dpC = (vents.dpCloseK ?? 0.5) * dpO;
  // ligament length for the fatigue target at full opening
  const epsAllow = fs / Es / SFtarget, Llig = Math.max(0.01, t * th / (2 * epsAllow)), eps = t * th / (2 * Llig), SF = fs / (Es * eps);
  // panel layout (two legs + bridge); vented area = zone × arch area
  const Aven = vents.zone * S, flapCell = flapA * flapB * 1.15, nFlap = Math.round(Aven / flapCell), spans = [legHeight * 0.92, legHeight * 0.92, bridgeSpan * 0.92];
  const panels = spans.map((L, i) => ({ name: i < 2 ? `Leg kirigami relief panel (${i ? 'starboard' : 'port'})` : 'Bridge kirigami relief panel', span: L, chordLE: 0.10 * chord, chordAft: 0.67 * chord, flaps: Math.round(nFlap * L / spans.reduce((a, b) => a + b, 0)) }));
  // pressure that would hold a flap at full opening on its ligaments alone (the bistable barrier sets Δp_open above it)
  const kLig = 2 * Es * ligW * t ** 3 / (12 * Llig), dpHold = 2 * kLig * th / (flapB * flapA ** 2), Mopen = dpO * flapB * flapA ** 2 / 2;
  // lever: work to pop every flap over its barrier (≈ M_open × 0.2 rad) and hold it open on its ligaments
  const Wpop = nFlap * (Mopen * 0.2 + 0.5 * kLig * th ** 2), Fhand = Wpop / (eta * strokeHand), Fcable = Fhand * ratio, tendonStroke = strokeHand / ratio;
  const openArea = vents.phiMax, kerf = vents.phiClosed ?? 0;
  // added mass: flap-edge reinforcement + panel edge ribs (the panels replace skin area) + cable extension and 3 tendon tails
  const mEdge = 0.12 * panels.length, mRib = 0.08 * panels.length, mTendon = (3.0 + 3 * 1.2) * 0.009 + 3.0 * 0.03, mass = mEdge + mRib + mTendon;
  const parts = [
    ...panels.map(pn => [pn.name, 1, `${pn.span.toFixed(2)} m span; slit zones ${(pn.chordLE * 1000).toFixed(0)} mm (2–12 % c) + ${(pn.chordAft * 1000).toFixed(0)} mm (30–97 % c); ${pn.flaps} flaps ${(flapA * 1000).toFixed(0)} × ${(flapB * 1000).toFixed(0)} mm, ligaments ${(ligW * 1000).toFixed(0)} × ${(Llig * 1000).toFixed(0)} mm`, (mEdge + mRib) / panels.length, 'replaces skin area (elastic, 0 moving parts)']),
    ['Parking-lock lever, 3-position gate (drive / depower / park)', 1, `ratio ${ratio}:1, stroke ${(strokeHand * 1000).toFixed(0)} mm`, 0, 'replaces the 2-position parking-lock lever (same part count)'],
    ['Parking-lock cable extended to the pylon root + 3 tendon tails (stainless 7×19 Ø1.5 mm, PTFE-lined)', 1, '3.0 m extension + 3 × 1.2 m tails', mTendon, 'replaces the parking-lock cable (same part count)']];
  return { dpOpen: dpO, dpClose: dpC, nFlap, panels, Llig, epsLig: eps, SFlig: SF, dpHold, Mopen, Wpop, Fhand, Fcable, tendonStroke, ratio, openArea, kerf, mass, parts, weightN: mass * G0,
    newMechanisms: 0, newMovingParts: 0 };
}
