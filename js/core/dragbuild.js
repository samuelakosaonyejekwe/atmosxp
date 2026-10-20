// Geometry-based drag-area build-up — the ONE build-up used by Suite 01 (road load, P20) and Suite 07 (aerodynamics → Suite 20),
// computed for every revision from its own CAD geometry and flags.
import { vehicleAssembly } from './cad.js';

// Geometry-based drag-area build-up, identical for every revision (computed from each design's own CAD geometry):
// body — Hoerner's streamlined-body formula on frontal area, C_D = C_f·(3 l/d + 4.5 (d/l)^½ + 21 (d/l)²) (Hoerner,
// Fluid-Dynamic Drag, 1965, ch. 6), turbulent C_f = 0.455/(log₁₀Re)^2.58, ground-proximity factor 1 + 0.4·e^(−(h/d)/0.15)
// (drag rise of streamlined bodies near the ground, cf. Hucho, Aerodynamics of Road Vehicles); wheels in spats / bare,
// exposed axle arms (C_d 1.0), spat–body junctions, wing-root junctions (Hoerner: ΔC_DA = t²·(0.8 (t/c)³ − 0.0003)),
// arch crossbeam, canopy/mirrors/lights/leakage, and +15 % mutual interference.
export function dragBuildup(veh, { V = 8, nu = 1.5e-5, finCdA = 0 } = {}) {
  const meta = vehicleAssembly(veh, {}).meta, BS = meta.bodyStation, AM = meta.arch, l = veh.length;
  const Af = 3.53 * BS.aMax * BS.bMax, d = Math.sqrt(4 * Af / Math.PI), Re = V * l / nu, Cf = 0.455 / Math.log10(Re) ** 2.58, CDb = Cf * (3 * l / d + 4.5 * Math.sqrt(d / l) + 21 * (d / l) ** 2), hd = Math.max(0, BS.zBotMin) / d, kg = 1 + 0.4 * Math.exp(-hd / 0.15);
  const nWh = veh.nWheels || 3, spats = veh.spats !== false, ax = meta.axle, arm = spats && ax ? ax.exposedArm : Math.max(0, (veh.track || 1.8) / 2 - (ax?.bodyHalfWidth ?? (veh.width || 1.6) / 2));
  const tc = 0.12, junc = (c) => { const t = tc * c; return Math.max(0, t * t * (0.8 * tc ** 3 - 0.0003)); };
  // wheels: front pair in spats (0.012 each; 0.009 with a flexible lower lip that closes the tyre cut-out, Hoerner wheel-fairing
  // openings) or bare (0.03); a rear wheel enclosed in the tail with only a tyre slot ≈ 0.003 (internal pumping + slot leakage)
  const nFront = nWh - 1, wFront = spats ? (veh.spatLip ? 0.009 : 0.012) : 0.03, wRear = veh.rearWheelEnclosed ? 0.003 : (spats ? 0.012 : 0.03);
  const blendedPods = spats && ax && ax.exposedArm <= 0.03, juncEach = spats ? (blendedPods ? 0.002 : 0.004) : 0.012;
  // canopy: faired bump on the hull, frontal area = bulge height × width, C_d 0.04 if flush-blended (no edge steps) else 0.08
  const cH = veh.canopyBulgeH ?? 0.18, cW = veh.canopyW ?? 0.55, cdCan = veh.canopyBlend ? 0.04 : 0.08;
  const rows = [[`Body (Hoerner, l/d ${(l / d).toFixed(2)}, A_f ${Af.toFixed(3)} m², C_f ${Cf.toFixed(4)}, ground ×${kg.toFixed(2)})`, CDb * Af * kg], [`Front wheels ×${nFront} ${spats ? (veh.spatLip ? 'in lipped spats' : 'in spats') : 'bare'}`, wFront * nFront], [`Rear wheel ${veh.rearWheelEnclosed ? 'enclosed in the tail (tyre slot)' : spats ? 'in a spat' : 'bare'}`, wRear], [`Exposed front axle arms (2 × ${(arm * 1000).toFixed(0)} mm × 30 mm, C_d 1.0)`, 2 * arm * 0.03],
    [spats ? `Spat–body junctions (2, ${blendedPods ? 'blended pods' : 'plain'})` : 'Wheel–body interference (open)', 2 * juncEach],
    ...(AM ? [['Arch root crossbeam fairing (C_d 0.1 on 0.1 m × span)', 0.1 * 0.1 * (AM.gap + 0.08)], ['Arch leg–crossbeam junctions (2, Hoerner)', 2 * junc(AM.rootChord)]] : [[`Wing–body root junctions (${veh.nWings || 2}, Hoerner)`, (veh.nWings || 2) * junc(meta.cr)]]),
    [`Canopy bulge (${(cH * 1000).toFixed(0)} × ${(cW * 1000).toFixed(0)} mm, C_d ${cdCan})`, cdCan * cH * cW], ['Mirrors (2)', 0.004], ['Lights (flush, passive)', 0], ['Panel and seal leakage (common construction standard)', 0.004]];
  // root endplate above the deck (yaws with the arch): turbulent skin friction on both faces at its own length
  const pW = veh.rootPlateW ?? 0, pL = veh.rootPlateL ?? 0; if (AM && pW > 0 && pL > 0) { const Ap = Math.PI / 4 * pW * pL, Cfp = 0.455 / Math.log10(V * pL / nu) ** 2.58; rows.push([`Root endplate ${pW} × ${pL} m elliptical (both faces, C_f ${Cfp.toFixed(4)})`, 2 * Cfp * Ap]); }
  if (finCdA > 0) rows.push(['Thermal-store rejection fins (Suite 14)', finCdA]);
  const sub = rows.reduce((a, r) => a + r[1], 0), CdA = sub * 1.15;
  return { rows: [...rows, ['Mutual interference (+15 %)', sub * 0.15]], CdA, body: { Af, d, ld: l / d, Cf, CDb, kg } };
}
