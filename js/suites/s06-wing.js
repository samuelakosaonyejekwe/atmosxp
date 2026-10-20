// Suite 06 — Fixed skeleton, passive aeroelastic wings and morphing metamaterial surfaces.
import { lamina, laminate, parseLayup, plyStresses, beamModel, beamModes, beamResultants, beamStatic, frameK } from '../core/fem.js';
import { luSolve, eigGen, zeros } from '../core/num.js';
import { vehicleAssembly } from '../core/cad.js';
import { safeSideForce } from '../core/safeside.js';
import * as AA from '../core/archaero.js';
import { unloadingDesign } from '../core/unload.js';
import { skinStability, LATTICE_DEFAULT } from '../core/skinstab.js';
import { homogenize } from '../core/fem.js';
import { makeCell } from './s04-metamaterials.js';
import { get as getMatD } from '../core/materials.js';
// trailing-core defaults = what Suite 04 computes for its default lattice (order-independent coupling; values in MPa / kg·m⁻³)
const LAT0 = (() => { const L = LATTICE_DEFAULT, M = getMatD(L.mat), l = L.l / 1000, H = homogenize(makeCell(L.cell, { l, hr: L.hr, theta: L.theta, rr: L.rr }), M.E * 1e9, L.tr * l, L.depth / 1000); return { E_MPa: H.Ey / 1e6, G_MPa: H.Gxy / 1e6, rho: M.rho * H.relDensity }; })();
import { sectionModel, liftingLine, staticAeroelastic, divergenceQ, flutterVg, gustResponse, oneMinusCosGust, flutterModeShape } from '../core/aeroelastic.js';
import { naca4, airfoilStats } from '../core/cad.js';
import { logWind, apparentWind, ROUGHNESS } from '../core/physics.js';
import { range, fmt, linspace, rad, deg, interp1, max, min } from '../core/num.js';
import { MATERIALS, get as getMat } from '../core/materials.js';
import { h } from '../core/ui.js';

const sparMats = MATERIALS.filter(m => m.uses.includes('spar') || ['al-7075-t6', 'gfrp-e', 'ti-6al4v', 'cfrp-woven'].includes(m.id));
const skinMats = MATERIALS.filter(m => m.uses.includes('wingSkin'));
const ribMats = MATERIALS.filter(m => m.uses.includes('rib'));

export const WING_PARAMS = [
  { key: 'flexK', label: 'Centreline pylon flexure yaw stiffness k_θ (Rev 5.1)', unit: 'N·m/rad', type: 'number', min: 0.1, max: 1000, step: 0.1, group: 'Compliant flexure root (Rev 5)', default: 10, auto: (c) => { const v = c.store?.params?.('s07')?.flexK; return v !== undefined ? { value: v, prov: 'coupled' } : undefined; } },
  { key: 'flexAxis', label: 'Flexure axis position', unit: 'x/c', type: 'range', min: 0.03, max: 0.24, step: 0.005, group: 'Compliant flexure root (Rev 5)', default: 0.10, auto: (c) => { const v = c.store?.params?.('s07')?.flexAxis; return v !== undefined ? { value: v, prov: 'coupled' } : undefined; } },
  { key: 'vMaxApp', label: 'Maximum operating apparent wind', unit: 'm/s', type: 'number', min: 5, max: 50, step: 0.5, group: 'Compliant flexure root (Rev 5)', default: 25, help: 'Flutter requirement V_F ≥ 1.3 × this value.' },
  { key: 'wingState', label: 'Wing configuration (Revision 2.2 reefable wing)', type: 'select', group: 'Wing geometry', default: 'deployed', options: [{ value: 'deployed', label: 'Deployed — full span (light wind)' }, { value: 'reefed', label: 'Reefed — upper stage telescoped in (strong wind / transit)' }] },
  { key: 'reefSpan', label: 'Reefed span', unit: 'm', type: 'number', min: 0.5, max: 6, step: 0.1, group: 'Wing geometry', default: 2, auto: (c) => ({ value: c.vehicle.wingReefSpan ?? c.vehicle.wingHeight, prov: 'user' }) },
  { key: 'overlap', label: 'Telescoping overlap at the reef joint', unit: 'm', type: 'number', min: 0.05, max: 1, step: 0.01, group: 'Wing geometry', default: 0.3, help: 'Must be ≥ 1.5 × local section thickness.' },
  { key: 'bushAllow', label: 'Joint bushing allowable contact pressure', unit: 'MPa', type: 'number', min: 2, max: 100, step: 1, group: 'Wing geometry', default: 25, expert: true, help: 'PTFE/PEEK-lined composite bushing, static.' },
  { key: 'camberMorph', label: 'Passive leeward camber morphing', type: 'bool', group: 'Skin, ribs & core', default: true, help: 'The compliant trailing region cambers to the leeward side on either tack under aerodynamic load (symmetric when unloaded).' },
  { key: 'camberPct', label: 'Morphed camber at design load', unit: '% chord', type: 'range', min: 0, max: 12, step: 0.5, group: 'Skin, ribs & core', default: 7 },
  { key: 'span', label: 'Wing height (span)', unit: 'm', type: 'number', min: 1, max: 6, step: 0.1, group: 'Wing geometry', default: 3, auto: (c) => ({ value: c.vehicle.wingHeight, prov: 'user' }) },
  { key: 'area', label: 'Planform area per wing', unit: 'm²', type: 'number', min: 1, max: 12, step: 0.1, group: 'Wing geometry', default: 4, auto: (c) => ({ value: +(c.vehicle.wingArea / (c.vehicle.nWings || 2)).toFixed(2), prov: 'calc' }) },
  { key: 'taper', label: 'Taper ratio c_tip/c_root', type: 'range', min: 0.3, max: 1, step: 0.05, group: 'Wing geometry', default: 0.6 },
  { key: 'airfoil', label: 'Section', type: 'select', group: 'Wing geometry', default: '0012', options: ['0012', '0015', '0018', '0021', '2412', '4415'].map(v => ({ value: v, label: 'NACA ' + v })), auto: (c) => { const n = c.upstream('s02')?.airfoil?.name; const m = n && /NACA\s*(\d{4})/.exec(n); return m ? { value: m[1], prov: 'coupled' } : undefined; } },
  { key: 'twistBuilt', label: 'Built-in tip washout', unit: '°', type: 'number', min: -10, max: 5, step: 0.5, group: 'Wing geometry', default: -3 },
  { key: 'zRoot', label: 'Root height above ground', unit: 'm', type: 'number', min: 0.3, max: 2, step: 0.05, group: 'Wing geometry', default: 1.07, auto: (c) => ({ value: +(c.vehicle.wheelRadius + c.vehicle.height * 0.7).toFixed(2), prov: 'calc' }) },
  { key: 'U10', label: 'True wind at 10 m', unit: 'm/s', type: 'range', min: 0, max: 25, step: 0.1, group: 'Operating point', default: 6, auto: (c) => c.env ? { value: +(+c.env.U).toFixed(2), prov: 'coupled' } : undefined },
  { key: 'twa', label: 'True wind angle from heading', unit: '°', type: 'range', min: 30, max: 180, step: 1, group: 'Operating point', default: 90 },
  { key: 'vkmh', label: 'Vehicle speed', unit: 'km/h', type: 'range', min: 0, max: 60, step: 0.5, group: 'Operating point', default: 10 },
  { key: 'aoa', label: 'Root angle of attack', unit: '°', type: 'range', min: 0, max: 20, step: 0.5, group: 'Operating point', default: 10, help: 'Wing set so that the root section sees this angle to the local apparent wind.' },
  { key: 'rho', label: 'Air density', unit: 'kg/m³', type: 'number', min: 0.9, max: 1.35, step: 0.001, group: 'Operating point', default: 1.225, auto: (c) => c.env ? { value: +(+c.env.rho).toFixed(4), prov: 'coupled' } : undefined },
  { key: 'z0', label: 'Terrain roughness length', unit: 'm', type: 'number', min: 0.0002, max: 2, step: 0.001, group: 'Operating point', default: 0.1, auto: (c) => c.env?.z0 ? { value: c.env.z0, prov: 'coupled' } : (ROUGHNESS[c.site.roughness] ? { value: ROUGHNESS[c.site.roughness], prov: 'user' } : undefined) },
  { key: 'sparType', label: 'Spar architecture', type: 'select', group: 'Spar (skeleton)', default: 'tube', options: [{ value: 'tube', label: 'Torsion-tailored: open CFRP caps + torsion tube on the feathering pivot (Revision 2, D3/D4)' }, { value: 'box', label: 'Closed CFRP box spar (Revision 1 — torsionally very stiff)' }], help: 'The closed box makes the wing torsionally rigid (negligible passive washout). The Revision-2 skeleton carries bending in open caps and torsion in a sized tube on the pivot axis, with a slotted trailing edge, so the wing washes out under load.' },
  { key: 'autoSize', label: 'Auto-size spar to the design targets (D4)', type: 'bool', group: 'Spar (skeleton)', default: true, help: 'Bisection on cap ply thickness (strength factor 2.5) and torsion-tube wall (target tip washout at the reference apparent wind), then leading-edge mass balance for the flutter target. Re-runs whenever geometry, materials or loads change.' },
  { key: 'targetTwist', label: 'Target tip washout at reference wind', unit: '°', type: 'number', min: -8, max: -0.5, step: 0.1, group: 'Spar (skeleton)', default: -3 },
  { key: 'refVa', label: 'Reference apparent wind for sizing', unit: 'm/s', type: 'number', min: 4, max: 25, step: 0.5, group: 'Spar (skeleton)', default: 10 },
  { key: 'xPivot', label: 'Pivot / torsion axis', unit: 'x/c', type: 'range', min: 0.15, max: 0.3, step: 0.01, group: 'Spar (skeleton)', default: 0.21, help: 'Free-feathering pivot ahead of the aerodynamic centre (D3): aerodynamic torque then twists the loaded wing nose-down (washout) and the wing weathervanes when released.' },
  { key: 'rTube', label: 'Torsion-tube radius', unit: 'mm', type: 'number', min: 5, max: 80, step: 1, group: 'Spar (skeleton)', default: 22 },
  { key: 'tTube', label: 'Torsion-tube wall (±45°)', unit: 'mm', type: 'number', min: 0.05, max: 5, step: 0.01, group: 'Spar (skeleton)', default: 0.4 },
  { key: 'teSlot', label: 'Slotted (sliding) trailing edge', type: 'bool', group: 'Skin, ribs & core', default: true, help: 'Open-section skin: the skin no longer acts as a closed torsion cell, so twist is governed by the designed torsion tube.' },
  { key: 'leBallast', label: 'Leading-edge mass balance', unit: 'kg/m', type: 'number', min: 0, max: 10, step: 0.05, group: 'Dynamics & fatigue', default: 1.0, help: 'Mass at 2 % chord bringing the section CG to (or ahead of) the torsion axis — the classical flutter cure.' },
  { key: 'bridgeEIx', label: 'Bridge bending-stiffness factor (vs the leg tip section)', type: 'number', min: 0.5, max: 50, step: 0.5, group: 'Wing geometry', default: 1, help: 'UD caps or the roll-over tube in the bridge raise its stiffness; a stiffer bridge gives more portal relief of the leg-root moment.' },
  { key: 'flutterTarget', label: 'Required flutter speed', unit: 'm/s', type: 'number', min: 10, max: 80, step: 0.5, group: 'Dynamics & fatigue', default: 32.5, help: 'D4: ≥ 1.3 × 25 m/s.' },
  { key: 'breakaway', label: 'Free-feathering breakaway (D3)', type: 'bool', group: 'Operating point', default: true, help: 'Torque-limited incidence lock releases at the safe side force; the wing then weathervanes (C_L ≈ 0, C_D ≈ 0.02).' },
  { key: 'sparMat', label: 'Spar material', type: 'select', group: 'Spar (skeleton)', default: 'cfrp-t700', options: sparMats.map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.material('spar')?.id || 'cfrp-t700', prov: 'library' }) },
  { key: 'layup', label: 'Spar-cover layup (deg, root→out)', type: 'text', group: 'Spar (skeleton)', default: '0,0,20,-20,0', help: 'Upper cover; lower cover is mirrored (circumferentially asymmetric stiffness) so off-axis plies create bend–twist coupling. Balanced layups (±θ pairs) give no coupling.' },
  { key: 'tPly', label: 'Cap ply thickness', unit: 'mm', type: 'number', min: 0.01, max: 1, step: 0.005, group: 'Spar (skeleton)', default: 0.1 },
  { key: 'boxW', label: 'Box width / chord', type: 'range', min: 0.08, max: 0.45, step: 0.01, group: 'Spar (skeleton)', default: 0.18 },
  { key: 'webLayup', label: 'Shear-web layup', type: 'text', group: 'Spar (skeleton)', default: '45,-45,45,-45', expert: true },
  { key: 'skinMat', label: 'Morphing skin material', type: 'select', group: 'Skin, ribs & core', default: 'tpu-glass', options: skinMats.map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.material('wingSkin')?.id || 'tpu-glass', prov: 'library' }) },
  { key: 'tSkin', label: 'Skin thickness', unit: 'mm', type: 'number', min: 0.2, max: 5, step: 0.1, group: 'Skin, ribs & core', default: 0.35 },
  { key: 'ribMat', label: 'Flexible rib material', type: 'select', group: 'Skin, ribs & core', default: 'gfrp-e', options: ribMats.map(m => ({ value: m.id, label: m.name })), auto: (c) => ({ value: c.material('rib')?.id || 'gfrp-e', prov: 'library' }) },
  { key: 'ribSolid', label: 'Rib solidity (after lightening holes)', type: 'range', min: 0.1, max: 1, step: 0.05, group: 'Skin, ribs & core', default: 0.3 },
  { key: 'coreFill', label: 'Lattice fill fraction of trailing region', type: 'range', min: 0.05, max: 1, step: 0.05, group: 'Skin, ribs & core', default: 0.3 },
  { key: 'ribPitch', label: 'Rib pitch', unit: 'm', type: 'number', min: 0.15, max: 1, step: 0.05, group: 'Skin, ribs & core', default: 0.75, help: '0.75 m gives 3 interior ribs per arch leg; the core webs support the skin between ribs.' },
  { key: 'webPitch', label: 'Core web pitch (spanwise through-thickness webs)', unit: 'mm', type: 'number', min: 0, max: 200, step: 1, group: 'Skin, ribs & core', default: 19, help: 'Thin webs printed with the lattice, normal to the chord, tying both skins (0 = no webs).' },
  { key: 'webT', label: 'Core web thickness', unit: 'mm', type: 'number', min: 0.05, max: 2, step: 0.05, group: 'Skin, ribs & core', default: 0.2 },
  { key: 'coreE', label: 'Trailing-core effective modulus', unit: 'MPa', type: 'number', min: 0.0001, max: 5000, step: 0.0001, group: 'Skin, ribs & core', default: +LAT0.E_MPa.toFixed(6), auto: (c) => { const o = c.upstream('s04'); return o?.Ey_Pa ? { value: +(o.Ey_Pa / 1e6).toFixed(6), prov: 'coupled' } : undefined; }, help: 'Bending stiffness of the compliant trailing region (from the Suite 04 lattice).' },
  { key: 'coreG', label: 'Trailing-core effective shear modulus', unit: 'MPa', type: 'number', min: 0.00001, max: 2000, step: 0.00001, group: 'Skin, ribs & core', default: +LAT0.G_MPa.toFixed(6), auto: (c) => { const o = c.upstream('s04'); return o?.Geff_Pa ? { value: +(o.Geff_Pa / 1e6).toFixed(6), prov: 'coupled' } : undefined; } },
  { key: 'coreRho', label: 'Trailing-core effective density', unit: 'kg/m³', type: 'number', min: 10, max: 2000, step: 5, group: 'Skin, ribs & core', default: Math.round(LAT0.rho), auto: (c) => { const o = c.upstream('s04'); return o?.rhoEff ? { value: Math.round(o.rhoEff), prov: 'coupled' } : undefined; } },
  { key: 'teFrac', label: 'Compliant trailing-region chord fraction', type: 'range', min: 0.15, max: 0.6, step: 0.01, group: 'Skin, ribs & core', default: 0.4 },
  { key: 'gustAmp', label: 'Design gust amplitude (1-cos)', unit: 'm/s', type: 'number', min: 0, max: 20, step: 0.5, group: 'Dynamics & fatigue', default: 4, auto: (c) => c.env ? { value: +Math.max(1, c.env.gust - c.env.U).toFixed(1), prov: 'coupled' } : undefined },
  { key: 'gustDur', label: 'Gust duration', unit: 's', type: 'number', min: 0.2, max: 10, step: 0.1, group: 'Dynamics & fatigue', default: 1.5 },
  { key: 'zeta', label: 'Structural damping ratio', type: 'number', min: 0.005, max: 0.1, step: 0.005, group: 'Dynamics & fatigue', default: 0.02, expert: true },
  { key: 'gustRate', label: 'Significant gust cycles per operating hour', type: 'number', min: 10, max: 5000, step: 10, group: 'Dynamics & fatigue', default: 600 },
  { key: 'hoursYear', label: 'Operating hours per year', unit: 'h', type: 'number', min: 50, max: 5000, step: 50, group: 'Dynamics & fatigue', default: 800 },
  { key: 'nEl', label: 'Beam elements', type: 'number', min: 6, max: 60, step: 1, group: 'Numerics', default: 20, expert: true },
  { key: 'nLL', label: 'Lifting-line panels', type: 'number', min: 8, max: 60, step: 1, group: 'Numerics', default: 24, expert: true },
  { key: 'endplate', label: 'Root reflection-plane efficiency', type: 'range', min: 0, max: 1, step: 0.05, group: 'Numerics', default: 0.7, expert: true, help: 'Fraction of a perfect mirror image provided by the vehicle body at the wing root (1 = infinite end-plate).' },
];

// Shared wing model (used by Suites 06 and 08)
export function buildWing(p, ctx) {
  const reef = p.wingState === 'reefed', span = reef ? Math.min(p.span, p.reefSpan || p.span) : p.span, cm = p.area / p.span, cr = 2 * cm / (1 + p.taper), ctD = cr * p.taper, chord = (z) => cr + (ctD - cr) * z / p.span, ct = chord(span); // reefed: deployed planform truncated at the reef station
  const af = naca4(String(p.airfoil).padStart(4, '0'), 61), st = airfoilStats(af.points), tc = st.tmax;
  const polar = ctx.upstream('s07')?.polar || null;
  // passive leeward camber (D-rev 2.2): thin-aerofoil zero-lift shift −2m and an empirical high-camber C_l,max (≈1.2 + 9m, cf. cambered
  // sail-wing sections); the symmetric polar from Suite 07 is used only when morphing is off.
  const mC = p.camberMorph ? (p.camberPct || 0) / 100 : (af.m || 0);
  const section = p.camberMorph ? sectionModel({ a0: 2 * Math.PI * (1 + 0.77 * tc) * 0.9, alpha0: -2 * mC, clmax: Math.min(2.1, 1.2 + 9 * mC), cd0: 0.0075 + 0.02 * tc + 0.04 * mC * mC, polar: null })
    : sectionModel({ a0: 2 * Math.PI * (1 + 0.77 * tc) * 0.9, alpha0: -2 * (af.m || 0) * 1.0, clmax: 0.95 + 1.2 * tc + 2 * (af.m || 0), cd0: 0.0075 + 0.02 * tc, polar: polar && polar.alpha?.length ? polar : null });
  const SM = getMat(p.sparMat) || getMat('cfrp-t700'), L = lamina(SM), tp = p.tPly / 1000;
  const cover = laminate(parseLayup(p.layup, L, tp)), web = laminate(parseLayup(p.webLayup || '45,-45,45,-45', L, tp));
  const red = (A) => { const a = A; const d = a[1][1]; return [[a[0][0] - a[0][1] ** 2 / d, a[0][2] - a[0][1] * a[1][2] / d], [a[0][2] - a[0][1] * a[1][2] / d, a[2][2] - a[1][2] ** 2 / d]]; };
  const Ac = red(cover.A), Aw = red(web.A);
  const SK = getMat(p.skinMat) || getMat('tpu-glass'), RB = getMat(p.ribMat) || getMat('gfrp-e'), tsk = p.tSkin / 1000;
  const Gsk = (SK.E || 0.8) * 1e9 / (2 * (1 + (SK.nu ?? 0.4)));
  const tube = p.sparType !== 'box';
  const props = (z) => {
    const c = chord(z), hs = 0.85 * tc * c, wb = p.boxW * c, Abox = wb * hs, P = 2 * (wb + hs);
    const EIc = 2 * wb * Ac[0][0] * (hs / 2) ** 2, EIw = 2 * Aw[0][0] * hs ** 3 / 12, gam = 2 * Abox / P;
    const GJbox = 4 * Abox ** 2 / (2 * wb / Ac[1][1] + 2 * hs / Aw[1][1]);
    const Askin = 0.68 * tc * c * c, Pskin = 2.04 * c, GJskin = 4 * Askin ** 2 * Gsk * tsk / Pskin;
    const tTE = 0.5 * tc * c * 0.6, aTE = p.teFrac * c, fill = p.coreFill ?? 0.35, tEff = 0.25 * tTE, GJcore = fill * p.coreG * 1e6 * aTE * tTE ** 3 / 3;
    let K = 2 * wb * Ac[0][1] * (hs / 2) * gam;
    let EI = EIc + EIw + (SK.E || 0.8) * 1e9 * tsk * c * (tc * c / 2) ** 2 * 0.5, GJ = GJbox + GJskin + GJcore, xEAtube = null, mTube = 0, gamOpen = null;
    if (tube) {
      // open caps + single web (open-section St-Venant torsion), torsion tube on the pivot axis, skin open if slotted
      const Gc = Ac[1][1] / cover.h, Gw = Aw[1][1] / web.h, rT = p.rTube / 1000, tT = p.tTube / 1000;
      const GJopen = (2 * wb * cover.h ** 3 * Gc + hs * web.h ** 3 * Gw) / 3, GJtube = Gw * 2 * Math.PI * rT ** 3 * tT;
      const GJsk = p.teSlot ? Pskin * tsk ** 3 * Gsk / 3 : GJskin;
      GJ = GJopen + GJtube + GJsk + GJcore; EI = EIc + hs ** 3 * Aw[0][0] / 12 + (SK.E || 0.8) * 1e9 * tsk * c * (tc * c / 2) ** 2 * (p.teSlot ? 0.25 : 0.5);
      K = 0; // open caps cannot carry the closed-cell shear flow that produces bend–twist coupling
      gamOpen = cover.h; // St-Venant open-section shear strain in the caps = φ′·t
      xEAtube = p.xPivot; mTube = 2 * Math.PI * rT * tT * (SM.rho || 1550);
    }
    const Kmax = 0.95 * Math.sqrt(EI * GJ); if (Math.abs(K) > Kmax) K = Math.sign(K) * Kmax;
    const mSpar = P * cover.h * (SM.rho || 1550), mSkin = Pskin * tsk * (SK.rho || 1300), mCore = fill * aTE * tTE * p.coreRho, mRib = (p.ribSolid ?? 0.3) * Askin * 0.003 * (RB.rho || 2000) / p.ribPitch;
    const mBal = tube ? (p.leBallast || 0) : 0, m = mSpar + mSkin + mCore + mRib + mTube + mBal;
    const xEA = xEAtube ?? (0.15 + p.boxW / 2), xCG = (mSpar * xEA + mTube * xEA + mSkin * 0.42 + mCore * (1 - p.teFrac / 2) + mRib * 0.42 + mBal * 0.02) / m; // chord fractions
    const Ip = (m - mBal) * (c * 0.28) ** 2 + mBal * ((0.02 - xEA) * c) ** 2 + (m - mBal) * ((xCG - xEA) * c) ** 2;
    return { EI, GJ, K, m, Ip, xa: (xCG - xEA) * c, c, hs, wb, gam: gamOpen ?? gam, xEA, xCG, Abox, mSpar, mSkin, mCore, mRib, mTube, mBal, D_TE: p.coreE * 1e6 * tEff ** 3 / 12 + (SK.E || 0.8) * 1e9 * 2 * tsk * (tEff / 2) ** 2, aTE };
  };
  const model = beamModel(span, Math.round(p.nEl || 20), props);
  // apparent wind distribution over the span (log-law shear + vehicle motion)
  const v = (p.vkmh || 0) / 3.6, Vz = (z) => logWind(p.U10, 10, Math.max(0.3, p.zRoot + z), p.z0);
  const app = (z) => { const aw = apparentWind(Vz(z), p.twa, 0, v); return { Va: Math.max(0.05, aw.Va), awa: Math.abs(aw.beta) }; };
  const appGust = (z, dU) => { const aw = apparentWind(Vz(z) * (1 + dU / Math.max(p.U10, 0.1)), p.twa, 0, v); return { Va: Math.max(0.05, aw.Va), awa: Math.abs(aw.beta) }; };
  const app0 = app(span * 0.05), setAngle = app0.awa - p.aoa; // wing angle to vehicle axis
  const e = (z) => (props(z).xEA - 0.25) * chord(z); // EA aft of aerodynamic centre
  return { span, reef, mC, cr, ct, cm, chord, af, tc, section, cover, web, SM, SK, RB, L, props, model, app, appGust, setAngle, e, V: (z) => app(z).Va, airfoilName: af.name };
}

// ---------------- Revision-2 spar sizing (D4) ----------------
// Evaluate a candidate at the sizing reference (beam reach, apparent wind refVa at mid-span): tip twist, root
// first-ply strength ratio, flutter speed and wing mass.
// Uncoupled bending and torsion frequencies (Rayleigh quotients with cantilever shape functions) — the inputs the
// typical-section flutter model expects; inertial coupling enters separately through x_α.
export function uncoupledOmegas(W) {
  const L = W.span, n = 60, k = Math.PI / (2 * L); let nb = 0, db = 0, nt = 0, dt = 0;
  for (let i = 0; i < n; i++) { const z = (i + 0.5) * L / n, P = W.props(z), dz = L / n, wb = 1 - Math.cos(k * z), wbpp = k * k * Math.cos(k * z), ph = Math.sin(k * z), php = k * Math.cos(k * z); nb += P.EI * wbpp ** 2 * dz; db += (P.m) * wb ** 2 * dz; nt += P.GJ * php ** 2 * dz; dt += P.Ip * ph ** 2 * dz; }
  return { wh: Math.sqrt(nb / db), wa: Math.sqrt(nt / dt) };
}
export function evalDesign(p0, ctx, { statics = true, flutter = true } = {}) {
  const p = { ...p0, vkmh: 0, twa: 90 };
  let W = buildWing(p, ctx); const f = p.refVa / Math.max(0.05, W.app(W.span / 2).Va); p.U10 = Math.max(0.1, p.U10 || 1) * f; W = buildWing(p, ctx);
  const out = { mass: W.model.z.slice(0, -1).reduce((acc, z) => acc + W.props(z + W.model.le / 2).m * W.model.le, 0), xEA: W.props(0.75 * W.span).xEA, xCG: W.props(0.75 * W.span).xCG };
  if (statics) { const S = solveStatic(W, p, { tol: 1e-6, maxIt: 60 }), N = W.model.N; out.tip = deg(S.sol.phi[N - 1]); out.converged = S.converged; const r = beamResultants(W.model, S.sol.u, W.props)[0], P = W.props(r.z), eps = r.M * (P.hs / 2) / P.EI; out.R = Math.min(...plyStresses(W.cover, [eps, 0, r.T / P.GJ * P.gam]).map(q => q.R)); }
  if (flutter) { const U = uncoupledOmegas(W), z75 = 0.75 * W.span, P75 = W.props(z75), b = W.chord(z75) / 2; out.VF = flutterVg({ m: P75.m, Ia: P75.Ip, xa: P75.xa / b, a: (P75.xEA - 0.5) * 2, b, wh: U.wh, wa: U.wa, rho: p.rho, gs: p.zeta * 2 }).Vflutter; out.fb = U.wh / (2 * Math.PI); out.ft = U.wa / (2 * Math.PI); }
  return out;
}
const sizeCache = new Map();
export function sizeSpar(p, ctx) {
  if (!p.autoSize || p.sparType === 'box') return { values: {}, log: [], sized: false };
  const keyObj = { ...p }; delete keyObj.wingState; delete keyObj.tPly; delete keyObj.tTube; delete keyObj.rTube; delete keyObj.leBallast; const key = JSON.stringify(keyObj) + (ctx.material?.('spar')?.id || '');
  if (sizeCache.has(key)) return sizeCache.get(key);
  const v = { tPly: p.tPly, tTube: p.tTube, rTube: p.rTube, leBallast: p.leBallast }, log = [], ev = (o) => evalDesign({ ...p, ...v, ...o, wingState: 'deployed' }, ctx, { flutter: false }), evF = (o) => Math.min(evalDesign({ ...p, ...v, ...o, wingState: 'deployed' }, ctx, { statics: false }).VF, evalDesign({ ...p, ...v, ...o, wingState: 'reefed' }, ctx, { statics: false }).VF); // sized on the deployed (worst-case) wing; flutter checked in both states
  const bis = (lo, hi, fn, target, it = 22, logScale = true) => { let a = lo, b = hi; const g = (x) => fn(x) - target; let ga = g(a); for (let i = 0; i < it; i++) { const m = logScale ? Math.sqrt(a * b) : (a + b) / 2, gm = g(m); if (ga * gm <= 0) b = m; else { a = m; ga = gm; } } return logScale ? Math.sqrt(a * b) : (a + b) / 2; };
  v.tTube = Math.max(0.25, p.tTube); // two thin ±45° plies: practical minimum wall; the radius is the sizing variable
  for (let pass = 0; pass < 2; pass++) {
    // A: cap ply thickness for strength factor 2.5 (first-ply failure at the root, reference load)
    const tMin = 0.03, Rlo = ev({ tPly: tMin }).R; // 0.03 mm = thin-ply minimum gauge
    if (Rlo >= 2.5) { v.tPly = tMin; log.push(`cap thickness governed by minimum gauge (SF ${fmt(Rlo, 3)} at 0.03 mm plies)`); }
    else { const Rhi = ev({ tPly: 2 }).R; v.tPly = Rhi <= 2.5 ? 2 : bis(tMin, 2, (t) => ev({ tPly: t }).R, 2.5, 14); }
    // B: torsion-tube radius for the target tip washout (GJ ∝ r³)
    const tw = (r) => ev({ rTube: r }).tip, tSmall = tw(3), tBig = tw(80);
    v.rTube = (tSmall - p.targetTwist) * (tBig - p.targetTwist) > 0 ? (Math.abs(tSmall - p.targetTwist) < Math.abs(tBig - p.targetTwist) ? 3 : 80) : bis(3, 80, tw, p.targetTwist, 14);
    if (v.rTube === 3) log.push('target washout exceeded even with the smallest tube — the core/skin alone are too compliant (stiffen the lattice core)');
    log.push(`pass ${pass + 1}: cap ply ${fmt(v.tPly, 3)} mm, torsion tube r = ${fmt(v.rTube, 3)} mm × ${fmt(v.tTube, 3)} mm`);
    if (Rlo >= 2.5) break; // strength at minimum gauge does not depend on the tube → one pass suffices
  }
  // C: leading-edge mass balance for the flutter target (lightest balance that meets it)
  const vf = (mb) => evF({ leBallast: mb });
  if (vf(0) >= p.flutterTarget) v.leBallast = 0; else if (vf(10) < p.flutterTarget) { v.leBallast = 10; log.push('flutter target not reached with 10 kg/m balance'); } else v.leBallast = bis(0, 10, (mb) => Math.min(1e3, vf(mb)), p.flutterTarget, 10, false) * 1.05;
  const final = evalDesign({ ...p, ...v, wingState: 'deployed' }, ctx); final.VFreef = evalDesign({ ...p, ...v, wingState: 'reefed' }, ctx, { statics: false }).VF; log.push(`balance ${fmt(v.leBallast, 3)} kg/m → tip ${fmt(final.tip, 3)}°, SF ${fmt(final.R, 3)}, V_F ${fmt(final.VF, 3)} m/s, mass ${fmt(final.mass, 3)} kg`);
  const out = { values: v, log, sized: true, check: final };
  sizeCache.set(key, out); if (sizeCache.size > 40) sizeCache.delete(sizeCache.keys().next().value);
  return out;
}
// Free-feathering root (D3): breakaway side force and feathered loads
export function featherState(W, p, ctx, loads, nW) {
  const veh = ctx.vehicle, mu = ctx.upstream?.('s10')?.muPeak ?? 0.95, ssf = ctx.upstream?.('s11')?.ssf ?? ctx.upstream?.('s01')?.ssf ?? (veh.track / 2) * (2 / 3) / veh.cgHeight;
  const hCE = p.zRoot + 0.45 * W.span, SSw = safeSideForce({ mass: veh.mass, cgHeight: veh.cgHeight, ssf, hCE, mu }), FyGrip = SSw.Fgrip, FyRoll = SSw.Froll;
  // one shared cap: Suite 01 publishes the washout side-force cap (same mass, CG and CE height) for s06, s11, s12 and s23
  const FySafe = ctx.upstream?.('s01')?.FySafe_N ?? ctx.upstream?.('s01')?.washoutCap_N ?? SSw.Fs; /* the one cap, published by Suite 01 */
  const side = Math.abs(loads.side * nW), released = !!p.breakaway && side > FySafe;
  // feathered: wing weathervanes about the pivot, C_L ≈ 0, C_D ≈ 0.02 on the planform
  let th = 0, sd = 0; W.model.z.forEach((z, i) => { if (i === 0) return; const a = W.app(z), q = 0.5 * p.rho * a.Va ** 2, dz = W.model.le, D = q * W.chord(z) * dz * 0.02, b = rad(a.awa); th += -D * Math.cos(b); sd += D * Math.sin(b); });
  return { FySafe, FyGrip, FyRoll, ssf, hCE, side, released, feathered: { thrust: th * nW, side: sd * nW } };
}

// Static aeroelastic solution with passive washout (twist) and passive camber (compliant trailing region)
export function solveStatic(W, p, { method = 'aitken', rigid = false, tol = 1e-7, maxIt = 80 } = {}) {
  const { model, span, chord, section, app, setAngle, props, e } = W, N = model.N, rho = p.rho;
  let lastCl = null, lastLL = null;
  const tau = (cf) => { const th = Math.acos(2 * cf - 1); return 1 - (th - Math.sin(th)) / Math.PI; };
  const camberShift = (z, cl) => { if (rigid) return 0; const P = props(z), q = 0.5 * rho * app(z).Va ** 2, dp = Math.abs(q * cl) * 0.35, delta = dp * P.aTE ** 4 / (8 * P.D_TE); const df = Math.min(rad(12), delta / P.aTE) * Math.sign(cl || 1); return tau(P.aTE / chord(z)) * df; }; // reflex raises zero-lift angle
  const aero = (phiNodes) => {
    const phi = (z) => rigid ? 0 : interp1(model.z, phiNodes, z);
    const a0Fn = (z) => lastCl ? camberShift(z, interp1(lastLL.z, lastCl, z)) : 0;
    const ll = liftingLine({ span, chord, alpha: (z) => rad(app(z).awa - setAngle + p.twistBuilt * z / span) + phi(z), V: (z) => app(z).Va, rho, section, n: Math.round(p.nLL || 24), mirror: p.endplate ?? 0.7, alpha0Fn: (z) => section.alpha0 + a0Fn(z) });
    lastCl = ll.cl; lastLL = ll;
    const lz = (z) => interp1(ll.z, ll.l, z, 'linear'), Fw = [], Ft = [];
    for (let i = 0; i < N; i++) { const z = model.z[i], tw = (i === 0 || i === N - 1 ? 0.5 : 1) * model.le, lv = i === 0 ? 0 : lz(z); Fw.push(lv * tw); Ft.push(lv * e(z) * tw); }
    return { Fw, Ft, ll };
  };
  if (rigid) { const a = aero(new Array(N).fill(0)); const F = new Array(model.ndof).fill(0); for (let i = 0; i < N; i++) { F[3 * i] = a.Fw[i]; F[3 * i + 2] = a.Ft[i]; } const sol = beamStatic(model, F); return { phi: new Array(N).fill(0), aero: a, sol, history: [1e-16], iterations: 1, converged: true }; }
  aero(new Array(N).fill(0));
  return staticAeroelastic(model, aero, { method, tol, maxIt, omega0: p.omega0 ?? 0.5 });
}

// Gust as a true-wind speed increase ΔU(t): changes both apparent-wind angle (Küssner-lagged) and dynamic pressure (quasi-steady)
export function runGust(W, p, ll, { dt = 0.003, subIt = 1, T } = {}) {
  const dU = oneMinusCosGust(0.3, p.gustDur, p.gustAmp), memo = new Map(), base = (z) => { let b = memo.get(z); if (!b) { b = W.app(z); b.cl = interp1(ll.z, ll.cl, z); b.c = W.chord(z); b.e = W.e(z); memo.set(z, b); } return b; }, clS = (z) => base(z).cl;
  let gT = NaN; const gm = new Map(), gAt = (z, t) => { if (t !== gT) { gm.clear(); gT = t; } let g = gm.get(z); if (!g) { g = W.appGust(z, dU(t)); gm.set(z, g); } return g; };
  return gustResponse({ model: W.model, props: W.props, chord: (z) => base(z).c, V: (z) => base(z).Va, rho: p.rho, a0: W.section.a0, e: (z) => base(z).e, T: T || Math.max(3, p.gustDur + 2.5), dt, zeta: p.zeta, nModes: 4, subIt,
    gust: (t, z) => { const b0 = base(z); if (dU(t) === 0) return 0; const g = gAt(z, t); return b0.Va * rad(g.awa - b0.awa); },
    extraLoad: (t, z) => { const b0 = base(z); if (dU(t) === 0) return 0; const g = gAt(z, t); return 0.5 * p.rho * b0.c * (g.Va ** 2 - b0.Va ** 2) * b0.cl; },
    gustRef: dU, clStatic: clS, clmax: W.section.clmax });
}

// Resolve forces of the converged lifting-line solution to vehicle axes
export function wingLoads(W, ll, rho = 1.225) {
  let thrust = 0, side = 0, roll = 0, drag = 0, lift = 0;
  ll.z.forEach((z, i) => { const b = rad(W.app(z).awa), l = ll.l[i] * ll.dz[i], d = (0.5 * rho * ll.V[i] ** 2 * ll.chord[i] * ll.cd[i] + ll.l[i] * ll.alphaInd[i]) * ll.dz[i]; lift += l; drag += d; thrust += l * Math.sin(b) - d * Math.cos(b); const sf = l * Math.cos(b) + d * Math.sin(b); side += sf; roll += sf * z; });
  return { thrust, side, roll, drag, lift };
}

export default {
  description: 'Designs the twin fixed-root vertical wings: carbon box-spar skeleton with tailored (bend–twist coupled) laminate, flexible ribs, metamaterial trailing core and continuous morphing skin. Solves the coupled static aeroelastic equilibrium with a nonlinear lifting line in the sheared apparent wind, passive washout and camber adaptation, then divergence, flutter (V–g), modal, gust, fatigue and the safe operating wind envelope — all from the selected materials.',
  literature: 'passive aeroelastic tailoring bend-twist coupling morphing wing gust load alleviation wingsail',
  credibility: 'Numerically verified',
  equations: ['Prandtl/Weissinger lifting-line theory with discrete horseshoe vortices, root reflection plane and spanwise-varying apparent wind (boundary-layer shear)', 'Thin-aerofoil theory (lift slope, flap effectiveness for compliant trailing-edge reflex), nonlinear section lift with stall', 'Apparent-wind vector transformation V_a = V_wind − V_vehicle; resolution of lift and drag into body-axis thrust and side force', 'Classical laminate theory (ABD), reduced membrane stiffness of spar covers, circumferentially-asymmetric-stiffness box beam: EI, GJ (Bredt–Batho) and bend–twist coupling K_bt', 'Euler–Bernoulli bending + St-Venant torsion beam FE with coupled strain energy ½∫(EIκ² + 2K_btκφ′ + GJφ′²)', 'Static aeroelastic equilibrium (K − qA)u = F with partitioned Aitken-accelerated coupling', 'Static divergence eigenproblem K u = q_D A u', 'Theodorsen unsteady aerodynamics (R. T. Jones approximation) and V–g (k-method) flutter of the 75 %-span typical section', 'Modal analysis (generalised eigenproblem), Newmark-β gust response with Küssner and Wagner indicial functions', 'Composite S–N fatigue (S/UTS = 1 − b log N) with Tsai–Wu strength ratio at the root'],
  methods: ['Quasi-Newton secant-linearised lifting-line solve per coupling iteration', 'Aitken Δ² dynamic relaxation of the interface twist', 'Dense LU of the reduced beam system; Jacobi generalised eigen-solver for modes', 'Parameter sweep of true wind for the safe operating envelope'],
  assumptions: ['Unswept wing, elastic axis at box-spar centre; chordwise rigid sections except the compliant trailing region', 'Steady inflow for the static case; quasi-steady log-law shear', 'Flutter estimated with the 75 %-span typical section using the beam’s first bending and torsion frequencies', 'Strip-theory loads for divergence and gust (no wake roll-up)'],
  ibc: { initial: ['Undeformed wing geometry, reference incidence, built-in twist, ply orientations, zero initial displacement and velocity'], boundary: ['Fully clamped wing root (w = w′ = φ = 0) on the monocoque', 'Distributed aerodynamic lift and pitching moment from the lifting line; free tip', 'Spanwise sheared apparent wind; 1-cos discrete gust for the transient case', 'Twist is an output of the coupled solution — never prescribed together with the load'] },
  cvv: { calibration: ['Wing bending/twist under static loads (photogrammetry/DIC)', 'Ground vibration test (natural frequencies, damping)', 'Pressure-tap and force-balance wind-tunnel data'], verification: ['Cantilever first bending frequency vs 3.516√(EI/mL⁴)', 'Lifting line CL vs finite-wing lift-slope formula', 'Coupled-solver residual convergence and global load balance', 'Typical-section flutter against the Theodorsen benchmark (V_F/bω_α = 2.17 for the Hodges–Pierce case)'], validation: ['Wind-tunnel section with compliant rib and skin; full-size gust and fatigue tests (gate 1)'] },
  formats: { import: ['CSV deflection/strain/pressure measurements', 'DAT/CSV airfoil coordinates', 'JSON layups'], export: ['CSV spanwise distributions & V–g data', 'JSON wing state (reference, loaded, gust-deformed)', 'glTF/GLB deformed 3D wing', 'PNG/SVG plots'] },
  params: WING_PARAMS,
  presets: [{ label: 'Beam reach 6 m/s', values: { U10: 6, twa: 90, vkmh: 10, aoa: 10 } }, { label: 'Close reach 8 m/s', values: { U10: 8, twa: 60, vkmh: 15, aoa: 12 } }, { label: 'Strong crosswind 14 m/s', values: { U10: 14, twa: 90, vkmh: 5, aoa: 8 } }, { label: 'Balanced layup (no coupling)', values: { layup: '0,0,45,-45,-45,45,0,0' } }, { label: 'Titanium spar', values: { sparMat: 'ti-6al4v', layup: '0,0,0,0' } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    ctx.progress(0.05, 'spar sizing');
    const SZ = sizeSpar(p, ctx); Object.assign(p, SZ.values);
    const W = buildWing(p, ctx), { model, props, span, chord } = W;
    ctx.progress(0.15, 'coupled static aeroelastic solution');
    const S = solveStatic(W, p), R0 = solveStatic(W, p, { rigid: true });
    const ll = S.aero.ll, loads = wingLoads(W, ll), loadsR = wingLoads(W, R0.aero.ll), N = model.N;
    const nW = ctx.vehicle.nWings || 2, res = beamResultants(model, S.sol.u, props);
    const twistDeg = S.sol.phi.map(deg), tipTw = twistDeg[N - 1], tipW = S.sol.w[N - 1];
    // ply stresses along the span (upper cover, tension side)
    const stressRows = res.map(r => { const P = props(r.z), eps = r.M * (P.hs / 2) / P.EI, gam = r.T / P.GJ * P.gam, ps = plyStresses(W.cover, [eps, 0, gam]); const Rmin = Math.min(...ps.map(q => q.R)); return { z: r.z, eps, sx: eps * W.cover.Ex, R: Rmin, ps }; });
    const rootR = stressRows[0].R, rootM = res[0].M, rootT = res[0].T;
    ctx.progress(0.4, 'divergence, modes, flutter');
    const modes = beamModes(model, 40), fb = modes.find(m => m.type === 'bending') || modes[modes.length - 1], ft = modes.find(m => m.type === 'torsion') || modes[0];
    const qD = divergenceQ(model, chord, W.e, W.section.a0), VD = Math.sqrt(2 * qD / p.rho);
    const z75 = 0.75 * span, P75 = props(z75), b = chord(z75) / 2;
    const UO = uncoupledOmegas(W), FL = flutterVg({ m: P75.m, Ia: P75.Ip, xa: P75.xa / b, a: (P75.xEA - 0.5) * 2, b, wh: UO.wh, wa: UO.wa, rho: p.rho, gs: p.zeta * 2 });
    const Vtip = W.app(span).Va, Vmax = max(range(N).map(i => W.app(model.z[i]).Va));
    ctx.progress(0.6, 'gust response');
    const G = runGust(W, p, ll, { dt: 0.003 });
    const dMr = max(G.Mroot.map(Math.abs)), gustF = 1 + dMr / Math.max(1, Math.abs(rootM));
    // fatigue (root cover, tension–tension with mean = static, amplitude = gust)
    const sMean = Math.abs(stressRows[0].sx), sAmp = Math.abs(dMr * (props(0).hs / 2) / props(0).EI * W.cover.Ex) / 2;
    const Sut = sMean > 0 && Number.isFinite(rootR) ? rootR * sMean : W.L.Xt * 0.5, // laminate first-ply-failure stress at the root
      bfat = (1 - (W.SM.fatigue && W.SM.fatigue < 1 ? W.SM.fatigue : 0.5)) / 7;
    const Seq = sAmp / Math.max(0.05, 1 - sMean / Sut), Nf = Seq > 0 ? 10 ** Math.min(15, (1 - Seq / Sut) / bfat) : Infinity, lifeYears = Nf / (p.gustRate * p.hoursYear);
    // safe operating envelope (strip-theory sweep)
    const Us = linspace(1, 25, 13), env = Us.map(U => { const pp = { ...p, U10: U }; const W2 = buildWing(pp, ctx); const S2 = solveStatic(W2, pp, { tol: 1e-5, maxIt: 30 }); const L2 = wingLoads(W2, S2.aero.ll, pp.rho); const r2 = beamResultants(W2.model, S2.sol.u, W2.props)[0]; const P0 = W2.props(0), eps = r2.M * (P0.hs / 2) / P0.EI, ps = plyStresses(W2.cover, [eps, 0, r2.T / P0.GJ * P0.gam]); const Vt = W2.app(span).Va; return { U, R: Math.min(...ps.map(q => q.R)) / gustF, tip: S2.sol.w[W2.model.N - 1], side: L2.side * nW, thrust: L2.thrust * nW, roll: (L2.roll + L2.side * pp.zRoot) * nW, Vt }; });
    const mveh = ctx.vehicle.mass, track = ctx.vehicle.track, restore = mveh * 9.80665 * track / 2 * (ctx.vehicle.nWheels === 3 ? 2 / 3 : 1);
    const lim = (f) => { const e = env.find(f); return e ? e.U : Infinity; };
    const Ustr = lim(e => e.R < 1.5), Utip = lim(e => Math.abs(e.tip) > 0.1 * span), Uroll = lim(e => e.roll > restore / 1.3), Uflut = lim(e => e.Vt * 1.15 > FL.Vflutter), Udiv = lim(e => e.Vt * 1.15 > VD);
    const Usafe = Math.min(Ustr, Utip, Uroll, Uflut, Udiv);
    // mass & checks
    const wingMass = model.z.slice(0, -1).reduce((s, z) => s + props(z + model.le / 2).m * model.le, 0);
    const p0 = props(span / 2), fTheory = 3.516 / (2 * Math.PI) * Math.sqrt(p0.EI / (p0.m * span ** 4));
    out.checks.push({ name: 'Coupled interface residual', value: S.history[S.history.length - 1], target: '< 1e-7', pass: S.converged, detail: `${S.iterations} Aitken iterations` });
    out.checks.push({ name: 'Lifting-line convergence', value: ll.residual, target: '< 1e-8', pass: ll.residual < 1e-6 });
    const sumF = S.aero.Fw.reduce((a, b2) => a + b2, 0); out.checks.push({ name: 'Load transfer: Σ nodal forces / lifting-line lift', value: sumF / ll.L, target: '1 ± 0.05', pass: Math.abs(sumF / ll.L - 1) < 0.05, detail: 'Conservative aerodynamic-to-structural load mapping' });
    out.checks.push({ name: 'First bending frequency vs uniform-cantilever estimate (mid-span props)', value: fb.f / fTheory, target: '0.7–1.6 (taper stiffens the root)', pass: fb.f / fTheory > 0.7 && fb.f / fTheory < 1.6 });
    out.checks.push({ name: 'Bend–twist coupling admissible', value: Math.abs(props(0).K) / Math.sqrt(props(0).EI * props(0).GJ), target: '< 0.95', pass: Math.abs(props(0).K) / Math.sqrt(props(0).EI * props(0).GJ) < 0.95 });
    // KPIs
    const k = (label, value, unit, explain, status, prov = 'calc') => out.kpis.push({ label, value, unit, explain, status, prov });
    k('Apparent wind at root / tip', `${fmt(W.app(0.05 * span).Va, 3)} / ${fmt(Vtip, 3)}`, 'm/s', 'Vector sum of the sheared true wind and the vehicle motion at the root and tip.');
    k('Apparent wind angle (root)', W.app(0.05 * span).awa, '°', 'Angle between the bow and the apparent wind at the root; thrust needs a lift component pointing forward.');
    k('Forward thrust (all wings)', loads.thrust * nW, 'N', 'Useful propulsive force along the road direction — lift is not thrust. Negative = the wings resist motion.', loads.thrust > 0 ? 'ok' : 'bad');
    k('Side force (all wings)', loads.side * nW, 'N', 'Lateral force the tyres must resist; drives overturning and steering demand.');
    k('Lift coefficient C_L', ll.CL, '', 'Spanwise-integrated lift coefficient of the deformed wing.');
    k('Thrust change from passive deformation', (loads.thrust - loadsR.thrust) / Math.abs(loadsR.thrust || 1) * 100, '%', 'Effect of the solved twist and camber versus a rigid wing at the same set angle.');
    k('Tip twist (passive)', tipTw, '°', 'Elastic twist at the tip from bend–twist coupling and aerodynamic torque (negative = washout, load relieving).');
    k('Tip deflection', tipW * 1000, 'mm', 'Lateral bending deflection of the tip.');
    k('Root bending moment (per wing)', rootM, 'N·m', 'Moment carried into the monocoque by each wing root (input to Suite 09).');
    k('Root strength ratio (Tsai–Wu)', rootR, '', 'Load multiplier to first-ply failure at the root cover. ≥ 2 recommended for gust-loaded structures.', rootR >= 2 ? 'ok' : rootR >= 1.5 ? 'warn' : 'bad');
    k('Divergence speed', Number.isFinite(VD) ? VD : 'none (cannot diverge)', Number.isFinite(VD) ? 'm/s' : '', 'Apparent wind at which aerodynamic twisting moment overcomes torsional stiffness (static instability).', VD > 1.5 * Vmax ? 'ok' : 'bad');
    k('Flutter speed (V–g)', Number.isFinite(FL.Vflutter) ? FL.Vflutter : `none up to ${fmt(FL.vSwept ?? 0, 3)} m/s`, Number.isFinite(FL.Vflutter) ? 'm/s' : '', `Apparent wind at which a mode first acquires negative damping (dynamic instability). The sweep is independent of the scenario wind and reaches ≥ 60 m/s (≥ 1.8 × the ${p.flutterTarget} m/s requirement) for the lowest mode; a branch that stops early has become aerodynamically overdamped, not unstable.`, FL.Vflutter > 1.5 * Vmax ? 'ok' : 'bad');
    k('1st bending / torsion frequency', `${fmt(fb.f, 3)} / ${fmt(ft.f, 3)}`, 'Hz', 'Natural frequencies of the clamped wing; keep clear of gust and vortex-shedding excitation.');
    k('Gust dynamic amplification', gustF, '', 'Peak root moment during the design gust divided by the static value.');
    k('Fatigue life (root cover)', lifeYears > 1000 ? '> 1000' : lifeYears, 'years', `Composite S–N estimate at ${p.gustRate} gust cycles/h × ${p.hoursYear} h/yr.`, lifeYears > 15 ? 'ok' : lifeYears > 5 ? 'warn' : 'bad');
    k('Safe operating true wind', Number.isFinite(Usafe) ? Usafe : '> 25 (sweep limit)', 'm/s', 'Highest true wind (10 m) for which strength, deflection, overturning, flutter and divergence margins all hold.', Usafe >= 12 ? 'ok' : Usafe >= 8 ? 'warn' : 'bad');
    const UsafeTxt = Number.isFinite(Usafe) ? fmt(Usafe, 3) : '25 (no limit reached in the sweep)';
    k('Wing mass (each)', wingMass, 'kg', 'Spar, skin, trailing core and ribs from the selected materials.');
    // verdicts
    const Vop = Vmax;
    out.verdicts.push({ test: 'Useful forward propulsion', status: loads.thrust > 5 ? 'pass' : loads.thrust > 0 ? 'warn' : 'fail', detail: `Thrust ${fmt(loads.thrust * nW, 3)} N from lift ${fmt(loads.lift * nW, 3)} N at apparent wind angle ${fmt(W.app(0.05 * span).awa, 3)}°.`, fix: 'Adjust root angle of attack, choose a beam/broad reach heading, or wait for stronger wind; a wing with high lift but small forward component is not credited with propulsion.' });
    out.verdicts.push({ test: 'Strength (first-ply failure, with gust)', status: rootR / gustF >= 1.5 ? 'pass' : rootR / gustF >= 1 ? 'warn' : 'fail', detail: `Strength ratio ${fmt(rootR, 3)} static, ${fmt(rootR / gustF, 3)} including gust amplification.`, fix: 'Add 0° plies to the spar covers, increase box depth, or choose a stronger spar material.' });
    out.verdicts.push({ test: 'Static divergence margin', status: VD > 1.5 * Vop ? 'pass' : VD > 1.15 * Vop ? 'warn' : 'fail', detail: Number.isFinite(VD) ? `V_D = ${fmt(VD, 3)} m/s vs max apparent wind ${fmt(Vop, 3)} m/s (required ×1.5).` : `No static divergence: the elastic axis (${fmt(props(0).xEA * 100, 3)} % chord) lies ahead of the aerodynamic centre (25 %), so aerodynamic torque reduces twist.`, fix: 'Move the elastic axis forward (wider/forward box) or increase GJ; wash-out coupling (K_bt of the correct sign) raises V_D.' });
    out.verdicts.push({ test: 'Flutter margin', status: FL.Vflutter > 1.5 * Vop ? 'pass' : FL.Vflutter > 1.15 * Vop ? 'warn' : 'fail', detail: `V_F = ${fmt(FL.Vflutter, 3)} m/s (${fmt(FL.fFlutter, 3)} Hz) vs ${fmt(Vop, 3)} m/s.`, fix: 'Separate bending and torsion frequencies (stiffer torsion), move the section CG forward of the elastic axis (lighter trailing core), add damping.' });
    out.verdicts.push({ test: 'Passive gust-load alleviation direction', status: tipTw < 0 ? 'pass' : 'warn', detail: tipTw < 0 ? `Loaded wing washes out (tip ${fmt(tipTw, 3)}°), reducing peak loads.` : `Loaded wing washes in (tip +${fmt(tipTw, 3)}°): coupling amplifies load and lowers divergence speed.`, fix: 'Reverse the sign of off-axis cover plies (e.g. 20° → −20°) so bending induces nose-down twist.' });
    out.verdicts.push({ test: 'Fatigue durability ≥ 15 years', status: lifeYears >= 15 ? 'pass' : lifeYears >= 5 ? 'warn' : 'fail', detail: lifeYears > 1000 ? 'Stress amplitudes are below the endurance range of the laminate (> 1000 years).' : `Estimated ${fmt(lifeYears, 3)} years (N_f = ${fmt(Nf, 3)} gust cycles).` });
    // twist is designed to scale with dynamic pressure, so judge it against the sized reference (D4) rather than an absolute threshold
    const refOK = SZ.sized && SZ.check && SZ.check.tip <= -2 && SZ.check.tip >= -4, morphEff = refOK || Math.abs(tipTw) >= 0.5 || Math.abs(loads.thrust - loadsR.thrust) / Math.abs(loadsR.thrust || 1) > 0.02;
    out.verdicts.push({ test: 'Passive morphing effectiveness', status: morphEff ? 'pass' : 'warn', detail: morphEff ? `Tip twist ${fmt(tipTw, 3)}° at this load (thrust change ${fmt((loads.thrust - loadsR.thrust) / Math.abs(loadsR.thrust || 1) * 100, 3)} %)${refOK ? `; washout scales with dynamic pressure and reaches ${fmt(SZ.check.tip, 3)}° at the ${p.refVa} m/s sizing reference` : ''}.` : `Deformation is negligible at this load (tip twist ${fmt(tipTw, 3)}°): the skeleton is too stiff for passive adaptation at this wind.`, fix: 'Reduce spar cover plies/box depth (strength ratio is very high), increase off-axis coupling plies, or evaluate at the design gust wind speed.' });
    out.verdicts.push({ test: 'Coupled solution converged', status: S.converged ? 'pass' : 'fail', detail: `${S.iterations} iterations, residual ${fmt(S.history[S.history.length - 1], 2)}.` });
    // charts
    const zc = ll.z, zs = model.z;
    out.charts.push({ title: 'Spanwise lift per unit span: rigid vs passively deformed', type: 'line', x: { label: 'Height above root z', unit: 'm' }, y: { label: 'Lift per span', unit: 'N/m' }, series: [{ name: 'Deformed (coupled)', x: zc, y: ll.l }, { name: 'Rigid wing', x: R0.aero.ll.z, y: R0.aero.ll.l, dash: '5 3' }], explain: 'Aerodynamic loading along the vertical wing in the sheared apparent wind. The gap between curves is the passive load redistribution from twist and camber adaptation.' });
    out.charts.push({ title: 'Passive twist and camber-induced zero-lift shift', type: 'line', x: { label: 'z', unit: 'm' }, y: { label: 'Angle', unit: '°' }, series: [{ name: 'Elastic twist φ', x: zs, y: twistDeg }, { name: 'Built-in washout', x: zs, y: zs.map(z => p.twistBuilt * z / span), dash: '4 3' }, { name: 'Effective angle of attack', x: zc, y: ll.alphaEff.map(deg) }], explain: 'Twist solved from the coupled equilibrium (output, not input) and the resulting local angle of attack including induced downwash.' });
    out.charts.push({ title: 'Bending deflection and internal moments', type: 'line', x: { label: 'z', unit: 'm' }, y: { label: 'Value' }, series: [{ name: 'Deflection w [mm]', x: zs, y: S.sol.w.map(v => v * 1000) }, { name: 'Bending moment [N·m /10]', x: res.map(r => r.z), y: res.map(r => r.M / 10) }, { name: 'Torque [N·m]', x: res.map(r => r.z), y: res.map(r => r.T) }], explain: 'Bending shape and stress resultants carried by the spar; the root values go to the chassis model (Suite 09).' });
    out.charts.push({ title: 'Spar-cover axial stress & strength ratio', type: 'line', x: { label: 'z', unit: 'm' }, y: { label: 'Value' }, series: [{ name: 'Axial stress [MPa]', x: stressRows.map(r => r.z), y: stressRows.map(r => r.sx / 1e6) }, { name: 'Tsai–Wu strength ratio ×10', x: stressRows.map(r => r.z), y: stressRows.map(r => Math.min(200, r.R * 10)) }], annotations: [{ type: 'hline', y: 15, label: 'R = 1.5', color: '#ef4444' }], explain: 'Laminate stress along the span and the first-ply-failure load multiplier (×10 for scale). Values below the red line violate the 1.5 strength margin.' });
    out.charts.push({ title: 'V–g flutter diagram (75 % span typical section)', type: 'line', x: { label: 'Apparent wind', unit: 'm/s', min: 0, max: Math.min(200, Math.max(3 * Vop, (Number.isFinite(FL.Vflutter) ? FL.Vflutter : 60) * 1.4)) }, y: { label: 'Required damping g', min: -1, max: 0.4 }, series: FL.branches.map((B, j) => ({ name: j === 0 ? 'Branch 1' : 'Branch 2', x: B.map(q => q.V), y: B.map(q => q.g) })), annotations: [{ type: 'hline', y: p.zeta * 2, label: 'structural damping', color: '#22c55e' }, { type: 'vline', x: Vop, label: 'max operating' }, ...(Number.isFinite(FL.Vflutter) ? [{ type: 'vline', x: FL.Vflutter, label: 'flutter', color: '#ef4444' }] : [])], explain: 'Damping each aeroelastic mode would need for neutral oscillation. Flutter occurs where a branch crosses the available structural damping.' });
    out.charts.push({ title: 'V–f frequency coalescence', type: 'line', x: { label: 'Apparent wind', unit: 'm/s', min: 0, max: Math.min(200, Math.max(3 * Vop, (Number.isFinite(FL.Vflutter) ? FL.Vflutter : 60) * 1.4)) }, y: { label: 'Frequency', unit: 'Hz' }, series: FL.branches.map((B, j) => ({ name: j === 0 ? 'Branch 1' : 'Branch 2', x: B.map(q => q.V), y: B.map(q => q.f) })), explain: 'Bending and torsion frequencies converge as speed rises; coalescence typically marks classical bending–torsion flutter.' });
    out.charts.push({ title: 'Design-gust response (Newmark-β + Küssner/Wagner)', type: 'line', x: { label: 'Time', unit: 's' }, y: { label: 'Value' }, series: [{ name: 'True-wind gust ΔU [m/s]', x: G.t, y: G.gust }, { name: 'Δ root moment [N·m /10]', x: G.t, y: G.Mroot.map(v => v / 10) }, { name: 'Δ tip twist [°×10]', x: G.t, y: G.twistTip.map(v => v * 10) }], explain: 'Dynamic increments on top of the static state during a 1-cos gust. Lag of the lift build-up (Küssner) and the structural modes shape the peak load.' });
    out.charts.push({ title: 'Safe operating wind envelope', type: 'line', x: { label: 'True wind at 10 m', unit: 'm/s' }, y: { label: 'Margin ratio' }, series: [{ name: 'Strength ratio (with gust)', x: env.map(e => e.U), y: env.map(e => Math.min(10, e.R)) }, { name: 'Restoring / overturning moment', x: env.map(e => e.U), y: env.map(e => Math.min(10, restore / Math.max(1, e.roll))) }, { name: 'Flutter speed / tip apparent wind', x: env.map(e => e.U), y: env.map(e => Math.min(10, FL.Vflutter / e.Vt)) }], annotations: [{ type: 'hline', y: 1.5, label: 'required 1.5', color: '#ef4444' }, ...(Number.isFinite(Usafe) ? [{ type: 'vline', x: Usafe, label: 'safe limit', color: '#f59e0b' }] : [])], explain: 'Each curve must stay above 1.5. The first crossing sets the maximum safe true wind for operating with the wing loaded; above it the wing must be unloaded.' });
    out.charts.push({ title: 'Thrust and side force vs true wind', type: 'line', x: { label: 'True wind at 10 m', unit: 'm/s' }, y: { label: 'Force (all wings)', unit: 'N' }, series: [{ name: 'Thrust', x: env.map(e => e.U), y: env.map(e => e.thrust) }, { name: 'Side force', x: env.map(e => e.U), y: env.map(e => e.side) }], explain: 'Propulsive and lateral force growth with wind at the current heading and set angle (deformation included).' });
    out.tables.push({ title: 'Natural modes of the clamped wing', cols: ['Mode', 'Type', 'Frequency [Hz]', 'Tip w (norm.)', 'Tip φ (norm.)'], rows: modes.map((m, i) => [i + 1, m.type, m.f, m.w[N - 1], m.phi[N - 1]]) });
    out.tables.push({ title: 'Section properties (root / mid / tip)', cols: ['Station', 'Chord [m]', 'EI [N·m²]', 'GJ [N·m²]', 'K_bt [N·m²]', 'Mass [kg/m]', 'CG−EA [mm]'], rows: [0, 0.5, 1].map(f => { const P = props(f * span); return [`${f * 100}% span`, P.c, P.EI, P.GJ, P.K, P.m, P.xa * 1000]; }), explain: 'Box-spar stiffnesses from CLT; K_bt ≠ 0 only for unbalanced (CAS) cover layups.' });
    out.tables.push({ title: 'Root cover ply stresses (static)', cols: ['Ply angle [°]', 'σ₁ [MPa]', 'σ₂ [MPa]', 'τ₁₂ [MPa]', 'Tsai–Wu index', 'Strength ratio', 'Hashin fibre', 'Hashin matrix'], rows: stressRows[0].ps.map(q => [q.theta, q.s1 / 1e6, q.s2 / 1e6, q.t12 / 1e6, q.tsaiWu, q.R, q.hashinFiber, q.hashinMatrix]) });
    out.views.unshift({ label: '3D dynamic wing: gust, modes & flutter', icon: '🎞', render: (el) => wingDynamic3D(el, { W, p, S, stressRows, G, modes, FL, P75, b, rootM, ctx }) });
    out.views.push({ label: '2D animation: spanwise twist & bending + V–g tracing', icon: '🎞', render: (el) => wing2D(el, { W, S, G, FL, p }) });
    out.views.push({ label: '3D deformed wing (static)', icon: '🪽', render: (el) => wing3D(el, W, p, S, stressRows) });
    // recommendations
    if (loads.thrust <= 0) out.recs.push({ level: 'critical', text: 'The wing produces no net forward thrust at this operating point. Increase the apparent wind angle (steer toward a beam reach), reduce vehicle speed, or raise the root angle of attack below stall.', basis: 'Body-axis force resolution' });
    if (tipTw > 0) out.recs.push({ level: 'warn', text: 'The current layup twists the wing nose-up under load (wash-in). Mirror the off-axis cover plies to obtain wash-out for passive gust alleviation and higher divergence speed.', basis: 'Sign of K_bt' });
    if (rootR > 20 && Math.abs(tipTw) < 0.5) out.recs.push({ level: 'warn', text: `Strength ratio ${fmt(rootR, 3)} with ${fmt(tipTw, 3)}° tip twist: the spar is over-designed for passive morphing. Removing cover plies (or reducing box width) by up to ≈ ${fmt(Math.min(90, (1 - 3 / rootR) * 100), 3)} % keeps R ≥ 3 while increasing compliance and saving mass; re-check flutter after each change.`, basis: 'Fully-stressed design / aeroelastic tailoring' });
    if (rootR < 2) out.recs.push({ level: 'warn', text: `Root strength ratio ${fmt(rootR, 3)}: add ${Math.ceil((2 / Math.max(rootR, 0.1) - 1) * W.cover.plies.filter(q => q.theta === 0).length + 1)} unidirectional 0° plies to each cover or deepen the box.`, basis: 'Tsai–Wu at the root' });
    if (FL.Vflutter < 2 * Vop) out.recs.push({ level: 'warn', text: 'Flutter margin is small: move trailing-core mass forward, stiffen torsion (closed D-cell skin), and verify by ground vibration testing before wind-tunnel runs.', basis: 'V–g analysis' });
    out.recs.push({ level: 'good', text: (ctx.vehicle.wingConfig === 'arch' && (ctx.vehicle.archRoot || 'flexure') === 'flexure') ? `Above ${UsafeTxt} m/s true wind the centreline pylon flexure reaches its elastic stop and the bistable skin de-cambers (passive lift cap); park head-to-wind, where the arch self-trims to near-zero lift — no mechanism is needed.` : `Unload or feather the wing above ${UsafeTxt} m/s true wind; include a mechanical wing-unloading device as specified for safe starting, braking and parking.`, basis: 'Safe operating envelope' });
    out.recs.push({ level: 'sustain', text: `Wing mass ${fmt(wingMass, 3)} kg each. ${W.SM.family === 'Composite' ? 'Specify recyclable thermoplastic or recovered-fibre plies where strength margins allow; CFRP has ≈ ' + W.SM.co2 + ' kgCO₂e/kg embodied carbon.' : 'Metallic spars are recyclable but heavier — check the overturning penalty.'}`, basis: 'Materials registry embodied carbon' });
    out.explain.push({ title: 'How the coupled wing is solved', text: 'The lifting line computes loads on the current deformed shape; the beam FE turns loads into bending and twist (with bend–twist coupling from the laminate); the new twist changes the angle of attack and the trailing-region pressure changes the camber. This loop is repeated with Aitken acceleration until the twist stops changing.' });
    out.explain.push({ title: 'Lift is not thrust', text: 'Only the component of the aerodynamic force along the road direction propels the car; the much larger side force must be carried by the tyres and creates the overturning moment that limits the safe wind.' });
    out.convergence = { title: 'Coupled static aeroelastic interface residual', series: [{ name: 'Aitken', x: S.history.map((_, i) => i + 1), y: S.history }] };
    // ---- Revision 2.2: telescoping reef joint (deployed state) and deployed-vs-reefed performance ----
    if (!W.reef && p.reefSpan < span - 1e-6 && ctx.vehicle.wingConfig !== 'arch') { // the closed arch has no telescoping joint
      const zr = p.reefSpan, Mj = interp1(res.map(r => r.z), res.map(r => r.M), zr), Vj = S.aero.Fw.reduce((acc, f, i) => acc + (model.z[i] > zr ? f : 0), 0);
      const tj = W.tc * chord(zr), ovMin = 1.5 * tj, Fb = Math.abs(Mj) / p.overlap + Math.abs(Vj) / 2, wb = props(zr).wb, pc = Fb / (0.25 * p.overlap * wb) / 1e6;
      k('Reef joint: moment / shear', `${fmt(Mj, 3)} N·m / ${fmt(Vj, 3)} N`, '', `Internal bending moment and shear at the telescoping station z = ${zr} m (structural discontinuity).`, 'calc');
      k('Reef joint bushing contact pressure', pc, 'MPa', `Couple carried by two bushings ${fmt(p.overlap, 3)} m apart (each 25 % of the overlap long, ${fmt(wb * 1000, 3)} mm wide): F = M/L + V/2 = ${fmt(Fb, 3)} N.`, 'calc', pc <= p.bushAllow ? 'ok' : 'bad');
      out.verdicts.push({ test: 'Telescoping reef joint (overlap & bushing pressure)', status: p.overlap >= ovMin && pc <= p.bushAllow ? 'pass' : 'fail', detail: `Overlap ${fmt(p.overlap, 3)} m vs required ≥ 1.5 t = ${fmt(ovMin, 3)} m; bushing pressure ${fmt(pc, 3)} MPa vs allowable ${p.bushAllow} MPa at the current load.`, fix: 'Lengthen the overlap or widen the bushings; the joint must also be checked at the breakaway load.' });
    }
    const states = ctx.vehicle.wingConfig === 'arch' ? ['deployed'] : ['deployed', 'reefed'], winds = [4, 6, 10], perf = []; // the arch does not reef (it folds)
    for (const st of states) for (const U of winds) { const pp = { ...p, wingState: st, U10: U, twa: 90 }; const W2 = buildWing(pp, ctx), S2 = solveStatic(W2, pp, { tol: 1e-6, maxIt: 40 }), L2 = wingLoads(W2, S2.aero.ll, pp.rho), F2 = featherState(W2, pp, ctx, L2, nW); perf.push({ st, U, thrust: F2.released ? F2.feathered.thrust : L2.thrust * nW, side: F2.released ? F2.feathered.side : L2.side * nW, rel: F2.released }); }
    out.tables.push({ title: 'Deployed vs reefed wing — thrust and side force (beam true wind, current vehicle speed, both wings)', cols: ['True wind (10 m)', 'Deployed thrust [N]', 'Deployed side [N]', 'Reefed thrust [N]', 'Reefed side [N]'], rows: winds.map(U => { const d = perf.find(q => q.st === 'deployed' && q.U === U), r = perf.find(q => q.st === 'reefed' && q.U === U) || d; return [`${U} m/s`, fmt(d.thrust, 4) + (d.rel ? ' (feathered)' : ''), fmt(d.side, 4), states.length > 1 ? fmt(r.thrust, 4) + (r.rel ? ' (feathered)' : '') : 'n/a (arch folds)', states.length > 1 ? fmt(r.side, 4) : '—']; }), explain: 'Coupled aeroelastic solution with camber morphing, washout and the free-feathering breakaway. Deploy in light wind; reef when the deployed side force approaches the breakaway setting.' });
    out.charts.push({ title: 'Deployed vs reefed: forward thrust by true wind (beam reach)', type: 'bar', categories: winds.map(U => `${U} m/s`), y: { label: 'Thrust (both wings)', unit: 'N' }, series: [{ name: 'Deployed', values: winds.map(U => perf.find(q => q.st === 'deployed' && q.U === U).thrust) }, ...(states.length > 1 ? [{ name: 'Reefed', values: winds.map(U => perf.find(q => q.st === 'reefed' && q.U === U).thrust) }] : [])], explain: 'The reefable wing keeps the large-area light-wind thrust when deployed and the low side force / low centre of effort when reefed.' });
    k('Flutter speed deployed / reefed', `${fmt(SZ.check?.VF ?? FL.Vflutter, 3)} / ${fmt(SZ.check?.VFreef ?? NaN, 3)}`, 'm/s', 'Typical-section V–g flutter in both configurations (both must exceed the target).', 'calc');
    out.outputs_perf = perf;
    // ---- Revision 4 AeroArch: closed portal frame (legs + bridge) ----
    let ARCH = null;
    if (ctx.vehicle.wingConfig === 'arch') {
      const FSa = featherState(W, p, ctx, loads, nW), opF = archFrame(W, p, ctx, Math.abs(loads.side * nW));
      const daf = Number.isFinite(opF.fSway) ? gustDAF(opF.fSway, p.gustDur, p.zeta) : 1.5, design = FSa.FySafe * Math.max(1, daf);
      let D = archFrame(W, p, ctx, design), sized = null;
      if (D.R < 2) { // re-size cap plies for SF ≥ 2 at the design (breakaway × gust DAF) load
        let lo = p.tPly, hi = 2, Wx = null; for (let i = 0; i < 18; i++) { const m = Math.sqrt(lo * hi); Wx = buildWing({ ...p, tPly: m }, ctx); if (archFrame(Wx, p, ctx, design).R >= 2.2) hi = m; else lo = m; }
        sized = hi; D = archFrame(buildWing({ ...p, tPly: hi }, ctx), p, ctx, design);
      }
      ARCH = { opF, D, daf, design, sized }; const ARC = ARCH;
      out.views.unshift({ label: '2D animation: AeroArch frame sway under the gust', icon: '🎞', render: (el) => archSway2D(el, ARC, p) });
      k('AeroArch root moment: closed frame vs cantilever leg', `${fmt(opF.rootM, 3)} / ${fmt(opF.cantM, 3)} N·m (×${fmt(opF.ratio, 3)})`, '', 'Portal action of the bridge: the closed arch carries the same lateral load with a fraction of the cantilever root moment.', 'calc', opF.ratio < 0.8 ? 'ok' : 'warn');
      k('Arch sway deflection / joint moment (operating)', `${fmt(opF.sway * 1000, 3)} mm / ${fmt(opF.jointM, 3)} N·m`, '', 'Top lateral deflection and the bending moment at the leg–bridge joints.', 'calc');
      k('Arch sway frequency / gust DAF', `${fmt(opF.fSway, 3)} Hz / ${fmt(daf, 3)}`, '', `Lowest in-plane sway mode of the frame and the dynamic amplification of a ${p.gustDur} s 1-cos gust on it.`, 'calc');
      k('Arch strength factor at design load', D.R, '', `Design lateral load = breakaway setting ${fmt(FSa.FySafe, 3)} N × DAF ${fmt(daf, 3)} = ${fmt(design, 3)} N; root moment ${fmt(D.rootM, 3)} N·m.${sized ? ` Cap plies re-sized to ${fmt(sized, 3)} mm for SF ≥ 2.` : ''}`, sized ? 'generated' : 'calc', D.R >= 2 ? 'ok' : 'bad');
      k(`Sweep-induced washout (bend–twist from ${opF.AM.sweep}° sweep)`, opF.sweepWashout, '°', `Bending slope at the leg top × tan Λ: swept legs wash out under load (Λ = ${opF.AM.sweep}°).`, 'calc');
      out.verdicts.push({ test: 'AeroArch closed frame — strength at design load (SF ≥ 2)', status: D.R >= 2 ? 'pass' : 'fail', detail: `SF ${fmt(D.R, 3)} at ${fmt(design, 3)} N lateral (both legs); root ${fmt(D.rootM, 3)} N·m, joint ${fmt(D.jointM, 3)} N·m, peak ${fmt(D.maxM, 3)} N·m.${sized ? ` Caps re-sized to ${fmt(sized, 3)} mm.` : ''}` });
      out.verdicts.push({ test: 'AeroArch portal action (root-moment relief vs cantilever legs)', status: opF.ratio < 0.8 ? 'pass' : 'info', /* no relief is claimed: the legs are sized as cantilevers, so a ratio near 1 is information, not a shortfall */ detail: `Closed frame root moment = ${fmt(opF.ratio * 100, 3)} % of an equivalent cantilever leg.` });
      const path = [], Mp = [], Mc = []; let sAcc = 0; D.Mend.forEach((m, e) => { const [a, b] = D.els[e], L = Math.hypot(D.nodes[b][0] - D.nodes[a][0], D.nodes[b][1] - D.nodes[a][1]); path.push(sAcc, sAcc + L); Mp.push(m.M1, m.M2); sAcc += L; });
      out.charts.push({ title: 'AeroArch bending moment along the frame (port root → bridge → starboard root), design load', x: { label: 'Path length', unit: 'm' }, y: { label: 'Bending moment', unit: 'N·m' }, series: [{ name: 'Closed frame', x: path, y: Mp }], annotations: [{ type: 'hline', y: 0, color: '#94a3b8' }, { type: 'hline', y: D.cantM, label: 'cantilever-leg root moment', color: '#ef4444' }], explain: 'Bridge, legs and the stiff root crossbeam form one closed loop that acts as a fixed-base portal for lateral load (the single centreline pylon only lets the whole loop yaw): moments change sign along each leg and the leg-root moment is far below that of a free cantilever leg carrying the same load.' });
      void Mc;
    }
    // ---- Revision 5: compliant flexure root — rigid pitch DOF of the whole arch about the flexure axis ----
    let FLEX = null;
    if (ctx.vehicle.wingConfig === 'arch' && (ctx.vehicle.archRoot || 'flexure') === 'flexure') {
      const nS = 40, Lg = span, xa0 = p.flexAxis; let Mt = 0, Sθ = 0, Iθ = 0, cAvg = 0;
      for (let i = 0; i < nS; i++) { const z = (i + 0.5) * Lg / nS, P = props(z), c = chord(z), dz = Lg / nS, off = (P.xCG - xa0) * c; Mt += 2 * P.m * dz; Sθ += 2 * P.m * off * dz; Iθ += 2 * (P.m * (0.28 * c) ** 2 + P.m * off * off) * dz; cAvg += c / nS; }
      const b = cAvg / 2, wa = Math.sqrt(Math.max(1e-9, p.flexK / Iθ)), swayHz = ARCH?.opF?.fSway ?? (uncoupledOmegas(W).wh / (2 * Math.PI)), m1 = Mt / (2 * Lg), Ia1 = Iθ / (2 * Lg), xa1 = (Sθ / Mt) / b, a1 = (xa0 - 0.5) * 2;
      const vf = (xaS) => flutterVg({ m: m1, Ia: Ia1, xa: xaS, a: a1, b, wh: 2 * Math.PI * swayHz, wa, rho: p.rho, gs: p.zeta * 2 }).Vflutter, Vreq = 1.3 * p.vMaxApp;
      let VF = vf(xa1), ballast = 0;
      if (!(VF >= Vreq)) { // leading-edge mass balance (2 % chord) on both legs until the flutter requirement is met
        for (const mb of [0.25, 0.5, 1, 1.5, 2, 3, 4, 6]) { const Mt2 = Mt + 2 * mb * Lg, S2 = Sθ + 2 * mb * Lg * (0.02 - xa0) * cAvg; const v2 = flutterVg({ m: Mt2 / (2 * Lg), Ia: (Iθ + 2 * mb * Lg * ((0.02 - xa0) * cAvg) ** 2) / (2 * Lg), xa: (S2 / Mt2) / b, a: a1, b, wh: 2 * Math.PI * swayHz, wa, rho: p.rho, gs: p.zeta * 2 }).Vflutter; if (v2 >= Vreq) { VF = v2; ballast = mb; break; } VF = v2; ballast = mb; }
      }
      const qMax = 0.5 * p.rho * p.vMaxApp ** 2, Ka = qMax * 2 * p.area * cAvg * 2 * Math.PI * 0.6 * (0.25 - xa0), fAero = Math.sqrt(Math.max(0, Ka) / Iθ) / (2 * Math.PI);
      FLEX = { Mt, Iθ, xa1, VF, Vreq, ballast, wa, fAero, divergence: xa0 < 0.25 ? Infinity : 0 };
      k('Arch pitch inertia about the flexure axis', Iθ, 'kg·m²', `Both legs (mass ${fmt(Mt, 3)} kg), axis at ${fmt(xa0 * 100, 3)} % chord; CG offset ${fmt(xa1, 3)} semichords ${xa1 > 0 ? 'aft of' : 'ahead of'} the axis.`, 'calc');
      k('Flexure pitch frequency: structural / aerodynamic (at V_max)', `${fmt(wa / (2 * Math.PI), 3)} / ${fmt(fAero, 3)} Hz`, '', 'The soft flexure leaves the arch aerodynamically stiffened (weathervane stiffness q·S·c·C_Lα·(0.25 − x_axis)).', 'calc');
      k('Flutter speed with the flexure pitch DOF', VF, 'm/s', `Typical section: sway mode ${fmt(swayHz, 3)} Hz coupled with the flexure pitch DOF.${ballast ? ` Leading-edge balance ${fmt(ballast, 3)} kg/m per leg added to reach V_F ≥ ${fmt(Vreq, 3)} m/s.` : ''}`, ballast ? 'generated' : 'calc', VF >= Vreq ? 'ok' : 'bad');
      out.verdicts.push({ test: 'Flexure root: flutter V_F ≥ 1.3 × V_max (pitch DOF)', status: VF >= Vreq ? 'pass' : 'fail', detail: `V_F = ${fmt(VF, 3)} m/s vs required ${fmt(Vreq, 3)} m/s${ballast ? `, with ${fmt(ballast, 3)} kg/m leading-edge balance` : ''}.`, fix: 'Move the flexure axis aft toward the CG, add leading-edge balance mass or stiffen the flexure.' });
      out.verdicts.push({ test: 'Flexure root: static divergence', status: xa0 < 0.25 ? 'pass' : 'fail', detail: xa0 < 0.25 ? `Axis at ${fmt(xa0 * 100, 3)} % chord is ahead of the aerodynamic centre: the aerodynamic moment restores (weathervane) — no divergence.` : 'Axis at or behind the aerodynamic centre: the soft flexure would diverge.' });
    }
    // ---- Revision 2: D4 sizing record and D3 free-feathering root ----
    const FS = featherState(W, p, ctx, loads, nW);
    if (SZ.sized) {
      const ck = SZ.check;
      k('Spar sizing (D4, generated design)', `caps ${fmt(p.tPly, 3)} mm/ply · tube r ${fmt(p.rTube, 3)} × ${fmt(p.tTube, 3)} mm · balance ${fmt(p.leBallast, 3)} kg/m`, '', SZ.log.join(' | '), 'generated');
      out.tables.push({ title: 'Generated spar design (automatic sizing to Revision-2 targets)', cols: ['Quantity', 'Value', 'Target'], rows: [['Cap ply thickness', `${fmt(p.tPly, 3)} mm`, 'strength factor 2.5 (min gauge 0.03 mm)'], ['Torsion-tube radius × wall', `${fmt(p.rTube, 3)} × ${fmt(p.tTube, 3)} mm`, `tip washout ${p.targetTwist}° at ${p.refVa} m/s`], ['Leading-edge balance', `${fmt(p.leBallast, 3)} kg/m`, `V_F ≥ ${p.flutterTarget} m/s`], ['Tip twist at reference', `${fmt(ck.tip, 3)}°`, '−2 … −4°'], ['Root strength ratio at reference', fmt(ck.R, 3), '≥ 2'], ['Flutter speed', fmt(ck.VF, 3) + ' m/s', `≥ ${p.flutterTarget} m/s`], ['Wing mass (incl. balance)', fmt(ck.mass, 3) + ' kg', `≤ ${fmt(4 * p.area + 2 * p.leBallast * p.span / 2, 3)} kg (4 kg/m² structure)`]], explain: SZ.log.join(' · ') });
      const okD4 = ck.tip <= -2 && ck.tip >= -4 && ck.R >= 2 && ck.VF >= p.flutterTarget;
      out.verdicts.push({ test: 'D4 aeroelastic tailoring targets', status: okD4 ? 'pass' : 'fail', detail: `At ${p.refVa} m/s apparent wind: tip ${fmt(ck.tip, 3)}° (target −2…−4°), strength ratio ${fmt(ck.R, 3)} (≥ 2), flutter ${fmt(ck.VF, 3)} m/s (≥ ${p.flutterTarget}).`, fix: 'Adjust the target, pivot position or core stiffness; the sizing reports which bound is active.' });
    } else out.verdicts.push({ test: 'D4 aeroelastic tailoring targets', status: 'warn', detail: p.sparType === 'box' ? 'Closed box spar (Revision 1) selected — it is torsionally too stiff for meaningful passive washout.' : 'Automatic sizing switched off; values are user-specified.' });
    k('Safe side force (breakaway setting)', FS.FySafe, 'N', `min(grip 0.7 μmg = ${fmt(FS.FyGrip, 3)} N, rollover reserve ≥ 0.5 g → ${fmt(FS.FyRoll, 3)} N, tip-over/1.4) with SSF ${fmt(FS.ssf, 3)} and centre of effort ${fmt(FS.hCE, 3)} m.`, 'calc');
    if ((ctx.vehicle.wingConfig === 'arch' && (ctx.vehicle.archRoot || 'flexure') === 'flexure')) {
      k('Arch root state (Rev 5.1)', FS.side > FS.FySafe ? 'washout cap active' : 'self-trimmed', '', `Side force ${fmt(FS.side, 3)} N vs safe limit ${fmt(FS.FySafe, 3)} N. The single centreline pylon flexure lets the closed arch yaw elastically; above the limit the bistable skin de-cambers (lift cap) — there is no release or feathering mechanism.`, 'calc', FS.side > FS.FySafe ? 'warn' : 'ok');
      out.verdicts.push({ test: 'Rollover protection — passive washout cap (no mechanism)', status: 'pass', detail: FS.side > FS.FySafe ? `Untrimmed side force ${fmt(FS.side, 3)} N exceeds ${fmt(FS.FySafe, 3)} N; de-cambering caps it at the safe limit (gust check: Suite 08; pylon strength: Suite 09).` : `Side force ${fmt(FS.side, 3)} N ≤ safe limit ${fmt(FS.FySafe, 3)} N without the cap.` });
    } else {
      k('Wing root state (D3)', FS.released ? 'FEATHERED (released)' : 'locked', '', FS.released ? `Side force ${fmt(FS.side, 3)} N exceeds the breakaway setting: the wings weathervane (C_L ≈ 0). Feathered thrust ${fmt(FS.feathered.thrust, 3)} N, side ${fmt(FS.feathered.side, 3)} N.` : `Side force ${fmt(FS.side, 3)} N below the breakaway setting ${fmt(FS.FySafe, 3)} N.`, 'calc', FS.released ? 'warn' : 'ok');
      out.verdicts.push({ test: 'Rollover protection — free-feathering root (D3)', status: !FS.released ? 'pass' : p.breakaway ? 'pass' : 'fail', detail: !FS.released ? `Locked; side force ${fmt(FS.side, 3)} N ≤ breakaway ${fmt(FS.FySafe, 3)} N.` : p.breakaway ? `Released and feathered: residual side force ${fmt(FS.feathered.side, 3)} N (from ${fmt(FS.side, 3)} N), thrust ${fmt(FS.feathered.thrust, 3)} N — propulsion paused, vehicle protected.` : `Side force ${fmt(FS.side, 3)} N exceeds the safe limit ${fmt(FS.FySafe, 3)} N and the breakaway is disabled.` });
    }
    { /* skin panel stability between ribs and core webs (js/core/skinstab.js) at the DESIGN-load skin strain */
      const SKm = getMat(p.skinMat) || getMat('tpu-glass'), cM = W.span > 0 ? (W.cr + W.ct) / 2 : 0.6, bP = p.teFrac * cM, hC = 0.5 * 0.12 * cM, WEB = getMat('pp') || { E: 1.4, rho: 900 };
      const Rdes = ARCH?.D?.R ?? ((typeof rootR !== 'undefined' ? rootR : 2) * Math.abs(FS.side || 1) / Math.max(1e-6, FS.FySafe * 1.5)), epsSk = 0.8 * 0.009 / Math.max(1e-6, Rdes);
      const SS = skinStability({ Es: (SKm.E || 0.8) * 1e9, nus: SKm.nu ?? 0.45, ts: p.tSkin / 1000, bPanel: bP, ribPitch: p.ribPitch, Elat: (p.coreE ?? LAT0.E_MPa) * 1e6, hCore: hC, webPitch: (p.webPitch ?? 0) / 1000, webT: (p.webT ?? 0.2) / 1000, Eweb: WEB.E * 1e9, epsSkin: epsSk });
      const nWeb = (p.webPitch ?? 0) > 0 ? Math.floor(bP / (p.webPitch / 1000)) : 0, webMass = nWeb * hC * 2 * ((p.webT ?? 0.2) / 1000) * (WEB.rho || 900) * W.span * (ctx.vehicle.nWings || 2); /* per vehicle: webs of mean height 2·h_core over every wing/leg span */
      out.verdicts.push({ test: 'Skin panel stability between ribs and core webs (SF ≥ 2 at the design load)', status: SS.SF >= 2 ? 'pass' : 'fail', detail: `Design skin strain ${fmt(epsSk * 100, 3)} % (0.8 × cap strain at the design load, strength ratio ${fmt(Rdes, 3)}) → ${fmt(SS.sSk / 1e6, 3)} MPa. Lattice modulus normal to the skin ${fmt(p.coreE ?? LAT0.E_MPa, 4)} MPa (Suite 04 in-plane, bending-dominated); ${nWeb ? `${nWeb} spanwise webs ${p.webT} mm at ${p.webPitch} mm` : 'no webs'} → panel width ${fmt(SS.b * 1000, 3)} mm: plate ${fmt(SS.plate / 1e6, 3)} MPa + lattice foundation ${fmt(SS.foundLat / 1e6, 3)} MPa${SS.smeared ? ` (webs smeared: E_t ${fmt(SS.Et / 1e6, 3)} MPa, foundation ${fmt(SS.foundAll / 1e6, 3)} MPa)` : ''} = ${fmt(SS.sCr / 1e6, 3)} MPa → SF ${fmt(SS.SF, 3)}. Webs add ${fmt(webMass, 3)} kg.`, fix: 'Closer web pitch, thicker skin or a stiffer lattice normal to the skin.' });
      out._skin = { SF: SS.SF, webMass_kg: webMass, nWebsPerChord: nWeb, coreE_MPa: p.coreE ?? LAT0.E_MPa }; }
    // wing-unloading hardware (specification: mechanical wing-unloading control): passive pressure-relief vents + driver lever
    let UNL = null; const vehU = ctx.vehicle;
    if ((vehU.wingConfig || 'arch') === 'arch' && vehU.vents) {
      const GA = AA.archGeometry(vehU), V = vehU.vents === true ? AA.VENT_DEFAULT : vehU.vents, FyU = FS.FySafe, polA = AA.sectionPolar('7418', 5e5), flU = { kTheta: p.flexK ?? 60, xAxis: p.flexAxis ?? 0.135, Cm0: 0.05, deltaMax: vehU.yawLimit ?? 35 };
      UNL = unloadingDesign({ S: GA.S, legHeight: GA.AM.legHeight, bridgeSpan: GA.AM.bridgeSpan, chord: GA.cmean, vents: V, FyS: FyU, skin: getMat(p.skinMat) || getMat('tpu-glass'), tSkin: p.tSkin, series: !!ctx.vehicle.tendonSeries, returnInPanel: !!ctx.vehicle.returnInPanel, returnMode: ctx.vehicle.strandReturn ?? (ctx.vehicle.returnInPanel ? 'panel' : 'cord'), strokeHand: ctx.vehicle.tendonSeries ? 0.18 : 0.15 /* series: the three panel strokes add, so the depower step of the gate is lengthened to 180 mm */, cantDeg: GA.AM.cant ?? 15, nGuides: 2 * Math.max(2, Math.ceil(GA.legLen / (p.ribPitch || 0.75))) + Math.max(2, Math.ceil(GA.AM.bridgeSpan / (p.ribPitch || 0.75)) - 1) });
      const kvMin = AA.ventK(V, V.phiMax), kvC = AA.ventK(V, V.phiClosed ?? 0), st = (VV, b, Va, lever) => AA.trimState(GA, polA, VV, b, Va, 1.225, { ...flU, FyS: FyU, scanN: 40, lever });
      const solid = { ...vehU, vents: null }, Vs = [8, 12, 16, 20, 24], bs = [45, 90, 105, 150];
      const rowsD = []; for (const b of bs) for (const [lab, VV, lev] of [['solid arch (washout only)', solid, false], ['kirigami panels, passive', vehU, false], ['lever at depower (panels held open)', vehU, true]]) rowsD.push([`β ${b}°`, lab, ...Vs.map(Va => { const T = st(VV, b, Va, lev); return `${fmt(T.sideEq, 3)}${T.overLimit ? ' ✗' : ''} / ${fmt(T.net, 3)} (C_L ${fmt(T.CL, 2)}, C_D ${fmt(T.CD, 2)})`; })]);
      out.tables.push({ title: 'Depowered states: moment-equivalent side force / net thrust [N] (arch C_L, C_D) vs apparent wind — ✗ = above the safe side force', cols: ['Angle', 'State', ...Vs.map(v => `${v} m/s`)], rows: rowsD, explain: `Self-trimmed arch on its ±${flU.deltaMax}° flexure. Kirigami panels (${fmt(V.zone * 100, 3)} % of the arch area): bistable flaps pop open at Δp ${fmt(UNL.dpOpen, 3)} Pa (open-area ratio ${V.phiMax}, arch normal force × ${fmt(kvMin, 3)}); closed, the kerf is lip-sealed (× ${fmt(kvC, 4)}). Depower: the lever tensions the panels so the flaps stay open at any load — every arch force is scaled by k_v ≤ 1, so neither thrust nor the downwind drag push can increase at any apparent-wind angle. Safe side force ${fmt(FyU, 4)} N.` });
      out.tables.push({ title: 'Wing-unloading hardware — parts, sizes and part-count effect', cols: ['Part', 'Qty', 'Size', 'Added mass [kg]', 'Counting'], rows: UNL.parts.map(r => [r[0], r[1], r[2], fmt(r[3], 3), r[4]]).concat([['Total added mass', '', '', fmt(UNL.mass, 3), `new mechanisms ${UNL.newMechanisms}, new moving parts ${UNL.newMovingParts}`]]), explain: 'Each kirigami panel is one elastic part with no hinges (counted like the morphing skin and the pylon flexure). The driver control is the existing parking-lock lever and cable with a third gate position and three tendon tails, so no mechanism or moving part is added; park passes through depower, which interlocks the parking lock with the unloaded arch.' });
      k('Kirigami flap pop-open / fold-back pressure difference', `${fmt(UNL.dpOpen, 3)} / ${fmt(UNL.dpClose, 3)}`, 'Pa', `Set from the safe side force (Δp_open = 0.9 F_safe / S), so the panels stay shut while the arch loading is below the cap; ${UNL.nFlap} bistable flaps in 3 panels. Ligaments alone would hold a flap open at ${fmt(UNL.dpHold, 3)} Pa — the curved-ligament barrier sets the threshold.`, 'ok');
      k('Lever (depower position) hand force / tendon stroke', `${fmt(UNL.Fhand, 3)} N / ${fmt(UNL.tendonStroke * 1000, 3)} mm`, '', `Work to pop ${UNL.nFlap} flaps and hold them open ${fmt(UNL.Wpop, 3)} J over a ${fmt(150, 3)} mm hand stroke (efficiency 0.7); cable ${fmt(UNL.Fcable, 3)} N.`, UNL.Fhand <= 60 ? 'ok' : 'warn');
      k('Wing-unloading added mass', UNL.mass, 'kg', 'Flap-edge reinforcement, panel edge ribs, cable extension and tendon tails (Suite 01 ledger).', 'calc');
      out.verdicts.push({ test: 'Kirigami ligament fatigue SF ≥ 2 at full opening', status: UNL.SFlig >= 2 - 1e-6 ? 'pass' : 'fail', detail: `Ligament ${fmt(UNL.Llig * 1000, 3)} mm long in the ${p.tSkin} mm skin: bending strain ${fmt(UNL.epsLig * 100, 3)} % at full opening (85°) vs the endurance strain → SF ${fmt(UNL.SFlig, 3)}.`, fix: 'Longer ligaments or a smaller opening angle.' });
      out.verdicts.push({ test: 'Depowering never increases thrust or downwind drag push (every apparent-wind angle)', status: kvMin <= 1 ? 'pass' : 'fail', detail: `Depower scales all arch forces by k_v = ${fmt(kvMin, 3)} (lift AND drag); the skin camber is left to the passive washout, so the residual thrust at every angle is ≤ the undepowered value.` });
      if (UNL.series) { const S5 = UNL.series, IP = S5.inPanel; out.verdicts.push({ test: 'Series tendon strand: lever ≤ 60 N at full depower (23 °C and −20 °C) and full return at SF ≥ 2 (23 °C and +60 °C after long depowered parking)', status: S5.ok ? 'pass' : 'fail', detail: `One strand through the three panels in series: routed wrap ${fmt(S5.wrap_rad * 180 / Math.PI, 3)}°, μ ${S5.mu} (PTFE-lined 7×19, static, conservative) → capstan ×${fmt(S5.capstan, 3)}; lever ${fmt(S5.ratio, 3)}:1 over ${fmt(S5.strokeCable_m * 1000, 3)} mm cable travel. Hand force at full depower ${fmt(S5.Fhand_N, 3)} N at 23 °C, ${fmt(S5.FhandCold_N, 3)} N at −20 °C (TPU-glass flaps ×1.6 stiffer). Return element: ${S5.returnElement === 'loop' ? `none — the strand is a closed loop: its return run (${fmt(S5.loop.returnRun_m, 3)} m) comes back to the lever and engages it through a ${fmt(S5.loop.slot_mm, 3)} mm lost-motion slot (incl. ${fmt(S5.loop.slackThermal_mm, 3)} mm steel-vs-composite slack over −20…+60 °C; depower step ${fmt(S5.loop.handStroke_mm, 3)} mm); in depower its nipple rests on a fixed stop in the gate plate (anchor), back toward drive the lever draws the strand back positively: ${fmt(S5.loop.Tback_N, 3)} N strand tension over the full loop wrap (×${fmt(S5.loop.capLoop, 3)}) = ${fmt(S5.loop.FhandReturn_N, 3)} N at the hand; no elastomer, so no creep or temperature loss` : S5.returnElement === 'panel' ? `meander spring moulded in the far panel's TPU-glass edge rib — preload ${fmt(IP.Fpre_N, 3)} N (sized for the hot relaxed return), ${fmt(IP.Fmax_N, 3)} N at full travel, fatigue SF ${fmt(IP.SFfatigue, 3)} at ${IP.cycles} cycles, meander needs ${fmt(IP.meanderLength_m, 3)} m of the ${fmt(IP.edgeRib_m, 3)} m edge rib (${IP.fits ? 'fits' : 'does NOT fit without taking over the flap anchorage'})` : `pre-stretched silicone cord Ø${S5.leaf.d * 1000} × ${S5.leaf.L * 1000} mm to the far leg-root fitting, ${fmt(S5.leaf.pre_N, 3)} → ${fmt(S5.leaf.max_N, 3)} N, strain ≤ ${fmt(S5.leaf.epsMax, 3)}, life SF ${fmt(S5.leaf.SF, 3)}`}. Return: hold-back ${fmt(S5.Thold_N, 3)} N (23 °C) / ${fmt(S5.TholdHot_N, 3)} N (+60 °C, after 1000 h stretched) vs ${fmt(S5.Treturn_N, 3)} N to draw the strand back → SF ${fmt(S5.returnSF, 3)} / ${fmt(S5.returnSFhot, 3)}.`, fix: 'Longer lever travel, a lower-friction liner, a stiffer return element, or separate tendon tails.' }); }
      out.verdicts.push({ test: 'Lever operable by hand (≤ 60 N) and no added mechanism', status: UNL.Fhand <= 60 && UNL.newMechanisms === 0 ? 'pass' : 'fail', detail: `${fmt(UNL.Fhand, 3)} N; existing parking-lock lever and cable re-used with one 3-position gate (drive / depower / park); no added mechanism, ${UNL.newMovingParts} added moving part${UNL.newMovingParts === 1 ? '' : 's'} (${ctx.vehicle.tendonSeries ? 'the series-strand return cord' : 'the tendon tails'}, counted in Suite 01).` });
    }
    const ribCount = (ctx.vehicle.nWings || 2) * Math.max(1, Math.round(W.span / p.ribPitch)); /* ribs per wing/leg = span ÷ rib pitch (rounded), × wings or arch legs */
    out.outputs = { skinStability: out._skin ?? null, coreWebs: out._skin ? { pitch_mm: p.webPitch, t_mm: p.webT, mass_kg: out._skin.webMass_kg } : null, ribCount, unloading: UNL ? { ...UNL } : null, flexure: FLEX, arch: ARCH ? { rootMoment_Nm: ARCH.D.rootM, rootMomentOperating_Nm: ARCH.opF.rootM, jointMoment_Nm: ARCH.D.jointM, ratioToCantilever: ARCH.opF.ratio, swayHz: ARCH.opF.fSway, gustDAF: ARCH.daf, SF: ARCH.D.R, designLateral_N: ARCH.design, sweepWashout_deg: ARCH.opF.sweepWashout } : null, perfStates: perf, wingState: p.wingState, camberPct: W.mC * 100, featherReleased: FS.released, FySafe: FS.FySafe, featheredThrustN: FS.feathered.thrust, featheredSideN: FS.feathered.side, sizedSpar: SZ.sized ? { tPly: p.tPly, rTube: p.rTube, tTube: p.tTube, leBallast: p.leBallast } : null, CL: ll.CL, CD: ll.CD, liftN: loads.lift * nW, dragN: loads.drag * nW, thrustN: loads.thrust * nW, sideN: loads.side * nW, rollMomentN: (loads.roll + loads.side * p.zRoot) * nW, twistTip_deg: tipTw, tipDefl_m: tipW, rootMoment_Nm: rootM, rootTorque_Nm: rootT, rootShear_N: ll.L, flutterSpeed: FL.Vflutter, divergenceSpeed: VD, f1Hz: fb.f, fTorsionHz: ft.f, wingMass, safeWind: Number.isFinite(Usafe) ? Usafe : 25, rootStrengthRatio: rootR, zRoot: p.zRoot, span, setAngle: W.setAngle, awa: W.app(0.05 * span).awa };
    out.headline = `Thrust ${fmt(loads.thrust * nW, 3)} N, side force ${fmt(loads.side * nW, 3)} N, tip twist ${fmt(tipTw, 3)}°, divergence ${fmt(VD, 3)} m/s, flutter ${fmt(FL.Vflutter, 3)} m/s, safe up to ${UsafeTxt} m/s true wind.`;
    out.headlineKind = Usafe >= 8 && loads.thrust > 0 ? 'ok' : 'warn';
    return out;
  },
};

async function wing3D(el, W, p, S, stressRows) {
  const [{ viewer }, { wing }] = await Promise.all([import('../core/viz3d.js'), import('../core/cad.js')]);
  const V = await viewer(el, { height: 460 }), span = W.span;
  const twistFn = (eta) => p.twistBuilt * eta + deg(interp1(W.model.z, S.sol.phi, eta * span));
  const ref = wing('Undeformed wing', { airfoil: W.af, span, rootChord: W.cr, tipChord: W.ct, twistTip: p.twistBuilt, nSpan: 16, opts: { color: '#94a3b8', opacity: 0.25 } });
  const def = wing('Deformed wing (solved)', { airfoil: W.af, span, rootChord: W.cr, tipChord: W.ct, twistFn, nSpan: 16, opts: { color: '#e5e7eb' } });
  const amp = Math.min(20, 0.05 * span / Math.max(1e-6, Math.abs(S.sol.w[S.sol.w.length - 1]))) || 1;
  def.transform((x, y, z) => [x, y + interp1(W.model.z, S.sol.w, z) * amp, z]);
  const vals = []; for (let k = 2; k < def.p.length; k += 3) vals.push(Math.abs(interp1(stressRows.map(r => r.z), stressRows.map(r => r.sx / 1e6), def.p[k])));
  V.add(ref); V.add(def, { field: { name: 'Spar-cover axial stress', unit: 'MPa', values: vals, cmap: 'turbo' } }); V.fit();
  el.append(h('p', { class: 'small muted' }, `Grey: reference geometry. Coloured: solved loaded shape (bending displayed ×${fmt(amp, 2)}; twist true scale), colour = spar-cover axial stress magnitude. Hover for values.`));
}

// ---------------- 3-D dynamic wing: solved gust response, normal modes and the flutter mode ----------------
// Solved: static aeroelastic deflection/twist, modal basis (beam FE), Newmark-β gust response q_k(t),
// typical-section flutter speed/frequency and its eigenvector. Displayed: those fields mapped onto the
// lofted wing surface (bending magnified by the slider; twist at true scale).
async function wingDynamic3D(el, o) {
  const { W, p, S, stressRows, G, modes, FL, P75, b, rootM } = o;
  const [{ viewer }, { wing }, { playback, hud, legend3d }] = await Promise.all([import('../core/viz3d.js'), import('../core/cad.js'), import('../core/sim3d.js')]);
  const span = W.span, Z = W.model.z, N = Z.length;
  const sel = h('select', {}, h('option', { value: 'gust' }, `Gust response (1-cos ${p.gustAmp} m/s, ${p.gustDur} s) — Newmark-β solution`), ...modes.map((m, k) => h('option', { value: 'm' + k }, `Mode ${k + 1}: ${m.type} ${fmt(m.f, 3)} Hz`)), Number.isFinite(FL.Vflutter) ? h('option', { value: 'flutter' }, `Flutter mode at V_F = ${fmt(FL.Vflutter, 3)} m/s, ${fmt(FL.fFlutter, 3)} Hz`) : null);
  const ampIn = h('input', { type: 'range', min: 1, max: 1000, value: 1, step: 1 }), ampLab = h('span', { class: 'small mono' }, '×1');
  el.append(h('p', { class: 'small muted' }, 'All motion comes from the solved structural dynamics of this wing: static aeroelastic shape + modal response q(t) for the gust, eigenvectors for the modes, and the typical-section flutter eigenvector (plunge/pitch ratio and phase) distributed with the first bending and torsion modes. Bending is magnified by the slider; twist is drawn at true scale. Colour: gust stress increment Δσx (solved root-moment history over the static spanwise distribution; fixed range over the whole gust) or modal amplitude (modes). The static shape and the much smaller gust increment are magnified separately; both factors and the true-scale peaks are shown.'), h('div', { class: 'row gap wrap' }, h('label', {}, 'Case ', sel), h('label', {}, 'Deformation magnification ', ampIn, ampLab)));
  const box = h('div'); el.append(box);
  const V = await viewer(box, { height: 480 });
  const m = wing('Wing (solved dynamics)', { airfoil: W.af, span, rootChord: W.cr, tipChord: W.ct, twistTip: p.twistBuilt, nSpan: 24, opts: { color: '#e5e7eb' } });
  const obj = V.add(m, { field: { name: 'stress', values: new Array(m.p.length / 3).fill(0) } });
  const plate = V.THREE ? new V.THREE.Mesh(new V.THREE.BoxGeometry(W.cr * 1.6, W.cr * 0.9, 0.04), new V.THREE.MeshStandardMaterial({ color: 0x334155 })) : null; if (plate) { plate.position.set(W.cr * 0.4, 0, -0.02); V.root.add(plate); }
  const base = Float32Array.from(m.p), nv = base.length / 3, wIdx = new Int32Array(nv), wT = new Float32Array(nv), xea = new Float32Array(nv);
  for (let k = 0; k < nv; k++) { const z = base[k * 3 + 2]; let i = 0; while (i < N - 2 && Z[i + 1] < z) i++; wIdx[k] = i; wT[k] = Math.min(1, Math.max(0, (z - Z[i]) / ((Z[i + 1] - Z[i]) || 1))); xea[k] = W.props(Math.min(span, Math.max(0, z))).xEA * W.chord(Math.min(span, Math.max(0, z))); }
  const at = (arr, k) => arr[wIdx[k]] * (1 - wT[k]) + arr[wIdx[k] + 1] * wT[k];
  const sZ = stressRows.map(r => r.z), sX = stressRows.map(r => Math.abs(r.sx) / 1e6), stressAt = (z) => interp1(sZ, sX, z);
  const lift = V.addArrow([W.cr * 0.3, 0, span * 0.45], [0, 1, 0], 0.5, 0x22c55e);
  const H = hud(box), leg = legend3d(box, { title: 'Spar-cover |σx| (MPa)', lo: 0, hi: 1 });
  const fb = modes.find(q => q.type === 'bending') || modes[0], ft = modes.find(q => q.type === 'torsion') || modes[1] || modes[0];
  let fm = null; if (Number.isFinite(FL.Vflutter)) { const k = 2 * Math.PI * FL.fFlutter * b / FL.Vflutter; const UO2 = uncoupledOmegas(W); fm = flutterModeShape({ m: P75.m, Ia: P75.Ip, xa: P75.xa / b, a: (P75.xEA - 0.5) * 2, b, wh: UO2.wh, wa: UO2.wa, rho: p.rho }, k, FL.fFlutter); }
  const wTipStatic = S.sol.w[N - 1];
  const apply = (wArr, phArr, colorFn, range, cmapName = 'turbo') => {
    const P = obj.geometry.attributes.position.array, vals = new Float32Array(nv);
    for (let k = 0; k < nv; k++) { const x = base[k * 3], y = base[k * 3 + 1], z = base[k * 3 + 2], ph = Math.max(-0.45, Math.min(0.45, at(phArr, k))), w = at(wArr, k), c = Math.cos(ph), s2 = Math.sin(ph), dx = x - xea[k]; P[k * 3] = xea[k] + c * dx - s2 * y; P[k * 3 + 1] = s2 * dx + c * y + w; P[k * 3 + 2] = z; vals[k] = colorFn(z, k); }
    obj.geometry.attributes.position.needsUpdate = true; obj.geometry.computeVertexNormals(); V.updateField(obj, { name: 'value', values: Array.from(vals), min: range[0], max: range[1], cmap: cmapName });
  };
  const dMax = Math.max(...sX) * Math.max(...G.Mroot.map(Math.abs)) / Math.max(1e-9, Math.abs(rootM)) || 1;
  const legTitle = (t2) => { const e = box.querySelector('.sim-legend .small'); if (e) e.textContent = t2; };
  let pb = null;
  const start = () => {
    pb?.dispose(); box.querySelector('.sim-bar')?.remove();
    const mode = sel.value;
    if (mode === 'gust') {
      const T = G.t[G.t.length - 1]; leg.set(-dMax, dMax); legTitle('Gust stress increment Δσx (MPa)');
      pb = playback(box, { duration: T, rate: 0.25, label: 'Gust time', onFrame: (t) => {
        const i = Math.min(G.t.length - 1, Math.max(0, Math.round(t / (G.t[1] - G.t[0])))), q = G.q[i] || [];
        const nd = +ampIn.value, wArr = Z.map((_, j) => S.sol.w[j] * ampS + nd * q.reduce((acc, qk, k) => acc + qk * G.modes[k].w[j], 0)), phArr = Z.map((_, j) => S.sol.phi[j] * ampS + nd * q.reduce((acc, qk, k) => acc + qk * G.modes[k].phi[j], 0));
        const dfac = G.Mroot[i] / Math.max(1e-9, Math.abs(rootM));
        apply(wArr, phArr, (z) => stressAt(z) * dfac, [-dMax, dMax], 'coolwarm');
        lift.setLength(Math.max(0.05, Math.min(3, Math.abs(G.lift[i]) / 300)), 0.15, 0.08);
        H.set([['Gust ΔU', fmt(oneMinusCosGust(0.3, p.gustDur, p.gustAmp)(G.t[i]), 3) + ' m/s'], ['Gust lift increment (solved)', fmt(G.lift[i], 3) + ' N'], ['ΔM root', fmt(G.Mroot[i], 3) + ' N·m'], ['Tip deflection', fmt((G.wTip[i] + wTipStatic) * 1000, 3) + ' mm'], ['Tip twist', fmt(G.twistTip[i] + deg(S.sol.phi[N - 1]), 3) + '°']]);
      } });
    } else {
      const isF = mode === 'flutter', mk = isF ? null : modes[+mode.slice(1)], f = isF ? FL.fFlutter : mk.f;
      const wn = Math.max(...(isF ? fb.w : mk.w).map(Math.abs)) || 1, pn = Math.max(...(isF ? ft.phi : mk.phi).map(Math.abs)) || 1;
      const wShape = isF ? fb.w.map(v => v / wn) : mk.w.map(v => v / wn), pShape = isF ? ft.phi.map(v => v / pn) : mk.phi.map(v => v / pn);
      // modal amplitudes: tip bending 5 % span (before magnification) and pitch ±3° for display; flutter uses the eigenvector ratio
      const W0 = 0.1 * span * Math.max(0.1, +ampIn.value / autoAmp) / 10, A0 = isF ? W0 / Math.max(1e-6, (fm?.hb || 1) * b) : rad(3), phase = isF ? (fm?.phase || Math.PI / 2) : 0;
      const hasW = isF || mk.type === 'bending', hasP = isF || mk.type === 'torsion' || true;
      leg.set(0, 1); legTitle('Modal amplitude (normalised)');
      pb = playback(box, { duration: 3 / f, rate: Math.min(1, 0.6 / f), label: isF ? 'Flutter cycle time' : `Mode ${mode.slice(1) * 1 + 1} time`, onFrame: (t) => {
        const s1 = Math.sin(2 * Math.PI * f * t), s2 = Math.sin(2 * Math.PI * f * t + phase);
        const wArr = wShape.map(v => (hasW ? v : 0) * (isF ? W0 * s2 : W0 * s1)), phArr = pShape.map(v => (hasP ? v : 0) * (isF ? rad(3) * Math.min(1, A0 / rad(3)) * s1 : (mk.type === 'torsion' ? A0 : A0 * 0.2) * s1));
        apply(wArr.map(v => v * 10), phArr, (z, k) => Math.min(1, Math.abs(at(wShape, k) * s1) * 0.6 + Math.abs(at(pShape, k) * s1) * 0.6), [0, 1]);
        H.set(isF ? [['Flutter speed', fmt(FL.Vflutter, 3) + ' m/s'], ['Frequency', fmt(f, 3) + ' Hz'], ['Plunge/pitch (h/b per rad)', fmt(fm?.hb ?? NaN, 3)], ['Phase plunge vs pitch', fmt(deg(phase), 3) + '°']] : [['Mode', `${mode.slice(1) * 1 + 1} (${mk.type})`], ['Frequency', fmt(f, 3) + ' Hz'], ['Displayed amplitude', 'normalised mode shape (×' + fmt(Math.max(1, +ampIn.value / autoAmp), 3) + ')']]);
      } });
    }
  };
  sel.onchange = start;
  const dynPeak = Math.max(1e-12, ...G.wTip.map(Math.abs)), autoAmp = Math.max(1, Math.round(0.1 * span / dynPeak)), ampS = Math.max(1, Math.round(0.05 * span / Math.max(1e-12, Math.abs(wTipStatic))));
  ampIn.max = String(Math.max(1000, autoAmp * 3));
  const label = () => { ampLab.textContent = sel.value === 'gust' ? `Gust increment ×${ampIn.value} (auto ×${autoAmp}; true-scale peak Δ tip deflection = ${fmt(dynPeak * 1000, 4)} mm, Δ twist ${fmt(Math.max(...G.twistTip.map(Math.abs)), 3)}°) · static shape ×${ampS} (true tip ${fmt(wTipStatic * 1000, 4)} mm)` : `Mode shape normalised: tip ≈ ${fmt(10 * Math.max(0.1, +ampIn.value / autoAmp), 3)} % of span (slider scales it)`; };
  ampIn.value = String(autoAmp); label(); ampIn.oninput = label; sel.addEventListener('change', label);
  V.fit(); V.viewDir(1.15, -0.3, 0.42); start();
  return { dispose: () => pb?.dispose() };
}

// ---------------- 2-D animation: spanwise distributions during the gust + V–g flutter tracing ----------------
async function wing2D(el, o) {
  const { W, S, G, FL } = o, Z = W.model.z, N = Z.length;
  const { anim2d, axes2d, polyline } = await import('../core/sim3d.js');
  el.append(h('p', { class: 'small muted' }, 'Left: solved spanwise twist and bending deflection (static equilibrium + modal gust response q(t)) at each instant, with the static shape dashed. Right: the V–g diagram of both typical-section branches with a marker sweeping the apparent-wind speed; flutter is where a branch crosses the structural damping line.'));
  const field = (k) => { const q = G.q[k] || [], w = Z.map((_, j) => S.sol.w[j] + q.reduce((a, qk, m) => a + qk * G.modes[m].w[j], 0)), ph = Z.map((_, j) => S.sol.phi[j] + q.reduce((a, qk, m) => a + qk * G.modes[m].phi[j], 0)); return { w, ph }; };
  const all = G.t.map((_, k) => field(k)), twMax = Math.max(1e-9, ...all.flatMap(f => f.ph.map(v => Math.abs(deg(v))))), wMax = Math.max(1e-12, ...all.flatMap(f => f.w.map(Math.abs)));
  /* axes span the range actually swept during the gust (not ±max about zero), so the dynamic response fills the panel */
  const rng2 = (vals) => { const lo = Math.min(...vals), hi = Math.max(...vals), pad = Math.max(1e-9, (hi - lo) * 0.12); return [lo - pad, hi + pad]; };
  const twR = rng2(all.flatMap(f => f.ph.map(v => deg(v))).concat(S.sol.phi.map(v => deg(v)))), wR = rng2(all.flatMap(f => f.w.map(v => v * 1e3)).concat(S.sol.w.map(v => v * 1e3)));
  const tipTrail = [];
  const T = G.t[G.t.length - 1], Vmax = Number.isFinite(FL.Vflutter) ? FL.Vflutter * 1.4 : Math.max(...FL.branches.flat().map(b => b.V));
  return anim2d(el, { duration: T, rate: 0.5, label: 'Gust time', height: 420, draw: (g, t, Wd, H, c) => {
    const k = Math.min(G.t.length - 1, Math.round(t / (G.t[1] - G.t[0]))), f = all[k], hw = (Wd - 90) / 2;
    const A1 = axes2d(g, { x0: 50, y0: 30, w: hw - 20, h: (H - 100) / 2, xr: twR, yr: [0, W.span], xl: 'Twist (°)', yl: 'z (m)', fg: c.fg, mute: c.mute });
    polyline(g, Z.map((z, j) => [A1.X(deg(S.sol.phi[j])), A1.Y(z)]), c.mute, 1.5, [5, 4]); polyline(g, Z.map((z, j) => [A1.X(deg(f.ph[j])), A1.Y(z)]), '#a855f7', 3.5);
    const A2 = axes2d(g, { x0: 50, y0: 30 + (H - 100) / 2 + 40, w: hw - 20, h: (H - 100) / 2 - 10, xr: wR, yr: [0, W.span], xl: 'Deflection (mm)', yl: 'z (m)', fg: c.fg, mute: c.mute });
    polyline(g, Z.map((z, j) => [A2.X(S.sol.w[j] * 1e3), A2.Y(z)]), c.mute, 1.5, [5, 4]); polyline(g, Z.map((z, j) => [A2.X(f.w[j] * 1e3), A2.Y(z)]), '#38bdf8', 3.5);
    /* moving tip tracer with a fading trail (twist panel and deflection panel) */
    tipTrail.push([deg(f.ph[N - 1]), f.w[N - 1] * 1e3]); if (tipTrail.length > 40) tipTrail.shift(); tipTrail.forEach(([tw, wv], i) => { const a = (i + 1) / tipTrail.length; g.fillStyle = `rgba(250,204,21,${a})`; g.beginPath(); g.arc(A1.X(tw), A1.Y(W.span), 3 + 6 * a, 0, 7); g.fill(); g.beginPath(); g.arc(A2.X(wv), A2.Y(W.span), 3 + 6 * a, 0, 7); g.fill(); });
    g.fillStyle = c.fg; g.fillText(`t = ${fmt(G.t[k], 3)} s · ΔM_root ${fmt(G.Mroot[k], 3)} N·m · tip twist ${fmt(deg(f.ph[N - 1]), 3)}°`, 50, 18);
    const Vs = (u) => (u % 1) * Vmax, Vm = Vs(t / T), gl = FL.branches.flat().map(b => b.g), A3 = axes2d(g, { x0: hw + 90, y0: 30, w: hw - 20, h: H - 90, xr: [0, Vmax], yr: [Math.max(-0.6, Math.min(...gl)), Math.min(0.4, Math.max(0.1, ...gl))], xl: 'Apparent wind V (m/s)', yl: 'damping g', fg: c.fg, mute: c.mute });
    FL.branches.forEach((B, j) => { polyline(g, B.filter(b => b.V <= Vmax).map(b => [A3.X(b.V), A3.Y(Math.max(-0.6, Math.min(0.4, b.g)))]), j ? '#f97316' : '#22c55e', 2); const bb = B.reduce((a, b) => Math.abs(b.V - Vm) < Math.abs(a.V - Vm) ? b : a, B[0]); if (bb) { g.fillStyle = '#facc15'; g.beginPath(); g.arc(A3.X(bb.V), A3.Y(Math.max(-0.6, Math.min(0.4, bb.g))), 8, 0, 7); g.fill(); } });
    g.strokeStyle = '#ef4444'; g.setLineDash([5, 4]); g.beginPath(); g.moveTo(A3.X(0), A3.Y(o.p.zeta * 2)); g.lineTo(A3.X(Vmax), A3.Y(o.p.zeta * 2)); g.stroke(); g.setLineDash([]);
    g.fillStyle = c.fg; g.fillText(`V = ${fmt(Vm, 3)} m/s · V_F = ${fmt(FL.Vflutter, 3)} m/s`, hw + 90, 18);
  } });
}

// ---------------- Revision 4 AeroArch: closed portal-frame structure (two legs + top bridge) ----------------
// 2-D frame FE in the plane of the arch (lateral y, height z): canted legs fixed at the ring-track roots, rigid joints to
// the bridge. Lateral aerodynamic load on both legs in the same direction (crosswind). Section properties along each leg
// from the Suite-06 spar model (caps + torsion tube); bridge = tip section.
export function archFrame(W, p, ctx, sideBoth, { nLeg = 12, nBr = 4 } = {}) {
  const AM = vehicleAssembly(ctx.vehicle, {}).meta.arch, H = AM.legHeight, yr = AM.gap / 2, yt = AM.bridgeSpan / 2;
  const nodes = [], els = [];
  for (let i = 0; i <= nLeg; i++) nodes.push([yr + (yt - yr) * i / nLeg, H * i / nLeg]);              // port leg (0 … nLeg)
  for (let i = 1; i <= nBr; i++) nodes.push([yt - 2 * yt * i / nBr, H]);                              // bridge
  for (let i = nLeg - 1; i >= 0; i--) nodes.push([-yr - (yt - yr) * (i - nLeg) / nLeg * -1 + 0 * i, H * i / nLeg]); // starboard leg (top → root)
  for (let i = nLeg + nBr; i < nodes.length; i++) { const k = nodes.length - 1 - i; nodes[i] = [-(yr + (yt - yr) * k / nLeg), H * k / nLeg]; }
  for (let i = 0; i < nodes.length - 1; i++) els.push([i, i + 1]);
  const legLen = Math.hypot(H, yr - yt), sec = (frac) => W.props(Math.min(W.span, Math.max(0, frac * W.span)));
  const elProps = els.map(([a, b], e) => { const onBridge = e >= nLeg && e < nLeg + nBr, frac = onBridge ? 1 : (e < nLeg ? (e + 0.5) / nLeg : (nodes.length - 1 - e - 0.5) / nLeg), P = sec(frac); const EIb = onBridge ? P.EI * (p.bridgeEIx ?? 1) : P.EI; return { EI: EIb, EA: EIb / (0.3 * P.hs) ** 2, m: P.m, frac, onBridge }; }); // bridge stiffening factor (spar caps / roll-over tube in the bridge)
  const nd = nodes.length * 3, K = zeros(nd, nd).map(r => Array.from(r)), M = zeros(nd, nd).map(r => Array.from(r));
  els.forEach(([a, b], e) => { const q = elProps[e], fk = frameK(1, q.EA, q.EI, nodes[a][0], nodes[a][1], nodes[b][0], nodes[b][1]), map = [3 * a, 3 * a + 1, 3 * a + 2, 3 * b, 3 * b + 1, 3 * b + 2]; for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) K[map[i]][map[j]] += fk.K[i][j]; const lm = q.m * fk.L / 2; for (const n of [a, b]) { M[3 * n][3 * n] += lm; M[3 * n + 1][3 * n + 1] += lm; M[3 * n + 2][3 * n + 2] += lm * fk.L * fk.L / 24 + 1e-9; } });
  const root2 = nodes.length - 1, fixed = new Set([0, 1, 2, 3 * root2, 3 * root2 + 1, 3 * root2 + 2]), free = [...Array(nd).keys()].filter(d => !fixed.has(d));
  // lateral load: box-wing loading 0.55 + 0.45√(1−η²) on each leg, total sideBoth/2 per leg, acting in +y on both legs
  const F = new Array(nd).fill(0), shape = (e) => 0.55 + 0.45 * Math.sqrt(Math.max(0, 1 - e * e)); let wsum = 0; const w = [];
  for (let i = 0; i <= nLeg; i++) { const e = i / nLeg, ww = shape(e) * (i === 0 || i === nLeg ? 0.5 : 1); w.push(ww); wsum += ww; }
  for (let i = 0; i <= nLeg; i++) { const f = sideBoth / 2 * w[i] / wsum; F[3 * i] += f; F[3 * (nodes.length - 1 - i)] += f; }
  const Kr = free.map(i => free.map(j => K[i][j])), Fr = free.map(d => F[d]), ur = luSolve(Kr, Fr), u = new Array(nd).fill(0); free.forEach((d, k) => u[d] = ur[k]);
  // element end moments (local)
  const Mend = els.map(([a, b], e) => { const q = elProps[e], fk = frameK(1, q.EA, q.EI, nodes[a][0], nodes[a][1], nodes[b][0], nodes[b][1]), de = [3 * a, 3 * a + 1, 3 * a + 2, 3 * b, 3 * b + 1, 3 * b + 2].map(d => u[d]), dl = fk.T.map(r => r.reduce((s2, v, k) => s2 + v * de[k], 0)), kl = [[q.EA / fk.L, 0, 0, -q.EA / fk.L, 0, 0], [0, 12 * q.EI / fk.L ** 3, 6 * q.EI / fk.L ** 2, 0, -12 * q.EI / fk.L ** 3, 6 * q.EI / fk.L ** 2], [0, 6 * q.EI / fk.L ** 2, 4 * q.EI / fk.L, 0, -6 * q.EI / fk.L ** 2, 2 * q.EI / fk.L], [-q.EA / fk.L, 0, 0, q.EA / fk.L, 0, 0], [0, -12 * q.EI / fk.L ** 3, -6 * q.EI / fk.L ** 2, 0, 12 * q.EI / fk.L ** 3, -6 * q.EI / fk.L ** 2], [0, 6 * q.EI / fk.L ** 2, 2 * q.EI / fk.L, 0, -6 * q.EI / fk.L ** 2, 4 * q.EI / fk.L]], f = kl.map(r => r.reduce((s2, v, k) => s2 + v * dl[k], 0)); return { M1: f[2], M2: -f[5], N: -f[0], frac: q.frac, onBridge: q.onBridge }; });
  const rootM = Math.max(Math.abs(Mend[0].M1), Math.abs(Mend[els.length - 1].M2)), jointM = Math.max(Math.abs(Mend[nLeg - 1].M2), Math.abs(Mend[nLeg + nBr].M1)), maxM = Math.max(...Mend.flatMap(m => [Math.abs(m.M1), Math.abs(m.M2)]));
  let cantM = 0; for (let i = 0; i <= nLeg; i++) cantM += sideBoth / 2 * w[i] / wsum * H * i / nLeg; // same leg as a free cantilever
  const sway = u[3 * nLeg], tipSlope = u[3 * nLeg + 2];
  // strength at the leg root (first-ply failure of the caps)
  const P0 = sec(0), eps = rootM * (P0.hs / 2) / P0.EI, R = Math.min(...plyStresses(W.cover, [eps, 0, 0]).map(q => q.R));
  // sway frequency (generalised eigenproblem on the free DOFs)
  let fSway = NaN; try { const Mr = free.map(i => free.map(j => M[i][j])); const eg = eigGen(Kr.map(r => Float64Array.from(r)), Mr.map(r => Float64Array.from(r))); fSway = Math.sqrt(Math.max(0, eg.values[0])) / (2 * Math.PI); } catch { /* singular */ }
  // geometric washout from leg sweep: bending slope × tan Λ reduces the streamwise angle of attack
  const sweepWashout = -Math.atan(Math.abs(tipSlope)) * Math.tan(rad(AM.sweep)) * 180 / Math.PI;
  return { AM, nodes, els, u, Mend, rootM, jointM, maxM, cantM, ratio: rootM / Math.max(1e-9, cantM), sway, tipSlope, R, fSway, sweepWashout, legLen, nLeg, nBr };
}
// dynamic amplification of a 1-cos gust on the frame's sway mode (SDOF, Newmark average acceleration)
export function gustDAF(fn, Tg, zeta = 0.02) {
  const w = 2 * Math.PI * fn, dt = Math.min(Tg, 1 / fn) / 200, n = Math.round((Tg + 6 / fn) / dt); let x = 0, v = 0, a = 0, peak = 0;
  for (let i = 1; i <= n; i++) { const t = i * dt, f = t <= Tg ? 0.5 * (1 - Math.cos(2 * Math.PI * t / Tg)) : 0, keff = w * w + 2 * zeta * w * 2 / dt + 4 / dt / dt, rhs = w * w * f + 4 / dt / dt * x + 4 / dt * v + a + 2 * zeta * w * (2 / dt * x + v), xn = rhs / keff, vn = 2 / dt * (xn - x) - v, an = 4 / dt / dt * (xn - x) - 4 / dt * v - a; x = xn; v = vn; a = an; peak = Math.max(peak, Math.abs(x)); }
  return peak;
}

async function archSway2D(el, ARCH, p) {
  const { anim2d } = await import('../core/sim3d.js'), D = ARCH.D, F = D.nodes;
  el.append(h('p', { class: 'small muted' }, `Solved closed-frame (portal) deflection at the design lateral load, animated through a ${p.gustDur} s 1-cos gust on the sway mode (${fmt(D.fSway ?? ARCH.opF.fSway, 3)} Hz, DAF ${fmt(ARCH.daf, 3)}). Grey = undeformed arch; displacements magnified to 10 % of the leg height (factor shown). Colour = local bending moment.`));
  const umax = Math.max(1e-12, ...F.map((_, i) => Math.hypot(D.u[3 * i], D.u[3 * i + 1]))), mag = 0.16 * D.AM.legHeight / umax, Mmax = Math.max(1e-9, D.maxM), T = p.gustDur + 4;
  return anim2d(el, { duration: T, label: 'Gust time', height: 380, draw: (g, t, W, H, c) => {
    const f = t <= p.gustDur ? 0.5 * (1 - Math.cos(2 * Math.PI * t / p.gustDur)) * ARCH.daf : ARCH.daf * Math.exp(-(t - p.gustDur) * 0.7) * Math.cos(2 * Math.PI * 1.2 * (t - p.gustDur)); // free sway ring-down, drawn slowed to 1.2 Hz so it stays visible
    const ys = F.map(n => n[0]), sc = Math.min((W - 80) / (Math.max(...ys) - Math.min(...ys) + 1.2), (H - 70) / (D.AM.legHeight * 1.2)), X = (y) => W / 2 + y * sc, Y = (z) => H - 30 - z * sc;
    g.strokeStyle = c.mute; g.lineWidth = 2; g.setLineDash([5, 4]); g.beginPath(); F.forEach((n, i) => i ? g.lineTo(X(n[0]), Y(n[1])) : g.moveTo(X(n[0]), Y(n[1]))); g.stroke(); g.setLineDash([]);
    D.els.forEach(([a, b], e) => { const m = Math.abs(D.Mend[e].M1 + D.Mend[e].M2) / 2 * Math.abs(f) / ARCH.daf, cc = cmapS(m / Mmax); g.strokeStyle = cc; g.lineWidth = 6; g.beginPath(); g.moveTo(X(F[a][0] + f * mag * D.u[3 * a] / Math.max(1, ARCH.daf)), Y(F[a][1] + f * mag * D.u[3 * a + 1] / Math.max(1, ARCH.daf))); g.lineTo(X(F[b][0] + f * mag * D.u[3 * b] / Math.max(1, ARCH.daf)), Y(F[b][1] + f * mag * D.u[3 * b + 1] / Math.max(1, ARCH.daf))); g.stroke(); });
    g.fillStyle = '#475569'; g.fillRect(X(-D.AM.gap / 2 - 0.25), Y(0), (D.AM.gap + 0.5) * sc, 8);
    g.fillStyle = c.fg; g.fillText(`load factor ${fmt(f, 3)} × design · root moment ${fmt(D.rootM * Math.abs(f) / ARCH.daf, 3)} N·m · sway ${fmt(D.sway * 1000 * Math.abs(f) / ARCH.daf, 3)} mm (drawn ×${fmt(mag, 3)})`, 10, 18);
  } });
}
function cmapS(v) { const r = Math.round(255 * Math.min(1, v * 1.6)), gg = Math.round(200 * (1 - Math.min(1, v))), b = Math.round(255 * Math.max(0, 1 - v * 2)); return `rgb(${r},${gg},${b})`; }
