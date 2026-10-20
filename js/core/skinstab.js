// Morphing-skin panel stability between ribs and core webs (Suite 06). The trailing core is a 2-D lattice whose cell plane is the
// aerofoil section, so its modulus normal to the skin is the lattice IN-PLANE modulus (bending-dominated, tiny for a chiral
// cell). Thin spanwise through-thickness webs printed with the lattice tie the two skins (stretch-dominated in the thickness
// direction, compliant to chordwise camber change). Two bounds, the lower governs:
//   (a) plate between webs/ribs: σ = k π² D/(t b²), b = min(web pitch, panel width), k = (mb/a + a/(mb))² minimised over m;
//   (b) elastic-foundation (Winkler) skin: σ = 2√(D k_f)/t, k_f = E_t/h_core, E_t = E_lattice,⊥ + E_web·t_w/s_w,
//       valid only when the web pitch is well below the foundation buckle half-wavelength λ = π (D/k_f)^¼ (else (a) applies).
export function skinStability({ Es, nus = 0.45, ts, bPanel, ribPitch, Elat, hCore, webPitch = 0, webT = 0, Eweb = 0, epsSkin }) {
  const D = Es * ts ** 3 / (12 * (1 - nus * nus)), b = webPitch > 0 ? Math.min(webPitch, bPanel) : bPanel, a = ribPitch;
  const mMax = Math.max(1, Math.ceil(a / b) + 2), kP = Math.min(...Array.from({ length: mMax }, (_, i) => i + 1).map(m => (m * b / a + a / (m * b)) ** 2)), /* half-waves along the rib pitch: k → 4 for long panels */ plate = kP * Math.PI ** 2 * D / (ts * b * b);
  const Et = Elat + (webPitch > 0 ? Eweb * webT / webPitch : 0), kf = Et / Math.max(1e-4, hCore), lam = Math.PI * (D / Math.max(1e-9, kf)) ** 0.25;
  const foundLat = 2 * Math.sqrt(D * Elat / Math.max(1e-4, hCore)) / ts, foundAll = 2 * Math.sqrt(D * kf) / ts;
  const smeared = webPitch > 0 && webPitch < 0.5 * lam, sCr = smeared ? foundAll : plate + foundLat, sSk = Es * epsSkin;
  return { D, b, plate, foundLat, foundAll, Et, lambda: lam, smeared, sCr, sSk, SF: sCr / Math.max(1e-12, sSk) };
}
// defaults of the Suite 04 lattice (so Suite 06 gives the same answer with or without Suite 04 upstream)
export const LATTICE_DEFAULT = { cell: 'tetrachiral', tr: 0.0055, l: 75, depth: 60, mat: 'pp', rr: 0.25, hr: 2, theta: 20 };
