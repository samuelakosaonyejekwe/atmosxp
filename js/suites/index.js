// Registry of the 26 engineering suites (lazy-loaded modules). Numbering follows the input/output specification.
const G1 = 'Automobile geometry, materials & aerodynamics', G2 = 'Mechanical systems, thermal physics & active materials', G3 = 'Metamaterial propulsion, energy management & vehicle operation', G4 = 'Manufacturing, safety, research credibility & economics';
const S = (num, short, title, group, icon, file) => ({ id: 's' + String(num).padStart(2, '0'), num, short, title, group, icon, load: () => import(`./${file}.js`) });
export const SUITES = [
  S(1, 'Architecture', 'Complete automobile architecture & system engineering', G1, '🚗', 's01-architecture'),
  S(2, 'CAD & meshing', 'Parametric 2D/3D CAD, geometry generation & numerical meshing', G1, '📐', 's02-cad-mesh'),
  S(3, 'Materials', 'Engineering materials library & property prediction', G1, '🧱', 's03-materials'),
  S(4, 'Metamaterials', 'Metamaterial unit-cell design, mechanics & homogenisation', G1, '⬡', 's04-metamaterials'),
  S(5, 'Atmosphere', 'Live atmospheric environment, meteorology & environmental physics', G1, '🌦', 's05-atmosphere'),
  S(6, 'Aeroelastic wing', 'Fixed skeleton, passive aeroelastic wings & morphing surfaces', G1, '🪽', 's06-wing'),
  S(7, 'CFD', 'Aerodynamics & computational fluid dynamics', G1, '🌀', 's07-cfd'),
  S(8, 'FSI', 'Coupled fluid–structure interaction & aeroelastic simulation', G1, '🔁', 's08-fsi'),
  S(9, 'Chassis FEA', 'Carbon-composite monocoque, structural analysis & crashworthiness', G2, '🏗', 's09-chassis'),
  S(10, 'Wheels & tyres', 'Road wheels, tyres, bearings & tyre–road interaction', G2, '🛞', 's10-wheels'),
  S(11, 'Steering & handling', 'Steering, directional stability, handling & vehicle dynamics', G2, '🧭', 's11-steering'),
  S(12, 'Braking', 'Mechanical braking, stopping & parking stability', G2, '🛑', 's12-braking'),
  S(13, 'Photonic collectors', 'Photonic metamaterials, solar absorption & radiative cooling', G2, '☀', 's13-photonic'),
  S(14, 'Thermal metamaterials', 'Thermal metamaterials, heat spreaders, insulation & management', G2, '🌡', 's14-thermal'),
  S(15, 'NiTi actuator', 'NiTi shape-memory metamaterial & thermomechanical conversion', G2, '🔩', 's15-niti'),
  S(16, 'Alternative actuators', 'Competing artificial-muscle & thermoresponsive actuators', G2, '🧵', 's16-actuators'),
  S(17, 'Direct-rim hub drive', 'Direct-rim metamaterial thermomechanical wheel-hub propulsion', G3, '⚙', 's17-hub'),
  S(18, 'Energy buffer', 'Bistable metamaterial energy storage & elastic buffering', G3, '🔋', 's18-buffer'),
  S(19, 'Passive switching', 'Passive mechanical mode switching & control', G3, '🔀', 's19-switching'),
  S(20, 'Vehicle energy', 'Complete automobile propulsion, energy management & operating modes', G3, '⚡', 's20-energy'),
  S(21, 'Acoustics', 'Acoustics, structural vibration & near-silent operation', G3, '🔊', 's21-acoustics'),
  S(22, 'Manufacturing', 'Manufacturing, additive fabrication & quality engineering', G4, '🏭', 's22-manufacturing'),
  S(23, 'Safety & reliability', 'Safety, reliability, structural durability & failure prediction', G4, '🛡', 's23-safety'),
  S(24, 'Verification & validation', 'Calibration, verification, validation & experimental methodology', G4, '✅', 's24-vv'),
  S(25, 'Sensitivity & optimisation', 'Sensitivity, uncertainty quantification, AI & multidisciplinary optimisation', G4, '🎯', 's25-optimisation'),
  S(26, 'Economics & sustainability', 'Economics, lifecycle sustainability & manufacturability', G4, '♻', 's26-economics'),
];
export const GROUPS = [G1, G2, G3, G4];
export const suiteIndex = (id) => SUITES.findIndex(s => s.id === id);
