// Release notes shown in the update / "What's new" pop-up. Newest first; `id` must increase.
export const CHANGELOG = [
  { id: 7, date: '2026-10-20', title: 'Revision 5.1 — the new PHOENIX-ATMOS-XP automobile', items: [
    'One closed, splayed arch wing on a single compliant titanium pylon: it self-trims to the wind by bending — no ring track, bearing, hinge, drive or lock. Lifting body 3.5 m long, 2.0 m wide over the wheel pods, 3.1 m tall, 181 kg.',
    'Faster than every earlier revision and the original at every wind: 46.5 / 61 / 73.5 / 82.5 / 94 / 104 km/h at 2 / 2.5 / 3 / 4 / 6 / 10 m/s (same models for every design).',
    'Much more stable: stability factor 1.61, stays upright to 26.5 m/s of side wind (17 m/s even if gust washout failed), nose-over limit 1.6 g with a front-brake limiter.',
    'Lower drag (C_dA 0.127 m²), 53 W at 20 km/h, cheaper to build and run, 20 g CO₂e/km over its life, fewest moving parts (8 mechanisms), 43 wind-free starts per sunny day.',
    'Every part checked: tyres, rims and front axle; pylon, root frame and brush seal; rear fork; crash cone, side sills and harness anchors; buffer containment; collectors and canopy heat gain; passive (non-electric) conspicuity.',
    'Honest limits shown in the suites: the heat-driven backup cannot give an hour of driving with demonstrated materials (the latent-heat store is an optional research setting), public-road use is daylight-only with passive retroreflective conspicuity (no lamps, per the specification), and the brush-seal light-wind gain needs a wind-tunnel check.',
  ] },
  { id: 6, date: '2026-10-19', title: 'Visible wing evolution, reset controls & update notices', items: [
    'Suite 08: new 3D fluid–structure playback — the gust-loaded wing visibly bends and twists (labelled magnification with true-scale values), colours change over the gust, flow particles stream past and the HUD shows the solved values live.',
    'Every suite has ⟲ Reset (inputs, locks, results, experiments), “Reset & run” and an optional reset before each run; 3D timelines have ⏮ reset.',
    'This update pop-up and a “What’s new” summary after every update.',
    'Design Revision 2.2 (lower, more stable automobile with reefable wings, forward cockpit, mast-step bulkheads and keel beam) is rolling out across all suites; the full update follows once every suite passes verification.',
  ] },
  { id: 5, date: '2026-10-19', title: '3D dynamic simulation engine', items: ['Play/pause/scrub/speed/record timeline, articulated vehicle with spinning & steering wheels, morphing wings, force arrows, wind particles and HUD.', 'Live wind-propulsion start on the console page.'] },
  { id: 4, date: '2026-10-19', title: 'All 26 suites online', items: ['CAD & meshing, CFD, V&V, metamaterials, aeroelastic wing, FSI, chassis FEA, thermal chain, hub drive, vehicle dynamics, energy, acoustics, manufacturing, safety, optimisation and economics.'] },
];
export const LATEST = CHANGELOG[0].id;
