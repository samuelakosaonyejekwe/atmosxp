// Release notes shown in the update / "What's new" pop-up. Newest first; `id` must increase.
export const CHANGELOG = [
  { id: 7, date: '2026-10-20', title: 'Revision 5.1 — the new PHOENIX-ATMOS-XP automobile', items: [
    'One closed, splayed arch wing on a single compliant titanium pylon that self-trims to the wind by bending — no bearing, hinge, drive or lock — bolted on so it can be removed for garage storage. Lifting body 3.5 m long, 1.94 m wide, 2.975 m tall (the lowest of every revision), 184 kg.',
    'Faster than every earlier revision and the original at every wind: 54 / 70 / 78 / 88 / 100 / 112 km/h at 2 / 2.5 / 3 / 4 / 6 / 10 m/s (same models for every design; a deck-level root plate gives the light-wind lead without relying on any seal).',
    'Most stable of all: stability factor 1.63, upright to 18.5 m/s of side wind with the arch held and 26.5 m/s with passive washout, nose-over limit 1.63 g with a front-brake limiter, parked yaw limiter on the parking lever.',
    'Lowest drag (C_dA 0.109 m²) and power (52 W at 20 km/h), lightest, cheapest to build and own, 20 g CO₂e/km over its life, fewest moving parts (8 mechanisms), 39 wind-free starts per sunny day; titanium bistable launch buffer recharged by solar heat and by braking.',
    'Every part checked and modelled: tyres, rims, axle, pylon, bolted root joint, root plate, fork, brakes, crash cone, sills, harness, buffer containment, collectors and canopy; 3MF and IGES exports and 4K images added.',
    'Honest limits stated in the suites: width (1.94 m vs the original 1.65 m), share of trips possible on wind alone (63 % vs 68 % for the 4 m-tall rigs), no hour-long backup with demonstrated materials, and daylight-only public-road use with passive retroreflective conspicuity (no lamps, per the specification).',
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
