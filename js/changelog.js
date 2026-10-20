// Release notes shown in the update / "What's new" pop-up. Newest first; `id` must increase.
export const CHANGELOG = [
  { id: 7, date: '2026-10-21', title: 'Revision 5.1 — the new PHOENIX-ATMOS-XP automobile', items: [
    'One closed, splayed arch wing on a single compliant titanium pylon that self-trims to the wind by bending — no bearing, hinge, drive or lock — bolted on so it can be removed for garage storage or storms. Lifting body 3.5 m long, 1.94 m wide, 2.975 m tall (the lowest of every revision), 185 kg.',
    'Faster than every earlier revision and the original at every wind from 2 to 15 m/s: 54 / 68 / 75 / 86 / 104 / 104 / 103 / 74 km/h at 2 / 2.5 / 3 / 4 / 6 / 10 / 12 / 15 m/s (same models and the same safety rule for every design).',
    'Safety first: the arch side force is capped so the car still corners at 0.5 g with the arch fully loaded (worst-case centre of effort over 2 800 operating states + 5 %); a single drive / depower / park lever progressively unloads the arch, kirigami relief panels spill gusts, and the parking brake can only be set with the arch depowered.',
    'Lowest drag (C_dA 0.111 m²) and power (52 W at 20 km/h), lightest, cheapest to build and own, 20 g CO₂e/km over its life, fewest moving parts (8 mechanisms / 64 parts), 39 wind-free starts per sunny day; stowed (arch removed) it stays upright to 44 m/s of side wind.',
    'Honest limits stated in the suites: width (1.94 m vs the original 1.65 m), share of trips possible on wind alone (62 % vs 68 % for the 4 m-tall rigs), arch-up parked tip-over 22 m/s (remove the arch before storms), no hour-long backup with demonstrated materials, and daylight-only public-road use with passive retroreflective conspicuity (no lamps, per the specification).',
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
