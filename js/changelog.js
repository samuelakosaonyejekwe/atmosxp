// Release notes shown in the update / "What's new" pop-up. Newest first; `id` must increase.
export const CHANGELOG = [
  { id: 7, date: '2026-10-21', title: 'Revision 5.1 — the new PHOENIX-ATMOS-XP automobile', items: [
    'One closed, splayed arch wing on a single compliant titanium pylon that self-trims to the wind by bending — no bearing, hinge, drive or lock — bolted on so it can be removed for garage storage or storms. Lifting body 3.5 m long, 1.94 m wide, 2.975 m tall (the lowest of every revision), 188.9 kg.',
    'Predicted by the suites (simulation, not yet validated by test): faster than every earlier revision and the original at every wind from 2 to 15 m/s: 52 / 68 / 75 / 87 / 104 / 118 / 114 / 107 km/h at 2 / 2.5 / 3 / 4 / 6 / 10 / 12 / 15 m/s (same models and the same safety rule for every design).',
    'Safety first (predicted, to be confirmed by the s24 test plan): the arch side force is capped so the car still corners at 0.5 g with the arch fully loaded (worst-case centre of effort over 2 800 operating states + 5 %); one driver lever with a single 3-position gate — drive / depower / park — unloads the arch through one closed-loop strand to the kirigami relief panels, and the parking brake (held by the same strand) and the forked park wedge can only be set with the arch depowered.',
    'Predicted (simulation): lowest drag (C_dA 0.111 m²) and power (52 W at 20 km/h), lightest, cheapest to build and own, 20.8 g CO₂e/km over its life, fewest moving parts under one counting rule for every design (9 mechanisms / 165 parts vs 10 / 166 for the next best), most wind-free starts per sunny day (23 vs 21 for the next best, corrected hub efficiency); the hub stays engaged on every design (no sail), its small night drag included; stowed (arch removed) it stays upright to ≈ 44.8 m/s.',
    'Honest limits stated in the suites: width (1.94 m vs the original 1.65 m), share of trips possible on wind alone (64 % vs 68 % for the 4 m-tall rigs), arch-up parked tip-over 24.8 m/s (remove the arch for storms), no hour-long backup with demonstrated materials (the latent-heat store becomes viable only if a measured converter efficiency of ≈26 % of Carnot is imported), and daylight-only public-road use with passive retroreflective conspicuity — no lamps of any kind, per the specification.',
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
