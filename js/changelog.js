// Release notes shown in the update / "What's new" pop-up. Newest first; `id` must increase.
export const CHANGELOG = [
  { id: 6, date: '2026-10-19', title: 'Design Revision 2, reset controls & update notices', items: [
    'Redesigned automobile (Revision 2.2): cockpit moved forward for a clear view, wings aft with forward-facing leading edges, mast-step bulkheads and a keel beam, wider 1.8 m track and a lighter wing core for a lower centre of gravity.',
    'Two-stage reefable wings keep every advantage of the original: the same light-wind starting thrust, a higher safe speed at every wind speed and better stability — 4.0 m deployed, ≈3.0 m reefed, ≈1.2 m parked.',
    'Free-feathering wing root with gust breakaway and tilt-down stowage, so gusts can no longer overturn the car (dossier §2.11–2.12).',
    'Every suite now has ⟲ Reset (inputs, locks, results, experiments) and an optional “reset before each run”; 3D timelines have a ⏮ reset.',
    'Wing thrust is capped at the safe tyre-grip and rollover side-force limit in journey simulations.',
    'This update pop-up and a “What’s new” summary after every update.',
  ] },
  { id: 5, date: '2026-10-19', title: '3D dynamic simulation engine', items: ['Play/pause/scrub/speed/record timeline, articulated vehicle with spinning & steering wheels, morphing wings, force arrows, wind particles and HUD.', 'Live wind-propulsion start on the console page.'] },
  { id: 4, date: '2026-10-19', title: 'All 26 suites online', items: ['CAD & meshing, CFD, V&V, metamaterials, aeroelastic wing, FSI, chassis FEA, thermal chain, hub drive, vehicle dynamics, energy, acoustics, manufacturing, safety, optimisation and economics.'] },
];
export const LATEST = CHANGELOG[0].id;
