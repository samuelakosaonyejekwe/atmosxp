// Release notes shown in the update / "What's new" pop-up. Newest first; `id` must increase.
export const CHANGELOG = [
  { id: 7, date: '2026-10-19', title: 'AeroArch — the new PHOENIX-ATMOS-XP automobile', items: [
    'One closed, swept “halo” arch wing replaces separate wings: 3.0 m tall (was 4.06 m), rising behind a forward teardrop cockpit on an airfoil-shaped lifting body with wheel spats and a pearl-white/graphite/cobalt finish.',
    'Far more robust: tip-over wind 16 m/s with the arch working and 40 m/s feathered (was 9.3 m/s); 268 kg; the arch doubles as a roll-over hoop; directionally stable.',
    'As fast as the original design from 4 m/s of wind and faster above 8 m/s (e.g. 79 vs 78 km/h at 10 m/s), within the safe tip-over and grip limits; slightly slower only in very light wind.',
    'The whole arch turns on a ring track around the cockpit for any wind direction and feathers itself in gusts.',
    'Coupled inputs now update automatically from upstream results; many hidden calculation issues fixed across the suites.',
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
