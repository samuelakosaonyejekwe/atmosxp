// Release notes shown in the update / "What's new" pop-up. Newest first; `id` must increase.
export const CHANGELOG = [
  { id: 7, date: '2026-10-19', title: 'Revision 3 automobile — lower, safer, faster in moderate wind', items: [
    'Three fixed tandem 2.0 m wings on a 5.4 m lifting body: 3.0 m overall at all times (was 4.06 m), no telescoping mechanism.',
    'Far more stable: static stability 1.04 g (was 0.73) and the stationary car stays upright in a 17 m/s crosswind with wings loaded (was 9.3 m/s).',
    'Stronger than the original from ≈ 8 m/s wind upward (519 vs 380 N at 10 m/s); in very light wind it gives about half the original thrust — the unavoidable price of lower wings.',
    'Forward cockpit with clear view, airfoil-plan lifting body, wheel spats, root fairings, rounded tips and a pearl-white/graphite/cobalt livery.',
    'Every suite now has 3D and 2D dynamic simulations driven by the solved results.',
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
