# PHOENIX-ATMOS-XP — design revision record

> **Current design: Revision 5.1** (see the last section). Revisions 2–5 below are kept as the historical record; their free-feathering pivots, ring track, reefing and folding mechanisms are **superseded** and are non-compliant with the specification's rule of no rotating mechanical assemblies other than the road wheels. They remain selectable in the suites only for like-for-like comparison.

## Revision 2 (historical)

Revision 1 (the specification's starting model: 2 × 3 m wings, 8 m², 1.5 m track) failed the software's own
checks: overall height 4.07 m, CG 1.03 m, static stability factor 0.34 g, stationary tip-over at 8.6 m/s,
wing too stiff to morph, NiTi transformation temperatures unmatched to the solved interface temperatures, a 214 kg
energy buffer, 1.9 GPa bearing stress and 0.5-year actuator life. Revision 2 resolves every issue that design
can resolve and states plainly the ones that only the environment can (night-time thermal power).

| # | Area | Decision |
|---|---|---|
| D1 | Geometry (Rev 2.2) | Body 4.4 × 1.6 × 1.0 m, wheelbase 2.8 m, track 1.8 m (tadpole), wheel R 0.30 m. Cockpit forward between the front wheels (clear forward view); two tandem wings aft of it with leading edges forward, pivot at 22 % chord, chord ≈ 0.85–0.9 m, gap ≥ 0.3 chord. **Two-stage reefable wings**: 2.0 m fixed lower section + 1.0 m telescoping upper section → 5.0 m² and 4.0 m overall deployed (light wind, open road), 3.3 m² and ≈ 3.0 m reefed (transit, strong wind), ≈ 1.2 m tilted down for parking. Mast-step bulkheads and a keel beam carry root loads to the wheel hard points. Solar collector on the unshaded nose deck, radiator on the rear deck. |
| D2 | Mass & CG | Wing structure ≤ 4 kg/m² (0.6 mm skin, polypropylene chiral lattice core at 3 % relative density, CFRP spar sized for SF ≈ 2.5). Recumbent occupant CG at R + 0.22 m. Mass biased forward (≥ 60 % on the twin front wheels). Computed mass is used globally. |
| D3 | Gust & rollover protection | Free-feathering wing root: pivot at 20–22 % chord (ahead of the aerodynamic centre), mechanical incidence lock with a torque-limited breakaway set at the safe side force F_y,safe = min(0.7 μmg, 0.8·SSF·mg·h_cg/h_CE). Beyond release the wing weathervanes (C_L ≈ 0, C_D ≈ 0.02). Rollover checks above release use feathered loads; the parked/stationary car must survive the 20 m/s design gust with wings feathered or stowed. Disclosed as an additional mechanical moving part. |
| D4 | Aeroelastic tailoring | Spar laminate with off-axis plies for bend–twist coupling: tip washout −2° to −4° at 10 m/s apparent wind, flutter speed ≥ 1.3 × 25 m/s, strength SF ≥ 2. |
| D5 | Thermal chain | Glazed solar-selective collector (low-iron glazing, selective absorber, insulated back) operated at its exergy-optimal temperature. NiTi transformation temperatures matched to the solved interface temperatures (A_f ≤ T_hot − 3 K, M_f ≥ T_cold + 3 K where physically possible); low-hysteresis NiTiCu as default; element thickness set by the cycle-frequency requirement. |
| D6 | Actuator durability | Strain amplitude 1.0 % (fatigue target ≥ 10⁷ cycles); cartridges modular and replaceable with a stated inspection interval. |
| D7 | Hub drive | Cam phasing and hot-sector angle optimised automatically for maximum net work per revolution; start-up from the elastic buffer; never positive work without a sufficient ΔT. |
| D8 | Energy buffer | Bistable cell geometry optimised to ≥ 25 % of the material resilience limit; 10 kJ usable for start-up and short gaps (≈ 20–25 kg). |
| D9 | Wheels | Bearing size selected for Hertzian contact pressure ≤ 1.5 GPa. |
| D10 | Numerics | Chassis FEA default mesh refined until GCI < 5 %. |

**Hard physical limit (not removable by design):** a 5 K night-time air–sky gradient gives at most ≈ 0.44 W/m² of
mechanical work. Night-time propulsion therefore relies on wind or finite stored energy, and the software reports this.

## Head-to-head check (no lost advantages)

| Design | Height in use | SSF | Start thrust 4 / 6 m/s | Safe max speed at 3 / 4 / 6 / 10 m/s |
|---|---|---|---|---|
| Rev 1 (specification starting model) | 4.04 m always | 0.73 | 44 / 99 N | 30 / 47 / 60 / 76 km/h |
| Rev 2.1 fixed 2 m wings | 3.0 m | 0.98 | 25 / 55 N | 0 / 20 / 48 / 87 km/h |
| **Rev 2.2 reefable (adopted)** | 4.0 deployed · 3.0 reefed · 1.2 parked | 0.90 | 43 / 97 N | 31 / 50 / 70 / 89 km/h |

Rev 2.2 equals the original in light-wind starting thrust, is faster at every wind speed, more stable, never taller,
and lower when reefed or parked. The only added item is a manually cranked telescoping wing section (≈ 3.5 kg, disclosed
as a moving part).

## Revision 4 — AeroArch (owner's choice, replaces separate wings)

One closed **box wing** shaped as a swept arch over the forward cockpit: two swept (20°), tapered (tip 0.67 × root),
inward-canted (12°) airfoil legs 1.75 m tall rising from the body shoulders and joined by a gently cambered top bridge.
Leg area 4.0 m² plus the bridge. Overall height ≈ 3.0 m; the arch folds back over the body for parking (≈ 1.3 m).

* **Aerodynamics** — the closed ring has no free wing tips; with the body acting as a ground plane under the leg roots, the
  system behaves like a Prandtl box wing (induced-drag factor k ≈ (1 + 0.45 g/b)/(1.04 + 2.81 g/b) referred to the mirrored span),
  raising the effective aspect ratio and lift slope well above separate wings of the same area. The two legs act as a
  laterally separated biplane in crosswind (mutual interference included). Camber morphing on both legs; lifting body.
* **Incidence, feathering, gust breakaway** — the whole arch rotates on a ring track around the cockpit (one mechanism):
  incidence for any wind direction, torque-limited breakaway to the feathered position in gusts.
* **Structure** — the closed frame is far stiffer and lighter than cantilever wings for the same load and doubles as a
  roll-over protection hoop; fore/aft ring-track bulkheads and the keel beam carry its loads.
* **Thermal** — the bridge top is a sky-facing solar-selective collector (≈ 1 m² instead of 0.65 m² on the nose); the rear deck
  carries the radiative cooler; the bridge also shades the cockpit.

## Revision 5 — AeroArch with compliant flexure root (owner: no rotating mechanism, shorter, more stable, faster)

* **No rotating mechanism.** The ring track, incidence drive and breakaway lock are removed. Each arch leg root is a
  **compliant architected-lattice flexure** (titanium β-21S or CFRP leaf-spring metamaterial, no bearings, no joints) that
  lets the whole arch twist elastically within about ±25° about a vertical axis ahead of its aerodynamic centre. The
  aerodynamic moment self-trims the arch to a near-constant angle of attack, and a **bistable camber skin** snaps to the
  leeward side on each tack. This is elastic deformation of the structure, the same principle as the morphing skins.
* **Passive gust protection.** Above the design load the flexure's stiffening curve and the skin's de-cambering shed lift
  (passive washout); no release mechanism is needed. The flexure must meet a fatigue life ≥ 10⁷ load cycles.
* **Wind-direction envelope.** With incidence limited to ±25° relative to the body, thrust is available when the apparent
  wind arrives within the corresponding sector; outside it the driver chooses a heading that brings the apparent wind into
  the sector (sailing practice). The envelope, and any heading penalty, is reported.
* **Targets.** Length as short as possible toward the original 3.6 m; static stability factor ≥ 1.11 g; safe sustainable
  speed ≥ Revision 3 tandem with the same low-loss tyres and computed drag; overall height ≤ 3.0 m; mass ≤ 260 kg.

### Revision 5 acceptance criterion (owner): equal or better than EVERY previous revision and the original in EVERY measured respect
Compared respects: overall height (≤ 3.0 m), length (≤ 3.6 m, the original), mass (≤ 258 kg, AeroArch 4.1), static
stability factor (≥ 1.11 g, Rev 3), tip-over wind loaded (≥ 17 m/s, Rev 3), safe sustainable speed at 2.5/3/4/6/10 m/s
(≥ the best of all revisions at each wind, computed with identical tyre/drag assumptions), wheel power at 20 km/h
(≤ 75 W), moving parts (≤ original: none beyond wheels/steering/brakes — flexure deformation only), structural safety
factors (≥ 2), cost and life-cycle carbon (≤ best previous). If a respect is physically unattainable together with the
others, the optimiser must report it explicitly with the size of the shortfall and the binding constraint.
