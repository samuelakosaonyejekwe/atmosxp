# PHOENIX-ATMOS-XP — Design Revision 2 (applies to every suite and mechanism)

Revision 1 (the specification's starting model: 2 × 3 m wings, 8 m², 1.5 m track) failed the software's own
checks: overall height 4.07 m, CG 1.03 m, static stability factor 0.34 g, stationary tip-over at 8.6 m/s,
wing too stiff to morph, NiTi transformation temperatures unmatched to the solved interface temperatures, a 214 kg
energy buffer, 1.9 GPa bearing stress and 0.5-year actuator life. Revision 2 resolves every issue that design
can resolve and states plainly the ones that only the environment can (night-time thermal power).

| # | Area | Decision |
|---|---|---|
| D1 | Geometry | Body 4.2 × 1.6 × 1.0 m, wheelbase 2.8 m, track 1.8 m (tadpole), wheel R 0.30 m. Two tandem wings 2.0 m span × 1.6 m chord (6.4 m²), gap ≥ 1.05 chord. Overall height ≈ 3.0 m. Wings tilt down for parking/transit (stowed height ≈ 1.2 m). |
| D2 | Mass & CG | Wing structure ≤ 4 kg/m² (0.6 mm skin, PA12 chiral lattice core at 3 % relative density, CFRP spar sized for SF ≈ 2.5). Recumbent occupant CG at R + 0.22 m. Mass biased forward (≥ 60 % on the twin front wheels). Computed mass is used globally. |
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
