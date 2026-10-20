// Engineering materials registry. Values are typical room-temperature handbook/datasheet figures
// (ASM Handbooks, MMPDS, manufacturer technical datasheets, peer-reviewed SMA literature). Each record
// carries an uncertainty band and a source note; values must be replaced by supplier-certified or
// measured data for validated claims (provenance is shown wherever a property is used).
//
// Units: rho kg/m³ · E,G GPa · strengths MPa · k W/(m·K) · cp J/(kg·K) · alpha µm/(m·K) · T °C ·
//        cost USD/kg · co2 kg CO2e/kg · energy MJ/kg (cradle-to-gate, primary production).

const HB = 'Typical handbook value (ASM Handbook / MMPDS / supplier datasheets)';
const LIT = 'Peer-reviewed literature range';
const M = [];
const add = (o) => M.push({ unc: 10, src: HB, recyclable: 0.5, processes: [], uses: [], ...o });

// ---------- Fibre-reinforced polymer composites (lamina properties, Vf ≈ 0.6) ----------
add({ id: 'cfrp-t700', name: 'CFRP UD — T700S/epoxy', family: 'Composite', cls: 'cfrp', rho: 1550, E: 135, E2: 9.5, G12: 4.8, nu: 0.3, Xt: 2550, Xc: 1470, Yt: 69, Yc: 200, S12: 100, sy: 2550, su: 2550, fatigue: 0.6, k: 7, k2: 0.8, cp: 900, alpha: 0.2, alpha2: 30, Tmax: 120, Tmin: -55, cost: 38, co2: 24, energy: 290, recyclable: 0.25, unc: 8, processes: ['Prepreg autoclave', 'Out-of-autoclave prepreg', 'AFP / ATL'], uses: ['chassis', 'spar', 'wingSkin', 'leadingEdge', 'wheel', 'rib'], note: 'Standard-modulus carbon. Fatigue ratio = fraction of UTS at 10⁷ cycles (tension–tension).' });
add({ id: 'cfrp-im7', name: 'CFRP UD — IM7/8552', family: 'Composite', cls: 'cfrp', rho: 1570, E: 161, E2: 11.4, G12: 5.2, nu: 0.32, Xt: 2560, Xc: 1590, Yt: 64, Yc: 285, S12: 91, sy: 2560, su: 2560, fatigue: 0.6, k: 6, k2: 0.7, cp: 900, alpha: -0.1, alpha2: 32, Tmax: 121, Tmin: -55, cost: 85, co2: 30, energy: 330, recyclable: 0.25, unc: 6, src: 'Supplier datasheet / NCAMP qualification values (mean)', processes: ['Autoclave prepreg'], uses: ['spar', 'chassis'] });
add({ id: 'cfrp-woven', name: 'CFRP woven 0/90 fabric/epoxy', family: 'Composite', cls: 'cfrp', rho: 1550, E: 70, E2: 70, G12: 5, nu: 0.05, Xt: 600, Xc: 570, Yt: 600, Yc: 570, S12: 90, sy: 600, su: 600, fatigue: 0.5, k: 4, cp: 900, alpha: 2.5, Tmax: 120, Tmin: -55, cost: 45, co2: 24, energy: 290, recyclable: 0.25, processes: ['RTM', 'Vacuum infusion', 'Prepreg'], uses: ['chassis', 'wingSkin', 'leadingEdge', 'cabin'] });
add({ id: 'gfrp-e', name: 'GFRP UD — E-glass/epoxy', family: 'Composite', cls: 'gfrp', rho: 2000, E: 39, E2: 8.6, G12: 3.8, nu: 0.28, Xt: 1080, Xc: 620, Yt: 39, Yc: 128, S12: 89, sy: 1080, su: 1080, fatigue: 0.3, k: 1.1, cp: 1000, alpha: 7, alpha2: 22, Tmax: 120, Tmin: -55, cost: 6, co2: 8, energy: 110, recyclable: 0.2, processes: ['Hand lay-up', 'Vacuum infusion', 'Filament winding'], uses: ['chassis', 'spar', 'rib', 'wingSkin'] });
add({ id: 'hybrid-cg', name: 'Hybrid carbon/glass UD (50/50)', family: 'Composite', cls: 'cfrp', rho: 1780, E: 85, E2: 9, G12: 4.3, nu: 0.29, Xt: 1700, Xc: 1000, Yt: 55, Yc: 160, S12: 95, sy: 1700, su: 1700, fatigue: 0.45, k: 4, cp: 950, alpha: 3, Tmax: 120, Tmin: -55, cost: 20, co2: 16, energy: 200, recyclable: 0.22, unc: 15, processes: ['Prepreg', 'Infusion'], uses: ['spar', 'chassis'] });

// ---------- Sandwich cores ----------
add({ id: 'core-pet', name: 'PET structural foam 100 kg/m³', family: 'Core', cls: 'core', rho: 100, E: 0.08, G12: 0.025, nu: 0.3, sy: 1.4, su: 1.4, k: 0.035, cp: 1200, alpha: 50, Tmax: 100, Tmin: -60, cost: 18, co2: 3.5, energy: 80, recyclable: 0.6, processes: ['Extruded foam (recycled PET available)'], uses: ['chassis', 'wingSkin'] });
add({ id: 'core-alhc', name: 'Aluminium honeycomb 5052 72 kg/m³', family: 'Core', cls: 'core', rho: 72, E: 1.0, G12: 0.31, nu: 0.3, sy: 3.6, su: 3.6, k: 2, cp: 900, alpha: 23, Tmax: 175, Tmin: -55, cost: 70, co2: 12, energy: 200, recyclable: 0.8, processes: ['Expanded honeycomb'], uses: ['chassis'] });

// ---------- Light metals ----------
add({ id: 'al-6061-t6', name: 'Aluminium 6061-T6', family: 'Metal', cls: 'al', rho: 2700, E: 68.9, nu: 0.33, sy: 276, su: 310, fatigue: 97, k: 167, cp: 896, alpha: 23.6, Tmax: 150, Tmin: -200, cost: 3.5, co2: 9.0, energy: 155, recyclable: 0.9, unc: 5, processes: ['Extrusion', 'CNC machining', 'TIG/MIG welding'], uses: ['chassis', 'wheel', 'spreader', 'rootInsert', 'cam'] , basquin: { sf: 520, b: -0.085 } });
add({ id: 'al-7075-t6', name: 'Aluminium 7075-T6', family: 'Metal', cls: 'al', rho: 2810, E: 71.7, nu: 0.33, sy: 503, su: 572, fatigue: 159, k: 130, cp: 960, alpha: 23.4, Tmax: 120, Tmin: -200, cost: 6, co2: 9.6, energy: 165, recyclable: 0.9, unc: 5, src: 'MMPDS-typical', processes: ['Forging', 'CNC machining'], uses: ['wheel', 'rootInsert', 'chassis', 'cam'], basquin: { sf: 1100, b: -0.11 } });
add({ id: 'al-1050', name: 'Aluminium 1050 (high conductivity)', family: 'Metal', cls: 'al', rho: 2705, E: 69, nu: 0.33, sy: 35, su: 85, fatigue: 30, k: 229, cp: 900, alpha: 24, Tmax: 200, Tmin: -200, cost: 3, co2: 8.5, energy: 150, recyclable: 0.95, processes: ['Rolling', 'Stamping'], uses: ['spreader'] });
add({ id: 'mg-az31b', name: 'Magnesium AZ31B', family: 'Metal', cls: 'mg', rho: 1770, E: 45, nu: 0.35, sy: 200, su: 260, fatigue: 90, k: 96, cp: 1000, alpha: 26, Tmax: 120, Tmin: -100, cost: 5, co2: 25, energy: 300, recyclable: 0.6, unc: 8, processes: ['Extrusion', 'Sheet forming'], uses: ['chassis', 'wheel'], note: 'Galvanic corrosion with CFRP must be isolated.' });
add({ id: 'ti-6al4v', name: 'Titanium Ti-6Al-4V (Grade 5)', family: 'Metal', cls: 'ti', rho: 4430, E: 113.8, nu: 0.342, sy: 880, su: 950, fatigue: 510, k: 6.7, cp: 526, alpha: 8.6, Tmax: 400, Tmin: -250, cost: 30, co2: 40, energy: 650, recyclable: 0.6, unc: 5, processes: ['Forging', 'CNC machining', 'L-PBF additive', 'EBM additive'], uses: ['rootInsert', 'spar', 'rib', 'buffer', 'lattice', 'cam', 'cartridgeHousing'], basquin: { sf: 1700, b: -0.095 } });
add({ id: 'ti-gr2', name: 'Titanium CP Grade 2', family: 'Metal', cls: 'ti', rho: 4510, E: 105, nu: 0.37, sy: 275, su: 345, fatigue: 200, k: 16.4, cp: 523, alpha: 8.6, Tmax: 300, Tmin: -250, cost: 22, co2: 38, energy: 600, recyclable: 0.6, processes: ['Sheet', 'Machining'], uses: ['rib', 'buffer'] });
add({ id: 'ti-beta21s', name: 'Titanium β-21S (spring temper)', family: 'Metal', cls: 'ti', rho: 4940, E: 105, nu: 0.33, sy: 1300, su: 1400, fatigue: 600, k: 7.6, cp: 500, alpha: 9.7, Tmax: 300, Tmin: -100, cost: 45, co2: 42, energy: 680, recyclable: 0.5, unc: 10, processes: ['Cold rolling', 'Ageing'], uses: ['buffer', 'flexure'], note: 'High resilience σy²/2E for elastic energy storage.' });

// ---------- Steels & bearing alloys ----------
add({ id: 'st-4130', name: 'Steel AISI 4130 (Q&T)', family: 'Metal', cls: 'steel', rho: 7850, E: 205, nu: 0.29, sy: 460, su: 670, fatigue: 300, k: 42.7, cp: 477, alpha: 12.2, Tmax: 400, Tmin: -50, cost: 2.2, co2: 2.3, energy: 30, recyclable: 0.9, unc: 5, processes: ['Tube drawing', 'TIG welding'], uses: ['chassis', 'rootInsert', 'steering'] });
add({ id: 'st-17-4', name: 'Stainless 17-4PH H1025', family: 'Metal', cls: 'steel', rho: 7800, E: 197, nu: 0.27, sy: 1000, su: 1070, fatigue: 500, k: 18.3, cp: 460, alpha: 10.8, Tmax: 315, Tmin: -100, cost: 6, co2: 4.8, energy: 60, recyclable: 0.9, processes: ['Machining', 'L-PBF additive'], uses: ['cam', 'rootInsert', 'cartridgeHousing', 'steering'] });
add({ id: 'st-spring', name: 'Spring steel 50CrV4 (SAE 6150)', family: 'Metal', cls: 'steel', rho: 7850, E: 206, nu: 0.29, sy: 1200, su: 1400, fatigue: 650, k: 46, cp: 470, alpha: 12.5, Tmax: 250, Tmin: -40, cost: 2.5, co2: 2.6, energy: 32, recyclable: 0.9, processes: ['Hot coiling', 'Q&T', 'Shot peening'], uses: ['buffer', 'flexure', 'brake'] });
add({ id: 'st-52100', name: 'Bearing steel AISI 52100', family: 'Metal', cls: 'steel', rho: 7810, E: 210, nu: 0.3, sy: 2030, su: 2240, fatigue: 900, k: 46.6, cp: 475, alpha: 11.9, Tmax: 150, Tmin: -50, cost: 4, co2: 2.8, energy: 35, recyclable: 0.9, processes: ['Through-hardening', 'Grinding'], uses: ['bearing', 'cam'] });
add({ id: 'st-440c', name: 'Stainless bearing steel 440C', family: 'Metal', cls: 'steel', rho: 7800, E: 200, nu: 0.28, sy: 1900, su: 1970, fatigue: 750, k: 24.2, cp: 460, alpha: 10.2, Tmax: 250, Tmin: -70, cost: 9, co2: 5, energy: 65, recyclable: 0.9, processes: ['Hardening', 'Grinding'], uses: ['bearing'] });
add({ id: 'ci-grey', name: 'Grey cast iron (brake disc grade)', family: 'Metal', cls: 'ci', rho: 7150, E: 110, nu: 0.26, sy: 200, su: 250, fatigue: 100, k: 50, cp: 500, alpha: 11, Tmax: 600, Tmin: -40, cost: 1.2, co2: 1.9, energy: 25, recyclable: 0.95, processes: ['Sand casting', 'Machining'], uses: ['brake'] });
add({ id: 'cu-c101', name: 'Copper C10100 (OFE)', family: 'Metal', cls: 'cu', rho: 8940, E: 115, nu: 0.34, sy: 69, su: 220, fatigue: 76, k: 391, cp: 385, alpha: 17, Tmax: 200, Tmin: -250, cost: 10, co2: 3.8, energy: 60, recyclable: 0.95, unc: 4, processes: ['Rolling', 'Machining', 'Brazing'], uses: ['spreader', 'heatpipe'] });

// ---------- Ceramics & friction materials ----------
add({ id: 'si3n4', name: 'Silicon nitride Si₃N₄ (bearing balls)', family: 'Ceramic', cls: 'ceramic', rho: 3200, E: 310, nu: 0.27, sy: 3000, su: 800, fatigue: 500, k: 30, cp: 680, alpha: 3.2, Tmax: 1000, Tmin: -200, cost: 120, co2: 12, energy: 150, recyclable: 0.1, processes: ['HIP', 'Lapping'], uses: ['bearing'] });
add({ id: 'c-sic', name: 'Carbon–ceramic C/SiC brake composite', family: 'Ceramic', cls: 'ceramic', rho: 2400, E: 70, nu: 0.15, sy: 150, su: 150, fatigue: 100, k: 30, cp: 1100, alpha: 2.6, Tmax: 1350, Tmin: -40, cost: 300, co2: 40, energy: 500, recyclable: 0.1, unc: 20, processes: ['Liquid silicon infiltration'], uses: ['brake'], mu: 0.42 });
add({ id: 'fric-organic', name: 'Low-metallic friction pad (phenolic binder, mineral and steel-fibre fillers)', family: 'Friction', cls: 'friction', rho: 2300, E: 3, nu: 0.25, sy: 30, su: 30, k: 1.2, cp: 1100, alpha: 15, Tmax: 350, Tmin: -40, cost: 8, co2: 3, energy: 40, recyclable: 0.05, unc: 25, processes: ['Hot pressing'], uses: ['brake'], mu: 0.38, wear: 2.5e-14 });
add({ id: 'fric-sintered', name: 'Sintered metallic friction pad', family: 'Friction', cls: 'friction', rho: 5500, E: 30, nu: 0.25, sy: 80, su: 80, k: 25, cp: 600, alpha: 13, Tmax: 600, Tmin: -40, cost: 20, co2: 6, energy: 80, recyclable: 0.3, unc: 25, processes: ['Powder sintering'], uses: ['brake'], mu: 0.45, wear: 1.2e-14 });

// ---------- Shape-memory alloys (thermal actuators) ----------
// Ms,Mf,As,Af °C (stress-free); CM, CA stress-influence coefficients MPa/K; EA, EM GPa; epsL max transformation strain; Lh latent heat J/kg; hyst K
add({ id: 'niti-actuator', name: 'NiTi actuator grade (Ni-50.0 at%)', family: 'SMA', cls: 'sma', rho: 6450, E: 70, EA: 70, EM: 28, nu: 0.33, sy: 560, su: 1000, fatigue: 200, k: 18, kM: 8.6, cp: 837, alpha: 11, Tmax: 150, Tmin: -100, Ms: 50, Mf: 38, As: 65, Af: 80, CM: 6.5, CA: 7.5, epsL: 0.045, Lh: 24200, hyst: 28, cost: 200, co2: 60, energy: 900, recyclable: 0.4, unc: 15, src: LIT + ' (Lagoudas 2008; Brinson 1993 parameter sets)', processes: ['Vacuum induction melting', 'Hot/cold drawing', 'Shape setting 500 °C', 'L-PBF additive (lattices)'], uses: ['actuator'], note: 'Transformation temperatures are composition- and heat-treatment-sensitive (≈ 10 K per 0.1 at% Ni).' });
add({ id: 'niti-lowT', name: 'NiTi near-ambient grade (Ni-50.8 at%)', family: 'SMA', cls: 'sma', rho: 6450, E: 65, EA: 65, EM: 30, nu: 0.33, sy: 450, su: 1000, fatigue: 220, k: 18, kM: 8.6, cp: 837, alpha: 11, Tmax: 120, Tmin: -100, Ms: 18, Mf: 8, As: 25, Af: 35, CM: 6.0, CA: 6.8, epsL: 0.04, Lh: 19000, hyst: 17, cost: 220, co2: 60, energy: 900, recyclable: 0.4, unc: 18, src: LIT, processes: ['Drawing', 'Shape setting'], uses: ['actuator'], note: 'Transformation window near ambient for low-ΔT environmental heat engines.' });
add({ id: 'nitihf', name: 'NiTiHf high-temperature SMA', family: 'SMA', cls: 'sma', rho: 8200, E: 80, EA: 80, EM: 50, nu: 0.33, sy: 900, su: 1100, fatigue: 250, k: 12, cp: 450, alpha: 11, Tmax: 300, Tmin: -40, Ms: 120, Mf: 105, As: 130, Af: 150, CM: 13, CA: 13.5, epsL: 0.025, Lh: 21000, hyst: 30, cost: 900, co2: 120, energy: 1500, recyclable: 0.3, unc: 20, src: LIT, processes: ['VIM', 'Hot extrusion'], uses: ['actuator'] });
add({ id: 'nitcu', name: 'NiTiCu low-hysteresis SMA (Ti-50Ni-5Cu-ish)', family: 'SMA', cls: 'sma', rho: 6500, E: 60, EA: 60, EM: 35, nu: 0.33, sy: 450, su: 800, fatigue: 260, k: 17, cp: 840, alpha: 11, Tmax: 120, Tmin: -40, Ms: 40, Mf: 33, As: 45, Af: 52, CM: 6.2, CA: 6.6, epsL: 0.035, Lh: 15000, hyst: 9, cost: 260, co2: 62, energy: 920, recyclable: 0.4, unc: 20, src: LIT, processes: ['Melt-spinning', 'Drawing'], uses: ['actuator'], note: 'Narrow hysteresis improves cycle efficiency and functional fatigue.' });
add({ id: 'cualni', name: 'Cu-Al-Ni single-crystal SMA', family: 'SMA', cls: 'sma', rho: 7120, E: 85, EA: 85, EM: 80, nu: 0.34, sy: 400, su: 600, fatigue: 120, k: 75, cp: 440, alpha: 17, Tmax: 200, Tmin: -40, Ms: 60, Mf: 50, As: 70, Af: 82, CM: 2.5, CA: 2.5, epsL: 0.04, Lh: 9000, hyst: 20, cost: 25, co2: 6, energy: 80, recyclable: 0.8, unc: 25, src: LIT, processes: ['Bridgman crystal growth'], uses: ['actuator'], note: 'Brittle in polycrystalline form; high thermal conductivity aids fast cycling.' });
add({ id: 'femnsi', name: 'Fe-Mn-Si-Cr SMA', family: 'SMA', cls: 'sma', rho: 7400, E: 170, EA: 170, EM: 140, nu: 0.3, sy: 450, su: 750, fatigue: 200, k: 12, cp: 470, alpha: 16, Tmax: 300, Tmin: -40, Ms: 25, Mf: 0, As: 110, Af: 200, CM: 3, CA: 3, epsL: 0.025, Lh: 4000, hyst: 120, cost: 5, co2: 3, energy: 45, recyclable: 0.9, unc: 25, src: LIT, processes: ['Rolling', 'Thermo-mechanical training'], uses: ['actuator'], note: 'Large hysteresis — suited to one-way prestressing, poor for cyclic heat engines.' });

// ---------- Thermoresponsive polymer actuators ----------
add({ id: 'tcp-pe', name: 'Twisted-coiled UHMWPE fibre muscle', family: 'Polymer actuator', cls: 'tcp', rho: 970, E: 0.4, nu: 0.4, sy: 400, su: 2500, fatigue: 100, k: 0.4, cp: 1900, alpha: -100, Tmax: 120, Tmin: -40, strainPerK: 0.0009, stressMax: 40, cost: 30, co2: 4, energy: 110, recyclable: 0.3, unc: 30, src: LIT, processes: ['Twist insertion', 'Mandrel coiling'], uses: ['actuator'] });
add({ id: 'lce', name: 'Liquid-crystal elastomer (nematic)', family: 'Polymer actuator', cls: 'lce', rho: 1100, E: 0.005, nu: 0.49, sy: 3, su: 5, k: 0.2, cp: 1800, alpha: 200, Tmax: 140, Tmin: -10, strainPerK: 0.006, stressMax: 0.5, cost: 500, co2: 10, energy: 200, recyclable: 0.1, unc: 40, src: LIT, processes: ['Two-step crosslinking', 'DIW printing'], uses: ['actuator'] });

// ---------- Polymers, elastomers, glazing ----------
add({ id: 'tpu', name: 'Thermoplastic polyurethane (TPU 90A)', family: 'Polymer', cls: 'elastomer', rho: 1200, E: 0.025, nu: 0.48, sy: 40, su: 45, fatigue: 8, k: 0.21, cp: 1800, alpha: 150, Tmax: 90, Tmin: -40, C10: 3.5, C01: 1.0, cost: 6, co2: 4.5, energy: 95, recyclable: 0.5, processes: ['Extrusion', 'FDM printing', 'Calendering'], uses: ['wingSkin'] });
add({ id: 'tpu-glass', name: 'Glass-reinforced TPU skin composite', family: 'Polymer', cls: 'elastomer', rho: 1400, E: 0.8, E2: 0.05, nu: 0.45, sy: 60, su: 70, fatigue: 15, k: 0.3, cp: 1500, alpha: 60, Tmax: 90, Tmin: -40, cost: 10, co2: 5.5, energy: 105, recyclable: 0.3, unc: 20, processes: ['Lamination'], uses: ['wingSkin'] });
add({ id: 'silicone', name: 'Silicone elastomer (Shore 40A)', family: 'Polymer', cls: 'elastomer', rho: 1150, E: 0.0015, nu: 0.49, sy: 7, su: 8, fatigue: 1.5, k: 0.2, cp: 1460, alpha: 300, Tmax: 200, Tmin: -55, C10: 0.18, C01: 0.03, cost: 12, co2: 6, energy: 120, recyclable: 0.05, processes: ['Casting', 'LSR moulding'], uses: ['wingSkin', 'seal'] });
add({ id: 'pc', name: 'Polycarbonate (glazing)', family: 'Polymer', cls: 'polymer', rho: 1200, E: 2.35, nu: 0.37, sy: 62, su: 66, fatigue: 14, k: 0.2, cp: 1250, alpha: 68, Tmax: 125, Tmin: -40, cost: 4, co2: 6, energy: 110, recyclable: 0.6, processes: ['Thermoforming', 'Injection moulding'], uses: ['cabin'], optics: { tvis: 0.88 } });
add({ id: 'glass-lam', name: 'Laminated safety glass (PVB)', family: 'Ceramic', cls: 'glass', rho: 2500, E: 70, nu: 0.22, sy: 45, su: 45, k: 1.0, cp: 840, alpha: 9, Tmax: 80, Tmin: -40, cost: 6, co2: 1.6, energy: 20, recyclable: 0.3, processes: ['Lamination', 'Autoclave'], uses: ['cabin'] });
add({ id: 'pp', name: 'Polypropylene (PP, powder-bed fused or moulded; non-absorbing)', family: 'Polymer', cls: 'polymer', rho: 900, E: 1.4, nu: 0.42, sy: 30, su: 33, fatigue: 11, k: 0.22, cp: 1900, alpha: 110, Tmax: 100, Tmin: -30, cost: 45, co2: 2.2, energy: 85, recyclable: 0.8, processes: ['MJF / SLS printing', 'Injection or bead moulding'], uses: ['rib', 'lattice'] });
add({ id: 'pei', name: 'Polyetherimide (PEI) high-temperature polymer', family: 'Polymer', cls: 'polymer', rho: 1270, E: 3.2, nu: 0.36, sy: 105, su: 110, fatigue: 30, k: 0.22, cp: 1100, alpha: 56, Tmax: 170, Tmin: -40, cost: 30, co2: 9, energy: 160, recyclable: 0.3, processes: ['Injection moulding', 'FDM'], uses: ['insulator', 'cartridgeHousing'] });
add({ id: 'tyre-nr', name: 'Low-rolling-resistance tread compound (silica/NR/SBR)', family: 'Elastomer', cls: 'rubber', rho: 1150, E: 0.006, nu: 0.499, sy: 18, su: 20, k: 0.25, cp: 1900, alpha: 200, Tmax: 100, Tmin: -40, C10: 0.8, C01: 0.2, tanDelta: 0.08, mu: 0.9, muWet: 0.6, cost: 3, co2: 3.2, energy: 75, recyclable: 0.3, unc: 20, processes: ['Mixing', 'Calendering', 'Vulcanisation'], uses: ['tyre'] });

// ---------- Thermal management & insulation ----------
add({ id: 'pgs', name: 'Pyrolytic graphite sheet', family: 'Thermal', cls: 'graphite', rho: 2100, E: 20, nu: 0.2, sy: 20, su: 20, k: 1500, k2: 10, cp: 850, alpha: 0.5, Tmax: 400, Tmin: -200, cost: 400, co2: 30, energy: 400, recyclable: 0.1, unc: 15, processes: ['Lamination'], uses: ['spreader'], note: 'In-plane k ≈ 700–1950 W/(m·K) depending on thickness; through-plane ≈ 5–15.' });
add({ id: 'aln', name: 'Aluminium nitride (thermal interface ceramic)', family: 'Ceramic', cls: 'ceramic', rho: 3260, E: 330, nu: 0.24, sy: 300, su: 300, k: 170, cp: 740, alpha: 4.5, Tmax: 1000, Tmin: -200, cost: 150, co2: 15, energy: 200, recyclable: 0.05, processes: ['Tape casting', 'Sintering'], uses: ['spreader', 'insulator'] });
add({ id: 'aerogel', name: 'Hydrophobic silica aerogel blanket', family: 'Thermal', cls: 'insul', rho: 150, E: 0.001, nu: 0.2, sy: 0.1, su: 0.1, k: 0.015, cp: 1000, alpha: 3, Tmax: 650, Tmin: -200, cost: 60, co2: 10, energy: 150, recyclable: 0.05, processes: ['Supercritical drying'], uses: ['insulator'] });
add({ id: 'heatpipe-cuw', name: 'Copper/water sintered-wick heat pipe (effective)', family: 'Thermal', cls: 'heatpipe', rho: 4500, E: 115, nu: 0.34, sy: 69, su: 220, k: 10000, cp: 400, alpha: 17, Tmax: 150, Tmin: 1, cost: 40, co2: 5, energy: 80, recyclable: 0.8, unc: 40, src: 'Effective conductivity range from vendor data (depends on length, orientation & heat load)', processes: ['Sintering', 'Evacuation & charging'], uses: ['heatpipe'], hp: { fluid: 'water', Qmax_W: 60, dia_mm: 8, Tmin_op: 5 } });

// ---------- Optical / radiative surfaces (constant-band representation for screening; spectral data in Suite 13) ----------
add({ id: 'coat-cermet', name: 'Solar-selective cermet absorber (Cr–Cr₂O₃ type)', family: 'Coating', cls: 'coating', rho: 6000, k: 20, cp: 500, Tmax: 400, Tmin: -40, optics: { aSolar: 0.94, eIR: 0.08, eWindow: 0.08 }, cost: 400, co2: 20, energy: 300, recyclable: 0.1, unc: 5, src: LIT, processes: ['Sputtering', 'Electroplating'], uses: ['collector'] });
add({ id: 'coat-tinox', name: 'Titanium-oxynitride-family selective absorber on Al', family: 'Coating', cls: 'coating', rho: 2700, k: 150, cp: 900, Tmax: 300, Tmin: -40, optics: { aSolar: 0.95, eIR: 0.04, eWindow: 0.04 }, cost: 75, co2: 15, energy: 250, recyclable: 0.6, unc: 50, src: 'Optics: manufacturer datasheet typical. Cost (Rev 6.2): bottom-up estimate for roll-coated selective-absorber aluminium sheet, ≈ $60/m² at 0.3 mm (0.81 kg/m²) ≈ $75/kg including the coating — aluminium coil ≈ $4/kg plus vacuum roll-coating, and a low-cost coil-coated alternative exists; no published bulk price was found, so the ±50 % band stands until a supplier quote replaces it (former value $250/kg ≈ $200/m² had no stated basis)', processes: ['PVD roll coating'], uses: ['collector'] });
add({ id: 'coat-blackpaint', name: 'Black paint (non-selective)', family: 'Coating', cls: 'coating', rho: 1500, k: 0.5, cp: 1200, Tmax: 150, Tmin: -40, optics: { aSolar: 0.95, eIR: 0.92, eWindow: 0.92 }, cost: 20, co2: 4, energy: 70, recyclable: 0.1, processes: ['Spray'], uses: ['collector'] });
add({ id: 'coat-pdrc', name: 'Daytime radiative-cooling photonic film (polymer/dielectric)', family: 'Coating', cls: 'coating', rho: 1200, k: 0.2, cp: 1300, Tmax: 120, Tmin: -40, optics: { aSolar: 0.04, eIR: 0.93, eWindow: 0.95 }, cost: 60, co2: 8, energy: 140, recyclable: 0.2, unc: 10, src: LIT, processes: ['Phase-inversion coating', 'Multilayer sputtering'], uses: ['radiator'] });
add({ id: 'coat-whitepaint', name: 'TiO₂ white paint', family: 'Coating', cls: 'coating', rho: 1600, k: 0.4, cp: 1100, Tmax: 150, Tmin: -40, optics: { aSolar: 0.18, eIR: 0.90, eWindow: 0.90 }, cost: 15, co2: 4, energy: 70, recyclable: 0.1, processes: ['Spray'], uses: ['radiator'] });
add({ id: 'coat-baso4', name: 'BaSO₄ nanoparticle ultra-white paint', family: 'Coating', cls: 'coating', rho: 2500, k: 0.6, cp: 500, Tmax: 150, Tmin: -40, optics: { aSolar: 0.025, eIR: 0.95, eWindow: 0.96 }, cost: 40, co2: 3, energy: 60, recyclable: 0.1, unc: 10, src: LIT, processes: ['Spray'], uses: ['radiator'] });

export const MATERIALS = M;
export const byId = Object.fromEntries(M.map(m => [m.id, m]));
export const get = (id) => byId[id] || null;
export const forUse = (use) => M.filter(m => m.uses.includes(use));
export const families = () => [...new Set(M.map(m => m.family))];

// Component catalogue: every part of the automobile with default material and alternatives (used by the dossier, CAD and console)
export const COMPONENTS = {
  chassis: { name: 'Monocoque chassis (sandwich)', use: 'chassis', def: 'cfrp-woven', suite: 's09' },
  chassisCore: { name: 'Monocoque sandwich core', use: 'chassis', def: 'core-pet', suite: 's09' },
  spar: { name: 'Wing main box spar', use: 'spar', def: 'cfrp-t700', suite: 's06' },
  rib: { name: 'Flexible wing ribs', use: 'rib', def: 'gfrp-e', suite: 's06' },
  wingSkin: { name: 'Morphing aerodynamic skin', use: 'wingSkin', def: 'tpu-glass', suite: 's06' },
  leadingEdge: { name: 'Wing leading-edge shell', use: 'leadingEdge', def: 'cfrp-woven', suite: 's06' },
  lattice: { name: 'Wing metamaterial cellular core (constituent)', use: 'lattice', def: 'pp', suite: 's04' },
  rootInsert: { name: 'Wing-root metallic inserts', use: 'rootInsert', def: 'ti-6al4v', suite: 's09' },
  collector: { name: 'Day solar-selective collector', use: 'collector', def: 'coat-tinox', suite: 's13' },
  radiator: { name: 'Night radiative-cooling surface', use: 'radiator', def: 'coat-pdrc', suite: 's13' },
  spreader: { name: 'Heat spreaders / thermal pathways', use: 'spreader', def: 'al-1050', suite: 's14' },
  heatpipe: { name: 'Heat pipes', use: 'heatpipe', def: 'heatpipe-cuw', suite: 's14' },
  insulator: { name: 'Thermal insulation / isolation', use: 'insulator', def: 'aerogel', suite: 's14' },
  actuator: { name: 'Thermal actuator cartridges (active)', use: 'actuator', def: 'nitcu', suite: 's15' },
  cartridgeHousing: { name: 'Cartridge housing / containment ring', use: 'cartridgeHousing', def: 'ti-6al4v', suite: 's17' },
  cam: { name: 'Stationary reaction cam', use: 'cam', def: 'st-17-4', suite: 's17' },
  flexure: { name: 'Compliant force-transfer shoes / flexures', use: 'flexure', def: 'ti-beta21s', suite: 's17' },
  buffer: { name: 'Bistable energy-buffer cells', use: 'buffer', def: 'ti-beta21s', suite: 's18' },
  wheel: { name: 'Road wheel rim', use: 'wheel', def: 'al-7075-t6', suite: 's10' },
  tyre: { name: 'Tyre tread compound', use: 'tyre', def: 'tyre-nr', suite: 's10' },
  bearing: { name: 'Wheel bearings', use: 'bearing', def: 'st-52100', suite: 's10' },
  brake: { name: 'Brake disc', use: 'brake', def: 'ci-grey', suite: 's12' },
  brakePad: { name: 'Brake pads', use: 'brake', def: 'fric-organic', suite: 's12' },
  steering: { name: 'Steering linkage', use: 'steering', def: 'st-4130', suite: 's11' },
  cabin: { name: 'Cabin canopy / glazing', use: 'cabin', def: 'pc', suite: 's01' },
};
export function componentMaterial(store, key) { const c = COMPONENTS[key]; return get(store.material(key, c?.def)) || get(c?.def); }

// Derived performance indices (Ashby)
export const indices = {
  stiffBeam: (m) => (m.E ?? 0) ** 0.5 / m.rho * 1e3,       // E^1/2/ρ — light stiff beam
  strongBeam: (m) => (m.sy ?? m.su ?? 0) ** (2 / 3) / m.rho * 1e3,
  stiffPanel: (m) => (m.E ?? 0) ** (1 / 3) / m.rho * 1e3,
  resilience: (m) => (m.sy ?? 0) ** 2 / (2 * (m.E || 1)),            // σy²/2E: MPa²/GPa = kJ/m³
  specificResilience: (m) => (m.sy ?? 0) ** 2 / (2 * (m.E || 1)) * 1e3 / m.rho, // J/kg
  thermalDiffusivity: (m) => m.k / (m.rho * m.cp),
  carbonPerStiffness: (m) => m.co2 * m.rho / Math.max(1e-6, (m.E ?? 0) ** 0.5),
};
