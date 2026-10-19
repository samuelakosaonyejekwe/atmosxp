# Suite development guide (internal)

Each suite is an ES module `js/suites/sNN-name.js` exporting a default object. The registry
(`js/suites/index.js`) supplies `id, num, title, short, group, icon`; the module supplies the rest.

```js
export default {
  description, literature /* OpenAlex query for live best practice */, econ?, sustain?, credibility,
  equations: [...], methods: [...], assumptions: [...],
  ibc: { initial: [...], boundary: [...] },
  cvv: { calibration: [...], verification: [...], validation: [...] },
  formats: { import: [...], export: [...] },
  params: [{ key, label, unit, type: 'number'|'range'|'select'|'bool'|'text', min, max, step, options,
             default, group, help, expert?, auto?: (ctx) => value | {value, prov} }],
  presets?: [{ label, values: {...} }],
  async run(p, ctx) { return results; },
};
```

`ctx`: `store, num, physics, analysis, live, materials, vehicle, site, env, upstream(id), upstreamMeta(id),
material(componentKey), progress(frac,msg), experiments, yield()`.

Results object (all optional): `kpis[{label,value,unit,explain,prov,status:'ok'|'warn'|'bad'}]`,
`verdicts[{test,status:'pass'|'fail'|'warn'|'info',detail,fix}]`, `charts[chartSpec]` (each with `explain`),
`tables[{title,cols,rows,explain}]`, `views[{label,icon,render(el,ctx)}]`, `explain[string|{title,text}]`,
`recs[{level:'critical'|'warn'|'info'|'good'|'sustain'|'econ',text,basis}]`, `checks[{name,value,target,pass,detail}]`,
`convergence{title,series}`, `exports[{label,action}]`, `outputs{...}` (coupling), `credibility`, `headline`, `headlineKind`.

Chart specs (`js/core/chart.js`): `line|scatter|area|stack` (`x,y,series[{name,x,y,dash,mode,band}]`,
`annotations[hline|vline|band|xband|point]`), `bar` (`categories, series[{name,values}]`, `horizontal`),
`tornado` (`rows[{label,lo,hi}], base`), `heatmap` (`x.values, y.values, z[j][i], cmap, isolines, vectors, overlay, equal`),
`polar` (`sectors, bins`), `gauge` (`value,max,unit,caption`).

## Coupling contract (outputs published by each suite)

| Suite | Output keys |
|---|---|
| s01 | mass, cg[3], inertia{Ixx,Iyy,Izz}, wheelLoads[], wheelbase, track, cgHeight, ssf, massBreakdown{} |
| s02 | airfoil{name,points}, wing{span,rootChord,tipChord,area}, mesh{cells,minJ,maxSkew,maxAR,level}, gci |
| s03 | assignments{component: materialId} |
| s04 | cellType, Eeff_Pa, Geff_Pa, nuEff, rhoRel, energyDensity_J_m3, Kbt (bend–twist coupling ratio), snapForce_N |
| s05 | env: T, p, rho, RH, U, Uz, dir, gust, ghi, dni, dhi, cloud, Tsky(°C), TskyK, LW, eps, isDay, nu, mu, series[], turbulence{t,u} |
| s06 | CL, CD, liftN, dragN, thrustN, sideN, twistTip_deg, tipDefl_m, rootMoment_Nm, flutterSpeed, divergenceSpeed, f1Hz, wingMass |
| s07 | polar{alpha[],CL[],CD[]}, CLmax, alphaStall, Re, Va, awa, thrustN, sideN, yawMoment, CdA_body, thrustVsAWA{awa[],thrust[]} |
| s08 | converged, iterations, twistTip_deg, tipDefl_m, CLdeformed, flutterMargin, gustPeakRootMoment |
| s09 | maxVonMises_MPa, minSF, maxDefl_m, torsionalStiffness_Nm_deg, bucklingFactor, chassisMass, tsaiWuMax |
| s10 | Crr, rollingForce_N, muPeak, Fymax_N, bearingLoss_W, wheelInertia, contactPressure_kPa |
| s11 | understeerGradient, criticalSpeed, maxCornerSpeed_kmh, rolloverMargin, ltr, ssf |
| s12 | stoppingDistance_m, decel_ms2, brakeTempMax_C, fadeRisk, parkingHoldOK |
| s13 | qNetDay_W_m2, qNetNight_W_m2, Pabs_W, Pcool_W, Thot_C, Tcold_C, collectorArea, radiatorArea |
| s14 | Qhot_W, Qleak_W, ThotIface_C, TcoldIface_C, Rth_K_W, tauThermal_s |
| s15 | wNet_J_kg, Wcycle_J, maxFreqHz, powerDensity_W_kg, efficiency, carnotFrac, fatigueCycles, netPositive, recoveryStress_MPa |
| s16 | ranking[], best, wNet_J_kg, powerDensity_W_kg |
| s17 | Wrev_J, Pwheel_W, tauMean_Nm, torqueRipple, startupOK, heatDemand_W, deficit_W, feasible |
| s18 | Euse_J, Etotal_J, efficiency, backupDuration_s, bufferMass_kg |
| s19 | switchCount, chatter, engageN, disengageN, safe |
| s20 | sustainableSpeed_kmh, range_km, energyDeficit_J, journeySuccess, sourceShares{} |
| s21 | SPLext_dBA, SPLcabin_dBA |
| s22 | productionCost, Cpk, toleranceStack_mm |
| s23 | minSafetyFactor, pfWingRoot, fatigueLife_cycles, topRisks[] |
| s24 | passRate, results[] |
| s25 | sobol{}, paretoFront[] |
| s26 | prototypeCost, lifecycleCost, embodiedCO2_kg, npv |

Downstream suites must work when an upstream output is absent (use documented defaults, provenance `estimated`),
and should label coupled values with provenance `coupled`.

## Rules
- Real governing physics only; never fabricate success. Show failures with reasons and fixes.
- Every KPI and chart carries an `explain`. Every suite gives recommendations aligned with best engineering
  practice, economics and sustainability where relevant.
- Do not name commercial simulation software. Do not reference hygroscopic/moisture-responsive materials.
- Keep modules self-contained; heavy shared numerics go in new files under `js/core/` owned by one builder.
