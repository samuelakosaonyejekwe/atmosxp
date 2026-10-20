// Core regression tests against published reference values. Run: node tools/tests.mjs
import * as N from '../js/core/num.js';
import * as P from '../js/core/physics.js';
import * as A from '../js/core/analysis.js';
import { naca4, airfoilStats, box, cylinder, toSTL, toSTEP, vehicleAssembly } from '../js/core/cad.js';
import { parseDelimited, parseAirfoil, parseSTL, zip } from '../js/core/io.js';

let pass = 0, fail = 0; const out = [];
const t = (name, got, want, tol, rel = true) => { const ok = Number.isFinite(got) && Math.abs(got - want) <= (rel ? tol * Math.abs(want || 1) : tol); ok ? pass++ : fail++; if (!ok) out.push(`FAIL ${name}: got ${got}, want ${want} ±${tol}${rel ? ' (rel)' : ''}`); };
const ok = (name, cond, info = '') => { cond ? pass++ : fail++; if (!cond) out.push(`FAIL ${name} ${info}`); };

// atmosphere
t('ISA ρ(0)', P.isa(0).rho, 1.2250, 1e-3); t('ISA p(11 km)', P.isa(11000).p, 22632, 2e-3); t('ISA T(5 km)', P.isa(5000).T, 255.65, 1e-4);
t('psat(20 °C) Buck', P.psat(20), 2338.8, 2e-3); t('dew point 20 °C/50 %', P.dewPoint(20, 50), 9.26, 0.05, false);
t('moist air ρ 15 °C dry', P.moistAir(101325, 15, 0).rho, 1.2250, 2e-3); ok('moist air lighter than dry', P.moistAir(101325, 30, 90).rho < P.moistAir(101325, 30, 0).rho);
t('Sutherland μ(288 K)', P.moistAir(101325, 15, 0).mu, 1.789e-5, 1e-2);
// solar position (NOAA calculator: 2026-06-21 12:00 UTC at 0°N 0°E → elevation ≈ 66.6°, declination ≈ 23.44°)
const sp = P.solarPosition(new Date(Date.UTC(2026, 5, 21, 12, 0, 0)), 0, 0); t('solar declination solstice', sp.declination, 23.44, 0.05, false); t('solar elevation equator solstice noon', sp.elevation, 66.56, 0.3, false);
const sp2 = P.solarPosition(new Date(Date.UTC(2026, 2, 20, 12, 0, 0)), 0, 0); ok('equinox noon sun near zenith', sp2.zenith < 3, sp2.zenith);
ok('night: sun below horizon', P.solarPosition(new Date(Date.UTC(2026, 2, 20, 0, 0, 0)), 0, 0).elevation < -60);
const cs = P.clearSky(0, 0, 3); ok('clear-sky GHI at zenith 900–1150 W/m²', cs.ghi > 900 && cs.ghi < 1150, cs.ghi); ok('clear-sky zero below horizon', P.clearSky(95).ghi === 0);
// sky radiation
const eps = P.skyEmissivity['Berdahl–Martin'](293.15, 10); ok('sky emissivity 0.7–0.9', eps > 0.7 && eps < 0.9, eps); t('overcast emissivity = 1', P.cloudyEmissivity(0.8, 1), 1, 1e-9);
t('Carnot 293/288', P.carnot(293, 288), 0.017065, 1e-3); t('Planck integral → σT⁴', N.trapz(N.linspace(0.1e-6, 200e-6, 40000).map(l => Math.PI * P.planck(l, 300)), N.linspace(0.1e-6, 200e-6, 40000)), 5.670374e-8 * 300 ** 4, 1e-2);
// road load (specification: 124 W at 20 km/h, 406 W at 40 km/h for 300 kg, Crr 0.006, CdA 0.25)
const v0 = { ...P.DEFAULT_VEHICLE, mass: 300, Crr: 0.006, CdA: 0.25 };
t('road load 20 km/h', P.roadLoad(P.kmh(20), v0).P, 124.3, 5e-3); t('road load 40 km/h', P.roadLoad(P.kmh(40), v0).P, 406.2, 5e-3);
// apparent wind & force resolution
const aw = P.apparentWind(6, 90, 0, 0); t('beam wind at rest: AWA 90°', Math.abs(aw.beta), 90, 1e-6, false); t('beam wind Va', aw.Va, 6, 1e-9);
const awH = P.apparentWind(0, 0, 0, 10); t('still air, moving: head-on AWA 0°', Math.abs(awH.beta), 0, 1e-6, false); t('still air Va = v', awH.Va, 10, 1e-9);
const wf = P.wingForces(6, 90, 1, 0.05, 8); t('beam: thrust = lift − 0 drag component', wf.thrust, wf.L, 1e-9); t('beam: side = drag', Math.abs(wf.side), wf.D, 1e-9);
ok('head-on: no positive thrust', P.wingForces(6, 0, 1, 0.05, 8).thrust < 0);
// numerics
t('brent root cos', N.brent(Math.cos, 1, 2).x, Math.PI / 2, 1e-10);
const ode = N.rk45((tt, y) => [-y[0]], 0, [1], 5, { rtol: 1e-9, atol: 1e-12 }); t('rk45 exp(-5)', ode.y[ode.y.length - 1][0], Math.exp(-5), 1e-6);
const eg = N.eigSym([[2, 1], [1, 2]]); t('eigSym λ1', eg.values[0], 1, 1e-10); t('eigSym λ2', eg.values[1], 3, 1e-10);
const egg = N.eigGeneral([[0, 1], [-4, 0]]); t('eigGeneral |imag| = 2', Math.abs(egg[0].im), 2, 1e-8);
t('luSolve', N.luSolve([[4, 1], [2, 3]], [1, 2])[1], 0.6, 1e-12);
const sb = new N.SparseBuilder(3); [[0, 0, 4], [0, 1, 1], [1, 0, 1], [1, 1, 3], [1, 2, 1], [2, 1, 1], [2, 2, 2]].forEach(([i, j, v]) => sb.add(i, j, v)); const cg = N.pcg(sb.toCSR(), [1, 2, 3]); const ref = N.luSolve([[4, 1, 0], [1, 3, 1], [0, 1, 2]], [1, 2, 3]); t('pcg vs direct', cg.x[2], ref[2], 1e-8);
t('trapz x² 0..1', N.trapz(N.linspace(0, 1, 2001).map(x => x * x), N.linspace(0, 1, 2001)), 1 / 3, 1e-6);
const lm = N.levenbergMarquardt((x, p) => p[0] * Math.exp(-p[1] * x), N.linspace(0, 4, 40), N.linspace(0, 4, 40).map(x => 3 * Math.exp(-0.7 * x)), [1, 1]); t('LM fit a', lm.p[0], 3, 1e-6); t('LM fit k', lm.p[1], 0.7, 1e-6);
// GCI (Celik et al. 2008 worked example: φ1 = 6.063, φ2 = 5.972, φ3 = 5.863, r21 = 1.5, r32 = 1.333) → p = 1.53, GCI21 = 2.17 %
const g = N.gci(6.063, 5.972, 5.863, 1, 1.5, 2.0); t('GCI observed order (Celik 1.53)', g.p, 1.53, 0.02); t('GCI fine % (Celik 2.17)', g.gci21 * 100, 2.17, 0.02); t('GCI extrapolated (Celik 6.1685)', g.fext, 6.1685, 1e-4);
ok('range(a,b)', JSON.stringify(N.range(2, 5)) === '[2,3,4]'); ok('range(n)', JSON.stringify(N.range(3)) === '[0,1,2]');
t('normInv(0.975)', N.normInv(0.975), 1.95996, 1e-4); t('normCdf(1.96)', N.normCdf(1.96), 0.975, 1e-3);
// sensitivity (Ishigami: S1 ≈ 0.314, 0.442, 0; ST3 ≈ 0.244)
const ish = (x) => Math.sin(x[0]) + 7 * Math.sin(x[1]) ** 2 + 0.1 * x[2] ** 4 * Math.sin(x[0]);
const so = A.sobol(ish, [[-Math.PI, Math.PI], [-Math.PI, Math.PI], [-Math.PI, Math.PI]], { N: 4096 }); t('Sobol S1 x1', so.S1[0], 0.314, 0.06, false); t('Sobol S1 x2', so.S1[1], 0.442, 0.06, false); t('Sobol ST x3', so.ST[2], 0.244, 0.06, false);
const fr = A.form((x) => 10 - x[0] - x[1], [{ mu: 2, sigma: 1 }, { mu: 3, sigma: 1 }]); t('FORM β linear', fr.beta, 5 / Math.SQRT2, 1e-4);
// geometry & exporters
const af = naca4('0012', 101), st = airfoilStats(af.points); t('NACA 0012 t/c', st.tmax, 0.12, 0.01); t('NACA 0012 area ≈ 0.0822', st.area, 0.0822, 0.02);
const bx = box('b', 1, 2, 3); t('box volume', bx.volume(), 6, 1e-9); ok('box closed & oriented', bx.checkManifold().closed && bx.checkManifold().oriented);
const cy = cylinder('c', 0.5, 2, 256); t('cylinder volume', cy.volume(), Math.PI * 0.25 * 2, 2e-3);
const asm = vehicleAssembly(P.DEFAULT_VEHICLE); const notClosed = asm.parts.filter(m => !m.checkManifold().closed).map(m => m.name); ok('all vehicle parts closed solids', notClosed.length === 0, notClosed.join(', '));
{ // envelope: no non-wheel part below 0.10 m (except skirts, spats, wheels, brakes, fork, axle arms), none wider than the spats
  const wheelish = /Tyre|Rim|hub|Hub|Brake disc|Brake caliper|Spoke|spat|Spat|skirt|fork|Fork|axle|Axle|Bearing/;
  const A2 = vehicleAssembly(P.DEFAULT_VEHICLE), yMax = A2.meta.axle ? A2.meta.axle.overallWidth / 2 + 1e-3 : Infinity;
  const low = A2.parts.filter(m => !wheelish.test(m.name) && m.bbox()[2] < 0.10).map(m => m.name);
  const wide = A2.parts.filter(m => { const b = m.bbox(); return Math.max(Math.abs(b[1]), Math.abs(b[4])) > yMax; }).map(m => m.name);
  ok('no non-wheel part below 0.10 m', low.length === 0, low.join(', ')); ok('no part outside the overall width', wide.length === 0, wide.join(', ')); }
const negVol = asm.parts.filter(m => m.volume() <= 0).map(m => m.name); ok('all vehicle parts positive volume', negVol.length === 0, negVol.join(', '));
ok('overall height ≤ 3.10 m (owner cap)', asm.meta.height <= 3.10, asm.meta.height);
ok('overall width ≤ 2.00 m', (asm.meta.overallWidthAll ?? asm.meta.width) <= 2.005, asm.meta.overallWidthAll);
ok('length ≤ 3.5 m', asm.meta.length <= 3.5 + 1e-9, asm.meta.length);
const stl = new Uint8Array(await toSTL([bx]).arrayBuffer()); const back = parseSTL(stl.buffer); ok('STL round-trip triangles', back.triangles === bx.triangleCount, `${back.triangles} vs ${bx.triangleCount}`);
const step = toSTEP([bx]); ok('STEP header & closed shell', step.startsWith('ISO-10303-21;') && step.includes('CLOSED_SHELL') && step.trim().endsWith('END-ISO-10303-21;'));
const { to3MF, toIGES } = await import('../js/core/export2.js');
const igs = (await toIGES([bx]).text()).trim().split('\n'); ok('IGES fixed 80-column records', igs.every(l => l.length === 80) && igs[igs.length - 1].includes('T      1'));
const m3 = new Uint8Array(await (await to3MF([bx])).arrayBuffer()); ok('3MF package is a ZIP', m3[0] === 0x50 && m3[1] === 0x4b);
// importers
const tb = parseDelimited('time,temperature [°C],wind\n1,20,3\n2,21,4\n'); ok('CSV parse', tb.nRows === 2 && tb.units['temperature [°C]'] === '°C');
const tb2 = parseDelimited('a;b\n1,5;2\n'); ok('semicolon CSV', tb2.header.length === 2);
const fa = parseAirfoil('NACA test\n1 0\n0.5 0.05\n0 0\n0.5 -0.05\n1 0\n1 0\n0.5 0.05\n0 0\n0.5 -0.05\n1 0\n0.9 0.01\n0.8 0.02\n'); ok('airfoil parse', fa.points.length > 10);
const z = await zip([{ name: 'a.txt', data: 'hello' }]); ok('zip has end-of-central-directory', new Uint8Array(await z.arrayBuffer()).slice(-22)[0] === 0x50);

// measured converter efficiency → latent-heat-store viability (synthetic numbers, test of the pipeline only)
{ const { measuredEfficiency } = await import('../js/core/measured.js'); const PS = await import('../js/core/pcmstore.js');
  const mk = (frac, n = 6, jitter = 0.005) => { const Th = 160, Tc = 40, carnot = 1 - (Tc + 273.15) / (Th + 273.15), Q = 1000; const cols = { test_id: [], Q_in_J: [], W_net_J: [], T_hot_C: [], T_cold_C: [], Q_out_J: [] };
    for (let i = 0; i < n; i++) { const f = frac + (i % 2 ? jitter : -jitter), W = Q * f * carnot; cols.test_id.push('T' + (i % 3)); cols.Q_in_J.push(Q); cols.W_net_J.push(W); cols.T_hot_C.push(Th); cols.T_cold_C.push(Tc); cols.Q_out_J.push(Q - W); }
    return [{ name: 'synthetic', use: 'validation', header: Object.keys(cols), columns: cols }]; };
  const lo = measuredEfficiency(mk(0.039)), hi = measuredEfficiency(mk(0.32));
  ok('measured η: fraction of Carnot recovered', Math.abs(hi.mean - 0.32) < 1e-9, hi.mean); ok('measured η: lower 95 % below mean', hi.lo95 < hi.mean && hi.lo95 > 0.3, hi.lo95); ok('measured η: validation rules met', hi.validated);
  ok('measured η: no data → null', measuredEfficiency([]) === null);
  const pcm = PS.pcmById('hdpe'), Tlow = pcm.Tm - 25, Ttop = Math.min(pcm.Tmax, pcm.Tm + 8), size = (frac) => { let best = null; for (let Af = 0; Af <= 30; Af += 1) { const rej = PS.rejection({ v: 20 / 3.6, Afin: Af }), op = PS.storeOperatingPoint({ pcm, Ta: 30, Proad: 55, frac, rej }); if (!op.feasible) continue; const st = PS.storeForEnergy(pcm, op.Q * 3600 * 1.1, { Tlow, Ttop, Tamb: 30 }), tot = st.total + rej.finMass + 2 * 0.25 * op.Q / 1000 + 0.8; if (!best || tot < best) best = tot; } return best ?? Infinity; };
  const base = 186, cap = 258; ok('store NOT viable on the built-device value', base + size(lo.lo95) > cap, size(lo.lo95)); ok('store viable on a high measured value (sizing flips)', base + size(hi.lo95) <= cap, size(hi.lo95)); }

console.log(out.join('\n')); console.log(`\n${pass} passed, ${fail} failed`); process.exit(fail ? 1 : 0);
