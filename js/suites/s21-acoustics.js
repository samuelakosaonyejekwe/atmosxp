// Suite 21 — Acoustics, structural vibration & near-silent operation.
import { fmt, linspace, range, max, sum } from '../core/num.js';
import { kmh, moistAir } from '../core/physics.js';
import { THIRD_OCT, aWeight, dbSum, massLawTL } from '../core/vehicle.js';
import { h } from '../core/ui.js';

const SURF = { 'dense asphalt (DAC 0/11)': 0, 'stone-mastic asphalt': -1, 'porous asphalt': -4, 'brushed concrete': 2.5, 'paving / cobbles': 6, 'gravel': 4 };
// BPM turbulent-boundary-layer trailing-edge spectral shape A_min(a)
const Amin = (a) => a < 0.204 ? Math.sqrt(Math.max(0, 67.552 - 886.788 * a * a)) - 8.219 : a <= 0.244 ? -32.665 * a + 3.981 : -142.795 * a ** 3 + 103.656 * a * a - 57.757 * a + 6.006;

export default {
  description: 'Exterior and cabin noise of the near-silent vehicle: tyre–road noise (speed-scaled pass-by correlation with surface corrections), aerodynamic trailing-edge noise of the wings and body (U⁵ boundary-layer scaling, BPM spectral shape), tonal wheel-hub contact noise at the cartridge passing frequency radiated by the rim (plate mobility, radiation efficiency), A-weighting, 1/3-octave spectra, distance attenuation, cabin transmission by the mass law, and regulatory context including minimum-sound requirements for quiet vehicles.',
  literature: 'tyre road noise trailing edge noise prediction quiet vehicle pedestrian safety sound',
  credibility: 'Analytical screening', sustain: true,
  equations: ['Linear acoustic wave equation & sound-pressure level L_p = 20 log₁₀(p/20 µPa); incoherent energy summation of sources', 'Lighthill / Ffowcs Williams–Hawkings scaling for trailing-edge noise: p² ∝ δ* M⁵ L D / r² (Brooks–Pope–Marcolini TBL-TE model with Strouhal peak 0.02 M^−0.6)', 'Tyre–road noise speed law L = A + B log₁₀(v/v_ref) with surface, width and wheel-count corrections', 'Point mobility of an infinite plate Y = 1/(8√(B m″)); radiated power W = ρc σ S ⟨v²⟩; critical frequency f_c = c²/(1.8 c_L h)', 'Hemispherical spreading L_p = L_W − 20 log₁₀ r − 8', 'Mass-law transmission loss TL = 20 log₁₀(m″ f) − 47 (field incidence); cabin level L_in = L_out − TL + 10 log₁₀(S/A)', 'IEC 61672 A-weighting'],
  methods: ['1/3-octave synthesis 25 Hz–10 kHz for each source', 'Harmonic series of the periodic cam contact force (smooth compliant cam: amplitude ∝ 1/k²) at k·f₀; low structural damping amplifies response', 'Speed sweeps for source ranking'],
  assumptions: ['Empirical tyre correlation calibrated for passenger-car tyres; narrow low-rolling-resistance tyres corrected by width', 'Free-field propagation over hard ground (+3 dB ground reflection included in hemispherical term)', 'Hub impulse force from Suite 17 torque ripple when available'],
  ibc: { initial: ['Initial structural displacement/velocity, acoustic pressure field, ambient acoustic state, vehicle speed, wheel speed, initial vibration modes'], boundary: ['Unsteady aerodynamic surface loading, wheel–road excitation, cam contact forces, structural attachments, damping, acoustic radiation/absorbing boundaries, cabin wall transmission, ambient pressure'] },
  cvv: { calibration: ['Microphone pass-by measurements (ISO 362-type), sound intensity, accelerometers on rim and hub, acoustic spectra'], verification: ['A-weighting at 1 kHz = 0 dB', 'Energy sum of identical sources +3 dB', 'Mass-law TL slope 6 dB/octave'], validation: ['Coast-by measurements (ISO 13325) with and without hub engaged; cabin microphone'] },
  formats: { import: ['CSV spectra (frequency, level)', 'CSV time signals'], export: ['CSV 1/3-octave spectra', 'PNG/SVG plots'] },
  params: [
    { key: 'speed', label: 'Vehicle speed', unit: 'km/h', type: 'range', min: 5, max: 80, step: 1, group: 'Operating point', default: 30 },
    { key: 'dist', label: 'Observer distance', unit: 'm', type: 'number', min: 1, max: 50, step: 0.5, group: 'Operating point', default: 7.5, help: '7.5 m = standard pass-by position; 2 m = minimum-sound check position.' },
    { key: 'surface', label: 'Road surface', type: 'select', group: 'Tyre–road', default: 'dense asphalt (DAC 0/11)', options: Object.keys(SURF) },
    { key: 'width', label: 'Tyre width', unit: 'mm', type: 'number', min: 25, max: 250, step: 1, group: 'Tyre–road', default: 80 },
    { key: 'headwind', label: 'Apparent-wind speed over wings', unit: 'm/s', type: 'number', min: 0, max: 40, step: 0.5, group: 'Aerodynamic', default: 10, auto: (c) => { const o = c.upstream('s07'); if (o?.Va) return { value: +o.Va.toFixed(2), prov: 'coupled' }; return c.env ? { value: +(Math.hypot(c.env.Uz, kmh(30))).toFixed(2), prov: 'calc' } : undefined; } },
    { key: 'hubOn', label: 'Hub drive engaged', type: 'bool', group: 'Hub drive', default: true },
    { key: 'nCart', label: 'Cartridges per driven wheel', type: 'number', min: 1, max: 64, step: 1, group: 'Hub drive', default: 24 },
    { key: 'Fimp', label: 'Cam contact impulse force amplitude', unit: 'N', type: 'number', min: 0, max: 2000, step: 1, group: 'Hub drive', default: 40, auto: (c) => { const o = c.upstream('s17'); if (o?.torqueRipple !== undefined && o?.tauMean_Nm !== undefined) return { value: +Math.max(1, Math.abs(o.torqueRipple * o.tauMean_Nm) / (0.42 * 0.6 * c.vehicle.wheelRadius)).toFixed(1), prov: 'coupled' }; return undefined; } },
    { key: 'rimH', label: 'Effective hub/rim structure thickness', unit: 'mm', type: 'number', min: 3, max: 50, step: 0.5, group: 'Hub drive', default: 15, expert: true, help: 'Equivalent plate thickness giving the point mobility of the stiff hub/rim assembly (≈ 10⁻⁴ m/(N·s)).' },
    { key: 'radArea', label: 'Radiating area per wheel', unit: 'm²', type: 'number', min: 0.02, max: 1, step: 0.01, group: 'Hub drive', default: 0.18, expert: true },
    { key: 'eta', label: 'Structural loss factor', type: 'number', min: 0.001, max: 0.2, step: 0.001, group: 'Hub drive', default: 0.02, expert: true },
    { key: 'wallMass', label: 'Cabin wall surface mass', unit: 'kg/m²', type: 'number', min: 1, max: 40, step: 0.5, group: 'Cabin', default: 6, auto: (c) => { const ch = c.material('chassis'), co = c.material('chassisCore'); return { value: +(2 * 0.0012 * ch.rho + 0.015 * co.rho).toFixed(2), prov: 'calc' }; } },
    { key: 'cabinS', label: 'Cabin boundary area', unit: 'm²', type: 'number', min: 1, max: 20, step: 0.5, group: 'Cabin', default: 6 },
    { key: 'cabinA', label: 'Cabin absorption area', unit: 'm² Sabine', type: 'number', min: 0.2, max: 10, step: 0.1, group: 'Cabin', default: 1.5 },
  ],
  presets: [{ label: 'Pass-by 50 km/h', values: { speed: 50, dist: 7.5 } }, { label: 'Pedestrian check 20 km/h @ 2 m', values: { speed: 20, dist: 2 } }, { label: 'Porous road', values: { surface: 'porous asphalt' } }, { label: 'Hub disengaged', values: { hubOn: false } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    const veh = ctx.vehicle, R = veh.wheelRadius, air = moistAir(101325, ctx.env?.T ?? 20, ctx.env?.RH ?? 60), c0 = Math.sqrt(1.4 * 287 * ((ctx.env?.T ?? 20) + 273.15)), rho = air.rho, nu = air.nu;
    const F = THIRD_OCT, Aw = F.map(aWeight);
    const toA = (bands) => dbSum(bands.map((L, i) => L + Aw[i]));
    const spectra = (vkmh, r) => {
      const v = kmh(vkmh);
      // tyre–road: overall A-weighted level at 7.5 m then spectral shape peaked near 1 kHz
      const LA75 = 70 + 30 * Math.log10(Math.max(vkmh, 1) / 70) + 10 * Math.log10((veh.nWheels || 3) / 4) + 6 * Math.log10(p.width / 205) + SURF[p.surface];
      const shape = F.map(f => -10 * ((Math.log10(f / 1000)) / 0.32) ** 2 / 2.3), shA = toA(shape), tyre = shape.map(s => s + LA75 - shA - 20 * Math.log10(r / 7.5));
      // wing trailing-edge noise (BPM TBL-TE, both sides) + body
      const Ua = Math.max(0.5, p.headwind * (vkmh / Math.max(1, p.speed)) ** 0.5 + 0 * v), te = (chord, span, n) => { const M = Ua / c0, Re = Ua * chord / nu, dstar = 0.046 * chord * Re ** -0.2, Stp = 0.02 * M ** -0.6; return F.map(f => { const St = f * dstar / Ua, a = Math.abs(Math.log10(St / Stp)); return 10 * Math.log10(Math.max(1e-30, dstar * M ** 5 * span * 1 / (r * r))) + Amin(a) + 128.5 - 3 + 3 + 10 * Math.log10(n); }); };
      const cw = veh.wingArea / (veh.nWings || 2) / veh.wingHeight, aero = te(cw, veh.wingHeight, veh.nWings || 2), body = (() => { const M = v / c0, Re = Math.max(1, v * veh.length / nu), dstar = 0.046 * veh.length * Re ** -0.2, Stp = 0.02 * Math.max(M, 1e-4) ** -0.6; return F.map(f => { const St = f * dstar / Math.max(v, 0.1), a = Math.abs(Math.log10(St / Stp)); return 10 * Math.log10(Math.max(1e-30, dstar * Math.max(M, 1e-4) ** 5 * veh.width / (r * r))) + Amin(a) + 128.5; }); })();
      // hub contact tones
      const hub = F.map(() => -Infinity), tones = [];
      if (p.hubOn && (veh.driveWheels || 2) > 0) {
        const nrps = v / (2 * Math.PI * R), f0 = p.nCart * nrps, E = 71.7e9, rhoM = 2810, nuM = 0.33, hh = p.rimH / 1000, B = E * hh ** 3 / (12 * (1 - nuM * nuM)), mpp = rhoM * hh, Y = 1 / (8 * Math.sqrt(B * mpp)), cL = Math.sqrt(E / (rhoM * (1 - nuM * nuM))), fc = c0 * c0 / (1.8 * cL * hh);
        for (let k2 = 1; k2 <= 12; k2++) { const f = k2 * f0; if (f < 20 || f > 11000) continue; const Fk = p.Fimp / (k2 * k2), vel = Fk * Y * (1 + 0.02 / Math.max(p.eta, 1e-3)) ** 0.5, sigma = f < fc ? Math.min(1, 0.5 * (f / fc) ** 0.5 * (f / 200) ** 1) : 1, W = rho * c0 * sigma * p.radArea * vel * vel / 2 * (veh.driveWheels || 2), Lw = 10 * Math.log10(Math.max(W, 1e-20) / 1e-12), Lp = Lw - 20 * Math.log10(r) - 8; tones.push({ f, Lp, LpA: Lp + aWeight(f) }); const bi = F.reduce((bb, fb, i) => Math.abs(Math.log(fb / f)) < Math.abs(Math.log(F[bb] / f)) ? i : bb, 0); hub[bi] = dbSum([hub[bi], Lp]); }
      }
      const total = F.map((_, i) => dbSum([tyre[i], aero[i], body[i], hub[i]]));
      return { tyre, aero, body, hub, total, tones, LA: { tyre: toA(tyre), aero: toA(aero), body: toA(body), hub: toA(hub), total: toA(total) } };
    };
    const S = spectra(p.speed, p.dist), S1 = spectra(p.speed, 1);
    // cabin
    const TL = F.map(f => massLawTL(p.wallMass, f)), cabinBands = F.map((f, i) => dbSum([S1.tyre[i] - TL[i], S1.aero[i] - TL[i], S1.body[i] - TL[i], S1.hub[i] - Math.max(0, TL[i] - 10)]) + 10 * Math.log10(p.cabinS / p.cabinA)), LAcab = toA(cabinBands);
    // AVAS-style minimum sound check at 2 m (context)
    const S2_10 = spectra(10, 2).LA.total, S2_20 = spectra(20, 2).LA.total;
    // checks
    out.checks.push({ name: 'A-weighting at 1 kHz', value: aWeight(1000), target: '0 ± 0.1 dB', pass: Math.abs(aWeight(1000)) < 0.1 });
    out.checks.push({ name: 'Two equal sources sum', value: dbSum([60, 60]), target: '63.01 dB', pass: Math.abs(dbSum([60, 60]) - 63.0103) < 1e-3 });
    out.checks.push({ name: 'Mass-law slope per octave', value: massLawTL(p.wallMass, 2000) - massLawTL(p.wallMass, 1000), target: '6.02 dB', pass: Math.abs(massLawTL(p.wallMass, 2000) - massLawTL(p.wallMass, 1000) - 6.0206) < 1e-3 });
    // KPIs
    const k = (label, value, unit, explain, prov = 'calc', status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('Exterior level L_A', S.LA.total, 'dB(A)', `Total A-weighted level at ${p.dist} m, ${p.speed} km/h.`, 'calc', S.LA.total < 68 ? 'ok' : 'warn');
    k('Tyre–road contribution', S.LA.tyre, 'dB(A)', 'Speed-scaled tyre/road correlation with surface and width corrections.');
    k('Wing trailing-edge noise', S.LA.aero, 'dB(A)', 'Turbulent boundary layer convecting past the wing trailing edges (U⁵ scaling).');
    k('Body flow noise', S.LA.body, 'dB(A)', 'Same mechanism over the body.');
    k('Hub contact tones', p.hubOn ? S.LA.hub : '—', 'dB(A)', 'Cartridge–cam contact impulses at N × wheel rev/s radiated by the rim.', ctx.upstream('s17') ? 'coupled' : 'estimated');
    k('Cartridge passing frequency', p.nCart * kmh(p.speed) / (2 * Math.PI * R), 'Hz', 'Fundamental of the periodic hub excitation (N_cart × rev/s).');
    k('Cabin level', LAcab, 'dB(A)', `Mass-law transmission through ${fmt(p.wallMass, 3)} kg/m² walls with ${p.cabinA} m² absorption.`, 'calc', LAcab < 65 ? 'ok' : 'warn');
    k('Level at 2 m, 10 km/h', S2_10, 'dB(A)', 'Pedestrian-awareness context (quiet-vehicle rules require ≈ 50 dB(A) at 10 km/h, 2 m).', 'calc', S2_10 >= 50 ? 'ok' : 'warn');
    k('Level at 2 m, 20 km/h', S2_20, 'dB(A)', 'Quiet-vehicle rules require ≈ 56 dB(A) at 20 km/h, 2 m.', 'calc', S2_20 >= 56 ? 'ok' : 'warn');
    // verdicts
    out.verdicts.push({ test: 'Near-silent objective (pass-by)', status: spectra(50, 7.5).LA.total <= 68 ? 'pass' : 'warn', detail: `${fmt(spectra(50, 7.5).LA.total, 3)} dB(A) at 50 km/h, 7.5 m (passenger-car limit context ≈ 68–72 dB(A)).` });
    out.verdicts.push({ test: 'Completely noiseless? (requirement statement)', status: 'info', detail: 'Information: tyre–road and airflow noise cannot be eliminated in any moving vehicle; the specification’s objective is near-silent operation by minimisation, which the other checks assess.' });
    out.verdicts.push({ test: 'Pedestrian audibility at low speed', status: S2_20 >= 56 && S2_10 >= 50 ? 'pass' : 'warn', detail: `${fmt(S2_10, 3)} / ${fmt(S2_20, 3)} dB(A) at 10 / 20 km/h (2 m). Quiet vehicles must emit a minimum sound under current rules (e.g. UN R138 in many markets).`, fix: 'A non-electric, speed-proportional acoustic alert (e.g. tuned mechanical or aeroacoustic sounder) would be needed because powered sound generators conflict with the no-electricity rule.' });
    if (p.hubOn) out.verdicts.push({ test: 'Hub tonal annoyance', status: S.LA.hub < S.LA.tyre - 3 ? 'pass' : 'warn', detail: `Hub tones ${fmt(S.LA.hub, 3)} dB(A) vs tyre ${fmt(S.LA.tyre, 3)} dB(A). Tones are perceived as more annoying than broadband noise of equal level.` });
    // charts
    out.charts.push({ title: `1/3-octave spectra by source (A-weighted, ${p.dist} m)`, type: 'line', x: { label: 'Frequency', unit: 'Hz', log: true }, y: { label: 'Level', unit: 'dB(A)' }, series: [{ name: 'Tyre–road', x: F, y: S.tyre.map((L, i) => L + Aw[i]) }, { name: 'Wing TE', x: F, y: S.aero.map((L, i) => L + Aw[i]) }, { name: 'Body flow', x: F, y: S.body.map((L, i) => L + Aw[i]) }, { name: 'Hub tones', x: F, y: S.hub.map((L, i) => Number.isFinite(L) ? L + Aw[i] : NaN), mode: 'lines+markers' }, { name: 'Total', x: F, y: S.total.map((L, i) => L + Aw[i]), width: 3 }], y2: null, wide: true, explain: 'Each source’s spectrum after A-weighting. Tyre–road noise peaks near 1 kHz; wing trailing-edge noise depends on the boundary-layer thickness; hub tones appear at multiples of the cartridge passing frequency.' });
    const vs = linspace(5, 80, 31), sw = vs.map(s => spectra(s, p.dist).LA);
    out.charts.push({ title: 'Source contributions vs speed', type: 'line', x: { label: 'Speed', unit: 'km/h' }, y: { label: 'L_A', unit: 'dB(A)' }, series: [{ name: 'Total', x: vs, y: sw.map(s => s.total) }, { name: 'Tyre–road', x: vs, y: sw.map(s => s.tyre) }, { name: 'Wing TE', x: vs, y: sw.map(s => s.aero) }, { name: 'Body', x: vs, y: sw.map(s => s.body) }, { name: 'Hub', x: vs, y: sw.map(s => s.hub) }], annotations: [{ type: 'hline', y: 68, label: 'car pass-by context', color: '#f59e0b' }], explain: 'Tyre noise grows ≈ 30 log₁₀(v); flow noise ≈ 50 log₁₀(U). At low speed the vehicle may be quieter than pedestrians can safely hear.' });
    out.charts.push({ title: 'Cabin spectrum and wall transmission loss', type: 'line', x: { label: 'Frequency', unit: 'Hz', log: true }, y: { label: 'dB' }, series: [{ name: 'Cabin L (dB(A))', x: F, y: cabinBands.map((L, i) => L + Aw[i]) }, { name: 'Wall TL (dB)', x: F, y: TL, dash: '4 3' }], explain: 'The lightweight composite wall is acoustically thin, so low-frequency noise passes easily (mass law: +6 dB per octave of frequency or per doubling of surface mass).' });
    if (S.tones.length) out.charts.push({ title: 'Hub tonal components', type: 'bar', categories: S.tones.map(t => fmt(t.f, 3) + ' Hz'), y: { label: 'L_p', unit: 'dB(A)' }, series: [{ name: 'tone', values: S.tones.map(t => t.LpA) }], explain: 'Harmonics of the contact impulse train. Avoid coincidence with rim/wing structural modes (Suites 06, 09) to prevent resonant amplification.' });
    const sp = linspace(5, 80, 40); out.charts.push({ title: 'Excitation frequency map (Campbell-type)', type: 'line', x: { label: 'Speed', unit: 'km/h' }, y: { label: 'Frequency', unit: 'Hz' }, series: [1, 2, 3].map(k2 => ({ name: `${k2}× cartridge passing`, x: sp, y: sp.map(s => k2 * p.nCart * kmh(s) / (2 * Math.PI * R)) })).concat([{ name: 'Wheel rotation', x: sp, y: sp.map(s => kmh(s) / (2 * Math.PI * R)) }]), annotations: (ctx.upstream('s06')?.f1Hz ? [{ type: 'hline', y: ctx.upstream('s06').f1Hz, label: 'wing 1st mode', color: '#ef4444' }] : []), explain: 'Where an excitation line crosses a structural natural frequency, resonance can occur at that speed.' });
    out.tables.push({ title: 'Source ranking at the operating point', cols: ['Source', 'L_A at ' + p.dist + ' m (dB(A))', 'Share of acoustic energy (%)'], rows: ['tyre', 'aero', 'body', 'hub'].map(s => [s, S.LA[s], 100 * 10 ** (S.LA[s] / 10) / 10 ** (S.LA.total / 10)]) });
    out.tables.push({ title: 'Regulatory context (indicative — verify the rule applicable to the vehicle category & market)', cols: ['Requirement', 'Indicative value', 'This vehicle'], rows: [['Passenger-car pass-by limit (UN R51-03, phase 3)', '68–70 dB(A)', fmt(spectra(50, 7.5).LA.total, 3) + ' dB(A) @ 50 km/h'], ['Three-wheel (L-category) limits (UN R9 / R41 family)', '≈ 75–80 dB(A)', '—'], ['Quiet-vehicle minimum sound @ 10 km/h, 2 m (UN R138-type)', '≥ 50 dB(A)', fmt(S2_10, 3) + ' dB(A)'], ['Quiet-vehicle minimum sound @ 20 km/h, 2 m', '≥ 56 dB(A)', fmt(S2_20, 3) + ' dB(A)']] });
    const rank = ['tyre', 'aero', 'body', 'hub'].sort((a, b) => S.LA[b] - S.LA[a]);
    out.recs.push({ level: 'info', text: `Dominant source at ${p.speed} km/h: ${rank[0]} (${fmt(S.LA[rank[0]], 3)} dB(A)). Target it first; reducing a secondary source by 10 dB changes the total by less than 1 dB.`, basis: 'Energy summation' });
    if (p.hubOn && S.LA.hub > S.LA.tyre - 3) out.recs.push({ level: 'warn', text: 'Hub tones are audible above tyre noise. Use a ratchet-free compliant cam profile with smooth contact transitions, elastomeric damping layers (raise loss factor), and stagger cartridge pitch to spread energy over frequency.', basis: 'Tonal harmonics of the contact impulse train' });
    out.recs.push({ level: 'warn', text: 'Consider pedestrian safety: at walking-pace speeds the car is near-silent. Provide a passive, speed-proportional acoustic alert compatible with the no-electricity design and check applicable minimum-sound rules.', basis: 'Quiet-vehicle regulations (indicative)' });
    out.recs.push({ level: 'sustain', text: 'Low-noise porous or stone-mastic surfaces and narrow low-hysteresis tyres reduce both noise and rolling energy; serrated or porous wing trailing edges reduce trailing-edge noise by several dB.', basis: 'Source mechanisms modelled here' });
    out.explain.push({ title: 'Why “near-silent” is not “silent”', text: 'Without an engine the remaining noise comes from the tyres rolling on the road, from turbulent air flowing off the wing and body edges, and from contact impacts inside the hub drive. The suite quantifies each, so design effort can be aimed at the dominant one.' });
    out.views.push({ label: '2D animation: SPL map with moving vehicle & wavefronts', icon: '🎞', render: (el) => acoustic2D(el, { p, ctx, S1, S }) });
    out.views.unshift({ label: '3D acoustic field: pass-by, wavefronts & listeners', icon: '🔊', render: (el) => acoustic3D(el, { p, ctx, S1, S, dbSum }) });
    out.outputs = { SPLext_dBA: S.LA.total, SPLcabin_dBA: LAcab, sources: S.LA, minSound10_dBA: S2_10, minSound20_dBA: S2_20 };
    out.headline = `${fmt(S.LA.total, 3)} dB(A) at ${p.dist} m and ${p.speed} km/h (dominant: ${rank[0]}); cabin ${fmt(LAcab, 3)} dB(A).`;
    void range; void max; void sum;
    return out;
  },
};

// ---------------- 3-D acoustic field around the moving vehicle ----------------
// Solved: A-weighted source levels per mechanism referenced to 1 m (tyre/road, wing trailing edge, body, hub
// tones) from this suite's spectra; hub tone frequency from cartridge passing. Displayed: point sources at their
// physical locations with spherical spreading L(r) = L(1 m) − 20 log10 r, energy-summed on the ground plane and at
// listener probes while the vehicle drives past; wavefront rings are drawn at the true hub-tone wavelength
// (propagation slowed for display).
async function acoustic3D(el, o) {
  const { p, ctx, S1, S } = o;
  const [{ vehicleScene, playback, hud, legend3d, fieldMesh, Mesh }, mats] = await Promise.all([import('../core/sim3d.js'), import('../core/materials.js')]);
  el.append(h('p', { class: 'small muted' }, 'Pass-by at the selected speed. Source strengths are this suite’s solved 1 m A-weighted levels (tyre–road at the contact patches, wing trailing-edge noise at the wing centres, body flow noise, hub contact tones at the wheel hubs); propagation uses spherical spreading over a reflecting ground (+3 dB). The ground colour is the instantaneous total L_A; probes record the pass-by. Wavefront rings show the true hub-tone wavelength λ = c/f, with propagation slowed for display.'));
  const box = h('div'); el.append(box);
  const veh = ctx.vehicle, materials = Object.fromEntries(Object.keys(mats.COMPONENTS).map(k => [k, ctx.material(k)]));
  const SC = await vehicleScene(box, { vehicle: veh, materials, follow: false, height: 480, ground: 200 });
  const { V, THREE } = SC, Hd = hud(box);
  const L1 = S1.LA, v = p.speed / 3.6, c = 343, f0 = (S.tones && S.tones[0]?.f) || (p.nCart * v / (2 * Math.PI * veh.wheelRadius)), lam = c / Math.max(f0, 1);
  const wp = SC.asm.meta.wheelPos, zCE = SC.asm.meta.height - SC.asm.meta.span * 0.5;
  const nW = wp.length, dbAdd = (Ls) => 10 * Math.log10(Ls.reduce((a, L) => a + 10 ** (L / 10), 0) || 1e-12);
  const srcs = () => { const x0 = SC.car.position.x; const list = []; wp.forEach(([x, y]) => { list.push({ x: x0 + x, y, z: 0.05, L: L1.tyre - 10 * Math.log10(nW) }); if (p.hubOn) list.push({ x: x0 + x, y, z: veh.wheelRadius, L: L1.hub - 10 * Math.log10(nW) }); }); SC.asm.meta.wingX.forEach(x => list.push({ x: x0 + x, y: 0, z: zCE, L: L1.aero - 10 * Math.log10(SC.asm.meta.wingX.length) })); list.push({ x: x0, y: 0, z: veh.wheelRadius + veh.height * 0.5, L: L1.body }); return list; };
  const LA = (x, y, z) => dbAdd(srcs().map(s2 => s2.L + 3 - 20 * Math.log10(Math.max(0.5, Math.hypot(x - s2.x, y - s2.y, z - s2.z)))));
  // ground field mesh
  const gm = new Mesh('SPL ground'), NX = 96, NY = 64, X0 = -60, X1 = 60, Y0 = -30, Y1 = 30, id = [];
  for (let j = 0; j <= NY; j++) { id.push([]); for (let i = 0; i <= NX; i++) id[j].push(gm.v(X0 + (X1 - X0) * i / NX, Y0 + (Y1 - Y0) * j / NY, 0.03)); }
  for (let j = 0; j < NY; j++) for (let i = 0; i < NX; i++) gm.q(id[j][i], id[j][i + 1], id[j + 1][i + 1], id[j + 1][i]);
  const Lmax = Math.max(...Object.values(L1).filter(Number.isFinite)) + 3, Lmin = Lmax - 45;
  const field = fieldMesh(V, gm, () => Lmin, { range: [Lmin, Lmax], cmapName: 'inferno', opacity: 0.88 });
  legend3d(box, { title: 'L_A at ground (dB(A))', lo: Lmin, hi: Lmax, cmapName: 'inferno' });
  // probes (standard pass-by microphone 7.5 m, near-side 2 m, far 20 m) at 1.2 m height
  const probes = [{ n: '7.5 m pass-by mic', x: 0, y: -7.5 }, { n: '2 m kerb listener', x: 0, y: -2 }, { n: '20 m listener', x: 0, y: -20 }];
  const pmat = new THREE.MeshStandardMaterial({ color: 0x22d3ee, emissive: 0x0e7490 }); probes.forEach(q => { const m = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 1.2, 12), pmat); m.rotation.x = Math.PI / 2; m.position.set(q.x, q.y, 0.6); V.root.add(m); q.max = -Infinity; });
  // wavefront rings at true wavelength (hub tone)
  const rings = []; for (let k = 0; k < 14; k++) { const r = new THREE.Mesh(new THREE.TorusGeometry(1, 0.02, 6, 96), new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.6 })); V.root.add(r); rings.push(r); }
  const xStart = -45, xEnd = 45, T = (xEnd - xStart) / Math.max(v, 0.5);
  V.camera.position.set(0, -38, 16); V.controls.target.set(0, 0, 1); V.controls.update();
  const pb = playback(box, { duration: T, label: 'Pass-by time', onFrame: (t) => {
    const x = xStart + v * t; SC.setPose({ x, y: 0, heading: 0, dist: x - xStart });
    field.update((gx, gy) => LA(gx, gy, 1.2), [Lmin, Lmax]);
    const hx = x + wp[0][0], phase = ((t * 0.02 * f0) % 1);
    rings.forEach((r, k) => { const rad = (k + phase) * lam; r.visible = p.hubOn && rad < 60; r.scale.set(rad, rad, rad); r.position.set(hx, 0, 0.05); r.material.opacity = 0.6 * Math.max(0, 1 - rad / 60); });
    const vals = probes.map(q => { const L = LA(q.x, q.y, 1.2); q.max = Math.max(q.max, L); return L; });
    Hd.set([['Vehicle x', fmt(x, 3) + ' m'], ['Speed', fmt(p.speed, 3) + ' km/h'], ...probes.map((q, i) => [q.n, `${fmt(vals[i], 3)} (max ${fmt(q.max, 3)}) dB(A)`]), ['Hub tone f / λ', p.hubOn ? `${fmt(f0, 3)} Hz / ${fmt(lam, 3)} m` : 'off']]);
  } });
  return { dispose: () => pb.dispose() };
}

// ---------------- 2-D animation: top-view SPL map during a pass-by ----------------
async function acoustic2D(el, o) {
  const { p, ctx, S1, S } = o;
  const [{ anim2d }, { cmap }] = await Promise.all([import('../core/sim3d.js'), import('../core/chart.js')]);
  el.append(h('p', { class: 'small muted' }, 'Top view of the instantaneous A-weighted level at 1.2 m height from this suite’s solved 1 m source levels (tyre/road, wing trailing edges, body, hub tones) with spherical spreading and ground reflection; rings mark the hub-tone wavelength (propagation slowed for display); the 7.5 m pass-by microphone trace builds up underneath.'));
  const veh = ctx.vehicle, v = p.speed / 3.6, L1 = S1.LA, f0 = (S.tones && S.tones[0]?.f) || 100, lam = 343 / f0, nW = 3;
  const srcs = (x0) => { const L = []; for (const [dx, dy] of [[0.58 * veh.wheelbase, veh.track / 2], [0.58 * veh.wheelbase, -veh.track / 2], [-0.42 * veh.wheelbase, 0]]) { L.push([x0 + dx, dy, 0.05, L1.tyre - 10 * Math.log10(nW)]); if (p.hubOn) L.push([x0 + dx, dy, veh.wheelRadius, L1.hub - 10 * Math.log10(nW)]); } L.push([x0, 0, 2, L1.aero]); L.push([x0, 0, 0.8, L1.body]); return L; };
  const LA = (x, y, list) => 10 * Math.log10(list.reduce((a, q) => a + 10 ** ((q[3] + 3 - 20 * Math.log10(Math.max(0.5, Math.hypot(x - q[0], y - q[1], 1.2 - q[2])))) / 10), 0) || 1e-12);
  const Lmax = Math.max(...Object.values(L1).filter(Number.isFinite)) + 3, Lmin = Lmax - 45, xs = -45, xe = 45, T = (xe - xs) / Math.max(v, 0.5), trace = [];
  return anim2d(el, { duration: T, label: 'Pass-by time', height: 420, draw: (g, t, W, H, c) => {
    const x0 = xs + v * t, list = srcs(x0), nx = 90, ny = 40, mapH = H - 120, cw = W / nx, ch = mapH / ny;
    for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) { const X = -60 + 120 * (i + 0.5) / nx, Y = 25 - 50 * (j + 0.5) / ny, cc = cmap((LA(X, Y, list) - Lmin) / (Lmax - Lmin), 'inferno'); g.fillStyle = `rgb(${cc.map(Math.round).join(',')})`; g.fillRect(i * cw, 20 + j * ch, cw + 1, ch + 1); }
    const PX = (X) => (X + 60) / 120 * W, PY = (Y) => 20 + (25 - Y) / 50 * mapH;
    if (p.hubOn) { const ph = (t * 0.02 * f0) % 1; g.strokeStyle = 'rgba(56,189,248,.7)'; for (let k = 0; k < 18; k++) { const r = (k + ph) * lam; if (r > 60) break; g.beginPath(); g.ellipse(PX(x0 + 0.58 * veh.wheelbase), PY(0), r / 120 * W, r / 50 * mapH, 0, 0, 7); g.stroke(); } }
    g.fillStyle = '#e5e7eb'; g.fillRect(PX(x0 - veh.length * 0.42), PY(veh.width / 2), veh.length / 120 * W, veh.width / 50 * mapH);
    g.fillStyle = '#22d3ee'; g.beginPath(); g.arc(PX(0), PY(-7.5), 5, 0, 7); g.fill();
    const Lm = LA(0, -7.5, list); trace.push([t, Lm]); while (trace.length && trace[0][0] > t) trace.shift(); if (trace.length > 1 && trace[trace.length - 2][0] > t) trace.length = 0;
    const ty = H - 90, th = 80; g.strokeStyle = c.mute; g.strokeRect(0, ty, W, th); g.strokeStyle = '#22d3ee'; g.lineWidth = 2; g.beginPath(); trace.forEach(([tt, L], i) => { const X = tt / T * W, Y = ty + th - (L - Lmin) / (Lmax - Lmin) * th; i ? g.lineTo(X, Y) : g.moveTo(X, Y); }); g.stroke();
    g.fillStyle = c.fg; g.fillText(`x = ${fmt(x0, 3)} m · L_A at 7.5 m mic = ${fmt(Lm, 3)} dB(A) · max ${fmt(Math.max(...trace.map(q => q[1])), 3)} dB(A) · colour ${fmt(Lmin, 3)}–${fmt(Lmax, 3)} dB(A)`, 8, 14);
  } });
}
