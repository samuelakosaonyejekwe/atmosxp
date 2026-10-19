// Suite 17 — Direct-rim metamaterial thermomechanical wheel-hub propulsion (MATERA-P).
import * as TH from '../core/thermo.js';
import { linspace, fmt, range, trapz, mean, max, min, G0, KELVIN } from '../core/num.js';
import { roadLoad, kmh } from '../core/physics.js';
import { MATERIALS, get } from '../core/materials.js';
import { h } from '../core/ui.js';
import { matchAlloy, capStrain } from './s15-niti.js';
import { evaluateActuator } from './s16-actuators.js';
const POLY = MATERIALS.filter(m => m.uses?.includes('actuator') && m.cls !== 'sma');

const SMAS = MATERIALS.filter(m => m.cls === 'sma');
const wrap = (a) => ((a % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);

// Periodic thermomechanical response of ONE cartridge travelling round the wheel (all cartridges identical, phase-shifted)
export function simulateCartridge(mat, q, { revs = 4, nTheta = 720 } = {}) {
  const { Th, Tc, omega, a, lobes, c2, hotSector, sectorCenter, dpre, A, L, t, kf, hc, wFace } = q;
  const s = new TH.SMA(mat); s.xi = 1;
  const dphi = 2 * Math.PI / nTheta, dt = omega > 0 ? dphi / omega : 1e-3, m = mat.rho * A * L, As = wFace * L;
  const inHot = (phi) => { const psi = wrap(lobes * phi), d = Math.abs(((psi - sectorCenter + 3 * Math.PI) % (2 * Math.PI)) - Math.PI); return d <= hotSector / 2; };
  const delta = (phi) => dpre + a * (1 + Math.cos(lobes * phi) + c2 * Math.cos(2 * lobes * phi));
  const ddelta = (phi) => -a * lobes * (Math.sin(lobes * phi) + 2 * c2 * Math.sin(2 * lobes * phi));
  const force = (d) => { let F = 0; for (let k = 0; k < 2; k++) { const E = s.E(); F = Math.max(0, A * E * (d / L - mat.epsL * s.xi) / (1 + A * E / (kf * L))); s.update(T, F / A / 1e6); } return F; };
  let T = Tc; for (const TT of linspace(Tc + 40, Tc, 30)) s.update(TT, 0); // start cold, fully martensitic history
  let rec, Qin = 0, Qout = 0, Wel = 0, prevDe = null, prevF = 0, revsDone = 0;
  for (let r = 0; r < Math.max(revs, 2) + 10; r++) {
    rec = { phi: [], T: [], xi: [], F: [], d: [], de: [], tau: [], hot: [] }; Qin = 0; Qout = 0; Wel = 0; const T0 = T, xi0r = s.xi;
    for (let k = 0; k < nTheta; k++) {
      const phi = k * dphi, hot = inHot(phi), Tenv = hot ? Th : Tc;
      const dq = hc * As * (Tenv - T) * dt; T += dq / (m * mat.cp);
      const xi0 = s.xi, F = force(delta(phi)); const dxi = s.xi - xi0; T += mat.Lh * dxi / mat.cp; // latent: exothermic forward, endothermic reverse
      const de = delta(phi) - F / kf; // element extension
      if (prevDe !== null) { const dW = -0.5 * (F + prevF) * (de - prevDe); T -= dW / (m * mat.cp); Wel += dW; } // first-law work term
      if (dq > 0) Qin += dq; else Qout -= dq; rec.phi.push(phi); rec.T.push(T); rec.xi.push(s.xi); rec.F.push(F); rec.d.push(delta(phi)); rec.de.push(de); rec.tau.push(-F * ddelta(phi)); rec.hot.push(hot);
      prevDe = de; prevF = F;
    }
    revsDone = r + 1;
    // periodic once the state repeats: temperature and phase back to start-of-revolution values
    if (r + 1 >= revs && Math.abs(T - T0) < 1e-3 * Math.max(1, Th - Tc) && Math.abs(s.xi - xi0r) < 1e-3) break;
  }
  const Wtau = trapz(rec.tau.concat(rec.tau[0]), rec.phi.concat(2 * Math.PI)); // ∮τ dφ
  const Wfd = (() => { let w = 0; const n = rec.F.length; for (let i = 0; i < n; i++) { const j = (i + 1) % n; w += -0.5 * (rec.F[i] + rec.F[j]) * (rec.d[j] - rec.d[i]); } return w; })();
  return { rec, Wtau, Wfd, Wel, Qin, Qout, revsDone, mass: m, Fmax: max(rec.F), Tmax: max(rec.T), Tmin: min(rec.T), dxi: max(rec.xi) - min(rec.xi), inHot, delta, ddelta };
}
// Static torque at stand-still (steady sector temperatures) for an initial wheel angle θ
export function staticTorque(mat, q, theta, N) {
  let tau = 0, Fs = 0;
  for (let i = 0; i < N; i++) {
    const phi = wrap(theta + 2 * Math.PI * i / N), psi = wrap(q.lobes * phi), dd = Math.abs(((psi - q.sectorCenter + 3 * Math.PI) % (2 * Math.PI)) - Math.PI), hot = dd <= q.hotSector / 2;
    const s = new TH.SMA(mat); s.xi = 1; const T = hot ? q.Th : q.Tc; for (const TT of linspace(q.Tc + 40, q.Tc, 20)) s.update(TT, 0); if (hot) for (const TT of linspace(q.Tc, q.Th, 30)) s.update(TT, 0);
    const d = q.dpre + q.a * (1 + Math.cos(q.lobes * phi) + q.c2 * Math.cos(2 * q.lobes * phi)); let F = 0;
    for (let k = 0; k < 4; k++) { const E = s.E(); F = Math.max(0, q.A * E * (d / q.L - mat.epsL * s.xi) / (1 + q.A * E / (q.kf * q.L))); s.update(T, F / q.A / 1e6); }
    tau += -F * (-q.a * q.lobes * (Math.sin(q.lobes * phi) + 2 * q.c2 * Math.sin(2 * q.lobes * phi))); Fs += F;
  }
  return { tau, Fsum: Fs };
}

export default {
  description: 'The decisive experimental mechanism: NiTi actuator cartridges rotating with the wheel past fixed hot and cold thermal sectors and pressing on a stationary reaction cam. Solves each cartridge’s heat transfer, phase transformation, force and cam reaction through complete revolutions, integrates torque over 360° including reset work, rolling-contact friction and bearing loss, tests self-starting at every initial angle, checks heat demand against the environmental supply, and evaluates centrifugal, containment, contact and fatigue limits.',
  literature: 'shape memory alloy heat engine rotary wheel cam mechanism torque thermal cycling',
  sustain: true, credibility: 'Analytical screening',
  equations: [
    'Newton–Euler rotational dynamics and virtual work: τ = −F·dδ/dφ for a cartridge whose imposed extension δ(φ) follows the stationary cam profile',
    'Cam–follower kinematics with harmonic cam profile δ(φ) = δ₀ + a[1 + cos nφ + c₂ cos 2nφ]; series compliant force-transfer flexure (stiffness k_f)',
    'Unilateral (compression-free) contact condition F ≥ 0; rolling-contact friction torque μ_r F r_c; Hertz line-contact pressure p_max = √(F E*/(π R′ ℓ))',
    'Complete SMA thermomechanical cycle per cartridge (Suite 15 constitutive law) driven by angle-dependent thermal boundary conditions and residence time t = Δφ/ω',
    'Transient lumped heat balance with interface conductance, latent heat of transformation and first-law work term: m c_p dT = h_cA(T_sector − T)dt + m L_h dξ − dW',
    'Net work per revolution W = ∮τ dθ, mean wheel power P = ⟨τω⟩, torque ripple; heat demand Q = N·f·∮dQ_in; Carnot bound on P/Q',
    'Centrifugal load m ω² r and containment-ring hoop stress σ = ρω²r² + N m ω² r/(2πA); thermal-expansion interference; Basquin/functional fatigue',
  ],
  methods: ['Explicit angular marching (720 steps/rev) over 4 revolutions to a periodic state; last revolution reported', 'Superposition of N phase-shifted identical cartridges for total wheel torque', 'Quasi-static stand-still torque scan over all initial angles for self-starting', 'Speed sweep (wheel power vs road demand) and hot-sector phasing optimisation'],
  assumptions: ['Identical cartridges; cam and sectors rigidly fixed to the axle (reaction path explicit)', 'Rate-independent transformation; uniform lamella temperature (Biot < 0.1)', 'Heat supply per sector limited by the interface conductance; global environmental supply from Suite 14 in the atmospheric mode', 'Rolling-element follower (no sliding) — sliding shoes would multiply friction'],
  ibc: {
    initial: ['Wheel angular position and velocity (prescribed speed or standstill)', 'Cartridge temperatures at T_c and fully martensitic phase state (consistent cooling history)', 'Preload extension δ₀, cam contact states, flexure deformation', 'Buffer energy and applied load torque (from Suite 18/20 when coupled)'],
    boundary: ['Fixed axle-mounted reaction cam (stationary frame)', 'Angle-dependent hot-sector and cold-sector thermal contact (residence-time dependent)', 'Normal contact with friction between cam and compliant shoes', 'Bearing resistance (Suite 10) and road-load reaction torque (vehicle model)', 'Three experiment modes: actuator-only, hub-only with imposed ΔT (labelled), full atmospheric with environmental supply'],
  },
  cvv: {
    calibration: ['Torque–angle measurements on a stationary-axle/rotating-wheel dynamometer', 'Cam reaction forces, cartridge temperatures, contact friction', 'Startup torque at controlled initial angles'],
    verification: ['Torque-integrated work ∮τdθ equals force–displacement work −∮F dδ', 'First law over a periodic cycle: Q_in − Q_out = W', 'Second law: W/Q_in ≤ η_Carnot', 'Periodicity (temperature/phase repeat after the transient revolutions)'],
    validation: ['Instrumented hub dynamometer under externally imposed heating, solar-only and night atmosphere-only operation; continuous positive net work and repeatable self-starting must be demonstrated'],
  },
  formats: { import: ['CSV torque–angle / dynamometer logs', 'XLSX', 'JSON'], export: ['CSV torque–angle and cartridge histories', 'STEP AP242 / STL / glTF hub assembly (Suite 02)', 'PNG/SVG'] },
  params: [
    { key: 'mode', label: 'Experiment mode', type: 'select', group: 'Experiment', default: 'atmospheric', options: [{ value: 'actuator', label: 'Actuator-only (single cartridge cycle)' }, { value: 'hub', label: 'Hub-only, imposed laboratory ΔT' }, { value: 'atmospheric', label: 'Full atmospheric (coupled supply)' }] },
    { key: 'hubActuator', label: 'Hub actuator technology', type: 'select', group: 'Actuator', default: 'auto', options: [{ value: 'auto', label: 'Auto — Suite 16 matched-condition winner if it beats NiTi' }, { value: 'sma', label: 'Shape-memory alloy (grade below)' }, ...POLY.map(m => ({ value: m.id, label: m.name }))] },
    { key: 'tcpMass', label: 'Polymer-muscle mass per cartridge', unit: 'g', type: 'number', min: 1, max: 500, step: 1, group: 'Actuator', default: 30 },
    { key: 'grade', label: 'Actuator alloy', type: 'select', group: 'Actuator', default: 'nitcu', options: SMAS.map(m => ({ value: m.id, label: m.name })), auto: (c) => { const o = c.upstream('s15'); return o?.grade ? { value: o.grade, prov: 'coupled' } : { value: c.store.project.materials?.actuator || 'nitcu', prov: 'library' }; } },
    { key: 'Th', label: 'Hot-sector temperature', unit: '°C', type: 'number', min: -20, max: 200, step: 0.5, group: 'Thermal sectors', default: 25, auto: (c) => { const o = c.upstream('s14'); return o ? { value: +o.ThotIface_C.toFixed(2), prov: 'coupled' } : { value: +((c.env?.T ?? 20) + 5).toFixed(1), prov: 'hypothetical' }; } },
    { key: 'Tc', label: 'Cold-sector temperature', unit: '°C', type: 'number', min: -40, max: 150, step: 0.5, group: 'Thermal sectors', default: 20, auto: (c) => { const o = c.upstream('s14'); return o ? { value: +o.TcoldIface_C.toFixed(2), prov: 'coupled' } : { value: +(c.env?.T ?? 20).toFixed(1), prov: 'hypothetical' }; } },
    { key: 'hotSector', label: 'Hot-sector angle', unit: '°', type: 'range', min: 30, max: 300, step: 5, group: 'Thermal sectors', default: 180 },
    { key: 'sectorCenter', label: 'Hot-sector centre (cam phase)', unit: '°', type: 'range', min: 0, max: 359, step: 1, group: 'Thermal sectors', default: 80, help: '90° aligns the hot sector with the falling side of the cam; thermal lag shifts the optimum earlier (see phasing sweep).' },
    { key: 'autoKf', label: 'Optimise flexure stiffness automatically', type: 'bool', group: 'Cam', default: true, help: 'Maximises net work after friction while keeping peak alloy stress ≤ 0.8 σ_y.' },
    { key: 'autoSector', label: 'Optimise hot-sector angle automatically', type: 'bool', group: 'Thermal sectors', default: true, help: 'Revision 2 (D7): sector angle and cam phase chosen jointly for maximum net work per revolution.' },
    { key: 'matchAlloy', label: 'Use transformation temperatures matched to the sectors', type: 'bool', group: 'Actuator', default: true, help: 'Same composition tuning as Suite 15 (D5).' },
    { key: 'autoPhase', label: 'Optimise hot-sector phasing automatically', type: 'bool', group: 'Thermal sectors', default: true, help: 'Generated design: selects the sector centre that maximises net work for the current speed and ΔT.' },
    { key: 'hc', label: 'Sector interface conductance', unit: 'W/m²K', type: 'number', min: 200, max: 100000, step: 100, group: 'Thermal sectors', default: 8000 },
    { key: 'vKmh', label: 'Vehicle speed', unit: 'km/h', type: 'range', min: 0, max: 60, step: 1, group: 'Operating point', default: 20 },
    { key: 'R', label: 'Wheel effective radius', unit: 'm', type: 'number', min: 0.15, max: 0.5, step: 0.005, group: 'Hub geometry', default: 0.30, auto: (c) => ({ value: c.vehicle.wheelRadius, prov: 'coupled' }) },
    { key: 'nCart', label: 'Cartridges per driven wheel', type: 'number', min: 1, max: 96, step: 1, group: 'Hub geometry', default: 24 },
    { key: 'nWheels', label: 'Driven wheels', type: 'number', min: 1, max: 3, step: 1, group: 'Hub geometry', default: 2, auto: (c) => ({ value: c.vehicle.driveWheels || 2, prov: 'coupled' }) },
    { key: 'rc', label: 'Cam / cartridge radius', unit: 'm', type: 'number', min: 0.04, max: 0.25, step: 0.002, group: 'Hub geometry', default: 0.126 },
    { key: 'camAmp', label: 'Cam amplitude a', unit: 'mm', type: 'number', min: 0.1, max: 10, step: 0.1, group: 'Cam', default: 2.4 },
    { key: 'lobes', label: 'Cam lobes per revolution', type: 'number', min: 1, max: 4, step: 1, group: 'Cam', default: 1 },
    { key: 'c2', label: 'Second-harmonic content', type: 'range', min: -0.4, max: 0.4, step: 0.02, group: 'Cam', default: 0, expert: true },
    { key: 'dpre', label: 'Preload extension δ₀', unit: 'mm', type: 'number', min: 0, max: 10, step: 0.1, group: 'Cam', default: 0.4 },
    { key: 'kf', label: 'Flexure stiffness k_f', unit: 'N/mm', type: 'number', min: 20, max: 50000, step: 10, group: 'Cam', default: 300, help: 'The compliant shoe limits actuator stress; too stiff and the hot stroke overloads the alloy.' },
    { key: 't', label: 'NiTi lamella thickness', unit: 'mm', type: 'number', min: 0.05, max: 3, step: 0.05, group: 'Actuator', default: 0.3, auto: (c) => { const o = c.upstream('s15'); return o?.elem ? { value: o.elem.t, prov: 'coupled' } : undefined; } },
    { key: 'w', label: 'NiTi lamella width', unit: 'mm', type: 'number', min: 1, max: 50, step: 0.5, group: 'Actuator', default: 12, auto: (c) => { const o = c.upstream('s15'); return o?.elem ? { value: o.elem.w, prov: 'coupled' } : undefined; } },
    { key: 'L', label: 'NiTi active length', unit: 'mm', type: 'number', min: 5, max: 300, step: 1, group: 'Actuator', default: 120, auto: (c) => { const o = c.upstream('s15'); return o?.elem ? { value: o.elem.L, prov: 'coupled' } : undefined; } },
    { key: 'mu', label: 'Rolling-contact friction coefficient', type: 'number', min: 0, max: 0.3, step: 0.001, group: 'Losses', default: 0.002, help: 'Roller follower ≈ 0.001–0.003; sliding shoe ≈ 0.05–0.15.' },
    { key: 'cartMass', label: 'Cartridge total mass', unit: 'g', type: 'number', min: 5, max: 1000, step: 5, group: 'Structure', default: 60 },
    { key: 'ringT', label: 'Containment ring section t × w', unit: 'mm²', type: 'number', min: 10, max: 2000, step: 10, group: 'Structure', default: 90 },
    { key: 'clear', label: 'Radial thermal clearance', unit: 'mm', type: 'number', min: 0.01, max: 2, step: 0.01, group: 'Structure', default: 0.2, expert: true },
  ],
  presets: [{ label: 'Lab ΔT 50 K (hub-only)', values: { mode: 'hub', Th: 70, Tc: 20 } }, { label: 'Calm night 5 K', values: { mode: 'hub', Th: 20, Tc: 15 } }, { label: '40 km/h duty point', values: { vKmh: 40 } }, { label: 'Sliding shoes (μ = 0.08)', values: { mu: 0.08 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
    // Actuator selection (owner decision): use the Suite 16 matched-condition winner when it beats the best NiTi
    const s16u = ctx.upstream('s16');
    // "Beats" is judged by delivered mechanical power under the ACTUAL heat supply: P = min(Q_avail·η, m_active·P/m),
    // because the hub is heat-limited (power density alone would favour slow, inefficient actuators).
    let polyId = null, gradeId = p.grade, selNote = 'user selection';
    const Qsel = Math.max(0, ctx.upstream('s14')?.Qhot_W ?? 0), mBudget = p.nCart * p.nWheels * (p.tcpMass / 1000);
    const delivered = (r) => Math.min(Qsel * Math.max(0, r.eta || 0), mBudget * Math.max(0, r.P_kg || 0));
    if ((p.hubActuator || 'auto') === 'auto' && s16u?.ranking?.length) { const cands = s16u.ranking.filter(r => r.net).map(r => ({ ...r, Pdel: delivered(r) })).sort((a, b) => b.Pdel - a.Pdel); const w = cands[0]; if (w) { if (w.cls === 'sma') gradeId = w.id; else polyId = w.id; selNote = `auto: highest delivered power ${fmt(w.Pdel, 3)} W = min(${fmt(Qsel, 3)} W × η ${fmt(100 * w.eta, 3)} %, ${fmt(mBudget, 3)} kg × ${fmt(w.P_kg, 3)} W/kg) among Suite 16 matched candidates`; } }
    else if (p.hubActuator && !['auto', 'sma'].includes(p.hubActuator)) polyId = p.hubActuator;
    if (polyId) return runPolymerHub(p, ctx, get(polyId), s16u, selNote);
    const mat0 = get(gradeId) || get(p.grade); if (!mat0) throw new Error('Unknown alloy');
    const s15u = ctx.upstream('s15');
    let alloyShift = 0;
    if (p.matchAlloy !== false) alloyShift = s15u?.grade === p.grade && Number.isFinite(s15u?.tShift_K) ? s15u.tShift_K : matchAlloy(mat0, { Th: p.Th, Tc: p.Tc, sH: 150, sL: 20 }).shift;
    const mat = capStrain({ ...mat0, Ms: mat0.Ms + alloyShift, Mf: mat0.Mf + alloyShift, As: mat0.As + alloyShift, Af: mat0.Af + alloyShift }, (s15u?.epsAmp ?? 1) / 100);
    if (p.Th <= p.Tc) throw new Error('Hot sector must be warmer than the cold sector.');
    const veh = ctx.vehicle, s10 = ctx.upstream('s10'), s14 = ctx.upstream('s14');
    const v = kmh(Math.max(p.vKmh, 0.5)), omega = v / p.R, rpm = omega * 60 / (2 * Math.PI), fRev = omega / (2 * Math.PI);
    const A = p.t * p.w * 1e-6, L = p.L / 1000;
    const q = { Th: p.Th, Tc: p.Tc, omega, a: p.camAmp / 1000, lobes: p.lobes, c2: p.c2, hotSector: p.hotSector * Math.PI / 180, sectorCenter: p.sectorCenter * Math.PI / 180, dpre: p.dpre / 1000, A, L, t: p.t / 1000, kf: p.kf * 1000, hc: p.hc, wFace: p.w / 1000 };
    ctx.progress(0.05, 'phasing');
    // D7: flexure stiffness chosen for maximum net cartridge work after contact friction, with peak alloy stress ≤ 0.8 σ_y
    let kfUsed = p.kf;
    if (p.autoKf !== false) { let best = null; for (const kf of [40, 60, 90, 130, 180, 250, 350, 500, 750, 1000, 1500, 2500]) { const Sx = simulateCartridge(mat, { ...q, kf: kf * 1000 }, { revs: 2, nTheta: 180 }); const net = Sx.Wtau - mean(Sx.rec.F.map(F => p.mu * F * p.rc)) * 2 * Math.PI; if (Sx.Fmax / A / 1e6 <= 0.8 * (mat.sy || 450) && (!best || net > best.net)) best = { kf, net }; } if (best) { kfUsed = best.kf; q.kf = best.kf * 1000; } await ctx.yield(); }
    // D7: joint optimisation of hot-sector angle and cam phase (coarse scan, then the chosen sector at full resolution)
    const centers = linspace(0, 345, 24);
    let usedSector = p.hotSector; const sectorScan = [];
    if (p.autoSector !== false) { let best = -Infinity; for (const sa of [90, 120, 150, 180, 210, 240, 270]) { const qq = { ...q, hotSector: sa * Math.PI / 180 }; const Wb = Math.max(...centers.filter((_, i) => i % 2 === 0).map(cd => simulateCartridge(mat, { ...qq, sectorCenter: cd * Math.PI / 180 }, { revs: 2, nTheta: 180 }).Wtau)); sectorScan.push({ sa, W: Wb * p.nCart }); if (Wb > best) { best = Wb; usedSector = sa; } await ctx.yield(); } q.hotSector = usedSector * Math.PI / 180; }
    const phaseW = centers.map(cdeg => simulateCartridge(mat, { ...q, sectorCenter: cdeg * Math.PI / 180 }, { revs: 3, nTheta: 360 }).Wtau * p.nCart);
    let usedCenter = p.sectorCenter;
    if (p.autoPhase) { usedCenter = centers[phaseW.indexOf(Math.max(...phaseW))]; q.sectorCenter = usedCenter * Math.PI / 180; }
    ctx.progress(0.12, 'cartridge cycle');
    const S = simulateCartridge(mat, q);
    const N = p.nCart, nT = S.rec.phi.length, tau1 = S.rec.tau;
    const fricC = S.rec.F.map(F => p.mu * F * p.rc);
    const bearingW = s10?.bearingLoss_W ?? veh.bearingLoss_W ?? 3, tauBear = bearingW / Math.max(omega, 1e-3) / p.nWheels;
    const tauTot = range(nT).map(k => { let s = 0, f = 0; for (let i = 0; i < N; i++) { const j = (k + Math.round(i * nT / N)) % nT; s += tau1[j]; f += fricC[j]; } return s - f - tauBear; });
    const tauGross = range(nT).map(k => { let s = 0; for (let i = 0; i < N; i++) s += tau1[(k + Math.round(i * nT / N)) % nT]; return s; });
    const tauMean = mean(tauTot), Wrev = tauMean * 2 * Math.PI, ripple = (max(tauTot) - min(tauTot)) / Math.max(1e-9, Math.abs(tauMean));
    const PwheelGross = tauMean * omega * p.nWheels; // mechanical power at the wheels (thermal supply unconstrained)
    const Qdemand = N * p.nWheels * S.Qin * fRev, etaC = 1 - (p.Tc + KELVIN) / (p.Th + KELVIN);
    let Qavail = Infinity, supplyNote = 'Imposed laboratory heat supply (unlimited, labelled).';
    if (p.mode === 'atmospheric') { if (s14) { Qavail = Math.max(0, s14.Qhot_W) + Math.max(0, s14.Qleak_W || 0) * 0; supplyNote = `Suite 14 environmental supply (${s14.case} case): ${fmt(s14.Qhot_W, 3)} W.`; } else { const s13 = ctx.upstream('s13'); Qavail = s13 ? Math.max(0, (ctx.env?.isDay ? s13.Pabs_W : s13.Pcool_W) || 0) : 0; supplyNote = s13 ? 'Suite 13 collector/radiator capacity (Suite 14 not run).' : 'No environmental supply computed (run Suites 13–14): supply taken as 0 W.'; } }
    const supplyFrac = Qdemand > 0 ? Math.min(1, Qavail / Qdemand) : 1;
    const Psustain = Math.max(0, PwheelGross) * supplyFrac;
    const demand = roadLoad(v, veh).P, demand20 = roadLoad(kmh(20), veh).P, demand40 = roadLoad(kmh(40), veh).P;
    const deficit = Math.max(0, demand - Psustain);
    await ctx.yield();
    ctx.progress(0.35, 'speed sweep');
    // speed sweep: wheel power (thermal supply limited in atmospheric mode) vs road demand
    const speeds = linspace(2, 60, 16), sweep = speeds.map(vk => { const om = kmh(vk) / p.R, Sx = simulateCartridge(mat, { ...q, omega: om }, { revs: 3, nTheta: 360 }); const fr = Sx.rec.F.map(F => p.mu * F * p.rc); const tm = mean(range(Sx.rec.tau.length).map(k => Sx.rec.tau[k] - fr[k])) * N - bearingW / om / p.nWheels; const Pg = Math.max(0, tm * om * p.nWheels), Qd = N * p.nWheels * Sx.Qin * om / (2 * Math.PI); return { P: p.mode === 'atmospheric' ? Pg * Math.min(1, Qd > 0 ? Qavail / Qd : 1) : Pg, D: roadLoad(kmh(vk), veh).P }; });
    let vmax = 0; for (let i = 0; i < speeds.length; i++) if (sweep[i].P >= sweep[i].D) vmax = speeds[i];
    await ctx.yield();
    ctx.progress(0.6, 'start-up');
    // self-starting at all initial angles (one cartridge pitch covers all distinct positions, scan a full turn for sector edges)
    const thetas = linspace(0, 2 * Math.PI, 145), stat = thetas.map(th => staticTorque(mat, q, th, N));
    const Crr = s10?.Crr ?? veh.Crr, tauRoll = Crr * veh.mass * G0 * p.R / p.nWheels;
    const tauStatic = stat.map(s => s.tau - 1.5 * p.mu * s.Fsum * p.rc), tauResist = tauRoll + bearingW / 5 / p.nWheels;
    const dead = tauStatic.filter(t => t < tauResist).length / tauStatic.length, selfStart = dead === 0;
    // D7: start-up from the elastic buffer — energy to turn every driven wheel through one cartridge pitch against the
    // starting resistance plus accelerate the vehicle to 2 km/h, with a factor 3 margin, must be available in Suite 18.
    const s18u = ctx.upstream('s18'), Estart = 3 * (tauResist * p.nWheels * 2 * Math.PI / N + 0.5 * veh.mass * kmh(2) ** 2), bufE = s18u?.Euse_J ?? 10000; // Revision 2 buffer design (D8: 10 kJ usable) until Suite 18 is run
    const bufferStart = !selfStart && bufE >= Estart, startupOK = selfStart || bufferStart;
    // structural checks
    const mc = p.cartMass / 1000, Fcent = mc * omega ** 2 * p.rc, ring = get(ctx.store.material('cartridgeHousing', 'ti-6al4v')), Aring = p.ringT * 1e-6;
    const hoop = (ring.rho * omega ** 2 * p.rc ** 2 + N * mc * omega ** 2 * p.rc / (2 * Math.PI * Aring)) / 1e6, SFring = ring.sy / Math.max(hoop, 1e-9);
    const cam = get(ctx.store.material('cam', 'st-17-4')), Estar = 1 / ((1 - 0.09) / (cam.E * 1e9) + (1 - 0.09) / 210e9), pHertz = Math.sqrt(S.Fmax * Estar / (Math.PI * 0.006 * 0.010)) / 1e6, pAllow = 1.8 * cam.sy;
    const thermalGrowth = (mat.alpha || 11) * 1e-6 * (p.Th - p.Tc) * p.rc * 1000;
    const sPeak = S.Fmax / A / 1e6, sAllow = mat.sy || 500;
    const fat = TH.smaFatigue(mat.epsL * S.dxi, S.Fmax / A / 1e6), lifeH = fat / (fRev * p.lobes) / 3600;
    // verification
    out.checks.push({ name: 'Torque-integrated work ∮τdφ vs force–displacement work −∮F dδ', value: S.Wtau, target: `${fmt(S.Wfd, 6)} J (±1 %)`, pass: Math.abs(S.Wtau - S.Wfd) <= 0.01 * Math.max(1e-6, Math.abs(S.Wfd)) + 1e-9 });
    out.checks.push({ name: 'Element work vs cam work (flexure energy returns over a cycle)', value: S.Wel, target: `${fmt(S.Wfd, 6)} J (±3 % or ±1 mJ)`, pass: Math.abs(S.Wel - S.Wfd) <= 0.03 * Math.abs(S.Wfd) + 1e-3 });
    out.checks.push({ name: 'First law over the periodic cycle: Q_in − Q_out = W', value: (S.Qin - S.Qout - S.Wel), target: `≈ 0 (Q_in = ${fmt(S.Qin, 4)} J)`, pass: Math.abs(S.Qin - S.Qout - S.Wel) < 0.05 * Math.max(S.Qin, 1e-6) + 0.02 });
    out.checks.push({ name: 'Second law: W/Q_in ≤ η_Carnot', value: S.Qin > 0 ? S.Wel / S.Qin : 0, target: `≤ ${fmt(etaC, 4)}`, pass: S.Qin <= 0 || S.Wel / S.Qin <= etaC + 1e-6 });
    const S8 = simulateCartridge(mat, q, { revs: 6, nTheta: 720 }); out.checks.push({ name: 'Periodicity (4 vs 6 revolutions)', value: Math.abs(S8.Wtau - S.Wtau), target: '< 2 % of |W|', pass: Math.abs(S8.Wtau - S.Wtau) <= 0.02 * Math.abs(S.Wtau) + 1e-4 });
    const Sh = simulateCartridge(mat, q, { revs: 4, nTheta: 1440 }); out.checks.push({ name: 'Angular-step convergence (720 → 1440 steps/rev)', value: Math.abs(Sh.Wtau - S.Wtau), target: '< 3 % of |W|', pass: Math.abs(Sh.Wtau - S.Wtau) <= 0.03 * Math.abs(S.Wtau) + 1e-4 });
    // KPIs
    const K = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    const isAct = p.mode === 'actuator';
    K('Wheel speed', rpm, 'rpm', `${p.vKmh} km/h on a ${p.R} m wheel (spec: 20 km/h ≈ 177 rpm, 40 km/h ≈ 212 rpm at… see table).`, 'calc');
    K('Thermal cycle frequency', fRev * p.lobes, 'Hz', `Residence time per sector ≈ ${fmt(usedSector / 360 / fRev, 3)} s hot, ${fmt((360 - usedSector) / 360 / fRev, 3)} s cold.`, 'calc');
    K('Cartridge temperature swing', `${fmt(S.Tmin, 4)} → ${fmt(S.Tmax, 4)}`, '°C', `Sectors at ${p.Tc} / ${p.Th} °C. A small swing means the residence time is too short for heat transfer.`, 'calc');
    K('Phase-fraction swing Δξ', S.dxi, '', 'How much of each cartridge actually transforms per revolution.', 'calc', S.dxi > 0.3 ? 'ok' : S.dxi > 0.05 ? 'warn' : 'bad');
    K('Net work per cartridge per rev', S.Wtau, 'J', 'After reset work (cold-side compression) — friction excluded. Spec development target ≈ 3.54 J.', 'calc', S.Wtau >= 3.54 ? 'ok' : S.Wtau > 0 ? 'warn' : 'bad');
    if (!isAct) {
      K('Net work per wheel per revolution', Wrev, 'J', 'W = ∮τdθ for all cartridges minus rolling friction and bearing loss (spec: 170 J/rev for 600 W at 40 km/h).', 'calc', Wrev > 0 ? 'ok' : 'bad');
      K('Mean net wheel torque', tauMean, 'N·m', `Per driven wheel. Demand at this speed: ${fmt(demand / omega / p.nWheels, 3)} N·m.`, 'calc');
      K('Torque ripple (peak-to-peak / mean)', ripple, '', 'Large ripple causes vibration, noise and buffer cycling.', 'calc', ripple < 1 ? 'ok' : 'warn');
      K('Wheel power (thermal supply unconstrained)', PwheelGross, 'W', 'Mechanical output if heat could be supplied at the rate the cartridges absorb it.', 'calc');
      K('Heat demand of the hub', Qdemand, 'W', 'Heat the cartridges absorb from the hot sectors at this speed.', 'calc');
      K('Heat available', Number.isFinite(Qavail) ? Qavail : 'imposed (∞)', 'W', supplyNote, p.mode === 'atmospheric' ? (s14 ? 'coupled' : 'hypothetical') : 'user');
      K('Sustainable wheel power', Psustain, 'W', 'Wheel power after limiting by environmental heat supply.', 'calc', Psustain >= demand ? 'ok' : 'bad');
      K('Road-load demand at this speed', demand, 'W', `Ideal level-road wheel power (124 W at 20 km/h, 406 W at 40 km/h for the reference vehicle).`, 'calc');
      K('Power deficit', deficit, 'W', 'Missing power that must come from wind, stored energy or reduced speed.', 'calc', deficit > 0 ? 'bad' : 'ok');
      K('Maximum sustainable speed', vmax, 'km/h', 'Highest speed in the sweep where hub power meets road demand.', 'calc', vmax > 0 ? 'ok' : 'bad');
      K('Dead start-up positions', dead * 100, '% of angles', `Static torque below the starting resistance ${fmt(tauResist, 3)} N·m per wheel.`, 'calc', startupOK ? 'ok' : 'bad');
    }
    K('Peak actuator stress', sPeak, 'MPa', `Allowable ≈ ${sAllow} MPa for ${mat.name}; the compliant shoe (k_f) limits it.`, 'calc', sPeak < sAllow ? 'ok' : 'bad');
    K('Flexure stiffness used', kfUsed, 'N/mm', p.autoKf !== false ? 'Generated: maximum net work after friction with peak stress ≤ 0.8 σ_y.' : 'User-specified.', p.autoKf !== false ? 'generated' : 'user');
    K('Hot-sector angle used', usedSector, '°', p.autoSector !== false ? 'Generated: joint optimisation of sector angle and phase for maximum net work per revolution (D7).' : 'User-specified.', p.autoSector !== false ? 'generated' : 'user');
    K('Alloy transformation shift', alloyShift, 'K', s15u?.grade === p.grade ? 'Matched alloy from Suite 15.' : 'Matched with the Suite 15 rule (σ_H 150, σ_L 20 MPa).', 'generated');
    K('Start-up', selfStart ? 'self-starting' : bufferStart ? 'via buffer release' : 'NOT possible', '', `Static torque scan: ${fmt(dead * 100, 3)} % dead angles. Buffer start needs ${fmt(Estart, 3)} J; ${s18u ? 'Suite 18 provides' : 'Revision 2 buffer design (D8, Suite 18 not yet run) provides'} ${fmt(bufE, 4)} J.`, s18u ? 'coupled' : 'estimated', startupOK ? 'ok' : 'bad');
    K('Hot-sector centre used', usedCenter, '°', p.autoPhase ? 'Generated by the phasing optimiser.' : 'User-specified.', p.autoPhase ? 'generated' : 'user');
    K('Containment-ring hoop stress / SF', `${fmt(hoop, 3)} MPa / ${fmt(SFring, 3)}`, '', `${ring.name}; centrifugal load per cartridge ${fmt(Fcent, 3)} N.`, 'calc', SFring > 3 ? 'ok' : 'bad');
    K('Cam Hertz contact pressure', pHertz, 'MPa', `Roller Ø12 mm × 10 mm on ${cam.name}; allowable ≈ ${fmt(pAllow, 4)} MPa.`, 'calc', pHertz < pAllow ? 'ok' : 'bad');
    K('Actuator fatigue life', fat, 'cycles', `≈ ${fmt(lifeH, 3)} h of driving at this speed (estimate).`, 'estimated', fat > 1e8 ? 'ok' : 'bad');
    // verdicts
    out.verdicts.push({ test: 'Positive net work per revolution (after reset work and losses)', status: (isAct ? S.Wtau : Wrev) > 0 ? 'pass' : 'fail', detail: isAct ? `${fmt(S.Wtau, 4)} J per cartridge cycle.` : `${fmt(Wrev, 4)} J per wheel revolution; gross cartridge work ${fmt(mean(tauGross) * 2 * Math.PI, 4)} J, friction ${fmt(mean(fricC) * N * 2 * Math.PI, 4)} J, bearing ${fmt(tauBear * 2 * Math.PI, 4)} J.`, fix: 'Increase ΔT and residence time, phase the hot sector to the falling cam flank, use lower-hysteresis alloy, reduce friction.' });
    if (!isAct) {
      out.verdicts.push({ test: 'Start-up from every initial angle (self or buffer-assisted)', status: selfStart ? 'pass' : bufferStart ? 'pass' : 'fail', detail: bufferStart ? `Dead positions at ${fmt(dead * 100, 3)} % of angles are bridged by releasing ${fmt(Estart, 3)} J from the elastic buffer (${fmt(bufE, 4)} J available, Suite 18) — D7 start-up strategy.` : selfStart ? `Minimum static torque ${fmt(min(tauStatic), 3)} N·m > resistance ${fmt(tauResist, 3)} N·m.` : `${fmt(dead * 100, 3)} % of initial angles are dead positions (min ${fmt(min(tauStatic), 3)} N·m vs ${fmt(tauResist, 3)} N·m needed).`, fix: 'More cartridges / lobes to fill dead zones, larger hot sector, or a buffer-assisted first stroke (Suite 18).' });
      out.verdicts.push({ test: 'Thermal supply sustains the cycle frequency', status: p.mode !== 'atmospheric' ? 'warn' : supplyFrac >= 1 ? 'pass' : 'fail', detail: p.mode !== 'atmospheric' ? 'Imposed heat supply — does not demonstrate atmospheric-only operation.' : `Demand ${fmt(Qdemand, 3)} W vs available ${fmt(Qavail, 3)} W → ${fmt(supplyFrac * 100, 3)} % of required heat.` });
      out.verdicts.push({ test: `Meets road demand at ${p.vKmh} km/h`, status: Psustain >= demand ? 'pass' : 'fail', detail: Psustain >= demand ? `${fmt(Psustain, 3)} W ≥ ${fmt(demand, 3)} W.` : `Sustainable ${fmt(Psustain, 3)} W vs required ${fmt(demand, 3)} W — explicit power deficit ${fmt(deficit, 3)} W. The vehicle must slow, use wind or stored energy, or stop.` });
      out.verdicts.push({ test: 'Energy audit (no energy creation)', status: S.Qin <= 0 || S.Wel / S.Qin <= etaC + 1e-6 ? 'pass' : 'fail', detail: `Cycle efficiency ${fmt(S.Qin > 0 ? S.Wel / S.Qin * 100 : 0, 3)} % ≤ Carnot ${fmt(etaC * 100, 3)} %.` });
    }
    out.verdicts.push({ test: 'Structural integrity (containment, contact, fatigue)', status: SFring > 3 && pHertz < pAllow && fat > 1e8 && sPeak < sAllow ? 'pass' : SFring > 1.5 && pHertz < 1.2 * pAllow ? 'warn' : 'fail', detail: `Actuator peak stress ${fmt(sPeak, 3)}/${sAllow} MPa, ring SF ${fmt(SFring, 3)}, contact ${fmt(pHertz, 4)}/${fmt(pAllow, 4)} MPa, fatigue ${fmt(fat, 3)} cycles, thermal growth ${fmt(thermalGrowth, 3)} mm vs clearance ${p.clear} mm${thermalGrowth > p.clear ? ' — INTERFERENCE / SEIZURE RISK' : ''}.` });
    if (p.mode === 'hub') out.verdicts.push({ test: 'Experiment classification', status: 'info', detail: 'Hub-only experiment with an imposed, explicitly identified thermal supply: validates hub mechanics only, not atmospheric feasibility.' });
    // charts
    const deg = S.rec.phi.map(f => f * 180 / Math.PI), hotBands = [];
    { let st = null; S.rec.hot.forEach((hh, i) => { if (hh && st === null) st = deg[i]; if ((!hh || i === deg.length - 1) && st !== null) { hotBands.push({ type: 'xband', x0: st, x1: deg[i], color: '#ef4444', opacity: 0.1 }); st = null; } }); }
    if (!isAct) out.charts.push({ title: 'Net wheel torque vs wheel angle (one wheel)', type: 'line', wide: true, x: { label: 'Wheel angle', unit: '°', min: 0, max: 360 }, y: { label: 'Torque', unit: 'N·m' }, series: [{ name: 'Net torque (all cartridges − losses)', x: deg, y: tauTot, width: 2.5 }, { name: 'Gross cartridge torque', x: deg, y: tauGross, dash: '4 3' }, { name: 'One cartridge', x: deg, y: tau1, width: 1, color: '#94a3b8' }], annotations: [{ type: 'hline', y: tauMean, label: `mean ${fmt(tauMean, 3)} N·m`, color: '#22c55e' }, { type: 'hline', y: demand / omega / p.nWheels, label: 'road demand', color: '#ef4444' }, { type: 'hline', y: 0, color: '#64748b' }], explain: 'Torque about the axle produced by all phase-shifted cartridges pressing on the stationary cam. Positive peaks alone do not propel the car: only the mean (green) after reset work and friction counts, and it must exceed the red demand line.' });
    out.charts.push({ title: 'Cartridge temperature and martensite fraction around one revolution', type: 'line', x: { label: 'Cartridge angle', unit: '°', min: 0, max: 360 }, y: { label: 'Value' }, series: [{ name: 'Temperature (°C)', x: deg, y: S.rec.T }, { name: 'ξ × ΔT + T_c (scaled)', x: deg, y: S.rec.xi.map(x => p.Tc + x * (p.Th - p.Tc)), dash: '4 3' }], annotations: hotBands, explain: 'Shaded = hot sector. The cartridge only heats/cools within its residence time; if temperature never crosses the transformation band, ξ stays flat and no work is produced.' });
    out.charts.push({ title: 'Cartridge force–extension loop (work per revolution = enclosed area)', type: 'line', x: { label: 'Imposed extension δ', unit: 'mm' }, y: { label: 'Cartridge force', unit: 'N' }, series: [{ name: 'Cycle', x: S.rec.d.map(d => d * 1000), y: S.rec.F }], explain: 'Clockwise loop = net positive work delivered to the wheel: high force while the cam lets the cartridge contract (hot), low force while it is stretched (cold).' });
    if (!isAct) out.charts.push({ title: 'Hub power vs road demand across speed', type: 'line', x: { label: 'Vehicle speed', unit: 'km/h' }, y: { label: 'Power', unit: 'W' }, series: [{ name: p.mode === 'atmospheric' ? 'Sustainable hub power (supply-limited)' : 'Hub power (imposed supply)', x: speeds, y: sweep.map(r => r.P) }, { name: 'Road-load demand', x: speeds, y: sweep.map(r => r.D), color: '#ef4444' }, { name: 'Backup design targets', x: [10, 20, 30, 40, 60], y: [100, 200, 350, 600, 1500], mode: 'markers', color: '#a78bfa', r: 5 }], annotations: vmax ? [{ type: 'vline', x: vmax, label: `max ${vmax.toFixed(0)} km/h`, color: '#22c55e' }] : [], explain: 'Where the hub curve lies below the red demand curve the vehicle cannot hold that speed on hub power alone. Purple points are the specification’s 100–1500 W backup targets.' });
    if (!isAct) out.charts.push({ title: 'Self-starting: static torque vs initial wheel angle', type: 'line', x: { label: 'Initial wheel angle', unit: '°' }, y: { label: 'Static torque per wheel', unit: 'N·m' }, series: [{ name: 'Static torque (after static friction)', x: thetas.map(t => t * 180 / Math.PI), y: tauStatic }], annotations: [{ type: 'hline', y: tauResist, label: 'starting resistance', color: '#ef4444' }], explain: 'With the car stationary, cartridges sitting in the hot sector push on the cam. Any angle where the curve is below the red line is a dead position from which the wheel cannot start.' });
    if (sectorScan.length) out.charts.push({ title: 'Hot-sector angle optimisation (best phase at each angle)', type: 'line', x: { label: 'Hot-sector angle', unit: '°' }, y: { label: 'Gross work per wheel rev', unit: 'J' }, series: [{ name: 'max over phase', x: sectorScan.map(r => r.sa), y: sectorScan.map(r => r.W), mode: 'lines+markers' }], annotations: [{ type: 'vline', x: usedSector, label: 'selected' }], explain: 'A larger hot sector gives more heating time but less cooling time; the optimum balances both residence times against the cam geometry.' });
    out.charts.push({ title: 'Hot-sector phasing optimisation', type: 'line', x: { label: 'Hot-sector centre', unit: '°' }, y: { label: 'Gross work per wheel rev', unit: 'J' }, series: [{ name: 'W(N cartridges)', x: centers, y: phaseW }], annotations: [{ type: 'vline', x: usedCenter, label: p.autoPhase ? 'auto-selected' : 'current' }], explain: 'Placing the hot sector where the cam lets cartridges contract maximises work; thermal lag shifts the best phase.' });
    out.charts.push({ title: 'Heat demand vs available supply', type: 'bar', categories: ['Hub heat demand', 'Available heat', 'Wheel power (gross)', 'Road demand'], series: [{ name: 'W', values: [Qdemand, Number.isFinite(Qavail) ? Qavail : 0, PwheelGross, demand], colors: ['#f97316', '#22c55e', '#38bdf8', '#ef4444'] }], y: { label: 'Power', unit: 'W' }, showValues: true, explain: 'The hub cannot run faster than its heat supply allows; in atmospheric mode the available heat comes from Suites 13–14.' });
    // tables
    const P600 = 600, om40 = kmh(40) / p.R;
    out.tables.push({ title: 'Specification duty points recomputed for this geometry', cols: ['Quantity', 'This design', 'Specification reference'], rows: [['Wheel speed at 20 km/h', kmh(20) / p.R * 60 / (2 * Math.PI), '177 rpm'], ['Wheel speed at 40 km/h', om40 * 60 / (2 * Math.PI), '212 rpm'], ['Torque for 600 W at 40 km/h (total)', P600 / om40, '17.2 N·m'], ['Work per revolution for 600 W', P600 / (om40 / (2 * Math.PI)), '170 J'], [`Work per cartridge per rev (${N * p.nWheels} cartridges)`, P600 / (om40 / (2 * Math.PI)) / (N * p.nWheels), '3.54 J'], ['Average force over 5 mm stroke', P600 / (om40 / (2 * Math.PI)) / (N * p.nWheels) / 0.005, '708 N'], ['Cycle frequency at 40 km/h', om40 / (2 * Math.PI), '3.54 Hz'], ['Time per half-revolution sector', Math.PI / om40, '0.141 s'], ['Road demand 20 / 40 km/h', `${fmt(demand20, 3)} / ${fmt(demand40, 3)} W`, '124 / 406 W'], ['Carnot heat for 600 W at 293/288 K', 600 / (1 - 288 / 293), '35.2 kW']] });
    out.tables.push({ title: 'Energy accounting per wheel revolution', cols: ['Term', 'Value (J)'], rows: [['Gross cartridge work (N × ∮τdφ)', S.Wtau * N], ['Rolling-contact friction', -mean(fricC) * N * 2 * Math.PI], ['Bearing loss', -tauBear * 2 * Math.PI], ['Net work per wheel', Wrev], ['Heat absorbed (N cartridges)', S.Qin * N], ['Heat rejected (N cartridges)', S.Qout * N]] });
    out.tables.push({ title: 'Structural & durability checks', cols: ['Check', 'Value', 'Limit'], rows: [['Centrifugal load per cartridge', `${fmt(Fcent, 3)} N`, '—'], ['Containment ring hoop stress', `${fmt(hoop, 3)} MPa`, `σ_y ${ring.sy} MPa`], ['Peak cartridge force', `${fmt(S.Fmax, 4)} N`, '—'], ['Hertz contact pressure', `${fmt(pHertz, 4)} MPa`, `${fmt(pAllow, 4)} MPa`], ['Thermal radial growth', `${fmt(thermalGrowth, 3)} mm`, `clearance ${p.clear} mm`], ['Actuator fatigue', `${fmt(fat, 3)} cycles`, '≥ 10⁸ for vehicle life']] });
    // 3D animated hub (solved cartridge states vs wheel angle)
    out.views.push({ label: '3D hub drive (solved cycle)', icon: '⚙', render: (el) => view3dHub(el, { S, tauTot, N, nT, p: { ...p, hotSector: usedSector }, q, omega, Wrev, Psustain, isAct, mat }) });
    out.views.push({ label: '2D animation: hub cross-section', icon: '🎞', render: (el) => anim2dHub(el, { S, tauTot, N, nT, p: { ...p, hotSector: usedSector }, Wrev, omega }) });
    // recs
    if (S.dxi < 0.3) out.recs.push({ level: 'critical', text: `Cartridges transform only Δξ = ${fmt(S.dxi, 3)} per revolution: residence times of ${fmt(usedSector / 360 / fRev, 3)} s are too short or ΔT too small. Thinner lamellae, higher sector conductance, more lobes at lower speed, or lower speed.`, basis: 'Angle-resolved thermal response' });
    if (!isAct && !startupOK) out.recs.push({ level: 'warn', text: 'Dead start-up positions exist; add cartridges, use a 2-lobe cam with matched sectors, or release a bistable buffer cell for the first stroke.', basis: 'Static torque scan' });
    if (p.mu > 0.01) out.recs.push({ level: 'warn', text: `Friction coefficient ${p.mu} consumes ${fmt(mean(fricC) * N * 2 * Math.PI, 3)} J/rev — use rolling-element followers (μ ≈ 0.002).`, basis: 'Energy accounting' });
    if (!isAct && deficit > 0) out.recs.push({ level: 'critical', text: `Power deficit ${fmt(deficit, 3)} W at ${p.vKmh} km/h. The hub cannot replace wind propulsion under these conditions; it can at best extend range at lower speed${vmax ? ` (≤ ${vmax.toFixed(0)} km/h)` : ''}.`, basis: 'Hub power vs road demand' });
    if (thermalGrowth > p.clear) out.recs.push({ level: 'warn', text: 'Differential thermal growth exceeds the radial clearance — risk of cam seizure; increase clearance or match expansion coefficients.', basis: 'Thermal interference check' });
    out.recs.push({ level: 'good', text: 'Gate 3 experiment: build the stationary-axle / rotating-wheel dynamometer, impose metered hot/cold sector temperatures, measure torque vs angle and net work per revolution, then repeat with real collectors (solar, clear night, cloudy night).', basis: 'Specification experiments B and C' });
    out.recs.push({ level: 'sustain', text: 'Design cartridges as replaceable modules with alloy recovery; the wheel-integrated hub avoids gearbox/motor materials but concentrates high-embodied-energy NiTi — life (cycles) dominates its footprint.', basis: 'Life-cycle perspective' });
    out.explain.push({ title: 'How torque is produced', text: 'Each cartridge is stretched by the cam on one side of the wheel (cold, soft martensite, low force) and allowed to contract on the other (hot, austenite, high force). The axle-fixed cam converts this force asymmetry into tangential torque τ = −F dδ/dφ. Net work is the difference between work released on the contracting flank and work absorbed on the stretching flank, minus friction and bearing losses.' });
    out.explain.push({ title: 'Why heat supply limits speed', text: 'Faster rotation shortens the time each cartridge spends in each sector, so it transforms less per revolution; at the same time it needs heat at a higher rate. The atmospheric mode caps the hub at the heat the environment actually supplies (Suites 13–14).' });
    const selR = s16u?.ranking?.find(r => r.id === mat0.id);
    out.outputs = { actuator: mat0.id, actuatorClass: 'sma', actuatorSelection: selNote, actuatorEta: selR?.eta ?? ctx.upstream('s15')?.efficiency, actuatorPkg: selR?.P_kg ?? ctx.upstream('s15')?.powerDensity_W_kg, activeMass_kg: ctx.upstream('s15')?.activeMass_kg ?? mBudget, startEnergy_J: Estart, selfStart, bufferStart, hotSector_deg: usedSector, alloyShift_K: alloyShift, Wrev_J: isAct ? S.Wtau : Wrev, Pwheel_W: isAct ? 0 : Psustain, PwheelGross_W: PwheelGross, tauMean_Nm: tauMean, torqueRipple: ripple, startupOK, heatDemand_W: Qdemand, deficit_W: deficit, feasible: !isAct && Psustain >= demand && startupOK && Wrev > 0 && p.mode === 'atmospheric', vmax_kmh: vmax, mode: p.mode, WcartRev_J: S.Wtau, dxi: S.dxi, Qavail_W: Number.isFinite(Qavail) ? Qavail : null };
    out.headline = isAct ? `Actuator-only: ${fmt(S.Wtau, 4)} J per cartridge cycle at ${fmt(fRev, 3)} Hz (Δξ ${fmt(S.dxi, 3)}).` : `${p.mode === 'atmospheric' ? 'Atmospheric' : 'Imposed-ΔT'} hub at ${p.vKmh} km/h: ${fmt(Wrev, 4)} J/rev per wheel, sustainable ${fmt(Psustain, 4)} W vs demand ${fmt(demand, 4)} W${deficit > 0 ? ` → deficit ${fmt(deficit, 4)} W` : ' — demand met'}; start-up ${startupOK ? 'OK' : 'fails at ' + fmt(dead * 100, 3) + ' % of angles'}.`;
    out.headlineKind = !isAct && deficit === 0 && startupOK ? 'ok' : 'warn';
    return out;
  },
};

// Animated 3D direct-rim hub: every cartridge's temperature, martensite fraction and cam force are the solved
// angle-resolved states (cartridge i is phase-shifted by i/N of a revolution, exactly as in the torque summation);
// the torque arrow and running work counter integrate the solved net torque τ(θ). Cam amplitude drawn ×4.
async function view3dHub(el, { S, tauTot, N, nT, p, q, omega, Wrev, Psustain, isAct, mat }) {
  const [{ viewer }, { playback, hud, legend3d }, { hubAssembly }, { cmap }, { h }] = await Promise.all([import('../core/viz3d.js'), import('../core/sim3d.js'), import('../core/cad.js'), import('../core/chart.js'), import('../core/ui.js')]);
  let colorBy = 'T';
  const sel = h('select', { onchange: (e) => { colorBy = e.target.value; } }, h('option', { value: 'T' }, 'Cartridge colour: temperature'), h('option', { value: 'xi' }, 'Cartridge colour: martensite fraction'), h('option', { value: 'F' }, 'Cartridge colour: cam force'));
  el.append(h('p', { class: 'small muted' }, `Solved per cartridge and wheel angle: temperature in the hot/cold sectors (residence-time heat transfer), ${mat.name} phase fraction, cartridge force on the stationary cam, and the summed net wheel torque after friction and bearing loss. Yellow arrows = cam contact force (∝ N); green/red tangential arrow = net wheel torque (positive/negative). Playback is slowed; the HUD shows real time at ${p.vKmh} km/h.`), sel);
  const wrap = h('div'); el.append(wrap);
  const V = await viewer(wrap, { height: 460, grid: false }); const THREE = V.THREE;
  const asm = hubAssembly({ R: p.R, nCart: N, hotSector: p.hotSector, camAmp: q.a * 4 });
  const objs = asm.parts.map(m => ({ m, o: V.add(m) }));
  const phi = S.rec.phi, deg = phi.map(f => ((f % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI));
  // hot-sector centre in the solved angle frame (circular mean of hot samples) → rotate the stationary sector meshes to match
  let cx = 0, cy = 0; S.rec.hot.forEach((hh, i) => { if (hh) { cx += Math.cos(deg[i]); cy += Math.sin(deg[i]); } }); const hotC = Math.atan2(cy, cx);
  for (const { o } of objs) if (/sector/i.test(o.name)) o.rotation.z = hotC - Math.PI / 2;
  const carts = objs.filter(x => x.m.cartridge !== undefined).map(x => x.o), shoes = objs.filter(x => /Flexure shoe/.test(x.o.name)).map(x => x.o);
  const rotating = objs.filter(x => /Rim|Tyre|Containment/.test(x.o.name)).map(x => x.o);
  const rc = p.R * 0.42, Fmax = Math.max(...S.rec.F.map(Math.abs), 1e-9), tmax = Math.max(...tauTot.map(Math.abs), 1e-9);
  const fArrows = carts.map(() => { const a = new THREE.ArrowHelper(new THREE.Vector3(-1, 0, 0), new THREE.Vector3(), 0.05, 0xfacc15, 0.015, 0.01); V.root.add(a); return a; });
  const tArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, p.R + 0.04, 0.05), 0.1, 0x22c55e, 0.03, 0.02); V.root.add(tArrow);
  V.viewDir(0.15, -0.45, 0.8);
  const H = hud(wrap); const leg = legend3d(wrap, { title: 'Cartridge temperature', lo: p.Tc, hi: p.Th, unit: ' °C', cmapName: 'coolwarm' });
  const cum = [0]; for (let k = 1; k < nT; k++) cum.push(cum[k - 1] + 0.5 * (tauTot[k] + tauTot[k - 1]) * (phi[k] - phi[k - 1]));
  const T0 = Math.min(...S.rec.T), T1 = Math.max(...S.rec.T);
  playback(wrap, { duration: 720, rate: 36, label: 'Wheel rotation', unit: '°', onFrame: (t) => {
    const rev = Math.floor(t / 360), k = Math.min(nT - 1, Math.floor((t % 360) / 360 * nT));
    const thW = deg[k];
    for (const o of rotating) o.rotation.z = thW;
    carts.forEach((o, i) => {
      const j = (k + Math.round(i * nT / N)) % nT, ang = deg[j], base = 2 * Math.PI * i / N;
      o.rotation.z = ang - base; if (shoes[i]) shoes[i].rotation.z = ang - base;
      const v = colorBy === 'T' ? (S.rec.T[j] - p.Tc) / ((p.Th - p.Tc) || 1) : colorBy === 'xi' ? S.rec.xi[j] : S.rec.F[j] / Fmax, c = cmap(v, colorBy === 'T' ? 'coolwarm' : colorBy === 'xi' ? 'viridis' : 'inferno');
      o.material.color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255);
      const a = fArrows[i], L = 0.012 + 0.07 * Math.abs(S.rec.F[j]) / Fmax, r0 = rc * 0.62 + L + 0.01;
      a.position.set(r0 * Math.cos(ang), r0 * Math.sin(ang), 0.04); a.setDirection(new THREE.Vector3(-Math.cos(ang), -Math.sin(ang), 0)); a.setLength(L, 0.012, 0.008);
    });
    leg.set(colorBy === 'T' ? Math.min(p.Tc, T0) : 0, colorBy === 'T' ? Math.max(p.Th, T1) : colorBy === 'xi' ? 1 : Fmax);
    const tau = tauTot[k], L = 0.03 + 0.18 * Math.abs(tau) / tmax; tArrow.setColor(tau >= 0 ? 0x22c55e : 0xef4444); tArrow.setDirection(new THREE.Vector3(tau >= 0 ? 1 : -1, 0, 0)); tArrow.setLength(L, 0.03, 0.02);
    const W = rev * Wrev + cum[k];
    H.set([['Wheel motion', Wrev > 0 ? 'driven by solved net torque' : 'IMPOSED at test speed for visualisation', Wrev > 0 ? '#22c55e' : '#f59e0b'], ['Wheel angle', fmt(t % 360, 4) + '°'], ['Real time', fmt(t * Math.PI / 180 / Math.max(omega, 1e-9), 3) + ' s'], ['Net wheel torque', fmt(tau, 4) + ' N·m', tau >= 0 ? '#22c55e' : '#ef4444'], ['Net work this run', fmt(W, 4) + ' J', W >= 0 ? '#22c55e' : '#ef4444'], ['Net work / rev (solved)', fmt(Wrev, 4) + ' J'], ['Cartridge 1 T', fmt(S.rec.T[k], 4) + ' °C'], ['Cartridge 1 ξ', fmt(S.rec.xi[k], 3)], ['Cartridge 1 force', fmt(S.rec.F[k], 4) + ' N'], ...(isAct ? [] : [['Sustainable wheel power', fmt(Psustain, 4) + ' W']])]);
  } });
}

// 2D animated hub cross-section: stationary cam (amplitude ×4), hot/cold sectors from the solved residence pattern,
// cartridges coloured by solved ξ at their angles, net torque τ(θ) with a cursor and the running ∮τ dθ.
async function anim2dHub(el, { S, tauTot, N, nT, p, Wrev, omega }) {
  const [{ anim2d }, { h }, { cmap }] = await Promise.all([import('../core/sim3d.js'), import('../core/ui.js'), import('../core/chart.js')]);
  el.append(h('p', { class: 'small muted' }, `Solved angle-resolved states. Wheel rotation is ${Wrev > 0 ? 'driven by the solved net torque' : 'IMPOSED at the test speed for visualisation (solved net work is negative)'}; HUD shows the running integral of the solved net torque.`));
  const deg = S.rec.phi.map(f => ((f % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI) * 180 / Math.PI), cum = [0];
  for (let k = 1; k < nT; k++) cum.push(cum[k - 1] + 0.5 * (tauTot[k] + tauTot[k - 1]) * (S.rec.phi[k] - S.rec.phi[k - 1]));
  const tlo = Math.min(...tauTot), thi = Math.max(...tauTot), pad = (thi - tlo) * 0.1 || 1;
  anim2d(el, { height: 420, duration: 720, rate: 36, label: 'Wheel rotation', unit: '°', onFrame: (t, g) => {
    g.clear(); const ctx = g.ctx, R0 = Math.min(g.H, g.W * 0.45) * 0.42, cx = R0 + 30, cy = g.H / 2, k = Math.min(nT - 1, Math.floor((t % 360) / 360 * nT));
    // sectors
    S.rec.hot.forEach((hh, i) => { const a = -deg[i] * Math.PI / 180; ctx.strokeStyle = hh ? 'rgba(239,68,68,.55)' : 'rgba(59,130,246,.45)'; ctx.lineWidth = 10; ctx.beginPath(); ctx.arc(cx, cy, R0 * 0.62, a - 0.02, a + 0.02); ctx.stroke(); });
    // cam
    ctx.strokeStyle = g.mute; ctx.lineWidth = 2; ctx.beginPath(); for (let j = 0; j <= 120; j++) { const ph = 2 * Math.PI * j / 120, r = R0 * 0.45 * (1 + 4 * (p.camAmp / 1000) / (p.rc || 0.126) * Math.cos(ph)); const x = cx + r * Math.cos(-ph), y = cy + r * Math.sin(-ph); j ? ctx.lineTo(x, y) : ctx.moveTo(x, y); } ctx.stroke();
    // rim
    ctx.strokeStyle = g.fg; ctx.lineWidth = 3; ctx.beginPath(); ctx.arc(cx, cy, R0, 0, 2 * Math.PI); ctx.stroke();
    const spoke = -deg[k] * Math.PI / 180; ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + R0 * Math.cos(spoke), cy + R0 * Math.sin(spoke)); ctx.stroke();
    for (let i = 0; i < N; i++) { const j = (k + Math.round(i * nT / N)) % nT, a = -deg[j] * Math.PI / 180, c = cmap(S.rec.xi[j], 'viridis'), r = R0 * 0.78; ctx.fillStyle = `rgb(${c.map(Math.round).join(',')})`; ctx.beginPath(); ctx.arc(cx + r * Math.cos(a), cy + r * Math.sin(a), Math.max(5, R0 * 0.1), 0, 2 * Math.PI); ctx.fill(); }
    ctx.strokeStyle = g.fg; ctx.lineWidth = 4; for (let q = 0; q < 16; q++) { const a = spoke + 2 * Math.PI * q / 16; ctx.beginPath(); ctx.moveTo(cx + R0 * 0.93 * Math.cos(a), cy + R0 * 0.93 * Math.sin(a)); ctx.lineTo(cx + R0 * 1.07 * Math.cos(a), cy + R0 * 1.07 * Math.sin(a)); ctx.stroke(); } ctx.lineWidth = 1;
    g.text('cartridge colour = ξ (purple 0 → yellow 1); red/blue ring = hot/cold sector', 10, g.H - 8, { size: 10, color: g.mute });
    const P = g.panel(cx + R0 + 30, 0, g.W - (cx + R0 + 30), g.H, { xr: [0, 360], yr: [tlo - pad, thi + pad], xlabel: 'Wheel angle [°]', ylabel: 'Net torque [N·m]', title: 'τ(θ) per wheel' });
    P.line(deg.map((d, i) => (i && d < deg[i - 1]) ? NaN : d), tauTot, '#38bdf8', 2); P.hline(0, '#94a3b8'); P.vline(deg[k]); P.point(deg[k], tauTot[k], tauTot[k] >= 0 ? '#22c55e' : '#ef4444');
    const W = Math.floor(t / 360) * Wrev + cum[k]; g.text(`τ = ${fmt(tauTot[k], 4)} N·m   ∮τdθ so far = ${fmt(W, 4)} J   (per rev ${fmt(Wrev, 4)} J)   real time ${fmt(t * Math.PI / 180 / Math.max(omega, 1e-9), 3)} s`, P.px + 4, P.py + 16, { size: 11, color: W >= 0 ? '#22c55e' : '#ef4444' });
  } });
}

// Cycle-averaged hub model for twisted-coiled polymer (or LCE) muscle cartridges. The angle-resolved contact model above is
// specific to shape-memory alloys; here each cartridge completes min(1, f_achievable/f_rev) of one matched thermal cycle per
// wheel revolution (Suite 16 cycle model at the hub sector temperatures), with cam-contact friction, bearing loss,
// heat-supply limitation, fatigue and creep. Torque ripple is estimated from the cartridge count.
async function runPolymerHub(p, ctx, m, s16u, selNote = '') {
  const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [] };
  const veh = ctx.vehicle, s10 = ctx.upstream('s10'), s14 = ctx.upstream('s14'), s18u = ctx.upstream('s18');
  if (p.Th <= p.Tc) throw new Error('Hot sector must be warmer than the cold sector.');
  const v = kmh(Math.max(p.vKmh, 0.5)), omega = v / p.R, fRev = omega / (2 * Math.PI), N = p.nCart, mC = p.tcpMass / 1000;
  const q = { Th: p.Th, Tc: p.Tc, t_mm: Math.max(0.2, p.t), hc: p.hc, dwell: 0.1 }, r = evaluateActuator(m, q);
  const complete = r.f > 0 ? Math.min(1, r.f / fRev) : 0, wEff = Math.max(0, r.w_J_kg) * complete;
  const Lcoil = 0.12, F = mC * Math.max(0, r.w_J_kg) / (Math.max(1e-3, r.stroke) * Lcoil), Wfric = p.mu * F * p.rc * 2 * Math.PI; // actuator force consistent with cycle work = F·δ over a 0.12 m coil; rolling follower friction per rev
  const bearingW = s10?.bearingLoss_W ?? veh.bearingLoss_W ?? 3, Wbear = bearingW / Math.max(fRev, 1e-6) / p.nWheels;
  const Wgross = N * mC * wEff, Wrev = Wgross - N * Wfric - Wbear, tauMean = Wrev / (2 * Math.PI);
  const PwheelGross = Math.max(0, Wrev) * fRev * p.nWheels, Qdemand = r.eta > 0 ? N * p.nWheels * mC * wEff * fRev / r.eta : Infinity;
  let Qavail = Infinity, note = 'Imposed laboratory heat supply.'; if (p.mode === 'atmospheric') { Qavail = s14 ? Math.max(0, s14.Qhot_W) : 0; note = s14 ? `Suite 14 supply ${fmt(Qavail, 3)} W.` : 'Suite 14 not run: supply 0 W.'; }
  const supplyFrac = Qdemand > 0 && Number.isFinite(Qdemand) ? Math.min(1, Qavail / Qdemand) : 0, Psustain = PwheelGross * supplyFrac;
  const demand = roadLoad(v, veh).P, deficit = Math.max(0, demand - Psustain);
  const tauResist = (s10?.Crr ?? veh.Crr) * veh.mass * G0 * p.R / p.nWheels + bearingW / 5 / p.nWheels, Estart = 3 * (tauResist * p.nWheels * 2 * Math.PI / N + 0.5 * veh.mass * kmh(2) ** 2), bufE = s18u?.Euse_J ?? 10000, bufferStart = bufE >= Estart, startupOK = bufferStart;
  const ripple = N > 0 ? 2 / N : 1, lifeCycles = r.fatigue, lifeH = lifeCycles / Math.max(fRev, 1e-9) / 3600, creepLoss = Math.min(0.6, (m.id === 'tcp-pe' ? 0.04 : 0.02) * Math.max(0, Math.log10(Math.max(1, lifeCycles) / 1e3)));
  const K = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
  K('Hub actuator', m.name, '', `Selection: ${selNote}. NiTi remains selectable (Hub actuator technology → SMA).`, (p.hubActuator || 'auto') === 'auto' ? 'generated' : 'user');
  K('Cycle work at sector temperatures', r.w_J_kg, 'J/kg', `Suite 16 polymer cycle at ${p.Th} / ${p.Tc} °C, load ${fmt(r.stress, 3)} MPa, stroke ${fmt(r.stroke * 100, 3)} %.`, 'calc', r.net ? 'ok' : 'bad');
  K('Achievable cycle frequency vs wheel', `${fmt(r.f, 3)} / ${fmt(fRev, 3)}`, 'Hz', `Fraction of each cycle completed per revolution: ${fmt(complete * 100, 3)} % (polymer fibres heat/cool slowly).`, 'calc', complete >= 1 ? 'ok' : 'bad');
  K('Net work per wheel per revolution', Wrev, 'J', `Gross ${fmt(Wgross, 4)} J − friction ${fmt(N * Wfric, 4)} J − bearing ${fmt(Wbear, 4)} J.`, 'calc', Wrev > 0 ? 'ok' : 'bad');
  K('Wheel power (supply unconstrained)', PwheelGross, 'W', `${N} cartridges × ${p.nWheels} wheels × ${p.tcpMass} g of fibre.`, 'calc');
  K('Heat demand / available', `${fmt(Qdemand, 4)} / ${fmt(Qavail, 4)}`, 'W', note, s14 ? 'coupled' : 'hypothetical', supplyFrac >= 1 ? 'ok' : 'bad');
  K('Sustainable wheel power', Psustain, 'W', 'After limiting by environmental heat supply.', 'calc', Psustain >= demand ? 'ok' : 'bad');
  K('Road-load demand', demand, 'W', `${p.vKmh} km/h level road.`, 'calc');
  K('Power deficit', deficit, 'W', 'Must come from wind, stored energy or lower speed.', 'calc', deficit > 0 ? 'bad' : 'ok');
  K('Start-up', bufferStart ? 'via buffer release' : 'NOT possible', '', `Polymer muscles give no reliable static torque; buffer start needs ${fmt(Estart, 3)} J, available ${fmt(bufE, 4)} J.`, s18u ? 'coupled' : 'estimated', startupOK ? 'ok' : 'bad');
  K('Fatigue life / creep stroke loss', `${fmt(lifeCycles, 3)} cycles (${fmt(lifeH, 3)} h) / ${fmt(creepLoss * 100, 3)} %`, '', 'Literature-trend estimates for twisted-coiled fibres; creep reduces stroke at constant load — calibrate with cyclic tests.', 'estimated', lifeCycles > 1e7 ? 'ok' : 'warn');
  out.verdicts.push({ test: 'Positive net work per revolution (after friction and bearing loss)', status: Wrev > 0 ? 'pass' : 'fail', detail: `${fmt(Wrev, 4)} J per wheel revolution.` });
  out.verdicts.push({ test: 'Actuator can follow the wheel-imposed cycle', status: complete >= 1 ? 'pass' : complete > 0.2 ? 'warn' : 'fail', detail: `Polymer cycle ${fmt(r.f, 3)} Hz vs ${fmt(fRev, 3)} Hz at ${p.vKmh} km/h — ${fmt(complete * 100, 3)} % of each cycle completes.`, fix: 'Thinner fibres, forced-air cooling, or charge the buffer while parked instead of driving the wheel directly.' });
  out.verdicts.push({ test: 'Start-up (buffer-assisted)', status: startupOK ? 'pass' : 'fail', detail: `Needs ${fmt(Estart, 3)} J; buffer ${fmt(bufE, 4)} J.` });
  out.verdicts.push({ test: `Meets road demand at ${p.vKmh} km/h`, status: Psustain >= demand ? 'pass' : 'fail', detail: `Sustainable ${fmt(Psustain, 4)} W vs ${fmt(demand, 4)} W required — deficit ${fmt(deficit, 4)} W. The thermal backup cannot sustain driving; its credible role is recharging the elastic buffer (Suite 18).` });
  const speeds = linspace(2, 60, 30), Ps = speeds.map(vk => { const om = kmh(vk) / p.R, fr = om / (2 * Math.PI), cpl = r.f > 0 ? Math.min(1, r.f / fr) : 0, W = N * mC * Math.max(0, r.w_J_kg) * cpl - N * Wfric - bearingW / fr / p.nWheels, Pg = Math.max(0, W) * fr * p.nWheels, Qd = r.eta > 0 ? N * p.nWheels * mC * Math.max(0, r.w_J_kg) * cpl * fr / r.eta : Infinity; return Pg * (Number.isFinite(Qd) && Qd > 0 ? Math.min(1, Qavail / Qd) : 0); });
  out.charts.push({ title: `Hub power vs road demand — ${m.name}`, type: 'line', x: { label: 'Vehicle speed', unit: 'km/h' }, y: { label: 'Power', unit: 'W' }, series: [{ name: 'Sustainable hub power', x: speeds, y: Ps }, { name: 'Road-load demand', x: speeds, y: speeds.map(vk => roadLoad(kmh(vk), veh).P), color: '#ef4444' }], explain: 'Cycle-averaged polymer-muscle hub: completed-cycle fraction falls with speed because fibres cannot heat and cool faster than their thermal time constant.' });
  out.charts.push({ title: 'Matched-condition ranking used for the selection (Suite 16)', type: 'bar', horizontal: true, categories: (s16u?.ranking || []).slice(0, 9).map(x => x.name), series: [{ name: 'W/kg', values: (s16u?.ranking || []).slice(0, 9).map(x => x.P_kg) }], y: { label: 'Power density', unit: 'W/kg' }, explain: 'The default hub actuator is the matched-condition winner when it beats NiTi.' });
  out.explain.push({ title: 'Model scope', text: 'Cycle-averaged polymer-muscle cartridge model (matched cycle from Suite 16 at the hub sector temperatures). The angle-resolved cam-contact model of this suite applies to shape-memory alloys; select “Shape-memory alloy” to use it.' });
  out.recs.push({ level: 'critical', text: `The hub cannot sustain the ${fmt(demand, 3)} W road load (deficit ${fmt(deficit, 3)} W). Use the thermal converter to recharge the elastic buffer while parked or cruising (Suite 18) for wind-free starts.`, basis: 'Energy balance' });
  out.outputs = { actuator: m.id, actuatorClass: m.cls, selfStart: false, bufferStart, startEnergy_J: Estart, hotSector_deg: p.hotSector, alloyShift_K: 0, Wrev_J: Wrev, Pwheel_W: p.mode === 'actuator' ? 0 : Psustain, PwheelGross_W: PwheelGross, tauMean_Nm: tauMean, torqueRipple: ripple, startupOK, heatDemand_W: Qdemand, deficit_W: deficit, feasible: Psustain >= demand && startupOK && Wrev > 0 && p.mode === 'atmospheric', vmax_kmh: speeds.filter((vk, i) => Ps[i] >= roadLoad(kmh(vk), veh).P).pop() || 0, mode: p.mode, WcartRev_J: mC * wEff, fatigueCycles: lifeCycles, actuatorSelection: selNote, activeMass_kg: N * p.nWheels * mC, actuatorEta: r.eta, actuatorPkg: r.P_kg, Qavail_W: Number.isFinite(Qavail) ? Qavail : null };
  out.headline = `${m.name} hub at ${p.vKmh} km/h: ${fmt(Wrev, 4)} J/rev, sustainable ${fmt(Psustain, 4)} W vs ${fmt(demand, 4)} W demand (deficit ${fmt(deficit, 4)} W).`; out.headlineKind = deficit > 0 ? 'warn' : 'ok';
  out.credibility = 'Analytical screening';
  out.views.push({ label: '3D hub drive (cycle-averaged polymer muscles)', icon: '⚙', render: async (el) => {
    const [{ viewer }, { playback, hud }, { hubAssembly }, { cmap }, { h }] = await Promise.all([import('../core/viz3d.js'), import('../core/sim3d.js'), import('../core/cad.js'), import('../core/chart.js'), import('../core/ui.js')]);
    el.append(h('p', { class: 'small muted' }, `Cartridges coloured by their position in the matched polymer cycle (red heating / blue cooling); only ${fmt(complete * 100, 3)} % of each cycle completes per revolution at this speed, so colours barely swing at high speed. Wheel motion ${Wrev > 0 ? 'from the net work' : 'IMPOSED for visualisation (net work ≤ 0)'}.`));
    const box = h('div'); el.append(box); const V = await viewer(box, { height: 440, grid: false });
    const objs = hubAssembly({ R: p.R, nCart: N, hotSector: p.hotSector, camAmp: 0.006 }).parts.map(mm => ({ mm, o: V.add(mm) })); V.viewDir(0.15, -0.45, 0.8);
    const rot = objs.filter(x => !/cam|sector/i.test(x.o.name)).map(x => x.o), carts = objs.filter(x => x.mm.cartridge !== undefined); const H = hud(box);
    playback(box, { duration: 720, rate: 36, label: 'Wheel rotation', unit: '°', onFrame: (t) => { const th = t * Math.PI / 180; rot.forEach(o => o.rotation.z = th); carts.forEach(({ mm, o }) => { const ph = ((th + 2 * Math.PI * mm.cartridge / N) % (2 * Math.PI)) / (2 * Math.PI), sw = 0.5 + 0.5 * complete * Math.sin(2 * Math.PI * ph), c = cmap(sw, 'coolwarm'); o.material.color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255); }); H.set([['Actuator', m.name.slice(0, 28)], ['Cycle completed / rev', fmt(complete * 100, 3) + ' %'], ['Net work / rev', fmt(Wrev, 4) + ' J', Wrev > 0 ? '#22c55e' : '#ef4444'], ['Sustainable power', fmt(Psustain, 4) + ' W'], ['Deficit at ' + p.vKmh + ' km/h', fmt(deficit, 4) + ' W', '#ef4444']]); } });
  } });
  return out;
}
