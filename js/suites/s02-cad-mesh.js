// Suite 02 — Parametric 2D/3D CAD, geometry generation and numerical meshing.
import { naca4, cst, airfoilStats, vehicleAssembly, hubAssembly, toSTL, toOBJ, toPLY, toSTEP, toVTKPoly, conversionReport, engineeringDrawing, airfoilDAT, airfoilDXF, Mesh } from '../core/cad.js';
import { ogrid, unstructured, quality, potentialFlow, sensitivityStudy, selectMesh, toGmsh, toVTK, toSU2, toAbaqus, toNastran, toOpenFOAM, resampleAirfoil } from '../core/mesh.js';
import { panelSolver } from '../core/aero.js';
import { COMPONENTS, componentMaterial } from '../core/materials.js';
import { parseSTL } from '../core/io.js';
import { fmt, histogram, range, sum, max, min, linspace } from '../core/num.js';
import { h, table, toast, callout } from '../core/ui.js';
import { download } from '../core/chart.js';
import { zip } from '../core/io.js';

// Mass model per part: shells use surface area × effective laminate thickness; solids use enclosed volume × fill.
const MASS_MODEL = [
  [/Monocoque/, { mode: 'shell', t: 0.003, comp: 'chassis' }], [/canopy/i, { mode: 'shell', t: 0.004, comp: 'cabin' }],
  [/skin \+ lattice/, { mode: 'shell', t: 0.0015, comp: 'wingSkin' }], [/box spar/, { mode: 'shell', t: 0.003, comp: 'spar' }], [/root fitting/, { mode: 'solid', fill: 0.35, comp: 'rootInsert' }],
  [/^Tyre/, { mode: 'shell', t: 0.004, comp: 'tyre' }], [/^Rim/, { mode: 'solid', fill: 1, comp: 'wheel' }], [/^Spoke/, { mode: 'solid', fill: 0.5, comp: 'wheel' }],
  [/hub ring/, { mode: 'solid', fill: 0.35, comp: 'actuator' }], [/Brake disc/, { mode: 'solid', fill: 1, comp: 'brake' }],
  [/collector/, { mode: 'shell', t: 0.0015, comp: 'collector' }], [/Radiative/, { mode: 'solid', fill: 1, comp: 'radiator' }],
];
const massModel = (name) => (MASS_MODEL.find(([re]) => re.test(name)) || [null, { mode: 'shell', t: 0.003, comp: 'chassis' }])[1];

export function resolveAirfoilCAD(p, ctx) {
  const imp = ctx.store.project.imports?.airfoil || (ctx.experiments || []).find(d => d.kind === 'airfoil' && d.points);
  if (p.family === 'imported' && imp?.points?.length > 10) return { name: imp.name || 'Imported', points: imp.points, prov: 'user' };
  if (p.family === 'cst') return { ...cst([p.au1, p.au2, p.au3, p.au4], [p.al1, p.al2, p.al3, p.al4], 161), name: `CST [${[p.au1, p.au2, p.au3, p.au4].join(', ')} | ${[p.al1, p.al2, p.al3, p.al4].join(', ')}]`, prov: 'generated' };
  const code = /^\d{4}$/.test(String(p.naca)) ? String(p.naca) : '0018';
  return { ...naca4(code, 161), prov: 'generated' };
}

export default {
  description: 'Generates the complete parametric ATMOS-XP automobile — monocoque, cabin, three wheels with direct-rim hubs, twin vertical morphing wings with spars and root fittings, solar collector and radiative panels — as closed, oriented solids with mass properties; produces dimensioned third-angle engineering drawings; builds structured and unstructured CFD meshes around the selected wing section, assesses element quality, runs a systematic mesh-sensitivity study with the grid-convergence index and selects the mesh by credible criteria; and exports to neutral CAD, mesh and visualisation formats with conversion reports.',
  literature: 'airfoil mesh generation grid convergence index O-grid quality metrics',
  credibility: 'Numerically verified',
  equations: [
    'Parametric geometric constraint equations (planform: S = c̄·b, taper λ = c_t/c_r, linear twist distribution)',
    'NACA 4-digit thickness and camber equations; Kulfan Class–Shape Transformation (Bernstein polynomial) airfoils; coordinate-based reconstruction of imported sections',
    'Lofting / ruled-surface generation through sections, sweep and twist transforms (homogeneous rotation matrices)',
    'Divergence-theorem mass properties: volume, surface area, centroid of closed triangulated shells',
    'Hyperbolic-type normal marching with geometric (first-cell, growth-ratio) stretching for structured O-grids; trailing-edge rounding for O-topology',
    'Bowyer–Watson Delaunay triangulation (empty-circumcircle criterion) with boundary-layer node layers and distance-based sizing',
    'Element quality: scaled Jacobian, equiangle skewness, aspect ratio, face non-orthogonality, area positivity',
    'P1 Galerkin finite-element solution of the Laplace equation (potential flow) as the mesh-sensitivity functional',
    'Richardson extrapolation, observed order of accuracy and grid-convergence index (Roache / Celik et al.)',
  ],
  methods: ['Manifold/orientation check of every edge (shared by exactly two oppositely oriented triangles)', 'Export → re-import round trip (binary STL) with triangle-count and volume comparison', 'Mesh selection: coarsest level meeting GCI and quality thresholds', 'Silhouette/crease-edge projection for orthographic drawings (third-angle, ISO 7200 title block)'],
  assumptions: ['Parametric solids are tessellated B-rep-like shells; exact NURBS curvature is not preserved in neutral exports (faceted STEP)', 'Shell mass uses effective laminate thickness; detailed laminate schedules belong to Suites 09 and 22', 'Mesh-sensitivity functional is the non-lifting potential-flow peak surface speed (geometry-resolving metric)'],
  ibc: {
    initial: ['Reference geometric configuration and feature-tree state from the project vehicle record', 'Reference airfoil (NACA 0018 starting section), unit-cell topology and material region assignment', 'Coordinate systems: Earth-fixed, vehicle body (x forward, z up), wing span axes, wheel axes'],
    boundary: ['Dimensional constraints (length, width, wheelbase, track, wheel radius, wing height/area)', 'Tangency/continuity of lofted surfaces, minimum wall thickness, manufacturing clearances', 'Mesh boundary groups: wall (airfoil), far-field (circle radius R), fluid region; association with CAD faces retained', 'Far-field Dirichlet potential φ = x·cosα + y·sinα and zero-flux wall for the potential mesh study'],
  },
  cvv: {
    calibration: ['Physical component dimensions, scanned geometry, measured masses, centre of gravity and inertia (Module 01–02 calibration set)'],
    verification: ['Closed/manifold/oriented shells; positive volumes', 'STL export/re-import round trip within 1e-6 relative volume', 'Grid convergence with observed order and GCI; mesh quality thresholds', 'FE potential flow vs panel-method reference'],
    validation: ['Coordinate-measuring-machine inspection of manufactured parts against CAD', 'Weighing of manufactured parts against predicted masses'],
  },
  formats: { import: ['Airfoil .dat (Selig/Lednicer)', 'STL / OBJ geometry (Data portal)', 'Gmsh MSH, VTK (Import mode)'], export: ['STEP AP214 faceted B-rep', 'STL (binary)', 'OBJ', 'PLY', 'glTF/GLB', 'VTK PolyData', 'DXF & SVG drawings', 'Airfoil DAT/CSV/DXF', 'Gmsh MSH 2.2', 'VTK unstructured', 'SU2', 'OpenFOAM polyMesh', 'Abaqus INP', 'Nastran BDF', 'Project package ZIP'] },
  params: [
    { key: 'family', label: 'Airfoil family', type: 'select', default: 'naca', options: [{ value: 'naca', label: 'NACA 4-digit' }, { value: 'cst', label: 'CST (Kulfan)' }, { value: 'imported', label: 'Imported (.dat via Data portal)' }], group: 'Wing section' },
    { key: 'naca', label: 'NACA code', type: 'text', default: '0018', group: 'Wing section', help: 'mpxx: max camber m %, position p/10, thickness xx %.' },
    ...[0.17, 0.16, 0.15, 0.14].map((v, i) => ({ key: 'au' + (i + 1), label: `CST upper weight A${i}`, type: 'number', min: -0.5, max: 0.6, step: 0.01, default: v, group: 'Wing section', expert: true })),
    ...[-0.17, -0.12, -0.10, -0.08].map((v, i) => ({ key: 'al' + (i + 1), label: `CST lower weight A${i}`, type: 'number', min: -0.6, max: 0.5, step: 0.01, default: v, group: 'Wing section', expert: true })),
    { key: 'incidence', label: 'Wing incidence in CAD', unit: '°', type: 'range', min: -45, max: 90, step: 1, default: 12, group: 'Assembly', auto: (c) => { const o = c.upstream('s07'); return o?.incidence !== undefined ? { value: Math.round(o.incidence), prov: 'coupled' } : undefined; } },
    { key: 'twist', label: 'Tip twist (wash-out)', unit: '°', type: 'range', min: -15, max: 5, step: 0.5, default: -4, group: 'Assembly', auto: (c) => { const o = c.upstream('s06'); return o?.twistTip_deg !== undefined ? { value: +o.twistTip_deg.toFixed(1), prov: 'coupled' } : undefined; } },
    { key: 'steer', label: 'Steer angle (display)', unit: '°', type: 'range', min: -30, max: 30, step: 1, default: 0, group: 'Assembly' },
    { key: 'detail', label: 'Tessellation detail', type: 'select', default: 1, options: [{ value: 1, label: 'Standard' }, { value: 2, label: 'High' }], group: 'Assembly', expert: true },
    { key: 'nCart', label: 'Hub cartridges (hub drawing)', type: 'number', min: 6, max: 48, step: 1, default: 24, group: 'Assembly', auto: (c) => c.store.params('s17')?.nCart },
    { key: 'meshType', label: 'Mesh topology', type: 'select', default: 'ogrid', options: [{ value: 'ogrid', label: 'Structured O-grid (quads)' }, { value: 'unstructured', label: 'Unstructured Delaunay (triangles + BL layers)' }], group: 'Meshing' },
    { key: 'ni', label: 'Base cells around section', type: 'number', min: 24, max: 200, step: 4, default: 64, group: 'Meshing' },
    { key: 'nj', label: 'Base radial layers (O-grid)', type: 'number', min: 10, max: 100, step: 2, default: 24, group: 'Meshing' },
    { key: 'R', label: 'Far-field radius', unit: 'chords', type: 'number', min: 5, max: 50, step: 1, default: 12, group: 'Meshing' },
    { key: 'h1', label: 'First-cell height', unit: 'chord', type: 'number', min: 1e-5, max: 0.02, step: 1e-4, default: 0.002, group: 'Meshing', expert: true, help: 'For viscous RANS, size for y⁺ ≈ 1 at the operating Reynolds number.' },
    { key: 'levels', label: 'Refinement levels', type: 'number', min: 3, max: 4, step: 1, default: 3, group: 'Mesh sensitivity' },
    { key: 'ratio', label: 'Refinement ratio r', type: 'select', default: 1.4142, options: [{ value: 1.4142, label: '√2' }, { value: 2, label: '2' }], group: 'Mesh sensitivity' },
    { key: 'gciMax', label: 'Acceptance: discretisation error', unit: '%', type: 'number', min: 0.1, max: 10, step: 0.1, default: 1, group: 'Mesh sensitivity' },
    { key: 'maxSkew', label: 'Acceptance: max equiangle skewness', type: 'number', min: 0.3, max: 0.98, step: 0.01, default: 0.85, group: 'Mesh sensitivity', expert: true },
    { key: 'minJ', label: 'Acceptance: min scaled Jacobian', type: 'number', min: 0, max: 0.9, step: 0.01, default: 0.05, group: 'Mesh sensitivity', expert: true },
    { key: 'dwgNo', label: 'Drawing number', type: 'text', default: 'AXP-GA-001', group: 'Drawing', expert: true },
    { key: 'author', label: 'Drawn by', type: 'text', default: '', group: 'Drawing', expert: true },
  ],
  presets: [{ label: 'NACA 0018 (spec start)', values: { family: 'naca', naca: '0018' } }, { label: 'NACA 0012 thin', values: { family: 'naca', naca: '0012' } }, { label: 'Cambered 2415', values: { family: 'naca', naca: '2415' } }, { label: 'Fine O-grid', values: { meshType: 'ogrid', ni: 96, nj: 36 } }, { label: 'Unstructured', values: { meshType: 'unstructured', ni: 80 } }],

  async run(p, ctx) {
    const out = { kpis: [], charts: [], tables: [], verdicts: [], recs: [], checks: [], explain: [], views: [], exports: [] };
    const veh = ctx.vehicle, af = resolveAirfoilCAD(p, ctx), st = airfoilStats(af.points);
    const nW = veh.nWings || 2, Sw = veh.wingArea / nW, span = veh.wingHeight, cm = Sw / span, cr = cm * 1.25, ct = cm * 0.75;
    const mats = Object.fromEntries(Object.keys(COMPONENTS).map(k => [k, componentMaterial(ctx.store, k)]));
    ctx.progress(0.05, 'CAD');
    const asm = vehicleAssembly(veh, { wingIncidence: p.incidence, airfoil: { ...af, points: resampleClosed(af.points, 61) }, twistTip: p.twist, steer: p.steer, detail: +p.detail, materials: mats });
    const parts = asm.parts;
    // mass properties
    const rows = [], massRows = []; let M = 0, mx = 0, my = 0, mz = 0, allClosed = true, allOriented = true, negVol = 0;
    for (const m of parts) {
      const mm = massModel(m.name), mat = mats[mm.comp] || mats.chassis, V = m.volume(), A = m.area(), c = m.centroid(), man = m.checkManifold();
      const mass = mm.mode === 'shell' ? A * mm.t * mat.rho : Math.abs(V) * (mm.fill ?? 1) * mat.rho;
      m.materialName = mat.name; allClosed &&= man.closed; allOriented &&= man.oriented; if (V <= 0) negVol++;
      M += mass; mx += mass * c[0]; my += mass * c[1]; mz += mass * c[2];
      rows.push([m.name, m.group, mat.name, V * 1e3, A, mass, c[0], c[1], c[2], man.closed && man.oriented ? '✔' : '✖']);
      massRows.push({ group: m.group, mass });
    }
    const cg = [mx / M, my / M, mz / M];
    const groups = [...new Set(massRows.map(r => r.group))], gm = groups.map(g => sum(massRows.filter(r => r.group === g).map(r => r.mass)));
    // export round trip
    const stl = toSTL(parts), buf = await stl.arrayBuffer(), back = parseSTL(buf);
    const Vorig = sum(parts.map(m => m.volume())); let Vback = 0; const P = back.positions; for (let k = 0; k < P.length; k += 9) Vback += (P[k] * (P[k + 4] * P[k + 8] - P[k + 5] * P[k + 7]) - P[k + 1] * (P[k + 3] * P[k + 8] - P[k + 5] * P[k + 6]) + P[k + 2] * (P[k + 3] * P[k + 7] - P[k + 4] * P[k + 6])) / 6;
    const triCount = sum(parts.map(m => m.triangleCount));
    let bx0 = Infinity, bx1 = -Infinity; for (const m of parts) { const b = m.bbox(); bx0 = Math.min(bx0, b[0]); bx1 = Math.max(bx1, b[3]); }
    out.checks.push({ name: 'All solids closed (2-manifold)', value: parts.filter(m => m.checkManifold().closed).length + '/' + parts.length, target: 'all', pass: allClosed });
    out.checks.push({ name: 'Consistent outward orientation', value: allOriented ? 'yes' : 'no', target: 'all edges opposite-oriented', pass: allOriented });
    out.checks.push({ name: 'Positive enclosed volumes', value: parts.length - negVol + '/' + parts.length, target: 'all > 0', pass: negVol === 0 });
    out.checks.push({ name: 'STL export → re-import triangle count', value: back.triangles, target: String(triCount), pass: back.triangles === triCount });
    out.checks.push({ name: 'STL round-trip volume deviation', value: Math.abs(Vback - Vorig) / Vorig, target: '< 1e-5 (float32 precision)', pass: Math.abs(Vback - Vorig) / Vorig < 1e-5 });
    out.checks.push({ name: 'Overall length vs specification', value: bx1 - bx0, target: `${veh.length} m ± 10 % (wheels/tyres may protrude)`, pass: Math.abs((bx1 - bx0) - veh.length) / veh.length < 0.1 });
    await ctx.yield();
    // ---- meshing & sensitivity ----
    ctx.progress(0.3, 'meshing');
    const r = +p.ratio, levels = range(p.levels).map(k => r ** k);
    const gen = (f) => p.meshType === 'ogrid' ? ogrid(af.points, { ni: Math.round(p.ni * f / 2) * 2, nj: Math.round(p.nj * f), R: p.R, h1: p.h1 / f }) : unstructured(af.points, { ns: Math.round(p.ni * f), layers: 4, growth: 1.25, h1: 1.2 / (p.ni * f), R: p.R, sizeFactor: 0.2 / f });
    let lvlIdx = 0;
    const study = sensitivityStudy((f) => { ctx.progress(0.3 + 0.5 * (lvlIdx++ / levels.length), `mesh level ${lvlIdx}`); return gen(f); }, (m) => potentialFlow(m, 0).VmaxSurface, levels);
    const sel = selectMesh(study, { gciMax: p.gciMax / 100, maxSkew: p.maxSkew, minJ: p.minJ });
    const ref = panelSolver(resampleClosed(af.points, 240)).solveNonLifting(0).VmaxSurface;
    const chosen = sel.chosen, Q = quality(chosen.mesh), G = study.gci, chosenLevel = study.levels.findIndex(l => l.cells === chosen.cells) + 1;
    out.checks.push({ name: 'Mesh: no negative/inverted cells (all levels)', value: sum(study.levels.map(l => l.quality.negativeCells)), target: '0', pass: study.levels.every(l => l.quality.negativeCells === 0) });
    if (G) {
      out.checks.push({ name: 'Observed order of accuracy p', value: G.p, target: 'P1 FE: ≈ 1–2', pass: G.p > 0.5 && G.p < 3.5, detail: G.oscillatory ? 'oscillatory convergence' : 'monotone convergence' });
      out.checks.push({ name: 'GCI fine grid', value: G.gci21, target: `< ${p.gciMax} %`, pass: G.gci21 * 100 < p.gciMax });
      out.checks.push({ name: 'Asymptotic-range ratio', value: G.asymptoticRatio, target: '≈ 1 (within 10 %)', pass: G.inAsymptoticRange });
    }
    out.checks.push({ name: 'FE potential vs panel-method reference (V_max/V∞)', value: Math.abs((G?.fext ?? chosen.value) - ref) / ref, target: '< 2 % (finite far-field blockage)', pass: Math.abs((G?.fext ?? chosen.value) - ref) / ref < 0.02, detail: `extrapolated ${fmt(G?.fext ?? chosen.value, 5)} vs panel ${fmt(ref, 5)}` });
    // ---- KPIs ----
    const k = (label, value, unit, explain, prov, status) => out.kpis.push({ label, value, unit, explain, prov, status });
    k('Wing section', af.name, '', 'Selected section family/parameters (coupled to Suites 06–08).', af.prov);
    k('Max thickness t/c', st.tmax * 100, '% at ' + fmt(st.xt * 100, 3) + ' %c', 'From the generated coordinates.', 'calc');
    k('Wing planform per wing', `${fmt(cr, 3)} → ${fmt(ct, 3)} m × ${span} m`, '', `Root → tip chord, span; area ${fmt(Sw, 3)} m² each, taper ${fmt(ct / cr, 3)}.`, 'generated');
    k('Geometric aspect ratio', span * span / Sw, '', 'b²/S per wing (Suite 07 applies the end-plate effect of the body).', 'calc');
    k('Solids in assembly', parts.length, '', 'Closed, oriented shells generated parametrically.', 'generated');
    k('Triangles (tessellation)', triCount, '', 'Polygonal representation for visualisation and neutral exports; the native parametric record is retained.', 'generated');
    k('Estimated structural mass (CAD)', M, 'kg', 'Area × effective thickness (shells) or volume × fill (solids) × assigned material density. Excludes occupant, payload and actuator internals.', 'estimated');
    k('CAD centre of mass', `${fmt(cg[0], 3)}, ${fmt(cg[1], 3)}, ${fmt(cg[2], 3)}`, 'm', 'Body frame x forward, y left, z up from the ground at the assembly origin.', 'calc');
    k('Selected mesh', `${chosen.cells} cells (level ${chosenLevel} of ${study.levels.length})`, '', `Coarsest level meeting the error (${p.gciMax} %) and quality criteria.`, 'generated', sel.chosen.ok ? 'ok' : 'warn');
    k('Mesh min scaled Jacobian', Q.summary.minJacobian, '', '1 = ideal element shape; ≤ 0 means an inverted cell.', 'calc', Q.summary.minJacobian > 0.2 ? 'ok' : Q.summary.minJacobian > 0 ? 'warn' : 'bad');
    k('Mesh max skewness', Q.summary.maxSkew, '', 'Equiangle skewness (0 ideal, 1 degenerate).', 'calc', Q.summary.maxSkew < 0.7 ? 'ok' : Q.summary.maxSkew < 0.9 ? 'warn' : 'bad');
    k('Mesh max non-orthogonality', Q.summary.maxNonOrth, '°', 'Angle between the cell-centre vector and face normal; > 70° degrades finite-volume gradients.', 'calc', Q.summary.maxNonOrth < 60 ? 'ok' : 'warn');
    if (G) k('GCI (fine grid)', G.gci21 * 100, '%', `Discretisation uncertainty of the functional with observed order ${fmt(G.p, 3)}.`, 'calc', G.gci21 * 100 < p.gciMax ? 'ok' : 'warn');
    // ---- verdicts ----
    out.verdicts.push({ test: 'CAD validity (closed, oriented, positive volume)', status: allClosed && allOriented && negVol === 0 ? 'pass' : 'fail', detail: `${parts.length} solids; ${triCount} triangles; round-trip volume error ${fmt(Math.abs(Vback - Vorig) / Vorig, 2)}.` });
    out.verdicts.push({ test: 'Mesh independence', status: sel.chosen.ok ? 'pass' : 'warn', detail: G ? `Selected level error vs extrapolated ${fmt(100 * sel.chosen.errExtrap, 3)} % (criterion ${p.gciMax} %), GCI fine ${fmt(100 * G.gci21, 3)} %.` : 'Need ≥ 3 levels.', fix: 'Increase base resolution or refinement levels; reduce the acceptance threshold only with justification.' });
    out.verdicts.push({ test: 'Mesh quality', status: Q.summary.negativeCells === 0 && Q.summary.minJacobian > p.minJ && Q.summary.maxSkew <= p.maxSkew ? 'pass' : 'warn', detail: `min J ${fmt(Q.summary.minJacobian, 3)}, max skew ${fmt(Q.summary.maxSkew, 3)}, max AR ${fmt(Q.summary.maxAR, 3)}, max non-orthogonality ${fmt(Q.summary.maxNonOrth, 3)}°.` });
    // ---- charts ----
    const afp = af.points;
    out.charts.push({ title: `Wing section geometry — ${af.name}`, x: { label: 'x/c', min: -0.02, max: 1.02 }, y: { label: 'y/c', min: -0.35, max: 0.35 }, series: [{ name: 'Contour', x: afp.map(q => q[0]), y: afp.map(q => q[1]) }, ...(af.camber ? [{ name: 'Mean camber line', x: af.camber.map(q => q[0]), y: af.camber.map(q => q[1]), dash: '4 3' }] : [])], explain: 'Generated section coordinates (normalised by chord). Axis limits are equal-span so the thickness is shown to scale.' });
    out.charts.push({ title: 'Mesh-sensitivity study', x: { label: 'Representative cell size h', unit: 'chord', log: true }, y: { label: 'V_max / V∞ on surface' }, series: [{ name: 'FE potential flow', x: study.levels.map(l => l.h), y: study.levels.map(l => l.value), mode: 'lines+markers' }], annotations: [...(G ? [{ type: 'hline', y: G.fext, label: 'Richardson extrapolation', color: '#22c55e' }] : []), { type: 'hline', y: ref, label: 'panel-method reference', color: '#f59e0b' }, { type: 'vline', x: chosen.h, label: 'selected' }], explain: 'Functional versus cell size on successively refined meshes; convergence toward the extrapolated value demonstrates mesh independence and quantifies discretisation error.' });
    const hs = histogram(Q.skew, 20, 0, 1), ha = histogram(Q.ar.map(a => Math.log10(Math.max(1, a))), 20), hj = histogram(Q.minJ, 20, Math.min(0, min(Q.minJ)), 1);
    out.charts.push({ title: 'Equiangle skewness distribution (selected mesh)', type: 'bar', categories: hs.centers.map(c => fmt(c, 2)), y: { label: 'Cells' }, series: [{ name: 'cells', values: hs.counts }], explain: 'Most cells should lie well below 0.5; a tail above 0.85 indicates regions needing local refinement or smoothing.' });
    out.charts.push({ title: 'Aspect-ratio distribution (log10)', type: 'bar', categories: ha.centers.map(c => fmt(c, 2)), y: { label: 'Cells' }, series: [{ name: 'cells', values: ha.counts }], explain: 'High aspect ratios are intended inside the boundary layer (wall-normal stretching) and acceptable where aligned with the flow.' });
    out.charts.push({ title: 'Scaled Jacobian distribution', type: 'bar', categories: hj.centers.map(c => fmt(c, 2)), y: { label: 'Cells' }, series: [{ name: 'cells', values: hj.counts }], explain: 'Element shape regularity; values ≤ 0 would indicate folded (invalid) cells.' });
    out.charts.push({ title: 'Estimated mass by group', type: 'bar', categories: groups, y: { label: 'Mass', unit: 'kg' }, series: [{ name: 'kg', values: gm }], showValues: true, explain: 'CAD-based mass build-up using assigned materials (Suite 03). Suite 01 combines this with occupants, payload and the actuator system.' });
    // ---- tables ----
    out.tables.push({ title: 'Mass properties per solid', cols: ['Part', 'Group', 'Material', 'Enclosed volume (L)', 'Area (m²)', 'Mass (kg)', 'x̄ (m)', 'ȳ (m)', 'z̄ (m)', 'Valid'], rows, explain: 'Enclosed volume by the divergence theorem; mass model per part type (shell or solid) using assigned material density.' });
    out.tables.push({ title: 'Mesh levels and selection criteria', cols: ['Level', 'Cells', 'Nodes', 'h', 'V_max/V∞', 'Error vs extrapolated', 'min J', 'max skew', 'max non-orth (°)', 'Quality OK', 'Accuracy OK'], rows: sel.rows.map((l, i) => [i + 1, l.cells, l.quality.nodes, l.h, l.value, fmt(100 * l.errExtrap, 3) + ' %', l.quality.minJacobian, l.quality.maxSkew, l.quality.maxNonOrth, l.qualOk ? '✔' : '✖', l.accOk ? '✔' : '✖']) });
    // ---- views ----
    const matMap = mats;
    out.views.push({ label: '3D CAD', icon: '🧊', render: (el) => cadView(el, parts, p, ctx, veh) });
    out.views.push({ label: '3D styling & finishes', icon: '🎨', render: (el) => view3dStyling(el, { veh, ctx }) });
    out.views.push({ label: '3D assembly & rolling', icon: '🎬', render: (el) => view3dAssembly(el, { veh, ctx }) });
    out.views.push({ label: '2D animation: mesh refinement levels', icon: '🎞', render: (el) => anim2dMesh(el, { study, chosen, p }) });
    out.views.push({ label: '2D drawings', icon: '📐', render: (el) => drawingsView(el, parts, p, af, { cr, ct, span, st }, veh) });
    out.views.push({ label: 'Mesh', icon: '▦', render: (el) => meshView(el, chosen.mesh, Q, p) });
    out.views.push({ label: 'Export manager', icon: '⇩', render: (el) => exportView(el, parts, chosen.mesh, af, Q, p) });
    void matMap;
    // ---- recommendations ----
    if (!sel.chosen.ok) out.recs.push({ level: 'warn', text: 'No refinement level met both the accuracy and quality criteria; use the finest level only for screening and refine further before using CFD forces for design decisions.', basis: 'GCI + quality thresholds' });
    if (Q.summary.maxNonOrth > 60) out.recs.push({ level: 'warn', text: 'Non-orthogonality above 60° near the trailing edge; apply non-orthogonal correctors in the finite-volume solver or increase smoothing/cell count there.', basis: 'Quality metric' });
    out.recs.push({ level: 'good', text: `For viscous RANS at Re ≈ ${fmt((ctx.upstream('s07')?.Re) || 7e5, 2)}, set first-cell height for y⁺ ≈ 1 (≈ ${fmt(6 / ((ctx.upstream('s07')?.Re) || 7e5) ** 0.9 * 10, 2)} c) with growth ≤ 1.2 and ≥ 25 layers inside the boundary layer.`, basis: 'Flat-plate skin-friction estimate' });
    out.recs.push({ level: 'sustain', text: `Estimated structural mass ${fmt(M, 3)} kg. Every kilogram saved reduces rolling-resistance power by ≈ ${fmt(9.81 * veh.Crr * 20 / 3.6, 2)} W at 20 km/h — prioritise mass reduction on the largest groups above.`, basis: 'P = m g C_rr v' });
    out.recs.push({ level: 'info', text: 'Use STEP AP214 for CAD interchange and 3MF/STL for additive manufacture of lattices; keep the native project JSON as the parametric master (neutral formats lose the feature tree).', basis: 'Specification Part C' });
    out.explain.push({ title: 'What was generated', text: `A ${parts.length}-solid parametric assembly from the project vehicle record (length ${veh.length} m, track ${veh.track} m, wheelbase ${veh.wheelbase} m, ${veh.nWheels} wheels, ${nW} wings of ${span} m). Every solid is checked as a closed, consistently oriented shell before export, and its mass properties are computed from the assigned material.` });
    out.explain.push({ title: 'Why mesh sensitivity', text: 'Numerical results depend on discretisation. The same functional was computed on systematically refined meshes; Richardson extrapolation estimates the mesh-independent value, the GCI bounds the error of each mesh, and the cheapest mesh satisfying the pre-set error and quality criteria is selected for downstream CFD.' });
    out.outputs = { airfoil: { name: af.name, points: af.points }, wing: { span, rootChord: cr, tipChord: ct, area: veh.wingArea, nWings: nW }, mesh: { type: p.meshType, cells: chosen.cells, minJ: Q.summary.minJacobian, maxSkew: Q.summary.maxSkew, maxAR: Q.summary.maxAR, level: chosenLevel }, gci: G ? G.gci21 : null, massCAD: M, cgCAD: cg, massByGroup: Object.fromEntries(groups.map((g, i) => [g, gm[i]])) };
    out.headline = `Generated ${parts.length} valid solids (${fmt(M, 3)} kg structural estimate) and a ${chosen.cells}-cell ${p.meshType === 'ogrid' ? 'O-grid' : 'unstructured'} mesh around ${af.name} with ${G ? fmt(100 * G.gci21, 3) + ' % GCI' : 'n/a'}.`;
    out.headlineKind = allClosed && sel.chosen.ok ? 'ok' : 'warn';
    out.credibility = out.checks.every(c => c.pass) ? 'Numerically verified' : 'Analytical screening';
    return out;
  },
};

function resampleClosed(pts, n) { const r = resampleAirfoil(pts, (n - 1)); r.push(r[0].slice()); return r; }

async function cadView(el, parts, p, ctx, veh) {
  const { viewer } = await import('../core/viz3d.js');
  const bar = h('div', { class: 'row gap wrap', style: 'margin-bottom:8px' }), box = h('div');
  el.append(bar, box);
  const V = await viewer(box, { height: 480 }); V.controls.autoRotate = true; V.controls.autoRotateSpeed = 0.8; // slow orbit; drag to take control
  V.renderer.domElement.addEventListener('pointerdown', () => { V.controls.autoRotate = false; });
  const show = (list) => { V.clear(); for (const m of list) V.add(m); V.fit(); };
  show(parts);
  const imp = ctx.store.project.imports?.geometry;
  const isArch = (veh.wingConfig || 'tandem') === 'arch';
  let reefed = false; const reefBtn = h('button', { class: 'btn sm', title: isArch ? 'AeroArch: fold the arch back over the body for parking (≈ 1.3 m overall height)' : 'Revision 2.2: telescope the upper wing sections into the lower ones (transit / strong wind)', onclick: () => { reefed = !reefed; reefBtn.textContent = isArch ? (reefed ? 'Unfold arch' : 'Fold arch (parking)') : (reefed ? 'Deploy wings' : 'Reef wings'); const mats = Object.fromEntries(Object.keys(COMPONENTS).map(k => [k, componentMaterial(ctx.store, k)])); const asm2 = vehicleAssembly(veh, { materials: mats, ...(isArch ? { folded: reefed } : { reefed }) }); show(asm2.parts); toast(`Overall height ${fmt(asm2.meta.height, 3)} m`, 'info'); } }, isArch ? 'Fold arch (parking)' : 'Reef wings');
  if (isArch || (ctx.vehicle.wingReefSpan ?? ctx.vehicle.wingHeight) < ctx.vehicle.wingHeight - 1e-6) bar.append(reefBtn);
  bar.append(h('button', { class: 'btn sm', onclick: () => show(parts) }, 'Full vehicle'), h('button', { class: 'btn sm', onclick: () => show(parts.filter(m => m.group === 'Wings')) }, 'Wings only'), h('button', { class: 'btn sm', onclick: () => show(parts.filter(m => ['Wheels', 'Hub drive', 'Brakes'].includes(m.group))) }, 'Running gear'),
    h('button', { class: 'btn sm', onclick: () => { const hub = hubAssembly({ R: veh.wheelRadius, nCart: p.nCart }).parts; show(hub); let th = 0; V.animate = (dt) => { th += dt * 0.6; for (const o of V.objects) if (o.userData.cartridge !== undefined || o.name.startsWith('Flexure') || ['Rim', 'Tyre', 'Containment ring'].includes(o.name)) o.rotation.z = th; }; } }, 'Direct-rim hub (animated)'),
    ...(imp ? [h('button', { class: 'btn sm', onclick: () => { const n = imp.positions.length / 3, m = { name: imp.name || 'Imported geometry', positions: imp.positions, indices: Uint32Array.from(range(n)), color: '#eab308', group: 'Imported', centroid: () => [0, 0, 0] }; V.clear(); V.add(m); V.fit(); } }, 'Imported geometry')] : []));
  if (imp) { const P = imp.positions; let Vv = 0, A = 0; for (let k = 0; k < P.length; k += 9) { Vv += (P[k] * (P[k + 4] * P[k + 8] - P[k + 5] * P[k + 7]) - P[k + 1] * (P[k + 3] * P[k + 8] - P[k + 5] * P[k + 6]) + P[k + 2] * (P[k + 3] * P[k + 7] - P[k + 4] * P[k + 6])) / 6; const ux = P[k + 3] - P[k], uy = P[k + 4] - P[k + 1], uz = P[k + 5] - P[k + 2], vx = P[k + 6] - P[k], vy = P[k + 7] - P[k + 1], vz = P[k + 8] - P[k + 2]; A += 0.5 * Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx); } el.append(h('p', { class: 'small muted' }, `Imported geometry “${imp.name}”: ${imp.triangles} triangles, enclosed volume ${fmt(Vv, 4)} (model units³), area ${fmt(A, 4)} (units²).`)); }
  el.append(h('p', { class: 'small muted' }, 'Hover a part for name, group and material. Explode, X-ray and Section give exploded, transparent and cut-away views; GLB exports the scene for glTF viewers and AR.'));
}

function drawingsView(el, parts, p, af, g, veh) {
  const ga = engineeringDrawing(parts, { title: 'PHOENIX-ATMOS-XP — General arrangement', dwgNo: p.dwgNo, author: p.author, material: 'see materials register (Suite 03)', dims: [] });
  const hub = engineeringDrawing(hubAssembly({ R: veh.wheelRadius, nCart: p.nCart }).parts, { title: 'Direct-rim thermomechanical hub — arrangement', dwgNo: p.dwgNo.replace('GA', 'HUB'), author: p.author, scale: '1:4', material: 'NiTi cartridges / Ti containment / 17-4PH cam' });
  const sec = sectionDrawing(af, g);
  const block = (title, svg, dxf, name) => h('section', { class: 'card' }, h('h3', { class: 'card-title' }, title), h('div', { class: 'dwg-box', html: svg }), h('div', { class: 'row gap wrap', style: 'margin-top:8px' }, h('button', { class: 'btn sm', onclick: () => download(new Blob([svg], { type: 'image/svg+xml' }), name + '.svg') }, 'SVG'), dxf ? h('button', { class: 'btn sm', onclick: () => download(new Blob([dxf], { type: 'application/dxf' }), name + '.dxf') }, 'DXF') : null, h('button', { class: 'btn sm', onclick: () => printSVG(svg, title) }, 'Print / PDF')));
  el.append(block('General arrangement (third-angle, dimensions in mm)', ga.svg, ga.dxf, 'atmosxp-ga'), block('Direct-rim hub arrangement', hub.svg, hub.dxf, 'atmosxp-hub'), block('Wing section & planform', sec, airfoilDXF(af, g.cr), 'atmosxp-wing-section'));
}
function printSVG(svg, title) { const w = window.open('', '_blank'); if (!w) { toast('Allow pop-ups to print', 'fail'); return; } w.document.write(`<title>${title}</title><style>@page{size:A3 landscape;margin:8mm}body{margin:0}svg{width:100%;height:auto}</style>${svg}`); w.document.close(); setTimeout(() => w.print(), 300); }

function sectionDrawing(af, g) {
  const W = 1000, H = 520, c = 560, ox = 70, oy = 190, X = (x) => ox + x * c, Y = (y) => oy - y * c;
  const pts = af.points.map(q => `${X(q[0]).toFixed(1)},${Y(q[1]).toFixed(1)}`).join(' ');
  const dim = (x1, y1, x2, y2, t) => `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="#2563eb" stroke-width="0.8" marker-start="url(#da)" marker-end="url(#da)"/><text x="${(x1 + x2) / 2}" y="${(y1 + y2) / 2 - 4}" font-size="12" fill="#1d4ed8" text-anchor="middle">${t}</text>`;
  const pl = { x0: 700, y0: 470, s: 150 / Math.max(g.span, g.cr) };
  const plan = `<polygon points="${pl.x0},${pl.y0} ${pl.x0 + g.cr * pl.s},${pl.y0} ${pl.x0 + 0.25 * g.cr * pl.s + 0.75 * g.ct * pl.s},${pl.y0 - g.span * pl.s} ${pl.x0 + 0.25 * g.cr * pl.s - 0.25 * g.ct * pl.s},${pl.y0 - g.span * pl.s}" fill="#e2e8f0" stroke="#111" stroke-width="1"/>`;
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" font-family="Arial"><defs><marker id="da" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0,1L10,5L0,9z" fill="#2563eb"/></marker></defs><rect width="${W}" height="${H}" fill="#fff"/>
  <text x="20" y="28" font-size="16" font-weight="700">WING SECTION — ${af.name} (root chord ${(g.cr * 1000).toFixed(0)} mm)</text>
  <polygon points="${pts}" fill="#f1f5f9" stroke="#111" stroke-width="1.2"/><line x1="${X(-0.05)}" y1="${oy}" x2="${X(1.05)}" y2="${oy}" stroke="#94a3b8" stroke-dasharray="10 4 2 4"/>
  <line x1="${X(0.25)}" y1="${oy - 60}" x2="${X(0.25)}" y2="${oy + 60}" stroke="#ef4444" stroke-dasharray="6 3"/><text x="${X(0.25) + 4}" y="${oy - 64}" font-size="11" fill="#b91c1c">c/4 (spar axis)</text>
  ${dim(X(0), oy + 90, X(1), oy + 90, `c = ${(g.cr * 1000).toFixed(0)} (root) / ${(g.ct * 1000).toFixed(0)} (tip)`)}
  ${dim(X(g.st.xt), Y(g.st.tmax / 2) - 0, X(g.st.xt), Y(-g.st.tmax / 2), '')}<text x="${X(g.st.xt) + 8}" y="${oy + 4}" font-size="12" fill="#1d4ed8">t = ${(g.st.tmax * 100).toFixed(1)} % c at ${(g.st.xt * 100).toFixed(0)} % c</text>
  <text x="20" y="${H - 60}" font-size="12">Section area ${(g.st.area * g.cr * g.cr * 1e4).toFixed(1)} cm² (root) · coordinates exported as DAT/CSV/DXF</text>
  <text x="${pl.x0 - 20}" y="${pl.y0 - g.span * pl.s - 16}" font-size="13" font-weight="700">PLANFORM (per wing)</text>${plan}
  ${dim(pl.x0, pl.y0 + 18, pl.x0 + g.cr * pl.s, pl.y0 + 18, (g.cr * 1000).toFixed(0))}${dim(pl.x0 - 18, pl.y0, pl.x0 - 18, pl.y0 - g.span * pl.s, (g.span * 1000).toFixed(0))}
  <text x="20" y="${H - 20}" font-size="11">Units mm · Third-angle projection · General tolerance ISO 2768-mK · PHOENIX-ATMOS-XP</text></svg>`;
}

function meshView(el, m, Q, p) {
  const zoom = h('select', {}, h('option', { value: 'near' }, 'Near field (section)'), h('option', { value: 'te' }, 'Trailing edge'), h('option', { value: 'le' }, 'Leading edge'), h('option', { value: 'far' }, 'Whole domain'));
  const color = h('select', {}, h('option', { value: 'none' }, 'Edges only'), h('option', { value: 'skew' }, 'Colour by skewness'), h('option', { value: 'ar' }, 'Colour by aspect ratio'));
  const box = h('div'); el.append(h('div', { class: 'row gap wrap' }, h('label', {}, 'View ', zoom), h('label', {}, 'Colour ', color)), box);
  const draw = () => {
    const win = { near: [-0.4, 1.4, -0.45, 0.45], te: [0.9, 1.1, -0.06, 0.06], le: [-0.04, 0.12, -0.05, 0.05], far: [0.5 - p.R * 1.05, 0.5 + p.R * 1.05, -p.R * 1.05, p.R * 1.05] }[zoom.value];
    const W = 900, Hh = Math.round(W * (win[3] - win[2]) / (win[1] - win[0])), sx = (x) => (x - win[0]) / (win[1] - win[0]) * W, sy = (y) => Hh - (y - win[2]) / (win[3] - win[2]) * Hh;
    let s = `<svg class="mesh-svg" viewBox="0 0 ${W} ${Hh}">`; const N = m.nodes;
    const { cmap: cm } = { cmap: (t) => `hsl(${(1 - Math.min(1, Math.max(0, t))) * 240},80%,50%)` };
    m.elems.forEach((e, k) => {
      const P = e.map(i => N[i]); if (P.every(q => q[0] < win[0] || q[0] > win[1] || q[1] < win[2] || q[1] > win[3])) return;
      const fill = color.value === 'skew' ? cm(Q.skew[k]) : color.value === 'ar' ? cm(Math.log10(Q.ar[k]) / 3) : 'none';
      s += `<path d="M${P.map(q => sx(q[0]).toFixed(1) + ',' + sy(q[1]).toFixed(1)).join('L')}Z" fill="${fill}" fill-opacity="${fill === 'none' ? 0 : 0.75}" stroke="currentColor" stroke-opacity="0.55" stroke-width="0.6"/>`;
    });
    s += `<path d="M${m.surface.map(q => sx(q[0]).toFixed(1) + ',' + sy(q[1]).toFixed(1)).join('L')}Z" fill="#64748b" stroke="#ef4444" stroke-width="1.5"/></svg>`;
    box.innerHTML = s;
  };
  zoom.onchange = draw; color.onchange = draw; draw();
  el.append(table(['Metric', 'Value'], [['Topology', m.type === 'quad' ? 'Structured O-grid (quadrilaterals)' : 'Unstructured Delaunay (triangles)'], ['Cells', Q.summary.cells], ['Nodes', Q.summary.nodes], ['Wall faces', m.bnd.wall.length], ['Far-field faces', m.bnd.farfield.length], ['First-cell height', m.h1], ['Growth ratio', m.growth ?? '—'], ['Trailing-edge rounding radius (O-topology)', m.teRounded ?? '—'], ['Boundary recovery (unstructured)', m.boundaryRecovery ?? '—'], ['Min scaled Jacobian', Q.summary.minJacobian], ['Max / mean skewness', `${fmt(Q.summary.maxSkew, 3)} / ${fmt(Q.summary.meanSkew, 3)}`], ['Max aspect ratio', Q.summary.maxAR], ['Max / mean non-orthogonality (°)', `${fmt(Q.summary.maxNonOrth, 3)} / ${fmt(Q.summary.meanNonOrth, 3)}`], ['Inverted cells', Q.summary.negativeCells]]));
}

function exportView(el, parts, m, af, Q, p) {
  const name = 'atmosxp';
  const rep = (fmtName) => { const r = conversionReport(fmtName, parts); return `Conversion report — ${fmtName}\nParts: ${r.parts}, triangles: ${r.triangles}\nAll closed: ${r.allClosed}, all oriented: ${r.allOriented}\nPreserved: ${r.preserved.join('; ')}\nLost / approximated: ${r.lost.join('; ') || 'none'}\n`; };
  const repBox = h('pre', { class: 'small', style: 'white-space:pre-wrap' }, rep('STEP'));
  const B = (label, fn, fmtName) => h('button', { class: 'btn sm', onclick: async () => { try { await fn(); if (fmtName) repBox.textContent = rep(fmtName); toast(label + ' exported', 'ok'); } catch (e) { toast(e.message, 'fail'); } } }, label);
  const txt = (s, n, t = 'text/plain') => download(new Blob([s], { type: t }), n);
  el.append(h('h4', {}, '3D CAD & visualisation'), h('div', { class: 'export-grid' },
    B('STEP AP214 (faceted B-rep)', () => txt(toSTEP(parts, 'ATMOSXP'), name + '.step', 'application/step'), 'STEP'),
    B('STL binary', () => download(toSTL(parts), name + '.stl'), 'STL'), B('OBJ', () => txt(toOBJ(parts), name + '.obj'), 'OBJ'), B('PLY', () => txt(toPLY(parts), name + '.ply'), 'PLY'), B('VTK PolyData', () => txt(toVTKPoly(parts), name + '.vtk'), 'VTK'),
    B('glTF binary (GLB)', async () => { const { viewer } = await import('../core/viz3d.js'); const tmp = h('div', { style: 'position:absolute;left:-9999px;width:400px' }); document.body.append(tmp); const V = await viewer(tmp, { height: 200, grid: false }); for (const q of parts) V.add(q); await V.exportGLB(); setTimeout(() => { V.dispose(); tmp.remove(); }, 1500); }, 'glTF'),
    B('Native parametric project (JSON)', () => txt(JSON.stringify({ format: 'atmosxp-native', version: 1, params: p, airfoil: af, parts: parts.map(q => ({ name: q.name, group: q.group, triangles: q.triangleCount })) }, null, 1), name + '.atmosxp.json', 'application/json'))),
    h('h4', {}, 'Airfoil section'), h('div', { class: 'export-grid' }, B('Selig DAT', () => txt(airfoilDAT(af), 'airfoil.dat')), B('CSV coordinates', () => txt('x,y\n' + af.points.map(q => q.join(',')).join('\n'), 'airfoil.csv', 'text/csv')), B('DXF profile', () => txt(airfoilDXF(af, 1), 'airfoil.dxf', 'application/dxf'), 'DXF')),
    h('h4', {}, 'Computational mesh (selected level)'), h('div', { class: 'export-grid' }, B('Gmsh MSH 2.2', () => txt(toGmsh(m), 'mesh.msh')), B('VTK unstructured (+quality fields)', () => txt(toVTK(m, { skewness: Q.skew, aspectRatio: Q.ar, scaledJacobian: Q.minJ }), 'mesh.vtk')), B('SU2', () => txt(toSU2(m), 'mesh.su2')), B('Abaqus INP', () => txt(toAbaqus(m), 'mesh.inp')), B('Nastran BDF', () => txt(toNastran(m), 'mesh.bdf')),
      B('OpenFOAM polyMesh (ZIP)', async () => { const of = toOpenFOAM(m); download(await zip(Object.entries(of).map(([k, v]) => ({ name: `constant/polyMesh/${k}`, data: v }))), 'polyMesh.zip'); })),
    h('h4', {}, 'Conversion report'), repBox,
    callout('info', 'Neutral formats cannot carry the full parametric feature tree, constitutive models or solver settings; the native project JSON remains the master record. CGNS, Parasolid, ACIS, JT and DWG require licensed or binary kernels and are provided through STEP/STL/glTF interchange.'));
  void max; void linspace; void Mesh;
}

// 3D assembly sequence (sub-assemblies installed in build order) followed by the vehicle rolling at the coupled
// sustainable speed: wheel spin = distance / wheel radius (exact rolling kinematics). Driving dynamics are solved in Suite 20.
async function view3dAssembly(el, { veh, ctx }) {
  const [{ vehicleScene, playback, hud }, mats] = await Promise.all([import('../core/sim3d.js'), import('../core/materials.js')]);
  const vS = ctx.upstream('s20')?.sustainableSpeed_kmh, vk = Number.isFinite(vS) && vS > 0 ? Math.min(vS, 60) : 20, v = vk / 3.6;
  el.append(h('p', { class: 'small muted' }, `0–8 s: assembly sequence (monocoque → cabin → wheels, hub drive & brakes → wings → thermal surfaces). 8–24 s: rolling at ${fmt(vk, 3)} km/h (${Number.isFinite(vS) ? 'Suite 20 sustainable speed' : 'reference 20 km/h — run Suite 20 to couple'}); wheel rotation follows exact rolling kinematics θ = s/R. Geometry and materials are the current CAD configuration.`));
  const wrap = h('div'); el.append(wrap);
  const matMap = Object.fromEntries(Object.keys(mats.COMPONENTS).map(k => [k, ctx.material(k)]));
  const S = await vehicleScene(wrap, { vehicle: veh, materials: matMap, follow: true, height: 460 });
  for (const k of ['thrust', 'side', 'drag', 'hub', 'wind']) S.setArrow(k, [0, 0, 0]);
  const order = { Chassis: 0, Cabin: 1, Wheels: 2, 'Hub drive': 2, Brakes: 2, Wings: 3, Thermal: 4 };
  const items = S.car.children.filter(o => o.type !== 'Object3D' || o.children.length).filter(o => !o.isArrowHelper && o.type !== 'ArrowHelper').map(o => ({ o, base: o.position.clone(), stage: o.userData?.group !== undefined ? (order[o.userData.group] ?? 4) : 2 }));
  const H = hud(wrap), stages = ['Monocoque', 'Cabin', 'Wheels, hub drive & brakes', 'Wings (spar, skin, root fittings)', 'Thermal surfaces'];
  playback(wrap, { duration: 24, label: 'Time', unit: 's', onFrame: (t) => {
    const asmT = Math.min(t, 8) / 8 * 5;
    for (const it of items) { const pr = Math.max(0, Math.min(1, asmT - it.stage)); it.o.visible = pr > 0; it.o.position.set(it.base.x, it.base.y, it.base.z + (1 - pr) ** 2 * 2.5); }
    const tr = Math.max(0, t - 8), x = v * tr;
    S.setPose({ x, y: 0, heading: 0, dist: x });
    H.set(t < 8 ? [['Phase', 'Assembly'], ['Installing', stages[Math.min(4, Math.floor(asmT))]], ['Progress', fmt(100 * asmT / 5, 3) + ' %']] : [['Phase', 'Rolling'], ['Speed', fmt(vk, 3) + ' km/h'], ['Distance', fmt(x, 4) + ' m'], ['Wheel speed', fmt(v / veh.wheelRadius * 60 / (2 * Math.PI), 4) + ' rpm'], ['Wheel revolutions', fmt(x / (2 * Math.PI * veh.wheelRadius), 4)]]);
  } });
}

// 2D animation stepping through the solved mesh-sensitivity levels (coarse → fine) with a slow zoom from the whole
// section to the leading edge, and the convergence of the monitored quantity toward the Richardson-extrapolated value.
async function anim2dMesh(el, { study, chosen, p }) {
  const [{ anim2d }] = await Promise.all([import('../core/sim3d.js')]);
  const L = study.sorted.slice().reverse(), n = L.length, fext = study.gci?.fext, per = 6;
  el.append(h('p', { class: 'small muted' }, `${n} solved mesh levels (coarse → fine), each shown for ${per} s while zooming from the section to the leading edge. The selected level (${chosen.cells} cells) is highlighted.`));
  anim2d(el, { height: 440, duration: n * per, label: 'Animation time', unit: 's', onFrame: (t, g) => {
    g.clear(); const k = Math.min(n - 1, Math.floor(t / per)), f = (t - k * per) / per, M = L[k].mesh, ctx = g.ctx;
    const z = 1 - 0.85 * (0.5 - 0.5 * Math.cos(Math.PI * f)), cxw = 0.5 - 0.48 * (1 - z), win = [cxw - 0.9 * z, cxw + 0.9 * z, -0.45 * z, 0.45 * z];
    const W = g.W * 0.62, Hh = g.H, sx = (x) => (x - win[0]) / (win[1] - win[0]) * W, sy = (y) => Hh / 2 - y / (win[1] - win[0]) * W;
    ctx.save(); ctx.beginPath(); ctx.rect(0, 0, W, Hh); ctx.clip(); ctx.strokeStyle = L[k].cells === chosen.cells ? 'rgba(34,197,94,.7)' : 'rgba(56,189,248,.55)'; ctx.lineWidth = 0.6; ctx.beginPath();
    for (const e of M.elems) { const a = M.nodes[e[0]]; if (!a) continue; let inside = false; for (const id of e) { const q = M.nodes[id]; if (q[0] > win[0] && q[0] < win[1] && q[1] > win[2] && q[1] < win[3]) { inside = true; break; } } if (!inside) continue; ctx.moveTo(sx(a[0]), sy(a[1])); for (let m2 = 1; m2 <= e.length; m2++) { const q = M.nodes[e[m2 % e.length]]; ctx.lineTo(sx(q[0]), sy(q[1])); } }
    ctx.stroke(); ctx.restore();
    g.text(`Level ${k + 1}/${n}: ${L[k].cells.toLocaleString()} cells, h = ${fmt(L[k].h, 3)}${L[k].cells === chosen.cells ? '  ← selected' : ''}`, 8, 18, { bold: true, color: L[k].cells === chosen.cells ? '#22c55e' : g.fg });
    const P = g.panel(W, 0, g.W - W, g.H, { xr: [0, n + 1], yr: [Math.min(...L.map(l => l.value), fext ?? Infinity) * 0.995, Math.max(...L.map(l => l.value), fext ?? -Infinity) * 1.005], xlabel: 'Level (coarse → fine)', ylabel: 'V_max/V∞', title: 'Convergence' });
    P.line(L.slice(0, k + 1).map((_, i) => i + 1), L.slice(0, k + 1).map(l => l.value), '#38bdf8', 2); L.slice(0, k + 1).forEach((l, i) => P.point(i + 1, l.value, l.cells === chosen.cells ? '#22c55e' : '#38bdf8', 4)); if (Number.isFinite(fext)) P.hline(fext, '#a855f7');
  } });
}

// Styling view: paint schemes and finishes on the current lifting-body CAD (spats and root fairings included), with the
// quantities that matter for performance and sustainability: painted area, finish mass, coating CO₂e and solar absorptance
// (a dark body heats more in sun; light finishes keep the cabin and composite cooler).
const SCHEMES = {
  'Arctic white & carbon': { body: '#f1f5f9', wings: '#e2e8f0', accents: '#111827', aS: 0.25 },
  'Solar orange': { body: '#f97316', wings: '#f8fafc', accents: '#1f2937', aS: 0.55 },
  'Midnight blue': { body: '#1e3a8a', wings: '#cbd5e1', accents: '#f59e0b', aS: 0.85 },
  'Raw carbon (clear coat)': { body: '#262a31', wings: '#3b4250', accents: '#9ca3af', aS: 0.92 },
  'Material truth (as assigned)': null,
};
const FINISH = { Gloss: { r: 0.18, m: 0.1 }, Satin: { r: 0.45, m: 0.1 }, Matte: { r: 0.85, m: 0.0 } };
async function view3dStyling(el, { veh, ctx }) {
  const [{ viewer }, { hud }] = await Promise.all([import('../core/viz3d.js'), import('../core/sim3d.js')]);
  let scheme = 'Arctic white & carbon', finish = 'Satin';
  const selS = h('select', { onchange: (e) => { scheme = e.target.value; paint(); } }, ...Object.keys(SCHEMES).map(k => h('option', {}, k)));
  const selF = h('select', { onchange: (e) => { finish = e.target.value; paint(); } }, ...Object.keys(FINISH).map(k => h('option', { selected: k === finish }, k)));
  el.append(h('div', { class: 'row gap wrap' }, h('label', {}, 'Paint scheme ', selS), h('label', {}, 'Finish ', selF)), h('p', { class: 'small muted' }, 'Finishes apply to the lifting body, spats and root fairings; the wing skins use a light UV-stable film; the collector and radiator keep their functional coatings. Areas come from the CAD surfaces; coating mass 0.12 kg/m² (dry film ≈ 80 µm), 3.0 kg CO₂e/kg (waterborne system).'));
  const box = h('div'); el.append(box);
  const V = await viewer(box, { height: 480 }); V.controls.autoRotate = true; V.controls.autoRotateSpeed = 1.2; V.renderer.domElement.addEventListener('pointerdown', () => { V.controls.autoRotate = false; });
  const mats = Object.fromEntries(Object.keys(COMPONENTS).map(k => [k, componentMaterial(ctx.store, k)]));
  const parts = vehicleAssembly(veh, { materials: mats }).parts, objs = parts.map(m => ({ m, o: V.add(m) })); V.fit();
  const role = (m) => /collector|radiat/i.test(m.name) || ['collector', 'radiator'].includes(m.component) ? 'functional' : /tyre|rim|spoke|disc|hub|cartridge/i.test(m.name) ? 'running' : m.group === 'Wings' && !/fairing|root/i.test(m.name) ? 'wings' : /canopy|cabin/i.test(m.name) ? 'glass' : /fairing|spat/i.test(m.name) ? 'accents' : /monocoque|lifting body|body shell|nose|tail/i.test(m.name) ? 'body' : 'internal';
  const H = hud(box);
  const areaOf = (r) => objs.filter(x => role(x.m) === r).reduce((s2, x) => s2 + x.m.area(), 0);
  const Abody = areaOf('body') + areaOf('accents'), Awing = areaOf('wings') / 2;
  function paint() {
    const S = SCHEMES[scheme], F = FINISH[finish];
    for (const { m, o } of objs) { const r = role(m); if (S && (r === 'body' || r === 'wings' || r === 'accents')) o.material.color.set(r === 'body' ? S.body : r === 'wings' ? S.wings : S.accents); else o.material.color.set(m.color); o.material.roughness = F.r; o.material.metalness = F.m; o.material.needsUpdate = true; }
    const paintMass = Abody * 0.12, co2 = paintMass * 3.0, aS = S ? S.aS : 0.9;
    H.set([['Scheme', scheme], ['Finish', finish], ['Painted area (body, spats, fairings)', fmt(Abody, 3) + ' m²'], ['Wing film area', fmt(Awing, 3) + ' m²'], ['Coating mass', fmt(paintMass, 3) + ' kg'], ['Coating CO₂e', fmt(co2, 3) + ' kg'], ['Body solar absorptance', fmt(aS, 2), aS > 0.7 ? '#f59e0b' : '#22c55e'], ['Peak skin heating (850 W/m², still air)', '≈ +' + fmt(aS * 850 / 18, 3) + ' K']]);
  }
  paint();
}
