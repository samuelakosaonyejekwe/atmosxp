// Heavy Suite 07 computations for the compliant-flexure arch (Revision 5.1), shared by the Web Worker and the inline fallback:
// Reynolds-band polars, the thrust/side envelope vs apparent-wind angle, the speed-resolved trimmed coefficient grid used by
// Suite 20, and the best-heading sustainable-speed table. Inputs and outputs are plain data (structured-clone safe).
import * as AA from './archaero.js';
import { polar } from './aero.js';
import { linspace } from './num.js';

const strip = (p) => ({ alpha: p.alpha, CL: p.CL, CD: p.CD, alphaStall: p.alphaStall, CLmax: p.CLmax });
export function flexCompute(args) {
  const { veh, pol, pts, polOpts, Re, cmean, nu, Va, rho, flexP, FySafe, flexMax, S, awa, VaGrid, W5 } = args;
  const GA = AA.archGeometry(veh);
  const bands = [3.5, 8, 16].map(V => ({ V, pol: Math.abs(V * cmean / nu - Re) / Re < 0.15 ? pol : strip(polar(pts, Math.max(2e4, V * cmean / nu), polOpts)) }));
  const polAt = (V) => bands.reduce((a, b) => Math.abs(Math.log(b.V / V)) < Math.abs(Math.log(a.V / V)) ? b : a).pol;
  const E = { thrOpt: [], sideOpt: [], incOpt: [], thrFix: [], cxOpt: [], thrUnsafe: [], thrNoTandem: [], thrIdeal: [], dTrim: [], wThr: [], wSide: [], wAlpha: [] };
  for (const b of awa) { const bb = Math.max(0.5, b), t = AA.trimState(GA, pol, veh, bb, Va, rho, { ...flexP, FyS: FySafe, scanN: 30 }), id = AA.bestInEnvelope(GA, pol, veh, bb, Va, rho, { deltaMax: flexMax, FyS: FySafe }); let bu = -Infinity; for (const a of linspace(-5, 35, 41)) bu = Math.max(bu, AA.archForces(GA, pol, veh, bb, a, Va, rho).net);
    E.thrOpt.push(t.net); E.sideOpt.push(t.side); E.incOpt.push(t.delta); E.wThr.push(t.wingThrust); E.wSide.push(t.wingSide); E.wAlpha.push(t.alpha); E.thrFix.push(AA.archForces(GA, pol, veh, bb, bb, Va, rho).net); E.cxOpt.push(t.net / (0.5 * rho * Va * Va * S)); E.thrUnsafe.push(bu); E.thrNoTandem.push(id.net); E.thrIdeal.push(id.net); E.dTrim.push(t.delta); }
  const grid = { Va: VaGrid, awa, CL: [], CD: [], CL0: [], CD0: [], delta: [], Re: VaGrid.map(V => Math.round(V * cmean / nu)) };
  for (const V of VaGrid) { const cl = [], cd = [], c0 = [], d0 = [], dl = [], pV = polAt(V); for (const b of awa) { const t = AA.trimState(GA, pV, veh, Math.max(0.5, b), V, rho, { ...flexP, FyS: Infinity, scanN: 30 }), f0 = AA.archForces(GA, pV, veh, Math.max(0.5, b), t.alpha, V, rho, { interf: 0 }), f = f0.stack ? f0 : AA.archForces(GA, pV, veh, Math.max(0.5, b), t.alpha, V, rho, { interf: 0.6 }); /* washout floor: only the camber lift can be shed — stacked model: interf 0 (camber-only); post-stall / large-angle model: symmetric-section fraction 0.6, as in cappedForces */ cl.push(t.CLsys ?? t.CL); cd.push(t.CDsys ?? t.CD); c0.push(f.CLsys ?? f.CL); d0.push(f.CDsys ?? f.CD); dl.push(t.delta); } grid.CL.push(cl); grid.CD.push(cd); grid.CL0.push(c0); grid.CD0.push(d0); grid.delta.push(dl); } // CL0/CD0: skin fully de-cambered (washout floor)
  const sustained = W5.map(U => ({ U, ...AA.sustainableSpeed(GA, polAt, veh, U, { ...flexP, FyS: FySafe }, { headings: [60, 90, 120], scanN: 30, dv: 0.1, reach: false, coarse: 2 }) })); // steady force balance, as Suite 20
  // arch-opening fill comparison (hollow / solid / hybrid) at identical conditions: beam-wind steady speeds, light-wind L/D,
  // arch-locked tip-over (side area scaled by the fill), parked side force ratio
  const fills = args.fills ? ['hollow', 'solid', 'hybrid'].map(fill => { const vF = { ...veh, archFill: fill }, GF = AA.archGeometry(vF), FP = AA.archFillProps(vF, GF), areaR = (GF.S + FP.area) / GF.S;
    const v = W5.map(U => AA.sustainableSpeed(GF, polAt, vF, U, { ...flexP, FyS: FySafe }, { headings: [90], scanN: 30, dv: 0.1, reach: false, coarse: 2 }).v * 3.6);
    const t = AA.trimState(GF, polAt(9), vF, 12, 9, rho, { ...flexP, FyS: Infinity, scanN: 30 }), LD = t.L / Math.max(1e-9, t.D);
    const tip = AA.tipOverWind(GF, pol, vF, { deltaMax: flexP.deltaMax }) / Math.sqrt(areaR);
    return { fill, v, LD, tip, areaFill: FP.area, zc: FP.zc, torsionFactor: FP.torsionFactor, parts: FP.parts, parkedRatio: areaR }; }) : null;
  return { bands, E, grid, sustained, fills };
}
// run in a module Web Worker when available (keeps the UI thread free on slow devices); inline otherwise
let worker = null, seq = 0; const pending = new Map();
export function flexComputeAsync(args) {
  if (typeof Worker === 'undefined') return Promise.resolve(flexCompute(args));
  try { if (!worker) { worker = new Worker(new URL('../workers/s07flex.js', import.meta.url), { type: 'module' }); worker.onmessage = (e) => { const { id, res, err } = e.data, h = pending.get(id); pending.delete(id); if (h) err ? h.rej(new Error(err)) : h.res(res); }; worker.onerror = () => { for (const [, h] of pending) h.res(null); pending.clear(); worker = null; }; } } catch { return Promise.resolve(flexCompute(args)); }
  const id = ++seq; return new Promise((res, rej) => { pending.set(id, { res, rej }); worker.postMessage({ id, args }); }).then(r => r ?? flexCompute(args));
}
