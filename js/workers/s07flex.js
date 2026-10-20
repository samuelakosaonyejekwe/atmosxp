// Module worker for Suite 07's heavy flexure-arch computations (see js/core/s07flex.js)
import { flexCompute } from '../core/s07flex.js';
self.onmessage = (e) => { const { id, args } = e.data; try { self.postMessage({ id, res: flexCompute(args) }); } catch (err) { self.postMessage({ id, err: String(err?.message || err) }); } };
