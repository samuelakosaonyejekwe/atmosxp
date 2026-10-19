// The 18 standard simulation scenarios generated automatically (input/output specification §11).
import { h, toast } from '../core/ui.js';
import { runSuite } from '../core/pipeline.js';
import { store } from '../core/state.js';
export const title = 'Standard scenarios';

export const STANDARD = [
  [1, 'Stationary, favourable wind', 'Vehicle at rest, prescribed wing incidence and wind', 'Can the vehicle self-start?', 'favorable', 's20'],
  [2, 'Wind-powered acceleration', 'Transient apparent wind and road resistance', 'Acceleration, speed and thrust', 'acceleration', 's20'],
  [3, 'Strong crosswind', 'High aerodynamic side loads', 'Steering, grip and overturning limits', 'crosswind', 's11'],
  [4, 'Sudden gust', 'Gust time history and deformable wing', 'Morphing response and structural safety', 'gust', 's08'],
  [5, 'Wind reversal', 'Wind direction changes through a defined sequence', 'Loss/reversal of thrust and control response', 'reversal', 's19'],
  [6, 'Calm sunny day', 'Solar radiation and atmospheric heat sinks', 'Available continuous thermal propulsion', 'sunnyCalm', 's17'],
  [7, 'Calm clear night', 'Ambient-to-sky thermal gradient', 'Maximum sustainable night-time backup power', 'clearNight', 's17'],
  [8, 'Calm cloudy night', 'Reduced sky-cooling potential', 'Power deficit and reserve endurance', 'cloudyNight', 's18'],
  [9, 'Thermal actuator start-up', 'NiTi initial phase state and thermal interfaces', 'Start-up work, force and thermal response', 'sunnyCalm', 's15'],
  [10, 'Direct-rim hub operation', 'Cartridge cycle and wheel-speed demand', 'Net positive or negative work per revolution', 'sunnyCalm', 's17'],
  [11, 'Wind-to-backup transition', 'Declining wind and available backup power', 'Engagement behaviour and speed continuity', 'reversal', 's19'],
  [12, 'Backup-to-wind transition', 'Increasing useful wind thrust', 'Disengagement response and stability', 'acceleration', 's19'],
  [13, 'Emergency braking', 'Initial speed, wing thrust and road conditions', 'Stopping distance and wheel stability', 'crosswind', 's12'],
  [14, 'Fatigue and durability', 'Repeated thermal, aerodynamic and road cycles', 'Life and likely failure mechanisms', 'gust', 's23'],
  [15, 'Material competition', 'Common loads and environmental conditions', 'Ranked materials with uncertainty', 'sunnyCalm', 's16'],
  [16, 'Global optimisation', 'Adjustable geometry and material variables', 'Best feasible vehicle configurations', 'favorable', 's25'],
  [17, 'Day/night journey', 'Time-dependent weather and route', 'Distance, speed, reserve use and interruptions', 'live', 's20'],
  [18, 'Worst-case energy test', 'Insufficient wind, weak radiation and depleted buffer', 'Whether propulsion can continue without violating conservation laws', 'worstCase', 's20'],
];

export async function render(root) {
  root.append(h('h1', {}, 'Standard simulation scenarios'), h('p', { class: 'muted', style: 'max-width:980px' }, 'Ready-made engineering experiments. Launching one sets the atmospheric boundary conditions (Suite 05), runs them through the coupling chain and opens the suite that answers the scenario’s question. Scenario 18 must never display a successful powered journey unless adequate environmental or stored energy is present.'));
  const grid = h('div', { class: 'suite-grid' });
  for (const [n, name, setup, result, sc, target] of STANDARD) grid.append(h('button', { class: 'suite-card', style: 'text-align:left;cursor:pointer;font:inherit', onclick: async (e) => { const b = e.currentTarget; b.disabled = true; b.querySelector('.d').textContent = 'Preparing boundary conditions…'; try { store.setParam('s05', 'scenario', sc, 'user'); await runSuite('s05', { scenario: sc }); store.project.scenario = sc; location.hash = `#/suite/${target}`; } catch (err) { toast('Scenario set-up failed: ' + err.message, 'fail'); b.disabled = false; } } }, h('div', { class: 'top' }, h('span', { class: 'no' }, 'SCENARIO ' + String(n).padStart(2, '0')), h('span', { class: 'ic' }, '▶')), h('b', {}, name), h('span', { class: 'd' }, setup), h('span', { class: 'd', style: 'color:var(--accent)' }, '→ ' + result)));
  root.append(grid);
}
