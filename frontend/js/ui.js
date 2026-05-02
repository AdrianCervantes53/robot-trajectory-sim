/**
 * ui.js
 * DOM helpers: joint sliders, status bar, WebSocket indicator.
 *
 * Responsibilities:
 *  - Builds the slider rows for each robot joint.
 *  - Exposes setStatus() and setDot() for global UI feedback.
 *  - Exports the sliders array so other modules can read/write joint values.
 *
 * Consumed by: main.js
 */

export const JOINT_LIMITS = [
  [-180, 180],
  [-90,  90],
  [-90,  90],
  [-180, 180],
  [-90,  90],
  [-180, 180],
];

/** Live references to each joint's <input type="range"> element. */
export const sliders = [];

/**
 * Builds the 6 joint slider rows and appends them to #joints-ctrl.
 * @param {function(number[]): void} onChange - Called with current joints_deg on any slider input.
 */
export function buildSliders(onChange) {
  const container = document.getElementById('joints-ctrl');

  JOINT_LIMITS.forEach(([min, max], i) => {
    const row = document.createElement('div');
    row.className = 'joint-row';
    row.innerHTML = `
      <span class="joint-label">q${i + 1}</span>
      <input type="range" min="${min}" max="${max}" value="0" step="1" id="sl-${i}">
      <span class="joint-val" id="sv-${i}">0°</span>
    `;
    container.appendChild(row);

    const sl = row.querySelector(`#sl-${i}`);
    const sv = row.querySelector(`#sv-${i}`);

    sl.addEventListener('input', () => {
      sv.textContent = `${sl.value}°`;
      onChange(sliders.map(s => parseFloat(s.value)));
    });

    sliders.push(sl);
  });
}

/**
 * Updates a joint slider position and its displayed value label.
 * Does not fire the input event (avoids feedback loops with applyState).
 * @param {number} index - Joint index (0-based).
 * @param {number} valueDeg - Angle in degrees.
 */
export function setSlider(index, valueDeg) {
  const rounded = Math.round(valueDeg);
  if (sliders[index]) sliders[index].value = rounded;
  const sv = document.getElementById(`sv-${index}`);
  if (sv) sv.textContent = `${rounded}°`;
}

/**
 * Updates the status bar message.
 * @param {string} msg
 * @param {'ok' | 'err' | ''} type
 */
export function setStatus(msg, type) {
  const el = document.getElementById('status-msg');
  el.textContent = msg;
  el.className = type === 'ok' ? 'ok' : type === 'err' ? 'err' : '';
}

/**
 * Updates the WebSocket connection indicator dot and label.
 * @param {'on' | 'run' | 'off'} state
 */
export function setDot(state) {
  const dot = document.getElementById('ws-dot');
  const lbl = document.getElementById('ws-label');
  dot.className = `dot${state === 'on' ? ' on' : state === 'run' ? ' run' : ''}`;
  lbl.textContent = state === 'on' ? 'Connected' : state === 'run' ? 'Running' : 'Disconnected';
}
