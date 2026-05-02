/**
 * main.js
 * Application entry point.
 *
 * Responsibilities:
 *  - Imports and wires all modules together.
 *  - Owns applyState(): the single function that syncs the full UI to a robot state snapshot.
 *  - Attaches all DOM event listeners.
 *  - Runs init() on load.
 *
 * Depends on: config.js, robot3d.js, ui.js, api.js, websocket.js
 */

import { updateRobot, clearTrail }              from './robot3d.js';
import { buildSliders, sliders, setSlider,
         setStatus, setDot }                    from './ui.js';
import * as api                                 from './api.js';
import { createWSConnection }                   from './websocket.js';

// ── WebSocket instance ────────────────────────────────
let ws;
let _running = false;

// ── Trajectory selector state ─────────────────────────
let _selectedTraj = null;

// ── Gripper state ─────────────────────────────────────
let _gripperOpen = true;

// ═══════════════════════════════════════════════════════
// STATE SYNC
// ═══════════════════════════════════════════════════════

/**
 * Applies a full robot state snapshot to all UI elements and the 3D scene.
 * This is the single source of truth for rendering; every data source
 * (REST responses, WS frames) funnels through here.
 *
 * @param {object} data - RobotState dict from the API.
 */
function applyState(data) {
  // 3D scene
  if (data.links?.length === 3) {
    updateRobot(data.links, data.trajectory);
  }

  // Joint sliders + live table
  data.joints_deg?.forEach((q, i) => {
    setSlider(i, q);
    const cell = document.getElementById(`lv-q${i + 1}`);
    if (cell) cell.textContent = `${q.toFixed(1)}°`;
  });

  // Cartesian position table
  if (data.position) {
    document.getElementById('lv-px').textContent = data.position.px?.toFixed(2) ?? '—';
    document.getElementById('lv-py').textContent = data.position.py?.toFixed(2) ?? '—';
    document.getElementById('lv-pz').textContent = data.position.pz?.toFixed(2) ?? '—';
  }

  // Gripper button
  if (data.gripper_open !== undefined) {
    _gripperOpen = data.gripper_open;
    const btn = document.getElementById('gripper-btn');
    btn.textContent = _gripperOpen ? '● Open' : '● Closed';
    btn.className = `gripper-btn ${_gripperOpen ? 'open' : 'closed'}`;
  }
}

// ═══════════════════════════════════════════════════════
// TRAJECTORY TYPE SELECTOR
// ═══════════════════════════════════════════════════════

function selectTraj(type) {
  _selectedTraj = type;
  ['ptp', 'lin', 'cir'].forEach(t => {
    const btn = document.getElementById(`btn-${t}`);
    const active = t === type;
    btn.style.borderColor = active ? 'var(--cyan)' : '';
    btn.style.color       = active ? 'var(--cyan)' : '';
    document.getElementById(`extra-${t}`).classList.toggle('visible', active);
  });
  document.getElementById('btn-run').disabled = false;
  setStatus(`Trayectoria ${type.toUpperCase()} seleccionada`, 'ok');
}

// ═══════════════════════════════════════════════════════
// TRAJECTORY EXECUTION
// ═══════════════════════════════════════════════════════

function buildTrajectoryMessage() {
  const get = id => parseFloat(document.getElementById(id).value);

  if (_selectedTraj === 'ptp') {
    return {
      type: 'ptp',
      q_end: ['ptp-q1','ptp-q2','ptp-q3','ptp-q4','ptp-q5','ptp-q6'].map(get),
    };
  }
  if (_selectedTraj === 'lin') {
    return { type: 'linear', p_end: ['lin-px','lin-py','lin-pz'].map(get) };
  }
  if (_selectedTraj === 'cir') {
    return {
      type: 'circular',
      p_mid: ['cir-mx','cir-my','cir-mz'].map(get),
      p_end: ['cir-fx','cir-fy','cir-fz'].map(get),
      plane: parseInt(document.getElementById('cir-plane').value),
    };
  }
  return null;
}

function runTrajectory() {
  if (!_selectedTraj)   { setStatus('Selecciona una trayectoria', 'err'); return; }
  if (!ws.isOpen())     { setStatus('WebSocket no conectado', 'err');     return; }
  if (_running)         return;

  const msg = buildTrajectoryMessage();
  if (!msg) return;

  _running = true;
  setDot('run');
  setStatus(`Ejecutando ${_selectedTraj.toUpperCase()}…`, '');
  ws.send(msg);
}

function stopTrajectory() {
  ws.close();   // triggers onClose → auto-reconnect
  _running = false;
  setStatus('Detenido', 'err');
}

// ═══════════════════════════════════════════════════════
// SAVED TRAJECTORY LIST
// ═══════════════════════════════════════════════════════

async function refreshTrajList() {
  const data = await api.listTrajectories();
  const ul = document.getElementById('saved-list');
  ul.innerHTML = '';
  data.trajectories.forEach(name => {
    const li = document.createElement('li');
    li.innerHTML = `<span>${name}</span>
      <button class="load-btn">load</button>`;
    li.querySelector('.load-btn').addEventListener('click', () => loadTraj(name));
    ul.appendChild(li);
  });
}

async function loadTraj(name) {
  const r = await api.loadTrajectory(name);
  if (r.ok) setStatus(`Cargada: ${name}`, 'ok');
  else setStatus('Error al cargar trayectoria', 'err');
}

// ═══════════════════════════════════════════════════════
// EVENT LISTENERS
// ═══════════════════════════════════════════════════════

function wireEvents() {
  // Slider debounce
  let _joTimer = null;
  buildSliders(() => {
    clearTimeout(_joTimer);
    _joTimer = setTimeout(async () => {
      try {
        const data = await api.postJoints(sliders.map(s => parseFloat(s.value)));
        applyState(data);
      } catch { setStatus('Error al mover juntas', 'err'); }
    }, 60);
  });

  // Joint home
  document.getElementById('btn-home').addEventListener('click', async () => {
    try {
      applyState(await api.postJoints([0, 0, 0, 0, 0, 0]));
      setStatus('Home', 'ok');
    } catch { setStatus('Error', 'err'); }
  });

  // Clear trajectory trail
  document.getElementById('btn-clear-traj').addEventListener('click', async () => {
    await api.clearTrajectoryHistory();
    clearTrail();
    setStatus('Trayectoria limpiada', 'ok');
  });

  // IK pose
  document.getElementById('btn-move-pose').addEventListener('click', async () => {
    const get = id => parseFloat(document.getElementById(id).value);
    try {
      const r = await api.postPose(get('ip-px'), get('ip-py'), get('ip-pz'));
      if (!r.ok) {
        const err = await r.json();
        setStatus(`Singularidad: ${err.detail}`, 'err');
        return;
      }
      applyState(await r.json());
      setStatus('Pose aplicada', 'ok');
    } catch { setStatus('Error de conexión', 'err'); }
  });

  // Trajectory type buttons
  ['ptp', 'lin', 'cir'].forEach(t =>
    document.getElementById(`btn-${t}`).addEventListener('click', () => selectTraj(t)));

  // Run / Stop
  document.getElementById('btn-run').addEventListener('click', runTrajectory);
  document.getElementById('btn-stop').addEventListener('click', stopTrajectory);

  // Config
  document.getElementById('btn-config').addEventListener('click', async () => {
    const vel = parseFloat(document.getElementById('cfg-vel').value);
    const dur = parseFloat(document.getElementById('cfg-dur').value);
    await api.postConfig(vel, dur);
    setStatus('Config aplicada', 'ok');
  });

  // Gripper
  document.getElementById('gripper-btn').addEventListener('click', async () => {
    _gripperOpen = !_gripperOpen;
    applyState(await api.postGripper(_gripperOpen));
  });

  // Save / list trajectories
  document.getElementById('btn-save-traj').addEventListener('click', async () => {
    const name = document.getElementById('traj-name').value.trim();
    if (!name) { setStatus('Escribe un nombre primero', 'err'); return; }
    const r = await api.saveTrajectory(name);
    if (r.ok) { setStatus(`Guardada: ${name}`, 'ok'); refreshTrajList(); }
    else { const e = await r.json(); setStatus(e.detail, 'err'); }
  });
  document.getElementById('btn-list-traj').addEventListener('click', refreshTrajList);
}

// ═══════════════════════════════════════════════════════
// INIT
// ═══════════════════════════════════════════════════════

async function init() {
  wireEvents();

  // WebSocket
  ws = createWSConnection({
    onOpen:  ()      => { setDot('on');  setStatus('Conectado', 'ok'); },
    onClose: ()      => { setDot('off'); setStatus('Desconectado — reintentando…', ''); },
    onError: ()      => setStatus('Error WebSocket', 'err'),
    onFrame: (frame) => {
      if (frame.frame_type === 'error') {
        setStatus(`Error: ${frame.detail}`, 'err');
        setDot('on');
        _running = false;
        return;
      }
      applyState(frame);
      if (frame.frame_type === 'trajectory_end') {
        setDot('on');
        _running = false;
        setStatus('Trayectoria completada', 'ok');
      }
    },
  });

  // Initial REST state
  try {
    applyState(await api.fetchState());
    setStatus('Estado inicial cargado', 'ok');
  } catch {
    setStatus('API no disponible', 'err');
  }
}

init();
