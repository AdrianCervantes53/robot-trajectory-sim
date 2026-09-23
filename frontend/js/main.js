/**
 * main.js
 * Application entry point.
 *
 * Responsibilities:
 *  - Imports and connects all modules together.
 *  - Owns applyState(): syncs UI and 3D scene to a robot state snapshot.
 *  - Attaches all DOM event listeners.
 *  - Runs init() on page load.
 *
 * Depends on: config.js, robot3d.js, ui.js, api.js, websocket.js
 */

import { updateRobot, updateGripper, clearTrail } from './robot3d.js';
import { buildSliders, sliders, setSlider, setStatus, setDot } from './ui.js';
import * as api from './api.js';
import { createWSConnection } from './websocket.js';

// WebSocket instance
let ws;
let _running = false;

// Trajectory selector state
let _selectedTraj = null;

// Gripper state
let _gripperOpen = true;

// STATE SYNC

/**
 * Applies a full robot state snapshot to all UI elements and the 3D scene.
 * @param {object} data - RobotState dict from the API.
 */
function applyState(data) {
  if (!data) return;

  // 3D scene
  if (data.links && data.links.length === 3) {
    updateRobot(data.links, data.trajectory, data.transform, data.gripper_open);
  } else if (data.gripper_open !== undefined) {
    updateGripper(data.gripper_open);
  }

  // Joint sliders and live table
  if (Array.isArray(data.joints_deg)) {
    data.joints_deg.forEach((q, i) => {
      setSlider(i, q);
      const cell = document.getElementById(`lv-q${i + 1}`);
      if (cell) cell.textContent = `${q.toFixed(1)} deg`;
    });
  }

  // Cartesian position table
  if (data.position) {
    document.getElementById('lv-px').textContent = data.position.px !== undefined ? data.position.px.toFixed(2) : '-';
    document.getElementById('lv-py').textContent = data.position.py !== undefined ? data.position.py.toFixed(2) : '-';
    document.getElementById('lv-pz').textContent = data.position.pz !== undefined ? data.position.pz.toFixed(2) : '-';
  }

  // Gripper button
  if (data.gripper_open !== undefined) {
    _gripperOpen = data.gripper_open;
    const btn = document.getElementById('gripper-btn');
    if (btn) {
      btn.textContent = _gripperOpen ? 'Open' : 'Closed';
      btn.className = `gripper-btn ${_gripperOpen ? 'open' : 'closed'}`;
    }
  }
}

// TRAJECTORY TYPE SELECTOR

function selectTraj(type) {
  _selectedTraj = type;
  ['ptp', 'lin', 'cir'].forEach(t => {
    const btn = document.getElementById(`btn-${t}`);
    const active = t === type;
    if (btn) {
      btn.style.borderColor = active ? 'var(--cyan)' : '';
      btn.style.color       = active ? 'var(--cyan)' : '';
    }
    const extra = document.getElementById(`extra-${t}`);
    if (extra) extra.classList.toggle('visible', active);
  });
  const runBtn = document.getElementById('btn-run');
  if (runBtn) runBtn.disabled = false;
  setStatus(`Trajectory ${type.toUpperCase()} selected`, 'ok');
}

// TRAJECTORY EXECUTION

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
      plane: parseInt(document.getElementById('cir-plane').value, 10),
    };
  }
  return null;
}

function runTrajectory() {
  if (!_selectedTraj) { setStatus('Select a trajectory first', 'err'); return; }
  if (!ws.isOpen())   { setStatus('WebSocket not connected', 'err');   return; }
  if (_running)       return;

  const msg = buildTrajectoryMessage();
  if (!msg) return;

  _running = true;
  setDot('run');
  setStatus(`Running ${_selectedTraj.toUpperCase()}...`, '');
  ws.send(msg);
}

function stopTrajectory() {
  ws.close();
  _running = false;
  setStatus('Stopped', 'err');
}

// SAVED TRAJECTORY LIST

async function refreshTrajList() {
  try {
    const data = await api.listTrajectories();
    const ul = document.getElementById('saved-list');
    ul.innerHTML = '';
    (data.trajectories || []).forEach(name => {
      const li = document.createElement('li');
      li.innerHTML = `<span>${name}</span><button class="load-btn">load</button>`;
      li.querySelector('.load-btn').addEventListener('click', () => loadTraj(name));
      ul.appendChild(li);
    });
  } catch {
    setStatus('Failed to list trajectories', 'err');
  }
}

async function loadTraj(name) {
  try {
    const r = await api.loadTrajectory(name);
    if (r.ok) setStatus(`Loaded: ${name}`, 'ok');
    else setStatus('Failed to load trajectory', 'err');
  } catch {
    setStatus('Failed to load trajectory', 'err');
  }
}

// EVENT LISTENERS

function wireEvents() {
  // Slider debounce
  let _joTimer = null;
  buildSliders(() => {
    clearTimeout(_joTimer);
    _joTimer = setTimeout(async () => {
      try {
        const data = await api.postJoints(sliders.map(s => parseFloat(s.value)));
        applyState(data);
      } catch {
        setStatus('Failed to move joints', 'err');
      }
    }, 60);
  });

  // Joint home
  document.getElementById('btn-home').addEventListener('click', async () => {
    try {
      applyState(await api.postJoints([0, 0, 0, 0, 0, 0]));
      setStatus('Home position set', 'ok');
    } catch {
      setStatus('Error setting home', 'err');
    }
  });

  // Clear trajectory trail
  document.getElementById('btn-clear-traj').addEventListener('click', async () => {
    try {
      await api.clearTrajectoryHistory();
      clearTrail();
      setStatus('Trajectory cleared', 'ok');
    } catch {
      setStatus('Error clearing trajectory', 'err');
    }
  });

  // IK pose
  document.getElementById('btn-move-pose').addEventListener('click', async () => {
    const get = id => parseFloat(document.getElementById(id).value);
    try {
      const r = await api.postPose(get('ip-px'), get('ip-py'), get('ip-pz'));
      if (!r.ok) {
        const err = await r.json();
        setStatus(`Singularity: ${err.detail}`, 'err');
        return;
      }
      applyState(await r.json());
      setStatus('Pose applied', 'ok');
    } catch {
      setStatus('Connection error', 'err');
    }
  });

  // Trajectory type buttons
  ['ptp', 'lin', 'cir'].forEach(t => {
    const btn = document.getElementById(`btn-${t}`);
    if (btn) btn.addEventListener('click', () => selectTraj(t));
  });

  // Run / Stop
  document.getElementById('btn-run').addEventListener('click', runTrajectory);
  document.getElementById('btn-stop').addEventListener('click', stopTrajectory);

  // Config
  document.getElementById('btn-config').addEventListener('click', async () => {
    const vel = parseFloat(document.getElementById('cfg-vel').value);
    const dur = parseFloat(document.getElementById('cfg-dur').value);
    try {
      await api.postConfig(vel, dur);
      setStatus('Configuration applied', 'ok');
    } catch {
      setStatus('Failed to apply configuration', 'err');
    }
  });

  // Gripper
  document.getElementById('gripper-btn').addEventListener('click', async () => {
    _gripperOpen = !_gripperOpen;
    updateGripper(_gripperOpen);
    const btn = document.getElementById('gripper-btn');
    btn.textContent = _gripperOpen ? 'Open' : 'Closed';
    btn.className = `gripper-btn ${_gripperOpen ? 'open' : 'closed'}`;
    try {
      const state = await api.postGripper(_gripperOpen);
      applyState(state);
    } catch {
      setStatus('Failed to update gripper on server', 'err');
    }
  });

  // Save / list trajectories
  document.getElementById('btn-save-traj').addEventListener('click', async () => {
    const name = document.getElementById('traj-name').value.trim();
    if (!name) { setStatus('Enter a name first', 'err'); return; }
    try {
      const r = await api.saveTrajectory(name);
      if (r.ok) {
        setStatus(`Saved: ${name}`, 'ok');
        refreshTrajList();
      } else {
        const e = await r.json();
        setStatus(e.detail || 'Failed to save', 'err');
      }
    } catch {
      setStatus('Failed to save trajectory', 'err');
    }
  });
  document.getElementById('btn-list-traj').addEventListener('click', refreshTrajList);
}

// INIT

async function init() {
  wireEvents();

  // WebSocket
  ws = createWSConnection({
    onOpen:  ()      => { setDot('on');  setStatus('Connected', 'ok'); },
    onClose: ()      => { setDot('off'); setStatus('Disconnected - reconnecting...', ''); },
    onError: ()      => setStatus('WebSocket error', 'err'),
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
        setStatus('Trajectory completed', 'ok');
      }
    },
  });

  // Initial REST state
  try {
    applyState(await api.fetchState());
    setStatus('Initial state loaded', 'ok');
  } catch {
    setStatus('API unavailable', 'err');
  }
}

init();
