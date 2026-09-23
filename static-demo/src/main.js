import './styles.css';
import { forwardKinematics } from './kinematics/forward.js';
import { inverseKinematics } from './kinematics/inverse.js';
import { ptpTrajectory } from './trajectories/ptp.js';
import { linearTrajectory } from './trajectories/linear.js';
import { circularTrajectory } from './trajectories/circular.js';
import { createRobotRenderer } from './render/robot3d.js';
import { SimulationStore } from './simulation/store.js';

const JOINT_LIMITS = [[-180, 180], [-90, 90], [-90, 90], [-180, 180], [-90, 90], [-180, 180]];
const store = new SimulationStore();
const renderer = createRobotRenderer(document.querySelector('#robot-canvas'));
let selectedTrajectory = 'ptp';
let playbackController = null;
let sliders = [];

function setStatus(message, type = '') {
  const element = document.querySelector('#status-message');
  element.textContent = message;
  element.className = type;
}

function setRunning(isRunning) {
  document.querySelector('#status-dot').className = `dot ${isRunning ? 'run' : 'on'}`;
  document.querySelector('#mode-label').textContent = isRunning ? 'Running locally' : 'Local simulation';
}

function applyState(snapshot) {
  renderer.render(snapshot);
  snapshot.joints_deg.forEach((value, index) => {
    sliders[index].value = String(Math.round(value));
    document.querySelector(`#joint-value-${index}`).textContent = `${value.toFixed(1)} deg`;
  });
  document.querySelector('#hud-position').textContent = `${snapshot.position.px.toFixed(2)}, ${snapshot.position.py.toFixed(2)}, ${snapshot.position.pz.toFixed(2)}`;
  const tbody = document.querySelector('#state-table tbody');
  tbody.replaceChildren(...[
    ...snapshot.joints_deg.map((value, index) => createStateRow(`q${index + 1}`, `${value.toFixed(2)} deg`)),
    createStateRow('px', snapshot.position.px.toFixed(2)),
    createStateRow('py', snapshot.position.py.toFixed(2)),
    createStateRow('pz', snapshot.position.pz.toFixed(2)),
  ]);
  const gripperButton = document.querySelector('#gripper-button');
  gripperButton.textContent = snapshot.gripper_open ? 'Open' : 'Closed';
  gripperButton.className = `gripper-button ${snapshot.gripper_open ? 'open' : 'closed'}`;
}

function createStateRow(label, value) {
  const row = document.createElement('tr');
  const labelCell = document.createElement('td');
  const valueCell = document.createElement('td');
  labelCell.textContent = label;
  valueCell.textContent = value;
  row.append(labelCell, valueCell);
  return row;
}

function numberValue(id) {
  const value = Number(document.querySelector(`#${id}`).value);
  if (!Number.isFinite(value)) throw new Error('All values must be finite numbers.');
  return value;
}

function buildJointControls() {
  const container = document.querySelector('#joint-controls');
  sliders = JOINT_LIMITS.map(([minimum, maximum], index) => {
    const row = document.createElement('div');
    row.className = 'joint-row';
    const label = document.createElement('span');
    label.textContent = `q${index + 1}`;
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = String(minimum);
    slider.max = String(maximum);
    slider.value = '0';
    const value = document.createElement('span');
    value.id = `joint-value-${index}`;
    row.append(label, slider, value);
    container.append(row);
    slider.addEventListener('input', () => {
      try {
        applyState(store.moveJoints(sliders.map((item) => Number(item.value))));
      } catch (error) {
        setStatus(error.message, 'error');
      }
    });
    return slider;
  });
}

function refreshSavedTrajectories() {
  const list = document.querySelector('#saved-list');
  list.replaceChildren(...store.listTrajectories().map((name) => {
    const item = document.createElement('li');
    const label = document.createElement('span');
    label.textContent = name;
    const load = document.createElement('button');
    load.className = 'load-button';
    load.textContent = 'Load';
    load.addEventListener('click', () => {
      applyState(store.loadTrajectory(name));
      setStatus(`Loaded ${name}`, 'success');
    });
    item.append(label, load);
    return item;
  }));
}

function selectTrajectory(type) {
  selectedTrajectory = type;
  document.querySelectorAll('.trajectory-button').forEach((button) => button.classList.toggle('active', button.dataset.trajectory === type));
  document.querySelectorAll('.trajectory-inputs').forEach((element) => element.classList.add('hidden'));
  document.querySelector(`#${type}-inputs`).classList.remove('hidden');
}

function createTrajectory() {
  const snapshot = store.snapshot();
  if (selectedTrajectory === 'ptp') {
    const destination = [1, 2, 3, 4, 5, 6].map((index) => numberValue(`ptp-q${index}`));
    return { frames: [...ptpTrajectory(snapshot.joints_deg, destination, snapshot.velocity_pct)], frameDuration: 100 };
  }
  if (selectedTrajectory === 'linear') {
    const destination = ['px', 'py', 'pz'].map((axis) => numberValue(`linear-${axis}`));
    return { frames: [...linearTrajectory(snapshot.position, destination, snapshot.joints_deg, snapshot.trajectory_duration)], frameDuration: 100 };
  }
  const middle = ['mx', 'my', 'mz'].map((axis) => numberValue(`circular-${axis}`));
  const destination = ['fx', 'fy', 'fz'].map((axis) => numberValue(`circular-${axis}`));
  const plane = numberValue('circular-plane');
  const frames = [...circularTrajectory([snapshot.position.px, snapshot.position.py, snapshot.position.pz], middle, destination, snapshot.joints_deg, plane, snapshot.trajectory_duration)];
  return { frames, frameDuration: (snapshot.trajectory_duration * 1000) / Math.max(frames.length, 1) };
}

async function playTrajectory() {
  if (playbackController) return;
  try {
    const { frames, frameDuration } = createTrajectory();
    playbackController = new AbortController();
    setRunning(true);
    for (const frame of frames) {
      if (playbackController.signal.aborted) throw new DOMException('Trajectory stopped.', 'AbortError');
      applyState(store.moveToResult(frame));
      await new Promise((resolve, reject) => {
        const timer = window.setTimeout(resolve, Math.max(frameDuration, 16));
        playbackController.signal.addEventListener('abort', () => { window.clearTimeout(timer); reject(new DOMException('Trajectory stopped.', 'AbortError')); }, { once: true });
      });
    }
    setStatus('Trajectory completed', 'success');
  } catch (error) {
    setStatus(error.name === 'AbortError' ? 'Trajectory stopped' : error.message, error.name === 'AbortError' ? '' : 'error');
  } finally {
    playbackController = null;
    setRunning(false);
  }
}

function wireEvents() {
  document.querySelector('#home-button').addEventListener('click', () => applyState(store.moveJoints([0, 0, 0, 0, 0, 0])));
  document.querySelector('#clear-button').addEventListener('click', () => { applyState(store.clearTrajectory()); setStatus('Trajectory cleared', 'success'); });
  document.querySelector('#pose-button').addEventListener('click', () => {
    try {
      const target = { px: numberValue('pose-px'), py: numberValue('pose-py'), pz: numberValue('pose-pz') };
      const joints = inverseKinematics(target, store.snapshot().orientation);
      applyState(store.moveJoints(joints));
      setStatus('Pose applied', 'success');
    } catch (error) { setStatus(error.message, 'error'); }
  });
  document.querySelectorAll('.trajectory-button').forEach((button) => button.addEventListener('click', () => selectTrajectory(button.dataset.trajectory)));
  document.querySelector('#run-button').addEventListener('click', playTrajectory);
  document.querySelector('#stop-button').addEventListener('click', () => playbackController?.abort());
  document.querySelector('#config-button').addEventListener('click', () => {
    try { applyState(store.setConfiguration(numberValue('velocity-input'), numberValue('duration-input'))); setStatus('Configuration applied', 'success'); }
    catch (error) { setStatus(error.message, 'error'); }
  });
  document.querySelector('#gripper-button').addEventListener('click', () => applyState(store.setGripper(!store.snapshot().gripper_open)));
  document.querySelector('#save-button').addEventListener('click', () => {
    try { store.saveTrajectory(document.querySelector('#trajectory-name').value.trim()); refreshSavedTrajectories(); setStatus('Trajectory saved in this browser', 'success'); }
    catch (error) { setStatus(error.message, 'error'); }
  });
  document.querySelector('#refresh-button').addEventListener('click', refreshSavedTrajectories);
}

buildJointControls();
wireEvents();
selectTrajectory('ptp');
refreshSavedTrajectories();
applyState(store.snapshot());
