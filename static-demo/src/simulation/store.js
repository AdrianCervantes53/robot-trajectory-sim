import { forwardKinematics } from '../kinematics/forward.js';

const STORAGE_KEY = 'robosim.static.trajectories.v1';
const MAX_TRAJECTORY_POINTS = 1000;
const NAME_PATTERN = /^[a-zA-Z0-9_-]{1,40}$/;

function clone(value) {
  return structuredClone(value);
}

export class SimulationStore {
  constructor() {
    this.state = this.createInitialState();
    this.savedTrajectories = this.readSavedTrajectories();
  }

  createInitialState() {
    const result = forwardKinematics([0, 0, 0, 0, 0, 0]);
    return { ...result, trajectory: [], gripper_open: true, velocity_pct: 50, trajectory_duration: 2 };
  }

  snapshot() {
    return clone(this.state);
  }

  applyKinematics(result, recordPoint = true) {
    this.state = { ...this.state, ...result };
    if (recordPoint) {
      this.state.trajectory = [...this.state.trajectory, { ...result.position }].slice(-MAX_TRAJECTORY_POINTS);
    }
    return this.snapshot();
  }

  moveJoints(joints) {
    return this.applyKinematics(forwardKinematics(joints));
  }

  moveToResult(result) {
    return this.applyKinematics(result);
  }

  setGripper(isOpen) {
    this.state.gripper_open = Boolean(isOpen);
    return this.snapshot();
  }

  setConfiguration(velocity, duration) {
    if (!Number.isFinite(velocity) || velocity < 0 || velocity > 100) throw new RangeError('Velocity must be between 0 and 100.');
    if (!Number.isFinite(duration) || duration <= 0 || duration > 30) throw new RangeError('Duration must be between 0 and 30 seconds.');
    this.state.velocity_pct = velocity;
    this.state.trajectory_duration = duration;
    return this.snapshot();
  }

  clearTrajectory() {
    this.state.trajectory = [];
    return this.snapshot();
  }

  saveTrajectory(name) {
    if (!NAME_PATTERN.test(name)) throw new Error('Use 1-40 letters, numbers, underscores, or hyphens.');
    if (this.state.trajectory.length === 0) throw new Error('Create a trajectory before saving it.');
    this.savedTrajectories[name] = clone(this.state.trajectory);
    localStorage.setItem(STORAGE_KEY, JSON.stringify(this.savedTrajectories));
  }

  loadTrajectory(name) {
    const trajectory = this.savedTrajectories[name];
    if (!trajectory) throw new Error('Trajectory not found in this browser.');
    this.state.trajectory = clone(trajectory);
    return this.snapshot();
  }

  listTrajectories() {
    return Object.keys(this.savedTrajectories).sort((left, right) => left.localeCompare(right));
  }

  readSavedTrajectories() {
    try {
      const parsed = JSON.parse(localStorage.getItem(STORAGE_KEY) ?? '{}');
      return Object.fromEntries(Object.entries(parsed).filter(([name, points]) => NAME_PATTERN.test(name) && Array.isArray(points)));
    } catch {
      return {};
    }
  }
}
