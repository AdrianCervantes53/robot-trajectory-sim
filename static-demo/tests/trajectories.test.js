import { describe, expect, it } from 'vitest';
import { ptpTrajectory } from '../src/trajectories/ptp.js';
import { linearTrajectory } from '../src/trajectories/linear.js';
import { circularTrajectory, circumcenter } from '../src/trajectories/circular.js';
import { forwardKinematics } from '../src/kinematics/forward.js';

describe('client-side trajectories', () => {
  it('ends a PTP trajectory at the requested arm position', () => {
    const frames = [...ptpTrajectory([0, 0, 0, 0, 0, 0], [45, 30, 20, 0, 0, 0], 50)];
    expect(frames.at(-1).joints_deg.slice(0, 3)).toEqual([45, 30, 20]);
  });

  it('creates a linear trajectory between reachable endpoints', () => {
    const start = forwardKinematics([30, 20, 10, 0, 0, 0]);
    const end = forwardKinematics([50, 10, 15, 0, 0, 0]);
    const frames = [...linearTrajectory(start.position, Object.values(end.position), start.joints_deg, 1)];
    expect(frames.at(-1).position.px).toBeCloseTo(end.position.px, 1);
  });

  it('calculates a circular center and rejects collinear points', () => {
    expect(circumcenter([0, 0], [4, 0], [0, 4]).center).toEqual([2, 2]);
    expect(() => circumcenter([0, 0], [1, 1], [2, 2])).toThrow('collinear');
  });

  it('emits circular frames for a reachable arc', () => {
    const frames = [...circularTrajectory([7, 0, 5], [4, 3, 5], [1, 0, 5], [0, 0, 0, 0, 0, 0])];
    expect(frames.length).toBeGreaterThan(2);
  });
});
