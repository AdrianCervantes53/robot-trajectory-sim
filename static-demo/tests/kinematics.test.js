import { describe, expect, it } from 'vitest';
import { forwardKinematics } from '../src/kinematics/forward.js';
import { inverseKinematics, SingularityError } from '../src/kinematics/inverse.js';

describe('client-side kinematics', () => {
  it('calculates the documented home position', () => {
    const result = forwardKinematics([0, 0, 0, 0, 0, 0]);
    expect(result.position).toEqual({ px: 10, py: 0, pz: 5 });
  });

  it('round-trips a reachable pose', () => {
    const source = forwardKinematics([45, 30, 20, 0, 0, 0]);
    const solved = inverseKinematics(source.position, source.orientation);
    const restored = forwardKinematics(solved);
    expect(restored.position.px).toBeCloseTo(source.position.px, 1);
    expect(restored.position.py).toBeCloseTo(source.position.py, 1);
    expect(restored.position.pz).toBeCloseTo(source.position.pz, 1);
  });

  it('rejects unreachable positions', () => {
    expect(() => inverseKinematics({ px: 999, py: 999, pz: 999 }, { alpha: 0, beta: 0, gamma: 0 })).toThrow(SingularityError);
  });
});
