import { forwardKinematics } from '../kinematics/forward.js';
import { inverseKinematics } from '../kinematics/inverse.js';

export function* linearTrajectory(startPosition, endPosition, currentJoints, duration = 2, stepSeconds = 0.1) {
  const steps = Math.max(Math.floor(duration / stepSeconds), 1);
  const wrist = currentJoints.slice(3);
  const orientation = forwardKinematics(currentJoints).orientation;
  let joints = [...currentJoints];

  for (let index = 0; index <= steps; index += 1) {
    const ratio = index / steps;
    const position = {
      px: startPosition.px + (endPosition[0] - startPosition.px) * ratio,
      py: startPosition.py + (endPosition[1] - startPosition.py) * ratio,
      pz: startPosition.pz + (endPosition[2] - startPosition.pz) * ratio,
    };
    const solvedJoints = inverseKinematics(position, orientation);
    joints = [...solvedJoints.slice(0, 3), ...wrist];
    yield forwardKinematics(joints);
  }
}
