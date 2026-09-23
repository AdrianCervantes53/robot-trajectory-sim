import { degreesToRadians, dhMatrix, multiplyMatrices, roundTo } from './matrix.js';

export const ROBOT_DIMENSIONS = Object.freeze({ L1: 5, L2: 5, L3: 5 });

export function forwardKinematics(jointsDeg) {
  if (!Array.isArray(jointsDeg) || jointsDeg.length !== 6 || jointsDeg.some((value) => !Number.isFinite(value))) {
    throw new TypeError('Forward kinematics requires six finite joint angles.');
  }

  const [q1, q2, q3, q4, q5, q6] = jointsDeg.map(degreesToRadians);
  const { L1, L2, L3 } = ROBOT_DIMENSIONS;
  const a1 = dhMatrix(q1, L1, 0, Math.PI / 2);
  const a2 = dhMatrix(q2, 0, L2, 0);
  const a3 = dhMatrix(q3, 0, L3, 0);
  const a4 = dhMatrix(0, 0, 0, q4);
  const a5 = dhMatrix(q5, 0, 0, -Math.PI / 2);
  const a6 = dhMatrix(q6, 0, 0, 0);
  const a21 = multiplyMatrices(a1, a2);
  const a321 = multiplyMatrices(a21, a3);
  const transform = multiplyMatrices(multiplyMatrices(multiplyMatrices(a321, a4), a5), a6);

  const position = {
    px: roundTo(transform[0][3]),
    py: roundTo(transform[1][3]),
    pz: roundTo(transform[2][3]),
  };

  return {
    joints_deg: jointsDeg.map((value) => roundTo(value)),
    position,
    orientation: {
      alpha: Math.atan2(transform[1][0], transform[0][0]),
      beta: Math.atan2(-transform[2][0], Math.hypot(transform[0][0], transform[1][0])),
      gamma: Math.atan2(transform[2][1], transform[2][2]),
    },
    links: [
      { from: [0, 0, 0], to: [a1[0][3], a1[1][3], a1[2][3]] },
      { from: [a1[0][3], a1[1][3], a1[2][3]], to: [a21[0][3], a21[1][3], a21[2][3]] },
      { from: [a21[0][3], a21[1][3], a21[2][3]], to: [position.px, position.py, position.pz] },
    ],
    transform,
    A1: a1,
    A2: a2,
    A3: a3,
  };
}
