import { degreesToRadians, dhMatrix, multiplyMatrices, radiansToDegrees, rotationPart, transposeMatrix } from './matrix.js';
import { ROBOT_DIMENSIONS } from './forward.js';

export class SingularityError extends Error {
  constructor(message) {
    super(message);
    this.name = 'SingularityError';
  }
}

function targetRotation(alpha, beta, gamma) {
  const rz = [
    [Math.cos(alpha), -Math.sin(alpha), 0],
    [Math.sin(alpha), Math.cos(alpha), 0],
    [0, 0, 1],
  ];
  const ry = [
    [Math.cos(beta), 0, Math.sin(beta)],
    [0, 1, 0],
    [-Math.sin(beta), 0, Math.cos(beta)],
  ];
  const rx = [
    [1, 0, 0],
    [0, Math.cos(gamma), -Math.sin(gamma)],
    [0, Math.sin(gamma), Math.cos(gamma)],
  ];
  return multiplyMatrices(multiplyMatrices(rz, ry), rx);
}

export function inverseKinematics(position, orientation) {
  const { px, py, pz } = position;
  if (![px, py, pz, orientation.alpha, orientation.beta, orientation.gamma].every(Number.isFinite)) {
    throw new SingularityError('The target pose must contain finite values.');
  }

  const { L1, L2, L3 } = ROBOT_DIMENSIONS;
  const radialSquared = px ** 2 + py ** 2;
  const distanceSquared = radialSquared + (pz - L1) ** 2;
  let cosQ3 = (distanceSquared - L2 ** 2 - L3 ** 2) / (2 * L2 * L3);
  if (Math.abs(cosQ3) > 1 + 1e-6) {
    throw new SingularityError('The requested point is outside the robot workspace.');
  }
  cosQ3 = Math.max(-1, Math.min(1, cosQ3));
  const sinQ3 = Math.sqrt(Math.max(0, 1 - cosQ3 ** 2));
  let q3 = Math.atan2(sinQ3, cosQ3);
  const q1 = Math.atan2(py, px);
  const alfa = Math.atan2(pz - L1, Math.sqrt(radialSquared));
  const beta = Math.atan2(L3 * sinQ3, L2 + L3 * cosQ3);
  let q2 = alfa - beta;
  if (q2 < 0) {
    q2 = alfa + beta;
    q3 = -Math.atan2(sinQ3, cosQ3);
  }

  const armRotation = multiplyMatrices(
    multiplyMatrices(rotationPart(dhMatrix(q1, L1, 0, Math.PI / 2)), rotationPart(dhMatrix(q2, 0, L2, 0))),
    rotationPart(dhMatrix(q3, 0, L3, 0)),
  );
  const wristRotation = multiplyMatrices(transposeMatrix(armRotation), targetRotation(orientation.alpha, orientation.beta, orientation.gamma));
  const q4 = Math.atan2(wristRotation[2][2], wristRotation[1][2]);
  const q5 = Math.atan2(-wristRotation[0][2], Math.hypot(wristRotation[1][2], wristRotation[2][2]));
  const q6 = Math.atan2(-wristRotation[0][1], wristRotation[0][0]);

  return [q1, q2, q3, q4, q5, q6].map(radiansToDegrees);
}

export function degrees(value) {
  return radiansToDegrees(degreesToRadians(value));
}
