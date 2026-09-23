import { degreesToRadians, radiansToDegrees } from '../kinematics/matrix.js';
import { forwardKinematics } from '../kinematics/forward.js';

function polynomialCoefficients(start, end, startVelocity, endVelocity, duration) {
  const a0 = start;
  const a1 = startVelocity;
  const a2 = 0;
  const a3 = (20 * (end - start) - (8 * endVelocity + 12 * startVelocity) * duration) / (2 * duration ** 3);
  const a4 = (30 * (start - end) + (14 * endVelocity + 16 * startVelocity) * duration) / (2 * duration ** 4);
  return [a4, a3, a2, a1, a0];
}

function evaluatePolynomial(coefficients, time) {
  return coefficients.reduce((value, coefficient) => value * time + coefficient, 0);
}

function buildProfile(start, end, duration) {
  const quarter = duration * 0.25;
  const accelerationEnd = quarter * 2;
  const decelerationStart = duration - quarter * 2;
  const velocity = duration - quarter * 2 > 1e-10 ? (end - start) / (duration - quarter * 2) : 0;
  const accelerationPosition = velocity * (accelerationEnd - quarter) + start;
  const decelerationPosition = velocity * (decelerationStart - quarter) + start;
  return {
    acceleration: polynomialCoefficients(start, accelerationPosition, 0, velocity, accelerationEnd),
    velocity,
    accelerationPosition,
    quarter,
    deceleration: polynomialCoefficients(decelerationPosition, end, velocity, 0, duration - decelerationStart),
    accelerationEnd,
    decelerationStart,
  };
}

function evaluateProfile(profile, time) {
  if (time <= profile.accelerationEnd) return evaluatePolynomial(profile.acceleration, time);
  if (time <= profile.decelerationStart) return profile.velocity * (time - profile.quarter) + profile.accelerationPosition;
  return evaluatePolynomial(profile.deceleration, time - profile.decelerationStart);
}

export function* ptpTrajectory(startJoints, endJoints, velocityPercent, stepSeconds = 0.1) {
  const startArm = startJoints.slice(0, 3).map(degreesToRadians);
  const endArm = endJoints.slice(0, 3).map(degreesToRadians);
  const distance = Math.max(...startArm.map((value, index) => Math.abs(endArm[index] - value)));
  const duration = Math.max(3 * (Math.abs(velocityPercent - 100) / 100) * (distance / Math.PI), 0.1);
  const profiles = startArm.map((value, index) => buildProfile(value, endArm[index], duration));

  for (let time = 0; ; time = Math.min(time + stepSeconds, duration)) {
    const arm = profiles.map((profile) => radiansToDegrees(evaluateProfile(profile, time)));
    yield forwardKinematics([...arm, ...startJoints.slice(3)]);
    if (time >= duration) return;
  }
}
