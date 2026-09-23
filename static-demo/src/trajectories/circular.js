import { forwardKinematics } from '../kinematics/forward.js';
import { inverseKinematics } from '../kinematics/inverse.js';

function project(point, plane) {
  if (plane === 1) return [point[0], point[1]];
  if (plane === 2) return [point[0], point[2]];
  return [point[1], point[2]];
}

function unproject(point, reference, plane) {
  if (plane === 1) return { px: point[0], py: point[1], pz: reference[2] };
  if (plane === 2) return { px: point[0], py: reference[1], pz: point[1] };
  return { px: reference[0], py: point[0], pz: point[1] };
}

export function circumcenter(first, second, third) {
  const [ax, ay] = first;
  const [bx, by] = second;
  const [cx, cy] = third;
  const determinant = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
  if (Math.abs(determinant) < 1e-10) throw new Error('The circular trajectory points are collinear.');
  const x = ((ax ** 2 + ay ** 2) * (by - cy) + (bx ** 2 + by ** 2) * (cy - ay) + (cx ** 2 + cy ** 2) * (ay - by)) / determinant;
  const y = ((ax ** 2 + ay ** 2) * (cx - bx) + (bx ** 2 + by ** 2) * (ax - cx) + (cx ** 2 + cy ** 2) * (bx - ax)) / determinant;
  return { center: [x, y], radius: Math.hypot(first[0] - x, first[1] - y) };
}

function arcDirection(first, middle, last) {
  if (last > middle && middle > first) return [1, middle, last];
  if (first > middle && middle > last) return [-1, middle, last];
  if (first > last && last > middle) return [1, middle + 360, last];
  if (middle > last && last > first) return [-1, middle - 360, last];
  if (middle > first && first > last) return [1, middle, last + 360];
  return [-1, middle, last - 360];
}

export function* circularTrajectory(startPosition, middlePosition, endPosition, currentJoints, plane = 1, duration = 2, stepSeconds = 0.05) {
  const first = project(startPosition, plane);
  const middle = project(middlePosition, plane);
  const last = project(endPosition, plane);
  const { center, radius } = circumcenter(first, middle, last);
  const firstAngle = Math.atan2(first[1] - center[1], first[0] - center[0]) * 180 / Math.PI;
  const middleAngle = Math.atan2(middle[1] - center[1], middle[0] - center[0]) * 180 / Math.PI;
  const lastAngle = Math.atan2(last[1] - center[1], last[0] - center[0]) * 180 / Math.PI;
  const [direction, adjustedMiddle, adjustedLast] = arcDirection(firstAngle, middleAngle, lastAngle);
  const totalAngle = Math.abs(adjustedMiddle - firstAngle) + Math.abs(adjustedLast - adjustedMiddle) || 360;
  const angleStep = totalAngle / Math.max(Math.floor(duration / stepSeconds), 1);
  const orientation = forwardKinematics(currentJoints).orientation;
  const wrist = currentJoints.slice(3);

  const emitRange = function* (from, to) {
    for (let angle = from; direction > 0 ? angle <= to : angle >= to; angle += direction * angleStep) {
      const radians = angle * Math.PI / 180;
      const position = unproject([radius * Math.cos(radians) + center[0], radius * Math.sin(radians) + center[1]], startPosition, plane);
      const joints = inverseKinematics(position, orientation);
      yield forwardKinematics([...joints.slice(0, 3), ...wrist]);
    }
  };

  yield* emitRange(firstAngle, adjustedMiddle);
  yield* emitRange(adjustedMiddle, adjustedLast);
  const finalJoints = inverseKinematics(unproject(last, startPosition, plane), orientation);
  yield forwardKinematics([...finalJoints.slice(0, 3), ...wrist]);
}
