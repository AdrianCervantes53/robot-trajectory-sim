export const DEG_TO_RAD = Math.PI / 180;
export const RAD_TO_DEG = 180 / Math.PI;

export function degreesToRadians(value) {
  return value * DEG_TO_RAD;
}

export function radiansToDegrees(value) {
  return value * RAD_TO_DEG;
}

export function roundTo(value, decimals = 2) {
  const factor = 10 ** decimals;
  return Math.round((value + Number.EPSILON) * factor) / factor;
}

export function dhMatrix(theta, d, a, alpha) {
  const cosTheta = Math.cos(theta);
  const sinTheta = Math.sin(theta);
  const cosAlpha = Math.cos(alpha);
  const sinAlpha = Math.sin(alpha);

  return [
    [cosTheta, -cosAlpha * sinTheta, sinAlpha * sinTheta, a * cosTheta],
    [sinTheta, cosAlpha * cosTheta, -sinAlpha * cosTheta, a * sinTheta],
    [0, sinAlpha, cosAlpha, d],
    [0, 0, 0, 1],
  ];
}

export function multiplyMatrices(left, right) {
  return left.map((row) => right[0].map((_, columnIndex) =>
    row.reduce((sum, value, rowIndex) => sum + value * right[rowIndex][columnIndex], 0)));
}

export function transposeMatrix(matrix) {
  return matrix[0].map((_, columnIndex) => matrix.map((row) => row[columnIndex]));
}

export function rotationPart(matrix) {
  return matrix.slice(0, 3).map((row) => row.slice(0, 3));
}
