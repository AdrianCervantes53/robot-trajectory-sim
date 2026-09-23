/**
 * robot3d.js
 * Three.js scene setup and robot mesh management.
 *
 * Responsibilities:
 *  - Owns the renderer, camera, scene, lights, and grid.
 *  - Builds and positions link, joint, and gripper meshes each frame.
 *  - Applies the full 4x4 transform T to the gripper group so q4, q5, q6
 *    orient the end-effector correctly in 3D without any gaps.
 *  - Animates gripper finger open and close states.
 *  - Manages the end-effector trajectory trail.
 *
 * Depends on: config.js (SC)
 * Consumed by: main.js
 */

import { SC } from './config.js';

// Renderer and Camera
const canvas   = document.getElementById('c');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x080b10, 1);
renderer.shadowMap.enabled = true;

const scene  = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 2000);
camera.position.set(160, 120, 180);

const controls = new THREE.OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, 40, 0);
controls.update();

scene.fog = new THREE.FogExp2(0x080b10, 0.0018);

// Lights
scene.add(new THREE.AmbientLight(0x8899cc, 0.4));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(100, 200, 100);
scene.add(dirLight);
const pointCyan = new THREE.PointLight(0x00e5ff, 1.2, 300);
pointCyan.position.set(0, 80, 0);
scene.add(pointCyan);

// Grid and Axes
scene.add(new THREE.GridHelper(400, 40, 0x1c2333, 0x141b26));

const _axLine = (from, to, color) => {
  const g = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(...from), new THREE.Vector3(...to),
  ]);
  return new THREE.Line(g, new THREE.LineBasicMaterial({ color }));
};
scene.add(_axLine([0, 0, 0], [20, 0, 0], 0xff3344));
scene.add(_axLine([0, 0, 0], [0, 20, 0], 0x00ff88));
scene.add(_axLine([0, 0, 0], [0, 0, 20], 0x4488ff));

// Robot base
const baseMesh = new THREE.Mesh(
  new THREE.CylinderGeometry(8, 10, 6, 16),
  new THREE.MeshStandardMaterial({ color: 0x1c2a3a, metalness: 0.8, roughness: 0.4 }),
);
baseMesh.position.set(0, 3, 0);
scene.add(baseMesh);

// Link and Joint meshes
const LINK_COLORS = [0xff5252, 0x00b4d8, 0x06d6a0];
const RADIUS = 3;

const linkMeshes = LINK_COLORS.map(color => {
  const mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(RADIUS, RADIUS, 1, 12),
    new THREE.MeshStandardMaterial({
      color, metalness: 0.5, roughness: 0.35,
      emissive: color, emissiveIntensity: 0.12,
    }),
  );
  scene.add(mesh);
  return mesh;
});

const jointMeshes = Array.from({ length: 4 }, () => {
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(RADIUS * 1.2, 12, 12),
    new THREE.MeshStandardMaterial({
      color: 0xffffff, metalness: 0.9, roughness: 0.1,
      emissive: 0x334455, emissiveIntensity: 0.5,
    }),
  );
  scene.add(mesh);
  return mesh;
});

// Gripper Group
// Origin (0,0,0) is located at the wrist joint (tip of link 3).
// Tool extends forward along local +X. Fingers slide along local Z.
const gripperGroup = new THREE.Group();
const gMat = new THREE.MeshStandardMaterial({ color: 0xd0d8e2, metalness: 0.7, roughness: 0.3 });
const fingerMat = new THREE.MeshStandardMaterial({ color: 0xffaa00, metalness: 0.6, roughness: 0.3 });

// 1. Base collar (starts at x=0, length=6)
const collarGeo = new THREE.CylinderGeometry(RADIUS * 0.9, RADIUS * 0.9, 6, 12);
const collarMesh = new THREE.Mesh(collarGeo, gMat);
collarMesh.rotation.z = -Math.PI / 2; // align along local X
collarMesh.position.set(3, 0, 0);
gripperGroup.add(collarMesh);

// 2. Palm crossbar (centered at x=6, width along Z=18)
const palmGeo = new THREE.BoxGeometry(4, 5, 18);
const palmMesh = new THREE.Mesh(palmGeo, gMat);
palmMesh.position.set(6, 0, 0);
gripperGroup.add(palmMesh);

// 3. Two gripper fingers (extend forward from x=6 to x=22, length=16)
const fingerGeo = new THREE.BoxGeometry(16, 3.2, 3.2);
const fingers = [-1, 1].map(side => {
  const mesh = new THREE.Mesh(fingerGeo, fingerMat);
  mesh.position.set(14, 0, side * 8);
  gripperGroup.add(mesh);
  return { mesh, side };
});

scene.add(gripperGroup);

// End-effector trail
const MAX_TRAIL = 1000;
const trailPositions = new Float32Array(MAX_TRAIL * 3);
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
const trailLine = new THREE.Line(
  trailGeo,
  new THREE.LineBasicMaterial({ color: 0xffb700, linewidth: 2, transparent: true, opacity: 0.7 }),
);
trailLine.frustumCulled = false;
scene.add(trailLine);

// Helpers

/**
 * Converts a robot-space point [x, y, z] to Three.js space.
 * Robot frame is Z-up; Three.js frame is Y-up.
 * Mapping: three_x = robot_x * SC, three_y = robot_z * SC, three_z = -robot_y * SC
 */
function robotToThree(p) {
  return new THREE.Vector3(p[0] * SC, p[2] * SC, -p[1] * SC);
}

function positionCylinder(mesh, from3, to3) {
  const dir = new THREE.Vector3().subVectors(to3, from3);
  const len = dir.length();
  if (len < 0.001) return;
  mesh.position.copy(new THREE.Vector3().addVectors(from3, to3).multiplyScalar(0.5));
  mesh.scale.set(1, len, 1);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
}

/**
 * Converts the 4x4 robot transformation matrix T to Three.js Matrix4.
 * Applies exact change of basis: M * R * M^T and M * p * SC
 * where M = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
 */
function robotTransformToThree(T, scale) {
  const s = scale;
  const tx = T[0][3] * s;
  const ty = T[2][3] * s;
  const tz = -T[1][3] * s;

  const r00 = T[0][0], r01 = T[0][1], r02 = T[0][2];
  const r10 = T[1][0], r11 = T[1][1], r12 = T[1][2];
  const r20 = T[2][0], r21 = T[2][1], r22 = T[2][2];

  // M * R * M^T:
  // Row 0: r00, r02, -r01
  // Row 1: r20, r22, -r21
  // Row 2: -r10, -r12, r11
  const m = new THREE.Matrix4();
  m.set(
     r00,  r02, -r01, tx,
     r20,  r22, -r21, ty,
    -r10, -r12,  r11, tz,
       0,    0,    0,  1,
  );
  return m;
}

// Window resize
function onResize() {
  const w = canvas.parentElement.clientWidth;
  const h = canvas.parentElement.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', onResize);
onResize();

// Render loop
(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
})();

// Public API

/**
 * Updates finger positions based on open/closed state.
 * @param {boolean} isOpen
 */
export function updateGripper(isOpen) {
  const spread = (isOpen !== false) ? 8.0 : 2.0;
  fingers.forEach(({ mesh, side }) => {
    mesh.position.z = side * spread;
  });
}

/**
 * Updates all robot meshes, gripper orientation, and trajectory trail.
 *
 * @param {Array<{from: number[], to: number[]}>} links - 3 link segments.
 * @param {Array<{px: number, py: number, pz: number}>} [trajectory] - Trail points.
 * @param {number[][]} [transform] - 4x4 homogeneous transform T.
 * @param {boolean} [gripperOpen] - Gripper open state.
 */
export function updateRobot(links, trajectory, transform, gripperOpen) {
  const pts = [
    robotToThree(links[0].from),
    robotToThree(links[0].to),
    robotToThree(links[1].to),
    robotToThree(links[2].to),
  ];

  pts.forEach((p, i) => jointMeshes[i]?.position.copy(p));

  positionCylinder(linkMeshes[0], pts[0], pts[1]);
  positionCylinder(linkMeshes[1], pts[1], pts[2]);
  positionCylinder(linkMeshes[2], pts[2], pts[3]);

  // Orient and position gripper with full transform matrix T
  if (transform && transform.length === 4) {
    const mat = robotTransformToThree(transform, SC);
    gripperGroup.matrix.copy(mat);
    gripperGroup.matrixAutoUpdate = false;
    gripperGroup.matrixWorldNeedsUpdate = true; // force world-matrix recompute
  } else {
    gripperGroup.matrixAutoUpdate = true;
    gripperGroup.position.copy(pts[3]);
    gripperGroup.rotation.set(0, 0, 0);
  }

  // Update finger spread
  updateGripper(gripperOpen);

  // HUD
  document.getElementById('h-px').textContent = links[2].to[0].toFixed(2);
  document.getElementById('h-py').textContent = links[2].to[1].toFixed(2);
  document.getElementById('h-pz').textContent = links[2].to[2].toFixed(2);

  // Trajectory Trail
  if (trajectory && trajectory.length > 0) {
    const n = Math.min(trajectory.length, MAX_TRAIL);
    for (let i = 0; i < n; i++) {
      const p = robotToThree([trajectory[i].px, trajectory[i].py, trajectory[i].pz]);
      trailPositions[i * 3]     = p.x;
      trailPositions[i * 3 + 1] = p.y;
      trailPositions[i * 3 + 2] = p.z;
    }
    trailGeo.setDrawRange(0, n);
    trailGeo.attributes.position.needsUpdate = true;
  }
}

/** Clears the end-effector trajectory trail. */
export function clearTrail() {
  trailGeo.setDrawRange(0, 0);
  trailGeo.attributes.position.needsUpdate = true;
}
