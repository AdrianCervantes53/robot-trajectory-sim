/**
 * robot3d.js
 * Three.js scene setup and robot mesh management.
 *
 * Responsibilities:
 *  - Owns the renderer, camera, scene, lights, and grid.
 *  - Builds and positions link/joint/gripper meshes each frame.
 *  - Manages the end-effector trail.
 *
 * Depends on: config.js (SC)
 * Consumed by: main.js (updateRobot, clearTrail)
 */

import { SC } from './config.js';

// ── Renderer & Camera ─────────────────────────────────
const canvas   = document.getElementById('c');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(devicePixelRatio);
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

// ── Lights ────────────────────────────────────────────
scene.add(new THREE.AmbientLight(0x8899cc, 0.4));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(100, 200, 100);
scene.add(dirLight);
const pointCyan = new THREE.PointLight(0x00e5ff, 1.2, 300);
pointCyan.position.set(0, 80, 0);
scene.add(pointCyan);

// ── Grid & Axes ───────────────────────────────────────
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

// ── Robot base ────────────────────────────────────────
const baseMesh = new THREE.Mesh(
  new THREE.CylinderGeometry(8, 10, 6, 16),
  new THREE.MeshStandardMaterial({ color: 0x1c2a3a, metalness: .8, roughness: .4 }),
);
baseMesh.position.set(0, 3, 0);
scene.add(baseMesh);

// ── Link & Joint meshes ───────────────────────────────
const LINK_COLORS = [0xff5252, 0x00b4d8, 0x06d6a0];
const RADIUS = 3;

const linkMeshes = LINK_COLORS.map(color => {
  const mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(RADIUS, RADIUS, 1, 12),
    new THREE.MeshStandardMaterial({
      color, metalness: .5, roughness: .35,
      emissive: color, emissiveIntensity: .12,
    }),
  );
  scene.add(mesh);
  return mesh;
});

const jointMeshes = Array.from({ length: 4 }, () => {
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(RADIUS * 1.2, 12, 12),
    new THREE.MeshStandardMaterial({
      color: 0xffffff, metalness: .9, roughness: .1,
      emissive: 0x334455, emissiveIntensity: .5,
    }),
  );
  scene.add(mesh);
  return mesh;
});

// ── Gripper ───────────────────────────────────────────
const gripperGroup = new THREE.Group();
const gMat = new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: .7, roughness: .3 });
[-3, 3].forEach(x => {
  const finger = new THREE.Mesh(new THREE.BoxGeometry(2, 8, 2), gMat);
  finger.position.x = x;
  gripperGroup.add(finger);
});
scene.add(gripperGroup);

// ── End-effector trail ────────────────────────────────
const MAX_TRAIL = 1000;
const trailPositions = new Float32Array(MAX_TRAIL * 3);
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
const trailLine = new THREE.Line(
  trailGeo,
  new THREE.LineBasicMaterial({ color: 0xffb700, linewidth: 2, transparent: true, opacity: .7 }),
);
trailLine.frustumCulled = false;
scene.add(trailLine);

// ── Helpers ───────────────────────────────────────────

/**
 * Converts a MATLAB-space point [x, y, z] to Three.js space.
 * MATLAB uses Z-up; Three.js uses Y-up.
 */
function matlabToThree(p) {
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

// ── Resize ────────────────────────────────────────────
function onResize() {
  const w = canvas.parentElement.clientWidth;
  const h = canvas.parentElement.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', onResize);
onResize();

// ── Render loop ───────────────────────────────────────
(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
})();

// ── Public API ────────────────────────────────────────

/**
 * Updates all robot meshes and the end-effector trail.
 * @param {Array<{from: number[], to: number[]}>} links - Array of 3 link segments.
 * @param {Array<{px: number, py: number, pz: number}>} [trajectory] - Trail points.
 */
export function updateRobot(links, trajectory) {
  const pts = [
    matlabToThree(links[0].from),
    matlabToThree(links[0].to),
    matlabToThree(links[1].to),
    matlabToThree(links[2].to),
  ];

  pts.forEach((p, i) => jointMeshes[i]?.position.copy(p));

  positionCylinder(linkMeshes[0], pts[0], pts[1]);
  positionCylinder(linkMeshes[1], pts[1], pts[2]);
  positionCylinder(linkMeshes[2], pts[2], pts[3]);

  gripperGroup.position.copy(pts[3]);

  // HUD
  document.getElementById('h-px').textContent = links[2].to[0].toFixed(2);
  document.getElementById('h-py').textContent = links[2].to[1].toFixed(2);
  document.getElementById('h-pz').textContent = links[2].to[2].toFixed(2);

  // Trail
  if (trajectory?.length > 0) {
    const n = Math.min(trajectory.length, MAX_TRAIL);
    for (let i = 0; i < n; i++) {
      const p = matlabToThree([trajectory[i].px, trajectory[i].py, trajectory[i].pz]);
      trailPositions[i * 3]     = p.x;
      trailPositions[i * 3 + 1] = p.y;
      trailPositions[i * 3 + 2] = p.z;
    }
    trailGeo.setDrawRange(0, n);
    trailGeo.attributes.position.needsUpdate = true;
  }
}

/** Clears the end-effector trail from the scene. */
export function clearTrail() {
  trailGeo.setDrawRange(0, 0);
  trailGeo.attributes.position.needsUpdate = true;
}
