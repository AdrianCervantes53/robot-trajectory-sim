import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

const SCALE = 18;
const MAX_TRAIL = 1000;

function robotToThree(point) {
  return new THREE.Vector3(point[0] * SCALE, point[2] * SCALE, -point[1] * SCALE);
}

function positionCylinder(mesh, start, end) {
  const direction = new THREE.Vector3().subVectors(end, start);
  const length = direction.length();
  if (length < 0.001) return;
  mesh.position.copy(new THREE.Vector3().addVectors(start, end).multiplyScalar(0.5));
  mesh.scale.set(1, length, 1);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
}

function transformToThree(transform) {
  const [r0, r1, r2] = transform;
  return new THREE.Matrix4().set(
    r0[0], r0[2], -r0[1], r0[3] * SCALE,
    r2[0], r2[2], -r2[1], r2[3] * SCALE,
    -r1[0], -r1[2], r1[1], -r1[3] * SCALE,
    0, 0, 0, 1,
  );
}

export function createRobotRenderer(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setClearColor(0x080b10, 1);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 2000);
  camera.position.set(160, 120, 180);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.target.set(0, 40, 0);
  scene.add(new THREE.AmbientLight(0x8899cc, 0.5));
  const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
  directionalLight.position.set(100, 200, 100);
  scene.add(directionalLight, new THREE.GridHelper(400, 40, 0x1c2333, 0x141b26));
  scene.add(new THREE.Mesh(new THREE.CylinderGeometry(8, 10, 6, 16), new THREE.MeshStandardMaterial({ color: 0x1c2a3a, metalness: 0.8, roughness: 0.4 })));

  const linkMeshes = [0xff5252, 0x00b4d8, 0x06d6a0].map((color) => {
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(3, 3, 1, 12), new THREE.MeshStandardMaterial({ color, metalness: 0.5, roughness: 0.35 }));
    scene.add(mesh);
    return mesh;
  });
  const jointMeshes = Array.from({ length: 4 }, () => {
    const mesh = new THREE.Mesh(new THREE.SphereGeometry(3.6, 12, 12), new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.8, roughness: 0.2 }));
    scene.add(mesh);
    return mesh;
  });
  const gripper = new THREE.Group();
  const gripperMaterial = new THREE.MeshStandardMaterial({ color: 0xd0d8e2, metalness: 0.7, roughness: 0.3 });
  const fingerMaterial = new THREE.MeshStandardMaterial({ color: 0xffaa00, metalness: 0.6, roughness: 0.3 });
  const collar = new THREE.Mesh(new THREE.CylinderGeometry(2.7, 2.7, 6, 12), gripperMaterial);
  collar.rotation.z = -Math.PI / 2;
  collar.position.x = 3;
  gripper.add(collar);
  const palm = new THREE.Mesh(new THREE.BoxGeometry(4, 5, 18), gripperMaterial);
  palm.position.x = 6;
  gripper.add(palm);
  const fingers = [-1, 1].map((side) => {
    const finger = new THREE.Mesh(new THREE.BoxGeometry(16, 3.2, 3.2), fingerMaterial);
    finger.position.set(14, 0, side * 8);
    gripper.add(finger);
    return { finger, side };
  });
  scene.add(gripper);

  const trailPositions = new Float32Array(MAX_TRAIL * 3);
  const trailGeometry = new THREE.BufferGeometry();
  trailGeometry.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
  const trail = new THREE.Line(trailGeometry, new THREE.LineBasicMaterial({ color: 0xffb700, transparent: true, opacity: 0.8 }));
  trail.frustumCulled = false;
  scene.add(trail);

  function resize() {
    const { clientWidth: width, clientHeight: height } = canvas.parentElement;
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }
  window.addEventListener('resize', resize);
  resize();

  function render(snapshot) {
    const points = [snapshot.links[0].from, snapshot.links[0].to, snapshot.links[1].to, snapshot.links[2].to].map(robotToThree);
    points.forEach((point, index) => jointMeshes[index].position.copy(point));
    linkMeshes.forEach((mesh, index) => positionCylinder(mesh, points[index], points[index + 1]));
    gripper.matrix.copy(transformToThree(snapshot.transform));
    gripper.matrixAutoUpdate = false;
    const spread = snapshot.gripper_open ? 8 : 2;
    fingers.forEach(({ finger, side }) => { finger.position.z = side * spread; });
    const count = Math.min(snapshot.trajectory.length, MAX_TRAIL);
    snapshot.trajectory.slice(-count).forEach((point, index) => {
      const converted = robotToThree([point.px, point.py, point.pz]);
      trailPositions[index * 3] = converted.x;
      trailPositions[index * 3 + 1] = converted.y;
      trailPositions[index * 3 + 2] = converted.z;
    });
    trailGeometry.setDrawRange(0, count);
    trailGeometry.attributes.position.needsUpdate = true;
  }

  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }
  animate();
  return { render };
}
