// Placeholder drone mesh. Easy to swap for a GLB later: replace the contents
// of createDrone() with a GLTFLoader call and return that group instead.
import * as THREE from 'three';

export function createDrone(worldScale) {
  const group = new THREE.Group();
  group.name = 'drone';

  const bodyMat = new THREE.MeshStandardMaterial({ color: 0x222831, metalness: 0.6, roughness: 0.35 });
  const armMat = new THREE.MeshStandardMaterial({ color: 0x111418, metalness: 0.4, roughness: 0.5 });
  const rotorMat = new THREE.MeshStandardMaterial({ color: 0xffd400, metalness: 0.2, roughness: 0.5, emissive: 0x332200, emissiveIntensity: 0.4 });

  const s = worldScale;
  // body
  const body = new THREE.Mesh(new THREE.BoxGeometry(s * 1.2, s * 0.5, s * 1.2), bodyMat);
  group.add(body);

  // arms (X shape)
  const armGeo = new THREE.BoxGeometry(s * 3.0, s * 0.18, s * 0.18);
  const armA = new THREE.Mesh(armGeo, armMat);
  armA.rotation.y = Math.PI / 4;
  const armB = new THREE.Mesh(armGeo, armMat);
  armB.rotation.y = -Math.PI / 4;
  group.add(armA, armB);

  // rotors at the four arm ends
  const rotorGeo = new THREE.CylinderGeometry(s * 0.6, s * 0.6, s * 0.06, 24);
  const armLen = s * 1.5;
  const positions = [
    [armLen * Math.SQRT1_2, 0, armLen * Math.SQRT1_2],
    [-armLen * Math.SQRT1_2, 0, armLen * Math.SQRT1_2],
    [armLen * Math.SQRT1_2, 0, -armLen * Math.SQRT1_2],
    [-armLen * Math.SQRT1_2, 0, -armLen * Math.SQRT1_2]
  ];
  const rotors = [];
  for (const [x, y, z] of positions) {
    const r = new THREE.Mesh(rotorGeo, rotorMat);
    r.position.set(x, y + s * 0.3, z);
    group.add(r);
    rotors.push(r);
  }
  group.userData.rotors = rotors;

  // status light
  const lightGeo = new THREE.SphereGeometry(s * 0.18, 16, 16);
  const lightMat = new THREE.MeshBasicMaterial({ color: 0xff3344 });
  const light = new THREE.Mesh(lightGeo, lightMat);
  light.position.set(0, -s * 0.35, s * 0.5);
  group.add(light);

  return group;
}
