// "People" wandering the scene as moving lock-on targets. Each one walks a
// circular loop around its home position; radius/speed/direction are
// randomized so they don't move in lockstep. Caller supplies an array of
// home positions, which lets main.js mix a tighter cluster near the model
// center with additional people scattered across the rest of the model.

import * as THREE from 'three';

export function createPeople({ scene, worldScale, getTerrainY, homePositions, maxWanderRadius }) {
  const personHeight = worldScale * 0.18;
  const personWidth = worldScale * 0.06;
  const personDepth = worldScale * 0.06;

  const colors = [0x4ade80, 0x60a5fa, 0xc084fc, 0xfb7185, 0x38bdf8, 0xfbbf24, 0xa78bfa, 0x34d399];

  const people = [];
  for (let i = 0; i < homePositions.length; i++) {
    const home = homePositions[i];
    const cx = home.x;
    const cz = home.z;

    const geom = new THREE.BoxGeometry(personWidth, personHeight, personDepth);
    const mat = new THREE.MeshStandardMaterial({ color: colors[i % colors.length] });
    const mesh = new THREE.Mesh(geom, mat);
    mesh.name = `person-${i}`;

    // Per-person patrol — circle of randomized size, walking speed, and
    // direction. Bigger wander radii mean people drift further from each
    // other, which is what the user asked for.
    const r = maxWanderRadius * (0.4 + Math.random() * 0.6);
    const speed = 0.7 + Math.random() * 0.5;
    const dir = Math.random() < 0.5 ? 1 : -1;
    const angularSpeed = (speed / r) * dir;
    const phase = Math.random() * Math.PI * 2;

    const initX = cx + Math.cos(phase) * r;
    const initZ = cz + Math.sin(phase) * r;
    const ty = getTerrainY(initX, initZ);
    mesh.position.set(initX, (ty ?? 0) + personHeight / 2, initZ);
    scene.add(mesh);

    people.push({
      mesh,
      pathCx: cx,
      pathCz: cz,
      pathRadius: r,
      angularSpeed,
      angle: phase,
      personHeight
    });
  }

  function update(dt) {
    for (const p of people) {
      p.angle += p.angularSpeed * dt;
      const x = p.pathCx + Math.cos(p.angle) * p.pathRadius;
      const z = p.pathCz + Math.sin(p.angle) * p.pathRadius;
      const ty = getTerrainY(x, z);
      // If the person's circular patrol takes them over a gap in the model
      // mesh (terrain raycast misses), skip the move this frame so they
      // don't end up floating or snapping to y=0. Their angle still advances
      // so they exit the dead zone on the next valid sample.
      if (ty === null) continue;
      p.mesh.position.set(x, ty + p.personHeight / 2, z);
      const sign = p.angularSpeed >= 0 ? 1 : -1;
      const dx = -Math.sin(p.angle) * sign;
      const dz = Math.cos(p.angle) * sign;
      p.mesh.rotation.y = Math.atan2(dx, dz);
    }
  }

  function getHeadAnchor(idx, out = new THREE.Vector3()) {
    const p = people[idx];
    out.set(
      p.mesh.position.x,
      p.mesh.position.y + personHeight / 2 + worldScale * 0.18,
      p.mesh.position.z
    );
    return out;
  }

  return {
    people,
    objects: people.map(p => p.mesh),
    update,
    getHeadAnchor
  };
}
