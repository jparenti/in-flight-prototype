// Builds a smooth Catmull-Rom curve through the waypoints with intermediate
// "obstacle avoidance" perturbations so the path weaves rather than going straight.
import * as THREE from 'three';

// Deterministic pseudo-random so the path is stable across reloads.
function mulberry32(seed) {
  let s = seed >>> 0;
  return () => {
    s |= 0; s = (s + 0x6D2B79F5) | 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/**
 * Build a weaving curve through an ordered list of anchor points.
 * Between each consecutive pair we add a perpendicular offset + small altitude wobble
 * to make the path look like organic obstacle avoidance.
 */
export function buildDroneCurve(anchors, opts = {}) {
  const {
    weaveAmplitude = 1,         // how far it weaves perpendicular to each segment
    altitudeJitter = 0.4,       // vertical wobble in same units as weaveAmplitude
    intermediatesPerSegment = 2,
    seed = 1234
  } = opts;

  const rand = mulberry32(seed);
  const points = [];

  for (let i = 0; i < anchors.length - 1; i++) {
    const a = anchors[i];
    const b = anchors[i + 1];
    points.push(a.clone());

    const segment = new THREE.Vector3().subVectors(b, a);
    const length = segment.length();
    const dir = segment.clone().normalize();
    // perpendicular in horizontal plane (we treat Y as up)
    const perp = new THREE.Vector3(-dir.z, 0, dir.x).normalize();

    for (let j = 1; j <= intermediatesPerSegment; j++) {
      const t = j / (intermediatesPerSegment + 1);
      const base = a.clone().lerp(b, t);
      // alternate weave side per intermediate, plus jitter
      const sign = (j % 2 === 0 ? -1 : 1) * (rand() < 0.5 ? -1 : 1);
      const lateral = (0.5 + rand() * 0.5) * weaveAmplitude * sign;
      const lift = (rand() - 0.3) * altitudeJitter;
      base.addScaledVector(perp, lateral);
      base.y += lift;
      // also pull intermediate towards a small forward bias to keep curve flowing
      base.addScaledVector(dir, (rand() - 0.5) * length * 0.05);
      points.push(base);
    }
  }
  points.push(anchors[anchors.length - 1].clone());

  // Centripetal Catmull-Rom is nicely behaved on uneven spacing, no overshoots.
  const curve = new THREE.CatmullRomCurve3(points, false, 'centripetal', 0.5);
  return curve;
}

/**
 * Builds a tube mesh for a sub-range of a curve [tStart, tEnd].
 * Returns null if the range is empty.
 */
export function buildTubeForRange(curve, tStart, tEnd, samples, radius, color, opts = {}) {
  if (tEnd - tStart < 1e-4) return null;
  const pts = [];
  const n = Math.max(2, samples);
  for (let i = 0; i <= n; i++) {
    const t = tStart + (tEnd - tStart) * (i / n);
    pts.push(curve.getPoint(t));
  }
  const subCurve = new THREE.CatmullRomCurve3(pts, false, 'centripetal', 0.5);
  const geo = new THREE.TubeGeometry(subCurve, n, radius, 8, false);
  const mat = new THREE.MeshBasicMaterial({
    color,
    transparent: opts.transparent ?? false,
    opacity: opts.opacity ?? 1,
    depthTest: opts.depthTest ?? true
  });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.renderOrder = opts.renderOrder ?? 1;
  return mesh;
}
