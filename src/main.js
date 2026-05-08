import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader.js';
import GUI from 'lil-gui';

import { createWaypointSprite, createLandingPadSprite, createPhotoSprite } from './sprites.js';
import { createDrone } from './drone.js';
import { buildDroneCurve, buildTubeForRange } from './path.js';
import { attachGimbalControls } from './gimbalControls.js';
import { attachDragRotate } from './dragRotate.js';
import { attachDragMove } from './dragMove.js';
import { attachDoubleTap } from './doubleTap.js';

// Wire BVH into three.js so raycasts against meshes with .boundsTree are O(log n).
// Without this, getTerrainY walks every triangle of the 60MB model on every call.
import { computeBoundsTree, disposeBoundsTree, acceleratedRaycast } from 'three-mesh-bvh';
THREE.BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
THREE.BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;
THREE.Mesh.prototype.raycast = acceleratedRaycast;

// Reusable scratch — avoid allocating in the per-frame path.
const _camLookMat = new THREE.Matrix4();
const _gimbalQuat = new THREE.Quaternion();
const _gimbalEuler = new THREE.Euler();
const _yAxis = new THREE.Vector3(0, 1, 0);
const _aheadOffset = new THREE.Vector3();
const _droneFwdScratch = new THREE.Vector3();
const _terrainRaycaster = new THREE.Raycaster();
_terrainRaycaster.ray.direction.set(0, -1, 0);
_terrainRaycaster.firstHitOnly = true; // honored if BVH is enabled; harmless otherwise

const nowSec = () => performance.now() / 1000;
const clampNum = (v, lo, hi) => v < lo ? lo : v > hi ? hi : v;
const smoothstep01 = (x) => { const c = clampNum(x, 0, 1); return c * c * (3 - 2 * c); };

// ─── Renderer / Scene / Camera ────────────────────────────────────────────────
const app = document.getElementById('app');
const loadingEl = document.getElementById('loading');

const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.0;
app.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0d10);
scene.fog = null;

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 100000);
camera.position.set(200, 200, 200);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.screenSpacePanning = true;
controls.minDistance = 5;
controls.maxDistance = 5000;
controls.maxPolarAngle = Math.PI * 0.495; // don't allow flipping under the ground

// ─── Lights ───────────────────────────────────────────────────────────────────
const hemi = new THREE.HemisphereLight(0xddeeff, 0x223344, 0.7);
scene.add(hemi);
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(1, 2.5, 1);
scene.add(sun);

// ─── Load model ───────────────────────────────────────────────────────────────
const dracoLoader = new DRACOLoader();
dracoLoader.setDecoderPath(`${import.meta.env.BASE_URL}draco/`);
const gltfLoader = new GLTFLoader();
gltfLoader.setDRACOLoader(dracoLoader);

const state = {
  curve: null,
  totalLength: 0,
  drone: null,
  waypoints: [],          // [{ position, sprite, t }]
  waypointTs: [],         // t values on curve for each waypoint, in order
  landingPad: null,
  traveledMesh: null,
  projectedMesh: null,
  pathRoot: new THREE.Group(),
  photosRoot: new THREE.Group(),
  worldScale: 1,
  modelBox: new THREE.Box3(),
  photos: [],             // [{ t, label, sprite }]
  numPhotos: 4,
  photoSeed: 7,
  // animation
  t: 0,
  playing: true,
  duration: 30, // seconds for full route at speed=1
  speed: 0.1,
  loop: true,
  // camera
  cameraMode: 'firstPerson',  // 'firstPerson' | 'orbit'
  routeOverlayVisible: false, // path lines, waypoint sprites, photo sprites, landing pad, drone mesh
  // First-person smoothing — the curve tangent becomes the target; the camera eases toward it.
  cameraTargetPos: new THREE.Vector3(),
  cameraTargetQuat: new THREE.Quaternion(),
  cameraTargetReady: false,
  cameraSnap: true,           // snap on next tick (set on mode switch / first frame)
  cameraTurnRate: 4.0,        // damping rate; higher = snappier turns. Time to ~99% ≈ 4.6/rate seconds.
  cameraMoveRate: 8.0,        // position damping; higher = snappier translation

  // ── Gimbal (orientation-only camera mount on top of the drone's heading) ──
  gimbal: {
    yaw: 0,            // current applied yaw (deg, relative to drone forward)
    pitch: 0,          // current applied pitch (deg, relative to drone forward)
    zoom: 1.0,         // current applied zoom multiplier (camera fov = baseFov / zoom)
    yawSpeed: 60,      // deg/sec at full input
    pitchSpeed: 45,    // deg/sec at full input
    pitchMin: -85,     // looking nearly straight down
    pitchMax: 30,      // looking up
    yawMin: -120,
    yawMax: 120,
    zoomMin: 1.0,
    zoomMax: 5.0,
    baseFov: 75,
    zoomEaseRate: 8.0  // ~99% of fov target reached in ~0.575s
  },

  // ── Latency model ──
  // Discrete on-screen actions enqueue into actionQueue with executeAt = now + latency.
  // Continuous gimbal/movement input gets recorded with a timestamp; each frame we
  // read from the buffer at (now - latency) so input is replayed delayed.
  latency: 0.2,        // seconds, tunable via Settings panel
  actionQueue: [],     // [{ executeAt: number(s), action: () => void }]
  inputHistory: [],    // [{ t, yawRate, pitchRate, moveFwd, moveStrafe }]

  // ── Free-fly mode (engaged when paused via the on-screen pause button) ──
  // While paused, the drone leaves the curve and obeys the movement controls.
  // On play, it smoothly transitions back to the curve at the captured t.
  freeFly: {
    initialized: false,                  // becomes true after first pause
    pos: new THREE.Vector3(),            // current free-fly drone position
    yaw: 0                               // current free-fly drone yaw (radians, three.js convention; +Z forward, Y rotation)
  },
  transition: {                          // smooth return to route on play
    active: false,
    t: 0,
    duration: 0,                         // computed per-trigger below
    fromPos: new THREE.Vector3(),
    fromYaw: 0,
    returnSpeed: 18,                     // m/s — pace of the snap-back glide
    minDuration: 0.35,                   // s — short hop minimum
    maxDuration: 1.0                     // s — cap for far drifts
  },
  movement: {
    forward: 0, back: 0, left: 0, right: 0, up: 0, down: 0,  // 0 / 1, hold-to-move (raw input)
    speed: 8,                                 // m/s — max horizontal speed
    verticalSpeed: 4,                         // m/s — max altitude change rate (slower than horizontal)
    accelRate: 3.0,                           // ease rate for velocity → target. ~99% in 1.5s.
    yawAlignRate: 0.9,                        // rad/s — drone rotates toward velocity direction
    velocity: new THREE.Vector3(),            // current 3D world velocity (eased toward target each frame)
    minClearance: 4                           // never let the drone descend closer than this to the terrain
  },

  // Counts how many play/pause toggles are sitting in actionQueue right now.
  // ensurePaused() uses this to avoid stacking up duplicate pauses when the
  // user holds movement keys (each frame's keydown shouldn't enqueue another).
  playPauseInFlight: 0,

  // Double-click-to-focus animation: drone glides to a viewing position near
  // the selected point. Each frame the drone re-aims at the point (continuous
  // tracking) so the camera stays locked on through the glide.
  focusAnim: {
    active: false,
    t: 0,
    duration: 0,           // computed per-trigger from glide distance / cruiseSpeed
    fromPos: new THREE.Vector3(),
    toPos: new THREE.Vector3(),
    point: null,
    viewingDistance: 25,   // m horizontal distance from the point
    viewingHeight: 6,      // m above the point's altitude
    cruiseSpeed: 5,        // m/s — animation pace. Real inspection drones move at ~5–10 m/s.
    minDuration: 2.0,      // s — guarantees a visible animation even for short hops
    maxDuration: 6.0,      // s — caps very long flights so they don't drag
    turnRate: 1.5          // rad/s — gimbal turn rate while focus is active. Lower than
                           // the default cameraTurnRate so the camera eases into the new
                           // heading instead of snapping. Time to ~99% ≈ 3s.
  },
  focusMarker: null,  // 3D mesh in scene; null when no focus has been set

  // Lock-on / orbit mode. When active, movement controls take orbital semantics:
  // forward/back = closer/farther, left/right = orbit, up/down = altitude.
  // Each frame the drone+gimbal are forced to look at lockOn.point.
  lockOn: {
    active: false,
    point: null
  },

  // User-driven yaw during playback. Tracks the camera's *absolute world yaw*
  // (radians) so the camera holds its world heading even when the route
  // tangent changes. `userYawActive` is set on the first manual input (which
  // also snapshots the drone's current world yaw as the baseline). Cleared on
  // pause (current heading is absorbed into freeFly.yaw), on the transition
  // back to play, and on reset — all of which return the camera to following
  // the route tangent.
  userWorldYaw: 0,
  userYawActive: false
};
scene.add(state.pathRoot);
scene.add(state.photosRoot);
if (typeof window !== 'undefined') {
  window.__state = state;
  window.__camera = camera;
  window.__controls = controls;
  window.__scene = scene;
  window.__THREE = THREE;
  // For tests: expose updateDroneTransform after it's defined (deferred via setTimeout below)
  setTimeout(() => { window.__updateDroneTransform = updateDroneTransform; }, 0);
}

gltfLoader.load(
  `${import.meta.env.BASE_URL}models/property.glb`,
  (gltf) => {
    const model = gltf.scene;

    // The OBJ is Z-up; convert to three.js Y-up.
    model.rotation.x = -Math.PI / 2;
    model.updateMatrixWorld(true);

    // Compute bbox & recenter horizontally so origin is in the middle.
    const box = new THREE.Box3().setFromObject(model);
    const size = new THREE.Vector3(); box.getSize(size);
    const center = new THREE.Vector3(); box.getCenter(center);
    model.position.x -= center.x;
    model.position.z -= center.z;
    model.position.y -= box.min.y; // sit ground at y=0
    model.updateMatrixWorld(true);

    // Materials in obj2gltf default unlit -- ensure visibility regardless
    model.traverse((obj) => {
      if (obj.isMesh) {
        if (obj.material && obj.material.map) {
          obj.material.map.colorSpace = THREE.SRGBColorSpace;
          obj.material.map.anisotropy = 8;
          obj.material.needsUpdate = true;
        }
      }
    });

    scene.add(model);
    state.model = model; // referenced by terrain raycasts

    // Build a BVH for every mesh in the model so raycasts are fast. One-time cost
    // (~100ms here for the property mesh); makes per-frame getTerrainY cheap.
    const bvhStart = performance.now();
    let bvhMeshes = 0;
    model.traverse((obj) => {
      if (obj.isMesh && obj.geometry && !obj.geometry.boundsTree) {
        obj.geometry.computeBoundsTree();
        bvhMeshes++;
      }
    });
    console.log(`BVH built for ${bvhMeshes} meshes in ${(performance.now() - bvhStart).toFixed(0)}ms`);

    // Recompute bbox after transforms
    state.modelBox = new THREE.Box3().setFromObject(model);
    const dim = new THREE.Vector3(); state.modelBox.getSize(dim);
    const diag = dim.length();
    state.worldScale = diag * 0.012; // sprite/drone size relative to scene
    console.log('Model bbox size (m):', dim, 'diag:', diag.toFixed(1));

    setupRoute();
    setupGUI();
    setupOverlay();
    // setupRoute → applyCameraMode positions the camera correctly for the
    // current mode (first-person rides the drone; orbit fits the model).

    loadingEl.classList.add('hidden');
    setTimeout(() => loadingEl.remove(), 500);
  },
  (xhr) => {
    if (xhr.lengthComputable) {
      const pct = (xhr.loaded / xhr.total) * 100;
      loadingEl.textContent = `Loading model… ${pct.toFixed(0)}%`;
    }
  },
  (err) => {
    console.error('GLB load failed', err);
    loadingEl.textContent = 'Failed to load model. See console.';
  }
);

// Returns the model surface Y at world (x, z) by raycasting straight down from
// above the bbox. Returns null if the ray misses (e.g., gap in the mesh).
function getTerrainY(x, z) {
  if (!state.model) return null;
  _terrainRaycaster.ray.origin.set(x, state.modelBox.max.y + 100, z);
  const hits = _terrainRaycaster.intersectObject(state.model, true);
  return hits.length ? hits[0].point.y : null;
}

// Resamples a horizontal-weave curve at `samples` points, raycasts each to set
// y = terrain + clearance, then applies a sliding-window max so the drone climbs
// *before* it reaches a tall obstacle (and descends *after* it has cleared it)
// rather than slamming into the side. Returns a smooth Catmull-Rom curve.
function buildTerrainFollowingCurve(draftCurve, clearance, fallbackY, samples = 240, lookaheadWindow = 8) {
  const pts = [];
  for (let i = 0; i <= samples; i++) {
    const t = i / samples;
    const p = draftCurve.getPoint(t);
    const ty = getTerrainY(p.x, p.z);
    p.y = (ty ?? fallbackY) + clearance;
    pts.push(p);
  }
  // Sliding-window max smoothing — guarantees obstacle clearance before/after.
  // Window of N samples = N * (totalLength/samples) meters lookahead each side.
  const smoothed = pts.map((p, i) => {
    let maxY = p.y;
    const lo = Math.max(0, i - lookaheadWindow);
    const hi = Math.min(pts.length - 1, i + lookaheadWindow);
    for (let j = lo; j <= hi; j++) if (pts[j].y > maxY) maxY = pts[j].y;
    return new THREE.Vector3(p.x, maxY, p.z);
  });
  return new THREE.CatmullRomCurve3(smoothed, false, 'centripetal', 0.5);
}

// ─── Route setup (waypoints, landing pad, path, drone) ────────────────────────
function setupRoute() {
  const box = state.modelBox;
  const size = new THREE.Vector3(); box.getSize(size);
  const center = new THREE.Vector3(); box.getCenter(center);
  // Clearance above the local terrain (raycast per-sample below). Small absolute
  // buffer so the drone hugs the property without clipping. With terrain
  // following we can safely fly much closer than a flat ceiling above the peak.
  const flightClearance = 4;

  // Route radius — fraction of the property used. Lower = tighter route, less
  // chance the first-person camera looks off the map at the perimeter.
  const r = 0.5;

  // Landing pad near a corner of the property (within the inner r-fraction).
  // Y is set per-anchor below by raycasting the model.
  const landingPos = new THREE.Vector3(center.x - size.x * 0.35 * r, 0, center.z + size.z * 0.4 * r);
  const waypointPositions = [
    new THREE.Vector3(center.x + size.x * 0.30 * r, 0, center.z + size.z * 0.25 * r),
    new THREE.Vector3(center.x + size.x * 0.40 * r, 0, center.z - size.z * 0.20 * r),
    new THREE.Vector3(center.x - size.x * 0.05 * r, 0, center.z - size.z * 0.40 * r),
    new THREE.Vector3(center.x - size.x * 0.40 * r, 0, center.z - size.z * 0.10 * r),
    new THREE.Vector3(center.x - size.x * 0.20 * r, 0, center.z + size.z * 0.15 * r)
  ];

  // Snap each anchor to the local terrain altitude + clearance.
  const padFallback = box.max.y + flightClearance;
  landingPos.y = (getTerrainY(landingPos.x, landingPos.z) ?? padFallback) + flightClearance;
  waypointPositions.forEach((p) => {
    p.y = (getTerrainY(p.x, p.z) ?? padFallback) + flightClearance;
  });

  // Landing pad sprite
  state.landingPad = createLandingPadSprite(state.worldScale * 4);
  state.landingPad.position.copy(landingPos);
  scene.add(state.landingPad);

  // Waypoint sprites
  waypointPositions.forEach((p, i) => {
    const s = createWaypointSprite(i, state.worldScale * 3.5);
    s.position.copy(p);
    scene.add(s);
    state.waypoints.push({ position: p, sprite: s });
  });

  // Build a draft curve to define the horizontal weave through the waypoints.
  // We then re-sample it at high density and raycast each sample to get an
  // altitude that hugs the terrain.
  const anchors = [landingPos, ...waypointPositions, landingPos.clone()];
  const weaveAmp = Math.min(size.x, size.z) * 0.05;
  const draftCurve = buildDroneCurve(anchors, {
    weaveAmplitude: weaveAmp,
    altitudeJitter: 0, // altitude is set by terrain raycast below, not random jitter
    intermediatesPerSegment: 2,
    seed: 42
  });

  state.curve = buildTerrainFollowingCurve(draftCurve, flightClearance, padFallback);
  state.totalLength = state.curve.getLength();

  // Find each waypoint's t on the curve so we can label between-waypoint photos
  // by their preceding waypoint number.
  state.waypointTs = computeWaypointTs(state.curve, waypointPositions);
  state.waypoints.forEach((wp, i) => { wp.t = state.waypointTs[i]; });

  // Drone
  state.drone = createDrone(state.worldScale * 1.6);
  scene.add(state.drone);

  generatePhotos();
  rebuildPathMeshes();
  applyRouteOverlayVisibility();
  applyCameraMode();
  updateDroneTransform();
}

// Toggles the path tubes, waypoint sprites, photo sprites, landing pad, and drone mesh.
// Items remain in the scene graph; only `.visible` changes.
function applyRouteOverlayVisibility() {
  const v = state.routeOverlayVisible;
  state.pathRoot.visible = v;
  state.photosRoot.visible = v;
  if (state.landingPad) state.landingPad.visible = v;
  for (const wp of state.waypoints) wp.sprite.visible = v;
  // Drone mesh is part of the overlay too — in first-person you'd be inside it,
  // and in orbit with the overlay off there's nothing meaningful to look at.
  if (state.drone) state.drone.visible = v && state.cameraMode !== 'firstPerson';
}

function applyCameraMode() {
  if (state.cameraMode === 'firstPerson') {
    controls.enabled = false;
    // a near plane that tolerates being co-located with the drone
    camera.near = 0.1;
    camera.fov = 75;
    camera.updateProjectionMatrix();
    state.cameraSnap = true; // entering FP — snap to drone, don't glide from orbit pose
  } else {
    controls.enabled = true;
    camera.near = 0.1;
    camera.fov = 50;
    camera.updateProjectionMatrix();
    fitCameraToBox(state.modelBox);
  }
  // Drone mesh visibility depends on camera mode
  applyRouteOverlayVisibility();
}

// Sample the curve densely and find the closest t for each waypoint position.
function computeWaypointTs(curve, waypoints) {
  const N = 2000;
  const samples = curve.getSpacedPoints(N);
  return waypoints.map((wp) => {
    let bestI = 0, bestD = Infinity;
    for (let i = 0; i < samples.length; i++) {
      const d = samples[i].distanceToSquared(wp);
      if (d < bestD) { bestD = d; bestI = i; }
    }
    return bestI / N;
  });
}

// Returns [start, end] t-bounds for the segment a given t falls in.
// Segments: 0=start→wp1, 1=wp1→wp2, ..., N=wpN→landing.
function segmentIndexForT(t) {
  // Boundaries: 0, wp1.t, wp2.t, ..., wp5.t, 1
  const bounds = [0, ...state.waypointTs, 1];
  for (let i = 0; i < bounds.length - 1; i++) {
    if (t >= bounds[i] && t <= bounds[i + 1]) return i;
  }
  return bounds.length - 2;
}

// Build a few fake photo events. Photos are placed at random t values that aren't
// too close to a waypoint, then labeled "<prev-waypoint>a/b/c..." in order.
function generatePhotos() {
  // Clear previous photos
  for (const p of state.photos) {
    state.photosRoot.remove(p.sprite);
    p.sprite.material.map.dispose();
    p.sprite.material.dispose();
  }
  state.photos = [];

  const minSpacingFromWaypoint = 0.025; // in t-units
  const rand = mulberry32(state.photoSeed);

  // Pick t values inside random non-edge segments (skip the first and last
  // landing→wp1 / wp5→landing segments to keep example focused, but it's
  // valid to include them — toggle the slice() if you want them).
  const candidateSegments = [];
  for (let i = 0; i < state.waypointTs.length - 1; i++) {
    candidateSegments.push({ a: state.waypointTs[i], b: state.waypointTs[i + 1], idx: i + 1 });
  }

  const ts = [];
  for (let i = 0; i < state.numPhotos; i++) {
    const seg = candidateSegments[Math.floor(rand() * candidateSegments.length)];
    const inset = minSpacingFromWaypoint;
    const t = seg.a + inset + rand() * Math.max(0.001, (seg.b - seg.a) - inset * 2);
    ts.push(t);
  }
  ts.sort((x, y) => x - y);

  // Label each photo as "<prevWaypoint><letter>" where prevWaypoint is the
  // most recent waypoint number visited (0 = "before wp1", aka start landing).
  // Letter suffix counts up within each segment.
  const letterCounts = new Map();
  for (const t of ts) {
    let prevWp = 0;
    for (let i = 0; i < state.waypointTs.length; i++) {
      if (t >= state.waypointTs[i]) prevWp = i + 1; else break;
    }
    const count = (letterCounts.get(prevWp) ?? 0);
    const label = `${prevWp}${String.fromCharCode(97 + count)}`; // 1a, 1b, ...
    letterCounts.set(prevWp, count + 1);

    const sprite = createPhotoSprite(label, state.worldScale * 4.2);
    sprite.position.copy(state.curve.getPoint(t));
    state.photosRoot.add(sprite);
    state.photos.push({ t, label, sprite });
  }
}

function mulberry32(seed) {
  let s = seed >>> 0;
  return () => {
    s |= 0; s = (s + 0x6D2B79F5) | 0;
    let x = Math.imul(s ^ (s >>> 15), 1 | s);
    x = (x + Math.imul(x ^ (x >>> 7), 61 | x)) ^ x;
    return ((x ^ (x >>> 14)) >>> 0) / 4294967296;
  };
}

function rebuildPathMeshes() {
  const radius = state.worldScale * 0.5;
  const samples = 220;

  if (state.traveledMesh) { state.pathRoot.remove(state.traveledMesh); state.traveledMesh.geometry.dispose(); }
  if (state.projectedMesh) { state.pathRoot.remove(state.projectedMesh); state.projectedMesh.geometry.dispose(); }

  state.traveledMesh = buildTubeForRange(state.curve, 0, state.t, Math.max(2, Math.floor(samples * state.t)), radius, 0x37e29a);
  state.projectedMesh = buildTubeForRange(state.curve, state.t, 1, Math.max(2, Math.floor(samples * (1 - state.t))), radius * 0.85, 0x6a7280, { transparent: true, opacity: 0.85 });

  if (state.traveledMesh) state.pathRoot.add(state.traveledMesh);
  if (state.projectedMesh) state.pathRoot.add(state.projectedMesh);
}

function updateDroneTransform() {
  if (!state.curve || !state.drone) return;
  const t = THREE.MathUtils.clamp(state.t, 0, 1);
  const routePos = state.curve.getPoint(t);
  const lookT = t < 0.999 ? Math.min(1, t + 0.005) : t;
  const routeAhead = state.curve.getPoint(lookT);

  // Decide where the drone should actually be drawn this frame.
  let pos, ahead;
  if (state.transition.active) {
    // Smooth lerp from free-fly pose back to current route pose.
    const u = smoothstep01(state.transition.t);
    pos = state.transition.fromPos.clone().lerp(routePos, u);
    const fwd = routeAhead.clone().sub(routePos);
    if (fwd.lengthSq() < 1e-8) fwd.set(0, 0, 1);
    fwd.normalize();
    const routeYaw = Math.atan2(fwd.x, fwd.z); // matches state.freeFly.yaw convention (+Z forward)
    let dy = routeYaw - state.transition.fromYaw;
    while (dy > Math.PI) dy -= 2 * Math.PI;
    while (dy < -Math.PI) dy += 2 * Math.PI;
    const yaw = state.transition.fromYaw + dy * u;
    ahead = new THREE.Vector3(pos.x + Math.sin(yaw), pos.y, pos.z + Math.cos(yaw));
  } else if (state.freeFly.initialized && !state.playing) {
    pos = state.freeFly.pos;
    ahead = new THREE.Vector3(
      pos.x + Math.sin(state.freeFly.yaw),
      pos.y,
      pos.z + Math.cos(state.freeFly.yaw)
    );
  } else {
    pos = routePos;
    if (state.userYawActive) {
      // Camera holds an absolute world yaw — drone rotates to match. The
      // route's vertical tangent is dropped (camera goes horizontal); pitch
      // gives the user separate vertical control. Drone mesh is hidden in
      // first-person, so its orientation here is visually moot.
      ahead = new THREE.Vector3(
        pos.x + Math.sin(state.userWorldYaw),
        pos.y,
        pos.z + Math.cos(state.userWorldYaw)
      );
    } else {
      ahead = routeAhead;
    }
  }

  state.drone.position.copy(pos);
  if (ahead.distanceToSquared(pos) > 1e-6) {
    state.drone.lookAt(ahead);
  }

  // First-person camera target. The tick loop eases the actual camera toward
  // these targets so heading changes are smooth.
  //
  // Note: we can't just copy the drone's quaternion here. Object3D.lookAt()
  // orients with local +Z toward the target, but cameras use -Z forward —
  // copying the drone's quaternion would aim the camera 180° backwards.
  // We rebuild a camera-convention orientation via Matrix4.lookAt(eye, target, up).
  if (state.cameraMode === 'firstPerson') {
    state.cameraTargetPos.copy(pos);
    if (ahead.distanceToSquared(pos) > 1e-6) {
      _camLookMat.lookAt(pos, ahead, camera.up);
      state.cameraTargetQuat.setFromRotationMatrix(_camLookMat);
      // The real drone's gimbal only tilts (pitch). Yaw is always equal to the
      // drone's heading — the drone rotates instead of the gimbal. So we just
      // apply pitch on top of the drone-aligned base orientation; yaw stays 0.
      _gimbalEuler.set(
        THREE.MathUtils.degToRad(state.gimbal.pitch),
        0,
        0,
        'YXZ'
      );
      _gimbalQuat.setFromEuler(_gimbalEuler);
      state.cameraTargetQuat.multiply(_gimbalQuat);
    }
    state.cameraTargetReady = true;
  }

}

// ─── Camera framing ───────────────────────────────────────────────────────────
function fitCameraToBox(box) {
  const size = new THREE.Vector3(); box.getSize(size);
  const center = new THREE.Vector3(); box.getCenter(center);
  const maxDim = Math.max(size.x, size.z);
  const dist = maxDim * 1.4;

  camera.position.set(center.x + dist * 0.7, center.y + size.y * 1.5 + dist * 0.5, center.z + dist * 0.7);
  controls.target.set(center.x, center.y + size.y * 0.4, center.z);
  controls.maxDistance = maxDim * 4;
  controls.update();
}

// ─── GUI ──────────────────────────────────────────────────────────────────────
function setupGUI() {
  const gui = new GUI({ title: 'Settings', width: 300 });

  const sim = gui.addFolder('Simulation');
  sim.add(state, 'latency', 0, 2, 0.05).name('Control latency (s)');

  const cam = gui.addFolder('Camera');
  cam.add(state, 'cameraMode', { 'First-person (drone)': 'firstPerson', 'Orbit': 'orbit' })
     .name('Mode')
     .onChange(() => { applyCameraMode(); updateDroneTransform(); });
  cam.add(state, 'routeOverlayVisible').name('Show route overlay').onChange(applyRouteOverlayVisibility);
  cam.add(state, 'cameraTurnRate', 0.5, 20, 0.1).name('Heading ease (FP)');
  cam.add(state, 'cameraMoveRate', 0.5, 30, 0.1).name('Position ease (FP)');

  const playback = gui.addFolder('Playback');

  const ctl = {
    playPause: () => playPauseImmediate(),
    reset: () => { state.t = 0; rebuildPathMeshes(); updateDroneTransform(); },
    end: () => { state.t = 1; rebuildPathMeshes(); updateDroneTransform(); },
  };

  const tCtl = playback.add(state, 't', 0, 1, 0.0001).name('Progress (scrub)').onChange(() => {
    rebuildPathMeshes();
    updateDroneTransform();
  });
  playback.add(ctl, 'playPause').name('▶ / ⏸  Play/Pause');
  playback.add(state, 'duration', 5, 120, 1).name('Total duration (s)');
  playback.add(state, 'speed', 0.1, 5, 0.05).name('Speed multiplier');
  playback.add(state, 'loop').name('Loop at end');
  playback.add(ctl, 'reset').name('⏮ Reset to start');
  playback.add(ctl, 'end').name('⏭ Jump to end');

  const photos = gui.addFolder('Photos');
  photos.add(state, 'numPhotos', 0, 10, 1).name('Ad-hoc photo count').onChange(() => {
    generatePhotos();
    updateDroneTransform();
  });
  photos.add({ randomize: () => {
    state.photoSeed = Math.floor(Math.random() * 1e9);
    generatePhotos();
    updateDroneTransform();
  } }, 'randomize').name('🎲 Randomize positions');

  // Spacebar play/pause — routes through latency to mirror the on-screen button.
  window.addEventListener('keydown', (e) => {
    if (e.code === 'Space' && (e.target === document.body || e.target.tagName === 'CANVAS')) {
      e.preventDefault();
      enqueuePlayPauseToggle(document.getElementById('pause-btn'));
    }
  });

  // Expose live update of t slider when playing
  state._tController = tCtl;
}

// ─── Latency model ────────────────────────────────────────────────────────────
//
// Two parallel mechanisms:
//
//  1. Discrete actions (button taps) are queued: when the user clicks a control,
//     we capture the action and the time it should take effect (now + latency).
//     The tick loop fires actions whose time has come.
//
//  2. Continuous gimbal input (joystick/arrows) gets timestamped and pushed into
//     a ring buffer. Each tick we look up the input rate from (now - latency),
//     so the gimbal angle responds to what the user did `latency` seconds ago.
//
// Latency=0 makes both pass through immediately (queue executes on the next tick;
// buffer lookup returns the most recent entry).

function enqueueAction(action, btnEl) {
  state.actionQueue.push({ executeAt: nowSec() + state.latency, action });
  if (btnEl) {
    btnEl.classList.add('queued');
    setTimeout(() => btnEl.classList.remove('queued'), Math.max(120, state.latency * 1000));
  }
}

function processActionQueue() {
  const now = nowSec();
  while (state.actionQueue.length && state.actionQueue[0].executeAt <= now) {
    state.actionQueue.shift().action();
  }
}

function pushInputSample(yawRate, pitchRate, moveFwd, moveStrafe, moveUp) {
  const t = nowSec();
  state.inputHistory.push({ t, yawRate, pitchRate, moveFwd, moveStrafe, moveUp });
  const cutoff = t - 5;
  while (state.inputHistory.length > 1 && state.inputHistory[0].t < cutoff) {
    state.inputHistory.shift();
  }
}

function readDelayedInput() {
  if (!state.inputHistory.length) return { yawRate: 0, pitchRate: 0, moveFwd: 0, moveStrafe: 0, moveUp: 0 };
  const target = nowSec() - state.latency;
  let chosen = state.inputHistory[0];
  for (const s of state.inputHistory) {
    if (s.t <= target) chosen = s; else break;
  }
  return { yawRate: chosen.yawRate, pitchRate: chosen.pitchRate, moveFwd: chosen.moveFwd, moveStrafe: chosen.moveStrafe, moveUp: chosen.moveUp };
}

// Common entry point for any movement control press (button or key). Sets the
// raw input state, auto-pauses if currently playing, and cancels an in-progress
// focus animation so the user immediately takes control.
function onMovementPressed(dir) {
  state.movement[dir] = 1;
  ensurePaused();
  ensureFreeFlyInitialized();
  cancelFocusAnim();
}

// Safety net: free-fly defaults to uninitialized until the first pause captures
// the drone's pose. If something tries to enter free-fly without going through
// pause (e.g., the very first movement press), we capture the pose now.
function ensureFreeFlyInitialized() {
  if (state.freeFly.initialized || !state.drone) return;
  state.freeFly.pos.copy(state.drone.position);
  const dfwd = new THREE.Vector3(0, 0, 1).applyQuaternion(state.drone.quaternion);
  state.freeFly.yaw = Math.atan2(dfwd.x, dfwd.z);
  state.freeFly.initialized = true;
}

function cancelFocusAnim() {
  state.focusAnim.active = false;
}

// Build the focus marker once, lazily. Returns the existing one on subsequent calls.
function ensureFocusMarker() {
  if (state.focusMarker) return state.focusMarker;
  const s = state.worldScale; // world-relative size (model diag ~868m → s ≈ 10.4)
  const group = new THREE.Group();
  group.name = 'focusMarker';

  // Yellow ring lying flat on the ground.
  const ringGeo = new THREE.TorusGeometry(s * 0.15, s * 0.025, 10, 32);
  const mat = new THREE.MeshBasicMaterial({ color: 0xffd400, transparent: true, opacity: 0.95 });
  const ring = new THREE.Mesh(ringGeo, mat);
  ring.rotation.x = Math.PI / 2;
  group.add(ring);

  // Slim vertical pole so the marker is visible even when the ring is edge-on.
  const poleGeo = new THREE.CylinderGeometry(s * 0.015, s * 0.015, s * 0.30, 8);
  const pole = new THREE.Mesh(poleGeo, mat);
  pole.position.y = s * 0.15;
  group.add(pole);

  // Small sphere at the top of the pole.
  const tipGeo = new THREE.SphereGeometry(s * 0.045, 12, 12);
  const tip = new THREE.Mesh(tipGeo, mat);
  tip.position.y = s * 0.32;
  group.add(tip);

  group.visible = false;
  scene.add(group);
  state.focusMarker = group;
  return group;
}

function showFocusMarkerAt(point) {
  const m = ensureFocusMarker();
  m.position.copy(point);
  m.visible = true;
  document.body.classList.add('has-focus-marker');
}

function hideFocusMarker() {
  if (state.focusMarker) state.focusMarker.visible = false;
  document.body.classList.remove('has-focus-marker');
}

function setLockOn(active) {
  state.lockOn.active = !!active;
  if (active && state.focusMarker) {
    state.lockOn.point = state.focusMarker.position.clone();
  }
  document.body.classList.toggle('locked-on', state.lockOn.active);
  syncOverlayLockButton();
}

function syncOverlayLockButton() {
  const btn = document.getElementById('lock-btn');
  if (!btn) return;
  btn.textContent = state.lockOn.active ? 'Unlock' : 'Lock on target';
}

// Called via the latency queue in response to a double-click on the canvas.
// Auto-pauses (synchronously, since we're already inside a delayed action),
// then sets up an animated camera centering on `point`.
function focusOnPoint(point) {
  if (state.playing) playPauseImmediate(); // pause + capture freeFly
  ensureFreeFlyInitialized();              // safety net for paused-but-never-free-flown

  const fa = state.focusAnim;

  // Approach direction: from the focus point toward the drone's current horizontal
  // position. The drone glides in along that line and stops at viewingDistance.
  const dx = state.freeFly.pos.x - point.x;
  const dz = state.freeFly.pos.z - point.z;
  const currentHoriz = Math.hypot(dx, dz);
  let approachX, approachZ;
  if (currentHoriz > 0.5) {
    approachX = dx / currentHoriz;
    approachZ = dz / currentHoriz;
  } else {
    // Drone is right above the point — back off in the drone's current heading
    // (sin/cos of state.freeFly.yaw gives current +Z forward).
    approachX = Math.sin(state.freeFly.yaw);
    approachZ = Math.cos(state.freeFly.yaw);
  }

  fa.toPos.set(
    point.x + approachX * fa.viewingDistance,
    0,
    point.z + approachZ * fa.viewingDistance
  );
  // Altitude: above the point's altitude, but never closer to the terrain than minClearance.
  const ty = getTerrainY(fa.toPos.x, fa.toPos.z);
  fa.toPos.y = Math.max(point.y + fa.viewingHeight, (ty ?? point.y) + state.movement.minClearance);

  fa.fromPos.copy(state.freeFly.pos);
  fa.point = point.clone();
  fa.t = 0;
  // If we're already locked on a target, carry the lock to the new point so
  // orbital movement applies to the new focus once the glide finishes.
  if (state.lockOn.active) state.lockOn.point = point.clone();
  // Scale duration by glide distance so velocity is roughly constant across
  // short and long focus moves (capped at min/max so it always feels deliberate).
  const glideDist = fa.fromPos.distanceTo(fa.toPos);
  fa.duration = clampNum(glideDist / fa.cruiseSpeed, fa.minDuration, fa.maxDuration);
  fa.active = true;

  // Reset gimbal yaw/pitch toward zero so the drone-relative aim is purely
  // along the drone's forward axis. The tick fills in the values each frame.
  showFocusMarkerAt(point);
}

// ─── Action handlers (called either immediately from Settings panel, or via
//     enqueueAction from the on-screen overlay) ─────────────────────────────────
// Enqueues a play/pause toggle through the latency queue, but only if there
// isn't already a pending toggle that would land us where we want. The
// playPauseInFlight counter tracks pending toggles so we don't stack duplicates.
function ensurePaused() {
  // "effective playing" = state.playing flipped once per pending toggle
  const effectivePlaying = (state.playPauseInFlight % 2 === 0) ? state.playing : !state.playing;
  if (!effectivePlaying) return;
  state.playPauseInFlight++;
  enqueueAction(() => {
    state.playPauseInFlight--;
    playPauseImmediate();
  });
}

// Wraps the original playPauseImmediate from the overlay/Settings buttons so
// they also keep the in-flight counter accurate.
function enqueuePlayPauseToggle(btnEl) {
  state.playPauseInFlight++;
  enqueueAction(() => {
    state.playPauseInFlight--;
    playPauseImmediate();
  }, btnEl);
}

function playPauseImmediate() {
  if (state.transition.active) return; // ignore during return-to-route
  if (state.playing) {
    // → Pause + enter free-fly. Capture the drone's current world pose.
    state.playing = false;
    state.freeFly.pos.copy(state.drone.position);
    // Drone uses Object3D.lookAt (+Z forward), so its forward in world is +Z * quat.
    // Yaw is the angle around Y where forward = (sin θ, 0, cos θ). θ = atan2(fwd.x, fwd.z).
    // The drone's quaternion already reflects userWorldYaw (applied in
    // updateDroneTransform's route path), so freeFly.yaw absorbs it here.
    const dfwd = new THREE.Vector3(0, 0, 1).applyQuaternion(state.drone.quaternion);
    state.freeFly.yaw = Math.atan2(dfwd.x, dfwd.z);
    state.userYawActive = false;
    state.freeFly.initialized = true;
  } else if (state.freeFly.initialized) {
    // → Play (smooth return to route at current t). Lock-on is exited and the
    // focus marker is hidden so the route view is unobstructed.
    if (state.lockOn.active) setLockOn(false);
    hideFocusMarker();
    // Duration scales with how far the drone has drifted off the curve, so a
    // small free-fly excursion resumes near-instantly and a far drift gets a
    // short glide.
    const tr = state.transition;
    const t = THREE.MathUtils.clamp(state.t, 0, 1);
    const routePos = state.curve.getPoint(t);
    const dist = state.freeFly.pos.distanceTo(routePos);
    tr.active = true;
    tr.t = 0;
    tr.fromPos.copy(state.freeFly.pos);
    tr.fromYaw = state.freeFly.yaw;
    tr.duration = clampNum(dist / tr.returnSpeed, tr.minDuration, tr.maxDuration);
    // state.playing flips to true in the tick when the transition completes.
  } else {
    // First-ever play (no free-fly state captured yet — was paused at start)
    state.playing = true;
  }
  syncOverlayPauseIcon();
  syncOverlayMovementVisible();
}

function syncOverlayMovementVisible() {
  const showMovement = !state.playing && state.freeFly.initialized && !state.transition.active;
  document.body.classList.toggle('freefly', showMovement);
}

function resetGimbalImmediate() {
  state.gimbal.pitch = 0;
  state.gimbal.zoom = 1.0;
  // Heading: during playback (or before free-fly is initialized) the route
  // tangent is the "default" the camera should snap back to — clear the
  // user's manual yaw flag. In free-fly there is no default heading, so
  // leave freeFly.yaw alone and only reset the vertical look.
  if (state.playing || state.transition.active || !state.freeFly.initialized) {
    state.userYawActive = false;
  }
  updateZoomReadout();
  updateDroneTransform();
}

// Click steps: snap to next/prev integer tier (1×→2×→3×→4×→5×→1×). If zoom
// is currently fractional (from scroll), the first click completes the partial
// step rather than skipping it.
function applyZoomTier(direction) {
  const g = state.gimbal;
  let next;
  if (direction > 0) {
    next = Math.floor(g.zoom + 1);
    if (next > g.zoomMax) next = g.zoomMin;
  } else {
    next = Math.ceil(g.zoom - 1);
    if (next < g.zoomMin) next = g.zoomMax;
  }
  g.zoom = next;
  updateZoomReadout();
}

// Scroll: each event nudges zoom by deltaY * sensitivity. Camera fov eases
// toward baseFov/zoom in the tick, so this stays smooth even at high event
// rates. Convention: scroll up (deltaY < 0) = zoom in.
function applyZoomScroll(deltaY) {
  const g = state.gimbal;
  const sens = 0.005;
  g.zoom = clampNum(g.zoom - deltaY * sens, g.zoomMin, g.zoomMax);
  updateZoomReadout();
}

// Brief .held flash on the corresponding zoom button so the user sees feedback
// for scroll input (which has no native button-press visual).
let _zoomFlashBtn = null;
let _zoomFlashTimer = null;
function flashZoomButton(direction) {
  const id = direction > 0 ? 'zoom-in' : 'zoom-out';
  const btn = document.getElementById(id);
  if (!btn) return;
  if (_zoomFlashTimer) clearTimeout(_zoomFlashTimer);
  if (_zoomFlashBtn && _zoomFlashBtn !== btn) _zoomFlashBtn.classList.remove('held');
  btn.classList.add('held');
  _zoomFlashBtn = btn;
  _zoomFlashTimer = setTimeout(() => {
    btn.classList.remove('held');
    _zoomFlashBtn = null;
    _zoomFlashTimer = null;
  }, 120);
}

function syncOverlayPauseIcon() {
  const btn = document.getElementById('pause-btn');
  if (!btn) return;
  // During the return-to-route transition the user has *requested* play —
  // show the playing icon immediately so the press feels responsive, even
  // though state.playing only flips after the transition completes.
  const userWantsPlay = state.playing || state.transition.active;
  btn.textContent = userWantsPlay ? '⏸' : '▶';
}

function updateZoomReadout() {
  const el = document.getElementById('zoom-readout');
  if (el) el.textContent = `${state.gimbal.zoom.toFixed(1)}×`;
}

// ─── Overlay wiring ───────────────────────────────────────────────────────────
function setupOverlay() {
  const pauseBtn = document.getElementById('pause-btn');
  if (pauseBtn) {
    pauseBtn.addEventListener('click', () => enqueuePlayPauseToggle(pauseBtn));
  }
  const resetBtn = document.getElementById('gimbal-reset');
  if (resetBtn) {
    resetBtn.addEventListener('click', () => enqueueAction(resetGimbalImmediate, resetBtn));
  }
  const lockBtn = document.getElementById('lock-btn');
  if (lockBtn) {
    // Lock toggle goes through the latency queue like other on-screen controls.
    lockBtn.addEventListener('click', () => enqueueAction(() => setLockOn(!state.lockOn.active), lockBtn));
  }

  // Gimbal continuous input — joystick + keyboard + zoom buttons.
  // Zoom button taps are routed through the latency-applied action queue too.
  state.gimbalInput = attachGimbalControls({
    onZoomStep: (dir) => enqueueAction(() => applyZoomTier(dir))
  });

  // Scroll-to-zoom (desktop only). Skipped in orbit mode so OrbitControls'
  // own wheel handler stays in charge of dollying. deltaMode normalizes the
  // Firefox "lines" mode back to approximate pixels.
  renderer.domElement.addEventListener('wheel', (e) => {
    if (isTouchDevice()) return;
    if (state.cameraMode !== 'firstPerson') return;
    if (e.deltaY === 0) return;
    e.preventDefault();
    let dy = e.deltaY;
    if (e.deltaMode === 1) dy *= 32;     // lines → ~pixels
    else if (e.deltaMode === 2) dy *= 100; // pages → ~pixels
    applyZoomScroll(dy);
    flashZoomButton(-dy); // dy < 0 = zoom in (positive direction)
  }, { passive: false });

  // Translucent feedback joystick that shows at the touch point during drag.
  // One element per drag handler so the rotation and movement gestures can be
  // visualized simultaneously when the user has two fingers down.
  const overlayEl = document.getElementById('overlay');
  const dragRingRadius = 55;       // half of 110px width
  const dragThumbMax = dragRingRadius - 8;
  const createDragFeedback = () => {
    const ring = document.createElement('div');
    ring.className = 'drag-joystick';
    const thumb = document.createElement('div');
    thumb.className = 'drag-joystick-thumb';
    ring.appendChild(thumb);
    overlayEl.appendChild(ring);
    let lingerTimer = null;
    return {
      onDragStart(x, y) {
        if (lingerTimer) { clearTimeout(lingerTimer); lingerTimer = null; }
        ring.style.left = `${x - dragRingRadius}px`;
        ring.style.top = `${y - dragRingRadius}px`;
        thumb.style.transform = '';
        ring.classList.remove('lingering');
        ring.classList.add('active');
      },
      onDragMove(_x, _y, totalDx, totalDy) {
        let dx = totalDx, dy = totalDy;
        const dist = Math.hypot(dx, dy);
        if (dist > dragThumbMax) { dx = dx / dist * dragThumbMax; dy = dy / dist * dragThumbMax; }
        thumb.style.transform = `translate(${dx}px, ${dy}px)`;
      },
      onDragEnd() {
        ring.classList.remove('active');
        ring.classList.add('lingering');
        // Linger faded for the latency window — that's how long it takes for
        // input still in inputHistory to be replayed by readDelayedInput.
        const lingerMs = Math.max(120, state.latency * 1000);
        lingerTimer = setTimeout(() => {
          ring.classList.remove('lingering');
          lingerTimer = null;
        }, lingerMs);
      }
    };
  };

  // Touch-only devices split the canvas: right-half drags rotate; left-half
  // drags translate. On non-touch devices, a drag anywhere rotates and the
  // movement joystick stays inactive (the bottom-left WASD pad is still wired).
  const isTouchDevice = () => matchMedia('(hover: none) and (pointer: coarse)').matches;

  const rotateFeedback = createDragFeedback();
  state.dragInput = attachDragRotate(renderer.domElement, {
    // On mobile: right half only, and never while locked-on (rotation is
    // overridden each frame by the lock-on geometry). On desktop: full canvas.
    shouldStart: (e) => {
      if (!isTouchDevice()) return true;
      return !state.lockOn.active && e.clientX > window.innerWidth / 2;
    },
    ...rotateFeedback
  });

  const moveFeedback = createDragFeedback();
  state.dragMoveInput = attachDragMove(renderer.domElement, {
    // Mobile only. Left half normally; full canvas while locked-on (rotation
    // is disabled there, so the whole screen becomes movement).
    shouldStart: (e) => {
      if (!isTouchDevice()) return false;
      return state.lockOn.active || e.clientX <= window.innerWidth / 2;
    },
    onActivate: () => {
      // Same hooks as a WASD keypress — pause the route, capture free-fly
      // pose, and abort any in-progress focus animation.
      ensurePaused();
      ensureFreeFlyInitialized();
      cancelFocusAnim();
    },
    ...moveFeedback
  });

  // Movement buttons — hold-to-move. Pressing any movement control auto-pauses
  // (ensurePaused will enqueue a pause if currently playing, accounting for any
  // pause already in flight). It also cancels an in-progress focus animation.
  const movementBtns = ['forward', 'back', 'left', 'right', 'up', 'down'];
  for (const dir of movementBtns) {
    const btn = document.getElementById(`move-${dir}`);
    if (!btn) continue;
    const press = (e) => {
      e.preventDefault();
      try { btn.setPointerCapture(e.pointerId); } catch (_) {}
      onMovementPressed(dir);
      btn.classList.add('held');
    };
    const release = (e) => {
      try { btn.releasePointerCapture(e.pointerId); } catch (_) {}
      state.movement[dir] = 0;
      btn.classList.remove('held');
    };
    btn.addEventListener('pointerdown', press);
    btn.addEventListener('pointerup', release);
    btn.addEventListener('pointercancel', release);
    btn.addEventListener('pointerleave', release);
  }
  // WASD + Q/E keyboard equivalents (active only in free-fly via tick gating).
  // Toggling `.held` on the on-screen button mirrors the visual feedback the
  // user sees when pressing the button directly.
  const wasdMap = { KeyW: 'forward', KeyS: 'back', KeyA: 'left', KeyD: 'right', KeyE: 'up', KeyQ: 'down' };
  window.addEventListener('keydown', (e) => {
    const dir = wasdMap[e.code];
    if (!dir) return;
    if (e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA' || e.target.isContentEditable)) return;
    e.preventDefault();
    if (e.repeat) return; // ignore key-repeat — only treat the initial press as a "new" input
    onMovementPressed(dir);
    document.getElementById(`move-${dir}`)?.classList.add('held');
  });
  window.addEventListener('keyup', (e) => {
    const dir = wasdMap[e.code];
    if (!dir) return;
    state.movement[dir] = 0;
    document.getElementById(`move-${dir}`)?.classList.remove('held');
  });

  // Double-tap (touch) / double-click (mouse) on the canvas to focus the
  // camera on the tapped map point. Browser `dblclick` doesn't fire reliably
  // on mobile with `touch-action: none`, so we detect tap-tap-up explicitly.
  // Raycast happens immediately (uses current camera) so the hit point is
  // accurate; the actual focus action is queued through the latency model.
  const tapRay = new THREE.Raycaster();
  tapRay.firstHitOnly = true;
  attachDoubleTap(renderer.domElement, (clientX, clientY) => {
    if (!state.model) return;
    const rect = renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1
    );
    tapRay.setFromCamera(ndc, camera);
    const hits = tapRay.intersectObject(state.model, true);
    if (!hits.length) return;
    const point = hits[0].point.clone();
    enqueueAction(() => focusOnPoint(point));
  });

  syncOverlayPauseIcon();
  syncOverlayMovementVisible();
  updateZoomReadout();
}

// ─── Resize ───────────────────────────────────────────────────────────────────
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// ─── Animation loop ───────────────────────────────────────────────────────────
const clock = new THREE.Clock();

const fpsEl = document.getElementById('fps');
const fpsState = { accum: 0, count: 0, maxDt: 0, lastUpdate: 0 };

function tick() {
  const dt = clock.getDelta();

  // ── FPS counter (avg + worst-frame in a 0.5s window so hitches show) ──
  if (fpsEl) {
    fpsState.accum += dt;
    fpsState.count += 1;
    if (dt > fpsState.maxDt) fpsState.maxDt = dt;
    const now = nowSec();
    if (now - fpsState.lastUpdate > 0.5 && fpsState.count > 0) {
      const avgDt = fpsState.accum / fpsState.count;
      const fps = 1 / avgDt;
      const minFps = fpsState.maxDt > 0 ? 1 / fpsState.maxDt : 0;
      const cls = minFps < 30 ? 'bad' : minFps < 50 ? 'warn' : '';
      fpsEl.innerHTML = `${fps.toFixed(0)} fps · ${(avgDt * 1000).toFixed(1)}ms <span class="${cls}">(min ${minFps.toFixed(0)})</span>`;
      fpsState.accum = 0; fpsState.count = 0; fpsState.maxDt = 0; fpsState.lastUpdate = now;
    }
  }

  // Latency-applied user input — process before everything else so this frame
  // already reflects the (delayed) state changes.
  processActionQueue();
  const cur = state.gimbalInput ? state.gimbalInput.read() : { yawRate: 0, pitchRate: 0 };
  // Drag is also rate-based — its thumb offset produces a continuous rate
  // until the user releases. Sum with the joystick/keyboard rate, clamped.
  const dragR = state.dragInput ? state.dragInput.read() : { yawRate: 0, pitchRate: 0 };
  const yawRateNow = clampNum(cur.yawRate + dragR.yawRate, -1, 1);
  const pitchRateNow = clampNum(cur.pitchRate + dragR.pitchRate, -1, 1);
  const m = state.movement;
  const moveDrag = state.dragMoveInput ? state.dragMoveInput.read() : { fwdRate: 0, strafeRate: 0 };
  const moveFwdNow = clampNum((m.forward ? 1 : 0) - (m.back ? 1 : 0) + moveDrag.fwdRate, -1, 1);
  const moveStrafeNow = clampNum((m.right ? 1 : 0) - (m.left ? 1 : 0) + moveDrag.strafeRate, -1, 1);
  const moveUpNow = (m.up ? 1 : 0) - (m.down ? 1 : 0);
  pushInputSample(yawRateNow, pitchRateNow, moveFwdNow, moveStrafeNow, moveUpNow);

  // Apply (delayed) gimbal pitch + drone yaw rotation rates.
  const delayed = readDelayedInput();
  const g = state.gimbal;
  g.pitch = clampNum(g.pitch + delayed.pitchRate * g.pitchSpeed * dt, g.pitchMin, g.pitchMax);

  // Yaw input rotates the drone (real-world gimbal can't pan independently).
  // Joystick right (+1) → camera turns right; subtracting from yaw matches
  // three.js's Y-rotation sign convention.
  // - In lock-on: ignored (orbital code overrides drone yaw each frame).
  // - In free-fly: applied to freeFly.yaw (the drone's standalone world yaw).
  // - During playback / transition: applied to userWorldYaw — the *absolute*
  //   world yaw the camera should hold. The first input snapshots the drone's
  //   current world yaw as the baseline, so the camera stays world-locked
  //   even as the route turns.
  const yawDelta = delayed.yawRate * THREE.MathUtils.degToRad(g.yawSpeed) * dt;
  if (state.lockOn.active) {
    // no-op
  } else if (state.freeFly.initialized && !state.playing && !state.transition.active) {
    state.freeFly.yaw -= yawDelta;
  } else {
    if (yawDelta !== 0 && !state.userYawActive && state.drone) {
      _droneFwdScratch.set(0, 0, 1).applyQuaternion(state.drone.quaternion);
      state.userWorldYaw = Math.atan2(_droneFwdScratch.x, _droneFwdScratch.z);
      state.userYawActive = true;
    }
    state.userWorldYaw -= yawDelta;
  }
  g.yaw = 0; // gimbal yaw is no longer an independent control

  // ── Lock-on / orbit mode ─────────────────────────────────────────────
  // Higher priority than free-fly. Movement controls take orbital semantics:
  // forward/back = closer/farther, left/right = orbit around the target,
  // up/down = altitude. Drone+gimbal are forced to keep the target centered.
  if (state.lockOn.active && state.lockOn.point && state.freeFly.initialized && !state.playing && !state.transition.active) {
    const target = state.lockOn.point;
    const fwdIn = delayed.moveFwd || 0;
    const strafeIn = delayed.moveStrafe || 0;
    const vertIn = delayed.moveUp || 0;

    // Radial: from target toward drone, horizontal, normalized.
    const dx = state.freeFly.pos.x - target.x;
    const dz = state.freeFly.pos.z - target.z;
    const horiz = Math.hypot(dx, dz);
    let rX, rZ;
    if (horiz > 0.5) {
      rX = dx / horiz; rZ = dz / horiz;
    } else {
      // Drone right above target — radial is undefined. Fall back to +X so
      // pressing forward/back gets the drone out of the singularity.
      rX = 1; rZ = 0;
    }
    // Tangent = 90° CW rotation of radial in XZ plane (so right-input = clockwise from above).
    const tX = rZ, tZ = -rX;

    // Build target world velocity. Forward = closer = -radial direction.
    const targetVel = new THREE.Vector3(
      (-fwdIn * rX + strafeIn * tX) * m.speed,
      vertIn * m.verticalSpeed,
      (-fwdIn * rZ + strafeIn * tZ) * m.speed
    );

    const k = 1 - Math.exp(-m.accelRate * dt);
    m.velocity.lerp(targetVel, k);

    if (m.velocity.lengthSq() > 1e-4) {
      state.freeFly.pos.addScaledVector(m.velocity, dt);
      // Terrain floor — same as free-fly.
      const ty = getTerrainY(state.freeFly.pos.x, state.freeFly.pos.z);
      if (ty !== null) {
        const floor = ty + m.minClearance;
        if (state.freeFly.pos.y < floor) {
          state.freeFly.pos.y = floor;
          if (m.velocity.y < 0) m.velocity.y = 0;
        }
      }
    } else {
      m.velocity.set(0, 0, 0);
    }

    // Always re-aim drone+gimbal at the target so it stays centered.
    const tdx = target.x - state.freeFly.pos.x;
    const tdy = target.y - state.freeFly.pos.y;
    const tdz = target.z - state.freeFly.pos.z;
    const horizDist = Math.max(0.01, Math.hypot(tdx, tdz));
    state.freeFly.yaw = Math.atan2(tdx, tdz);
    state.gimbal.yaw = 0;
    state.gimbal.pitch = clampNum(
      THREE.MathUtils.radToDeg(Math.atan2(tdy, horizDist)),
      state.gimbal.pitchMin, state.gimbal.pitchMax
    );
  }
  // Free-fly movement (only when paused, free-fly initialized, and not transitioning).
  // Movement direction is camera-relative. Velocity eases toward the target each frame
  // so press/release feel like real drone inertia rather than instant on/off.
  else if (state.freeFly.initialized && !state.playing && !state.transition.active && state.curve) {
    const fwd = delayed.moveFwd || 0;
    const strafe = delayed.moveStrafe || 0;
    const vert = delayed.moveUp || 0;

    // Build target velocity in world space from current camera-relative input.
    // Horizontal is built from camera fwd/right (gimbal-relative); vertical is world Y.
    const camFwd = new THREE.Vector3(0, 0, -1).applyQuaternion(camera.quaternion);
    camFwd.y = 0;
    if (camFwd.lengthSq() < 1e-6) camFwd.set(0, 0, -1);
    camFwd.normalize();
    const camRight = new THREE.Vector3(-camFwd.z, 0, camFwd.x);

    const horiz = new THREE.Vector3()
      .addScaledVector(camFwd, fwd)
      .addScaledVector(camRight, strafe);
    if (horiz.lengthSq() > 1) horiz.normalize();
    horiz.multiplyScalar(m.speed);

    const targetVel = new THREE.Vector3(horiz.x, vert * m.verticalSpeed, horiz.z);

    // Frame-rate-independent velocity easing.
    const k = 1 - Math.exp(-m.accelRate * dt);
    m.velocity.lerp(targetVel, k);

    // Apply velocity if it's non-trivial (avoid drifting from float noise).
    if (m.velocity.lengthSq() > 1e-4) {
      state.freeFly.pos.addScaledVector(m.velocity, dt);
      // Terrain MIN clamp — never let the drone descend below (terrain + clearance),
      // but allow the user to climb above it. Also kill downward velocity at the floor
      // so we don't keep sliding into the ground.
      const ty = getTerrainY(state.freeFly.pos.x, state.freeFly.pos.z);
      if (ty !== null) {
        const floor = ty + m.minClearance;
        if (state.freeFly.pos.y < floor) {
          state.freeFly.pos.y = floor;
          if (m.velocity.y < 0) m.velocity.y = 0;
        }
      }

      // (Drone no longer auto-rotates to align with movement direction. The
      // omnidirectional drone can fly in any direction without changing its
      // heading; rotation is purely user-driven via yaw input above.)
    } else {
      m.velocity.set(0, 0, 0); // snap to zero once below threshold
    }
  } else {
    // Outside free-fly, decay any leftover velocity to zero so a future free-fly
    // session starts clean.
    if (m.velocity.lengthSq() > 0) m.velocity.set(0, 0, 0);
  }

  // Advance the focus animation (double-click-to-fly-and-look-at). Drone
  // glides along a smoothstep curve from fromPos to toPos; each frame the
  // drone+gimbal re-aim at the focus point, so the camera continuously tracks
  // the target through the glide. Cancelled by any movement input.
  if (state.focusAnim.active) {
    const fa = state.focusAnim;
    fa.t = Math.min(1, fa.t + dt / fa.duration);
    const u = smoothstep01(fa.t);
    state.freeFly.pos.lerpVectors(fa.fromPos, fa.toPos, u);
    // Aim drone+gimbal at the focus point. Position is C1-continuous via
    // smoothstep, so the angles change smoothly too — no snap.
    const dx = fa.point.x - state.freeFly.pos.x;
    const dy = fa.point.y - state.freeFly.pos.y;
    const dz = fa.point.z - state.freeFly.pos.z;
    const horizDist = Math.max(0.01, Math.hypot(dx, dz));
    state.freeFly.yaw = Math.atan2(dx, dz);
    state.gimbal.yaw = 0;
    state.gimbal.pitch = clampNum(
      THREE.MathUtils.radToDeg(Math.atan2(dy, horizDist)),
      state.gimbal.pitchMin, state.gimbal.pitchMax
    );
    if (fa.t >= 1) fa.active = false;
  }

  // Advance the return-to-route transition if active.
  if (state.transition.active) {
    state.transition.t += dt / state.transition.duration;
    if (state.transition.t >= 1) {
      state.transition.t = 1;
      state.transition.active = false;
      state.playing = true;
      // Free-fly stays "initialized" so the next pause re-captures from the new pose.
      // Update freeFly.pos/yaw to match the route now, in case the user pauses again immediately.
      state.freeFly.pos.copy(state.curve.getPoint(THREE.MathUtils.clamp(state.t, 0, 1)));
      syncOverlayPauseIcon();
      syncOverlayMovementVisible();
    }
  }
  // Gimbal targets the camera fov for zoom — eased so both button-tier jumps
  // and scroll deltas animate smoothly rather than snapping.
  const targetFov = g.baseFov / g.zoom;
  if (Math.abs(camera.fov - targetFov) > 0.001) {
    const k = 1 - Math.exp(-g.zoomEaseRate * dt);
    camera.fov += (targetFov - camera.fov) * k;
    camera.updateProjectionMatrix();
  }

  if (state.playing && state.curve) {
    state.t += (dt / state.duration) * state.speed;
    if (state.t >= 1) {
      if (state.loop) state.t = state.t % 1;
      else {
        state.t = 1;
        state.playing = false;
        syncOverlayPauseIcon();
      }
    }
    rebuildPathMeshes();
    if (state._tController) state._tController.updateDisplay();
  }
  // Always recompute drone transform (gimbal angle changes affect camera target
  // even when the drone isn't moving).
  updateDroneTransform();

  if (state.drone?.userData.rotors) {
    const spin = (state.playing ? 35 : 8) * dt;
    for (const r of state.drone.userData.rotors) r.rotation.y += spin;
  }

  // First-person camera easing — runs every frame so the camera smoothly catches
  // up to the drone's pose whether we're playing or paused-and-scrubbing.
  if (state.cameraMode === 'firstPerson' && state.cameraTargetReady) {
    if (state.cameraSnap) {
      camera.position.copy(state.cameraTargetPos);
      camera.quaternion.copy(state.cameraTargetQuat);
      state.cameraSnap = false;
    } else {
      // Frame-rate-independent damping: k = 1 - e^(-rate * dt). During a focus
      // animation we use a slower rotation rate so the camera eases into the new
      // heading like a physical gimbal instead of snapping to it.
      const turnRate = state.focusAnim.active ? state.focusAnim.turnRate : state.cameraTurnRate;
      const kRot = 1 - Math.exp(-turnRate * dt);
      const kPos = 1 - Math.exp(-state.cameraMoveRate * dt);
      camera.position.lerp(state.cameraTargetPos, kPos);
      camera.quaternion.slerp(state.cameraTargetQuat, kRot);
    }
  }

  // OrbitControls.update() ends with camera.lookAt(controls.target) — even
  // when disabled — which would silently force the camera to look at the
  // model center. Only run it in orbit mode.
  if (state.cameraMode === 'orbit') controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}
tick();
