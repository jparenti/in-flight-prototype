// Generates sprite textures procedurally on canvas. No external assets required.
import * as THREE from 'three';

// Draws a tiny camera glyph (body + viewfinder + lens) centered at (cx, cy).
function drawCameraGlyph(ctx, cx, cy, size, fillColor, lensColor) {
  ctx.fillStyle = fillColor;
  // viewfinder bump on top
  ctx.fillRect(cx - size * 0.18, cy - size * 0.55, size * 0.36, size * 0.18);
  // body
  const bodyW = size;
  const bodyH = size * 0.72;
  ctx.beginPath();
  const r = size * 0.1;
  const x = cx - bodyW / 2, y = cy - bodyH / 2 + size * 0.06;
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + bodyW, y, x + bodyW, y + bodyH, r);
  ctx.arcTo(x + bodyW, y + bodyH, x, y + bodyH, r);
  ctx.arcTo(x, y + bodyH, x, y, r);
  ctx.arcTo(x, y, x + bodyW, y, r);
  ctx.closePath();
  ctx.fill();
  // lens
  ctx.beginPath();
  ctx.arc(cx, cy + size * 0.06, size * 0.22, 0, Math.PI * 2);
  ctx.fillStyle = lensColor;
  ctx.fill();
}

function makeCircleSprite({
  size = 128, fill, stroke = '#0b0d10', strokeWidth = 6,
  label = '', labelColor = '#0b0d10', cameraBadge = false
}) {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext('2d');

  // drop shadow
  ctx.shadowColor = 'rgba(0,0,0,0.55)';
  ctx.shadowBlur = 18;
  ctx.shadowOffsetY = 4;

  // pin body
  const r = size / 2 - strokeWidth - 8;
  ctx.beginPath();
  ctx.arc(size / 2, size / 2, r, 0, Math.PI * 2);
  ctx.fillStyle = fill;
  ctx.fill();

  // remove shadow for stroke + label
  ctx.shadowColor = 'transparent';
  ctx.lineWidth = strokeWidth;
  ctx.strokeStyle = stroke;
  ctx.stroke();

  if (label) {
    ctx.fillStyle = labelColor;
    ctx.font = `bold ${Math.round(size * 0.42)}px -apple-system, "Segoe UI", sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, size / 2, size / 2 + size * 0.02);
  }

  if (cameraBadge) {
    // Badge: dark circle in lower-right with a camera glyph
    const bx = size * 0.78, by = size * 0.78, br = size * 0.20;
    ctx.shadowColor = 'rgba(0,0,0,0.55)';
    ctx.shadowBlur = 8;
    ctx.shadowOffsetY = 2;
    ctx.beginPath();
    ctx.arc(bx, by, br, 0, Math.PI * 2);
    ctx.fillStyle = '#0b0d10';
    ctx.fill();
    ctx.shadowColor = 'transparent';
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = '#ffd400';
    ctx.stroke();
    drawCameraGlyph(ctx, bx, by, br * 0.9, '#ffd400', '#0b0d10');
  }

  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  return tex;
}

function makeLandingPadSprite({ size = 128 }) {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext('2d');

  ctx.shadowColor = 'rgba(0,0,0,0.55)';
  ctx.shadowBlur = 18;
  ctx.shadowOffsetY = 4;

  const r = size / 2 - 14;
  ctx.beginPath();
  ctx.arc(size / 2, size / 2, r, 0, Math.PI * 2);
  ctx.fillStyle = '#10131a';
  ctx.fill();

  ctx.shadowColor = 'transparent';
  ctx.lineWidth = 6;
  ctx.strokeStyle = '#ffd400';
  ctx.stroke();

  ctx.fillStyle = '#ffd400';
  ctx.font = `bold ${Math.round(size * 0.5)}px -apple-system, "Segoe UI", sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText('H', size / 2, size / 2 + size * 0.02);

  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  return tex;
}

export function createWaypointSprite(index, worldScale) {
  const tex = makeCircleSprite({
    fill: '#3aa7ff',
    label: String(index + 1),
    labelColor: '#0b0d10',
    cameraBadge: true
  });
  const mat = new THREE.SpriteMaterial({
    map: tex,
    depthTest: false,
    depthWrite: false,
    transparent: true,
    sizeAttenuation: true
  });
  const sprite = new THREE.Sprite(mat);
  sprite.scale.setScalar(worldScale);
  sprite.renderOrder = 10;
  return sprite;
}

// Smaller orange marker for ad-hoc photos taken between waypoints.
// Label like "1a", "1b" — derived from preceding waypoint number.
export function createPhotoSprite(label, worldScale) {
  const size = 128;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext('2d');

  ctx.shadowColor = 'rgba(0,0,0,0.55)';
  ctx.shadowBlur = 14;
  ctx.shadowOffsetY = 3;

  const r = size * 0.34;
  ctx.beginPath();
  ctx.arc(size / 2, size * 0.46, r, 0, Math.PI * 2);
  ctx.fillStyle = '#ff8a3d';
  ctx.fill();

  ctx.shadowColor = 'transparent';
  ctx.lineWidth = 5;
  ctx.strokeStyle = '#0b0d10';
  ctx.stroke();

  // camera glyph in the circle
  drawCameraGlyph(ctx, size / 2, size * 0.46, r * 0.9, '#0b0d10', '#ff8a3d');

  // label tag below the circle
  const tagText = label;
  ctx.font = `bold ${Math.round(size * 0.18)}px -apple-system, "Segoe UI", sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  const tagW = Math.max(ctx.measureText(tagText).width + 16, size * 0.35);
  const tagH = size * 0.22;
  const tagX = size / 2 - tagW / 2;
  const tagY = size * 0.82 - tagH / 2;

  ctx.shadowColor = 'rgba(0,0,0,0.5)';
  ctx.shadowBlur = 6;
  ctx.shadowOffsetY = 2;
  ctx.fillStyle = '#0b0d10';
  ctx.beginPath();
  const rr = tagH / 2;
  ctx.moveTo(tagX + rr, tagY);
  ctx.arcTo(tagX + tagW, tagY, tagX + tagW, tagY + tagH, rr);
  ctx.arcTo(tagX + tagW, tagY + tagH, tagX, tagY + tagH, rr);
  ctx.arcTo(tagX, tagY + tagH, tagX, tagY, rr);
  ctx.arcTo(tagX, tagY, tagX + tagW, tagY, rr);
  ctx.closePath();
  ctx.fill();

  ctx.shadowColor = 'transparent';
  ctx.fillStyle = '#ff8a3d';
  ctx.fillText(tagText, size / 2, tagY + tagH / 2 + 1);

  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;

  const mat = new THREE.SpriteMaterial({
    map: tex,
    depthTest: false,
    depthWrite: false,
    transparent: true,
    sizeAttenuation: true
  });
  const sprite = new THREE.Sprite(mat);
  sprite.scale.setScalar(worldScale);
  sprite.renderOrder = 9;
  return sprite;
}

export function createLandingPadSprite(worldScale) {
  const tex = makeLandingPadSprite({});
  const mat = new THREE.SpriteMaterial({
    map: tex,
    depthTest: false,
    depthWrite: false,
    transparent: true,
    sizeAttenuation: true
  });
  const sprite = new THREE.Sprite(mat);
  sprite.scale.setScalar(worldScale * 1.2);
  sprite.renderOrder = 11;
  return sprite;
}
