// Drag rotation input.
//
// Pointer drag is *rate-based* — the offset of the finger from the original
// press point produces a normalized rate in [-1, +1] on each axis (clamped at
// `dragMaxOffsetPx` for full deflection). Holding still off-center keeps the
// rate active, exactly like a held joystick. read() returns this rate so it
// can be summed with the existing joystick/keyboard rate sources.
//
// Sign convention: camera moves *with* the gesture. Drag right → positive
// yaw rate; drag down → negative pitch rate (look down). Matches the existing
// joystick mapping (right = +x, up = +y).

export function attachDragRotate(target, opts = {}) {
  const dragThresholdPx = opts.dragThresholdPx ?? 4;     // ignore micro-jitter so taps still dblclick
  const dragMaxOffsetPx = opts.dragMaxOffsetPx ?? 47;    // distance from press point that maps to ±1 rate

  const shouldStart = opts.shouldStart || (() => true);  // gate per-pointerdown (e.g. right half on mobile)
  const onDragStart = opts.onDragStart || (() => {});
  const onDragMove = opts.onDragMove || (() => {});
  const onDragEnd = opts.onDragEnd || (() => {});

  let dragYawRate = 0;
  let dragPitchRate = 0;
  let pointerId = null;
  let dragging = false;
  let downX = 0, downY = 0;

  target.addEventListener('pointerdown', (e) => {
    if (e.pointerType === 'mouse' && e.button !== 0) return;
    if (!shouldStart(e)) return;
    pointerId = e.pointerId;
    downX = e.clientX;
    downY = e.clientY;
    dragging = false;
  });
  target.addEventListener('pointermove', (e) => {
    if (e.pointerId !== pointerId) return;
    const totalDx = e.clientX - downX;
    const totalDy = e.clientY - downY;
    if (!dragging) {
      if (Math.hypot(totalDx, totalDy) < dragThresholdPx) return;
      dragging = true;
      try { target.setPointerCapture(e.pointerId); } catch (_) {}
      onDragStart(downX, downY);
    }
    // Rate = clamped offset from press point, normalized to [-1, +1] on each axis.
    // Clamp magnitude to 1 so diagonal drags don't exceed full rate.
    const dist = Math.hypot(totalDx, totalDy);
    if (dist > 0) {
      const clampedMag = Math.min(1, dist / dragMaxOffsetPx);
      dragYawRate = (totalDx / dist) * clampedMag;
      dragPitchRate = -(totalDy / dist) * clampedMag;
    } else {
      dragYawRate = 0;
      dragPitchRate = 0;
    }
    onDragMove(e.clientX, e.clientY, totalDx, totalDy);
  });
  const endDrag = (e) => {
    if (e.pointerId !== pointerId) return;
    const wasDragging = dragging;
    if (dragging) { try { target.releasePointerCapture(e.pointerId); } catch (_) {} }
    pointerId = null;
    dragging = false;
    dragYawRate = 0;
    dragPitchRate = 0;
    if (wasDragging) onDragEnd();
  };
  target.addEventListener('pointerup', endDrag);
  target.addEventListener('pointercancel', endDrag);

  return {
    /** Current drag rate in [-1, +1]. Sum with the joystick/keyboard rate. */
    read() {
      return { yawRate: dragYawRate, pitchRate: dragPitchRate };
    },
    isDragging: () => dragging
  };
}
