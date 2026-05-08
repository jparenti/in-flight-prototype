// Drag-move joystick (mobile-left-half). Produces continuous translation rates
// `fwdRate` and `strafeRate` in [-1, +1] from the thumb offset, mirroring WASD
// semantics. Up-on-screen = forward, right-on-screen = strafe right.
//
// `onActivate` fires once when the drag crosses the start threshold — callers
// use it to ensurePaused() / cancelFocusAnim() so the route auto-pauses on
// first drag, identical to how a WASD keypress behaves.

export function attachDragMove(target, opts = {}) {
  const dragThresholdPx = opts.dragThresholdPx ?? 4;
  const dragMaxOffsetPx = opts.dragMaxOffsetPx ?? 47;

  const shouldStart = opts.shouldStart || (() => true);
  const onActivate = opts.onActivate || (() => {});
  const onDragStart = opts.onDragStart || (() => {});
  const onDragMove = opts.onDragMove || (() => {});
  const onDragEnd = opts.onDragEnd || (() => {});

  let fwdRate = 0;
  let strafeRate = 0;
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
      onActivate();
      onDragStart(downX, downY);
    }
    const dist = Math.hypot(totalDx, totalDy);
    if (dist > 0) {
      const mag = Math.min(1, dist / dragMaxOffsetPx);
      strafeRate = (totalDx / dist) * mag;
      fwdRate = -(totalDy / dist) * mag; // screen-up (-dy) = forward (+1)
    } else {
      strafeRate = 0; fwdRate = 0;
    }
    onDragMove(e.clientX, e.clientY, totalDx, totalDy);
  });
  const endDrag = (e) => {
    if (e.pointerId !== pointerId) return;
    const wasDragging = dragging;
    if (dragging) { try { target.releasePointerCapture(e.pointerId); } catch (_) {} }
    pointerId = null;
    dragging = false;
    fwdRate = 0;
    strafeRate = 0;
    if (wasDragging) onDragEnd();
  };
  target.addEventListener('pointerup', endDrag);
  target.addEventListener('pointercancel', endDrag);

  return {
    /** Current rate in [-1, +1]. Sum with WASD-derived raw input. */
    read() { return { fwdRate, strafeRate }; },
    isDragging: () => dragging
  };
}
