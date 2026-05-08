// Manual double-tap / double-click detector.
//
// The browser `dblclick` event is unreliable on mobile (especially when the
// canvas has `touch-action: none` to suppress browser gestures), so we listen
// for sub-threshold tap-tap-up sequences explicitly. Works for mouse, touch,
// and pen.

export function attachDoubleTap(target, onDoubleTap, opts = {}) {
  const tapMaxDur = opts.tapMaxDur ?? 300;        // ms — long press doesn't count
  const tapMaxMove = opts.tapMaxMove ?? 8;         // px — drag doesn't count
  const dblTapMaxGap = opts.dblTapMaxGap ?? 400;   // ms between taps (covers macOS dblclick default)
  const dblTapMaxDist = opts.dblTapMaxDist ?? 40;  // px between taps

  let downId = null;
  let downT = 0, downX = 0, downY = 0;
  let movedTooFar = false;
  let lastTapT = 0, lastTapX = 0, lastTapY = 0;

  target.addEventListener('pointerdown', (e) => {
    if (e.pointerType === 'mouse' && e.button !== 0) return;
    downId = e.pointerId;
    downT = performance.now();
    downX = e.clientX;
    downY = e.clientY;
    movedTooFar = false;
  });
  target.addEventListener('pointermove', (e) => {
    if (e.pointerId !== downId || movedTooFar) return;
    if (Math.hypot(e.clientX - downX, e.clientY - downY) > tapMaxMove) movedTooFar = true;
  });
  target.addEventListener('pointerup', (e) => {
    if (e.pointerId !== downId) return;
    downId = null;
    if (movedTooFar) return;
    const now = performance.now();
    if (now - downT > tapMaxDur) return;
    if (lastTapT &&
        now - lastTapT <= dblTapMaxGap &&
        Math.hypot(e.clientX - lastTapX, e.clientY - lastTapY) <= dblTapMaxDist) {
      lastTapT = 0; // consume the prior tap
      onDoubleTap(e.clientX, e.clientY);
    } else {
      lastTapT = now;
      lastTapX = e.clientX;
      lastTapY = e.clientY;
    }
  });
  target.addEventListener('pointercancel', (e) => {
    if (e.pointerId !== downId) return;
    downId = null;
    movedTooFar = true;
  });
}
