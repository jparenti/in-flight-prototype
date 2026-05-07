// Centralizes all "user input" for the gimbal: joystick (touch/mouse), arrow keys,
// and zoom buttons. Exposes a continuous {yawRate, pitchRate, zoomRate} reading
// in the range [-1, +1]. The main loop applies it (with latency) to the gimbal angles.

export function attachGimbalControls(opts = {}) {
  const onZoomStep = opts.onZoomStep || (() => {});
  // Internal input state: rates in [-1, +1] from each source. Combine = clamp(sum).
  const sources = { joystick: { x: 0, y: 0 }, keys: { x: 0, y: 0 } };

  // ── Keyboard arrows ──
  const keyState = { ArrowUp: 0, ArrowDown: 0, ArrowLeft: 0, ArrowRight: 0 };
  const updateKeyRates = () => {
    const x = (keyState.ArrowRight ? 1 : 0) - (keyState.ArrowLeft ? 1 : 0);
    const y = (keyState.ArrowUp ? 1 : 0) - (keyState.ArrowDown ? 1 : 0);
    sources.keys.x = x;
    sources.keys.y = y;
  };
  const isTextField = (el) => {
    if (!el) return false;
    const tag = el.tagName;
    return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || el.isContentEditable;
  };
  window.addEventListener('keydown', (e) => {
    if (isTextField(e.target)) return;
    if (e.code in keyState) {
      e.preventDefault();
      keyState[e.code] = 1;
      updateKeyRates();
    }
  });
  window.addEventListener('keyup', (e) => {
    if (e.code in keyState) {
      keyState[e.code] = 0;
      updateKeyRates();
    }
  });

  // ── Joystick (touch + mouse) ──
  const stick = document.getElementById('joystick');
  const thumb = document.getElementById('joystick-thumb');
  if (stick && thumb) {
    let active = false;
    let pointerId = null;
    let centerX = 0, centerY = 0, radius = 0;

    const begin = (clientX, clientY, id) => {
      const rect = stick.getBoundingClientRect();
      centerX = rect.left + rect.width / 2;
      centerY = rect.top + rect.height / 2;
      radius = rect.width / 2 - 8; // leave a small margin so thumb edge stays inside
      active = true;
      pointerId = id;
      stick.classList.add('active');
      move(clientX, clientY);
    };
    const move = (clientX, clientY) => {
      let dx = clientX - centerX;
      let dy = clientY - centerY;
      const dist = Math.hypot(dx, dy);
      const max = radius;
      if (dist > max) { dx = (dx / dist) * max; dy = (dy / dist) * max; }
      thumb.style.transform = `translate(${dx}px, ${dy}px)`;
      // Convert to normalized rates. Y is screen-down-positive, but for tilt
      // up should be positive — invert.
      sources.joystick.x = dx / max;
      sources.joystick.y = -dy / max;
    };
    const end = () => {
      active = false;
      pointerId = null;
      stick.classList.remove('active');
      thumb.style.transform = '';
      sources.joystick.x = 0;
      sources.joystick.y = 0;
    };

    stick.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      stick.setPointerCapture(e.pointerId);
      begin(e.clientX, e.clientY, e.pointerId);
    });
    stick.addEventListener('pointermove', (e) => {
      if (!active || e.pointerId !== pointerId) return;
      move(e.clientX, e.clientY);
    });
    const cancel = (e) => {
      if (!active || e.pointerId !== pointerId) return;
      try { stick.releasePointerCapture(e.pointerId); } catch (_) {}
      end();
    };
    stick.addEventListener('pointerup', cancel);
    stick.addEventListener('pointercancel', cancel);
  }

  // ── Zoom buttons (hold-to-zoom, but each press also fires an immediate step) ──
  const wireHoldZoom = (id, dir) => {
    const btn = document.getElementById(id);
    if (!btn) return;
    let interval = null;
    const stop = () => { if (interval) { clearInterval(interval); interval = null; } };
    btn.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      onZoomStep(dir);                                     // immediate step
      interval = setInterval(() => onZoomStep(dir), 90);   // continuous while held
    });
    ['pointerup', 'pointercancel', 'pointerleave'].forEach((ev) => btn.addEventListener(ev, stop));
  };
  wireHoldZoom('zoom-in', +1);
  wireHoldZoom('zoom-out', -1);

  // ── Public reading ──
  return {
    read() {
      const x = clamp(sources.joystick.x + sources.keys.x, -1, 1);
      const y = clamp(sources.joystick.y + sources.keys.y, -1, 1);
      return { yawRate: x, pitchRate: y };
    }
  };
}

function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }
