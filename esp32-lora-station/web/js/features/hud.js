// HUD kiểu Mission Planner cho phao báo qua MAVLink: chân trời nhân tạo có thang pitch,
// cung roll, thước hướng có vạch hướng đích, tốc độ, thanh ga, ARMED/chế độ, pin, GPS.
// Bên dưới là các ô số lớn như tab "Quick". Phao LoRa không có nhóm nav nên dùng thẻ tư thế cũ.
import { ATTITUDE_STALE_SEC, attitudeAgeSec } from '../state.js';
import { NONE } from '../util/format.js';

const $ = id => document.getElementById(id);
const SVGNS = 'http://www.w3.org/2000/svg';
const W = 320, H = 210, CX = 160, CY = 116;
const PX_PER_DEG = 3.2;        // thang pitch
const TAPE_PX_PER_DEG = 3.4;   // thước hướng
const TAPE_SPAN = 48;          // ± độ hiển thị trên thước hướng
const CARDINAL = { 0: 'N', 45: 'NE', 90: 'E', 135: 'SE', 180: 'S', 225: 'SW', 270: 'W', 315: 'NW' };

function el(name, attrs = {}, text) {
  const e = document.createElementNS(SVGNS, name);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  if (text !== undefined) e.textContent = text;
  return e;
}

const parts = {};

function build() {
  const svg = el('svg', { viewBox: `0 0 ${W} ${H}`, class: 'hud-svg', role: 'img',
                          'aria-label': 'Màn hình HUD: tư thế, hướng, tốc độ, ga và trạng thái phao' });
  const defs = el('defs');
  const clip = el('clipPath', { id: 'hud-clip' });
  clip.appendChild(el('rect', { x: 0, y: 0, width: W, height: H, rx: 8 }));
  defs.appendChild(clip);
  svg.appendChild(defs);
  const root = el('g', { 'clip-path': 'url(#hud-clip)' });
  svg.appendChild(root);

  // Thế giới: trời, đất, đường chân trời, thang pitch. Xoay theo roll, trượt theo pitch.
  const world = el('g', { class: 'hud-world' });
  world.appendChild(el('rect', { x: -400, y: CY - 900, width: W + 800, height: 900, class: 'hud-sky' }));
  world.appendChild(el('rect', { x: -400, y: CY, width: W + 800, height: 900, class: 'hud-ground' }));
  world.appendChild(el('line', { x1: -400, x2: W + 400, y1: CY, y2: CY, class: 'hud-horizon' }));
  for (let p = -40; p <= 40; p += 5) {
    if (p === 0) continue;
    const y = CY - p * PX_PER_DEG, half = p % 10 === 0 ? 28 : 14;
    world.appendChild(el('line', { x1: CX - half, x2: CX + half, y1: y, y2: y, class: 'hud-ladder' }));
    if (p % 10 === 0) {
      world.appendChild(el('text', { x: CX - half - 4, y: y + 3, 'text-anchor': 'end', class: 'hud-ladder-t' }, p));
      world.appendChild(el('text', { x: CX + half + 4, y: y + 3, class: 'hud-ladder-t' }, p));
    }
  }
  root.appendChild(world);
  parts.world = world;

  // Cung roll cố định + kim roll xoay theo thế giới
  const R = 84;
  const arcPt = a => [CX + R * Math.sin(a * Math.PI / 180), CY - R * Math.cos(a * Math.PI / 180)];
  const [ax, ay] = arcPt(-60), [bx, by] = arcPt(60);
  root.appendChild(el('path', { d: `M${ax},${ay} A${R},${R} 0 0 1 ${bx},${by}`, class: 'hud-arc' }));
  for (const a of [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
    const len = a % 30 === 0 ? 10 : 6;
    const [x1, y1] = arcPt(a);
    const x2 = CX + (R + len) * Math.sin(a * Math.PI / 180), y2 = CY - (R + len) * Math.cos(a * Math.PI / 180);
    root.appendChild(el('line', { x1, y1, x2, y2, class: 'hud-arc-tick' }));
  }
  const rollPtr = el('path', { d: `M${CX},${CY - R + 2} l-6,10 h12 z`, class: 'hud-roll-ptr' });
  root.appendChild(rollPtr);
  parts.rollPtr = rollPtr;

  // Ký hiệu thân phao cố định ở giữa (chữ W đỏ như Mission Planner)
  root.appendChild(el('path', { d: `M${CX - 42},${CY} h22 l8,8 l12,-12 l12,12 l8,-8 h22`, class: 'hud-boat' }));
  root.appendChild(el('circle', { cx: CX, cy: CY, r: 2.5, class: 'hud-boat-dot' }));

  // Thước hướng phía trên
  root.appendChild(el('rect', { x: 0, y: 0, width: W, height: 30, class: 'hud-band' }));
  const tape = el('g');
  root.appendChild(tape);
  parts.tape = tape;
  root.appendChild(el('path', { d: `M${CX},31 l-6,-7 h12 z`, class: 'hud-caret' }));
  root.appendChild(el('rect', { x: CX - 24, y: 33, width: 48, height: 18, rx: 3, class: 'hud-box' }));
  parts.hdg = el('text', { x: CX, y: 46, 'text-anchor': 'middle', class: 'hud-val' }, NONE);
  root.appendChild(parts.hdg);

  // Tốc độ (trái)
  root.appendChild(el('text', { x: 34, y: CY - 22, 'text-anchor': 'middle', class: 'hud-cap' }, 'TỐC ĐỘ'));
  root.appendChild(el('rect', { x: 6, y: CY - 16, width: 56, height: 26, rx: 3, class: 'hud-box' }));
  parts.gs = el('text', { x: 34, y: CY + 3, 'text-anchor': 'middle', class: 'hud-val big' }, NONE);
  root.appendChild(parts.gs);
  root.appendChild(el('text', { x: 34, y: CY + 22, 'text-anchor': 'middle', class: 'hud-cap' }, 'm/s'));

  // Thanh ga (phải): 0 ở giữa, tiến lên trên, lùi xuống dưới
  const TX = 292, TY = 56, TH = 120;
  root.appendChild(el('text', { x: TX + 7, y: TY - 6, 'text-anchor': 'middle', class: 'hud-cap' }, 'GA'));
  root.appendChild(el('rect', { x: TX, y: TY, width: 14, height: TH, rx: 3, class: 'hud-box' }));
  root.appendChild(el('line', { x1: TX - 3, x2: TX + 17, y1: TY + TH / 2, y2: TY + TH / 2, class: 'hud-arc-tick' }));
  parts.thrBar = el('rect', { x: TX + 2, y: TY + TH / 2, width: 10, height: 0, class: 'hud-thr' });
  root.appendChild(parts.thrBar);
  parts.thr = el('text', { x: TX + 7, y: TY + TH + 14, 'text-anchor': 'middle', class: 'hud-cap' }, NONE);
  root.appendChild(parts.thr);
  parts.thrGeom = { TY, TH };

  // ARMED / DISARMED, chế độ, pin, GPS
  parts.armed = el('text', { x: CX, y: CY + 44, 'text-anchor': 'middle', class: 'hud-armed' }, NONE);
  root.appendChild(parts.armed);
  root.appendChild(el('rect', { x: 0, y: H - 22, width: W, height: 22, class: 'hud-band' }));
  parts.mode = el('text', { x: 8, y: H - 7, class: 'hud-val' }, NONE);
  parts.gps = el('text', { x: CX, y: H - 7, 'text-anchor': 'middle', class: 'hud-small' }, NONE);
  parts.batt = el('text', { x: W - 8, y: H - 7, 'text-anchor': 'end', class: 'hud-small' }, NONE);
  root.append(parts.mode, parts.gps, parts.batt);

  parts.stale = el('text', { x: CX, y: 72, 'text-anchor': 'middle', class: 'hud-stale' }, 'DỮ LIỆU CŨ');
  parts.stale.style.display = 'none';
  root.appendChild(parts.stale);
  return svg;
}

function drawTape(yaw, target) {
  const g = parts.tape;
  g.replaceChildren();
  const start = Math.ceil((yaw - TAPE_SPAN) / 5) * 5;
  for (let d = start; d <= yaw + TAPE_SPAN; d += 5) {
    const x = CX + (d - yaw) * TAPE_PX_PER_DEG;
    const deg = ((d % 360) + 360) % 360;
    const major = deg % 15 === 0;
    g.appendChild(el('line', { x1: x, x2: x, y1: 30, y2: major ? 20 : 25, class: 'hud-tape-tick' }));
    if (major) {
      const label = CARDINAL[deg] ?? String(deg);
      g.appendChild(el('text', { x, y: 15, 'text-anchor': 'middle',
                                 class: CARDINAL[deg] ? 'hud-tape-t card' : 'hud-tape-t' }, label));
    }
  }
  if (target !== null) {
    const off = ((target - yaw + 540) % 360) - 180;
    const clamped = Math.max(-TAPE_SPAN, Math.min(TAPE_SPAN, off));
    const x = CX + clamped * TAPE_PX_PER_DEG;
    g.appendChild(el('path', { d: `M${x},30 l-6,-9 h12 z`, class: 'hud-target' + (clamped !== off ? ' edge' : '') }));
  }
}

const fmt = (v, digits) => (Number.isFinite(v) ? v.toFixed(digits) : NONE);

function setTiles(b) {
  const n = b.nav;
  $('q-gs').textContent = fmt(n.gs, 2);
  $('q-wp').textContent = n.wp_dist >= 0 ? fmt(n.wp_dist, 1) : NONE;
  $('q-hdg').textContent = fmt(b.yaw, 1);
  $('q-thr').textContent = String(n.thr);
  $('q-batt').textContent = n.batt_v > 0 ? fmt(n.batt_v, 2) : NONE;
  $('q-gps').textContent = `${n.sats}`;
  $('q-gps-sub').textContent = n.hdop < 99 ? `HDOP ${n.hdop.toFixed(1)}` : 'HDOP -';
  $('q-batt-sub').textContent = n.batt_pct >= 0 ? `${n.batt_pct} %` : 'V';
  $('q-wp-sub').textContent = n.wp_dist >= 0 ? `m · WP ${n.wp_seq}` : 'm';
}

// b: phao đang chọn. Trả về true nếu HUD đang hiển thị (phao có nhóm nav).
export function renderHud(b) {
  const box = $('d-hud');
  const show = !!(b && b.nav && b.hasAttitude);
  box.hidden = !show;
  if (!show) return false;
  if (!parts.world) $('hud-screen').appendChild(build());

  const n = b.nav;
  const stale = attitudeAgeSec(b) > ATTITUDE_STALE_SEC;
  parts.world.setAttribute('transform',
    `rotate(${-b.roll} ${CX} ${CY}) translate(0 ${Math.max(-60, Math.min(60, b.pitch)) * PX_PER_DEG})`);
  parts.rollPtr.setAttribute('transform', `rotate(${-b.roll} ${CX} ${CY})`);
  drawTape(b.yaw, b.targetYaw);
  parts.hdg.textContent = `${Math.round(b.yaw) % 360}°`;
  parts.gs.textContent = n.gs.toFixed(1);

  const { TY, TH } = parts.thrGeom;
  const h = Math.abs(n.thr) / 100 * (TH / 2 - 2);
  parts.thrBar.setAttribute('height', h.toFixed(1));
  parts.thrBar.setAttribute('y', (n.thr >= 0 ? TY + TH / 2 - h : TY + TH / 2).toFixed(1));
  parts.thrBar.setAttribute('class', 'hud-thr' + (n.thr < 0 ? ' rev' : ''));
  parts.thr.textContent = `${n.thr}%`;

  parts.armed.textContent = n.armed ? 'ARMED' : 'DISARMED';
  parts.armed.setAttribute('class', 'hud-armed ' + (n.armed ? 'on' : 'off'));
  parts.mode.textContent = n.mode_name;
  // Dải dưới hẹp (320 đơn vị): giữ chuỗi ngắn để chế độ, GPS và pin không đè nhau
  parts.gps.textContent = b.fix === 1 ? `GPS ${n.sats}` : 'GPS chưa fix';
  parts.batt.textContent = n.batt_v > 0 ? `${n.batt_v.toFixed(1)}V${n.batt_pct >= 0 ? ` ${n.batt_pct}%` : ''}` : 'Pin -';
  parts.stale.style.display = stale ? '' : 'none';
  box.classList.toggle('is-stale', stale);
  setTiles(b);
  return true;
}
