// Thẻ viễn trắc của phao đang chọn: tọa độ, RSSI/SNR, tuổi gói, đồ thị lịch sử.
import { state, statusOf, ageSec, attitudeAgeSec, ATTITUDE_STALE_SEC, STATUS_TEXT } from '../state.js';
import { fmtAgeSec, rssiClass, NONE } from '../util/format.js';
import { toDMS, distM, bearingDeg } from '../util/geo.js';
import { fmtDist } from '../util/format.js';
import { renderHud } from './hud.js';

const $ = id => document.getElementById(id);
const SVGNS = 'http://www.w3.org/2000/svg';
const WINDOWS = { '5': 300, '15': 900, all: Infinity };   // giây

let coordDMS = false;
let windowKey = '15';

const RSSI_MIN = -130, RSSI_MAX = -40;     // dBm hiển thị trên đồ thị
const SNR_MIN = -20, SNR_MAX = 15;         // dB

function el(name, attrs = {}) {
  const e = document.createElementNS(SVGNS, name);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  return e;
}

const signedHeadingError = (target, current) => (target - current + 540) % 360 - 180;

function renderAttitude(b) {
  const panel = $('d-attitude');
  const has = b.hasAttitude === true;
  const stale = has && attitudeAgeSec(b) > ATTITUDE_STALE_SEC;
  panel.classList.toggle('no-data', !has);
  panel.classList.toggle('is-stale', stale);

  const badge = $('d-attitude-state');
  badge.textContent = !has ? 'Chưa có IMU' : stale ? 'Dữ liệu cũ' : b.imuOk ? 'IMU tốt' : 'Lỗi IMU';
  badge.className = 'badge ' + (!has || stale ? 'stale' : b.imuOk ? 'online' : 'nofix');
  if (!has) {
    for (const id of ['d-roll','d-pitch','d-yaw','d-target-yaw','d-heading-mode','d-heading-error','d-imu-status','d-calib']) $(id).textContent = NONE;
    // Xóa dấu vết của phao chọn trước: inline style thắng lớp .no-data
    $('d-imu-status').className = '';
    $('d-target-needle').style.visibility = '';
    return;
  }

  $('d-roll').textContent = b.roll.toFixed(1);
  $('d-pitch').textContent = b.pitch.toFixed(1);
  $('d-yaw').textContent = b.yaw.toFixed(1);
  $('d-target-yaw').textContent = b.targetYaw === null ? NONE : b.targetYaw.toFixed(1);
  $('d-heading-mode').textContent = ({ A: 'AUTO', M: 'MANUAL', S: 'STOP' })[b.headingMode] || NONE;
  $('d-imu-status').textContent = b.imuOk ? 'OK' : 'LỖI';
  $('d-imu-status').className = b.imuOk ? 'ok' : 'bad';
  $('d-calib').textContent = `${b.calib}/3`;
  const error = b.targetYaw === null ? null : signedHeadingError(b.targetYaw, b.yaw);
  $('d-heading-error').textContent = error === null ? NONE : `${error >= 0 ? '+' : ''}${error.toFixed(1)}°`;

  $('d-horizon-world').style.transform = `translateY(${Math.max(-35, Math.min(35, b.pitch * 0.8))}px) rotate(${-b.roll}deg)`;
  $('d-yaw-needle').style.transform = `rotate(${b.yaw}deg)`;
  $('d-target-needle').style.transform = b.targetYaw === null ? 'rotate(0deg)' : `rotate(${b.targetYaw}deg)`;
  $('d-target-needle').style.visibility = b.targetYaw === null ? 'hidden' : 'visible';
}

function drawChart() {
  const box = $('d-chart');
  box.replaceChildren();
  const id = state.selected;
  const all = (state.history.get(id) || []).filter(p => Number.isFinite(p.rssi));
  const W = 320, H = 132, L = 34, R = 34, T = 8, B = 18;
  const svg = el('svg', { viewBox: `0 0 ${W} ${H}`, role: 'img', 'aria-label': 'Đồ thị RSSI và SNR theo thời gian' });

  const now = Date.now();
  const span = WINDOWS[windowKey];
  const pts = all.filter(p => now - p.t <= span * 1000);
  if (pts.length < 2) {
    const t = el('text', { x: W / 2, y: H / 2, 'text-anchor': 'middle', class: 'ch-empty' });
    t.textContent = all.length ? 'Chưa đủ mẫu trong khoảng thời gian này' : 'Chưa có dữ liệu lịch sử';
    svg.appendChild(t);
    box.appendChild(svg);
    return;
  }
  const t0 = span === Infinity ? pts[0].t : now - span * 1000;
  const t1 = now;
  const x = t => L + (t - t0) / Math.max(1, t1 - t0) * (W - L - R);
  const yR = v => T + (RSSI_MAX - Math.min(RSSI_MAX, Math.max(RSSI_MIN, v))) / (RSSI_MAX - RSSI_MIN) * (H - T - B);
  const yS = v => T + (SNR_MAX - Math.min(SNR_MAX, Math.max(SNR_MIN, v))) / (SNR_MAX - SNR_MIN) * (H - T - B);

  // Lưới ngang + nhãn trục RSSI (trái)
  for (const v of [-50, -70, -90, -110, -130]) {
    svg.appendChild(el('line', { x1: L, x2: W - R, y1: yR(v), y2: yR(v), class: 'ch-grid' }));
    const t = el('text', { x: L - 4, y: yR(v) + 3, 'text-anchor': 'end', class: 'ch-axis' });
    t.textContent = v;
    svg.appendChild(t);
  }
  // Ngưỡng cảnh báo
  for (const [v, cls] of [[-95, 'warn'], [-110, 'bad']]) {
    svg.appendChild(el('line', { x1: L, x2: W - R, y1: yR(v), y2: yR(v), class: 'ch-thr ' + cls }));
  }
  // Nhãn trục SNR (phải)
  for (const v of [10, 0, -10]) {
    const t = el('text', { x: W - R + 4, y: yS(v) + 3, class: 'ch-axis snr' });
    t.textContent = v;
    svg.appendChild(t);
  }
  // Nhãn thời gian
  const mins = Math.round((t1 - t0) / 60000);
  const tl = el('text', { x: L, y: H - 4, class: 'ch-axis' }); tl.textContent = mins >= 1 ? `-${mins} ph` : `-${Math.round((t1 - t0) / 1000)} s`;
  const tr = el('text', { x: W - R, y: H - 4, 'text-anchor': 'end', class: 'ch-axis' }); tr.textContent = 'bây giờ';
  svg.append(tl, tr);

  const line = (fy, key, cls) => {
    const d = pts.map((p, i) => `${i ? 'L' : 'M'}${x(p.t).toFixed(1)},${fy(p[key]).toFixed(1)}`).join('');
    svg.appendChild(el('path', { d, class: cls }));
  };
  line(yS, 'snr', 'ch-line snr');
  line(yR, 'rssi', 'ch-line rssi');
  const last = pts[pts.length - 1];
  svg.appendChild(el('circle', { cx: x(last.t), cy: yR(last.rssi), r: 3, class: 'ch-dot' }));
  box.appendChild(svg);
}

export function renderDetail() {
  const b = state.buoys.get(state.selected);
  $('detail-empty').hidden = !!b;
  $('detail-body').hidden = !b;
  if (!b) return;

  const st = statusOf(b);
  $('d-id').textContent = state.selected;
  const badge = $('d-badge');
  badge.textContent = STATUS_TEXT[st];
  badge.className = 'badge ' + st;

  if (b.hasPos) {
    $('d-lat').textContent = coordDMS ? toDMS(b.lat, true) : b.lat.toFixed(6);
    $('d-lon').textContent = coordDMS ? toDMS(b.lon, false) : b.lon.toFixed(6);
  } else {
    $('d-lat').textContent = NONE;
    $('d-lon').textContent = NONE;
  }
  const stn = state.station;
  const rel = !!(stn && stn.hasPos && b.hasPos);
  $('d-rel').hidden = !rel;
  if (rel) {
    $('d-rel-dist').textContent = fmtDist(distM(stn, b));
    $('d-rel-brg').textContent = Math.round(bearingDeg(stn, b)) + ' độ';
  }
  $('d-lat-stale').hidden = !(b.hasPos && b.fix !== 1);   // tọa độ là vị trí cuối, không phải bản tin hiện tại

  // MAVLink: HUD thay cho thẻ tư thế, không có RSSI/SNR nên ẩn khối tín hiệu LoRa
  const hud = renderHud(b);
  $('d-attitude').hidden = hud;
  const lora = b.rssi !== null;
  $('d-signal').hidden = !lora;
  $('d-link-mav').hidden = lora;
  if (!lora) {
    $('d-age-mav').textContent = fmtAgeSec(ageSec(b));
    $('d-raw').textContent = b.raw || NONE;
    return;
  }
  renderAttitude(b);

  const cls = rssiClass(b.rssi);
  $('d-rssi').textContent = b.rssi;
  $('d-rssi').className = 'big mono ' + cls;
  $('d-snr').textContent = b.snr.toFixed(1);
  $('d-age').textContent = fmtAgeSec(ageSec(b));
  $('d-age').className = 'big mono ' + (st === 'stale' ? 'warn' : '');

  const frac = Math.min(1, Math.max(0, (b.rssi + 130) / 90));
  const segs = $('d-rssi-bar').children;
  const on = Math.round(frac * segs.length);
  for (let i = 0; i < segs.length; i++) segs[i].className = i < on ? 'on ' + cls : '';
  $('d-raw').textContent = b.raw || NONE;
  drawChart();
}

export function initTelemetry({ onCenter, toast }) {
  const bar = $('d-rssi-bar');
  for (let i = 0; i < 12; i++) bar.appendChild(document.createElement('i'));

  $('d-fmt').addEventListener('click', () => {
    coordDMS = !coordDMS;
    $('d-fmt').textContent = coordDMS ? 'Độ thập phân' : 'Độ phút giây';
    renderDetail();
  });
  for (const btn of document.querySelectorAll('#d-window button')) {
    btn.addEventListener('click', () => {
      windowKey = btn.dataset.w;
      for (const b of document.querySelectorAll('#d-window button')) b.setAttribute('aria-pressed', String(b === btn));
      drawChart();
    });
  }
  $('btn-center').addEventListener('click', () => onCenter(state.selected));
  $('btn-copy').addEventListener('click', async () => {
    const b = state.buoys.get(state.selected);
    if (!b || !b.hasPos) { toast('Phao này chưa có tọa độ'); return; }
    const txt = b.lat.toFixed(6) + ', ' + b.lon.toFixed(6);
    try {
      await navigator.clipboard.writeText(txt);
    } catch {
      // Trang chạy trên http nên clipboard API có thể bị chặn: dùng cách dự phòng
      const ta = document.createElement('textarea');
      ta.value = txt; document.body.appendChild(ta); ta.select();
      try { document.execCommand('copy'); } catch { /* bỏ qua */ }
      ta.remove();
    }
    toast('Đã sao chép ' + txt);
  });
}
