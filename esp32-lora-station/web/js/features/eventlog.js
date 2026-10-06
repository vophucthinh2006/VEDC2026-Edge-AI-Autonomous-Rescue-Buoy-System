// Nhật ký sự kiện, suy ra ở phía trình duyệt từ bản tin WebSocket và chuyển trạng thái phao.
import { state, on, statusOf, stationStatus } from '../state.js';
import { hms } from '../util/format.js';

const MAX_LINES = 300;
const ICONS = { info: 'ph-info', ok: 'ph-check-circle', warn: 'ph-warning', err: 'ph-x-circle' };

const $ = id => document.getElementById(id);
const lines = [];          // mới nhất ở cuối
let filter = 'all';        // 'all' | 'warn'
let unseenAlerts = 0;

export function isLogVisible() {
  return !$('log-view').hidden;
}

function render() {
  const list = $('log-list');
  const shown = filter === 'warn' ? lines.filter(l => l.level === 'warn' || l.level === 'err') : lines;
  list.replaceChildren();
  if (!shown.length) {
    const e = document.createElement('div');
    e.className = 'log-empty';
    e.textContent = filter === 'warn' ? 'Không có cảnh báo nào.' : 'Chưa có sự kiện nào.';
    list.appendChild(e);
    return;
  }
  const frag = document.createDocumentFragment();
  for (let i = shown.length - 1; i >= 0; i--) {   // mới nhất lên đầu
    const l = shown[i];
    const row = document.createElement('div');
    row.className = 'log-row ' + l.level;
    const t = document.createElement('time'); t.textContent = hms(l.time);
    const ic = document.createElement('i'); ic.className = 'ph ' + ICONS[l.level]; ic.setAttribute('aria-hidden', 'true');
    const msg = document.createElement('span'); msg.textContent = l.text;
    row.append(t, ic, msg);
    frag.appendChild(row);
  }
  list.appendChild(frag);
}

function updateBadge() {
  const b = $('tab-log-badge');
  b.hidden = unseenAlerts === 0;
  b.textContent = unseenAlerts > 99 ? '99+' : String(unseenAlerts);
}

export function log(level, text) {
  lines.push({ level, text, time: new Date() });
  if (lines.length > MAX_LINES) lines.splice(0, lines.length - MAX_LINES);
  if ((level === 'warn' || level === 'err') && !isLogVisible()) { unseenAlerts++; updateBadge(); }
  render();
}

export function clearUnseen() { unseenAlerts = 0; updateBadge(); }

export function initEventLog() {
  for (const btn of document.querySelectorAll('#log-filter button')) {
    btn.addEventListener('click', () => {
      filter = btn.dataset.f;
      for (const b of document.querySelectorAll('#log-filter button')) b.setAttribute('aria-pressed', String(b === btn));
      render();
    });
  }
  render();

  on('ws', ({ up }) => log(up ? 'ok' : 'err', up ? 'Đã kết nối máy chủ' : 'Mất kết nối máy chủ'));
  on('session', ({ role, email }) => log('info', role === 'admin' ? `Đăng nhập admin: ${email}` : 'Vào chế độ khách (chỉ xem)'));
  on('mission', ({ by, rev, first }) => {
    if (first) log('info', `Đã nạp lộ trình từ máy chủ (bản ${rev})`);
    else log('info', `Lộ trình được cập nhật bởi ${by || 'admin'} (bản ${rev})`);
  });

  // Bản tin phao: nhận gói, đổi trạng thái GPS
  on('buoy', ({ id, prev, b, snapshot }) => {
    if (snapshot) return;   // bản phát lại lúc mở trang không phải sự kiện mới
    if (!prev) log('ok', `Phát hiện phao mới: ${id}`);
    if (prev && prev.fix === 1 && b.fix !== 1) log('warn', `${id} mất GPS, giữ vị trí cuối`);
    if (prev && prev.fix !== 1 && b.fix === 1) log('ok', `${id} có lại GPS`);
    // Số đếm tăng so với gói trước: phao vừa báo thêm người. Phao khởi động lại thì đếm lại từ 0.
    if (prev && Number.isInteger(prev.victims) && b.victims > prev.victims) {
      log('err', `${id} PHÁT HIỆN NGƯỜI (lần ${b.victims}) tại ${b.hasPos ? `${b.lat.toFixed(5)}, ${b.lon.toFixed(5)}` : 'vị trí chưa rõ'}`);
    }
    if (b.link === 'mavlink') {
      // MAVLink đến 1-2 gói/giây: chỉ ghi khi arm/disarm hoặc đổi chế độ, không ghi từng gói
      const was = prev && prev.nav, now = b.nav;
      if (!now) return;
      if (!was) { log('info', `${id} qua MAVLink: ${now.mode_name}, ${now.armed ? 'ĐANG ARM' : 'chưa arm'}`); return; }
      if (was.armed !== now.armed) log(now.armed ? 'warn' : 'info', `${id} ${now.armed ? 'ĐÃ ARM, động cơ có thể chạy' : 'đã disarm'}`);
      if (was.mode_name !== now.mode_name) log('info', `${id} đổi chế độ ${was.mode_name} → ${now.mode_name}`);
      return;
    }
    const pos = b.fix === 1 ? `${b.lat.toFixed(5)}, ${b.lon.toFixed(5)}` : 'không có tọa độ';
    log('info', `${id} gói tin: ${pos}, RSSI ${b.rssi} dBm, SNR ${b.snr.toFixed(1)} dB`);
  });

  on('station', ({ prev, s, snapshot }) => {
    if (snapshot) return;
    if (!prev) log('ok', 'Trạm bờ đã báo vị trí lên máy chủ');
    if (prev && prev.fix === 1 && s.fix !== 1) log('warn', 'GPS trạm bờ mất fix, giữ vị trí cuối');
    if ((!prev || prev.fix !== 1) && s.fix === 1) log('ok', `GPS trạm bờ đã có fix (${s.sats ?? '?'} vệ tinh)`);
  });
  let lastStation = 'none';

  // Mỗi giây so sánh trạng thái để ghi lại mất/có lại tín hiệu
  const last = new Map();
  setInterval(() => {
    const ss = stationStatus();
    if (lastStation !== 'none' && lastStation !== 'stale' && ss === 'stale') log('err', 'Trạm bờ mất kết nối (quá 60 giây không báo)');
    else if (lastStation === 'stale' && ss !== 'stale') log('ok', 'Trạm bờ đã kết nối lại');
    lastStation = ss;
    for (const [id, b] of state.buoys) {
      const s = statusOf(b);
      const before = last.get(id);
      last.set(id, s);
      if (!before) continue;
      if (before !== 'stale' && s === 'stale') log('warn', `${id} mất tín hiệu (quá 30 giây chưa có gói mới)`);
      else if (before === 'stale' && s !== 'stale') log('ok', `${id} có lại tín hiệu`);
    }
  }, 1000);
}
