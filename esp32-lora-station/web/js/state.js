// Trạng thái dùng chung và bus sự kiện nhỏ để các panel không gọi chéo nhau.

export const STALE_SEC = 30;      // quá ngưỡng này không có gói mới thì coi là mất tín hiệu
export const ATTITUDE_STALE_SEC = 12; // hai chu kỳ beacon 5 giây cộng dung sai
export const HISTORY_MAX = 300;   // số mẫu giữ trong bộ nhớ cho mỗi phao
export const CAPSIZE_SHOW_SEC = 60;   // a capsize alert labels the buoy "Bị lật" for this long
const SIGNAL_LATE_SEC = 12;       // two missed 5 s beacons: signal drops one level

export const state = {
  role: null,            // 'admin' | 'guest'
  email: null,
  connected: false,      // WebSocket tới máy chủ
  buoys: new Map(),      // id -> { lat, lon, hasPos, fix, rssi, snr, raw, lastSeen, marker }
  history: new Map(),    // id -> [{ t, rssi, snr, lat?, lon? }]
  selected: null,
  station: null,         // trạm bờ: { lat, lon, hasPos, fix, sats, hdop, lastSeen } (GPS của trạm, báo lên qua /api/station)
  lights: new Map(),     // id -> lệnh đèn của admin (mask 0..3); trạng thái thật nằm ở buoys.get(id).lights
  hold: new Map(),       // id -> lệnh DỪNG của admin (true = dừng); trạng thái thật ở buoys.get(id).hold
  eventBuoys: new Set(), // phao đã gửi sự kiện người/lật/xác nhận: không báo thêm theo số đếm `victims`
  capsizedAt: new Map(), // id -> time (ms) of the latest capsize alert
};

const handlers = new Map();
export const on = (name, fn) => { (handlers.get(name) || handlers.set(name, []).get(name)).push(fn); };
export const emit = (name, data) => { for (const fn of handlers.get(name) || []) fn(data); };

export const ageSec = b => (Date.now() - b.lastSeen) / 1000;
export const attitudeAgeSec = b => b.attitudeLastSeen ? (Date.now() - b.attitudeLastSeen) / 1000 : Infinity;
export const isStale = b => ageSec(b) > STALE_SEC;

export const STATION_STALE_SEC = 60;   // trạm báo vị trí mỗi 10 giây; quá ngưỡng này coi là mất tín hiệu

// 'none' (chưa từng báo) | 'stale' | 'nofix' | 'online'
export function stationStatus() {
  const s = state.station;
  if (!s) return 'none';
  if ((Date.now() - s.lastSeen) / 1000 > STATION_STALE_SEC) return 'stale';
  return s.fix === 1 ? 'online' : 'nofix';
}

// mode R: phao không nghe thấy trạm bờ quá 60 giây, tự chạy về trạm
export const isReturning = b => b.hasAttitude === true && b.headingMode === 'R';

// What the buoy is doing, in one word for the judges. Order matters: old data first, then danger.
export const STATUS = {
  lost:      { text: 'Mất tín hiệu',      tone: 'lost', icon: 'ph-cell-signal-slash' },
  capsized:  { text: 'Bị lật',            tone: 'bad',  icon: 'ph-warning' },
  stopped:   { text: 'ĐÃ DỪNG',           tone: 'bad',  icon: 'ph-hand-palm' },
  returning: { text: 'Đang quay về trạm', tone: 'warn', icon: 'ph-arrow-u-up-left' },
  patrol:    { text: 'Đang tuần tra',     tone: 'ok',   icon: 'ph-navigation-arrow' },
  idle:      { text: 'Chờ lộ trình',      tone: 'idle', icon: 'ph-pause-circle' },
};

export function buoyStatus(id, b) {
  if (isStale(b)) return 'lost';
  const cap = state.capsizedAt.get(id);
  if (cap && Date.now() - cap < CAPSIZE_SHOW_SEC * 1000) return 'capsized';
  if (b.hold === true) return 'stopped';
  if (b.nav) {   // Pixhawk over MAVLink: the autopilot mode says it
    if (b.nav.mode_name === 'RTL') return 'returning';
    if (b.nav.mode_name === 'HOLD') return 'stopped';
    return b.nav.armed ? 'patrol' : 'idle';
  }
  if (isReturning(b)) return 'returning';
  // The F407 reports mode A only while the Pi is driving it (a fresh NAV AUTO and heartbeat); without a
  // route, or with main.py not running, it stands still in mode S
  if (b.hasAttitude && b.headingMode === 'A') return 'patrol';
  return 'idle';
}

// Link quality from RSSI, SNR and packet age, shown as four bars and a word, never as numbers
export const SIGNAL = {
  strong: { text: 'Mạnh', bars: 4 },
  fair:   { text: 'Khá',  bars: 3 },
  weak:   { text: 'Yếu',  bars: 1 },
  lost:   { text: 'Mất',  bars: 0 },
};

export function signalOf(b) {
  const age = ageSec(b);
  if (age > STALE_SEC) return 'lost';
  const levels = ['weak', 'fair', 'strong'];
  let lvl;
  if (b.rssi === null) lvl = 2;   // MAVLink bridge has no RSSI: only freshness is known
  else if (b.rssi < -110 || b.snr < 0) lvl = 0;
  else if (b.rssi >= -90 && b.snr >= 5) lvl = 2;
  else lvl = 1;
  if (age > SIGNAL_LATE_SEC && lvl > 0) lvl--;
  return levels[lvl];
}

// Battery percent from the STM32 beacon or the MAVLink nav group, or null
export function batteryOf(b) {
  if (Number.isInteger(b.battery)) return b.battery;
  if (b.nav && b.nav.batt_pct >= 0) return b.nav.batt_pct;
  return null;
}

// Ground speed in m/s, or null
export function speedOf(b) {
  if (Number.isFinite(b.speed)) return b.speed;
  if (b.nav && Number.isFinite(b.nav.gs)) return b.nav.gs;
  return null;
}

export function pushHistory(id, sample) {
  let arr = state.history.get(id);
  if (!arr) state.history.set(id, arr = []);
  arr.push(sample);
  if (arr.length > HISTORY_MAX) arr.splice(0, arr.length - HISTORY_MAX);
}
