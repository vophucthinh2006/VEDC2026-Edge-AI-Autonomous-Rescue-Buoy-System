// Trạng thái dùng chung và bus sự kiện nhỏ để các panel không gọi chéo nhau.

export const STALE_SEC = 30;      // quá ngưỡng này không có gói mới thì coi là mất tín hiệu
export const ATTITUDE_STALE_SEC = 12; // hai chu kỳ beacon 5 giây cộng dung sai
export const HISTORY_MAX = 300;   // số mẫu giữ trong bộ nhớ cho mỗi phao

export const state = {
  role: null,            // 'admin' | 'guest'
  email: null,
  connected: false,      // WebSocket tới máy chủ
  buoys: new Map(),      // id -> { lat, lon, hasPos, fix, rssi, snr, raw, lastSeen, marker }
  history: new Map(),    // id -> [{ t, rssi, snr, lat?, lon? }]
  selected: null,
  lastRx: 0,             // thời điểm nhận gói LoRa gần nhất (ms)
  station: null,         // trạm bờ: { lat, lon, hasPos, fix, sats, hdop, lastSeen } (GPS của trạm, báo lên qua /api/station)
};

const handlers = new Map();
export const on = (name, fn) => { (handlers.get(name) || handlers.set(name, []).get(name)).push(fn); };
export const emit = (name, data) => { for (const fn of handlers.get(name) || []) fn(data); };

export const ageSec = b => (Date.now() - b.lastSeen) / 1000;
export const attitudeAgeSec = b => b.attitudeLastSeen ? (Date.now() - b.attitudeLastSeen) / 1000 : Infinity;
export const isStale = b => ageSec(b) > STALE_SEC;

// 'online' | 'stale' | 'nofix'
export function statusOf(b) {
  if (isStale(b)) return 'stale';
  return b.fix === 1 ? 'online' : 'nofix';
}
export const STATION_STALE_SEC = 60;   // trạm báo vị trí mỗi 10 giây; quá ngưỡng này coi là mất tín hiệu

// 'none' (chưa từng báo) | 'stale' | 'nofix' | 'online'
export function stationStatus() {
  const s = state.station;
  if (!s) return 'none';
  if ((Date.now() - s.lastSeen) / 1000 > STATION_STALE_SEC) return 'stale';
  return s.fix === 1 ? 'online' : 'nofix';
}

export const STATUS_TEXT = { online: 'Trực tuyến', stale: 'Mất tín hiệu', nofix: 'Chưa có GPS' };

export function pushHistory(id, sample) {
  let arr = state.history.get(id);
  if (!arr) state.history.set(id, arr = []);
  arr.push(sample);
  if (arr.length > HISTORY_MAX) arr.splice(0, arr.length - HISTORY_MAX);
}
