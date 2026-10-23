// Trạng thái dùng chung và bus sự kiện nhỏ để các panel không gọi chéo nhau.

export const STALE_SEC = 30;      // quá ngưỡng này không có gói mới thì coi là mất tín hiệu
export const ATTITUDE_STALE_SEC = 12; // hai chu kỳ beacon 5 giây cộng dung sai
export const HISTORY_MAX = 300;   // số mẫu giữ trong bộ nhớ cho mỗi phao
export const CAPSIZE_SHOW_SEC = 60;   // a capsize alert labels the buoy "Bị lật" for this long
// LoRa coverage of the shore station, drawn round it on the map. Estimated, not measured: RA-02 (SX1278) at both ends,
// 433 MHz, SF9 BW125 CR4/5, 17 dBm (STM32F4/App/app_config.h, esp32-lora-station/main/main.c); sensitivity -129 dBm,
// 2 dBi whips, 2 dB lost in cables: 148 dB of path loss at most. Over water or flat open ground, the boat's antenna
// 0.5 m and the station's 2 m up, the two-ray model loses 40 log10(d m): 133 dB, 15 dB kept for fading and the
// waves, at about 2 km; all 148 dB at about 5 km. Trees and buildings in the way shorten both a lot.
// Set both from a test on the water.
export const LORA_GOOD_M = 2000;   // solid circle: the link holds
export const LORA_WEAK_M = 5000;   // dashed circle: frames get lost between the two, none get through beyond

// Risk of a person the buoy found, index = the risk level of the event (0 safe .. 3 critical). Both pages.
export const RISKS = [
  { name: 'An toàn', color: '#22C55E' },
  { name: 'Trung bình', color: '#FACC15' },
  { name: 'Cao', color: '#F97316' },
  { name: 'Nguy kịch', color: '#EF4444' },
];
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
  diag: new Map(),       // id -> the latest diagnostics frame (STM32F4/App/diag_frame.h) + at (ms)
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

export const DIAG_STALE_SEC = 30;   // three missed 10 s diagnostics frames
// What the Pi does (Rasp_Pi/modules/mission.py Mission.code) from a diagnostics frame not older than
// DIAG_STALE_SEC, or null. The beacon's mode R only tells a lost shore; a low pack (B) shows only here.
export function diagMission(id) {
  const d = state.diag.get(id);
  return d && (Date.now() - d.at) / 1000 <= DIAG_STALE_SEC ? d.mission || null : null;
}
export const isGoingHome = id => ['R', 'B'].includes(diagMission(id));

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
  if (isReturning(b) || isGoingHome(id)) return 'returning';
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

// The fields of one buoy message (the Hub's broadcast of a beacon or a MAVLink frame) onto b, the buoy's
// record in state.buoys. Shared by the admin console and the guest page; side effects stay with them.
export function readBuoy(b, m) {
  b.fix = m.fix;
  // Cầu nối MAVLink không đi qua LoRa nên không có RSSI/SNR: để null
  b.link = m.link === 'mavlink' ? 'mavlink' : 'lora';
  b.rssi = Number.isFinite(m.rssi) ? m.rssi : null;
  b.snr = Number.isFinite(m.snr) ? m.snr : null;
  b.lastSeen = Date.now() - (Number(m.age_s) || 0) * 1000;
  // Nhóm điều hướng (ARMED, chế độ, tốc độ, ga, waypoint, pin, GPS) chỉ có ở MAVLink
  b.nav = m.nav && typeof m.nav === 'object' ? m.nav : null;
  // Số người phao đã báo phát hiện (bản STM32, qua LoRa); gói không có trường này thì giữ số cũ
  if (Number.isInteger(m.victims)) b.victims = m.victims;
  // Hướng đi và tốc độ theo GPS, % pin, trạng thái thật hai đèn, cờ dừng (bản STM32); gói không có thì để trống
  b.course = Number.isFinite(m.course) ? m.course : null;
  b.speed = Number.isFinite(m.speed) ? m.speed : null;
  b.battery = Number.isInteger(m.battery_pct) ? m.battery_pct : null;
  b.lights = Number.isInteger(m.lights) ? m.lights : null;
  b.hold = m.hold === 1 ? true : m.hold === 0 ? false : null;
  // Tiến độ nhận lộ trình gửi qua LoRa (bản STM32). Beacon LoRa không có w= là phao không giữ lộ trình
  // nào (F407 vừa khởi động lại); bản tin MAVLink không bao giờ có nhóm này nên giữ trạng thái cũ
  if (m.route && Number.isInteger(m.route.rev)) b.route = { rev: m.route.rev, have: m.route.have, pi: m.route.pi };
  else if (b.link === 'lora') b.route = null;
  const attitudeValid = [m.roll, m.pitch, m.yaw, m.target_yaw, m.imu_ok, m.calib, m.seq].every(Number.isFinite) && ['A', 'M', 'S', 'R'].includes(m.mode);
  if (attitudeValid) {
    b.hasAttitude = true;
    b.roll = Number(m.roll); b.pitch = Number(m.pitch); b.yaw = Number(m.yaw);
    b.targetYaw = Number(m.target_yaw) < 0 ? null : Number(m.target_yaw);
    b.headingMode = m.mode; b.imuOk = m.imu_ok === 1; b.calib = Number(m.calib); b.sequence = Number(m.seq);
    b.attitudeLastSeen = b.lastSeen;
  }
  // Chỉ ghi đè vị trí khi bản tin có tọa độ hợp lệ; NO_FIX giữ vị trí cũ
  if (Number.isFinite(m.lat) && Number.isFinite(m.lon)) { b.lat = m.lat; b.lon = m.lon; b.hasPos = true; }
  return b;
}

export function pushHistory(id, sample) {
  let arr = state.history.get(id);
  if (!arr) state.history.set(id, arr = []);
  arr.push(sample);
  if (arr.length > HISTORY_MAX) arr.splice(0, arr.length - HISTORY_MAX);
}
