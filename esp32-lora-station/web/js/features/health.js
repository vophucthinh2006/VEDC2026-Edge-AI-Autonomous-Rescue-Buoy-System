// The selected buoy's health on the console card: IMU, roll and pitch against the 35 deg overturn
// limit, the bow against the heading the Pi asks for, the LoRa zone round the station, the route still
// ahead, a low pack, and the boat's own diagnostics frame (STM32F4/App/diag_frame.h): what the Pi is
// doing and why, why the F407 does or does not drive, the Pi's detector frames a second, the pack, the
// I2C/IMU recoveries.
import { state, attitudeAgeSec, ATTITUDE_STALE_SEC, LORA_GOOD_M, LORA_WEAK_M, DIAG_STALE_SEC, diagMission, isGoingHome } from '../state.js';
import { fmtDist, fmtAgo, fmtDuration } from '../util/format.js';
import { distM } from '../util/geo.js';

const $ = id => document.getElementById(id);
const TILT_LIMIT_DEG = 35;      // MAX_PITCH_DEG / MAX_ROLL_DEG of the F407
const TILT_WARN_DEG = 20;
const TILT_BAD_DEG = 30;
const LOW_BATTERY_PCT = 20;     // Rasp_Pi config return.battery_low_pct: below it the boat goes home by itself
const FPS_LOW = 1.0;            // the detector under this many frames a second: shown as a warning

// What the Pi does (Rasp_Pi/modules/mission.py Mission.code)
export const MISSIONS = {
  I: { text: 'Chờ lộ trình', tone: 'idle' },
  P: { text: 'Tuần tra theo lộ trình', tone: 'ok' },
  F: { text: 'Đang tới chỗ người cần cứu', tone: 'ok' },
  R: { text: 'Về trạm: mất liên lạc trạm bờ', tone: 'warn' },
  B: { text: 'Về trạm: pin yếu', tone: 'bad' },
  H: { text: 'Đang giữ vị trí ở trạm', tone: 'warn' },
};
// The reason of the Pi's NAV command (navigation.py, safety.py, main.py); tone bad: the Pi stopped the boat
const REASONS = {
  waypoint: ['Đi tới điểm lộ trình', 'ok'],
  bow_no_gps: ['Không có GPS: chạy theo hướng mũi', 'warn'],
  approach_human: ['Tiến tới người cần cứu', 'ok'],
  face_human: ['Quay mũi về phía người cần cứu', 'ok'],
  leaving_obstacle: ['Thoát khỏi vật cản', 'warn'],
  turning_away: ['Mũi sát vật cản: bò chậm, quay mũi ra', 'warn'],
  backing_off: ['Kẹt trước mũi: lùi ra bằng motor trước', 'warn'],
  human_standoff: ['Giữ khoảng cách với người cần cứu', 'ok'],
  arrived: ['Đã tới điểm', 'idle'],
  no_goal: ['Không có điểm đến', 'idle'],
  obstacle_too_close: ['Dừng: vật cản quá gần trên hướng đi', 'bad'],
  bow_blocked: ['Dừng: mũi kẹt sát vật cản, không quay ra được', 'bad'],
  overspeed: ['Thả ga: thuyền chạy nhanh hơn lệnh', 'warn'],
  pi_silent: ['Pi không gửi lệnh (Pi tắt, main.py dừng hoặc mất UART)', 'bad'],
  imu_stale: ['Dừng: mất dữ liệu IMU', 'bad'],
  gps_stale_or_no_fix: ['Dừng: mất GPS', 'bad'],
  lidar_stale: ['Dừng: mất LiDAR', 'bad'],
  attitude_fault: ['Dừng: phao nghiêng quá mức', 'bad'],
  stm32_fault_or_estop: ['Dừng: F407 báo lỗi', 'bad'],
  outside_geofence: ['Dừng: ra ngoài vùng cho phép', 'bad'],
  dashboard_hold: ['Dừng theo lệnh DỪNG PHAO', 'warn'],
};
// Why the F407 drives or not (App/control.h drive_block_t, arm_block_t)
const DRIVE = [['Đang chạy motor theo lệnh Pi', 'ok'], ['Không chạy: chưa arm', 'bad'], ['Không chạy: Pi không ra lệnh chạy', 'idle'],
  ['Không chạy: dữ liệu IMU cũ', 'bad'], ['Không chạy: lệnh DỪNG PHAO', 'warn']];
const ARM = ['', 'mất IMU', 'phao bị lật'];

function put(id, text, tone = '') {
  const el = $(id);
  el.textContent = text;
  if (tone) el.dataset.tone = tone; else delete el.dataset.tone;
}

const signedDeg = d => ((d + 540) % 360) - 180;
const tiltTone = v => Math.abs(v) >= TILT_BAD_DEG ? 'bad' : Math.abs(v) >= TILT_WARN_DEG ? 'warn' : 'ok';
function tiltText(v) {
  const near = Math.abs(v) >= TILT_BAD_DEG ? ` (gần ngưỡng lật ${TILT_LIMIT_DEG}°)` : '';
  return `${v >= 0 ? '+' : ''}${v.toFixed(1)}°${near}`;
}

function renderBody(id, b) {
  const has = b.hasAttitude === true;
  $('d-health').hidden = false;
  const stale = has && attitudeAgeSec(b) > ATTITUDE_STALE_SEC ? ' (số cũ)' : '';
  if (!has) {
    put('h-imu', 'Chưa có dữ liệu');
    for (const k of ['h-roll', 'h-pitch', 'h-head']) put(k, '-');
  } else {
    put('h-imu', b.imuOk ? `Hoạt động, hiệu chuẩn ${b.calib}/3${stale}` : `Mất IMU${stale}`,
      !b.imuOk ? 'bad' : b.calib < 2 ? 'warn' : 'ok');
    put('h-roll', tiltText(b.roll) + stale, tiltTone(b.roll));
    put('h-pitch', tiltText(b.pitch) + stale, tiltTone(b.pitch));
    if (b.targetYaw === null) put('h-head', `${b.yaw.toFixed(0)}°, Pi chưa yêu cầu hướng${stale}`, 'idle');
    else {
      const off = signedDeg(b.targetYaw - b.yaw);
      const side = Math.abs(off) < 1 ? '' : off > 0 ? ' (cần quay phải)' : ' (cần quay trái)';
      put('h-head', `${b.yaw.toFixed(0)}° / yêu cầu ${b.targetYaw.toFixed(0)}°: lệch ${Math.abs(off).toFixed(0)}°${side}${stale}`,
        Math.abs(off) <= 10 ? 'ok' : Math.abs(off) <= 30 ? 'warn' : 'bad');
    }
  }
  const s = state.station;
  if (!s || !s.hasPos || !b.hasPos) put('h-zone', !b.hasPos ? 'Phao chưa có vị trí' : 'Chưa biết vị trí trạm bờ');
  else {
    const d = distM(s, b);
    if (d <= LORA_GOOD_M) put('h-zone', `Vùng tốt (≤ ${LORA_GOOD_M / 1000} km), cách trạm ${fmtDist(d)}`, 'ok');
    else if (d <= LORA_WEAK_M) put('h-zone', `Vùng yếu (${LORA_GOOD_M / 1000}–${LORA_WEAK_M / 1000} km), cách trạm ${fmtDist(d)}`, 'warn');
    else put('h-zone', `Ngoài vùng liên lạc, cách trạm ${fmtDist(d)}`, 'bad');
  }
}

function renderLowBattery(id, b) {
  const el = $('c-lowbatt');
  const low = Number.isInteger(b.battery) && b.battery < LOW_BATTERY_PCT;
  el.hidden = !low;
  if (!low) return;
  const m = diagMission(id);
  el.textContent = `Pin yếu ${b.battery}%: ` + (m === 'B' ? 'phao đang tự quay về trạm.' : m === 'H' ? 'phao đã về tới trạm và giữ vị trí.'
    : 'phao sẽ tự quay về trạm.');
}

// Going home (mission R or B) the route is left behind: the way still ahead is the one to the station
function renderProgress(id, ahead, b) {
  const el = $('d-prog');
  const s = state.station;
  const home = isGoingHome(id) && s && s.hasPos && b.hasPos ? distM(s, b) : null;
  el.hidden = home === null && !ahead;
  if (el.hidden) return;
  const v = Number.isFinite(b.speed) ? b.speed : null;
  const eta = m => v !== null && v >= 0.1 ? `, tới nơi khoảng ${fmtDuration(m / v)}` : ', phao chưa di chuyển';
  if (home !== null) put('h-prog', `Đang về trạm: còn ${fmtDist(home)}${eta(home)}`, 'warn');
  else put('h-prog', `Đang tới điểm ${ahead.k}/${ahead.n}: còn ${fmtDist(ahead.remaining)} tới ${ahead.loop ? 'hết vòng' : 'điểm cuối'}${eta(ahead.remaining)}`, 'ok');
}

function renderDiag(id) {
  const d = state.diag.get(id);
  $('d-diag').hidden = !d;
  if (!d) return;
  const old = (Date.now() - d.at) / 1000 > DIAG_STALE_SEC;
  put('g-age', `cập nhật ${fmtAgo(d.at)}`, old ? 'lost' : '');
  const m = MISSIONS[d.mission];
  put('g-mission', m ? m.text + (d.wp_total ? `, điểm ${d.wp}/${d.wp_total}` : '') : 'Pi chưa báo', m ? m.tone : 'idle');
  // Going home the Pi steers to the station as to one more waypoint
  const r = d.reason === 'waypoint' && (d.mission === 'R' || d.mission === 'B') ? ['Đi về trạm', 'warn'] : REASONS[d.reason];
  put('g-reason', d.reason ? (r ? r[0] : d.reason) : 'Pi chưa gửi lệnh', r ? r[1] : 'idle');
  const [driveText, driveTone] = DRIVE[d.drive] || ['-', 'idle'];
  put('g-drive', driveText + (d.drive === 1 && ARM[d.arm] ? ` (${ARM[d.arm]})` : ''), driveTone);
  // The Pi's detector: frames a second over its last 10 s (Rasp_Pi/modules/ai_vision.py VisionWorker.fps)
  put('g-fps', Number.isFinite(d.fps) ? `${d.fps.toFixed(1)} khung/giây` : d.mission === 'I' ? 'Tắt (chưa có lộ trình)' : 'Pi chưa báo',
      !Number.isFinite(d.fps) ? 'idle' : d.fps < FPS_LOW ? 'warn' : 'ok');
  put('g-pack', Number.isFinite(d.volts) ? `${d.volts.toFixed(2)} V, ${d.amps.toFixed(2)} A` : 'Chưa đọc được INA226', 'idle');
  put('g-recov', `Bus I2C ${d.i2c} lần, IMU ${d.imu} lần`, d.i2c || d.imu ? 'warn' : 'ok');
}

// ahead: { k, n, remaining, loop } from app.js (the route the boat runs, with the waypoint it goes to), or null
export function renderHealth(id, b, ahead) {
  renderLowBattery(id, b);
  renderBody(id, b);
  renderProgress(id, ahead, b);
  renderDiag(id);
}
