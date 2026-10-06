import { num } from "./util.js";

const inRange = (value, min, max) => value !== null && value >= min && value <= max;
// Làm tròn 0.1° rồi quấn về [0, 360): 359.97 không được thành 360.0
const heading01 = v => (Math.round(v * 10) % 3600) / 10;
const round = (v, digits) => Math.round(v * 10 ** digits) / 10 ** digits;

// Nguồn bản tin: "lora" (ESP32 nhận gói LoRa, có RSSI/SNR) hoặc "mavlink"
// (cầu nối đọc thẳng autopilot ArduPilot/giả lập, không qua LoRa nên không có RSSI/SNR).
const LINKS = ["lora", "mavlink"];

// Nhóm điều hướng từ MAVLink. Không bắt buộc; hỏng thì chỉ bỏ nhóm này (nav_bad).
function sanitizeNav(n) {
  if (!n || typeof n !== "object") return null;
  const gs = num(n.gs), thr = num(n.thr), wpDist = num(n.wp_dist), wpSeq = num(n.wp_seq);
  const battV = num(n.batt_v), battPct = num(n.batt_pct), sats = num(n.sats), hdop = num(n.hdop);
  if (![0, 1].includes(n.armed) ||
      typeof n.mode_name !== "string" || !/^[A-Z_]{1,16}$/.test(n.mode_name) ||
      !inRange(gs, 0, 50) || !inRange(thr, -100, 100) ||
      !(wpDist === -1 || inRange(wpDist, 0, 100000)) ||
      !Number.isInteger(wpSeq) || !inRange(wpSeq, 0, 65535) ||
      !inRange(battV, 0, 100) || !(battPct === -1 || inRange(battPct, 0, 100)) ||
      !Number.isInteger(sats) || !inRange(sats, 0, 99) || !inRange(hdop, 0, 99.99)) {
    return null;
  }
  return {
    armed: n.armed,
    mode_name: n.mode_name,
    gs: round(gs, 2),
    thr: Math.round(thr),
    wp_dist: wpDist === -1 ? -1 : round(wpDist, 1),
    wp_seq: wpSeq,
    batt_v: round(battV, 2),
    batt_pct: Math.round(battPct),
    sats,
    hdop: round(hdop, 2),
  };
}

// Chuẩn hóa và kiểm tra bản tin từ trạm; trả về null nếu không hợp lệ.
// Gói GPS-only cũ vẫn hợp lệ. Nhóm attitude chỉ được giữ khi đầy đủ và đúng
// miền; hỏng thì chỉ bỏ nhóm đó (đánh dấu attitude_bad), vị trí và RSSI vẫn giữ.
export function sanitizeBuoy(m) {
  if (!m || typeof m !== "object") return null;
  const link = m.link === undefined ? "lora" : m.link;
  if (!LINKS.includes(link)) return null;
  const rssi = num(m.rssi), snr = num(m.snr);
  // LoRa bắt buộc có RSSI/SNR; MAVLink không có (không đi qua LoRa)
  if (link === "lora" && (rssi === null || snr === null)) return null;
  const id = typeof m.id === "string" && m.id.trim()
    ? m.id.replace(/[^\w .\-]/g, "").slice(0, 32) || "PHAO-01"
    : "PHAO-01";
  const out = { id, fix: m.fix === 1 ? 1 : 0 };
  if (link === "mavlink") out.link = "mavlink";
  if (rssi !== null && snr !== null) {
    out.rssi = Math.round(rssi);
    out.snr = Math.round(snr * 10) / 10;
  }
  out.raw = typeof m.raw === "string" ? m.raw.slice(0, 100) : "";
  if (out.fix === 1) {
    const lat = num(m.lat), lon = num(m.lon);
    if (lat === null || lon === null || Math.abs(lat) > 90 || Math.abs(lon) > 180) out.fix = 0;
    else { out.lat = lat; out.lon = lon; }
  }

  if (m.nav !== undefined) {
    const nav = sanitizeNav(m.nav);
    if (nav) out.nav = nav; else out.nav_bad = 1;
  }

  // Số người phao đã báo phát hiện từ lúc khởi động; firmware cũ không gửi trường này
  if (Number.isInteger(m.victims) && inRange(m.victims, 0, 255)) out.victims = m.victims;

  const attitudeKeys =["roll", "pitch", "yaw", "target_yaw", "mode", "imu_ok", "calib", "seq"];
  const hasAttitude = attitudeKeys.some(key => Object.hasOwn(m, key));
  if (!hasAttitude) return out;

  const roll = num(m.roll), pitch = num(m.pitch), yaw = num(m.yaw), target = num(m.target_yaw);
  const sequence = num(m.seq);
  if (!inRange(roll, -180, 180) || !inRange(pitch, -90, 90) ||
      !inRange(yaw, 0, 359.9999) ||
      !(target === -1 || inRange(target, 0, 359.9999)) ||
      !["A", "M", "S"].includes(m.mode) ||
      ![0, 1].includes(m.imu_ok) || !Number.isInteger(m.calib) || !inRange(m.calib, 0, 3) ||
      !Number.isInteger(sequence) || !inRange(sequence, 0, 0xFFFFFFFF)) {
    out.attitude_bad = 1;
    return out;
  }

  Object.assign(out, {
    roll: Math.round(roll * 10) / 10,
    pitch: Math.round(pitch * 10) / 10,
    yaw: heading01(yaw),
    target_yaw: target === -1 ? -1 : heading01(target),
    mode: m.mode,
    imu_ok: m.imu_ok,
    calib: m.calib,
    seq: sequence,
  });
  return out;
}
