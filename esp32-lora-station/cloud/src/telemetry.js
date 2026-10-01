import { num } from "./util.js";

const inRange = (value, min, max) => value !== null && value >= min && value <= max;
// Làm tròn 0.1° rồi quấn về [0, 360): 359.97 không được thành 360.0
const heading01 = v => (Math.round(v * 10) % 3600) / 10;

// Chuẩn hóa và kiểm tra bản tin từ trạm; trả về null nếu không hợp lệ.
// Gói GPS-only cũ vẫn hợp lệ. Nhóm attitude chỉ được giữ khi đầy đủ và đúng
// miền; hỏng thì chỉ bỏ nhóm đó (đánh dấu attitude_bad), vị trí và RSSI vẫn giữ.
export function sanitizeBuoy(m) {
  if (!m || typeof m !== "object") return null;
  const rssi = num(m.rssi), snr = num(m.snr);
  if (rssi === null || snr === null) return null;
  const id = typeof m.id === "string" && m.id.trim()
    ? m.id.replace(/[^\w .\-]/g, "").slice(0, 32) || "PHAO-01"
    : "PHAO-01";
  const out = {
    id,
    fix: m.fix === 1 ? 1 : 0,
    rssi: Math.round(rssi),
    snr: Math.round(snr * 10) / 10,
    raw: typeof m.raw === "string" ? m.raw.slice(0, 100) : "",
  };
  if (out.fix === 1) {
    const lat = num(m.lat), lon = num(m.lon);
    if (lat === null || lon === null || Math.abs(lat) > 90 || Math.abs(lon) > 180) out.fix = 0;
    else { out.lat = lat; out.lon = lon; }
  }

  const attitudeKeys = ["roll", "pitch", "yaw", "target_yaw", "mode", "imu_ok", "calib", "seq"];
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
