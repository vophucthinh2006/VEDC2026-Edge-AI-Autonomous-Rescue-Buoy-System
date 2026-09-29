// Định dạng hiển thị. Giá trị trống dùng "-" (không dùng dấu gạch dài).

export const NONE = '-';
export const pad2 = n => String(n).padStart(2, '0');

export const fmtDist = m => m < 1000 ? Math.round(m) + ' m' : (m / 1000).toFixed(2) + ' km';

export function fmtDur(s) {
  s = Math.round(s);
  const h = Math.floor(s / 3600), m = Math.floor(s % 3600 / 60), r = s % 60;
  return h ? `${h}:${pad2(m)}:${pad2(r)}` : `${m}:${pad2(r)}`;
}

// Tuổi gói tin ngắn gọn: "3 s", "2 ph", "1 g 05 ph"
export function fmtAgeSec(s) {
  s = Math.max(0, Math.floor(s));
  if (s < 60) return s + ' s';
  if (s < 3600) return Math.floor(s / 60) + ' ph';
  return Math.floor(s / 3600) + ' g ' + pad2(Math.floor(s % 3600 / 60)) + ' ph';
}

export const hms = d => `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`;
export const hmsUTC = d => `${pad2(d.getUTCHours())}:${pad2(d.getUTCMinutes())}:${pad2(d.getUTCSeconds())}`;

// Mức tín hiệu 1..4 và lớp màu theo RSSI (dBm)
export const sigLevel = r => r >= -85 ? 4 : r >= -100 ? 3 : r >= -112 ? 2 : 1;
export const rssiClass = r => r < -110 ? 'bad' : r < -95 ? 'warn' : 'ok';
