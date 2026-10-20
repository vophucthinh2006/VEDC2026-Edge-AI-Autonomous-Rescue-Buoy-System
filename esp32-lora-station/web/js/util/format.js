// Định dạng hiển thị. Giá trị trống dùng "-" (không dùng dấu gạch dài).

export const NONE = '-';
export const pad2 = n => String(n).padStart(2, '0');

export const fmtDist = m => m < 1000 ? Math.round(m) + ' m' : (m / 1000).toFixed(m < 10000 ? 1 : 0) + ' km';

// "vừa xong", "3 phút trước", "2 giờ trước"
export function fmtAgo(ms) {
  const s = Math.max(0, Math.floor((Date.now() - ms) / 1000));
  if (s < 60) return 'vừa xong';
  if (s < 3600) return Math.floor(s / 60) + ' phút trước';
  if (s < 86400) return Math.floor(s / 3600) + ' giờ trước';
  return Math.floor(s / 86400) + ' ngày trước';
}

export const hm = d => `${pad2(d.getHours())}:${pad2(d.getMinutes())}`;
