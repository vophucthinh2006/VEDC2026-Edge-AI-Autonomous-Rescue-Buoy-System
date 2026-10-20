// Tính toán địa lý thuần túy, không phụ thuộc giao diện.

export const R_EARTH = 6371000;
export const rad = d => d * Math.PI / 180;

// Khoảng cách (m) giữa 2 điểm
export function distM(a, b) {
  const dLat = rad(b.lat - a.lat), dLon = rad(b.lon - a.lon);
  const h = Math.sin(dLat / 2) ** 2 + Math.cos(rad(a.lat)) * Math.cos(rad(b.lat)) * Math.sin(dLon / 2) ** 2;
  return 2 * R_EARTH * Math.asin(Math.min(1, Math.sqrt(h)));
}

// Góc phương vị (độ, 0 = Bắc) từ a tới b
export function bearingDeg(a, b) {
  const y = Math.sin(rad(b.lon - a.lon)) * Math.cos(rad(b.lat));
  const x = Math.cos(rad(a.lat)) * Math.sin(rad(b.lat)) - Math.sin(rad(a.lat)) * Math.cos(rad(b.lat)) * Math.cos(rad(b.lon - a.lon));
  return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
}

// Bearing -> one of eight Vietnamese compass words ("Đông Bắc")
const COMPASS = ['Bắc', 'Đông Bắc', 'Đông', 'Đông Nam', 'Nam', 'Tây Nam', 'Tây', 'Tây Bắc'];
export const compassWord = deg => COMPASS[Math.round(((deg % 360) + 360) % 360 / 45) % 8];
