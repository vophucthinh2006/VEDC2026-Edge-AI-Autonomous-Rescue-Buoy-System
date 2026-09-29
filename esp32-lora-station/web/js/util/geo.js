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

// Độ thập phân -> "10°45'45.4"N"
export function toDMS(value, isLat) {
  const hemi = isLat ? (value >= 0 ? 'N' : 'S') : (value >= 0 ? 'E' : 'W');
  const a = Math.abs(value);
  const d = Math.floor(a);
  const mFloat = (a - d) * 60;
  const m = Math.floor(mFloat);
  const s = (mFloat - m) * 60;
  return `${d}°${String(m).padStart(2, '0')}'${s.toFixed(1).padStart(4, '0')}"${hemi}`;
}

// Tạo tuyến quét kiểu "cắt cỏ" (boustrophedon) bên trong đa giác.
// Chiếu phẳng quanh tâm vùng (đủ chính xác với vùng vài km), xoay hệ trục theo hướng tuyến.
export function surveyPath(poly, space, angle) {
  const lat0 = poly.reduce((s, p) => s + p.lat, 0) / poly.length;
  const lon0 = poly.reduce((s, p) => s + p.lon, 0) / poly.length;
  const kx = R_EARTH * Math.cos(rad(lat0)) * Math.PI / 180, ky = R_EARTH * Math.PI / 180;
  const xy = poly.map(p => ({ e: (p.lon - lon0) * kx, n: (p.lat - lat0) * ky }));
  if (angle === null) {                    // tự động: song song cạnh dài nhất
    let best = 0;
    for (let i = 0; i < xy.length; i++) {
      const a = xy[i], b = xy[(i + 1) % xy.length], d = Math.hypot(b.e - a.e, b.n - a.n);
      if (d > best) { best = d; angle = (Math.atan2(b.e - a.e, b.n - a.n) * 180 / Math.PI + 360) % 360; }
    }
  }
  const th = rad(angle), s = Math.sin(th), c = Math.cos(th);
  const uv = xy.map(q => ({ u: q.e * s + q.n * c, v: q.e * c - q.n * s }));
  let vmin = Infinity, vmax = -Infinity;
  uv.forEach(q => { vmin = Math.min(vmin, q.v); vmax = Math.max(vmax, q.v); });
  const out = [];
  let k = 0;
  for (let v = vmin + space / 2; v < vmax; v += space, k++) {
    const us = [];
    for (let i = 0; i < uv.length; i++) {
      const a = uv[i], b = uv[(i + 1) % uv.length];
      if ((a.v <= v && v < b.v) || (b.v <= v && v < a.v)) us.push(a.u + (v - a.v) * (b.u - a.u) / (b.v - a.v));
    }
    us.sort((x, y) => x - y);
    const segs = [];
    for (let i = 0; i + 1 < us.length; i += 2) segs.push([us[i], us[i + 1]]);
    if (k % 2) { segs.reverse(); segs.forEach(sg => sg.reverse()); }   // chiều đi đổi sau mỗi tuyến
    for (const [u1, u2] of segs) for (const u of [u1, u2]) {
      const e = u * s + v * c, n = u * c - v * s;
      out.push({ lat: lat0 + n / ky, lon: lon0 + e / kx });
    }
  }
  return { pts: out, angle };
}
