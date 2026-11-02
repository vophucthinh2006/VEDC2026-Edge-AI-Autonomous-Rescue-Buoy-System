// Kế hoạch cho đội tàu (docs/fleet-control.md): chia một vùng tìm kiếm cho nhiều phao, hoặc một đường quét cho
// phao dẫn đầu và chỗ của từng phao trong đội hình. Hàm thuần, không phụ thuộc Workers: Hub gọi khi admin bấm
// "Gửi cho cả đội", dashboard gọi qua /api/fleet/plan để xem trước, test chạy bằng node.
//
// Tọa độ tính trong mặt phẳng địa phương (mét, x đông, y bắc) quanh tâm vùng; đủ chính xác cho vài trăm mét.

const R = 6371000, RAD = Math.PI / 180;

function toLocal(origin) {
  const kx = R * RAD * Math.cos(origin.lat * RAD), ky = R * RAD;
  return {
    xy: p => ({ x: (p.lon - origin.lon) * kx, y: (p.lat - origin.lat) * ky }),
    ll: q => ({ lat: +(origin.lat + q.y / ky).toFixed(7), lon: +(origin.lon + q.x / kx).toFixed(7) }),
  };
}

const rotate = (p, a) => ({ x: p.x * Math.cos(a) - p.y * Math.sin(a), y: p.x * Math.sin(a) + p.y * Math.cos(a) });

// Hướng quét (độ la bàn) mặc định: theo cạnh dài nhất của vùng, để số lượt quay đầu ít nhất
export function sweepAngle(poly) {
  const o = toLocal(poly[0]);
  let best = 0, len = -1;
  for (let i = 0; i < poly.length; i++) {
    const a = o.xy(poly[i]), b = o.xy(poly[(i + 1) % poly.length]);
    const d = Math.hypot(b.x - a.x, b.y - a.y);
    if (d > len) { len = d; best = (Math.atan2(b.x - a.x, b.y - a.y) / RAD + 360) % 180; }
  }
  return best;
}

// Các đoạn [x0, x1] mà đường ngang y cắt đa giác (đã xoay), từ trái sang phải
function crossings(pts, y) {
  const xs = [];
  for (let i = 0; i < pts.length; i++) {
    const a = pts[i], b = pts[(i + 1) % pts.length];
    if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y)) xs.push(a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x));
  }
  return xs.sort((p, q) => p - q);
}

// Các làn quét của vùng: { y, x0, x1 } trong hệ đã xoay sao cho hướng quét là trục x, cách nhau spacing mét.
// Vùng lõm: làn đi từ chỗ cắt đầu tới chỗ cắt cuối.
function lanes(poly, spacing, angleDeg) {
  const centre = { lat: poly.reduce((s, p) => s + p.lat, 0) / poly.length, lon: poly.reduce((s, p) => s + p.lon, 0) / poly.length };
  const o = toLocal(centre);
  // Hướng quét la bàn angleDeg -> trục x: xoay mọi điểm một góc (angleDeg - 90) độ theo chiều kim đồng hồ
  const a = (angleDeg - 90) * RAD;
  const pts = poly.map(p => rotate(o.xy(p), a));
  const ymin = Math.min(...pts.map(p => p.y)), ymax = Math.max(...pts.map(p => p.y));
  const out = [];
  const n = Math.max(1, Math.round((ymax - ymin) / spacing));
  const step = (ymax - ymin) / n;
  for (let i = 0; i < n; i++) {
    const y = ymin + step * (i + 0.5);
    const xs = crossings(pts, y);
    if (xs.length >= 2) out.push({ y, x0: xs[0], x1: xs[xs.length - 1] });
  }
  const back = q => o.ll(rotate(q, -a));
  return { lanes: out, back };
}

// Đường zíc-zắc qua các làn, đổi chiều mỗi làn
function serpentine(ls, back) {
  const path = [];
  ls.forEach((l, i) => {
    const [from, to] = i % 2 === 0 ? [l.x0, l.x1] : [l.x1, l.x0];
    path.push(back({ x: from, y: l.y }), back({ x: to, y: l.y }));
  });
  return path;
}

function distM(a, b) {
  const dLat = (b.lat - a.lat) * RAD, dLon = (b.lon - a.lon) * RAD;
  const h = Math.sin(dLat / 2) ** 2 + Math.cos(a.lat * RAD) * Math.cos(b.lat * RAD) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(h));
}

// Chia vùng poly ([{lat, lon}], ít nhất 3 điểm) cho các phao boats ([{id, lat?, lon?}]): mỗi phao một dải liền
// gồm các làn liên tiếp, số làn chia đều nhất có thể, đi zíc-zắc. Dải gán cho phao gần nó nhất, và phao bắt đầu
// từ đầu dải gần mình, để các phao không chạy chéo qua nhau. Trả về [{ id, pts: [[lat, lon]] }].
export function planSectors(poly, boats, spacing = 10, angle = null) {
  if (!Array.isArray(poly) || poly.length < 3 || !boats.length) return [];
  const { lanes: ls, back } = lanes(poly, spacing, angle ?? sweepAngle(poly));
  const n = Math.min(boats.length, Math.max(ls.length, 1));
  const sectors = [];
  let k = 0;
  for (let i = 0; i < n; i++) {
    const take = Math.floor(ls.length / n) + (i < ls.length % n ? 1 : 0);
    sectors.push(serpentine(ls.slice(k, k + take), back));
    k += take;
  }
  // Gán tham lam: lần lượt cặp (phao, dải) có khoảng cách tới một đầu dải nhỏ nhất
  const free = boats.map(b => ({ ...b }));
  const out = [];
  const left = sectors.map((path, i) => ({ path, i }));
  while (left.length && free.length) {
    let best = null;
    for (const s of left) {
      if (!s.path.length) continue;
      for (const b of free) {
        const known = b.lat !== undefined && b.lon !== undefined;
        const dStart = known ? distM(b, s.path[0]) : 0, dEnd = known ? distM(b, s.path.at(-1)) : Infinity;
        const d = Math.min(dStart, dEnd);
        if (!best || d < best.d) best = { s, b, d, reverse: dEnd < dStart };
      }
    }
    if (!best) break;
    const path = best.reverse ? [...best.s.path].reverse() : best.s.path;
    out.push({ id: best.b.id, pts: path.map(p => [p.lat, p.lon]) });
    left.splice(left.indexOf(best.s), 1);
    free.splice(free.indexOf(best.b), 1);
  }
  return out;
}

// Chỗ của từng phao trong đội hình, so với phao dẫn đầu (mét, sang phải và lùi sau, theo hướng phao dẫn đầu):
//   line   hàng ngang, dẫn đầu ở giữa: các phao lần lượt bên phải, bên trái, cách nhau spacing
//   v      chữ V: như hàng ngang nhưng lùi dần về sau
//   column hàng dọc: nối đuôi nhau
export function formationOffsets(shape, followers, spacing = 10) {
  const out = {};
  followers.forEach((id, i) => {
    const k = i + 1, rank = Math.ceil(k / 2), side = k % 2 === 1 ? 1 : -1;
    if (shape === "column") out[id] = { right: 0, back: Math.round(k * spacing) };
    else if (shape === "v") out[id] = { right: Math.round(side * rank * spacing), back: Math.round(rank * spacing * 0.75) };
    else out[id] = { right: Math.round(side * rank * spacing), back: 0 };
  });
  return out;
}

function bearingDeg(a, b) {
  const y = Math.sin((b.lon - a.lon) * RAD) * Math.cos(b.lat * RAD);
  const x = Math.cos(a.lat * RAD) * Math.sin(b.lat * RAD) - Math.sin(a.lat * RAD) * Math.cos(b.lat * RAD) * Math.cos((b.lon - a.lon) * RAD);
  return (Math.atan2(y, x) / RAD + 360) % 360;
}

// Đường quét của phao dẫn đầu: các làn cách nhau bằng bề rộng cả hàng (số phao x spacing), vì hàng ngang
// đối xứng quanh phao dẫn đầu nên đổi chiều mỗi làn không làm lệch vệt quét. Hàng dọc quét như một phao.
// followers: tên phao, hoặc { id, lat, lon }: khi biết vị trí, mỗi chỗ trong đội hình (tính ở đầu lộ trình) về tay
// phao theo gần nó nhất, để các phao không phải chạy cắt ngang nhau.
export function planFormation(poly, leader, followers, shape = "line", spacing = 10, angle = null) {
  const width = shape === "column" ? spacing : spacing * (followers.length + 1);
  const [route] = planSectors(poly, [leader], width, angle);
  const pts = route ? route.pts : [];
  const ids = followers.map(f => (typeof f === "string" ? f : f.id));
  const places = Object.values(formationOffsets(shape, ids, spacing));
  const members = {};
  const known = followers.filter(f => typeof f === "object" && f.lat !== undefined);
  if (pts.length >= 2 && known.length === followers.length) {
    const start = { lat: pts[0][0], lon: pts[0][1] }, h = bearingDeg(start, { lat: pts[1][0], lon: pts[1][1] }) * RAD;
    const o = toLocal(start);
    const at = p => ({ x: p.right * Math.cos(h) - p.back * Math.sin(h), y: -p.right * Math.sin(h) - p.back * Math.cos(h) });
    // The way of placing them that makes the least way to go in all: every order tried (a fleet has a few boats)
    const slots = places.map(at), boats = followers.map(f => o.xy(f));
    let best = null;
    const tryOrder = (order, used) => {
      if (order.length === boats.length) {
        const cost = order.reduce((s, k, i) => s + Math.hypot(boats[i].x - slots[k].x, boats[i].y - slots[k].y), 0);
        if (!best || cost < best.cost) best = { cost, order: [...order] };
        return;
      }
      for (let k = 0; k < slots.length; k++) if (!used.has(k)) { used.add(k); order.push(k); tryOrder(order, used); order.pop(); used.delete(k); }
    };
    tryOrder([], new Set());
    followers.forEach((f, i) => { members[f.id] = places[best.order[i]]; });
  } else {
    ids.forEach((id, i) => { members[id] = places[i]; });
  }
  return { leader: leader.id, shape, spacing, route: pts, members };
}
