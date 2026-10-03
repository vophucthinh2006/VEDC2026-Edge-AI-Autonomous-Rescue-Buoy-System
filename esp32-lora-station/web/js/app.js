// Điểm vào của dashboard: bản đồ, phao, lộ trình, đăng nhập, WebSocket.
// Các panel phụ (thanh trạng thái, viễn trắc, nhật ký, tiện ích bản đồ) nằm trong js/features/.
import { state, emit, statusOf, STATUS_TEXT, pushHistory } from './state.js';
import { distM, bearingDeg, surveyPath } from './util/geo.js';
import { fmtDist, fmtDur, fmtAgeSec, sigLevel } from './util/format.js';
import { initStatusbar, refreshStatusbar } from './features/statusbar.js';
import { initEventLog, clearUnseen } from './features/eventlog.js';
import { initTelemetry, renderDetail } from './features/telemetry.js';
import { initMapFx, updateTrail, fx, stopMeasuring } from './features/mapfx.js';

const $ = id => document.getElementById(id);
const DEFAULT_VIEW = [10.762622, 106.660172];
const MAX_WP = 300;
const ACCENT = '#22D3EE';
const ACCENT_INK = '#04171C';

// Tùy chọn giao diện: nhớ trên trình duyệt, không có thì dùng mặc định
const store = {
  get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : v; } catch { return d; } },
  set(k, v) { try { localStorage.setItem(k, v); } catch { /* bỏ qua */ } },
};
let darkMap = store.get('darkMap', '1') === '1';
let follow = store.get('follow', '0') === '1';
const isAdmin = () => state.role === 'admin';
// ?mock (admin giả) hoặc ?mock=guest (khách giả): chạy không cần máy chủ
const MOCK = (() => { const q = new URLSearchParams(location.search); return q.has('mock') ? (q.get('mock') || 'admin') : null; })();

function toast(msg) {
  const t = $('toast'); t.textContent = msg; t.classList.add('show');
  clearTimeout(toast.t); toast.t = setTimeout(() => t.classList.remove('show'), 2400);
}

// ================= BẢN ĐỒ =================
const map = L.map('map', { zoomControl: false }).setView(DEFAULT_VIEW, 13);
L.control.zoom({ position: 'bottomright' }).addTo(map);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '&copy; OpenStreetMap' }).addTo(map);
initMapFx(map, store);

// Bố cục đáp ứng: trên điện thoại/tablet dọc bảng điều khiển là ngăn kéo dưới bản đồ, có thể thu gọn.
// Điều kiện phải khớp media query trong css/layout.css.
const narrow = window.matchMedia('(max-width: 600px), (max-width: 820px) and (orientation: portrait)');
function setSheet(collapsed) {
  const side = $('side');
  side.classList.toggle('collapsed', collapsed && narrow.matches);
  const open = !side.classList.contains('collapsed');
  $('sheet-handle').setAttribute('aria-expanded', String(open));
  $('sheet-handle').textContent = open ? 'Thu gọn bảng điều khiển' : 'Mở bảng điều khiển';
}
$('sheet-handle').addEventListener('click', () => setSheet(!$('side').classList.contains('collapsed')));
narrow.addEventListener('change', () => setSheet(false));
new ResizeObserver(() => map.invalidateSize()).observe($('map-wrap'));

let mode = 'monitor';
let firstCentered = false;

function applyStyleBtn() {
  $('map').classList.toggle('dark', darkMap);
  $('btn-style').setAttribute('aria-pressed', String(darkMap));
}
function applyFollowBtn() { $('btn-follow').setAttribute('aria-pressed', String(follow)); }

// ================= GIÁM SÁT =================
function pinIcon(id, b) {
  const active = id === state.selected;
  // Mũi tên hướng mũi phao khi có yaw (giống biểu tượng phương tiện trên Mission Planner)
  const arrow = b.hasAttitude ? `<b class="hdg" style="transform:rotate(${b.yaw}deg)"></b>` : '';
  return L.divIcon({
    className: '', iconSize: [22, 22], iconAnchor: [11, 11],
    html: `<div class="pin ${statusOf(b)}${active ? ' active' : ''}">${arrow}<i></i></div>`,
  });
}

// "-87 dBm" cho LoRa, "MAVLink" cho cầu nối không có RSSI
const linkLabel = b => b.rssi === null ? 'MAVLink' : `${b.rssi} dBm`;

function updateMarker(id) {
  const b = state.buoys.get(id);
  if (!b || !b.hasPos) return;
  const label = `${id} ${linkLabel(b)}`;
  if (b.marker) {
    b.marker.setLatLng([b.lat, b.lon]).setIcon(pinIcon(id, b));
    b.marker.setTooltipContent(label);
  } else {
    b.marker = L.marker([b.lat, b.lon], { icon: pinIcon(id, b), zIndexOffset: 100 })
      .bindTooltip(label, { direction: 'top', offset: [0, -12], className: 'tip' })
      .addTo(map)
      .on('click', () => select(id));
  }
}

const STATUS_ORDER = { stale: 0, nofix: 1, online: 2 };   // cần chú ý lên đầu
function renderList() {
  const list = $('list');
  const ids = [...state.buoys.keys()].sort((a, b) =>
    STATUS_ORDER[statusOf(state.buoys.get(a))] - STATUS_ORDER[statusOf(state.buoys.get(b))] || a.localeCompare(b));
  $('count').textContent = ids.length;
  list.replaceChildren();
  if (!ids.length) {
    const e = document.createElement('div');
    e.id = 'list-empty'; e.textContent = 'Chưa nhận được phao nào qua LoRa.';
    list.appendChild(e);
    return;
  }
  for (const id of ids) {
    const b = state.buoys.get(id);
    const el = document.createElement('button');
    el.type = 'button';
    el.className = 'buoy ' + statusOf(b) + (id === state.selected ? ' active' : '');
    const dot = document.createElement('span'); dot.className = 'dot';
    const meta = document.createElement('div');
    const name = document.createElement('div'); name.className = 'name'; name.textContent = id;
    const sub = document.createElement('div'); sub.className = 'sub';
    sub.textContent = `${STATUS_TEXT[statusOf(b)]} | ${fmtAgeSec((Date.now() - b.lastSeen) / 1000)} | ${linkLabel(b)}`;
    meta.append(name, sub);
    let br;
    if (b.rssi === null) {
      br = document.createElement('span'); br.className = 'link-tag'; br.textContent = 'MAV';
    } else {
      br = document.createElement('div'); br.className = 'bars l' + sigLevel(b.rssi);
      br.append(...[1, 2, 3, 4].map(() => document.createElement('i')));
    }
    el.append(dot, meta, br);
    el.addEventListener('click', () => select(id));
    list.appendChild(el);
  }
}

function select(id) {
  state.selected = id;
  for (const k of state.buoys.keys()) updateMarker(k);
  renderList(); renderDetail(); renderRoute(); updateStation();
  const b = state.buoys.get(id);
  if (b && b.hasPos) map.panTo([b.lat, b.lon]);
}

// ---- Trạm bờ: marker đỏ, đường nối tới phao đang chọn ----
const STATION_RED = '#EF4444';
const stationLine = L.polyline([], { color: STATION_RED, weight: 1.5, opacity: .8, dashArray: '2 6', interactive: false }).addTo(map);
let stationMarker = null;

function updateStation() {
  const s = state.station;
  if (s && s.hasPos) {
    const icon = L.divIcon({
      className: '', iconSize: [30, 30], iconAnchor: [15, 15],
      html: '<div class="station-pin"><i class="ph-fill ph-broadcast" aria-hidden="true"></i></div>',
    });
    const label = 'TRẠM BỜ' + (s.fix === 1 ? '' : ' (vị trí cuối)');
    if (stationMarker) stationMarker.setLatLng([s.lat, s.lon]).setTooltipContent(label);
    else {
      stationMarker = L.marker([s.lat, s.lon], { icon, zIndexOffset: 1000 })
        .bindTooltip(label, { permanent: true, direction: 'bottom', offset: [0, 14], className: 'tip station-tip' })
        .addTo(map)
        .on('click', () => map.setView([s.lat, s.lon], Math.max(map.getZoom(), 16)));
    }
  }
  const b = state.buoys.get(state.selected);
  stationLine.setLatLngs(s && s.hasPos && b && b.hasPos ? [[s.lat, s.lon], [b.lat, b.lon]] : []);
}

function fitAll() {
  const pts = [...state.buoys.values()].filter(b => b.hasPos).map(b => [b.lat, b.lon]);
  if (state.station && state.station.hasPos) pts.push([state.station.lat, state.station.lon]);
  if (mode === 'mission') { wps.forEach(w => pts.push([w.lat, w.lon])); poly.forEach(w => pts.push([w.lat, w.lon])); }
  if (!pts.length) return;
  if (pts.length === 1) map.setView(pts[0], 16);
  else map.fitBounds(pts, { padding: [60, 60], maxZoom: 17 });
}

function centerOn(id) {
  const b = state.buoys.get(id);
  if (b && b.hasPos) map.setView([b.lat, b.lon], Math.max(map.getZoom(), 16));
  else toast('Phao này chưa có tọa độ');
}

// ================= LỘ TRÌNH =================
let wps = [];              // [{lat, lon}]
let selWp = -1;
let undoStack = [];        // ảnh chụp trạng thái để hoàn tác
const routeLine = L.polyline([], { color: ACCENT, weight: 4, opacity: .95 }).addTo(map);
const startLine = L.polyline([], { color: ACCENT, weight: 2, opacity: .7, dashArray: '6 8' }).addTo(map);
const wpLayer = L.layerGroup().addTo(map);
const legLayer = L.layerGroup().addTo(map);
let poly = [];             // đa giác vùng quét [{lat, lon}]
let drawing = false, draft = [];
const polyLayer = L.layerGroup().addTo(map);

const validPt = p => p && Number.isFinite(p.lat) && Number.isFinite(p.lon) && Math.abs(p.lat) <= 90 && Math.abs(p.lon) <= 180;
const speed = () => { const v = parseFloat($('m-speed').value); return Number.isFinite(v) && v > 0 ? v : 0; };
const spacing = () => { const v = parseFloat($('s-space').value); return Number.isFinite(v) ? Math.min(500, Math.max(1, v)) : 10; };
const angleVal = () => { const s = $('s-angle').value.trim(); const v = parseFloat(s); return s !== '' && Number.isFinite(v) ? ((v % 360) + 360) % 360 : null; };
function pushUndo() { undoStack.push(JSON.stringify(wps)); if (undoStack.length > 50) undoStack.shift(); }

// ---- Đồng bộ lộ trình với máy chủ: một bản chung, admin sửa, mọi người xem realtime ----
const cid = Math.random().toString(36).slice(2, 12);   // định danh tab này để bỏ qua bản phát lại của chính mình
let missionRev = 0;
let saveTimer = null;
let dragging = false;
let missionLoaded = false;

function applyMission(m) {
  wps = (m.waypoints || []).filter(validPt).slice(0, MAX_WP);
  poly = (m.poly || []).filter(validPt);
  $('s-space').value = m.spacing ?? 10;
  $('s-angle').value = m.angle ?? '';
  $('m-speed').value = m.speed ?? 1.5;
  selWp = -1; undoStack = [];
  missionRev = m.rev ?? 0;
  renderRoute();
}

async function pushMission() {
  if (MOCK) return;
  try {
    const res = await fetch('/api/mission', {
      method: 'PUT', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ waypoints: wps, poly, spacing: spacing(), angle: angleVal(), speed: speed() || 1.5, cid }),
    });
    if (res.ok) { missionRev = (await res.json()).rev; return; }
    toast(res.status === 401 || res.status === 403 ? 'Phiên hết hạn hoặc không có quyền, hãy đăng nhập lại' : 'Không lưu được lộ trình');
  } catch { toast('Mất kết nối, chưa lưu được lộ trình'); }
}

// Chỉ admin được lưu; gộp các thao tác liên tiếp thành một lần gửi
function saveMission() {
  if (!isAdmin()) return;
  clearTimeout(saveTimer);
  saveTimer = setTimeout(pushMission, 400);
}

function wpIcon(i, active) {
  if (wps.length > 40 && !active) {   // tuyến quét dài: chấm nhỏ, không đánh số
    return L.divIcon({ className: '', iconSize: [10, 10], iconAnchor: [5, 5],
      html: `<div style="width:10px;height:10px;border-radius:50%;background:${ACCENT};border:2px solid #fff"></div>` });
  }
  const s = active ? 32 : 26;
  return L.divIcon({
    className: '', iconSize: [s, s], iconAnchor: [s / 2, s / 2],
    html: `<div style="width:${s}px;height:${s}px;border-radius:50%;background:${ACCENT};color:${ACCENT_INK};border:2px solid #fff;
           box-shadow:${active ? '0 0 0 4px rgba(34,211,238,.45)' : '0 2px 6px rgba(0,0,0,.55)'};
           display:flex;align-items:center;justify-content:center;font:800 ${active ? 14 : 12}px system-ui">${i + 1}</div>`,
  });
}

// Vẽ vùng quét (đang vẽ dở hoặc đã chốt) và thanh điều khiển vẽ
function renderPoly() {
  polyLayer.clearLayers();
  $('drawbar').classList.toggle('show', mode === 'mission' && drawing);
  $('s-draw').querySelector('span').textContent = drawing ? 'Đang vẽ...' : (poly.length ? 'Vẽ lại vùng' : 'Vẽ vùng quét');
  $('s-draw').disabled = drawing;
  $('s-regen').disabled = drawing || poly.length < 3;
  $('s-clear').disabled = drawing || !poly.length;
  if (mode !== 'mission') return;
  const pts = drawing ? draft : poly;
  if (!pts.length) return;
  const ll = pts.map(p => [p.lat, p.lon]);
  const style = { color: ACCENT, weight: 2, interactive: false, fillColor: ACCENT, fillOpacity: drawing ? .08 : .14, dashArray: drawing ? '6 6' : null };
  (ll.length >= 3 ? L.polygon(ll, style) : L.polyline(ll, style)).addTo(polyLayer);
  if (drawing) {
    ll.forEach(c => L.circleMarker(c, { radius: 6, color: '#fff', weight: 2, fillColor: ACCENT, fillOpacity: 1, interactive: false }).addTo(polyLayer));
    $('draw-info').textContent = draft.length + ' điểm' + (draft.length >= 3 ? ', sẵn sàng' : ', cần ít nhất 3');
    $('draw-done').disabled = draft.length < 3;
    $('draw-undo').disabled = !draft.length;
  }
}

function generateSurvey() {
  if (!isAdmin() || poly.length < 3) return;
  const r = surveyPath(poly, spacing(), angleVal());
  if (!r.pts.length) { toast('Vùng quá hẹp so với khoảng cách tuyến'); return; }
  if (r.pts.length > MAX_WP) { toast(`Cần ${r.pts.length} điểm, vượt giới hạn ${MAX_WP}. Hãy tăng khoảng cách tuyến`); return; }
  pushUndo();
  wps = r.pts; selWp = -1;
  saveMission(); renderRoute();
  toast(`Đã tạo ${wps.length} điểm, hướng tuyến ${Math.round(r.angle)} độ`);
}

function selectedBuoyPos() {
  const b = state.buoys.get(state.selected);
  return b && b.hasPos ? { lat: b.lat, lon: b.lon } : null;
}

// Vẽ lại toàn bộ lộ trình trên bản đồ và bảng bên cạnh
function renderRoute() {
  wpLayer.clearLayers(); legLayer.clearLayers();
  renderPoly();
  const pts = wps.map(w => [w.lat, w.lon]);
  routeLine.setLatLngs(mode === 'mission' ? pts : []);
  const start = selectedBuoyPos();
  startLine.setLatLngs(mode === 'mission' && start && wps.length ? [[start.lat, start.lon], pts[0]] : []);
  if (mode !== 'mission') { renderMissionPanel(); return; }

  wps.forEach((w, i) => {
    const m = L.marker([w.lat, w.lon], { icon: wpIcon(i, i === selWp), draggable: isAdmin(), zIndexOffset: 500 }).addTo(wpLayer);
    m.on('click', () => { if (!fx.measuring) selectWp(i, false); });
    m.on('drag', e => {
      const p = e.target.getLatLng();
      w.lat = p.lat; w.lon = p.lng;
      routeLine.setLatLngs(wps.map(x => [x.lat, x.lon]));
    });
    m.on('dragstart', () => { dragging = true; pushUndo(); });
    m.on('dragend', () => { dragging = false; saveMission(); renderRoute(); });
    if (i > 0 && wps.length <= 40) {   // nhiều điểm (tuyến quét) thì bỏ nhãn cho đỡ rối
      const a = wps[i - 1], mid = [(a.lat + w.lat) / 2, (a.lon + w.lon) / 2];
      L.marker(mid, { interactive: false, icon: L.divIcon({
        className: '', html: `<span class="leg-label">${fmtDist(distM(a, w))}</span>`, iconSize: [0, 0] }) }).addTo(legLayer);
    }
  });
  renderMissionPanel();
}

// Cập nhật nhẹ khi phao di chuyển: không dựng lại marker để không làm gián đoạn thao tác kéo
function updateStart() {
  const s = selectedBuoyPos();
  startLine.setLatLngs(s && wps.length ? [[s.lat, s.lon], [wps[0].lat, wps[0].lon]] : []);
  renderMissionPanel();
}

function renderMissionPanel() {
  let total = 0;
  for (let i = 1; i < wps.length; i++) total += distM(wps[i - 1], wps[i]);
  $('m-count').textContent = wps.length;
  $('m-dist').textContent = wps.length > 1 ? fmtDist(total) : '-';
  $('m-time').textContent = wps.length > 1 && speed() ? fmtDur(total / speed()) : '-';
  $('m-undo').disabled = !undoStack.length;
  $('m-clear').disabled = !wps.length;
  $('m-export').disabled = !wps.length;
  $('m-hint').textContent = isAdmin()
    ? 'Chạm lên bản đồ để đặt điểm đến, kéo điểm để dời vị trí. Hoặc vẽ vùng quét để tự tạo tuyến.'
    : 'Bạn đang xem với tư cách khách: chỉ xem, không sửa được lộ trình.';

  const list = $('m-list');
  list.replaceChildren();
  if (!wps.length) {
    const e = document.createElement('div'); e.id = 'm-empty';
    const b = document.createElement('b'); b.textContent = 'Chưa có điểm đến nào';
    e.append(b, isAdmin() ? 'Chạm lên bản đồ để đặt điểm đầu tiên.' : 'Admin chưa tạo lộ trình.');
    list.appendChild(e);
    return;
  }
  const start = selectedBuoyPos();
  wps.forEach((w, i) => {
    const el = document.createElement('div');
    el.className = 'wp' + (i === selWp ? ' active' : '');
    el.tabIndex = 0;
    const n = document.createElement('span'); n.className = 'n'; n.textContent = i + 1;
    const meta = document.createElement('div');
    const pos = document.createElement('div'); pos.className = 'pos';
    pos.textContent = `${w.lat.toFixed(6)}, ${w.lon.toFixed(6)}`;
    const leg = document.createElement('div'); leg.className = 'leg';
    const prev = i > 0 ? wps[i - 1] : start;
    leg.textContent = prev
      ? `${i > 0 ? 'Từ điểm ' + i : 'Từ phao'}: ${fmtDist(distM(prev, w))} | ${Math.round(bearingDeg(prev, w))} độ`
      : 'Điểm xuất phát';
    meta.append(pos, leg);
    const del = document.createElement('button');
    del.type = 'button'; del.className = 'del'; del.setAttribute('aria-label', 'Xóa điểm ' + (i + 1));
    del.innerHTML = '<i class="ph ph-x" aria-hidden="true"></i>';
    del.addEventListener('click', ev => { ev.stopPropagation(); removeWp(i); });
    el.append(n, meta, del);
    el.addEventListener('click', () => selectWp(i, true));
    el.addEventListener('keydown', ev => { if (ev.key === 'Enter' || ev.key === ' ') { ev.preventDefault(); selectWp(i, true); } });
    list.appendChild(el);
  });
}

function selectWp(i, pan) {
  selWp = i;
  renderRoute();
  if (pan && wps[i]) map.panTo([wps[i].lat, wps[i].lon]);
}
function addWp(latlng) {
  if (!isAdmin()) return;
  if (wps.length >= MAX_WP) { toast('Tối đa ' + MAX_WP + ' điểm'); return; }
  pushUndo();
  wps.push({ lat: latlng.lat, lon: latlng.lng });
  selWp = wps.length - 1;
  saveMission(); renderRoute();
  const el = $('m-list'); el.scrollTop = el.scrollHeight;
}
function removeWp(i) {
  if (!isAdmin()) return;
  pushUndo();
  wps.splice(i, 1);
  selWp = -1;
  saveMission(); renderRoute();
}

map.on('click', e => {
  if (mode !== 'mission' || fx.measuring || !isAdmin()) return;
  if (drawing) { draft.push({ lat: e.latlng.lat, lon: e.latlng.lng }); renderPoly(); }
  else addWp(e.latlng);
});

function stopDrawing() { drawing = false; draft = []; renderPoly(); setSheet(false); }
$('s-draw').addEventListener('click', () => {
  if (!isAdmin()) return;
  if (wps.length && !confirm('Tạo tuyến quét mới sẽ thay thế lộ trình hiện tại. Tiếp tục vẽ?')) return;
  stopMeasuring();
  drawing = true; draft = []; selWp = -1; renderRoute();
  setSheet(true);   // trên điện thoại: thu ngăn điều khiển lại để có chỗ vẽ
});
$('draw-undo').addEventListener('click', () => { draft.pop(); renderPoly(); });
$('draw-cancel').addEventListener('click', stopDrawing);
$('draw-done').addEventListener('click', () => {
  if (draft.length < 3) return;
  poly = draft; stopDrawing(); saveMission(); generateSurvey();
});
$('s-regen').addEventListener('click', () => {
  if (wps.length && !confirm('Thay thế lộ trình hiện tại bằng tuyến quét mới?')) return;
  generateSurvey();
});
$('s-clear').addEventListener('click', () => { poly = []; saveMission(); renderPoly(); });
['s-space', 's-angle'].forEach(id => $(id).addEventListener('change', saveMission));

$('m-undo').addEventListener('click', () => {
  if (!isAdmin() || !undoStack.length) return;
  wps = JSON.parse(undoStack.pop()); selWp = -1;
  saveMission(); renderRoute();
});
$('m-clear').addEventListener('click', () => {
  if (!isAdmin() || !wps.length || !confirm('Xóa toàn bộ ' + wps.length + ' điểm đến?')) return;
  pushUndo(); wps = []; selWp = -1;
  saveMission(); renderRoute();
});
$('m-speed').addEventListener('input', () => { if (!isAdmin()) return; saveMission(); renderMissionPanel(); });

$('m-export').addEventListener('click', () => {
  const round = p => ({ lat: +p.lat.toFixed(7), lon: +p.lon.toFixed(7) });
  const data = {
    version: 1, speed_mps: speed() || 1.5,
    survey: poly.length >= 3 ? { polygon: poly.map(round), spacing_m: spacing(), angle_deg: angleVal() } : undefined,
    waypoints: wps.map(round),
  };
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' }));
  a.download = 'lo-trinh-phao.json';
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  toast('Đã xuất ' + wps.length + ' điểm');
});
$('m-import').addEventListener('click', () => $('m-file').click());
$('m-file').addEventListener('change', async ev => {
  const f = ev.target.files[0]; ev.target.value = '';
  if (!f || !isAdmin()) return;
  try {
    const d = JSON.parse(await f.text());
    const pts = Array.isArray(d) ? d : d.waypoints;
    if (!Array.isArray(pts) || !pts.length || !pts.every(validPt)) throw new Error('bad');
    pushUndo();
    wps = pts.slice(0, MAX_WP).map(p => ({ lat: +p.lat, lon: +p.lon }));
    if (Number.isFinite(d.speed_mps) && d.speed_mps > 0) $('m-speed').value = d.speed_mps;
    const sv = d.survey;
    poly = sv && Array.isArray(sv.polygon) && sv.polygon.length >= 3 && sv.polygon.every(validPt)
      ? sv.polygon.map(p => ({ lat: +p.lat, lon: +p.lon })) : [];
    if (sv && sv.spacing_m > 0) $('s-space').value = sv.spacing_m;
    $('s-angle').value = sv && sv.angle_deg !== null && Number.isFinite(sv.angle_deg) ? sv.angle_deg : '';
    selWp = -1; saveMission(); renderRoute(); fitAll();
    toast('Đã nhập ' + wps.length + ' điểm');
  } catch { toast('File không hợp lệ'); }
});

// ---- Chuyển chế độ ----
function setMode(m) {
  mode = m;
  if (m !== 'mission') { drawing = false; draft = []; }
  document.body.dataset.mode = m;
  for (const b of document.querySelectorAll('#mode button')) b.setAttribute('aria-selected', String(b.dataset.m === m));
  renderRoute();
  if (m === 'mission' && wps.length) fitAll();
}
for (const b of document.querySelectorAll('#mode button')) b.addEventListener('click', () => setMode(b.dataset.m));

// ---- Tab Phao / Nhật ký ----
function showTab(which) {
  const log = which === 'log';
  $('buoys-view').hidden = log;
  $('log-view').hidden = !log;
  $('tab-buoys').setAttribute('aria-selected', String(!log));
  $('tab-log').setAttribute('aria-selected', String(log));
  if (log) clearUnseen();
}
$('tab-buoys').addEventListener('click', () => showTab('buoys'));
$('tab-log').addEventListener('click', () => showTab('log'));

// ---- Nút công cụ bản đồ ----
$('btn-fit').addEventListener('click', fitAll);
$('btn-follow').addEventListener('click', () => { follow = !follow; store.set('follow', follow ? '1' : '0'); applyFollowBtn(); });
$('btn-style').addEventListener('click', () => { darkMap = !darkMap; store.set('darkMap', darkMap ? '1' : '0'); applyStyleBtn(); });

// ---- Tick mỗi giây: cập nhật "x giây trước" và trạng thái mất tín hiệu ----
setInterval(() => {
  if (!state.buoys.size) return;
  for (const k of state.buoys.keys()) updateMarker(k);
  renderList(); renderDetail();
}, 1000);

// ================= WEBSOCKET =================
function onMissionMessage(m) {
  if (m.cid === cid || !(m.rev > missionRev) || dragging) return;   // bản của chính mình / cũ / đang kéo điểm
  const first = !missionLoaded;
  missionLoaded = true;
  applyMission(m);
  emit('mission', { by: m.by, rev: m.rev, first });
}

function onBuoyMessage(m) {
  const id = String(m.id || 'PHAO-01');
  const existed = state.buoys.get(id);
  const prev = existed ? { fix: existed.fix, imuOk: existed.imuOk, nav: existed.nav } : null;
  const b = existed || { hasPos: false, marker: null };
  const snapshot = (Number(m.age_s) || 0) > 0;   // bản phát lại lúc mở trang, không phải gói mới
  b.fix = m.fix;
  // Cầu nối MAVLink không đi qua LoRa nên không có RSSI/SNR: để null, giao diện hiện "MAVLink"
  b.link = m.link === 'mavlink' ? 'mavlink' : 'lora';
  b.rssi = Number.isFinite(m.rssi) ? m.rssi : null;
  b.snr = Number.isFinite(m.snr) ? m.snr : null;
  b.raw = String(m.raw ?? '');
  b.lastSeen = Date.now() - (Number(m.age_s) || 0) * 1000;
  // Nhóm điều hướng (ARMED, chế độ, tốc độ, ga, waypoint, pin, GPS) chỉ có ở MAVLink
  b.nav = m.nav && typeof m.nav === 'object' ? m.nav : null;
  const attitudeValid = [m.roll, m.pitch, m.yaw, m.target_yaw, m.imu_ok, m.calib, m.seq].every(Number.isFinite) && ['A', 'M', 'S'].includes(m.mode);
  if (attitudeValid) {
    b.hasAttitude = true;
    b.roll = Number(m.roll); b.pitch = Number(m.pitch); b.yaw = Number(m.yaw);
    b.targetYaw = Number(m.target_yaw) < 0 ? null : Number(m.target_yaw);
    b.headingMode = m.mode; b.imuOk = m.imu_ok === 1; b.calib = Number(m.calib); b.sequence = Number(m.seq);
    b.attitudeLastSeen = b.lastSeen;
  }
  // Chỉ ghi đè vị trí khi bản tin có tọa độ hợp lệ; NO_FIX giữ vị trí cũ
  if (Number.isFinite(m.lat) && Number.isFinite(m.lon)) { b.lat = m.lat; b.lon = m.lon; b.hasPos = true; }
  state.buoys.set(id, b);
  if (!state.selected) state.selected = id;
  state.lastRx = Math.max(state.lastRx, b.lastSeen);

  // MAVLink đến 1-2 gói/giây: lấy mẫu lịch sử mỗi 2 giây để 300 mẫu phủ được 10 phút vệt đi
  const sampleDue = b.link !== 'mavlink' || !(b.lastSample > b.lastSeen - 2000);
  if (!snapshot && sampleDue) {
    b.lastSample = b.lastSeen;
    const s = { t: b.lastSeen };
    if (b.rssi !== null) { s.rssi = b.rssi; s.snr = b.snr; }
    if (m.fix === 1 && b.hasPos) { s.lat = b.lat; s.lon = b.lon; }
    pushHistory(id, s);
  }
  emit('buoy', { id, prev, b, snapshot });

  $('empty').hidden = true;
  updateMarker(id);
  updateTrail(id);
  if (b.hasPos && !firstCentered && !wps.length) { firstCentered = true; map.setView([b.lat, b.lon], 16); }
  else if (follow && id === state.selected && b.hasPos && !snapshot) map.panTo([b.lat, b.lon]);
  renderList();
  if (id === state.selected) renderDetail();
  if (mode === 'mission' && id === state.selected) updateStart();
  if (id === state.selected) updateStation();
  refreshStatusbar();
}

function onStationMessage(m) {
  const prev = state.station ? { fix: state.station.fix } : null;
  const s = state.station || { hasPos: false };
  const snapshot = (Number(m.age_s) || 0) > 0;
  s.fix = m.fix === 1 ? 1 : 0;
  s.sats = Number.isFinite(m.sats) ? m.sats : null;
  s.hdop = Number.isFinite(m.hdop) ? m.hdop : null;
  s.lastSeen = Date.now() - (Number(m.age_s) || 0) * 1000;
  // Mất fix vẫn giữ vị trí cuối (trạm đứng yên)
  if (Number.isFinite(m.lat) && Number.isFinite(m.lon)) { s.lat = m.lat; s.lon = m.lon; s.hasPos = true; }
  state.station = s;
  emit('station', { prev, s, snapshot });
  updateStation();
  if (s.hasPos && !firstCentered && !wps.length && !state.buoys.size) { firstCentered = true; map.setView([s.lat, s.lon], 15); }
  if (state.selected) renderDetail();
  refreshStatusbar();
}

function onHistoryMessage(m) {
  if (!Array.isArray(m.points)) return;
  // Mẫu MAVLink không có RSSI nhưng có tọa độ: vẫn giữ để vẽ vệt di chuyển
  const pts = m.points.filter(p => Number.isFinite(p.t) && (Number.isFinite(p.rssi) || Number.isFinite(p.lat)));
  state.history.set(String(m.id), pts.slice(-300));
  updateTrail(String(m.id));
  if (String(m.id) === state.selected) renderDetail();
}

function connect() {
  // Trên Cloudflare trang chạy qua https nên phải dùng wss; chạy cục bộ (http) thì ws
  const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.onopen = () => { state.connected = true; emit('ws', { up: true }); refreshStatusbar(); };
  ws.onclose = async () => {
    if (state.connected) emit('ws', { up: false });
    state.connected = false; refreshStatusbar();
    if (!MOCK) {   // phiên hết hạn thì quay lại màn hình đăng nhập thay vì thử lại mãi
      const me = await fetch('/api/me').then(r => r.json()).catch(() => null);
      if (me && !me.role) { location.reload(); return; }
    }
    setTimeout(connect, 2000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = ev => {
    let m;
    try { m = JSON.parse(ev.data); } catch { return; }
    if (m.type === 'mission') onMissionMessage(m);
    else if (m.type === 'history') onHistoryMessage(m);
    else if (m.type === 'station') onStationMessage(m);
    else onBuoyMessage(m);
  };
}

// ================= ĐĂNG NHẬP =================
const jsonPost = (path, body) => fetch(path, {
  method: 'POST', headers: { 'Content-Type': 'application/json' }, body: body === undefined ? undefined : JSON.stringify(body),
});

function enter(me) {
  state.role = me.role; state.email = me.email || null;
  document.body.dataset.role = state.role;
  $('login').hidden = true;
  $('role-name').textContent = isAdmin() ? 'Admin' : 'Khách';
  $('role-mail').textContent = state.email || '';
  $('role-chip').hidden = false;
  $('m-speed').readOnly = !isAdmin();
  emit('session', { role: state.role, email: state.email });
  renderRoute();
  connect();
}

function loginError(msg) { const e = $('login-err'); e.textContent = msg; e.hidden = !msg; }

async function onGoogleCredential(resp) {
  loginError('');
  try {
    const res = await jsonPost('/api/auth/google', { credential: resp.credential });
    const data = await res.json().catch(() => ({}));
    if (res.ok) enter(data); else loginError(data.error || 'Đăng nhập không thành công');
  } catch { loginError('Không kết nối được máy chủ'); }
}

function loadGoogle(clientId) {
  const s = document.createElement('script');
  s.src = 'https://accounts.google.com/gsi/client';
  s.async = true;
  s.onload = () => {
    google.accounts.id.initialize({ client_id: clientId, callback: onGoogleCredential });
    google.accounts.id.renderButton($('g-btn'), { theme: 'filled_black', size: 'large', text: 'signin_with', locale: 'vi', width: 280 });
  };
  s.onerror = () => loginError('Không tải được dịch vụ đăng nhập Google (cần internet)');
  document.head.appendChild(s);
}

async function showLogin() {
  $('login').hidden = false;
  const cfg = await fetch('/api/config').then(r => r.json()).catch(() => ({}));
  if (cfg.googleClientId) loadGoogle(cfg.googleClientId);
  else $('g-btn').textContent = 'Đăng nhập Google chưa được cấu hình';
}

$('btn-guest').addEventListener('click', async () => {
  loginError('');
  try {
    const res = await jsonPost('/api/auth/guest');
    if (res.ok) enter(await res.json()); else loginError('Không vào được chế độ khách');
  } catch { loginError('Không kết nối được máy chủ'); }
});
$('btn-logout').addEventListener('click', async () => {
  if (!MOCK) await jsonPost('/api/auth/logout').catch(() => { /* vẫn tải lại trang */ });
  location.reload();
});

// ================= KHỞI ĐỘNG =================
async function boot() {
  initStatusbar();
  initEventLog();
  initTelemetry({ onCenter: centerOn, toast });
  applyStyleBtn(); applyFollowBtn();
  setMode('monitor');
  renderDetail();

  // ?mock: chạy thử không cần backend; dev/mock.js không được deploy
  if (MOCK) {
    const s = document.createElement('script');
    s.src = 'dev/mock.js';
    s.onload = () => enter({ role: MOCK === 'guest' ? 'guest' : 'admin', email: 'mock@local' });
    document.head.appendChild(s);
    return;
  }
  const me = await fetch('/api/me').then(r => r.json()).catch(() => null);
  if (me && me.role) enter(me); else showLogin();
}
boot();
