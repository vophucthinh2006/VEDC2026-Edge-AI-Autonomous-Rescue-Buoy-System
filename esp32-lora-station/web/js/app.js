// Điểm vào của dashboard: bản đồ, phao, lộ trình, đăng nhập, WebSocket.
// Các panel phụ (thẻ phao, người cần cứu, dòng sự kiện, tìm địa chỉ, tiện ích bản đồ) nằm trong js/features/.
import { state, emit, pushHistory, readBuoy, isReturning, buoyStatus, STATUS, signalOf, SIGNAL, batteryOf, stationStatus,
  LORA_GOOD_M, LORA_WEAK_M } from './state.js';
import { distM } from './util/geo.js';
import { fmtDist } from './util/format.js';
import { initTimeline, clearUnseen, log } from './features/timeline.js';
import { initTelemetry, renderDetail } from './features/telemetry.js';
import { initMapFx, updateTrail } from './features/mapfx.js';
import { initVictims, onVictimsSnapshot, onVictimMessage, onAlertMessage } from './features/victims.js';
import { initLights, onLightsMessage, revertLights } from './features/lights.js';
import { initHold, onHoldMessage, revertHold, noteHold } from './features/hold.js';
import { initSearch, renderPlace, minimizeSearch } from './features/search.js';

const $ = id => document.getElementById(id);
const DEFAULT_VIEW = [10.762622, 106.660172];
const MAX_WP = 300;

// Tùy chọn giao diện: nhớ trên trình duyệt, không có thì dùng mặc định
const store = {
  get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : v; } catch { return d; } },
  set(k, v) { try { localStorage.setItem(k, v); } catch { /* bỏ qua */ } },
};
let follow = store.get('follow', '0') === '1';
const isAdmin = () => state.role === 'admin';
// ?mock (admin giả) hoặc ?mock=guest (khách giả): chạy không cần máy chủ
const MOCK = (() => { const q = new URLSearchParams(location.search); return q.has('mock') ? (q.get('mock') || 'admin') : null; })();

// kind: 'bad' (đỏ, cảnh báo người/lật), 'ok' (xanh) hoặc 'info' (màu thường); có kind thì hiện lâu hơn
function toast(msg, kind = '') {
  const t = $('toast'); t.textContent = msg; t.dataset.kind = kind; t.classList.add('show');
  clearTimeout(toast.t); toast.t = setTimeout(() => t.classList.remove('show'), kind ? 6000 : 2400);
}

// ================= BẢN ĐỒ =================
const map = L.map('map', { zoomControl: false }).setView(DEFAULT_VIEW, 13);
L.control.zoom({ position: 'bottomright' }).addTo(map);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '&copy; OpenStreetMap' }).addTo(map);
initMapFx(map);

// Bố cục đáp ứng: trên điện thoại/tablet dọc bảng điều khiển là ngăn kéo dưới bản đồ, có thể thu gọn.
// Điều kiện phải khớp media query trong css/layout.css.
const narrow = window.matchMedia('(max-width: 600px), (max-width: 820px) and (orientation: portrait)');
function setSheet(collapsed) {
  const side = $('side');
  side.classList.toggle('collapsed', collapsed && narrow.matches);
  const open = !side.classList.contains('collapsed');
  $('sheet-handle').setAttribute('aria-expanded', String(open));
  $('sheet-handle').querySelector('span').textContent = open ? 'Thu gọn' : 'Mở bảng điều khiển';
}
$('sheet-handle').addEventListener('click', () => setSheet(!$('side').classList.contains('collapsed')));
narrow.addEventListener('change', () => setSheet(false));
new ResizeObserver(() => map.invalidateSize()).observe($('map-wrap'));

let mode = 'monitor';
let firstCentered = false;

function applyFollowBtn() { $('btn-follow').setAttribute('aria-pressed', String(follow)); }

// ================= GIÁM SÁT =================
function pinIcon(id, b) {
  const active = id === state.selected;
  const tone = STATUS[buoyStatus(id, b)].tone;
  // Mũi tên hướng mũi phao khi có yaw (giống biểu tượng phương tiện trên Mission Planner)
  const arrow = b.hasAttitude ? `<b class="hdg" style="transform:rotate(${b.yaw}deg)"></b>` : '';
  // Mũi tên nét đứt: hướng đi thực theo GPS. Đứng yên thì hướng GPS vô nghĩa nên không vẽ
  const cog = b.course !== null && (b.speed === null || b.speed >= 0.2) ? `<b class="cog" style="transform:rotate(${b.course}deg)"></b>` : '';
  return L.divIcon({
    className: '', iconSize: [22, 22], iconAnchor: [11, 11],
    html: `<div class="pin" data-tone="${tone}"${active ? ' data-active' : ''}>${cog}${arrow}<i></i></div>`,
  });
}

function updateMarker(id) {
  const b = state.buoys.get(id);
  if (!b || !b.hasPos) return;
  const label = `${id} · ${STATUS[buoyStatus(id, b)].text}`;
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

const battIcon = p => p >= 75 ? 'ph-battery-full' : p >= 50 ? 'ph-battery-high' : p >= 25 ? 'ph-battery-medium' : p >= 10 ? 'ph-battery-low' : 'ph-battery-warning';
// Cần chú ý lên đầu: lật, dừng, mất tín hiệu, quay về, rồi đang chạy và đang chờ
const STATUS_ORDER = { capsized: 0, stopped: 1, lost: 2, returning: 3, patrol: 4, idle: 5 };
let listSig = '';

function renderList() {
  const rows = [...state.buoys.entries()].map(([id, b]) => ({ id, b, st: buoyStatus(id, b), sig: signalOf(b), batt: batteryOf(b) }))
    .sort((x, y) => STATUS_ORDER[x.st] - STATUS_ORDER[y.st] || x.id.localeCompare(y.id));
  $('count').textContent = rows.length;
  const sig = JSON.stringify([state.selected, rows.map(r => [r.id, r.st, r.sig, r.batt])]);
  if (sig === listSig) return;   // rebuilding every second would steal hover and keyboard focus
  listSig = sig;
  const list = $('list');
  list.replaceChildren();
  if (!rows.length) {
    const e = document.createElement('p');
    e.className = 'quiet list-empty'; e.textContent = 'Chưa nhận được phao nào qua LoRa.';
    list.appendChild(e);
    return;
  }
  for (const r of rows) {
    const st = STATUS[r.st];
    const el = document.createElement('button');
    el.type = 'button';
    el.className = 'buoy';
    if (r.id === state.selected) el.setAttribute('aria-current', 'true');
    const main = document.createElement('span'); main.className = 'b-main';
    const name = document.createElement('b'); name.textContent = r.id;
    const tag = document.createElement('span'); tag.className = 'tag'; tag.dataset.tone = st.tone; tag.textContent = st.text;
    main.append(name, tag);
    const bars = document.createElement('span'); bars.className = 'sig'; bars.dataset.level = r.sig;
    bars.title = 'Tín hiệu ' + SIGNAL[r.sig].text.toLowerCase();
    bars.append(...[1, 2, 3, 4].map(() => document.createElement('i')));
    const batt = document.createElement('span'); batt.className = 'b-batt';
    if (r.batt !== null) {
      batt.innerHTML = `<i class="ph ${battIcon(r.batt)}" aria-hidden="true"></i>`;
      batt.append(r.batt + '%');
      if (r.batt < 20) batt.dataset.low = '';
    }
    el.setAttribute('aria-label', `${r.id}: ${st.text}, tín hiệu ${SIGNAL[r.sig].text.toLowerCase()}${r.batt !== null ? `, pin ${r.batt}%` : ''}`);
    el.append(main, bars, batt);
    el.addEventListener('click', () => select(r.id));
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

// "Xem trên bản đồ" from the address search
function showBuoy(id) {
  const b = state.buoys.get(id);
  if (!b) return;
  if (mode !== 'monitor') setMode('monitor');
  select(id);
  if (b.hasPos) map.setView([b.lat, b.lon], Math.max(map.getZoom(), 16));
  if (narrow.matches) { minimizeSearch(); setSheet(false); }   // phone: show the buoy card under the map
}

// ---- Trạm bờ: marker đỏ, đường nét đứt tới phao đang chọn, trạng thái GPS trong nhãn ----
const STATION_RED = '#EF4444';
const stationLine = L.polyline([], { color: STATION_RED, weight: 1.5, opacity: .8, dashArray: '2 6', interactive: false }).addTo(map);
let stationMarker = null;
// LoRa coverage round the station (state.js LORA_GOOD_M, LORA_WEAK_M). Not clickable: taps go on to the map (route points).
const loraGood = L.circle([0, 0], { radius: LORA_GOOD_M, color: STATION_RED, weight: 2.5, opacity: .95,
  fillColor: STATION_RED, fillOpacity: .07, interactive: false });
const loraWeak = L.circle([0, 0], { radius: LORA_WEAK_M, color: STATION_RED, weight: 2.5, opacity: .9,
  dashArray: '10 8', fillColor: STATION_RED, fillOpacity: .04, interactive: false });

function stationLabel() {
  const ss = stationStatus();
  const s = state.station;
  const gps = ss === 'online' ? `GPS tốt${s.sats != null ? `, ${s.sats} vệ tinh` : ''}`
    : ss === 'nofix' ? 'Chưa có GPS, vị trí cuối' : 'Mất tín hiệu, vị trí cuối';
  return `<b>Trạm bờ</b><small>${gps}</small>`;
}

function updateStation() {
  const s = state.station;
  if (s && s.hasPos) {
    const icon = L.divIcon({
      className: '', iconSize: [30, 30], iconAnchor: [15, 15],
      html: '<div class="station-pin"><i class="ph-fill ph-broadcast" aria-hidden="true"></i></div>',
    });
    if (stationMarker) stationMarker.setLatLng([s.lat, s.lon]).setTooltipContent(stationLabel());
    else {
      stationMarker = L.marker([s.lat, s.lon], { icon, zIndexOffset: 1000 })
        .bindTooltip(stationLabel(), { permanent: true, direction: 'bottom', offset: [0, 14], className: 'tip station-tip' })
        .addTo(map)
        .on('click', () => map.setView([s.lat, s.lon], Math.max(map.getZoom(), 16)));
    }
    for (const c of [loraWeak, loraGood]) { c.setLatLng([s.lat, s.lon]); if (!map.hasLayer(c)) c.addTo(map); }
  }
  const b = state.buoys.get(state.selected);
  stationLine.setLatLngs(s && s.hasPos && b && b.hasPos ? [[s.lat, s.lon], [b.lat, b.lon]] : []);
}

function fitAll() {
  const pts = [...state.buoys.values()].filter(b => b.hasPos).map(b => [b.lat, b.lon]);
  if (state.station && state.station.hasPos) pts.push([state.station.lat, state.station.lon]);
  if (mode === 'mission') wps.forEach(w => pts.push([w.lat, w.lon]));
  if (!pts.length) return;
  if (pts.length === 1) map.setView(pts[0], 16);
  else map.fitBounds(pts, { padding: [60, 60], maxZoom: 17 });
}

// ================= LỘ TRÌNH (chỉ waypoint) =================
let wps = [];              // [{lat, lon}]
let selWp = -1;
let undoStack = [];        // ảnh chụp trạng thái để hoàn tác
const routeLine = L.polyline([], { color: '#22D3EE', weight: 4, opacity: .95 }).addTo(map);
const startLine = L.polyline([], { color: '#22D3EE', weight: 2, opacity: .7, dashArray: '6 8' }).addTo(map);
const wpLayer = L.layerGroup().addTo(map);

const validPt = p => p && Number.isFinite(p.lat) && Number.isFinite(p.lon) && Math.abs(p.lat) <= 90 && Math.abs(p.lon) <= 180;
function pushUndo() { undoStack.push(JSON.stringify(wps)); if (undoStack.length > 50) undoStack.shift(); }

// ---- Đồng bộ lộ trình với máy chủ: một bản chung, admin sửa, mọi người xem realtime ----
const cid = Math.random().toString(36).slice(2, 12);   // định danh tab này để bỏ qua bản phát lại của chính mình
let missionRev = 0;
let saveTimer = null;
let dragging = false;
let downlink = null;   // { rev, id, loop, n, mission_rev, by, at, delivered }: lần gửi xuống phao gần nhất
let downlinkLogged = 0;

function applyMission(m) {
  wps = (m.waypoints || []).filter(validPt).slice(0, MAX_WP);
  selWp = -1; undoStack = [];
  missionRev = m.rev ?? 0;
  renderRoute();
}

async function pushMission() {
  saveTimer = null;
  if (MOCK) { missionRev++; return; }
  try {
    const res = await fetch('/api/mission', {
      method: 'PUT', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ waypoints: wps, cid }),
    });
    if (res.ok) { missionRev = (await res.json()).rev; renderMissionPanel(); return; }
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
  if (wps.length > 40 && !active) {   // lộ trình dài: chấm nhỏ, không đánh số
    return L.divIcon({ className: '', iconSize: [10, 10], iconAnchor: [5, 5], html: '<div class="wp-dot"></div>' });
  }
  const s = active ? 32 : 26;
  return L.divIcon({
    className: '', iconSize: [s, s], iconAnchor: [s / 2, s / 2],
    html: `<div class="wp-pin"${active ? ' data-active' : ''}>${i + 1}</div>`,
  });
}

function selectedBuoyPos() {
  const b = state.buoys.get(state.selected);
  return b && b.hasPos ? { lat: b.lat, lon: b.lon } : null;
}

// Vẽ lại toàn bộ lộ trình trên bản đồ và bảng bên cạnh
function renderRoute() {
  wpLayer.clearLayers();
  const pts = wps.map(w => [w.lat, w.lon]);
  routeLine.setLatLngs(mode === 'mission' ? pts : []);
  const start = selectedBuoyPos();
  startLine.setLatLngs(mode === 'mission' && start && wps.length ? [[start.lat, start.lon], pts[0]] : []);
  if (mode !== 'mission') { renderMissionPanel(); return; }

  wps.forEach((w, i) => {
    const m = L.marker([w.lat, w.lon], { icon: wpIcon(i, i === selWp), draggable: isAdmin(), zIndexOffset: 500 }).addTo(wpLayer);
    m.on('click', () => selectWp(i, false));
    m.on('drag', e => {
      const p = e.target.getLatLng();
      w.lat = p.lat; w.lon = p.lng;
      routeLine.setLatLngs(wps.map(x => [x.lat, x.lon]));
    });
    m.on('dragstart', () => { dragging = true; pushUndo(); });
    m.on('dragend', () => { dragging = false; saveMission(); renderRoute(); });
  });
  renderMissionPanel();
}

// Cập nhật nhẹ khi phao di chuyển: không dựng lại marker để không làm gián đoạn thao tác kéo
function updateStart() {
  const s = selectedBuoyPos();
  startLine.setLatLngs(s && wps.length ? [[s.lat, s.lon], [wps[0].lat, wps[0].lon]] : []);
}

// One line of route progress for buoy `id`, or null (docs/lora-route-downlink.md)
// The route still ahead of a buoy: the waypoint it goes to (its diagnostics frame, from the Pi) and the
// points of the route it runs. Those are the map's when the last send was of this very mission; else unknown.
function routeAhead(id) {
  const d = state.diag.get(id), b = state.buoys.get(id);
  if (!d || !d.wp || !b || !b.hasPos || !downlink || downlink.id !== id || !downlink.n) return null;
  if (downlink.mission_rev !== missionRev || wps.length !== downlink.n || d.wp_total !== downlink.n || d.wp > wps.length) return null;
  let remaining = distM(b, wps[d.wp - 1]);
  for (let i = d.wp - 1; i + 1 < wps.length; i++) remaining += distM(wps[i], wps[i + 1]);
  return { k: d.wp, n: d.wp_total, remaining, loop: downlink.loop };
}

function onDiagMessage(m) {
  const id = String(m.id);
  const { type, id: _id, age_s, ...d } = m;
  const before = state.diag.get(id);
  state.diag.set(id, { ...d, at: Date.now() - (Number(age_s) || 0) * 1000 });
  // The Pi took the boat home for a low pack (mission B: below return.battery_low_pct for
  // battery_hold_s). Not on the beacon's reading itself: it sags under the motors for a second or two.
  if (before && before.mission !== 'B' && d.mission === 'B') {
    const pct = state.buoys.get(id)?.battery;
    const text = `${id}: pin yếu${Number.isInteger(pct) ? ` ${pct}%` : ''}, phao tự quay về trạm`;
    toast(text, 'bad');
    log('battery', text);
  }
  if (id === state.selected) renderDetail();
}

function routeProgress(id) {
  if (!downlink || !downlink.rev || downlink.id !== id) return null;
  const r = state.buoys.get(id)?.route;
  const n = downlink.n;
  const ours = r && r.rev === downlink.rev;
  // delivered: Worker thôi mời trạm gửi bản này, phao khởi động lại cũng không nhận lại
  if (!ours && downlink.delivered) return { tone: 'warn', text: 'Phao đã khởi động lại và bỏ lộ trình. Bấm Gửi tới phao để chạy lại.' };
  if (!ours) return { tone: 'pending', text: `Đã gửi lộ trình ${n} điểm, đang chờ phao nhận` };
  if (r.have < n) return { tone: 'pending', text: `Phao đang nhận lộ trình: ${r.have}/${n} điểm` };
  if (!r.pi) return { tone: 'pending', text: `Phao đã nhận đủ ${n} điểm, đang chuyển cho máy tính trên phao` };
  return { tone: 'ok', text: n ? `Phao đã nhận lộ trình ${n} điểm${downlink.loop ? ', đi vòng' : ''}` : 'Phao đã bỏ lộ trình và đứng yên' };
}

function renderSendStatus() {
  const r = downlink ? routeProgress(downlink.id) : null;
  const el = $('m-sendstat');
  el.hidden = !r;
  if (r) { el.dataset.tone = r.tone; el.textContent = `${downlink.id}: ${r.text}`; }
  const edited = downlink && downlink.rev && downlink.mission_rev !== missionRev && wps.length;
  $('m-send').querySelector('span').textContent = edited ? 'Gửi bản mới tới phao' : 'Gửi tới phao';
}

function renderMissionPanel() {
  let total = 0;
  for (let i = 1; i < wps.length; i++) total += distM(wps[i - 1], wps[i]);
  $('m-sum').textContent = wps.length ? `${wps.length} điểm · ${fmtDist(total)}` : '';
  $('m-undo').disabled = !undoStack.length;
  $('m-clear').disabled = !wps.length;
  renderSendStatus();
  $('m-hint').textContent = isAdmin()
    ? 'Chạm lên bản đồ để thêm điểm, kéo điểm để dời.'
    : 'Bạn đang xem với tư cách khách. Chỉ admin sửa được lộ trình.';

  const list = $('m-list');
  list.replaceChildren();
  if (!wps.length) {
    const e = document.createElement('li'); e.className = 'm-empty';
    const b = document.createElement('b'); b.textContent = 'Chưa có điểm nào';
    const p = document.createElement('span'); p.textContent = isAdmin() ? 'Chạm lên bản đồ để đặt điểm đầu tiên.' : 'Admin chưa tạo lộ trình.';
    e.append(b, p);
    list.appendChild(e);
    return;
  }
  const start = selectedBuoyPos();
  wps.forEach((w, i) => {
    const el = document.createElement('li');
    el.className = 'wp';
    if (i === selWp) el.setAttribute('aria-current', 'true');
    el.tabIndex = 0;
    const n = document.createElement('span'); n.className = 'n'; n.textContent = i + 1;
    const meta = document.createElement('div');
    const title = document.createElement('b'); title.textContent = 'Điểm ' + (i + 1);
    const leg = document.createElement('small');
    const prev = i > 0 ? wps[i - 1] : start;
    leg.textContent = prev ? `cách ${i > 0 ? 'điểm ' + i : 'phao'} ${fmtDist(distM(prev, w))}` : 'điểm xuất phát';
    meta.append(title, leg);
    el.append(n, meta);
    if (isAdmin()) {
      const del = document.createElement('button');
      del.type = 'button'; del.className = 'icon-btn del'; del.setAttribute('aria-label', 'Xoá điểm ' + (i + 1)); del.title = 'Xoá điểm';
      del.innerHTML = '<i class="ph ph-x" aria-hidden="true"></i>';
      del.addEventListener('click', ev => { ev.stopPropagation(); removeWp(i); });
      el.appendChild(del);
    }
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
  const el = $('m-list'); el.lastElementChild?.scrollIntoView({ block: 'nearest' });
}
function removeWp(i) {
  if (!isAdmin()) return;
  pushUndo();
  wps.splice(i, 1);
  selWp = -1;
  saveMission(); renderRoute();
}

map.on('click', e => {
  if (mode !== 'mission' || !isAdmin()) return;
  addWp(e.latlng);
});

$('m-undo').addEventListener('click', () => {
  if (!isAdmin() || !undoStack.length) return;
  wps = JSON.parse(undoStack.pop()); selWp = -1;
  saveMission(); renderRoute();
});
$('m-clear').addEventListener('click', () => {
  if (!isAdmin() || !wps.length || !confirm('Xoá toàn bộ ' + wps.length + ' điểm?')) return;
  pushUndo(); wps = []; selWp = -1;
  saveMission(); renderRoute();
});

// ---- Gửi lộ trình đang lưu xuống phao qua trạm bờ LoRa (docs/lora-route-downlink.md) ----
function onDownlinkMessage(m) {
  downlink = m;
  // A fresh send (not the replay on page load) goes into the timeline once per rev
  if (m.rev && m.rev !== downlinkLogged && Date.now() - m.at < 15000) {
    log('route', m.n ? `Đã gửi lộ trình ${m.n} điểm tới ${m.id}` : `Đã yêu cầu ${m.id} bỏ lộ trình`, m.at);
  }
  if (m.rev) downlinkLogged = Math.max(downlinkLogged, m.rev);
  renderSendStatus();
  if (m.id === state.selected) renderDetail();
}

$('m-send').addEventListener('click', async () => {
  if (!isAdmin()) return;
  const id = state.selected || 'PHAO-01';
  const clear = !wps.length;
  if (clear && !confirm(`Lộ trình đang trống. Gửi lộ trình rỗng để ${id} bỏ lộ trình cũ?`)) return;
  if (!clear && !confirm(`Gửi ${wps.length} điểm tới ${id} qua trạm bờ?`)) return;
  const loop = $('m-loop').checked;
  if (MOCK) { window.mockServer?.sendRoute(id, wps.length, loop, missionRev); return; }
  if (saveTimer) { clearTimeout(saveTimer); await pushMission(); }   // gửi đúng bản đang thấy
  try {
    const res = await jsonPost('/api/mission/send', { id, loop, clear });
    if (!res.ok) { toast(res.status === 401 || res.status === 403 ? 'Phiên hết hạn hoặc không có quyền' : 'Không gửi được lộ trình'); return; }
    onDownlinkMessage(await res.json());
    toast(clear ? `Đã yêu cầu ${id} bỏ lộ trình` : `Đã gửi ${wps.length} điểm, chờ trạm bờ phát xuống phao`);
  } catch { toast('Mất kết nối, chưa gửi được'); }
});

// ---- Chuyển chế độ ----
function setMode(m) {
  mode = m;
  document.body.dataset.mode = m;
  for (const b of document.querySelectorAll('#mode button')) b.setAttribute('aria-selected', String(b.dataset.m === m));
  renderRoute();
  if (m === 'mission' && wps.length) fitAll();
}
for (const b of document.querySelectorAll('#mode button')) b.addEventListener('click', () => setMode(b.dataset.m));

// ---- Tab Phao / Sự kiện ----
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
$('btn-follow').addEventListener('click', () => {
  follow = !follow; store.set('follow', follow ? '1' : '0'); applyFollowBtn();
  const b = state.buoys.get(state.selected);
  if (follow && b && b.hasPos) map.panTo([b.lat, b.lon]);
});

// ---- Tick mỗi giây: trạng thái suy ra từ tuổi gói (mất tín hiệu, hết nhãn "Bị lật") ----
setInterval(() => {
  for (const k of state.buoys.keys()) updateMarker(k);
  renderList(); renderDetail(); renderPlace();
  if (stationMarker) stationMarker.setTooltipContent(stationLabel());
}, 1000);

// ================= WEBSOCKET =================
function onMissionMessage(m) {
  if (m.cid === cid || !(m.rev > missionRev) || dragging) return;   // bản của chính mình / cũ / đang kéo điểm
  applyMission(m);
}

// Popup when a buoy starts being driven along its route, or stands still again. Its last status other
// than "lost" is remembered, so a buoy that drops out and comes back does not announce a start
const lastMove = new Map();
function noteMovement(id, b, snapshot) {
  const st = buoyStatus(id, b);
  if (st === 'lost') return;
  const before = lastMove.get(id);
  lastMove.set(id, st);
  if (snapshot || before === undefined || before === st) return;
  if (st === 'patrol' && before === 'idle') {
    toast(`${id} bắt đầu chạy lộ trình`, 'ok');
    log('run', `${id} bắt đầu chạy lộ trình`);
  } else if (st === 'patrol' && before === 'stopped') {
    toast(`${id} chạy tiếp lộ trình`, 'ok');   // hold.js already logs the resume
  } else if (st === 'idle' && before === 'patrol') {
    toast(`${id} đứng yên, chờ lộ trình mới`, 'info');
    log('idle', `${id} đứng yên, chờ lộ trình mới`);
  }
}

// Route fully handed to the Pi for the current downlink rev
const routeDone = (id, r) => !!(downlink && downlink.rev && downlink.id === id && r && r.rev === downlink.rev && r.have === downlink.n && r.pi === 1);

function onBuoyMessage(m) {
  const id = String(m.id || 'PHAO-01');
  const existed = state.buoys.get(id);
  const prev = existed ? { fix: existed.fix, nav: existed.nav, victims: existed.victims, returning: isReturning(existed), routeDone: routeDone(id, existed.route) } : null;
  const b = existed || { hasPos: false, marker: null };
  const snapshot = (Number(m.age_s) || 0) > 0;   // bản phát lại lúc mở trang, không phải gói mới
  readBuoy(b, m);
  state.buoys.set(id, b);
  noteHold(id);
  if (!state.selected) state.selected = id;
  if (!snapshot && prev && !prev.routeDone && routeDone(id, b.route)) {
    log('route', downlink.n ? `${id} đã nhận lộ trình ${downlink.n} điểm` : `${id} đã bỏ lộ trình`);
  }
  if (downlink && downlink.id === id) renderSendStatus();

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
  noteMovement(id, b, snapshot);
  if (!snapshot && prev && Number.isInteger(prev.victims) && b.victims > prev.victims && !state.eventBuoys.has(id)) toast(`${id} phát hiện người`, 'bad');

  $('empty').hidden = [...state.buoys.values()].some(x => x.hasPos);
  updateMarker(id);
  updateTrail(id);
  if (b.hasPos && !firstCentered && !wps.length) { firstCentered = true; map.setView([b.lat, b.lon], 16); }
  else if (follow && id === state.selected && b.hasPos && !snapshot) map.panTo([b.lat, b.lon]);
  renderList();
  if (id === state.selected) renderDetail();
  if (mode === 'mission' && id === state.selected) updateStart();
  if (id === state.selected) updateStation();
}

function onStationMessage(m) {
  const s = state.station || { hasPos: false };
  s.fix = m.fix === 1 ? 1 : 0;
  s.sats = Number.isFinite(m.sats) ? m.sats : null;
  s.hdop = Number.isFinite(m.hdop) ? m.hdop : null;
  s.lastSeen = Date.now() - (Number(m.age_s) || 0) * 1000;
  // Mất fix vẫn giữ vị trí cuối (trạm đứng yên)
  if (Number.isFinite(m.lat) && Number.isFinite(m.lon)) { s.lat = m.lat; s.lon = m.lon; s.hasPos = true; }
  state.station = s;
  updateStation();
  if (s.hasPos && !firstCentered && !wps.length && !state.buoys.size) { firstCentered = true; map.setView([s.lat, s.lon], 15); }
  if (state.selected) renderDetail();
}

function onHistoryMessage(m) {
  if (!Array.isArray(m.points)) return;
  // Mẫu MAVLink không có RSSI nhưng có tọa độ: vẫn giữ để vẽ vệt di chuyển
  const pts = m.points.filter(p => Number.isFinite(p.t) && (Number.isFinite(p.rssi) || Number.isFinite(p.lat)));
  state.history.set(String(m.id), pts.slice(-300));
  updateTrail(String(m.id));
}

function setConn(up) {
  state.connected = up;
  $('conn').dataset.up = String(up);
  $('conn-text').textContent = up ? 'Trực tuyến' : 'Mất kết nối';
}

function connect() {
  // Trên Cloudflare trang chạy qua https nên phải dùng wss; chạy cục bộ (http) thì ws
  const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.onopen = () => setConn(true);
  ws.onclose = async () => {
    setConn(false);
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
    else if (m.type === 'downlink') onDownlinkMessage(m);
    else if (m.type === 'history') onHistoryMessage(m);
    else if (m.type === 'station') onStationMessage(m);
    else if (m.type === 'victims') onVictimsSnapshot(m);
    else if (m.type === 'victim') onVictimMessage(m);
    else if (m.type === 'alert') { onAlertMessage(m); renderDetail(); renderList(); }
    else if (m.type === 'lights') onLightsMessage(m);
    else if (m.type === 'hold') onHoldMessage(m);
    else if (m.type === 'diag') onDiagMessage(m);
    else onBuoyMessage(m);
  };
}

// ================= LỆNH ADMIN: ĐÈN, DỪNG, XOÁ CHẤM =================
async function sendLights(id, lights, prev) {
  if (MOCK) { window.mockServer?.setLights(id, lights); return; }
  try {
    const res = await jsonPost('/api/lights', { id, lights });
    if (res.ok) { onLightsMessage(await res.json()); return; }
    toast(res.status === 401 || res.status === 403 ? 'Phiên hết hạn hoặc không có quyền' : 'Không gửi được lệnh đèn');
  } catch { toast('Mất kết nối, chưa gửi được lệnh đèn'); }
  revertLights(id, prev);
}

async function sendHold(id, hold, prev) {
  if (MOCK) { window.mockServer?.setHold(id, hold); return; }
  try {
    const res = await jsonPost('/api/hold', { id, hold });
    if (res.ok) { onHoldMessage(await res.json()); return; }
    toast(res.status === 401 || res.status === 403 ? 'Phiên hết hạn hoặc không có quyền' : 'Không gửi được lệnh dừng');
  } catch { toast('Mất kết nối, chưa gửi được lệnh dừng'); }
  revertHold(id, prev);
}

async function clearVictims() {
  if (MOCK) { window.mockServer?.clearVictims(); return; }
  try {
    const res = await jsonPost('/api/victims/clear');
    if (res.ok) onVictimsSnapshot(await res.json());
    else toast(res.status === 401 || res.status === 403 ? 'Phiên hết hạn hoặc không có quyền' : 'Không xoá được');
  } catch { toast('Mất kết nối, chưa xoá được'); }
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
  $('role-name').title = state.email || '';
  $('role-chip').hidden = false;
  renderRoute(); renderDetail();
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

// The Worker on this computer (wrangler dev) with the test sign-in (cloud/test/dev-idp.mjs): Google refuses the
// made-up client id of .dev.vars (invalid_client), so the admin signs in with a token of the test sign-in.
function showDevSignIn(url) {
  const box = $('g-btn');
  box.classList.add('login-dev');
  box.innerHTML = '<input id="dev-email" type="email" value="admin@example.com" aria-label="Email admin (ADMIN_EMAILS của .dev.vars)">'
    + '<button id="btn-dev" type="button" class="btn primary"><i class="ph ph-key" aria-hidden="true"></i>Đăng nhập admin (thử nghiệm)</button>';
  $('btn-dev').addEventListener('click', async () => {
    loginError('');
    let credential;
    try {
      credential = await fetch(url + '?email=' + encodeURIComponent($('dev-email').value.trim())).then(r => r.text());
    } catch { loginError('Không gọi được IdP thử (npm run dev:idp trong esp32-lora-station/cloud)'); return; }
    await onGoogleCredential({ credential });
  });
}

async function showLogin() {
  $('login').hidden = false;
  const cfg = await fetch('/api/config').then(r => r.json()).catch(() => ({}));
  if (cfg.devSignIn) showDevSignIn(cfg.devSignIn);
  else if (cfg.googleClientId) loadGoogle(cfg.googleClientId);
  else $('g-btn').textContent = 'Đăng nhập Google chưa được cấu hình';
}

// Guests have a page of their own (/, js/guest.js): the sign-in screen links there instead of letting them in here.
$('btn-logout').addEventListener('click', async () => {
  if (!MOCK) await jsonPost('/api/auth/logout').catch(() => { /* vẫn tải lại trang */ });
  location.reload();
});

// ================= KHỞI ĐỘNG =================
async function boot() {
  $('map').classList.add('dark');
  initTimeline();
  initTelemetry({ toast, routeLine: routeProgress, routeAhead });
  initVictims(map, { toast, onClear: clearVictims });
  initLights({ onSet: sendLights });
  initHold({ onSet: sendHold });
  initSearch(map, { onShowBuoy: showBuoy, onOpen: () => setSheet(true) });   // phone: give the results the whole map
  applyFollowBtn();
  setMode('monitor');
  renderList(); renderDetail();

  // ?mock: chạy thử không cần backend; dev/mock.js không được deploy
  if (MOCK) {
    const s = document.createElement('script');
    s.src = 'dev/mock.js';
    s.onload = () => enter({ role: MOCK === 'guest' ? 'guest' : 'admin', email: 'mock@local' });
    document.head.appendChild(s);
    return;
  }
  // The console is the admin's: a guest session (from the guest page) signs in here like no session at all
  const me = await fetch('/api/me').then(r => r.json()).catch(() => null);
  if (me && me.role === 'admin') enter(me); else showLogin();
}
boot();
