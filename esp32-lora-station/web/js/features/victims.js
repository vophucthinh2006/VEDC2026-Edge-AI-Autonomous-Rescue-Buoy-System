// Người phao phát hiện (chấm màu theo mức nguy hiểm), khối "Người cần cứu" ở cột trái,
// và cảnh báo lật phao / xác nhận nhận đồ cứu hộ.
// Máy chủ đã bỏ bản phao gửi lặp và gộp báo cáo của mọi phao cách nhau dưới 6 m thành một người; phao báo đầu là
// chủ (docs/victim-claim.md). Viền chấm theo màu phao chủ; admin đánh dấu đã cứu, nhả, hoặc giao cho phao khác.
import { state, RISKS, LORA_GOOD_M, LORA_WEAK_M } from '../state.js';
import { hm, fmtAgo } from '../util/format.js';
import { distM } from '../util/geo.js';
import { log, logPast } from './timeline.js';
import { colorOf } from './fleet.js';

export { RISKS };
const LIST_MAX = 3;   // rows shown before "Xem thêm"

const $ = id => document.getElementById(id);
const dotLayer = L.layerGroup();
const alertLayer = L.layerGroup();
const dots = new Map();      // vid -> circleMarker
const victims = new Map();   // vid -> latest victim record
let map = null;
let toast = () => {};
let onAction = () => {};
let showAll = false;

const STATUS_TEXT = { claimed: 'đang được cứu', rescued: 'đã cứu', released: 'chờ phao nhận' };
const ownerOf = v => v.owner ?? (v.status ? null : v.buoy);
const seenBy = v => v.seen_by || [v.buoy];

const timeText = t => hm(new Date(t));

function row(dl, k, v) {
  const dt = document.createElement('dt'); dt.textContent = k;
  const dd = document.createElement('dd'); dd.textContent = v;
  dl.append(dt, dd);
}

function victimPopup(v) {
  const box = document.createElement('div');
  box.className = 'map-pop';
  const h = document.createElement('b');
  h.textContent = 'Người cần cứu: ' + RISKS[v.risk].name;
  h.style.color = RISKS[v.risk].color;
  const dl = document.createElement('dl');
  row(dl, 'Độ tin cậy', Math.round(v.conf * 100) + '%');
  row(dl, 'Thấy lần cuối', `${timeText(v.last_at)} (${fmtAgo(v.last_at)})`);
  if (v.status) row(dl, 'Trạng thái', STATUS_TEXT[v.status] + (ownerOf(v) ? ` · ${ownerOf(v)}` : ''));
  else row(dl, 'Phao báo', v.buoy);
  if (seenBy(v).length > 1) row(dl, 'Thấy bởi', seenBy(v).join(', '));
  box.append(h, dl);
  if (state.role === 'admin' && v.status && v.status !== 'rescued') box.appendChild(adminActions(v));
  return box;
}

// Admin: đã cứu, nhả cho phao khác nhận, giao cho một phao (nó nhận lộ trình tới chỗ người này)
function adminActions(v) {
  const bar = document.createElement('div');
  bar.className = 'v-actions';
  const button = (text, cls, fn) => {
    const b = document.createElement('button');
    b.type = 'button'; b.className = ('btn small ' + cls).trim(); b.textContent = text;
    b.addEventListener('click', fn);
    bar.appendChild(b);
  };
  button('Đã cứu', 'primary', () => onAction(v.vid, 'rescued'));
  if (v.status === 'claimed') button('Nhả', '', () => onAction(v.vid, 'release'));
  const others = [...state.buoys.keys()].filter(id => id !== ownerOf(v) && state.buoys.get(id).link !== 'mavlink').sort();
  if (others.length) {
    const pick = document.createElement('select');
    pick.setAttribute('aria-label', 'Phao nhận người này');
    for (const id of others) pick.add(new Option(id, id));
    bar.appendChild(pick);
    button('Giao', '', () => onAction(v.vid, 'assign', pick.value));
  }
  return bar;
}

function putVictim(v) {
  if (!RISKS[v.risk] || !Number.isFinite(v.lat) || !Number.isFinite(v.lon)) return;
  victims.set(v.vid, v);
  // Viền theo màu phao chủ; đã cứu: nhạt đi, viền trắng
  const owner = ownerOf(v), rescued = v.status === 'rescued';
  const style = { radius: 7 + v.risk, color: rescued || !owner ? '#F8FAFC' : colorOf(owner), weight: owner && !rescued ? 3 : 2,
    dashArray: v.status === 'released' ? '3 3' : null, fillColor: RISKS[v.risk].color, fillOpacity: rescued ? .35 : .95 };
  const m = dots.get(v.vid);
  if (m) m.setLatLng([v.lat, v.lon]).setStyle(style).setRadius(style.radius).setPopupContent(() => victimPopup(v));
  else dots.set(v.vid, L.circleMarker([v.lat, v.lon], style).bindPopup(() => victimPopup(v), { className: 'pop' }).addTo(dotLayer));
}

function alertPopup(a) {
  const box = document.createElement('div');
  box.className = 'map-pop';
  const h = document.createElement('b');
  h.textContent = 'Phao bị lật';
  h.style.color = 'var(--bad)';
  const dl = document.createElement('dl');
  row(dl, 'Phao', a.buoy);
  row(dl, 'Lúc', `${timeText(a.at)} (${fmtAgo(a.at)})`);
  box.append(h, dl);
  return box;
}

// Chỉ cảnh báo lật có vị trí mới có dấu trên bản đồ
function putAlertMarker(a) {
  if (a.type !== 'T' || !Number.isFinite(a.lat) || !Number.isFinite(a.lon)) return;
  const icon = L.divIcon({ className: '', iconSize: [28, 28], iconAnchor: [14, 14],
    html: '<div class="alert-pin"><i class="ph-fill ph-warning" aria-hidden="true"></i></div>' });
  L.marker([a.lat, a.lon], { icon, zIndexOffset: 900, title: `${a.buoy} bị lật` })
    .bindTooltip(`${a.buoy} bị lật lúc ${timeText(a.at)}`, { direction: 'top', offset: [0, -14], className: 'tip' })
    .bindPopup(() => alertPopup(a), { className: 'pop' })
    .addTo(alertLayer);
}

function noteCapsize(a) {
  if (a.type !== 'T' || !Number.isFinite(a.at)) return;
  if (!(state.capsizedAt.get(a.buoy) >= a.at)) state.capsizedAt.set(a.buoy, a.at);
}

// Not rescued first, then the highest risk, then the most recently seen
const ordered = () => [...victims.values()].sort((a, b) =>
  (a.status === 'rescued') - (b.status === 'rescued') || b.risk - a.risk || b.last_at - a.last_at);

// The "Người cần cứu" block in the left column
function renderVictims() {
  const list = $('victim-list');
  const all = ordered();
  const admin = state.role === 'admin';
  $('victims-count').hidden = !all.length;
  $('victims-count').textContent = all.length;
  $('victims-empty').hidden = all.length > 0;
  $('btn-clear-victims').hidden = !admin || !all.length;
  $('victims').classList.toggle('has-victims', all.some(v => v.risk > 0 && v.status !== 'rescued'));
  list.replaceChildren();
  const shown = showAll ? all : all.slice(0, LIST_MAX);
  for (const v of shown) {
    const li = document.createElement('li');
    const btn = document.createElement('button');
    btn.type = 'button'; btn.className = 'victim';
    btn.style.setProperty('--risk', RISKS[v.risk].color);
    btn.classList.toggle('rescued', v.status === 'rescued');
    const dot = document.createElement('span'); dot.className = 'risk-dot'; dot.setAttribute('aria-hidden', 'true');
    const main = document.createElement('span'); main.className = 'v-main';
    const name = document.createElement('b'); name.textContent = RISKS[v.risk].name;
    const conf = document.createElement('span'); conf.className = 'v-conf'; conf.textContent = `${Math.round(v.conf * 100)}%`;
    conf.title = 'Độ tin cậy';
    main.append(name, conf);
    const who = v.status === 'rescued' ? 'đã cứu' : v.status === 'released' ? 'chờ phao nhận' : ownerOf(v) || v.buoy;
    // Các phao khác cũng thấy người này (và đi tiếp): "+1" trong dòng, đủ chữ khi đọc / di chuột
    const n = seenBy(v).length, many = n > 1 ? ` +${n - 1}` : '';
    const sub = document.createElement('span'); sub.className = 'v-sub'; sub.textContent = `${who}${many} · ${fmtAgo(v.last_at)}`;
    if (n > 1) sub.title = `${n} phao thấy: ${seenBy(v).join(', ')}`;
    const go = document.createElement('i'); go.className = 'ph ph-arrow-right'; go.setAttribute('aria-hidden', 'true');
    btn.append(dot, main, sub, go);
    btn.setAttribute('aria-label', `${RISKS[v.risk].name}, tin cậy ${Math.round(v.conf * 100)}%, ${who}${n > 1 ? `, ${n} phao thấy` : ''}, ${fmtAgo(v.last_at)}. Xem trên bản đồ`);
    btn.addEventListener('click', () => flyTo(v.vid));
    li.appendChild(btn);
    list.appendChild(li);
  }
  const more = $('victims-more');
  more.hidden = all.length <= LIST_MAX;
  more.textContent = showAll ? 'Thu gọn' : `Xem thêm ${all.length - LIST_MAX} người`;
}

function flyTo(vid) {
  const v = victims.get(vid), m = dots.get(vid);
  if (!v || !m) return;
  map.flyTo([v.lat, v.lon], Math.max(map.getZoom(), 17), { duration: .6 });
  map.once('moveend', () => m.openPopup());
}

// People within `radiusM` of a point (address search)
export function victimsNear(p, radiusM) {
  return [...victims.values()].filter(v => distM(p, v) <= radiusM);
}

// Dòng nhật ký của một người hay một cảnh báo: [loại, chữ, lúc, id]. Cùng một hàm cho sự kiện vừa đến và cho ảnh
// chụp lúc mở trang, nên tải lại trang không thêm dòng trùng (timeline.js so theo id).
const foundRow = v => ['victim', `${v.buoy} phát hiện người, mức ${RISKS[v.risk].name.toLowerCase()}`, v.first_at, `v${v.vid}@${v.first_at}:found`];
function statusRow(v, before) {
  const id = `v${v.vid}@${v.first_at}:${v.status}@${v.status_at}`;
  if (v.status === 'rescued') return ['rescue', `Người của ${ownerOf(v) || v.buoy} đã được cứu`, v.status_at, id];
  if (v.status === 'released') return ['claim', `Người của ${before || v.buoy} được nhả: phao khác thấy sẽ nhận`, v.status_at, id];
  if (v.status === 'claimed' && v.owner !== v.buoy) return ['claim', `${v.owner} nhận người${v.status_by ? ' (admin giao)' : ''}`, v.status_at, id];
  return null;   // claimed by the first boat to see them: the "found" row says it
}
function alertRow(a) {
  const id = `a${a.type}:${a.buoy}@${a.at}`;
  if (a.type === 'T') return ['capsize', `${a.buoy} bị lật`, a.at, id];
  if (a.type === 'C') return ['rescue', `${a.buoy}: người gặp nạn đã nhận đồ cứu hộ`, a.at, id];
  if (a.type === 'E') return ['leader', `${a.replaces} mất liên lạc: ${a.buoy} lên dẫn đầu đội hình, nhận phần lộ trình còn lại`, a.at, id];
  if (a.type === 'V' && RISKS[a.risk]) return ['victim', `${a.buoy} phát hiện người, mức ${RISKS[a.risk].name.toLowerCase()} (phao chưa có GPS)`, a.at, id];
  return null;
}

// Ảnh chụp toàn bộ lúc mở trang, hoặc sau khi admin xoá chấm: vẽ lại, không báo; những gì máy chủ còn giữ vào lại
// nhật ký (không tính là mới), kể cả trên trình duyệt chưa từng mở trang này
export function onVictimsSnapshot(m) {
  dotLayer.clearLayers(); dots.clear(); victims.clear(); alertLayer.clearLayers();
  for (const v of m.victims || []) {
    putVictim(v);
    if (!RISKS[v.risk]) continue;
    logPast(...foundRow(v));
    const s = v.status && statusRow(v, null);
    if (s) logPast(...s);
  }
  for (const a of m.alerts || []) {
    putAlertMarker(a); noteCapsize(a);
    const r = alertRow(a);
    if (r) logPast(...r);
  }
  renderVictims();
}

export function onVictimMessage(m) {
  const v = m.victim;
  if (!v || !RISKS[v.risk]) return;
  state.eventBuoys.add(v.buoy);
  const before = victims.get(v.vid);
  const was = before && { status: before.status, owner: before.owner, seen: seenBy(before).length };
  putVictim(v);
  renderVictims();
  if (m.new || !before) {
    log(...foundRow(v));
    toast(`${v.buoy} phát hiện người (${RISKS[v.risk].name})`, 'bad');
    return;
  }
  // Nhận nạn nhân: một phao khác thấy cùng người (nó đi tiếp), người đã được cứu, được nhả, đổi chủ
  if (seenBy(v).length > was.seen) {
    const boat = seenBy(v).at(-1);
    log('claim', `${boat} cũng thấy người của ${ownerOf(v) || v.buoy}: đi tiếp, không dừng`, v.last_at, `v${v.vid}@${v.first_at}:seen:${boat}`);
  }
  if (v.status !== was.status || v.owner !== was.owner) {
    const s = statusRow(v, was.owner);
    if (s) log(...s);
    if (v.status === 'rescued') toast('Đã cứu một người', 'ok');
  }
}

export function onAlertMessage(m) {
  const a = m.alert;
  if (!a) return;
  state.eventBuoys.add(a.buoy);
  const r = alertRow(a);
  if (r) log(...r);
  if (a.type === 'T') {
    putAlertMarker(a);
    noteCapsize(a);
    toast(`${a.buoy} bị lật`, 'bad');
  } else if (a.type === 'C') {
    toast(`${a.buoy}: người gặp nạn đã nhận đồ cứu hộ`, 'ok');
  } else if (a.type === 'L') {
    // The shore station has not heard the buoy for 30 s (docs/fleet-control.md); the timeline logs signal lost itself
    toast(`${a.buoy} mất liên lạc ${a.silent_s ?? 30} giây`, 'bad');
  } else if (a.type === 'E') {
    toast(`${a.buoy} lên dẫn đầu đội hình thay ${a.replaces}`, 'bad');
  } else if (a.type === 'V' && RISKS[a.risk]) {
    toast(`${a.buoy} phát hiện người (chưa có vị trí)`, 'bad');
  }
}

// Collapsible map legend; collapsed on phones so it does not cover the map
function legendControl() {
  const ctl = L.control({ position: 'bottomleft' });
  ctl.onAdd = () => {
    const box = L.DomUtil.create('div', 'map-legend');
    L.DomEvent.disableClickPropagation(box);
    L.DomEvent.disableScrollPropagation(box);
    const title = document.createElement('button');
    title.type = 'button'; title.className = 't';
    title.innerHTML = '<span>Chú giải</span><i class="ph ph-caret-down" aria-hidden="true"></i>';
    const collapse = on => { box.classList.toggle('collapsed', on); title.setAttribute('aria-expanded', String(!on)); };
    title.addEventListener('click', () => collapse(!box.classList.contains('collapsed')));
    collapse(window.matchMedia('(max-width: 600px), (max-width: 820px) and (orientation: portrait)').matches);
    const body = document.createElement('div'); body.className = 'b';
    const item = (mark, text) => { const d = document.createElement('div'); d.append(mark, text); body.appendChild(d); };
    const el = (cls, color) => { const i = document.createElement('i'); i.className = cls; if (color) i.style.background = color; i.setAttribute('aria-hidden', 'true'); return i; };
    for (const r of RISKS) item(el('dot', r.color), r.name);
    item(el('lg-buoy'), 'Phao');
    item(el('lg-station'), 'Trạm bờ');
    const km = m => (m / 1000).toLocaleString('vi-VN', { maximumFractionDigits: 1 });
    item(el('lora'), `LoRa tốt (≤ ${km(LORA_GOOD_M)} km)`);
    item(el('lora weak'), `LoRa yếu (${km(LORA_GOOD_M)}–${km(LORA_WEAK_M)} km)`);
    item(el('ph-fill ph-warning warn-ic'), 'Phao lật');
    item(el('hdg'), 'Mũi tàu');
    item(el('cog'), 'Hướng đi');
    box.append(title, body);
    return box;
  };
  return ctl;
}

export function initVictims(leafletMap, { toast: t, onClear, onVictimAction }) {
  map = leafletMap;
  toast = t;
  onAction = (vid, action, id) => { map.closePopup(); onVictimAction(vid, action, id); };
  alertLayer.addTo(map);
  dotLayer.addTo(map);
  legendControl().addTo(map);
  $('btn-clear-victims').addEventListener('click', () => {
    if (confirm('Xoá mọi người cần cứu và cảnh báo lật trên bản đồ? Mọi người đang xem cùng bị xoá.')) onClear();
  });
  $('victims-more').addEventListener('click', () => { showAll = !showAll; renderVictims(); });
  renderVictims();
  setInterval(renderVictims, 30000);   // keep "x phút trước" fresh
}
