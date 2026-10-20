// Người phao phát hiện (chấm màu theo mức nguy hiểm), khối "Người cần cứu" ở cột trái,
// và cảnh báo lật phao / xác nhận nhận đồ cứu hộ.
// Máy chủ đã bỏ bản phao gửi lặp và gộp phát hiện gần nhau (5 m), ở đây chỉ vẽ và báo.
import { state } from '../state.js';
import { hm, fmtAgo } from '../util/format.js';
import { distM } from '../util/geo.js';
import { log } from './timeline.js';

export const RISKS = [
  { name: 'An toàn', color: '#22C55E' },
  { name: 'Trung bình', color: '#FACC15' },
  { name: 'Cao', color: '#F97316' },
  { name: 'Nguy kịch', color: '#EF4444' },
];
const LIST_MAX = 3;   // rows shown before "Xem thêm"

const $ = id => document.getElementById(id);
const dotLayer = L.layerGroup();
const alertLayer = L.layerGroup();
const dots = new Map();      // vid -> circleMarker
const victims = new Map();   // vid -> latest victim record
let map = null;
let toast = () => {};
let showAll = false;

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
  row(dl, 'Phao báo', v.buoy);
  box.append(h, dl);
  return box;
}

function putVictim(v) {
  if (!RISKS[v.risk] || !Number.isFinite(v.lat) || !Number.isFinite(v.lon)) return;
  victims.set(v.vid, v);
  const style = { radius: 7 + v.risk, color: '#F8FAFC', weight: 2, fillColor: RISKS[v.risk].color, fillOpacity: .95 };
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
  L.marker([a.lat, a.lon], { icon, zIndexOffset: 900 })
    .bindTooltip(`${a.buoy} bị lật lúc ${timeText(a.at)}`, { direction: 'top', offset: [0, -14], className: 'tip' })
    .bindPopup(() => alertPopup(a), { className: 'pop' })
    .addTo(alertLayer);
}

function noteCapsize(a) {
  if (a.type !== 'T' || !Number.isFinite(a.at)) return;
  if (!(state.capsizedAt.get(a.buoy) >= a.at)) state.capsizedAt.set(a.buoy, a.at);
}

// Highest risk first, then the most recently seen
const ordered = () => [...victims.values()].sort((a, b) => b.risk - a.risk || b.last_at - a.last_at);

// The "Người cần cứu" block in the left column
function renderVictims() {
  const list = $('victim-list');
  const all = ordered();
  const admin = state.role === 'admin';
  $('victims-count').hidden = !all.length;
  $('victims-count').textContent = all.length;
  $('victims-empty').hidden = all.length > 0;
  $('btn-clear-victims').hidden = !admin || !all.length;
  $('victims').classList.toggle('has-victims', all.some(v => v.risk > 0));
  list.replaceChildren();
  const shown = showAll ? all : all.slice(0, LIST_MAX);
  for (const v of shown) {
    const li = document.createElement('li');
    const btn = document.createElement('button');
    btn.type = 'button'; btn.className = 'victim';
    btn.style.setProperty('--risk', RISKS[v.risk].color);
    const dot = document.createElement('span'); dot.className = 'risk-dot'; dot.setAttribute('aria-hidden', 'true');
    const main = document.createElement('span'); main.className = 'v-main';
    const name = document.createElement('b'); name.textContent = RISKS[v.risk].name;
    const conf = document.createElement('span'); conf.className = 'v-conf'; conf.textContent = `${Math.round(v.conf * 100)}%`;
    conf.title = 'Độ tin cậy';
    main.append(name, conf);
    const sub = document.createElement('span'); sub.className = 'v-sub'; sub.textContent = `${v.buoy} · ${fmtAgo(v.last_at)}`;
    const go = document.createElement('i'); go.className = 'ph ph-arrow-right'; go.setAttribute('aria-hidden', 'true');
    btn.append(dot, main, sub, go);
    btn.setAttribute('aria-label', `${RISKS[v.risk].name}, tin cậy ${Math.round(v.conf * 100)}%, ${v.buoy} báo ${fmtAgo(v.last_at)}. Xem trên bản đồ`);
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

// Ảnh chụp toàn bộ lúc mở trang, hoặc sau khi admin xoá chấm: vẽ lại, không báo
export function onVictimsSnapshot(m) {
  dotLayer.clearLayers(); dots.clear(); victims.clear(); alertLayer.clearLayers();
  for (const v of m.victims || []) putVictim(v);
  for (const a of m.alerts || []) { putAlertMarker(a); noteCapsize(a); }
  renderVictims();
}

export function onVictimMessage(m) {
  const v = m.victim;
  if (!v || !RISKS[v.risk]) return;
  state.eventBuoys.add(v.buoy);
  const before = victims.get(v.vid);
  putVictim(v);
  renderVictims();
  if (m.new || !before) {
    const risk = RISKS[v.risk].name;
    log('victim', `${v.buoy} phát hiện người, mức ${risk.toLowerCase()}`, v.first_at);
    toast(`${v.buoy} phát hiện người (${risk})`, 'bad');
  }
}

export function onAlertMessage(m) {
  const a = m.alert;
  if (!a) return;
  state.eventBuoys.add(a.buoy);
  if (a.type === 'T') {
    putAlertMarker(a);
    noteCapsize(a);
    log('capsize', `${a.buoy} bị lật`, a.at);
    toast(`${a.buoy} bị lật`, 'bad');
  } else if (a.type === 'C') {
    log('rescue', `${a.buoy}: người gặp nạn đã nhận đồ cứu hộ`, a.at);
    toast(`${a.buoy}: người gặp nạn đã nhận đồ cứu hộ`, 'ok');
  } else if (a.type === 'V' && RISKS[a.risk]) {
    log('victim', `${a.buoy} phát hiện người, mức ${RISKS[a.risk].name.toLowerCase()} (phao chưa có GPS)`, a.at);
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
    item(el('ph-fill ph-warning warn-ic'), 'Phao lật');
    item(el('hdg'), 'Mũi tàu');
    item(el('cog'), 'Hướng đi');
    box.append(title, body);
    return box;
  };
  return ctl;
}

export function initVictims(leafletMap, { toast: t, onClear }) {
  map = leafletMap;
  toast = t;
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
