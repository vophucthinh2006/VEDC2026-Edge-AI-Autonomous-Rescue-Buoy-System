// The guest page (/): for relatives far away following the rescue in their home place. They pick their
// home on the map, then read in plain words what the buoys near it are doing and who has been found
// there. Nothing here commands anything; the console for the people running the buoys is /admin (js/app.js).
// The data is the Hub's WebSocket stream, the same as the console's, read with the same rules (state.js).
import { state, readBuoy, buoyStatus, stationStatus, RISKS, LORA_GOOD_M, LORA_WEAK_M } from './state.js';
import { distM, bearingDeg, compassWord } from './util/geo.js';
import { fmtDist, fmtAgo, hm } from './util/format.js';

const $ = id => document.getElementById(id);
const DEFAULT_VIEW = [16.05, 108.2];   // central Vietnam, until the first buoy or the home place is known
const AREA_M = 5000;                   // "near your home": people and buoys this far from it
const FEED_MAX = 20;
const PLACE_KEY = 'guest-place';

// What a buoy is doing, in the words of someone who has never seen one (state.js buoyStatus)
const SAY = {
  patrol:    { text: 'Đang tìm kiếm người gặp nạn', tone: 'ok' },
  returning: { text: 'Đang quay về trạm', tone: 'warn' },
  stopped:   { text: 'Đang tạm dừng', tone: 'warn' },
  idle:      { text: 'Đang chờ điều động', tone: 'idle' },
  capsized:  { text: 'Đang gặp sự cố', tone: 'bad' },
  lost:      { text: 'Tạm mất liên lạc', tone: 'lost' },
};
const ACTIVE = new Set(['patrol', 'returning', 'stopped', 'idle']);   // heard lately and upright

const store = {
  get(k) { try { return JSON.parse(localStorage.getItem(k)); } catch { return null; } },
  set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch { /* private window: not remembered */ } },
};

const el = (tag, cls, text) => {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
};
const icon = name => { const i = el('i', 'ph ' + name); i.setAttribute('aria-hidden', 'true'); return i; };

// ================= MAP =================
const map = L.map('map', { zoomControl: false }).setView(DEFAULT_VIEW, 6);
L.control.zoom({ position: 'bottomright' }).addTo(map);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '&copy; OpenStreetMap' }).addTo(map);
// The map's box is laid out by the grid after Leaflet first measures it: measure again whenever it changes,
// and fit the home area only once the box has a size (fitted at 0 x 0 it zooms all the way in)
let fitDue = null;
function fitArea(p) {
  const box = $('map');
  if (!box.clientWidth || !box.clientHeight) { fitDue = p; return; }
  map.fitBounds(L.latLng(p.lat, p.lon).toBounds(AREA_M * 2), { padding: [20, 20] });
}
new ResizeObserver(() => {
  map.invalidateSize();
  if (fitDue) { const p = fitDue; fitDue = null; fitArea(p); }
}).observe($('map'));
const coverLayer = L.layerGroup().addTo(map);
const placeLayer = L.layerGroup().addTo(map);
const personLayer = L.layerGroup().addTo(map);
const buoyLayer = L.layerGroup().addTo(map);
let centered = false;

// ================= DATA =================
const victims = new Map();   // vid -> { lat, lon, risk, buoy, first_at, last_at }
const alerts = [];           // { type 'C' | 'T' | 'V', buoy, at, lat?, lon?, risk? }
const changes = [];          // buoy status changes seen live: { buoy, status, at }
const lastStatus = new Map();
let place = store.get(PLACE_KEY);

function onBuoy(m) {
  const id = String(m.id || 'PHAO-01');
  const b = readBuoy(state.buoys.get(id) || { hasPos: false }, m);
  state.buoys.set(id, b);
  const live = !(Number(m.age_s) > 0);
  const status = buoyStatus(id, b);
  const before = lastStatus.get(id);
  if (live && before && before !== status && ['patrol', 'returning', 'lost', 'capsized'].includes(status)) {
    changes.push({ buoy: id, status, at: Date.now() });
    if (changes.length > FEED_MAX) changes.shift();
  }
  lastStatus.set(id, status);
  if (b.hasPos && !centered && !place) { centered = true; map.setView([b.lat, b.lon], 14); }
  render();
}

function onStation(m) {
  const s = state.station || { hasPos: false };
  s.fix = m.fix === 1 ? 1 : 0;
  s.lastSeen = Date.now() - (Number(m.age_s) || 0) * 1000;
  if (Number.isFinite(m.lat) && Number.isFinite(m.lon)) { s.lat = m.lat; s.lon = m.lon; s.hasPos = true; }
  state.station = s;
  render();
}

function onAlert(a) {
  if (!a || !['C', 'T', 'V'].includes(a.type)) return;
  alerts.push(a);
  if (alerts.length > 200) alerts.shift();
  if (a.type === 'T' && Number.isFinite(a.at) && !(state.capsizedAt.get(a.buoy) >= a.at)) state.capsizedAt.set(a.buoy, a.at);
}

function onMessage(m) {
  if (m.type === 'station') onStation(m);
  else if (m.type === 'victims') {   // the whole list, on opening the page or after the admin cleared it
    victims.clear(); alerts.length = 0;
    for (const v of m.victims || []) if (RISKS[v.risk]) victims.set(v.vid, v);
    for (const a of m.alerts || []) onAlert(a);
    render();
  } else if (m.type === 'victim') { if (m.victim && RISKS[m.victim.risk]) victims.set(m.victim.vid, m.victim); render(); }
  else if (m.type === 'alert') { onAlert(m.alert); render(); }
  else if (['mission', 'downlink', 'history', 'lights', 'hold'].includes(m.type)) { /* the console's business */ }
  else onBuoy(m);
}

// ================= RENDER =================
let renderDue = false;
function render() {
  if (renderDue) return;
  renderDue = true;
  requestAnimationFrame(() => { renderDue = false; drawMap(); drawArea(); drawOverall(); drawFeed(); drawBuoys(); });
}

const buoysWithPos = () => [...state.buoys.entries()].filter(([, b]) => b.hasPos);
const near = p => q => Number.isFinite(q.lat) && Number.isFinite(q.lon) && distM(p, q) <= AREA_M;
const ordered = list => list.sort((a, b) => b.risk - a.risk || b.last_at - a.last_at);

function buoyIcon(id, b) {
  const say = SAY[buoyStatus(id, b)];
  return L.divIcon({ className: '', iconSize: [36, 36], iconAnchor: [18, 18],
    html: `<div class="g-pin" data-tone="${say.tone}"><i class="ph-fill ph-lifebuoy" aria-hidden="true"></i></div>` });
}

function personPopup(v) {
  const box = el('div', 'g-pop');
  const h = el('b', '', 'Người cần cứu');
  const risk = el('span', 'g-risk', 'Mức nguy hiểm: ' + RISKS[v.risk].name.toLowerCase());
  risk.style.setProperty('--c', RISKS[v.risk].color);
  box.append(h, risk, el('span', '', `Phát hiện lúc ${hm(new Date(v.first_at))}, thấy lần cuối ${fmtAgo(v.last_at)}`),
    el('span', '', `Phao ${v.buoy} báo về`));
  return box;
}

// Markers are kept and moved, not redrawn: a popup open on a person stays open while beacons come in
const buoyMarkers = new Map();     // id -> { marker, tone }
const personMarkers = new Map();   // vid -> circleMarker
let coverAt = '';

function drawMap() {
  for (const [id, b] of buoysWithPos()) {
    const say = SAY[buoyStatus(id, b)];
    const tip = `<b>${id}</b><br>${say.text}`;
    const had = buoyMarkers.get(id);
    if (!had) {
      const marker = L.marker([b.lat, b.lon], { icon: buoyIcon(id, b), keyboard: false })
        .bindTooltip(tip, { direction: 'top', offset: [0, -16], className: 'g-tip' }).addTo(buoyLayer);
      buoyMarkers.set(id, { marker, tone: say.tone });
      continue;
    }
    had.marker.setLatLng([b.lat, b.lon]).setTooltipContent(tip);
    if (had.tone !== say.tone) { had.marker.setIcon(buoyIcon(id, b)); had.tone = say.tone; }
  }
  for (const [vid, m] of personMarkers) if (!victims.has(vid)) { m.remove(); personMarkers.delete(vid); }
  for (const v of victims.values()) {
    const style = { radius: 8 + v.risk, color: '#FFFFFF', weight: 2.5, fillColor: RISKS[v.risk].color, fillOpacity: 1 };
    const m = personMarkers.get(v.vid);
    if (m) m.setLatLng([v.lat, v.lon]).setStyle(style).setRadius(style.radius);
    else personMarkers.set(v.vid, L.circleMarker([v.lat, v.lon], style)
      .bindPopup(() => personPopup(victims.get(v.vid) || v), { className: 'g-popup' }).addTo(personLayer));
  }
  const s = state.station;
  const at = s && s.hasPos ? `${s.lat},${s.lon}` : '';
  if (at === coverAt) return;
  coverAt = at;
  coverLayer.clearLayers();
  if (s && s.hasPos) {
    L.circle([s.lat, s.lon], { radius: LORA_WEAK_M, color: '#0E7490', weight: 1.5, opacity: .55, dashArray: '8 8', fillOpacity: 0, interactive: false }).addTo(coverLayer);
    L.circle([s.lat, s.lon], { radius: LORA_GOOD_M, color: '#0E7490', weight: 1.5, opacity: .7, fillColor: '#0E7490', fillOpacity: .06, interactive: false }).addTo(coverLayer);
    L.marker([s.lat, s.lon], { keyboard: false, icon: L.divIcon({ className: '', iconSize: [28, 28], iconAnchor: [14, 14],
      html: '<div class="g-station"><i class="ph-fill ph-broadcast" aria-hidden="true"></i></div>' }) })
      .bindTooltip('Trạm bờ', { direction: 'bottom', offset: [0, 14], className: 'g-tip' }).addTo(coverLayer);
  }
}

function drawPlace() {
  placeLayer.clearLayers();
  if (!place) return;
  L.circle([place.lat, place.lon], { radius: AREA_M, color: '#7C3AED', weight: 2, opacity: .85, dashArray: '6 6', fillColor: '#7C3AED', fillOpacity: .05, interactive: false }).addTo(placeLayer);
  L.marker([place.lat, place.lon], { zIndexOffset: 1500, keyboard: false, icon: L.divIcon({ className: '', iconSize: [30, 38], iconAnchor: [15, 36],
    html: '<div class="g-home"><i class="ph-fill ph-house-line" aria-hidden="true"></i></div>' }) })
    .bindTooltip(place.name.split(',')[0], { direction: 'top', offset: [0, -34], className: 'g-tip' }).addTo(placeLayer);
}

// The answer to "how is it at home": one sentence first, then the facts behind it
function drawArea() {
  const box = $('area');
  if (!place) { box.hidden = true; return; }
  const p = { lat: place.lat, lon: place.lon };
  const buoys = buoysWithPos().filter(([, b]) => distM(p, b) <= AREA_M);
  const active = buoys.filter(([id, b]) => ACTIVE.has(buoyStatus(id, b)));
  const people = ordered([...victims.values()].filter(near(p)));
  const aid = alerts.filter(a => a.type === 'C').filter(near(p));
  const nearest = buoysWithPos().map(([id, b]) => ({ id, b, d: distM(p, b) })).sort((x, y) => x.d - y.d)[0];

  const head = el('div', 'g-area-head');
  const name = el('div');
  name.append(el('small', '', 'Nơi bạn đang theo dõi'), el('h2', '', place.name.split(',').slice(0, 2).join(',')));
  const change = el('button', 'g-link', 'Đổi');
  change.type = 'button';
  change.addEventListener('click', () => { $('place-q').focus(); $('place-q').select(); window.scrollTo({ top: 0, behavior: 'smooth' }); });
  head.append(name, change);

  const searching = active.filter(([id, b]) => buoyStatus(id, b) === 'patrol').length;
  let summary;
  if (!active.length) summary = nearest
    ? `Chưa có phao cứu hộ nào trong vòng ${AREA_M / 1000} km. Phao gần nhất cách ${fmtDist(nearest.d)}.`
    : 'Chưa có phao cứu hộ nào báo về.';
  else summary = `Có ${active.length} phao cứu hộ trong vòng ${AREA_M / 1000} km` + (searching ? `, ${searching} phao đang tìm kiếm.` : '.');
  if (people.length) summary += ` Đã phát hiện ${people.length} người cần cứu ở khu vực này.`;
  if (aid.length) summary += ` ${aid.length} người đã nhận đồ cứu trợ.`;
  const lead = el('p', 'g-lead', summary);
  lead.dataset.tone = people.some(v => v.risk >= 2) ? 'bad' : active.length ? 'ok' : 'idle';

  const facts = el('ul', 'g-facts');
  if (nearest) {
    const say = SAY[buoyStatus(nearest.id, nearest.b)];
    const li = el('li');
    li.append(icon('ph-lifebuoy'), el('span', '', `Phao gần nhất: ${nearest.id}, cách ${fmtDist(nearest.d)} về phía ${compassWord(bearingDeg(p, nearest.b))}. `));
    const pill = el('span', 'g-pill', say.text); pill.dataset.tone = say.tone;
    li.append(pill);
    facts.append(li);
  }
  const s = state.station;
  if (s && s.hasPos) {
    const d = distM(p, s);
    const li = el('li');
    li.append(icon('ph-broadcast'), el('span', '', d <= LORA_GOOD_M ? 'Nằm trong vùng liên lạc tốt của trạm bờ.'
      : d <= LORA_WEAK_M ? 'Ở rìa vùng liên lạc của trạm bờ: tin báo về có thể chậm.' : 'Ngoài vùng liên lạc của trạm bờ hiện tại.'));
    facts.append(li);
  }
  box.replaceChildren(head, lead, facts);

  if (people.length) {
    const list = el('ul', 'g-people');
    for (const v of people.slice(0, 6)) {
      const li = el('li'); const b = el('button'); b.type = 'button';
      const dot = el('i', 'g-dot'); dot.style.background = RISKS[v.risk].color;
      const text = el('span'); text.append(el('b', '', 'Mức ' + RISKS[v.risk].name.toLowerCase()),
        el('small', '', `cách nhà ${fmtDist(distM(p, v))}, thấy lần cuối ${fmtAgo(v.last_at)}`));
      b.append(dot, text);
      b.addEventListener('click', () => map.flyTo([v.lat, v.lon], 17, { duration: .6 }));
      li.append(b); list.append(li);
    }
    box.append(el('h3', '', 'Người được phát hiện gần nhà'), list);
  }
  box.hidden = false;
}

function drawOverall() {
  const all = [...state.buoys.entries()];
  const active = all.filter(([id, b]) => ACTIVE.has(buoyStatus(id, b)));
  $('s-buoys').textContent = all.length ? `${active.length}/${all.length}` : '0';
  $('s-people').textContent = String(victims.size + alerts.filter(a => a.type === 'V').length);
  $('s-aid').textContent = String(alerts.filter(a => a.type === 'C').length);
  const last = Math.max(0, ...all.map(([, b]) => b.lastSeen || 0));
  $('s-updated').textContent = last ? `Tin mới nhất từ phao: ${fmtAgo(last)}.` : '';
}

function feedItems() {
  const items = [];
  for (const v of victims.values()) items.push({ at: v.first_at, tone: 'bad', ic: 'ph-user-focus', pos: v,
    text: `${v.buoy} phát hiện một người cần cứu, mức ${RISKS[v.risk].name.toLowerCase()}` });
  for (const a of alerts) {
    const pos = Number.isFinite(a.lat) && Number.isFinite(a.lon) ? a : null;
    if (a.type === 'C') items.push({ at: a.at, tone: 'ok', ic: 'ph-hand-heart', pos, text: `Một người đã nhận đồ cứu trợ từ ${a.buoy}` });
    else if (a.type === 'T') items.push({ at: a.at, tone: 'bad', ic: 'ph-warning', pos, text: `${a.buoy} gặp sự cố (bị lật)` });
    else items.push({ at: a.at, tone: 'bad', ic: 'ph-user-focus', pos: null, text: `${a.buoy} phát hiện một người, chưa xác định được vị trí` });
  }
  for (const c of changes) items.push({ at: c.at, tone: SAY[c.status].tone, ic: 'ph-lifebuoy', pos: state.buoys.get(c.buoy),
    text: `${c.buoy}: ${SAY[c.status].text.toLowerCase()}` });
  return items.filter(i => Number.isFinite(i.at)).sort((a, b) => b.at - a.at).slice(0, FEED_MAX);
}

function drawFeed() {
  const items = feedItems();
  $('feed-empty').hidden = items.length > 0;
  $('feed').replaceChildren(...items.map(i => {
    const li = el('li'); li.dataset.tone = i.tone;
    const time = el('time', '', hm(new Date(i.at)));
    time.title = fmtAgo(i.at);
    const body = el('span', '', i.text);
    li.append(icon(i.ic), body, time);
    if (i.pos && Number.isFinite(i.pos.lat)) {
      li.classList.add('go');
      li.tabIndex = 0;
      const go = () => map.flyTo([i.pos.lat, i.pos.lon], 17, { duration: .6 });
      li.addEventListener('click', go);
      li.addEventListener('keydown', e => { if (e.key === 'Enter') go(); });
    }
    return li;
  }));
}

function drawBuoys() {
  const p = place ? { lat: place.lat, lon: place.lon } : null;
  const rows = [...state.buoys.entries()].map(([id, b]) => ({ id, b, d: p && b.hasPos ? distM(p, b) : Infinity }))
    .sort((x, y) => x.d - y.d || x.id.localeCompare(y.id));
  $('buoys-empty').hidden = rows.length > 0;
  $('buoys').replaceChildren(...rows.map(({ id, b, d }) => {
    const say = SAY[buoyStatus(id, b)];
    const li = el('li'); const btn = el('button'); btn.type = 'button';
    const name = el('span', 'g-bname'); name.append(el('b', '', id), el('small', '', 'cập nhật ' + fmtAgo(b.lastSeen) + (Number.isFinite(d) ? ` · cách nhà ${fmtDist(d)}` : '')));
    const pill = el('span', 'g-pill', say.text); pill.dataset.tone = say.tone;
    btn.append(name, pill);
    if (b.hasPos) btn.addEventListener('click', () => map.flyTo([b.lat, b.lon], 16, { duration: .6 }));
    li.append(btn);
    return li;
  }));
}

// ================= HOME PLACE =================
function setPlace(p) {
  place = p;
  store.set(PLACE_KEY, p);
  $('place-choices').hidden = true;
  $('place-msg').hidden = true;
  $('place-q').value = '';
  drawPlace();
  fitArea(p);
  centered = true;
  render();
}

function say(text) { const m = $('place-msg'); m.textContent = text; m.hidden = false; }

$('place-form').addEventListener('submit', async e => {
  e.preventDefault();
  const q = $('place-q').value.trim();
  $('place-choices').hidden = true;
  if (q.length < 3) { say('Nhập ít nhất tên xã hoặc huyện.'); return; }
  say('Đang tìm…');
  try {
    const res = await fetch('/api/geocode?' + new URLSearchParams({ q }));
    if (res.status === 429) { say('Đang có nhiều người tìm cùng lúc. Thử lại sau vài giây.'); return; }
    const data = await res.json();
    const results = Array.isArray(data.results) ? data.results : [];
    if (!results.length) { say('Không tìm thấy địa chỉ này. Thử thêm tên huyện hoặc tỉnh.'); return; }
    if (results.length === 1) { setPlace(results[0]); return; }
    $('place-msg').hidden = true;
    $('place-choices').replaceChildren(...results.map(r => {
      const li = el('li'); const b = el('button'); b.type = 'button';
      const [title, ...rest] = String(r.name).split(',');
      b.append(icon('ph-map-pin'), el('span', '', title));
      b.lastChild.append(el('small', '', rest.join(',').trim()));
      b.addEventListener('click', () => setPlace(r));
      li.append(b); return li;
    }));
    $('place-choices').hidden = false;
  } catch { say('Không kết nối được máy chủ. Thử lại sau.'); }
});

// ================= CONNECTION =================
function setLive(up) {
  $('g-live').dataset.up = String(up);
  $('g-live').lastChild.textContent = up ? 'Trực tiếp' : 'Mất kết nối, đang thử lại';
}

// Anybody may watch: a guest session is opened without asking (it lets the page read, never command)
async function session() {
  const me = await fetch('/api/me').then(r => r.json()).catch(() => null);
  if (me && me.role) return true;
  const res = await fetch('/api/auth/guest', { method: 'POST' }).catch(() => null);
  return !!(res && res.ok);
}

async function connect() {
  if (!(await session())) { setLive(false); setTimeout(connect, 3000); return; }
  const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.onopen = () => setLive(true);
  ws.onclose = () => { setLive(false); setTimeout(connect, 2000); };
  ws.onerror = () => ws.close();
  ws.onmessage = ev => { try { onMessage(JSON.parse(ev.data)); } catch { /* not ours */ } };
}

if (place) { drawPlace(); fitArea(place); centered = true; }
render();
connect();
setInterval(render, 15000);   // "x phút trước" and "tạm mất liên lạc" move on with the clock
