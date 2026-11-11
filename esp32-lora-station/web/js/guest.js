// The guest page (/): for relatives far away following the rescue in their home place. They pick their
// home on the map, then read in plain words what the buoys near it are doing and who has been found
// there. Nothing here commands anything; the console for the people running the buoys is /admin (js/app.js).
// The data is the Hub's WebSocket stream, the same as the console's, read with the same rules (state.js).
import { state, readBuoy, buoyStatus, stationStatus, RISKS, LORA_GOOD_M, LORA_WEAK_M } from './state.js';
import { distM, bearingDeg, compassWord } from './util/geo.js';
import { addBaseMap, prefetchAround } from './util/basemap.js';
import { fmtDist, fmtAgo, hm } from './util/format.js';
import { initWeather, onFloodMessage } from './features/weather.js';

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
// The same, short, for the buoy chips
const SHORT = { patrol: 'Đang tìm kiếm', returning: 'Về trạm', stopped: 'Tạm dừng', idle: 'Chờ điều động', capsized: 'Gặp sự cố', lost: 'Mất liên lạc' };
// ?mock: dev/mock.js stands in for the Worker (never deployed, .assetsignore)
const MOCK = new URLSearchParams(location.search).has('mock');
let mockOn = false;   // dev/mock.js loaded: no guest session to open

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
addBaseMap(map);
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
// Rain, the forecast and flooding near the station (docs/weather-map.md); the relatives only look
initWeather(map, { admin: false, station: () => (state.station?.hasPos ? { lat: state.station.lat, lon: state.station.lon } : null) });
let centered = false;

// ================= DATA =================
const victims = new Map();   // vid -> { lat, lon, risk, buoy, first_at, last_at }
const alerts = [];           // { type 'C' | 'T' | 'V', buoy, at, lat?, lon?, risk? }
const changes = [];          // buoy status changes seen live: { buoy, status, at }
const lastStatus = new Map();
let place = store.get(PLACE_KEY);

// A place shared by a relative (?p=lat,lon&n=name, the "Gửi cho người nhà" button): it takes the place of the one
// remembered here, and the link is tidied away so a reload does not undo a change made afterwards
(() => {
  const q = new URLSearchParams(location.search);
  const [lat, lon] = String(q.get('p') || '').split(',').map(Number);
  if (!Number.isFinite(lat) || !Number.isFinite(lon) || Math.abs(lat) > 90 || Math.abs(lon) > 180) return;
  place = { lat, lon, name: String(q.get('n') || 'Nơi được chia sẻ').slice(0, 200) };
  store.set(PLACE_KEY, place);
  history.replaceState(null, '', location.pathname);
})();

// The weather at home (docs/weather-map.md): the forecast there and the river gauges round it, from the Worker
const HOME_WX_MS = 10 * 60 * 1000;
const GAUGE_NEAR_KM = 30;              // gauges this close to home are named when above an alarm level
const ALARM_TEXT = ['dưới mức báo động', 'vượt báo động 1', 'vượt báo động 2', 'vượt báo động 3'];
let homeWx = null;                      // { at, key, forecast, gauges }
async function loadHomeWeather(force = false) {
  if (!place) return;
  const key = `${place.lat.toFixed(3)},${place.lon.toFixed(3)}`;
  if (!force && homeWx && homeWx.key === key && Date.now() - homeWx.at < HOME_WX_MS) return;
  try {
    const [w, g] = await Promise.all([
      fetch(`/api/weather?lat=${place.lat.toFixed(4)}&lon=${place.lon.toFixed(4)}`).then(r => (r.ok ? r.json() : null)),
      fetch('/api/gauges').then(r => (r.ok ? r.json() : null)),
    ]);
    homeWx = { at: Date.now(), key, forecast: w?.forecast || null, gauges: g?.stations || [] };
  } catch { homeWx = homeWx && homeWx.key === key ? homeWx : null; }
  render();
}
setInterval(() => loadHomeWeather(), HOME_WX_MS);

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
  if (s.hasPos) prefetchAround(s.lat, s.lon);
}

function onAlert(a) {
  if (!a || !['C', 'T', 'V'].includes(a.type)) return;
  alerts.push(a);
  if (alerts.length > 200) alerts.shift();
  if (a.type === 'T' && Number.isFinite(a.at) && !(state.capsizedAt.get(a.buoy) >= a.at)) state.capsizedAt.set(a.buoy, a.at);
}

function onMessage(m) {
  if (m.type === 'station') onStation(m);
  else if (m.type === 'flood') onFloodMessage(m);
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
// The page is drawn as numbers and pictures (docs/guest-page-ui.md). The home card is built once in index.html and
// only its values change, so nothing flickers or replays its entrance every time a beacon comes in.
let renderDue = false;
function render() {
  if (renderDue) return;
  renderDue = true;
  const run = () => { if (!renderDue) return; renderDue = false; drawMap(); drawArea(); drawOverall(); drawFeed(); drawBuoys(); };
  requestAnimationFrame(run);
  setTimeout(run, 250);   // a tab in the background gets no animation frames: still drawn, ready when looked at
}

const buoysWithPos = () => [...state.buoys.entries()].filter(([, b]) => b.hasPos);
const near = p => q => Number.isFinite(q.lat) && Number.isFinite(q.lon) && distM(p, q) <= AREA_M;
const ordered = list => list.sort((a, b) => b.risk - a.risk || b.last_at - a.last_at);
const calm = window.matchMedia('(prefers-reduced-motion: reduce)');

function setText(node, t) { if (node && node.textContent !== t) node.textContent = t; }

// A count that changed rolls up to its new value (at once when motion is reduced or the page is hidden)
function countTo(node, n) {
  const to = Number(n) || 0;
  if (node.dataset.v === String(to)) return;
  const from = Number(node.dataset.v ?? 0) || 0;
  node.dataset.v = String(to);
  if (calm.matches || document.hidden || from === to) { node.textContent = String(to); return; }
  const t0 = performance.now(), dur = 650;
  const step = t => {
    if (node.dataset.v !== String(to)) return;   // a newer value took over
    const k = Math.min(1, (t - t0) / dur), e = 1 - Math.pow(1 - k, 3);
    node.textContent = String(Math.round(from + (to - from) * e));
    if (k < 1) requestAnimationFrame(step);
  };
  requestAnimationFrame(step);
  // A browser that holds animation frames back (a tab in the background) still ends on the right number
  setTimeout(() => { if (node.dataset.v === String(to)) node.textContent = String(to); }, dur + 80);
}

function buoyIcon(id, b) {
  const say = SAY[buoyStatus(id, b)];
  return L.divIcon({ className: '', iconSize: [36, 36], iconAnchor: [18, 18],
    html: `<div class="g-pin" data-tone="${say.tone}"><i class="ph-fill ph-lifebuoy" aria-hidden="true"></i></div>` });
}

// Where a person's rescue stands, in plain words (docs/victim-claim.md): who is with them, done, or waiting
function rescueStatus(v) {
  if (v.status === 'rescued') return { text: `Đã được cứu${Number.isFinite(v.status_at) ? ' lúc ' + hm(new Date(v.status_at)) : ''}`, short: 'Đã được cứu', tone: 'ok' };
  if (v.status === 'released') return { text: 'Đang chờ phao khác tới', short: 'Chờ phao', tone: 'warn' };
  if (v.status === 'claimed' && v.owner) return { text: `Phao ${v.owner} đang hỗ trợ`, short: 'Phao đang tới', tone: 'ok' };
  return null;
}

function personPopup(v) {
  const box = el('div', 'g-pop');
  const h = el('b', '', 'Người cần cứu');
  const risk = el('span', 'g-risk', 'Mức nguy hiểm: ' + RISKS[v.risk].name.toLowerCase());
  risk.style.setProperty('--c', RISKS[v.risk].color);
  box.append(h, risk, el('span', '', `Phát hiện lúc ${hm(new Date(v.first_at))}, thấy lần cuối ${fmtAgo(v.last_at)}`),
    el('span', '', `Phao ${v.buoy} báo về`));
  const st = rescueStatus(v);
  if (st) { const pill = el('span', 'g-pill', st.text); pill.dataset.tone = st.tone; box.append(pill); }
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
  $('map-home').hidden = !place;
  document.querySelector('.lg-home-row').hidden = !place;
  if (!place) return;
  L.circle([place.lat, place.lon], { radius: AREA_M, color: '#7C3AED', weight: 2, opacity: .85, dashArray: '6 6', fillColor: '#7C3AED', fillOpacity: .05, interactive: false }).addTo(placeLayer);
  L.marker([place.lat, place.lon], { zIndexOffset: 1500, keyboard: false, icon: L.divIcon({ className: '', iconSize: [30, 38], iconAnchor: [15, 36],
    html: '<div class="g-home"><i class="ph-fill ph-house-line" aria-hidden="true"></i></div>' }) })
    .bindTooltip(place.name.split(',')[0], { direction: 'top', offset: [0, -34], className: 'g-tip' }).addTo(placeLayer);
}

// How it is at home, in one coloured word: someone in danger, someone to rescue, buoys searching, nobody yet
const AREA_TONE = {
  bad:  { ic: 'ph-warning' },
  warn: { ic: 'ph-user-focus' },
  ok:   { ic: 'ph-lifebuoy' },
  idle: { ic: 'ph-clock' },
};
const LORA_ZONE = {
  good: { text: 'Trong vùng liên lạc tốt', ic: 'ph-wifi-high' },
  edge: { text: 'Rìa vùng liên lạc: tin có thể chậm', ic: 'ph-wifi-medium' },
  out:  { text: 'Ngoài vùng liên lạc của trạm bờ', ic: 'ph-wifi-slash' },
};
let picking = false;      // "Đổi": the search shows again in place of the home card
const seenPeople = new Set();

function drawArea() {
  const box = $('area');
  $('pick').hidden = !!place && !picking;
  $('pick-cancel').hidden = !place;
  box.hidden = !place || picking;
  $('people-card').hidden = true;
  if (!place) return;
  const p = { lat: place.lat, lon: place.lon };
  const buoys = buoysWithPos().filter(([, b]) => distM(p, b) <= AREA_M);
  const active = buoys.filter(([id, b]) => ACTIVE.has(buoyStatus(id, b)));
  const searching = active.filter(([id, b]) => buoyStatus(id, b) === 'patrol').length;
  const people = ordered([...victims.values()].filter(near(p)));
  const open = people.filter(v => v.status !== 'rescued');
  const claimsKnown = [...victims.values()].some(v => v.status);
  const helped = Math.max(people.filter(v => v.status === 'rescued').length,
    claimsKnown ? 0 : alerts.filter(a => a.type === 'C').filter(near(p)).length);
  const nearest = buoysWithPos().map(([id, b]) => ({ id, b, d: distM(p, b) })).sort((x, y) => x.d - y.d)[0];

  setText($('a-name'), place.name.split(',').slice(0, 2).join(','));

  // The ring: the most urgent thing first
  let tone, text;
  const need = open.filter(v => v.risk >= 1);   // "An toàn": found, not in danger
  if (need.some(v => v.risk >= 3)) { tone = 'bad'; text = 'Có người nguy kịch'; }
  else if (need.length) { tone = 'warn'; text = need.length > 1 ? `${need.length} người cần cứu` : 'Có người cần cứu'; }
  else if (active.length) { tone = 'ok'; text = searching ? 'Phao đang tìm kiếm' : 'Có phao gần nhà'; }
  else { tone = 'idle'; text = nearest ? 'Chưa có phao gần nhà' : 'Chưa có phao báo về'; }
  const st = $('a-status');
  if (st.dataset.tone !== tone) st.dataset.tone = tone;
  $('a-status-ic').className = 'ph-fill ' + AREA_TONE[tone].ic;
  setText($('a-status-text'), text);

  const s = state.station;
  const lora = $('a-lora');
  lora.hidden = !(s && s.hasPos);
  if (!lora.hidden) {
    const d = distM(p, s);
    const zone = d <= LORA_GOOD_M ? 'good' : d <= LORA_WEAK_M ? 'edge' : 'out';
    lora.dataset.zone = zone;
    lora.firstElementChild.className = 'ph ' + LORA_ZONE[zone].ic;
    setText(lora.lastElementChild, LORA_ZONE[zone].text);
  }

  countTo($('a-buoys'), active.length);
  countTo($('a-people'), people.length);
  countTo($('a-aid'), helped);
  const tile = $('a-near-tile');
  if (nearest) {
    const [num, unit] = fmtDist(nearest.d).split(' ');
    const deg = bearingDeg(p, nearest.b);
    setText($('a-near'), num);
    setText($('a-near-unit'), ' ' + unit);
    setText($('a-near-dir'), `phía ${compassWord(deg)}`);
    $('a-near-arrow').style.transform = `rotate(${Math.round(deg + 45)}deg)`;   // Phosphor's arrow points north-west
    tile.title = `Phao gần nhất: ${nearest.id}, ${SAY[buoyStatus(nearest.id, nearest.b)].text.toLowerCase()}`;
  } else {
    setText($('a-near'), '--'); setText($('a-near-unit'), ''); setText($('a-near-dir'), 'chưa có phao');
  }

  drawHomeWeather(p);
  drawPeople(p, people);
}

// The weather at home: rain to come and the river at the nearest gauge, as two small pictures
function drawHomeWeather(p) {
  const rain = $('w-rain'), river = $('w-river');
  rain.dataset.empty = String(!homeWx);
  river.dataset.empty = String(!homeWx);
  $('w-more').hidden = true;
  if (!homeWx) return;

  const hours = (homeWx.forecast?.hours || []).slice(0, 24);
  const bars = $('w-rain-bars');
  if (bars.children.length !== 24) bars.replaceChildren(...Array.from({ length: 24 }, () => el('i')));
  if (!hours.length) {
    setText($('w-rain-mm'), '--'); setText($('w-rain-sub'), ''); setText($('w-rain-peak'), 'Chưa có dự báo');
    for (const b of bars.children) b.style.transform = 'scaleY(.06)';
  } else {
    const total = hours.reduce((sum, h) => sum + (h.rain_mm || 0), 0), prob = Math.max(...hours.map(h => h.prob ?? 0));
    const peak = hours.reduce((a, h) => ((h.rain_mm || 0) > (a.rain_mm || 0) ? h : a), hours[0]);
    const top = Math.max(2, peak.rain_mm || 0);
    setText($('w-rain-mm'), total < 0.1 ? '0' : total.toLocaleString('vi-VN', { maximumFractionDigits: total < 10 ? 1 : 0 }));
    setText($('w-rain-sub'), ` mm · ${prob}%`);
    setText($('w-rain-peak'), total < 0.1 ? 'Không mưa' : peak.rain_mm >= 2 ? `To nhất khoảng ${hm(new Date(peak.t))}` : 'Mưa nhỏ');
    rain.dataset.heavy = String(total >= 50 || peak.rain_mm >= 10);
    rain.title = `Mưa 24 giờ tới: ${total.toLocaleString('vi-VN', { maximumFractionDigits: 1 })} mm, khả năng mưa ${prob}%`;
    [...bars.children].forEach((b, i) => {
      const mm = hours[i]?.rain_mm || 0;
      b.style.transform = `scaleY(${Math.max(0.06, mm / top).toFixed(3)})`;
      b.dataset.big = String(mm >= 8);
    });
  }

  const withLevel = homeWx.gauges.filter(g => g.level_m !== null && Number.isFinite(g.lat));
  const nearby = withLevel.map(g => ({ g, km: distM(p, g) / 1000 })).sort((a, b) => a.km - b.km);
  if (!nearby.length) {
    setText($('w-river-name'), 'Mực nước sông'); setText($('w-river-m'), '--'); setText($('w-river-at'), 'Chưa có trạm đo gần');
    river.dataset.alarm = '0';
    $('w-river-mark').style.left = '12.5%';
    setText($('w-river-mark').firstElementChild, '');
    return;
  }
  const { g, km } = nearby[0];
  setText($('w-river-name'), `Sông ${g.river || g.name}`);
  setText($('w-river-m'), g.level_m.toLocaleString('vi-VN'));
  setText($('w-river-at'), `Trạm ${g.name} · ${fmtDist(km * 1000)}`);
  river.dataset.alarm = String(g.alarm);
  river.title = `Sông ${g.river || g.name} ở trạm ${g.name}: ${g.level_m.toLocaleString('vi-VN')} m, ${ALARM_TEXT[g.alarm]}` + (g.at ? `, đo lúc ${hm(new Date(g.at))}` : '');
  $('w-river-mark').style.left = `${g.alarm * 25 + 12.5}%`;   // the band it is in: the thresholds themselves are not published
  setText($('w-river-mark').firstElementChild, g.alarm ? `BĐ${g.alarm}` : 'Dưới BĐ');
  const alarmed = nearby.filter(x => x.km <= GAUGE_NEAR_KM && x.g.alarm > 0 && x.g !== g);
  if (alarmed.length) {
    $('w-more').hidden = false;
    const t = `${alarmed.length} trạm khác trong ${GAUGE_NEAR_KM} km vượt báo động`;
    setText($('w-more').lastElementChild, t);
    $('w-more').title = alarmed.slice(0, 5).map(x => `${x.g.name} (BĐ${x.g.alarm})`).join(', ');
  }
}

// People found near home: danger dot, level, how far from home, how long ago, where the rescue stands
function drawPeople(p, people) {
  const card = $('people-card');
  card.hidden = !people.length;
  if (!people.length) return;
  const first = !seenPeople.size;
  $('near-list').replaceChildren(...people.slice(0, 6).map(v => {
    const li = el('li');
    if (!first && !seenPeople.has(v.vid)) li.className = 'new';
    seenPeople.add(v.vid);
    const b = el('button'); b.type = 'button';
    const dot = el('i', 'g-dot'); dot.style.setProperty('--c', RISKS[v.risk].color);
    const text = el('span', 'g-pmain');
    const meta = el('span', 'g-meta');
    const dist = el('span'); dist.title = 'Cách nhà'; dist.append(icon('ph-house-line'), document.createTextNode(fmtDist(distM(p, v))));
    const ago = el('span'); ago.title = `Thấy lần cuối ${fmtAgo(v.last_at)}`; ago.append(icon('ph-clock'), document.createTextNode(fmtAgo(v.last_at).replace(' trước', '')));
    meta.append(dist, ago);
    text.append(el('b', '', RISKS[v.risk].name), meta);
    b.append(dot, text);
    const st = rescueStatus(v);
    if (st) { const pill = el('span', 'g-pill', st.short); pill.dataset.tone = st.tone; pill.title = st.text; b.append(pill); }
    b.setAttribute('aria-label', `Mức ${RISKS[v.risk].name.toLowerCase()}, cách nhà ${fmtDist(distM(p, v))}, thấy lần cuối ${fmtAgo(v.last_at)}${st ? ', ' + st.text.toLowerCase() : ''}. Xem trên bản đồ`);
    b.addEventListener('click', () => map.flyTo([v.lat, v.lon], 17, { duration: .6 }));
    li.append(b);
    return li;
  }));
}

function drawOverall() {
  const all = [...state.buoys.entries()];
  const active = all.filter(([id, b]) => ACTIVE.has(buoyStatus(id, b)));
  setText($('s-buoys'), all.length ? `${active.length}/${all.length}` : '0');
  countTo($('s-people'), victims.size + alerts.filter(a => a.type === 'V').length);
  const rescued = [...victims.values()].filter(v => v.status === 'rescued').length;
  countTo($('s-aid'), Math.max(rescued, alerts.filter(a => a.type === 'C').length));
  drawTrend();
  const last = Math.max(0, ...all.map(([, b]) => b.lastSeen || 0));
  setText($('s-updated'), last ? `Tin mới nhất từ phao: ${fmtAgo(last)}` : '');
}

function feedItems() {
  const items = [];
  const claimsKnown = [...victims.values()].some(v => v.status);
  for (const v of victims.values()) {
    items.push({ at: v.first_at, tone: 'bad', ic: 'ph-user-focus', pos: v,
      text: `${v.buoy} thấy một người, mức ${RISKS[v.risk].name.toLowerCase()}` });
    if (v.status === 'rescued') items.push({ at: v.status_at, tone: 'ok', ic: 'ph-hand-heart', pos: v,
      text: `Người do ${v.buoy} thấy đã được cứu${v.owner && v.owner !== v.buoy ? ` (phao ${v.owner})` : ''}` });
  }
  for (const a of alerts) {
    const pos = Number.isFinite(a.lat) && Number.isFinite(a.lon) ? a : null;
    // The confirm button: said by the person's "rescued" line above when the Worker keeps who looks after whom
    if (a.type === 'C') { if (!claimsKnown) items.push({ at: a.at, tone: 'ok', ic: 'ph-hand-heart', pos, text: `Một người đã nhận đồ cứu trợ từ ${a.buoy}` }); }
    else if (a.type === 'T') items.push({ at: a.at, tone: 'bad', ic: 'ph-warning', pos, text: `${a.buoy} gặp sự cố (bị lật)` });
    else items.push({ at: a.at, tone: 'bad', ic: 'ph-user-focus', pos: null, text: `${a.buoy} thấy một người, chưa rõ vị trí` });
  }
  for (const c of changes) items.push({ at: c.at, tone: SAY[c.status].tone, ic: 'ph-lifebuoy', pos: state.buoys.get(c.buoy),
    text: `${c.buoy}: ${SAY[c.status].text.toLowerCase()}` });
  return items.filter(i => Number.isFinite(i.at)).sort((a, b) => b.at - a.at).slice(0, FEED_MAX);
}

const seenFeed = new Set();
function drawFeed() {
  const items = feedItems();
  $('feed-empty').hidden = items.length > 0;
  const first = !seenFeed.size;
  $('feed').replaceChildren(...items.map(i => {
    const key = `${i.at}|${i.text}`;
    const li = el('li'); li.dataset.tone = i.tone;
    if (!first && !seenFeed.has(key)) li.classList.add('new');   // slides in at the top
    seenFeed.add(key);
    const ic = el('span', 'f-ic'); ic.append(icon(i.ic));
    const time = el('time', '', fmtAgo(i.at).replace(' trước', ''));
    time.title = hm(new Date(i.at));
    li.append(ic, el('span', 'f-t', i.text), time);
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
    const li = el('li'); const btn = el('button'); btn.type = 'button'; btn.className = 'g-chip';
    btn.dataset.tone = say.tone;
    btn.title = `${say.text} · cập nhật ${fmtAgo(b.lastSeen)}${Number.isFinite(d) ? ` · cách nhà ${fmtDist(d)}` : ''}`;
    btn.append(el('i', 'g-cdot'), el('b', '', id), el('span', '', SHORT[buoyStatus(id, b)]));
    if (b.hasPos) btn.addEventListener('click', () => map.flyTo([b.lat, b.lon], 16, { duration: .6 }));
    else btn.disabled = true;
    li.append(btn);
    return li;
  }));
}

// "Gửi cho người nhà": a link that opens this page on the same place (the phone's share sheet, else copied)
async function sharePlace(btn) {
  const url = `${location.origin}/?` + new URLSearchParams({ p: `${place.lat.toFixed(5)},${place.lon.toFixed(5)}`, n: place.name });
  const title = `Tình hình cứu hộ ở ${place.name.split(',')[0]}`;
  const done = text => {
    btn.dataset.tip = text; btn.classList.add('done');
    setTimeout(() => { btn.classList.remove('done'); delete btn.dataset.tip; }, 2500);
  };
  try {
    if (navigator.share) { await navigator.share({ title, text: title, url }); return; }
    await navigator.clipboard.writeText(url);
    done('Đã sao chép đường dẫn');
  } catch (e) {
    if (e?.name === 'AbortError') return;          // the share sheet was closed
    window.prompt('Sao chép đường dẫn này và gửi cho người nhà:', url);
  }
}

// People found and rescued, hour by hour, the last TREND_HOURS hours (HTML bars, a sentence for screen readers)
const TREND_HOURS = 12;
let trendSig = '';
function drawTrend() {
  const box = $('trend');
  const now = new Date(), end = new Date(now); end.setMinutes(0, 0, 0);
  const start = end.getTime() - (TREND_HOURS - 1) * 3600e3;
  const found = new Array(TREND_HOURS).fill(0), saved = new Array(TREND_HOURS).fill(0);
  const slot = t => Math.floor((t - start) / 3600e3);
  const add = (arr, t) => { const i = slot(t); if (Number.isFinite(t) && i >= 0 && i < TREND_HOURS) arr[i]++; };
  for (const v of victims.values()) { add(found, v.first_at); if (v.status === 'rescued') add(saved, v.status_at); }
  for (const a of alerts) if (a.type === 'V') add(found, a.at);
  const claimsKnown = [...victims.values()].some(v => v.status);
  if (!claimsKnown) for (const a of alerts) if (a.type === 'C') add(saved, a.at);   // older Worker: the confirm presses
  const sig = `${start}|${found}|${saved}`;
  if (sig === trendSig) return;   // redrawn only when it changed: the bars grow once, not every 15 s
  trendSig = sig;
  const peak = Math.max(1, ...found, ...saved), totalF = found.reduce((a, b) => a + b, 0), totalS = saved.reduce((a, b) => a + b, 0);
  const chart = el('div', 'g-trend-chart');
  chart.setAttribute('role', 'img');
  chart.setAttribute('aria-label', `Trong ${TREND_HOURS} giờ qua: ${totalF} người được phát hiện, ${totalS} người được cứu.`);
  for (let i = 0; i < TREND_HOURS; i++) {
    const h = new Date(start + i * 3600e3).getHours();
    const col = el('div', 'g-trend-col');
    col.title = `${h}h–${h + 1}h: ${found[i]} phát hiện, ${saved[i]} được cứu`;
    col.style.setProperty('--d', `${i * 35}ms`);
    const bars = el('div', 'g-trend-bars');
    for (const [n, cls] of [[found[i], 'f'], [saved[i], 's']]) {
      const bar = el('i', cls);
      bar.style.height = n ? `${Math.max(8, Math.round(100 * n / peak))}%` : '0';
      if (n) bar.dataset.n = n;
      bars.append(bar);
    }
    col.append(bars, el('span', '', i % 3 === 2 || i === TREND_HOURS - 1 ? (i === TREND_HOURS - 1 ? 'Giờ này' : `${h}h`) : ''));
    chart.append(col);
  }
  const legend = el('div', 'g-trend-legend');
  legend.append(el('span', 'f', `Phát hiện ${totalF}`), el('span', 's', `Được cứu ${totalS}`));
  box.replaceChildren(legend, chart);
}

// ================= HOME PLACE =================
function setPlace(p) {
  place = p;
  picking = false;
  store.set(PLACE_KEY, p);
  $('place-choices').hidden = true;
  $('place-msg').hidden = true;
  $('place-q').value = '';
  drawPlace();
  fitArea(p);
  centered = true;
  homeWx = null;
  loadHomeWeather();
  render();
}

function say(text) { const m = $('place-msg'); m.textContent = text; m.hidden = false; }

// "Đổi": the search comes back in place of the home card; "Giữ nơi cũ" puts the card back
$('a-change').addEventListener('click', () => {
  picking = true; render();
  requestAnimationFrame(() => { $('pick').scrollIntoView({ behavior: calm.matches ? 'auto' : 'smooth', block: 'start' }); $('place-q').focus(); });
});
$('pick-cancel').addEventListener('click', () => { picking = false; $('place-choices').hidden = true; $('place-msg').hidden = true; render(); });
$('a-share').addEventListener('click', e => sharePlace(e.currentTarget));
$('map-home').addEventListener('click', () => { if (place) fitArea(place); });

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
  if (!mockOn && !(await session())) { setLive(false); setTimeout(connect, 3000); return; }
  const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.onopen = () => { setLive(true); loadHomeWeather(); };   // the weather at home needs the session just made
  ws.onclose = () => { setLive(false); setTimeout(connect, 2000); };
  ws.onerror = () => ws.close();
  ws.onmessage = ev => { try { onMessage(JSON.parse(ev.data)); } catch { /* not ours */ } };
}

if (place) { drawPlace(); fitArea(place); centered = true; }
render();
if (MOCK) {
  const s = document.createElement('script');
  s.src = '/dev/mock.js';
  s.onload = () => { mockOn = true; connect(); };
  s.onerror = connect;   // deployed there is no mock (.assetsignore): the real stream then
  document.head.append(s);
} else connect();
setInterval(render, 15000);   // "x phút trước" and "tạm mất liên lạc" move on with the clock
