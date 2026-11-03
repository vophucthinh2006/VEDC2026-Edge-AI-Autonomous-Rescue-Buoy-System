// The weather on the map (docs/weather-map.md), on the console and on the relatives' page alike:
//   - rain radar: the past 2 hours from RainViewer, frame by frame (coarse: its free tiles stop at zoom 7)
//   - forecast: rain, rain probability and wind for the next 24 hours at the shore station (Open-Meteo)
//   - flooding seen from space: NASA's 2-day VIIRS flood map of a day (where water is, not how deep)
//   - river gauges: the water levels of the national hydro-meteorological stations (VNDMS), by alarm level
//   - flood depth, estimated: a water level minus the ground's height (Terrarium DEM, ~30 m); the level is a
//     gauge's or one the admin typed in. An estimate, a few metres out in the delta: said so on the map.
// The Worker fetches and caches the forecast, the radar frames and the gauges (/api/weather, /api/gauges);
// the tiles come straight from each service. Everything sits in a pane of its own, over the base map and
// under the boats, so the dark map's colour inversion does not touch it.
import { fmtDist, hm } from '../util/format.js';

const STORE_KEY = 'weather.layers';
const REFRESH_MS = 10 * 60 * 1000;
const PLAY_MS = 700;
const DEM_URL = 'https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png';
const DEM_MAX_ZOOM = 15;
const DEM_MIN_ZOOM = 10;      // below this the depth layer would be a smear over the whole province
const BOTTOM_CLEAR = 36;      // px kept free at the bottom of the map for the scale and the sources line
const FLOOD_URL = 'https://gibs.earthdata.nasa.gov/wmts/epsg3857/best/VIIRS_Combined_Flood_2-Day/default/{time}/GoogleMapsCompatible_Level9/{z}/{y}/{x}.png';
export const ALARM = [
  { name: 'Dưới báo động', color: '#38BDF8' },
  { name: 'Báo động 1', color: '#FACC15' },
  { name: 'Báo động 2', color: '#FB923C' },
  { name: 'Báo động 3', color: '#EF4444' },
];
// Estimated depth, metres: [up to, colour]; deeper than the last is the last
export const DEPTH = [[0.5, [147, 197, 253, 110]], [1, [59, 130, 246, 140]], [2, [29, 78, 216, 165]], [Infinity, [49, 46, 129, 185]]];
const DEPTH_TEXT = ['< 0,5 m', '0,5–1 m', '1–2 m', '> 2 m'];

const ui = {};
let map = null, opts = {}, data = null, gauges = null, flood = null;
let radarLayer = null, frame = -1, playTimer = null;
const layers = {};           // name -> Leaflet layer
let on = { radar: false, flood: false, gauges: true, depth: false };

const el = (tag, cls, text) => {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
};
const isAdmin = () => (typeof opts.admin === 'function' ? opts.admin() : !!opts.admin);
const fmtM = v => `${v.toLocaleString('vi-VN', { maximumFractionDigits: 2 })} m`;
const dayStr = d => d.toISOString().slice(0, 10);
const dayText = d => `${d.getDate()}/${d.getMonth() + 1}`;

function save() { try { localStorage.setItem(STORE_KEY, JSON.stringify(on)); } catch { /* not remembered */ } }
function load() { try { on = { ...on, ...(JSON.parse(localStorage.getItem(STORE_KEY)) || {}) }; } catch { /* defaults */ } }

// ---- data ----------------------------------------------------------------------------------
async function fetchJson(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(String(res.status));
  return res.json();
}

// Where the forecast is for: the shore station once known, else the middle of the map
let asked = null;   // { lat, lon, station } of the last forecast asked for
function where() {
  const s = opts.station();
  if (s) return { lat: s.lat, lon: s.lon, station: true };
  const c = map.getCenter();
  return { lat: c.lat, lon: c.lng, station: false };
}

let retry = null;
let failure = null;   // why the last weather fetch failed: 'offline', 'auth', 'server'
const FAILURE_TEXT = {
  offline: 'Máy đang mất mạng: chưa lấy được dự báo.',
  auth: 'Phiên đăng nhập chưa sẵn sàng: đang thử lại.',
  server: 'Máy chủ hoặc nguồn dự báo chưa trả lời.',
};
async function refresh() {
  clearTimeout(retry);
  const c = where();
  asked = c;
  try {
    data = await fetchJson(`/api/weather?lat=${c.lat.toFixed(4)}&lon=${c.lon.toFixed(4)}`);
    flood = data.flood;
    failure = null;
  } catch (e) {
    // Not signed in yet (the relatives' page makes its session as it connects), or the Worker is away: soon again
    failure = !navigator.onLine ? 'offline' : /^(401|403)$/.test(e.message) ? 'auth' : 'server';
    retry = setTimeout(refresh, 15000);
  }
  if (on.gauges || on.depth || isAdmin()) {
    try { gauges = await fetchJson('/api/gauges'); } catch { /* keep the last list */ }
  }
  renderForecast();
  renderRadar();
  renderGauges();
  renderDepthInfo();
  if (layers.depth && on.depth) layers.depth.redraw();
}

// ---- forecast --------------------------------------------------------------------------------
function renderForecast() {
  const f = data?.forecast;
  ui.fc.replaceChildren();
  if (!f?.hours?.length) {
    // What went wrong, and a way to try again now rather than in 15 s
    const box = el('div', 'wx-error');
    box.setAttribute('role', 'status');
    const again = el('button', 'wx-retry', 'Thử lại');
    again.type = 'button';
    again.addEventListener('click', () => { again.disabled = true; again.textContent = 'Đang lấy…'; refresh(); });
    box.append(el('p', 'wx-note', failure ? FAILURE_TEXT[failure] : 'Đang lấy dự báo…'), again);
    ui.fc.append(box);
    return;
  }
  const hours = f.hours, total = hours.reduce((s, h) => s + (h.rain_mm || 0), 0);
  const prob = Math.max(...hours.map(h => h.prob ?? 0)), wind = Math.max(...hours.map(h => h.wind_kmh ?? 0));
  const head = el('div', 'wx-fc-head');
  head.append(el('b', '', total < 0.1 ? 'Không mưa 24 giờ tới' : `Mưa 24 giờ tới: ${total.toLocaleString('vi-VN', { maximumFractionDigits: 1 })} mm`),
    el('span', 'wx-note', `khả năng ${prob}% · gió tới ${Math.round(wind)} km/h${f.stale ? ' · số liệu cũ' : ''}`));
  const bars = el('div', 'wx-bars');
  bars.setAttribute('role', 'img');
  const peak = Math.max(2, ...hours.map(h => h.rain_mm || 0));
  for (const h of hours) {
    const b = el('i');
    b.style.height = `${Math.max(2, Math.round(100 * (h.rain_mm || 0) / peak))}%`;
    b.title = `${hm(new Date(h.t))}: ${h.rain_mm ?? 0} mm, khả năng ${h.prob ?? 0}%`;
    bars.append(b);
  }
  bars.setAttribute('aria-label', `Lượng mưa từng giờ trong 24 giờ tới, cao nhất ${peak.toLocaleString('vi-VN')} mm một giờ`);
  const axis = el('div', 'wx-axis');
  axis.append(el('span', '', hm(new Date(hours[0].t))), el('span', '', '+12 giờ'), el('span', '', '+24 giờ'));
  ui.fc.append(head, bars, axis);
}

// ---- rain radar ------------------------------------------------------------------------------
function radarUrl(i) {
  const r = data.radar;
  return `${r.host}${r.frames[i].path}/256/{z}/{x}/{y}/2/1_1.png`;
}

function renderRadar() {
  const r = data?.radar;
  ui.radarRow.hidden = !on.radar;
  if (!on.radar) { stopPlay(); if (radarLayer) radarLayer.remove(); return; }
  if (!r?.frames?.length) { ui.radarTime.textContent = 'Chưa lấy được radar'; return; }
  ui.radarSlider.max = String(r.frames.length - 1);
  if (frame < 0 || frame >= r.frames.length) frame = r.frames.length - 1;
  if (!radarLayer) {
    radarLayer = L.tileLayer(radarUrl(frame), { pane: 'weather', opacity: 0.7, maxNativeZoom: 7, maxZoom: 19,
      attribution: 'Mưa: <a href="https://www.rainviewer.com/" target="_blank" rel="noopener">RainViewer</a>' });
  }
  showFrame(frame);
  if (!map.hasLayer(radarLayer)) radarLayer.addTo(map);
}

function showFrame(i) {
  const r = data?.radar;
  if (!r?.frames?.length || !radarLayer) return;
  frame = i;
  radarLayer.setUrl(radarUrl(i), false);
  ui.radarSlider.value = String(i);
  const ago = Math.round((Date.now() - r.frames[i].time) / 60000);
  ui.radarTime.textContent = `${hm(new Date(r.frames[i].time))}${ago > 2 ? ` · ${ago} phút trước` : ' · mới nhất'}`;
}

function stopPlay() {
  clearInterval(playTimer); playTimer = null;
  ui.radarPlay.setAttribute('aria-pressed', 'false');
  ui.radarPlay.innerHTML = '<i class="ph-fill ph-play" aria-hidden="true"></i>';
  ui.radarPlay.setAttribute('aria-label', 'Phát 2 giờ qua');
}

function togglePlay() {
  if (playTimer) { stopPlay(); return; }
  const n = data?.radar?.frames?.length || 0;
  if (n < 2) return;
  ui.radarPlay.setAttribute('aria-pressed', 'true');
  ui.radarPlay.innerHTML = '<i class="ph-fill ph-pause" aria-hidden="true"></i>';
  ui.radarPlay.setAttribute('aria-label', 'Tạm dừng');
  playTimer = setInterval(() => showFrame((frame + 1) % n), PLAY_MS);
}

// ---- flooding seen from space ----------------------------------------------------------------
function renderFlood() {
  ui.floodRow.hidden = !on.flood;
  if (layers.flood) layers.flood.remove();
  if (!on.flood) return;
  const day = new Date(Date.now() - Number(ui.floodDay.value || 1) * 86400e3);
  layers.flood = L.tileLayer(FLOOD_URL.replace('{time}', dayStr(day)), { pane: 'weather', opacity: 0.85, maxNativeZoom: 9, maxZoom: 19,
    attribution: 'Ngập: <a href="https://www.earthdata.nasa.gov/data/instruments/viirs" target="_blank" rel="noopener">NASA</a>' }).addTo(map);
}

// ---- river gauges ----------------------------------------------------------------------------
function gaugePopup(s) {
  const box = el('div', 'map-pop wx-pop');
  const h = el('b', '', `Trạm ${s.name}`);
  const dl = el('dl');
  const row = (k, v) => { dl.append(el('dt', '', k), el('dd', '', v)); };
  if (s.river) row('Sông', s.river);
  if (s.province) row('Tỉnh', s.province);
  row('Mực nước', s.level_m === null ? 'không có số liệu' : fmtM(s.level_m));
  if (s.at) row('Lúc đo', `${hm(new Date(s.at))} ngày ${dayText(new Date(s.at))}`);
  row('Báo động', ALARM[s.alarm].name);
  box.append(h, dl, el('p', 'wx-note', 'Nguồn: KTTV qua Hệ thống giám sát thiên tai (vndms.gov.vn)'));
  if (isAdmin() && s.level_m !== null) {
    const b = el('button', 'btn small', 'Dùng mực nước này cho lớp độ sâu');
    b.type = 'button';
    b.addEventListener('click', () => { map.closePopup(); setLevel({ code: s.code }); });
    box.append(b);
  }
  return box;
}

function renderGauges() {
  if (!layers.gauges) layers.gauges = L.layerGroup();
  layers.gauges.clearLayers();
  if (!on.gauges) { layers.gauges.remove(); return; }
  for (const s of gauges?.stations || []) {
    // Above an alarm level the dot is bigger and carries the level's number: not told by colour alone
    const size = s.alarm ? 18 : 10;
    const icon = L.divIcon({ className: '', iconSize: [size, size], iconAnchor: [size / 2, size / 2],
      html: `<span class="wx-gauge" data-alarm="${s.alarm}" style="--c:${ALARM[s.alarm].color}">${s.alarm || ''}</span>` });
    L.marker([s.lat, s.lon], { pane: 'weather', icon, keyboard: false, title: `Trạm ${s.name}` })
      .bindTooltip(`${s.name}${s.level_m === null ? '' : `: ${fmtM(s.level_m)}`}${s.alarm ? ` (${ALARM[s.alarm].name})` : ''}`, { className: 'tip' })
      .bindPopup(() => gaugePopup(s), { className: 'pop' })
      .addTo(layers.gauges);
  }
  if (!map.hasLayer(layers.gauges)) layers.gauges.addTo(map);
}

// ---- estimated flood depth -------------------------------------------------------------------
const DepthLayer = L.GridLayer.extend({
  createTile(coords, done) {
    const tile = document.createElement('canvas');
    tile.width = tile.height = 256;
    const level = flood?.level_m;
    if (!Number.isFinite(level) || coords.z < DEM_MIN_ZOOM) { setTimeout(() => done(null, tile), 0); return tile; }
    // Above the DEM's last zoom: the part of its tile this one covers, blown up
    const dz = Math.max(0, coords.z - DEM_MAX_ZOOM), n = 1 << dz;
    const img = new Image();
    img.crossOrigin = 'anonymous';
    img.onload = () => {
      const ctx = tile.getContext('2d', { willReadFrequently: true });
      ctx.imageSmoothingEnabled = false;
      const s = 256 / n;
      ctx.drawImage(img, (coords.x % n) * s, (coords.y % n) * s, s, s, 0, 0, 256, 256);
      const px = ctx.getImageData(0, 0, 256, 256), d = px.data;
      for (let i = 0; i < d.length; i += 4) {
        const ground = d[i] * 256 + d[i + 1] + d[i + 2] / 256 - 32768;   // Terrarium encoding
        const depth = level - ground;
        if (depth <= 0) { d[i + 3] = 0; continue; }
        const c = DEPTH.find(([upTo]) => depth <= upTo)[1];
        d[i] = c[0]; d[i + 1] = c[1]; d[i + 2] = c[2]; d[i + 3] = c[3];
      }
      ctx.putImageData(px, 0, 0);
      done(null, tile);
    };
    img.onerror = () => done(null, tile);
    img.src = L.Util.template(DEM_URL, { z: coords.z - dz, x: coords.x >> dz, y: coords.y >> dz });
    return tile;
  },
});

function renderDepth() {
  ui.depthRow.hidden = !on.depth;
  if (!on.depth) { layers.depth?.remove(); return; }
  if (!layers.depth) {
    layers.depth = new DepthLayer({ pane: 'weather', opacity: 1, maxZoom: 19,
      attribution: 'DEM: <a href="https://registry.opendata.aws/terrain-tiles/" target="_blank" rel="noopener">Terrain Tiles</a>' });
  }
  if (!map.hasLayer(layers.depth)) layers.depth.addTo(map);
  layers.depth.redraw();
  renderDepthInfo();
}

function renderDepthInfo() {
  if (!ui.depthInfo) return;
  const f = flood;
  let text;
  if (!f) text = 'Chưa có mực nước: chưa lấy được số liệu trạm.';
  else if (f.source === 'manual') text = `Mực nước ${fmtM(f.level_m)} do admin nhập lúc ${hm(new Date(f.at))}.`;
  else text = `Mực nước ${fmtM(f.level_m)} ở trạm ${f.name}${f.river ? ` (sông ${f.river})` : ''}${Number.isFinite(f.km) ? `, cách ${fmtDist(f.km * 1000)}` : ''}`
    + `${f.at ? `, đo lúc ${hm(new Date(f.at))} ngày ${dayText(new Date(f.at))}` : ''}${f.stale ? ' (số liệu cũ)' : ''}.`;
  ui.depthInfo.textContent = text;
  ui.depthZoom.hidden = map.getZoom() >= DEM_MIN_ZOOM;
  if (ui.depthAdmin) ui.depthAdmin.hidden = !isAdmin();
}

async function setLevel(body) {
  try {
    const res = await (opts.post ? opts.post('/api/flood-level', body) : fetch('/api/flood-level', {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }));
    if (!res.ok) throw new Error(String(res.status));
    onFloodMessage(await res.json());
    if (!on.depth) { on.depth = true; ui.cb.depth.checked = true; save(); renderDepth(); }
  } catch { opts.toast?.('Chưa đổi được mực nước'); }
}

// The Hub's word on the depth layer's water level (the admin changed it, here or in another window)
export function onFloodMessage(m) {
  flood = m.flood || null;
  renderDepthInfo();
  if (layers.depth && on.depth) layers.depth.redraw();
}

// ---- the panel -------------------------------------------------------------------------------
function buildPanel() {
  // Top right, under the console's map tools: the bottom left is the legend's, the bottom middle the toasts'
  const ctl = L.control({ position: 'topright' });
  ctl.onAdd = () => {
    const box = L.DomUtil.create('div', 'wx-panel collapsed');
    L.DomEvent.disableClickPropagation(box);
    L.DomEvent.disableScrollPropagation(box);
    const title = el('button', 'wx-t');
    title.type = 'button';
    title.setAttribute('aria-controls', 'wx-body');
    title.innerHTML = '<span><i class="ph ph-cloud-rain" aria-hidden="true"></i> Thời tiết &amp; ngập</span><i class="ph ph-caret-down" aria-hidden="true"></i>';
    // Open, the panel takes the room left below it on the map, clear of the zoom buttons and scale bottom right
    const fit = () => {
      if (box.classList.contains('collapsed')) return;
      const m = map.getContainer().getBoundingClientRect(), b = box.getBoundingClientRect();
      const corner = map.getContainer().querySelector('.leaflet-bottom.leaflet-right')?.getBoundingClientRect().height || 0;
      box.style.maxHeight = `${Math.max(160, m.bottom - b.top - Math.max(BOTTOM_CLEAR, corner + 8))}px`;
    };
    const collapse = c => { box.classList.toggle('collapsed', c); title.setAttribute('aria-expanded', String(!c)); fit(); };
    map.on('resize', fit);
    title.addEventListener('click', () => collapse(!box.classList.contains('collapsed')));
    collapse(true);
    const body = el('div', 'wx-b');
    body.id = 'wx-body';
    ui.fc = el('div', 'wx-fc');
    body.append(ui.fc);

    ui.cb = {};
    const toggle = (name, label, hint, swatch) => {
      const row = el('label', 'wx-toggle');
      const cb = el('input');
      cb.type = 'checkbox'; cb.checked = !!on[name];
      cb.addEventListener('change', () => { on[name] = cb.checked; save(); render(name); });
      const txt = el('span');
      txt.append(el('b', '', label), el('small', '', hint));
      row.append(cb, swatch, txt);
      ui.cb[name] = cb;
      body.append(row);
    };
    const sw = cls => { const i = el('i', 'wx-sw ' + cls); i.setAttribute('aria-hidden', 'true'); return i; };

    toggle('radar', 'Radar mưa', '2 giờ qua, RainViewer', sw('rain'));
    ui.radarRow = el('div', 'wx-sub');
    ui.radarPlay = el('button', 'wx-play');
    ui.radarPlay.type = 'button';
    ui.radarPlay.addEventListener('click', togglePlay);
    ui.radarSlider = el('input');
    ui.radarSlider.type = 'range'; ui.radarSlider.min = '0'; ui.radarSlider.step = '1';
    ui.radarSlider.setAttribute('aria-label', 'Thời điểm radar');
    ui.radarSlider.addEventListener('input', () => { stopPlay(); showFrame(Number(ui.radarSlider.value)); });
    ui.radarTime = el('span', 'wx-note');
    ui.radarRow.append(ui.radarPlay, ui.radarSlider, ui.radarTime);
    body.append(ui.radarRow);
    stopPlay();

    toggle('flood', 'Vùng ngập từ vệ tinh', 'NASA VIIRS, 2 ngày gộp; mây che thì thiếu', sw('flood'));
    ui.floodRow = el('div', 'wx-sub');
    ui.floodDay = el('select');
    ui.floodDay.setAttribute('aria-label', 'Ngày chụp');
    for (let k = 1; k <= 7; k++) ui.floodDay.add(new Option(k === 1 ? `Hôm qua (${dayText(new Date(Date.now() - 864e5))})` : dayText(new Date(Date.now() - k * 864e5)), String(k)));
    ui.floodDay.addEventListener('change', renderFlood);
    ui.floodRow.append(el('span', 'wx-note', 'Ngày:'), ui.floodDay);
    body.append(ui.floodRow);

    toggle('gauges', 'Mực nước trạm thủy văn', 'KTTV, theo mức báo động', sw('gauge'));

    toggle('depth', 'Độ sâu ngập (ước lượng)', 'mực nước trừ độ cao địa hình', sw('depth'));
    ui.depthRow = el('div', 'wx-sub wx-depth');
    ui.depthInfo = el('p', 'wx-note');
    ui.depthZoom = el('p', 'wx-warn', 'Phóng to bản đồ thêm để thấy lớp độ sâu.');
    const scale = el('div', 'wx-scale');
    DEPTH.forEach(([, c], i) => {
      const s = el('span');
      const chip = el('i'); chip.style.background = `rgba(${c[0]},${c[1]},${c[2]},${(c[3] / 255).toFixed(2)})`;
      s.append(chip, document.createTextNode(DEPTH_TEXT[i]));
      scale.append(s);
    });
    ui.depthRow.append(ui.depthInfo, ui.depthZoom, scale, el('p', 'wx-warn', 'Chỉ là ước lượng từ bản đồ độ cao 30 m: có thể lệch vài mét ở đồng bằng.'));
    if (opts.admin !== false) {
      ui.depthAdmin = el('div', 'wx-depth');
      const form = el('form', 'wx-level');
      const input = el('input');
      input.type = 'number'; input.step = '0.01'; input.min = '-5'; input.max = '30'; input.placeholder = 'Mực nước (m)';
      input.setAttribute('aria-label', 'Mực nước nhập tay, mét');
      const apply = el('button', 'btn small', 'Áp dụng'); apply.type = 'submit';
      const nearest = el('button', 'btn small', 'Theo trạm gần nhất'); nearest.type = 'button';
      form.addEventListener('submit', e => { e.preventDefault(); const v = Number(input.value); if (input.value !== '' && Number.isFinite(v)) setLevel({ level_m: v }); });
      nearest.addEventListener('click', () => setLevel({}));
      form.append(input, apply, nearest);
      ui.depthAdmin.append(form, el('p', 'wx-note', 'Hoặc bấm một trạm thủy văn trên bản đồ để dùng mực nước của trạm đó.'));
      ui.depthRow.append(ui.depthAdmin);
    }
    body.append(ui.depthRow);

    const legend = el('div', 'wx-legend');
    for (const [k, a] of ALARM.entries()) {
      const s = el('span'); const dot = el('i', '', k ? String(k) : ''); dot.style.background = a.color;
      dot.setAttribute('aria-hidden', 'true');
      s.append(dot, document.createTextNode(a.name)); legend.append(s);
    }
    body.append(legend, el('p', 'wx-note wx-src', 'Dự báo: Open-Meteo. Radar: RainViewer. Vệ tinh: NASA. Mực nước: KTTV/VNDMS.'));
    box.append(title, body);
    return box;
  };
  return ctl;
}

function render(name) {
  if (name === 'radar') renderRadar();
  else if (name === 'flood') renderFlood();
  else if (name === 'gauges') { if (gauges) renderGauges(); else refresh(); }
  else if (name === 'depth') renderDepth();
}

// map: the Leaflet map. options: { admin (or a function saying so), station() -> {lat, lon} or null,
// toast(text), post(path, body) -> Response }
export function initWeather(leafletMap, options) {
  map = leafletMap;
  opts = options;
  load();
  map.attributionControl?.setPrefix(false);   // the sources line is long enough on a phone
  const pane = map.createPane('weather');
  pane.style.zIndex = 350;              // over the tiles (200), under paths and markers (400+)
  buildPanel().addTo(map);
  renderFlood();
  renderDepth();
  map.on('zoomend', renderDepthInfo);
  refresh();
  setInterval(refresh, REFRESH_MS);
  // The station heard after the first forecast, or moved: the forecast follows it
  setInterval(() => {
    const s = opts.station();
    if (s && (!asked?.station || Math.hypot(s.lat - asked.lat, s.lon - asked.lon) > 0.05)) refresh();
  }, 5000);
}
