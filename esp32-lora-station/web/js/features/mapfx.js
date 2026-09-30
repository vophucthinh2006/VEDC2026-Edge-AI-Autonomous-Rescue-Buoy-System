// Tiện ích bản đồ: thang tỷ lệ, tọa độ con trỏ, la bàn, vệt di chuyển, thước đo.
import { state } from '../state.js';
import { distM } from '../util/geo.js';
import { fmtDist } from '../util/format.js';

const $ = id => document.getElementById(id);
const TRAIL_MIN_STEP_M = 3;     // bỏ điểm cách điểm trước dưới 3 m để vệt không rối
const TRAIL_MAX_POINTS = 200;

export const fx = { measuring: false, trailsOn: true };

let map, store;
const trailLayer = L.layerGroup();
const trailLines = new Map();      // id -> polyline
let measurePts = [];
const measureLayer = L.layerGroup();
let measureLine = null;

// ---- Vệt di chuyển ----
function trailPoints(id) {
  const out = [];
  let prev = null;
  for (const p of state.history.get(id) || []) {
    if (!Number.isFinite(p.lat) || !Number.isFinite(p.lon)) continue;
    if (prev && distM(prev, p) < TRAIL_MIN_STEP_M) continue;
    out.push([p.lat, p.lon]);
    prev = p;
  }
  return out.slice(-TRAIL_MAX_POINTS);
}

export function updateTrail(id) {
  if (!fx.trailsOn) return;
  const pts = trailPoints(id);
  let line = trailLines.get(id);
  if (pts.length < 2) { if (line) { trailLayer.removeLayer(line); trailLines.delete(id); } return; }
  if (!line) {
    line = L.polyline(pts, { color: '#22D3EE', weight: 2.5, opacity: .7, interactive: false, lineCap: 'round' });
    trailLines.set(id, line);
    trailLayer.addLayer(line);
  } else {
    line.setLatLngs(pts);
  }
}

function setTrails(on) {
  fx.trailsOn = on;
  store.set('trails', on ? '1' : '0');
  $('btn-trails').setAttribute('aria-pressed', String(on));
  if (on) { map.addLayer(trailLayer); for (const id of state.buoys.keys()) updateTrail(id); }
  else map.removeLayer(trailLayer);
}

// ---- Thước đo ----
function renderMeasure() {
  measureLayer.clearLayers();
  if (measureLine) { measureLine = null; }
  const ll = measurePts.map(p => [p.lat, p.lon]);
  if (ll.length > 1) measureLine = L.polyline(ll, { color: '#F8FAFC', weight: 2, dashArray: '6 6', interactive: false }).addTo(measureLayer);
  let total = 0;
  measurePts.forEach((p, i) => {
    L.circleMarker([p.lat, p.lon], { radius: 5, color: '#0B1118', weight: 2, fillColor: '#F8FAFC', fillOpacity: 1, interactive: false }).addTo(measureLayer);
    if (i > 0) {
      const d = distM(measurePts[i - 1], p);
      total += d;
      const mid = [(measurePts[i - 1].lat + p.lat) / 2, (measurePts[i - 1].lon + p.lon) / 2];
      L.marker(mid, { interactive: false, icon: L.divIcon({ className: '', html: `<span class="leg-label">${fmtDist(d)}</span>`, iconSize: [0, 0] }) }).addTo(measureLayer);
    }
  });
  $('measure-total').textContent = measurePts.length > 1 ? fmtDist(total) : '0 m';
  $('measure-count').textContent = measurePts.length + ' điểm';
}

function setMeasuring(on) {
  fx.measuring = on;
  $('btn-measure').setAttribute('aria-pressed', String(on));
  $('measure-hud').hidden = !on;
  map.getContainer().classList.toggle('measuring', on);
  if (!on) { measurePts = []; measureLayer.clearLayers(); }
  else renderMeasure();
}

export function stopMeasuring() { if (fx.measuring) setMeasuring(false); }

// ---- Khởi tạo ----
export function initMapFx(leafletMap, storage) {
  map = leafletMap; store = storage;
  L.control.scale({ metric: true, imperial: false, position: 'bottomleft', maxWidth: 140 }).addTo(map);
  measureLayer.addTo(map);
  trailLayer.addTo(map);

  // Tọa độ con trỏ (chuột) hoặc tâm bản đồ (cảm ứng)
  const coord = $('hud-coord');
  const fmt = (lat, lon) => `${lat.toFixed(5)}, ${lon.toFixed(5)}`;
  const zoomTxt = () => 'Z' + map.getZoom();
  map.on('mousemove', e => { coord.textContent = fmt(e.latlng.lat, e.latlng.lng); });
  map.on('moveend zoomend', () => {
    const c = map.getCenter();
    if (!matchMedia('(hover: hover)').matches) coord.textContent = fmt(c.lat, c.lng);
    $('hud-zoom').textContent = zoomTxt();
  });
  const c0 = map.getCenter();
  coord.textContent = fmt(c0.lat, c0.lng);
  $('hud-zoom').textContent = zoomTxt();

  // Thước đo: mọi vai trò dùng được, không lưu gì
  map.on('click', e => {
    if (!fx.measuring) return;
    measurePts.push({ lat: e.latlng.lat, lon: e.latlng.lng });
    renderMeasure();
  });
  $('btn-measure').addEventListener('click', () => setMeasuring(!fx.measuring));
  $('measure-undo').addEventListener('click', () => { measurePts.pop(); renderMeasure(); });
  $('measure-exit').addEventListener('click', () => setMeasuring(false));
  document.addEventListener('keydown', e => { if (e.key === 'Escape') stopMeasuring(); });

  $('btn-trails').addEventListener('click', () => setTrails(!fx.trailsOn));
  fx.trailsOn = store.get('trails', '1') === '1';
  $('btn-trails').setAttribute('aria-pressed', String(fx.trailsOn));
  if (!fx.trailsOn) map.removeLayer(trailLayer);
}
