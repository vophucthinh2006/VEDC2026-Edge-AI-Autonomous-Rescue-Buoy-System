// Map extras: scale bar and the always-on movement trail of each buoy.
import { state } from '../state.js';
import { distM } from '../util/geo.js';

const TRAIL_MIN_STEP_M = 3;     // bỏ điểm cách điểm trước dưới 3 m để vệt không rối
const TRAIL_MAX_POINTS = 200;

const trailLayer = L.layerGroup();
const trailLines = new Map();      // id -> polyline

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
  const pts = trailPoints(id);
  let line = trailLines.get(id);
  if (pts.length < 2) { if (line) { trailLayer.removeLayer(line); trailLines.delete(id); } return; }
  if (!line) {
    line = L.polyline(pts, { color: '#22D3EE', weight: 2, opacity: .35, interactive: false, lineCap: 'round' });
    trailLines.set(id, line);
    trailLayer.addLayer(line);
  } else {
    line.setLatLngs(pts);
  }
}

export function initMapFx(map) {
  L.control.scale({ metric: true, imperial: false, position: 'bottomright', maxWidth: 120 }).addTo(map);
  trailLayer.addTo(map);
}
