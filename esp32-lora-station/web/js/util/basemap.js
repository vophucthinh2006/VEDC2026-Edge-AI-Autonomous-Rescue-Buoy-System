// The base map, built to hold up on a weak link (docs/weather-map.md, "Bản đồ khi mạng yếu"):
//   - tiles come through the Worker (/tiles/osm/...), cached by Cloudflare near the user, and kept on the
//     device by sw.js, so a place seen once draws again without the network
//   - a tile the Worker cannot give is asked of OpenStreetMap directly; where there is no Worker at all
//     (the static dev server answers /tiles/ with 404) the layers go direct for good. Some home routers
//     and ISPs resolve tile.openstreetmap.org to 127.0.0.1: there only the Worker's tiles show
//   - under the detailed map a coarse one (zoom 13, a handful of tiles) fills the gaps while the detail
//     loads, so the map is blurry for a moment rather than grey
//   - round the shore station the tiles for zooms 11-16 are fetched once in the background, quietly, while
//     the map itself is not loading
const OSM_PROXY = '/tiles/osm/{z}/{x}/{y}.png';
const OSM_DIRECT = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png';
const DEM_PROXY = '/tiles/dem/{z}/{x}/{y}.png';
const DEM_DIRECT = 'https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png';
const ATTRIBUTION = '&copy; <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener">OpenStreetMap</a>';
const BACKDROP_ZOOM = 13;
const PREFETCH_KEY = 'basemap.prefetched';
const PREFETCH = { radiusM: 3000, zMin: 11, zMax: 16, maxTiles: 500, gapMs: 150 };

// Shared by every layer on the page: false until a probe finds no /tiles/ route here. Not set by slow or
// failed tiles: on a weak link those fail direct too, and the Worker's cache is the better bet.
const proxy = { dead: false };

const FallbackTiles = L.TileLayer.extend({
  getTileUrl(coords) {
    const data = { x: coords.x, y: coords.y, z: this._getZoomForUrl() };
    return L.Util.template(proxy.dead ? this.options.directUrl : this._url, data);
  },
  createTile(coords, done) {
    const tile = L.TileLayer.prototype.createTile.call(this, coords, done);
    tile._direct = L.Util.template(this.options.directUrl, { x: coords.x, y: coords.y, z: this._getZoomForUrl() });
    return tile;
  },
  _tileOnError(done, tile, e) {
    if (!tile._triedDirect && tile.src !== tile._direct) {
      tile._triedDirect = true;
      tile.src = tile._direct;
      return;
    }
    L.TileLayer.prototype._tileOnError.call(this, done, tile, e);
  },
});

let detail = null;
let busy = 0;   // tiles of the visible map still loading: the prefetch waits for 0

export function addBaseMap(map) {
  fetch('/tiles/osm/0/0/0.png').then(r => { if (r.status === 404) { proxy.dead = true; detail?.redraw(); } }).catch(() => { /* offline: keep the proxy */ });
  // The coarse layer first, so it sits under the detail
  new FallbackTiles(OSM_PROXY, { directUrl: OSM_DIRECT, minZoom: BACKDROP_ZOOM + 1, maxNativeZoom: BACKDROP_ZOOM, maxZoom: 19,
    className: 'basemap-backdrop', keepBuffer: 1 }).addTo(map);
  detail = new FallbackTiles(OSM_PROXY, { directUrl: OSM_DIRECT, maxZoom: 19, keepBuffer: 4, attribution: ATTRIBUTION }).addTo(map);
  detail.on('tileloadstart', () => { busy++; });
  detail.on('tileload tileerror tileabort', () => { busy = Math.max(0, busy - 1); });
  detail.on('load', () => { busy = 0; });
  registerTileCache();
  return detail;
}

// The DEM tile for the flood-depth layer: [through the Worker, straight from AWS]
export function demUrls(z, x, y) {
  const d = { z, x, y };
  const direct = L.Util.template(DEM_DIRECT, d);
  return proxy.dead ? [direct] : [L.Util.template(DEM_PROXY, d), direct];
}

function registerTileCache() {
  if (!('serviceWorker' in navigator) || !window.isSecureContext) return;
  navigator.serviceWorker.register('/sw.js', { scope: '/' }).catch(() => { /* the HTTP cache still helps */ });
}

// ---- prefetch round the station ----------------------------------------------------------------
const lon2x = (lon, z) => Math.floor((lon + 180) / 360 * 2 ** z);
const lat2y = (lat, z) => {
  const r = lat * Math.PI / 180;
  return Math.floor((1 - Math.log(Math.tan(r) + 1 / Math.cos(r)) / Math.PI) / 2 * 2 ** z);
};

export function tilesAround(lat, lon, { radiusM, zMin, zMax, maxTiles } = PREFETCH) {
  const dLat = radiusM / 111320, dLon = radiusM / (111320 * Math.cos(lat * Math.PI / 180));
  const out = [];
  for (let z = zMin; z <= zMax; z++) {
    const x0 = lon2x(lon - dLon, z), x1 = lon2x(lon + dLon, z);
    const y0 = lat2y(lat + dLat, z), y1 = lat2y(lat - dLat, z);
    for (let x = x0; x <= x1; x++) for (let y = y0; y <= y1; y++) {
      if (out.length >= maxTiles) return out;
      out.push({ z, x, y });
    }
  }
  return out;
}

const sleep = ms => new Promise(r => setTimeout(r, ms));
let prefetching = false;

// Once per place (about 1 km) per device; skipped when the user asked the browser to save data
export async function prefetchAround(lat, lon) {
  if (prefetching || proxy.dead || navigator.connection?.saveData) return;
  const key = `${lat.toFixed(2)},${lon.toFixed(2)}`;
  let done = [];
  try { done = JSON.parse(localStorage.getItem(PREFETCH_KEY)) || []; } catch { /* none yet */ }
  if (done.includes(key)) return;
  prefetching = true;
  try {
    await sleep(5000);   // let the page and the visible map come first
    let fails = 0;
    for (const t of tilesAround(lat, lon)) {
      while (busy > 0 || document.hidden) await sleep(1000);
      try {
        const res = await fetch(L.Util.template(OSM_PROXY, t));
        if (!res.ok) fails++;
      } catch { fails++; }
      if (fails > 20) return;   // the link is down or the Worker is not there: try again next visit
      await sleep(PREFETCH.gapMs);
    }
    done = [key, ...done.filter(k => k !== key)].slice(0, 10);
    try { localStorage.setItem(PREFETCH_KEY, JSON.stringify(done)); } catch { /* fetched again next time */ }
  } finally {
    prefetching = false;
  }
}
