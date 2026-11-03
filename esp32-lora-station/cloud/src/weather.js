// Weather and flood data for the map (docs/weather-map.md), fetched by the Hub and cached, so every page
// open does not reach the outside services:
//   - forecast: Open-Meteo, hourly rain, rain probability and wind for the next 24 h at a point
//   - radar: RainViewer's list of radar frames (the past 2 hours, every 10 min); the tiles come straight from
//     its tile cache in the browser
//   - gauges: the river water levels of the Vietnamese hydro-meteorological stations, as shown on the
//     national disaster monitoring map VNDMS (vndms.gov.vn). Not an official API: the GeoJSON of its map,
//     the values inside each station's popup HTML. It answers only with its own page as Referer.
// The Hub owns a single Weather, as it does the Geocoder (geocode.js).

const FORECAST_URL = "https://api.open-meteo.com/v1/forecast";
const RADAR_URL = "https://api.rainviewer.com/public/weather-maps.json";
const GAUGES_URL = "https://vndms.gov.vn/water_level";
export const FORECAST_TTL_MS = 30 * 60 * 1000;
export const RADAR_TTL_MS = 5 * 60 * 1000;
export const GAUGES_TTL_MS = 15 * 60 * 1000;
const STALE_MS = 6 * 3600 * 1000;       // an old answer is still given, marked stale, while the source is down
const MAX_FORECASTS = 200;
const TIMEOUT_MS = 10000;
const GRID = 0.05;                      // forecasts are kept per ~5 km cell

// "<b>Mực nước (2.58(m) 13-03/11)</b>": level 2.58 m at 13 h on 3 November, Vietnam time (UTC+7). The year is
// not given: this year, or last year for a date after `now` by more than a day (January reading December).
export function parseLevel(text, now = Date.now()) {
  const m = /Mực nước\s*\(\s*(-?\d+(?:\.\d+)?)\s*\(m\)\s*(\d{1,2})-(\d{1,2})\/(\d{1,2})\s*\)/.exec(text || "");
  if (!m) return null;
  const level = Number(m[1]), hour = Number(m[2]), day = Number(m[3]), month = Number(m[4]);
  if (!Number.isFinite(level) || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12) return null;
  let year = new Date(now + 7 * 3600e3).getUTCFullYear();
  let at = Date.UTC(year, month - 1, day, hour) - 7 * 3600e3;
  if (at - now > 86400e3) at = Date.UTC(--year, month - 1, day, hour) - 7 * 3600e3;
  return { level_m: level, at };
}

const field = (html, label) => {
  const m = new RegExp(label + ":\\s*<b>([^<]*)</b>").exec(html || "");
  return m ? m[1].trim() : "";
};

// One feature of VNDMS water_level -> { code, name, river, province, lat, lon, level_m, at }, or null
export function parseStation(feature, now = Date.now()) {
  const p = feature?.properties || {};
  const [lon, lat] = feature?.geometry?.coordinates || [];
  const code = field(p.popupInfo, "Mã trạm");
  if (!code || !Number.isFinite(lat) || !Number.isFinite(lon) || Math.abs(lat) > 90 || Math.abs(lon) > 180) return null;
  const level = parseLevel(p.popupInfo, now);
  return {
    code, name: field(p.popupInfo, "Tên trạm") || String(p.label || code), river: field(p.popupInfo, "Sông"),
    province: field(p.popupInfo, "Địa điểm"), lat: +lat.toFixed(5), lon: +lon.toFixed(5),
    level_m: level ? level.level_m : null, at: level ? level.at : null, alarm: 0,
  };
}

// The lists for lv=0 (every station) and lv=1..3 (stations above alarm level 1..3) -> stations with their alarm
export function mergeGauges(lists, now = Date.now()) {
  const byCode = new Map();
  for (const f of lists[0]?.features || []) {
    const s = parseStation(f, now);
    if (s) byCode.set(s.code, s);
  }
  for (const alarm of [1, 2, 3]) {
    for (const f of lists[alarm]?.features || []) {
      const s = parseStation(f, now);
      if (!s) continue;
      const known = byCode.get(s.code) || s;
      known.alarm = Math.max(known.alarm, alarm);
      byCode.set(s.code, known);
    }
  }
  return [...byCode.values()];
}

// Open-Meteo hourly -> the next 24 hours from now: [{ t, rain_mm, prob, wind_kmh }]
export function toForecast(body, now = Date.now()) {
  const h = body?.hourly;
  if (!h || !Array.isArray(h.time)) return null;
  const hours = [];
  for (let i = 0; i < h.time.length && hours.length < 24; i++) {
    const t = Date.parse(h.time[i] + "Z") - (body.utc_offset_seconds || 0) * 1000;
    if (!Number.isFinite(t) || t < now - 3600e3) continue;
    hours.push({ t, rain_mm: num(h.precipitation?.[i]), prob: num(h.precipitation_probability?.[i]), wind_kmh: num(h.wind_speed_10m?.[i]) });
  }
  return hours;
}
const num = v => (Number.isFinite(v) ? +(+v).toFixed(1) : null);

// RainViewer weather-maps.json -> { host, frames: [{ time, path }] } (radar frames of the past 2 hours)
export function toRadar(body) {
  const past = body?.radar?.past;
  if (typeof body?.host !== "string" || !Array.isArray(past)) return null;
  const frames = past.filter(f => Number.isFinite(f?.time) && typeof f?.path === "string" && /^\/v2\/radar\/\w+$/.test(f.path))
    .map(f => ({ time: f.time * 1000, path: f.path }));
  return frames.length ? { host: body.host, frames } : null;
}

export function distKm(a, b) {
  const rad = Math.PI / 180, x = (b.lon - a.lon) * rad * Math.cos((a.lat + b.lat) * rad / 2), y = (b.lat - a.lat) * rad;
  return 6371 * Math.hypot(x, y);
}

// The station with a water level nearest to p, or null
export function nearestGauge(stations, p) {
  let best = null;
  for (const s of stations || []) {
    if (s.level_m === null) continue;
    const d = distKm(p, s);
    if (!best || d < best.d) best = { s, d };
  }
  return best ? { ...best.s, km: +best.d.toFixed(1) } : null;
}

export class Weather {
  forecasts = new Map();   // "lat,lon" cell -> { at, hours }
  radar = null;            // { at, value }
  gauges = null;           // { at, value }
  inflight = new Map();

  constructor({ fetch: doFetch = (...a) => fetch(...a), now = Date.now } = {}) {
    this.fetch = doFetch;
    this.now = now;
  }

  // One call per key at a time; a failure gives null
  once(key, fn) {
    if (!this.inflight.has(key)) this.inflight.set(key, fn().catch(() => null).finally(() => this.inflight.delete(key)));
    return this.inflight.get(key);
  }

  async getJson(url, headers = {}) {
    const res = await this.fetch(url, { headers: { Accept: "application/json", ...headers }, signal: AbortSignal.timeout(TIMEOUT_MS) });
    if (!res.ok) throw new Error("http " + res.status);
    return res.json();
  }

  // { hours, stale? } or null
  async forecast(lat, lon) {
    const cell = [Math.round(lat / GRID) * GRID, Math.round(lon / GRID) * GRID].map(v => +v.toFixed(2));
    const key = cell.join(","), now = this.now(), hit = this.forecasts.get(key);
    if (hit && now - hit.at < FORECAST_TTL_MS) return { hours: hit.hours };
    const params = new URLSearchParams({ latitude: cell[0], longitude: cell[1], hourly: "precipitation,precipitation_probability,wind_speed_10m",
      forecast_days: "2", timezone: "Asia/Bangkok" });
    const hours = await this.once("f" + key, async () => toForecast(await this.getJson(`${FORECAST_URL}?${params}`), this.now()));
    if (hours) {
      this.forecasts.delete(key);
      this.forecasts.set(key, { at: this.now(), hours });
      while (this.forecasts.size > MAX_FORECASTS) this.forecasts.delete(this.forecasts.keys().next().value);
      return { hours };
    }
    return hit && now - hit.at < STALE_MS ? { hours: hit.hours, stale: true } : null;
  }

  async radarFrames() {
    return this.cached("radar", RADAR_TTL_MS, async () => toRadar(await this.getJson(RADAR_URL)));
  }

  // { stations, at, stale? } or null
  async gaugeList() {
    return this.cached("gauges", GAUGES_TTL_MS, async () => {
      const headers = { Referer: "https://vndms.gov.vn/", "User-Agent": "Mozilla/5.0 (compatible; mliot-redgescue rescue dashboard)" };
      const lists = await Promise.all([0, 1, 2, 3].map(lv => this.getJson(`${GAUGES_URL}?lv=${lv}`, headers)));
      const stations = mergeGauges(lists, this.now());
      return stations.length ? stations : null;
    });
  }

  // A value kept ttl, refetched after; the last good one is given, marked stale, while the source is down
  async cached(name, ttl, fetchValue) {
    const now = this.now(), hit = this[name];
    if (hit && now - hit.at < ttl) return { value: hit.value, at: hit.at };
    const value = await this.once(name, fetchValue);
    if (value) {
      this[name] = { at: this.now(), value };
      return { value, at: this[name].at };
    }
    return hit && now - hit.at < STALE_MS ? { value: hit.value, at: hit.at, stale: true } : null;
  }
}
