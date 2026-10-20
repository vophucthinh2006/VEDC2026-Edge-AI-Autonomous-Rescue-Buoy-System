// Address search for the dashboard, proxied to Nominatim (OpenStreetMap).
// Nominatim usage policy: an identifying User-Agent, at most one request per second, cache results
// (https://operations.osmfoundation.org/policies/nominatim/). The Hub owns a single Geocoder, so the
// one-per-second spacing holds for the whole deployment instead of per Worker isolate.

const NOMINATIM_URL = "https://nominatim.openstreetmap.org/search";
export const CACHE_TTL_MS = 24 * 3600 * 1000;
export const UPSTREAM_GAP_MS = 1000;   // spacing between two calls to Nominatim, all sessions together
export const SESSION_GAP_MS = 1000;    // one uncached search per second per session
const MAX_QUEUE_MS = 5000;             // queue longer than this: tell the caller to retry
const MAX_CACHE = 500;
const MAX_RESULTS = 5;
const TIMEOUT_MS = 8000;

// Trimmed, single-spaced query of 2..200 characters, or null
export function normalizeQuery(q) {
  if (typeof q !== "string") return null;
  const s = q.normalize("NFC").replace(/\s+/g, " ").trim();
  return s.length >= 2 && s.length <= 200 ? s : null;
}

// Nominatim jsonv2 rows -> [{ name, lat, lon }]
function toResults(rows) {
  if (!Array.isArray(rows)) return [];
  const out = [];
  for (const r of rows) {
    const lat = Number(r?.lat), lon = Number(r?.lon);
    if (!Number.isFinite(lat) || !Number.isFinite(lon) || Math.abs(lat) > 90 || Math.abs(lon) > 180) continue;
    const name = String(r.display_name || r.name || "").slice(0, 200);
    if (!name) continue;
    out.push({ name, lat: +lat.toFixed(7), lon: +lon.toFixed(7) });
    if (out.length === MAX_RESULTS) break;
  }
  return out;
}

export class Geocoder {
  cache = new Map();     // query (lower case) -> { at, results }; Map order = least recently used first
  inflight = new Map();  // query -> Promise of the same upstream call
  lastBySession = new Map();
  nextSlot = 0;          // earliest time the next upstream call may start

  constructor({ userAgent, fetch: doFetch = (...a) => fetch(...a), now = Date.now,
                sleep = ms => new Promise(r => setTimeout(r, ms)) } = {}) {
    this.userAgent = userAgent || "mliot-redgescue/1.0 (rescue buoy shore station dashboard)";
    this.fetch = doFetch;
    this.now = now;
    this.sleep = sleep;
  }

  // Returns { status, body }: 200 { results }, 400 bad query, 429 rate limited, 502 upstream failed
  async search(rawQuery, sessionId) {
    const q = normalizeQuery(rawQuery);
    if (!q) return { status: 400, body: { error: "bad query" } };
    const key = q.toLowerCase();
    const now = this.now();

    const hit = this.cache.get(key);
    if (hit && now - hit.at < CACHE_TTL_MS) {
      this.cache.delete(key); this.cache.set(key, hit);   // refresh LRU position
      return { status: 200, body: { results: hit.results, cached: true } };
    }

    const sid = String(sessionId || "anon");
    const last = this.lastBySession.get(sid);
    if (last !== undefined && now - last < SESSION_GAP_MS) {
      return { status: 429, body: { error: "rate", retry_after_ms: SESSION_GAP_MS - (now - last) } };
    }
    this.lastBySession.set(sid, now);
    this.pruneSessions(now);

    if (!this.inflight.has(key)) {
      const wait = Math.max(0, this.nextSlot - now);
      if (wait > MAX_QUEUE_MS) return { status: 429, body: { error: "busy", retry_after_ms: wait } };
      this.nextSlot = now + wait + UPSTREAM_GAP_MS;
      const p = this.upstream(q, wait).finally(() => this.inflight.delete(key));
      this.inflight.set(key, p);
    }
    const results = await this.inflight.get(key);
    if (!results) return { status: 502, body: { error: "upstream" } };
    this.cache.delete(key);
    this.cache.set(key, { at: this.now(), results });
    while (this.cache.size > MAX_CACHE) this.cache.delete(this.cache.keys().next().value);
    return { status: 200, body: { results } };
  }

  // One Nominatim call after waiting for our slot; null on any failure (failures are not cached)
  async upstream(q, wait) {
    if (wait) await this.sleep(wait);
    const params = new URLSearchParams({
      q, format: "jsonv2", limit: String(MAX_RESULTS), countrycodes: "vn", "accept-language": "vi",
    });
    try {
      const res = await this.fetch(`${NOMINATIM_URL}?${params}`, {
        headers: { "User-Agent": this.userAgent, Accept: "application/json" },
        signal: AbortSignal.timeout(TIMEOUT_MS),
      });
      if (!res.ok) return null;
      return toResults(await res.json());
    } catch {
      return null;
    }
  }

  pruneSessions(now) {
    if (this.lastBySession.size < 1000) return;
    for (const [sid, t] of this.lastBySession) if (now - t > 60000) this.lastBySession.delete(sid);
  }
}
