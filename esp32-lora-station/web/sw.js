// Keeps the map tiles on the device (docs/weather-map.md, "Bản đồ khi mạng yếu"). Only /tiles/ is
// touched: pages, scripts and the API go to the network as before, so a new deploy shows at once.
// A tile seen once is answered from here at once, with or without a network; one older than a week is
// refreshed in the background when the device is online. The oldest tiles go past MAX_TILES.
const CACHE = 'tiles-v1';
const MAX_TILES = 4000;          // ~15-20 kB each
const REFRESH_MS = 7 * 86400e3;
let puts = 0;

self.addEventListener('install', () => self.skipWaiting());
self.addEventListener('activate', e => e.waitUntil((async () => {
  for (const k of await caches.keys()) if (k.startsWith('tiles-') && k !== CACHE) await caches.delete(k);
  await self.clients.claim();
})()));

self.addEventListener('fetch', e => {
  const req = e.request;
  if (req.method !== 'GET') return;
  const url = new URL(req.url);
  if (url.origin !== self.location.origin || !url.pathname.startsWith('/tiles/')) return;
  e.respondWith(tile(e, url.pathname));
});

async function tile(e, path) {
  const cache = await caches.open(CACHE);
  const hit = await cache.match(path);
  if (hit) {
    const age = Date.now() - Date.parse(hit.headers.get('Date') || 0);
    if (!(age < REFRESH_MS) && self.navigator.onLine) e.waitUntil(fetchAndKeep(cache, path).catch(() => {}));
    return hit;
  }
  return fetchAndKeep(cache, path);
}

async function fetchAndKeep(cache, path) {
  const res = await fetch(path);
  if (res.ok) {
    // Stamp the time kept, for the weekly refresh (a cached response's own Date can be the CDN's)
    const headers = new Headers(res.headers);
    headers.set('Date', new Date().toUTCString());
    const kept = new Response(await res.clone().blob(), { status: res.status, headers });
    await cache.put(path, kept);
    if (++puts % 100 === 0) await trim(cache);
  }
  return res;
}

async function trim(cache) {
  const keys = await cache.keys();   // oldest first
  for (let i = 0; i < keys.length - MAX_TILES; i++) await cache.delete(keys[i]);
}
