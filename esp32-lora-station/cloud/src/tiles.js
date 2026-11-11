// Map tiles through the Worker (docs/weather-map.md, "Bản đồ khi mạng yếu"):
//   /tiles/osm/{z}/{x}/{y}.png   the OpenStreetMap base map
//   /tiles/dem/{z}/{x}/{y}.png   the Terrarium elevation tiles behind the flood-depth layer
// Straight from the browser every tile crossed to Europe (OSM) or the US (AWS): slow and lossy on a weak
// link in the delta. Here the first page to ask fetches it once and Cloudflare's cache near the user keeps
// it for every page after, with long cache headers so the browser (and sw.js) keep it as well.
// OSM's tile policy (operations.osmfoundation.org/policies/tiles): a clear User-Agent, cache what is
// fetched, no bulk downloads. The prefetch in web/js/util/basemap.js is a few hundred tiles round the
// station, once per place per device.

export const SOURCES = {
  osm: { url: "https://tile.openstreetmap.org/{z}/{x}/{y}.png", maxZoom: 19, ttl: 7 * 86400 },
  dem: { url: "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png", maxZoom: 15, ttl: 30 * 86400 },
};
const USER_AGENT = "MLIoT-RedgeScue/1.0 (flood rescue console; tile cache)";
const PATH = /^\/tiles\/(osm|dem)\/(\d{1,2})\/(\d{1,7})\/(\d{1,7})\.png$/;

// "/tiles/osm/12/3245/1890.png" -> { source, z, x, y } or null when the path or the numbers are off
export function parseTilePath(path) {
  const m = PATH.exec(path);
  if (!m) return null;
  const source = m[1], z = +m[2], x = +m[3], y = +m[4];
  const n = 2 ** z;
  if (z > SOURCES[source].maxZoom || x >= n || y >= n) return null;
  return { source, z, x, y };
}

export function upstreamUrl({ source, z, x, y }) {
  return SOURCES[source].url.replace("{z}", z).replace("{x}", x).replace("{y}", y);
}

// deps: { cache (Cache API), fetch, waitUntil } so the tests can stand in for Cloudflare
export async function serveTile(request, t, deps) {
  if (request.method !== "GET" && request.method !== "HEAD") return new Response("method", { status: 405 });
  // Not an open tile server for other sites: a page elsewhere asking for our tiles is turned away
  if (request.headers.get("Sec-Fetch-Site") === "cross-site") return new Response("forbidden", { status: 403 });
  const key = new Request(`https://tiles.cache/${t.source}/${t.z}/${t.x}/${t.y}.png`);
  const hit = await deps.cache.match(key);
  if (hit) return hit;

  let up;
  try {
    up = await deps.fetch(upstreamUrl(t), { headers: { "User-Agent": USER_AGENT, Accept: "image/png,image/*" } });
  } catch {
    return new Response("upstream unreachable", { status: 502 });
  }
  // The DEM has no tiles over open sea at some zooms: an empty answer the page draws as nothing
  if (!up.ok) return new Response(null, { status: up.status === 404 ? 404 : 502, headers: { "Cache-Control": "no-store" } });

  const body = await up.arrayBuffer();
  const res = new Response(body, {
    headers: {
      "Content-Type": up.headers.get("Content-Type") || "image/png",
      "Cache-Control": `public, max-age=${SOURCES[t.source].ttl}, stale-while-revalidate=${SOURCES[t.source].ttl}`,
      "Access-Control-Allow-Origin": "*",
    },
  });
  deps.waitUntil(deps.cache.put(key, res.clone()));
  return res;
}
