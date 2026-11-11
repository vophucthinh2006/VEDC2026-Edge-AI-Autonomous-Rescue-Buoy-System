// The map tile proxy (src/tiles.js) with a fake cache and a fake fetch, run straight in Node.
// Run:  node --test test/tiles.test.mjs
import assert from "node:assert/strict";
import test from "node:test";
import { parseTilePath, serveTile, upstreamUrl } from "../src/tiles.js";

function setup({ status = 200, throws = false } = {}) {
  const store = new Map();
  const calls = [];
  const pending = [];
  const deps = {
    cache: {
      match: async req => store.get(req.url)?.clone(),
      put: async (req, res) => { store.set(req.url, res); },
    },
    fetch: async (url, init) => {
      calls.push({ url, init });
      if (throws) throw new Error("offline");
      return new Response(status === 200 ? new Uint8Array([137, 80, 78, 71]) : null, { status, headers: { "Content-Type": "image/png" } });
    },
    waitUntil: p => pending.push(p),
  };
  return { deps, store, calls, settle: () => Promise.all(pending) };
}
const get = (headers = {}) => new Request("https://x.dev/tiles/osm/12/3245/1890.png", { headers });

test("parses good paths and turns away bad ones", () => {
  assert.deepEqual(parseTilePath("/tiles/osm/12/3245/1890.png"), { source: "osm", z: 12, x: 3245, y: 1890 });
  assert.deepEqual(parseTilePath("/tiles/dem/15/0/0.png"), { source: "dem", z: 15, x: 0, y: 0 });
  assert.equal(parseTilePath("/tiles/dem/16/0/0.png"), null);      // past the DEM's last zoom
  assert.equal(parseTilePath("/tiles/osm/20/0/0.png"), null);
  assert.equal(parseTilePath("/tiles/osm/2/4/0.png"), null);       // x outside 0..3
  assert.equal(parseTilePath("/tiles/osm/2/0/0.jpg"), null);
  assert.equal(parseTilePath("/tiles/other/2/0/0.png"), null);
  assert.equal(parseTilePath("/tiles/osm/../2/0/0.png"), null);
  assert.equal(upstreamUrl({ source: "osm", z: 3, x: 1, y: 2 }), "https://tile.openstreetmap.org/3/1/2.png");
});

test("fetches once with a User-Agent, then answers from the cache", async () => {
  const { deps, calls, settle } = setup();
  const t = parseTilePath("/tiles/osm/12/3245/1890.png");
  const a = await serveTile(get(), t, deps);
  assert.equal(a.status, 200);
  assert.match(a.headers.get("Cache-Control"), /max-age=604800/);
  assert.equal(calls.length, 1);
  assert.match(calls[0].init.headers["User-Agent"], /MLIoT/);
  await settle();
  const b = await serveTile(get(), t, deps);
  assert.equal(b.status, 200);
  assert.equal((await b.arrayBuffer()).byteLength, 4);
  assert.equal(calls.length, 1);
});

test("an upstream failure is not cached", async () => {
  const { deps, store, settle } = setup({ status: 503 });
  const r = await serveTile(get(), parseTilePath("/tiles/osm/1/0/0.png"), deps);
  await settle();
  assert.equal(r.status, 502);
  assert.equal(store.size, 0);
  const off = setup({ throws: true });
  assert.equal((await serveTile(get(), parseTilePath("/tiles/osm/1/0/0.png"), off.deps)).status, 502);
});

test("other sites and other methods are turned away", async () => {
  const { deps, calls } = setup();
  const t = parseTilePath("/tiles/osm/1/0/0.png");
  assert.equal((await serveTile(get({ "Sec-Fetch-Site": "cross-site" }), t, deps)).status, 403);
  assert.equal((await serveTile(new Request("https://x.dev/tiles/osm/1/0/0.png", { method: "POST", body: "x" }), t, deps)).status, 405);
  assert.equal(calls.length, 0);
});
