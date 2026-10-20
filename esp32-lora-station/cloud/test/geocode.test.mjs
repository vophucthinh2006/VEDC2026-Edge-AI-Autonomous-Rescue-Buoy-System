// Address search (Nominatim proxy) with a fake fetch and a fake clock, run straight in Node.
// Run:  node --test test/geocode.test.mjs
import assert from "node:assert/strict";
import { registerHooks } from "node:module";
import test from "node:test";
import { Geocoder, CACHE_TTL_MS, UPSTREAM_GAP_MS, normalizeQuery } from "../src/geocode.js";

const ROWS = Array.from({ length: 7 }, (_, i) => ({
  display_name: `Bến Ninh Kiều ${i}, Cần Thơ, Việt Nam`, lat: String(10.0345 + i / 1000), lon: String(105.7884 + i / 1000),
}));

function setup({ status = 200, rows = ROWS } = {}) {
  const clock = { t: 1_000_000 };
  const calls = [];
  const slept = [];
  const geo = new Geocoder({
    userAgent: "test-agent/1.0",
    now: () => clock.t,
    sleep: async ms => { slept.push(ms); clock.t += ms; },
    fetch: async (url, init) => {
      calls.push({ url: new URL(url), init });
      return new Response(JSON.stringify(rows), { status, headers: { "content-type": "application/json" } });
    },
  });
  return { geo, clock, calls, slept };
}

test("asks Nominatim politely and returns five {name, lat, lon}", async () => {
  const { geo, calls } = setup();
  const r = await geo.search("  bến   Ninh Kiều ", "s1");
  assert.equal(r.status, 200);
  assert.equal(r.body.results.length, 5);
  assert.deepEqual(r.body.results[0], { name: "Bến Ninh Kiều 0, Cần Thơ, Việt Nam", lat: 10.0345, lon: 105.7884 });
  assert.equal(calls.length, 1);
  const { url, init } = calls[0];
  assert.equal(url.origin + url.pathname, "https://nominatim.openstreetmap.org/search");
  assert.equal(url.searchParams.get("q"), "bến Ninh Kiều");
  assert.equal(url.searchParams.get("countrycodes"), "vn");
  assert.equal(url.searchParams.get("accept-language"), "vi");
  assert.equal(url.searchParams.get("limit"), "5");
  assert.equal(url.searchParams.get("format"), "jsonv2");
  assert.equal(init.headers["User-Agent"], "test-agent/1.0");
});

test("serves a repeated query from the cache for a day", async () => {
  const { geo, clock, calls } = setup();
  await geo.search("Bến Ninh Kiều", "s1");
  const again = await geo.search("bến ninh  kiều", "s1");   // same session, same instant: cache hits are free
  assert.equal(again.status, 200);
  assert.equal(again.body.cached, true);
  assert.equal(calls.length, 1);
  clock.t += CACHE_TTL_MS + 1;
  await geo.search("Bến Ninh Kiều", "s1");
  assert.equal(calls.length, 2);
});

test("allows one uncached search per second per session", async () => {
  const { geo, clock, calls } = setup();
  assert.equal((await geo.search("Cần Thơ", "s1")).status, 200);
  clock.t += 300;
  const fast = await geo.search("Vĩnh Long", "s1");
  assert.equal(fast.status, 429);
  assert.equal(fast.body.retry_after_ms, 700);
  clock.t += 700;
  assert.equal((await geo.search("Vĩnh Long", "s1")).status, 200);
  assert.equal(calls.length, 2);
});

test("spaces upstream calls one second apart across sessions", async () => {
  const { geo, calls, slept } = setup();
  const [a, b] = await Promise.all([geo.search("Cần Thơ", "s1"), geo.search("Vĩnh Long", "s2")]);
  assert.equal(a.status, 200);
  assert.equal(b.status, 200);
  assert.equal(calls.length, 2);
  assert.deepEqual(slept, [UPSTREAM_GAP_MS]);
});

test("one upstream call answers concurrent identical queries", async () => {
  const { geo, calls } = setup();
  const [a, b] = await Promise.all([geo.search("Cần Thơ", "s1"), geo.search("cần thơ", "s2")]);
  assert.equal(a.status, 200);
  assert.deepEqual(a.body.results, b.body.results);
  assert.equal(calls.length, 1);
});

test("upstream failure is reported and not cached", async () => {
  const { geo, clock, calls } = setup({ status: 503 });
  assert.equal((await geo.search("Cần Thơ", "s1")).status, 502);
  clock.t += 1000;
  assert.equal((await geo.search("Cần Thơ", "s1")).status, 502);
  assert.equal(calls.length, 2);
});

test("drops rows without usable coordinates", async () => {
  const { geo } = setup({ rows: [{ display_name: "A", lat: "x", lon: "1" }, { display_name: "", lat: "1", lon: "1" }, { display_name: "B", lat: "10", lon: "106" }] });
  assert.deepEqual((await geo.search("ab", "s1")).body.results, [{ name: "B", lat: 10, lon: 106 }]);
});

test("rejects empty or oversized queries", async () => {
  const { geo, calls } = setup();
  assert.equal(normalizeQuery(" a "), null);
  assert.equal(normalizeQuery("x".repeat(201)), null);
  assert.equal(normalizeQuery(42), null);
  assert.equal((await geo.search("", "s1")).status, 400);
  assert.equal(calls.length, 0);
});

test("the Hub answers /geocode through its single Geocoder", async () => {
  registerHooks({
    resolve(specifier, context, next) {
      if (specifier !== "cloudflare:workers") return next(specifier, context);
      const src = "export class DurableObject { constructor(ctx, env) { this.ctx = ctx; this.env = env; } }";
      return { url: "data:text/javascript," + encodeURIComponent(src), shortCircuit: true };
    },
  });
  const { Hub } = await import("../src/hub.js");
  const realFetch = globalThis.fetch;
  const seen = [];
  globalThis.fetch = async (url, init) => {
    seen.push(init.headers["User-Agent"]);
    return new Response(JSON.stringify(ROWS.slice(0, 2)));
  };
  try {
    const hub = new Hub({ storage: { get: async () => undefined, put: async () => {} }, getWebSockets: () => [] },
                        { GEOCODE_CONTACT: "ops@example.org" });
    const res = await hub.fetch(new Request("https://hub/geocode?" + new URLSearchParams({ q: "Cần Thơ", sid: "abc" })));
    assert.equal(res.status, 200);
    assert.equal((await res.json()).results.length, 2);
    assert.match(seen[0], /mliot-redgescue\/1\.0 .*ops@example\.org/);
    const fast = await hub.fetch(new Request("https://hub/geocode?" + new URLSearchParams({ q: "Vĩnh Long", sid: "abc" })));
    assert.equal(fast.status, 429);
  } finally {
    globalThis.fetch = realFetch;
  }
});
