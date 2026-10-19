// Kiểm thử Hub (Durable Object) chạy thẳng trong Node, không cần `wrangler dev`.
// Chạy:  node --test test/hub.test.mjs
import assert from "node:assert/strict";
import { registerHooks } from "node:module";
import test from "node:test";

// "cloudflare:workers" chỉ có trong runtime Workers: thay bằng lớp DurableObject tối thiểu
registerHooks({
  resolve(specifier, context, next) {
    if (specifier !== "cloudflare:workers") return next(specifier, context);
    const src = "export class DurableObject { constructor(ctx, env) { this.ctx = ctx; this.env = env; } }";
    return { url: "data:text/javascript," + encodeURIComponent(src), shortCircuit: true };
  },
});
const { Hub } = await import("../src/hub.js");

// Kho lưu trữ giả giữ nguyên qua các lần tạo Hub, như Durable Object bị gỡ khỏi bộ nhớ rồi dựng lại
function makeHub(store = new Map()) {
  const sent = [];
  const storage = {
    async get(key) { return structuredClone(store.get(key)); },
    async put(key, value) {
      const entries = typeof key === "string" ? [[key, value]] : Object.entries(key);
      for (const [k, v] of entries) store.set(k, structuredClone(v));
    },
  };
  const ctx = { storage, getWebSockets: () => [{ send: s => sent.push(JSON.parse(s)) }] };
  return { hub: new Hub(ctx, {}), store, sent };
}

async function call(hub, method, path, body) {
  const res = await hub.fetch(new Request("https://hub" + path, { method, body: body === undefined ? undefined : JSON.stringify(body) }));
  return res.json();
}

const WAYPOINTS = [{ lat: 10.8834, lon: 106.7961 }, { lat: 10.8835, lon: 106.7962 }, { lat: 10.8836, lon: 106.7960 }];
const beacon = route => ({ id: "PHAO-01", fix: 1, lat: 10.8834, lon: 106.7961, rssi: -80, snr: 7, raw: "", route });

async function sendRoute(hub, extra = {}) {
  await call(hub, "PUT", "/mission", { mission: { waypoints: WAYPOINTS, poly: [], spacing: 10, angle: null, speed: 1.5 } });
  return call(hub, "PUT", "/downlink", { id: "PHAO-01", loop: false, ...extra });
}

test("offers nothing before the first send", async () => {
  const { hub } = makeHub();
  assert.deepEqual(await call(hub, "GET", "/downlink"), { rev: 0 });
});

test("keeps offering a route until the Pi has all of it", async () => {
  const { hub } = makeHub();
  const sent = await sendRoute(hub);
  assert.equal(sent.rev, 1);
  assert.equal(sent.delivered, null);
  for (const route of [undefined, { rev: 1, have: 2, pi: 0 }, { rev: 1, have: 3, pi: 0 }]) {
    await call(hub, "POST", "/ingest", beacon(route));
    const dl = await call(hub, "GET", "/downlink");
    assert.equal(dl.rev, 1);
    assert.equal(dl.pts.length, 3);
  }
});

test("a delivered route is not offered again, even after the hub reloads", async () => {
  const { hub, store, sent } = makeHub();
  await sendRoute(hub);
  // Ack from another buoy or for another rev does not count
  await call(hub, "POST", "/ingest", { ...beacon({ rev: 1, have: 3, pi: 1 }), id: "PHAO-02" });
  await call(hub, "POST", "/ingest", beacon({ rev: 9, have: 3, pi: 1 }));
  assert.equal((await call(hub, "GET", "/downlink")).rev, 1);

  await call(hub, "POST", "/ingest", beacon({ rev: 1, have: 3, pi: 1 }));
  assert.deepEqual(await call(hub, "GET", "/downlink"), { rev: 0 });
  // The dashboard hears it
  const summary = sent.filter(m => m.type === "downlink").at(-1);
  assert.equal(summary.rev, 1);
  assert.equal(typeof summary.delivered, "number");

  // A freshly booted F407 sends no w=: still nothing to offer
  await call(hub, "POST", "/ingest", beacon(undefined));
  assert.deepEqual(await call(hub, "GET", "/downlink"), { rev: 0 });
  const { hub: reloaded } = makeHub(store);
  assert.deepEqual(await call(reloaded, "GET", "/downlink"), { rev: 0 });
});

test("a new send after delivery is offered with the next rev", async () => {
  const { hub } = makeHub();
  await sendRoute(hub);
  await call(hub, "POST", "/ingest", beacon({ rev: 1, have: 3, pi: 1 }));
  const again = await sendRoute(hub);
  assert.equal(again.rev, 2);
  assert.equal(again.delivered, null);
  const dl = await call(hub, "GET", "/downlink");
  assert.equal(dl.rev, 2);
  assert.equal(dl.pts.length, 3);
});

test("an empty route (stop) is offered, then delivered like any other", async () => {
  const { hub } = makeHub();
  await sendRoute(hub);
  await call(hub, "POST", "/ingest", beacon({ rev: 1, have: 3, pi: 1 }));
  const stop = await sendRoute(hub, { clear: true });
  assert.equal(stop.n, 0);
  const dl = await call(hub, "GET", "/downlink");
  assert.equal(dl.rev, 2);
  assert.deepEqual(dl.pts, []);
  await call(hub, "POST", "/ingest", beacon({ rev: 2, have: 0, pi: 0 }));
  assert.equal((await call(hub, "GET", "/downlink")).rev, 2);
  await call(hub, "POST", "/ingest", beacon({ rev: 2, have: 0, pi: 1 }));
  assert.deepEqual(await call(hub, "GET", "/downlink"), { rev: 0 });
});
