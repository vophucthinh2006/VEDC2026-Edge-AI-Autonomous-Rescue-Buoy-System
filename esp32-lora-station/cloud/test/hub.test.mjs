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

const victim = (seq, extra = {}) => ({ id: "PHAO-01", fix: 0, rssi: -80, snr: 6, raw: "EVT",
  event: { seq, type: "V", lat: 10.8834, lon: 106.7961, risk: 1, conf: 0.6, count: 1, ...extra } });
const alertFrame = event => ({ ...victim(0), event });

test("an event-only frame does not touch the buoy's position or state", async () => {
  const { hub, sent } = makeHub();
  await call(hub, "POST", "/ingest", beacon(undefined));
  await call(hub, "POST", "/ingest", victim(1));
  const [buoy] = await call(hub, "GET", "/state");
  assert.equal(buoy.fix, 1);
  assert.equal(Object.hasOwn(buoy, "event"), false);
  assert.equal(sent.filter(m => m.id === "PHAO-01" && !m.type).length, 1);
});

test("a repeated event (same id, seq) is recorded once", async () => {
  const { hub, sent } = makeHub();
  assert.equal((await call(hub, "POST", "/ingest", victim(1))).event, "new");
  assert.equal((await call(hub, "POST", "/ingest", victim(1, { risk: 3 }))).event, "dup");
  const { victims } = await call(hub, "GET", "/victims");
  assert.equal(victims.length, 1);
  assert.equal(victims[0].risk, 1);
  assert.equal(sent.filter(m => m.type === "victim").length, 1);
  // Another buoy may use the same seq
  assert.equal((await call(hub, "POST", "/ingest", { ...victim(1), id: "PHAO-02" })).event, "new");
});

test("a detection within 5 m of a stored person updates it, farther away adds one", async () => {
  const { hub, store, sent } = makeHub();
  await call(hub, "POST", "/ingest", victim(1));
  // ~3 m north: same person, risk and confidence updated
  await call(hub, "POST", "/ingest", victim(2, { lat: 10.88343, risk: 3, conf: 0.9 }));
  let { victims } = await call(hub, "GET", "/victims");
  assert.equal(victims.length, 1);
  assert.equal(victims[0].risk, 3);
  assert.equal(victims[0].conf, 0.9);
  assert.equal(victims[0].lat, 10.8834);
  assert.ok(victims[0].last_at >= victims[0].first_at);
  const update = sent.filter(m => m.type === "victim").at(-1);
  assert.equal(update.new, false);
  assert.equal(update.victim.vid, victims[0].vid);
  // ~11 m away: a second person
  await call(hub, "POST", "/ingest", victim(3, { lat: 10.8835 }));
  // Same spot seen by another buoy: kept as its own dot
  await call(hub, "POST", "/ingest", { ...victim(1), id: "PHAO-02" });
  ({ victims } = await call(hub, "GET", "/victims"));
  assert.deepEqual(victims.map(v => v.buoy), ["PHAO-01", "PHAO-01", "PHAO-02"]);
  // Survives a reload
  const { hub: reloaded } = makeHub(store);
  assert.equal((await call(reloaded, "GET", "/victims")).victims.length, 3);
});

test("overturn, confirm and a person without a position become alerts", async () => {
  const { hub, sent } = makeHub();
  await call(hub, "POST", "/ingest", victim(1, { lat: null, lon: null }));
  await call(hub, "POST", "/ingest", alertFrame({ seq: 2, type: "T", lat: 10.8834, lon: 106.7961, roll: 120, pitch: 5 }));
  await call(hub, "POST", "/ingest", alertFrame({ seq: 3, type: "C", lat: null, lon: null, presses: 2 }));
  const { victims, alerts } = await call(hub, "GET", "/victims");
  assert.equal(victims.length, 0);
  assert.deepEqual(alerts.map(a => a.type), ["V", "T", "C"]);
  assert.equal(alerts[1].roll, 120);
  assert.equal(alerts[1].buoy, "PHAO-01");
  assert.equal(alerts[2].presses, 2);
  assert.equal(sent.filter(m => m.type === "alert").length, 3);
});

test("the alert list is bounded", async () => {
  const { hub } = makeHub();
  for (let seq = 1; seq <= 60; seq++) await call(hub, "POST", "/ingest", alertFrame({ seq, type: "C", lat: null, lon: null, presses: 1 }));
  const { alerts } = await call(hub, "GET", "/victims");
  assert.equal(alerts.length, 50);
  assert.equal(alerts[0].seq, 11);
});

test("clearing removes the dots, a repeated event does not bring one back", async () => {
  const { hub, sent } = makeHub();
  await call(hub, "POST", "/ingest", victim(1));
  assert.deepEqual(await call(hub, "DELETE", "/victims"), { type: "victims", victims: [], alerts: [] });
  assert.equal(sent.at(-1).type, "victims");
  await call(hub, "POST", "/ingest", victim(1));
  assert.equal((await call(hub, "GET", "/victims")).victims.length, 0);
  await call(hub, "POST", "/ingest", victim(2));
  assert.equal((await call(hub, "GET", "/victims")).victims.length, 1);
});

test("light commands ride along the downlink, even with no route", async () => {
  const { hub, store, sent } = makeHub();
  const m = await call(hub, "PUT", "/lights", { id: "PHAO-01", lights: 2, by: "admin@example.com" });
  assert.equal(m.want, 2);
  assert.equal(m.have, null);
  assert.equal(sent.at(-1).type, "lights");
  assert.deepEqual(await call(hub, "GET", "/downlink"), { rev: 0, lights: { "PHAO-01": 2 } });
  await call(hub, "PUT", "/lights", { id: "PHAO-02", lights: 0 });
  const { hub: reloaded } = makeHub(store);
  assert.deepEqual(await call(reloaded, "GET", "/downlink"), { rev: 0, lights: { "PHAO-01": 2, "PHAO-02": 0 } });
});

test("changing lights does not touch the route's delivered state", async () => {
  const { hub } = makeHub();
  await sendRoute(hub);
  await call(hub, "PUT", "/lights", { id: "PHAO-01", lights: 1 });
  const dl = await call(hub, "GET", "/downlink");
  assert.equal(dl.rev, 1);
  assert.equal(dl.pts.length, 3);
  assert.deepEqual(dl.lights, { "PHAO-01": 1 });
  await call(hub, "POST", "/ingest", beacon({ rev: 1, have: 3, pi: 1 }));
  await call(hub, "PUT", "/lights", { id: "PHAO-01", lights: 3 });
  assert.deepEqual(await call(hub, "GET", "/downlink"), { rev: 0, lights: { "PHAO-01": 3 } });
});

test("the dashboard hears when the boat's lights match the command", async () => {
  const { hub, sent } = makeHub();
  await call(hub, "PUT", "/lights", { id: "PHAO-01", lights: 1 });
  await call(hub, "POST", "/ingest", { ...beacon(undefined), lights: 0 });
  await call(hub, "POST", "/ingest", { ...beacon(undefined), lights: 1 });
  await call(hub, "POST", "/ingest", { ...beacon(undefined), lights: 1 });   // unchanged: no new message
  assert.deepEqual(sent.filter(m => m.type === "lights").map(m => [m.want, m.have]), [[1, null], [1, 0], [1, 1]]);
});

test("a stop command rides along the downlink next to the lights, and survives a reload", async () => {
  const { hub, store, sent } = makeHub();
  const m = await call(hub, "PUT", "/hold", { id: "PHAO-01", hold: true, by: "admin@example.com" });
  assert.deepEqual([m.type, m.want, m.have], ["hold", true, null]);
  assert.equal(sent.at(-1).type, "hold");
  await call(hub, "PUT", "/lights", { id: "PHAO-01", lights: 1 });
  assert.deepEqual(await call(hub, "GET", "/downlink"), { rev: 0, lights: { "PHAO-01": 1 }, hold: { "PHAO-01": 1 } });
  const { hub: reloaded } = makeHub(store);
  assert.deepEqual((await call(reloaded, "GET", "/downlink")).hold, { "PHAO-01": 1 });
  await call(reloaded, "PUT", "/hold", { id: "PHAO-01", hold: false });
  assert.deepEqual((await call(reloaded, "GET", "/downlink")).hold, { "PHAO-01": 0 });
});

test("stopping keeps the route: it is still offered, and delivered as before", async () => {
  const { hub } = makeHub();
  await sendRoute(hub);
  await call(hub, "PUT", "/hold", { id: "PHAO-01", hold: true });
  const dl = await call(hub, "GET", "/downlink");
  assert.equal(dl.rev, 1);
  assert.equal(dl.pts.length, 3);
  await call(hub, "POST", "/ingest", { ...beacon({ rev: 1, have: 3, pi: 1 }), hold: 1 });
  assert.deepEqual(await call(hub, "GET", "/downlink"), { rev: 0, hold: { "PHAO-01": 1 } });
});

test("the dashboard hears when the boat has really stopped", async () => {
  const { hub, sent } = makeHub();
  await call(hub, "PUT", "/hold", { id: "PHAO-01", hold: true });
  await call(hub, "POST", "/ingest", { ...beacon(undefined), hold: 0 });
  await call(hub, "POST", "/ingest", { ...beacon(undefined), hold: 1 });
  await call(hub, "POST", "/ingest", { ...beacon(undefined), hold: 1 });   // unchanged: no new message
  assert.deepEqual(sent.filter(m => m.type === "hold").map(m => [m.want, m.have]), [[true, null], [true, false], [true, true]]);
});

test("a diagnostics frame is kept beside the buoy, never in place of its beacon", async () => {
  const { hub, store, sent } = makeHub();
  await call(hub, "POST", "/ingest", beacon());
  const diag = { reason: "obstacle_too_close", mission: "P", wp: 2, wp_total: 5, arm: 0, drive: 2, volts: 11.84, amps: -0.35, i2c: 0, imu: 0 };
  const result = await call(hub, "POST", "/ingest", { id: "PHAO-01", fix: 0, rssi: -80, snr: 7, raw: "", diag });
  assert.equal(result.diag, "stored");
  assert.equal(store.get("buoys")["PHAO-01"].msg.lat, 10.8834);         // the beacon is untouched
  assert.deepEqual(sent.at(-1), { type: "diag", id: "PHAO-01", ...diag, age_s: 0 });
  assert.deepEqual(store.get("diag")["PHAO-01"].d, diag);
  // A page opened later gets it with the rest
  const later = makeHub(store);
  const opened = [];
  later.hub.ctx.acceptWebSocket = () => {};
  globalThis.WebSocketPair ??= class { constructor() { return { 0: {}, 1: { send: s => opened.push(JSON.parse(s)) } }; } };
  const res = await later.hub.fetch(new Request("https://hub/ws", { headers: { Upgrade: "websocket" } })).catch(() => null);
  void res;
  assert.ok(opened.some(m => m.type === "diag" && m.id === "PHAO-01" && m.reason === "obstacle_too_close"));
});
