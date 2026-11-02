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
  // Same spot seen by another buoy: the same person, still the first buoy's (docs/victim-claim.md)
  await call(hub, "POST", "/ingest", { ...victim(1), id: "PHAO-02" });
  ({ victims } = await call(hub, "GET", "/victims"));
  assert.deepEqual(victims.map(v => v.buoy), ["PHAO-01", "PHAO-01"]);
  assert.deepEqual(victims[0].seen_by, ["PHAO-01", "PHAO-02"]);
  assert.equal(victims[0].owner, "PHAO-01");
  // Survives a reload
  const { hub: reloaded } = makeHub(store);
  assert.equal((await call(reloaded, "GET", "/victims")).victims.length, 2);
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

// ---- Đội tàu: mỗi phao một bản gửi, lệnh về bờ, lệnh cả đội, điểm đỗ, mất liên lạc ----

const beaconOf = (id, route, extra = {}) => ({ ...beacon(route), id, ...extra });

test("routes for two buoys do not overwrite each other, and are delivered one by one", async () => {
  const { hub, store } = makeHub();
  await sendRoute(hub);
  const own = [[10.8840, 106.7970], [10.8841, 106.7971]];
  const two = await call(hub, "PUT", "/downlink", { id: "PHAO-02", loop: true, pts: own });
  assert.equal(two.rev, 2);
  let dl = await call(hub, "GET", "/downlink");
  assert.deepEqual(dl.routes.map(r => [r.id, r.rev, r.pts.length]), [["PHAO-01", 1, 3], ["PHAO-02", 2, 2]]);
  assert.equal(dl.id, "PHAO-02");                 // an old station reads the newest one at the top
  assert.deepEqual(dl.routes[1].pts, own);

  await call(hub, "POST", "/ingest", beaconOf("PHAO-02", { rev: 2, have: 2, pi: 1 }));
  dl = await call(hub, "GET", "/downlink");
  assert.deepEqual(dl.routes.map(r => r.id), ["PHAO-01"]);
  assert.equal(dl.id, "PHAO-01");
  await call(hub, "POST", "/ingest", beacon({ rev: 1, have: 3, pi: 1 }));
  assert.deepEqual(await call(hub, "GET", "/downlink"), { rev: 0 });
  // A new send still gets a rev no buoy has seen
  const { hub: reloaded } = makeHub(store);
  assert.equal((await sendRoute(reloaded)).rev, 3);
});

test("a hub stored by the old code (one downlink) keeps offering that route", async () => {
  const store = new Map([["downlink", { rev: 7, id: "PHAO-01", loop: false, pts: [[10.88, 106.79]], at: 1 }]]);
  const { hub } = makeHub(store);
  const dl = await call(hub, "GET", "/downlink");
  assert.equal(dl.rev, 7);
  assert.deepEqual(dl.routes.map(r => r.rev), [7]);
  assert.equal((await call(hub, "PUT", "/downlink", { id: "PHAO-02", pts: [] })).rev, 8);
});

test("return home rides along the downlink, changes the task, and a new route lifts it", async () => {
  const { hub, sent } = makeHub();
  await sendRoute(hub);
  assert.deepEqual(sent.filter(m => m.type === "task").at(-1), { type: "task", id: "PHAO-01", kind: "sector", ref: { rev: 1, n: 3 }, since: sent.filter(m => m.type === "task").at(-1).since });
  const m = await call(hub, "PUT", "/return", { id: "PHAO-01", ret: true, by: "admin@example.com" });
  assert.deepEqual([m.type, m.want], ["ret", true]);
  assert.deepEqual((await call(hub, "GET", "/downlink")).ret, { "PHAO-01": 1 });
  assert.equal(sent.filter(x => x.type === "task").at(-1).kind, "return");
  await sendRoute(hub);
  assert.deepEqual((await call(hub, "GET", "/downlink")).ret, { "PHAO-01": 0 });
  assert.equal(sent.filter(x => x.type === "task").at(-1).kind, "sector");
});

test("fleet commands reach every buoy heard lately, or the ones named", async () => {
  const { hub } = makeHub();
  for (const id of ["PHAO-01", "PHAO-02", "PHAO-03"]) await call(hub, "POST", "/ingest", beaconOf(id));
  const r = await call(hub, "PUT", "/fleet", { action: "hold" });
  assert.deepEqual(r.ids, ["PHAO-01", "PHAO-02", "PHAO-03"]);
  assert.deepEqual((await call(hub, "GET", "/downlink")).hold, { "PHAO-01": 1, "PHAO-02": 1, "PHAO-03": 1 });
  await call(hub, "PUT", "/fleet", { action: "return", ids: ["PHAO-02", "PHAO-03"] });
  assert.deepEqual((await call(hub, "GET", "/downlink")).ret, { "PHAO-02": 1, "PHAO-03": 1 });
  await call(hub, "PUT", "/fleet", { action: "resume" });
  const dl = await call(hub, "GET", "/downlink");
  assert.deepEqual(dl.hold, { "PHAO-01": 0, "PHAO-02": 0, "PHAO-03": 0 });
  assert.deepEqual(dl.ret, { "PHAO-02": 0, "PHAO-03": 0 });
});

test("each buoy gets its own parking spot along the shore, PHAO-01 at the station", async () => {
  const { hub } = makeHub();
  const station = { lat: 10.8830, lon: 106.7960 };
  await call(hub, "POST", "/station", { fix: 1, ...station });
  // The buoys are north of the station: the water is north, the spots run east-west
  for (const [i, id] of ["PHAO-01", "PHAO-02", "PHAO-03"].entries()) {
    await call(hub, "POST", "/ingest", { ...beaconOf(id), lat: 10.8840, lon: 106.7960 + i * 0.0001 });
  }
  const { home } = await call(hub, "GET", "/downlink");
  assert.deepEqual(home["PHAO-01"], [station.lat, station.lon]);
  const d = (a, b) => Math.hypot((a[0] - b[0]) * 111320, (a[1] - b[1]) * 111320 * Math.cos(station.lat * Math.PI / 180));
  assert.ok(Math.abs(d(home["PHAO-02"], home["PHAO-01"]) - 8) < 0.5);
  assert.ok(Math.abs(d(home["PHAO-03"], home["PHAO-01"]) - 8) < 0.5);
  assert.ok(Math.abs(d(home["PHAO-02"], home["PHAO-03"]) - 16) < 0.5);
  assert.ok(Math.abs(home["PHAO-02"][0] - station.lat) < 1e-5);   // along the shore, not out on the water
  // Buoys coming home do not turn the line
  await call(hub, "POST", "/ingest", { ...beaconOf("PHAO-02"), lat: 10.8830, lon: 106.79605 });
  assert.deepEqual((await call(hub, "GET", "/downlink")).home["PHAO-02"], home["PHAO-02"]);
});

test("a buoy silent for 30 s raises one lost alert, cleared when it is heard again", async () => {
  const { hub, sent } = makeHub();
  const realNow = Date.now;
  let now = realNow();
  Date.now = () => now;
  try {
    await call(hub, "POST", "/ingest", beaconOf("PHAO-01"));
    await call(hub, "POST", "/ingest", beaconOf("PHAO-02"));
    now += 20_000;
    await call(hub, "POST", "/ingest", beaconOf("PHAO-01"));
    now += 15_000;
    await call(hub, "GET", "/downlink");
    await call(hub, "GET", "/downlink");
    const alerts = sent.filter(m => m.type === "alert" && m.alert.type === "L");
    assert.deepEqual(alerts.map(m => m.alert.buoy), ["PHAO-02"]);
    assert.deepEqual(sent.filter(m => m.type === "link").map(m => [m.id, m.lost]), [["PHAO-02", true]]);
    await call(hub, "POST", "/ingest", beaconOf("PHAO-02"));
    assert.deepEqual(sent.filter(m => m.type === "link").map(m => [m.id, m.lost]), [["PHAO-02", true], ["PHAO-02", false]]);
  } finally {
    Date.now = realNow;
  }
});

// ---- Kế hoạch đội tàu: chia vùng, đội hình, bầu lại phao dẫn đầu ----

const kx = 111320 * Math.cos(10.8834 * Math.PI / 180), ky = 111320;
const at = (x, y) => ({ lat: 10.8834 + y / ky, lon: 106.7960 + x / kx });
const AREA = [at(0, 0), at(120, 0), at(120, 60), at(0, 60)];

test("a sector plan sends each buoy its own strip; dry only shows it", async () => {
  const { hub } = makeHub();
  for (const [i, id] of ["PHAO-01", "PHAO-02", "PHAO-03"].entries()) {
    await call(hub, "POST", "/ingest", { ...beaconOf(id), ...at(0, 55 - i * 25) });
  }
  const dry = await call(hub, "PUT", "/plan", { mode: "sectors", poly: AREA, ids: ["PHAO-01", "PHAO-02", "PHAO-03"], spacing: 10, dry: true });
  assert.equal(dry.routes.length, 3);
  assert.deepEqual(await call(hub, "GET", "/downlink"), { rev: 0 });
  await call(hub, "PUT", "/plan", { mode: "sectors", poly: AREA, ids: ["PHAO-01", "PHAO-02", "PHAO-03"], spacing: 10 });
  const dl = await call(hub, "GET", "/downlink");
  assert.deepEqual(dl.routes.map(r => r.id).sort(), ["PHAO-01", "PHAO-02", "PHAO-03"]);
  assert.ok(dl.routes.every(r => r.pts.length === 4));
  assert.equal(dl.fol, undefined);
});

test("a formation sends the route to the leader, places to the followers, and a new leader takes over when it is lost", async () => {
  const { hub, sent } = makeHub();
  const realNow = Date.now;
  let now = realNow();
  Date.now = () => now;
  try {
    for (const id of ["PHAO-01", "PHAO-02", "PHAO-03"]) await call(hub, "POST", "/ingest", { ...beaconOf(id), ...at(0, 0) });
    const f = await call(hub, "PUT", "/plan", { mode: "formation", poly: AREA, ids: ["PHAO-01", "PHAO-02", "PHAO-03"], leader: "PHAO-02", shape: "line", spacing: 10 });
    assert.equal(f.leader, "PHAO-02");
    let dl = await call(hub, "GET", "/downlink");
    assert.deepEqual(dl.fol, { "PHAO-01": ["PHAO-02", 10, 0], "PHAO-03": ["PHAO-02", -10, 0] });
    const leaderRoute = dl.routes.find(r => r.id === "PHAO-02");
    assert.equal(leaderRoute.pts.length, 4);                     // two lanes 30 m apart over the 60 m
    assert.deepEqual(dl.routes.filter(r => r.id !== "PHAO-02").map(r => r.pts.length), [0, 0]);
    // The leader reports it is going to waypoint 3, then falls silent
    await call(hub, "POST", "/ingest", { id: "PHAO-02", fix: 0, rssi: -80, snr: 7, raw: "", diag: { reason: "", mission: "P", wp: 3, wp_total: 4, arm: 0, drive: 0, i2c: 0, imu: 0 } });
    now += 20_000;
    for (const id of ["PHAO-01", "PHAO-03"]) await call(hub, "POST", "/ingest", { ...beaconOf(id), ...at(0, 0) });
    now += 15_000;
    dl = await call(hub, "GET", "/downlink");
    assert.deepEqual(dl.fol, { "PHAO-03": ["PHAO-01", 10, 0] });
    const taken = dl.routes.find(r => r.id === "PHAO-01");
    assert.deepEqual(taken.pts, leaderRoute.pts.slice(2));          // what the old leader had not done yet
    assert.ok(sent.some(m => m.type === "alert" && m.alert.type === "E" && m.alert.buoy === "PHAO-01" && m.alert.replaces === "PHAO-02"));
    // A sector plan ends the formation
    await call(hub, "PUT", "/plan", { mode: "sectors", poly: AREA, ids: ["PHAO-01", "PHAO-03"], spacing: 10 });
    assert.equal((await call(hub, "GET", "/downlink")).fol, undefined);
  } finally {
    Date.now = realNow;
  }
});
