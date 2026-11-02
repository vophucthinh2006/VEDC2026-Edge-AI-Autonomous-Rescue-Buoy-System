// Một người, một phao (docs/victim-claim.md): Hub gộp báo cáo của mọi phao, phao báo đầu là chủ, nút xác nhận
// của chủ là đã cứu, phao mất liên lạc hay bị gọi về thì nhả, admin đánh dấu / nhả / giao; trạm nhận "claims".
// Chạy:  node --test test/claims.test.mjs
import assert from "node:assert/strict";
import { registerHooks } from "node:module";
import test from "node:test";

registerHooks({
  resolve(specifier, context, next) {
    if (specifier !== "cloudflare:workers") return next(specifier, context);
    const src = "export class DurableObject { constructor(ctx, env) { this.ctx = ctx; this.env = env; } }";
    return { url: "data:text/javascript," + encodeURIComponent(src), shortCircuit: true };
  },
});
const { Hub } = await import("../src/hub.js");
const { sanitizeVictimAction } = await import("../src/worker.js");

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

const PERSON = { lat: 10.8834, lon: 106.7961 };
const report = (id, seq, extra = {}) => ({ id, fix: 0, rssi: -80, snr: 6, raw: "EVT",
  event: { seq, type: "V", ...PERSON, risk: 2, conf: 0.7, count: 1, ...extra } });
const press = (id, seq, at = PERSON) => ({ id, fix: 0, rssi: -80, snr: 6, raw: "EVT",
  event: { seq, type: "C", lat: at ? at.lat : null, lon: at ? at.lon : null, presses: 1 } });
const beaconOf = id => ({ id, fix: 1, lat: 10.8834, lon: 106.7961, rssi: -80, snr: 7, raw: "" });
const tasks = sent => Object.fromEntries(sent.filter(m => m.type === "task").map(m => [m.id, m]));

test("the first boat to report a person owns them; the others only add to who saw them", async () => {
  const { hub, sent } = makeHub();
  await call(hub, "POST", "/ingest", report("PHAO-02", 1));
  await call(hub, "POST", "/ingest", report("PHAO-01", 1, { lat: 10.88343, risk: 3 }));   // ~3 m off
  await call(hub, "POST", "/ingest", report("PHAO-03", 1, { lat: 10.88342 }));
  const { victims } = await call(hub, "GET", "/victims");
  assert.equal(victims.length, 1);
  assert.equal(victims[0].owner, "PHAO-02");
  assert.equal(victims[0].status, "claimed");
  assert.deepEqual(victims[0].seen_by, ["PHAO-02", "PHAO-01", "PHAO-03"]);
  assert.equal(victims[0].risk, 2);                        // only the owner's sightings change it
  assert.deepEqual(tasks(sent)["PHAO-02"].ref, { vid: victims[0].vid });
  assert.equal(tasks(sent)["PHAO-01"], undefined);
  // The station hears of them: [lat, lon, owner, 1]
  assert.deepEqual((await call(hub, "GET", "/downlink")).claims, [[PERSON.lat, PERSON.lon, "PHAO-02", 1]]);
});

test("the owner's confirm button by the person rescues them and gives the boat its route back", async () => {
  const { hub, sent } = makeHub();
  await call(hub, "PUT", "/downlink", { id: "PHAO-01", pts: [[10.88, 106.79], [10.881, 106.79]] });
  await call(hub, "POST", "/ingest", report("PHAO-01", 1));
  assert.equal(tasks(sent)["PHAO-01"].kind, "victim");
  // Another boat's button, or the owner's far away, does nothing
  await call(hub, "POST", "/ingest", press("PHAO-02", 2));
  await call(hub, "POST", "/ingest", press("PHAO-01", 3, { lat: 10.8836, lon: 106.7961 }));   // ~22 m
  assert.equal((await call(hub, "GET", "/victims")).victims[0].status, "claimed");
  await call(hub, "POST", "/ingest", press("PHAO-01", 4, null));                                // no GPS: its newest
  const [v] = (await call(hub, "GET", "/victims")).victims;
  assert.equal(v.status, "rescued");
  assert.equal(tasks(sent)["PHAO-01"].kind, "sector");
  assert.deepEqual((await call(hub, "GET", "/downlink")).claims, [[PERSON.lat, PERSON.lon, "PHAO-01", 2]]);
  // Reported again later: still rescued, nobody takes them
  await call(hub, "POST", "/ingest", report("PHAO-03", 5));
  assert.equal((await call(hub, "GET", "/victims")).victims[0].status, "rescued");
});

test("a boat lost or called home lets go of its people; the next boat to report them takes them", async () => {
  const { hub, sent } = makeHub();
  const realNow = Date.now;
  let now = realNow();
  Date.now = () => now;
  try {
    await call(hub, "POST", "/ingest", beaconOf("PHAO-01"));
    await call(hub, "POST", "/ingest", beaconOf("PHAO-02"));
    await call(hub, "POST", "/ingest", report("PHAO-01", 1));
    now += 20_000;
    await call(hub, "POST", "/ingest", beaconOf("PHAO-02"));
    now += 15_000;
    await call(hub, "GET", "/downlink");                    // PHAO-01 silent 35 s: lost
    let [v] = (await call(hub, "GET", "/victims")).victims;
    assert.deepEqual([v.status, v.owner], ["released", null]);
    assert.deepEqual((await call(hub, "GET", "/downlink")).claims, [[PERSON.lat, PERSON.lon, "", 0]]);
    await call(hub, "POST", "/ingest", report("PHAO-02", 1));
    [v] = (await call(hub, "GET", "/victims")).victims;
    assert.deepEqual([v.status, v.owner], ["claimed", "PHAO-02"]);
    // Called home: let go again
    await call(hub, "PUT", "/return", { id: "PHAO-02", ret: true });
    [v] = (await call(hub, "GET", "/victims")).victims;
    assert.equal(v.status, "released");
    assert.equal(tasks(sent)["PHAO-02"].kind, "return");
    // Released people drop out of the station's list after 2 min
    now += 3 * 60_000;
    assert.equal((await call(hub, "GET", "/downlink")).claims, undefined);
  } finally {
    Date.now = realNow;
  }
});

test("the admin marks a person rescued, lets them go, or gives them to a boat with a route to them", async () => {
  const { hub, sent } = makeHub();
  await call(hub, "POST", "/ingest", report("PHAO-01", 1));
  const vid = (await call(hub, "GET", "/victims")).victims[0].vid;
  let m = await call(hub, "PUT", "/victim", { vid, action: "release", by: "admin" });
  assert.deepEqual([m.victim.status, m.victim.owner, m.victim.status_by], ["released", null, "admin"]);
  assert.equal(tasks(sent)["PHAO-01"].kind, "idle");
  m = await call(hub, "PUT", "/victim", { vid, action: "assign", id: "PHAO-03", by: "admin" });
  assert.deepEqual([m.victim.status, m.victim.owner], ["claimed", "PHAO-03"]);
  const dl = await call(hub, "GET", "/downlink");
  assert.deepEqual(dl.routes.find(r => r.id === "PHAO-03").pts, [[PERSON.lat, PERSON.lon]]);
  assert.deepEqual(tasks(sent)["PHAO-03"].ref, { vid });
  m = await call(hub, "PUT", "/victim", { vid, action: "rescued", by: "admin" });
  assert.equal(m.victim.status, "rescued");
  assert.equal(tasks(sent)["PHAO-03"].kind, "sector");
  assert.equal((await call(hub, "PUT", "/victim", { vid: 999, action: "rescued" })).error, "no such person");
  assert.equal((await call(hub, "PUT", "/victim", { vid, action: "assign" })).error, "bad request");
});

test("the admin's word on a person is checked by the Worker", () => {
  assert.deepEqual(sanitizeVictimAction({ action: "rescued", id: "x" }), { action: "rescued" });
  assert.deepEqual(sanitizeVictimAction({ action: "assign", id: "PHAO-02<" }), { action: "assign", id: "PHAO-02" });
  assert.equal(sanitizeVictimAction({ action: "assign" }), null);
  assert.equal(sanitizeVictimAction({ action: "sink" }), null);
});
