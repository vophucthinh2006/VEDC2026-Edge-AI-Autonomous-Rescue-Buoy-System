// Thời tiết, mưa, ngập cho bản đồ (docs/weather-map.md): phân tích mực nước VNDMS, dự báo Open-Meteo, khung radar
// RainViewer, bộ nhớ đệm và bản cũ khi nguồn hỏng, mực nước cho lớp độ sâu.
// Chạy:  node --test test/weather.test.mjs
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
const { Weather, mergeGauges, nearestGauge, parseLevel, parseStation, toForecast, toRadar } = await import("../src/weather.js");
const { Hub } = await import("../src/hub.js");
const { sanitizeFloodLevel } = await import("../src/worker.js");

// The popup of a VNDMS station as the site sends it (03/11/2026), the chart image cut
const popup = (code, name, level) => `<div class="station-popup-info"><div class="row-popup"><div class="col-popup-left"><li>Tên trạm: <b>${name}</b> </li>`
  + `<li>Mã trạm: <b>${code}</b></li><li>Địa điểm: <b>TP. Đà Nẵng</b></li><li>   Sông: <b>Vu Gia</b></li><li>Nguồn: <b>KTTV</b></li>`
  + `<li>Cảnh báo vượt mức báo động: <b>-</b></li><li><b>${level}</b> </li><li class='detalRain' data-id='${code}'></li></div><li><img></li></div></div>`;
const feature = (code, name, level, lon = 108.117, lat = 15.883) => ({ type: "Feature", geometry: { type: "Point", coordinates: [lon, lat] },
  properties: { longtitude: lon, latitude: lat, label: name, popupInfo: popup(code, name, level) } });
const NOW = Date.UTC(2026, 10, 3, 7);   // 14 h on 3 November in Vietnam

test("a VNDMS water level is read with its time in Vietnam", () => {
  assert.deepEqual(parseLevel("<b>Mực nước (2.58(m) 13-03/11)</b>", NOW), { level_m: 2.58, at: Date.UTC(2026, 10, 3, 6) });
  assert.equal(parseLevel("<b>Mực nước (-0.4(m) 7-03/11)</b>", NOW).level_m, -0.4);
  assert.equal(parseLevel("<b>Mực nước :Không có số liệu</b>", NOW), null);
  // Read on 1 January, the reading of 31 December is last year's
  assert.equal(parseLevel("Mực nước (1.2(m) 23-31/12)", Date.UTC(2027, 0, 1, 2)).at, Date.UTC(2026, 11, 31, 16));
});

test("stations get their alarm level from the lv=1..3 lists, a station without a reading has no level", () => {
  const s = parseStation(feature("71527", "Ái Nghĩa", "Mực nước (2.58(m) 13-03/11)"), NOW);
  assert.deepEqual([s.code, s.name, s.river, s.province, s.level_m, s.alarm], ["71527", "Ái Nghĩa", "Vu Gia", "TP. Đà Nẵng", 2.58, 0]);
  const lists = [
    { features: [feature("1", "A", "Mực nước (2(m) 13-03/11)"), feature("2", "B", "Mực nước :Không có số liệu"), { geometry: {}, properties: {} }] },
    { features: [feature("1", "A", "Mực nước (2(m) 13-03/11)")] }, { features: [] }, { features: [feature("1", "A", "Mực nước (2(m) 13-03/11)")] },
  ];
  const all = mergeGauges(lists, NOW);
  assert.deepEqual(all.map(g => [g.code, g.alarm, g.level_m]), [["1", 3, 2], ["2", 0, null]]);
  assert.equal(nearestGauge(all, { lat: 15.9, lon: 108.1 }).code, "1");   // the one with a reading
});

test("the forecast keeps the next 24 hours, in UTC", () => {
  const time = Array.from({ length: 48 }, (_, i) => `2026-11-03T${String(i % 24).padStart(2, "0")}:00`.replace("11-03T", i < 24 ? "11-03T" : "11-04T"));
  const body = { utc_offset_seconds: 25200, hourly: { time, precipitation: time.map((_, i) => i / 10), precipitation_probability: time.map(() => 40),
    wind_speed_10m: time.map(() => 12.34) } };
  const hours = toForecast(body, NOW);
  assert.equal(hours.length, 24);
  assert.equal(hours[0].t, Date.UTC(2026, 10, 3, 6));      // 13 h Vietnam: from an hour ago on
  assert.deepEqual([hours[0].rain_mm, hours[0].prob, hours[0].wind_kmh], [1.3, 40, 12.3]);
  assert.equal(toForecast({}, NOW), null);
});

test("radar frames: only well-formed paths", () => {
  const r = toRadar({ host: "https://tilecache.rainviewer.com", radar: { past: [{ time: 1, path: "/v2/radar/abc123" }, { time: 2, path: "../x" }] } });
  assert.deepEqual(r, { host: "https://tilecache.rainviewer.com", frames: [{ time: 1000, path: "/v2/radar/abc123" }] });
  assert.equal(toRadar({ host: "x", radar: { past: [] } }), null);
});

function fakeFetch(routes) {
  const calls = [];
  const f = async (url, init) => {
    calls.push({ url, init });
    const key = Object.keys(routes).find(k => url.includes(k));
    const body = key && routes[key]();
    if (body === undefined) return new Response("down", { status: 503 });
    return new Response(JSON.stringify(body), { status: 200 });
  };
  f.calls = calls;
  return f;
}

test("gauges are fetched with the Referer VNDMS wants, cached, and given stale while it is down", async () => {
  let now = NOW, up = true;
  const lists = lv => ({ features: lv === 0 ? [feature("1", "A", "Mực nước (2(m) 13-03/11)")] : [] });
  const fetch = fakeFetch({ "lv=0": () => (up ? lists(0) : undefined), "lv=1": () => (up ? lists(1) : undefined),
    "lv=2": () => (up ? lists(2) : undefined), "lv=3": () => (up ? lists(3) : undefined) });
  const w = new Weather({ fetch, now: () => now });
  const a = await w.gaugeList();
  assert.equal(a.value.length, 1);
  assert.equal(fetch.calls[0].init.headers.Referer, "https://vndms.gov.vn/");
  await w.gaugeList();
  assert.equal(fetch.calls.length, 4);              // cached
  now += 20 * 60e3; up = false;
  const b = await w.gaugeList();
  assert.equal(b.stale, true);
  assert.equal(b.value.length, 1);
  now += 7 * 3600e3;
  assert.equal(await w.gaugeList(), null);          // too old to give
});

test("the Hub answers the weather at a point, and the admin picks the flood layer's water level", async () => {
  const store = new Map(), sent = [];
  const ctx = { storage: { async get(k) { return structuredClone(store.get(k)); }, async put(k, v) { store.set(k, structuredClone(v)); } },
    getWebSockets: () => [{ send: s => sent.push(JSON.parse(s)) }] };
  const hub = new Hub(ctx, {});
  const time = Array.from({ length: 48 }, (_, i) => new Date(NOW + (i - 1) * 3600e3 + 7 * 3600e3).toISOString().slice(0, 16));
  hub.weather = new Weather({ now: () => NOW, fetch: fakeFetch({
    "open-meteo": () => ({ utc_offset_seconds: 25200, hourly: { time, precipitation: time.map(() => 2), precipitation_probability: time.map(() => 80), wind_speed_10m: time.map(() => 20) } }),
    "rainviewer": () => ({ host: "https://tilecache.rainviewer.com", radar: { past: [{ time: 1, path: "/v2/radar/abc" }] } }),
    "lv=0": () => ({ features: [feature("near", "Gần", "Mực nước (1.5(m) 13-03/11)", 106.66, 10.76), feature("far", "Xa", "Mực nước (3(m) 13-03/11)")] }),
    "lv=": () => ({ features: [] }),
  }) });
  const call = async (method, path, body) => (await hub.fetch(new Request("https://hub" + path, { method, body: body && JSON.stringify(body) }))).json();
  let w = await call("GET", "/weather?lat=10.77&lon=106.67");
  assert.equal(w.forecast.hours.length, 24);
  assert.equal(w.radar.frames.length, 1);
  assert.deepEqual([w.flood.source, w.flood.code, w.flood.level_m], ["nearest", "near", 1.5]);
  assert.equal((await call("GET", "/weather?lat=x&lon=1")).error, "bad point");
  // The admin picks another gauge, then types a level, then goes back to the nearest one
  let r = await call("PUT", "/flood-level", { code: "far", by: "admin" });
  assert.deepEqual([r.flood.source, r.flood.code, r.flood.level_m], ["gauge", "far", 3]);
  assert.equal(sent.at(-1).type, "flood");
  w = await call("GET", "/weather?lat=10.77&lon=106.67");
  assert.equal(w.flood.code, "far");
  r = await call("PUT", "/flood-level", { level_m: 2.2, by: "admin" });
  assert.deepEqual([r.flood.source, r.flood.level_m], ["manual", 2.2]);
  r = await call("PUT", "/flood-level", {});
  assert.equal(r.flood.source, "nearest");
  assert.equal((await call("GET", "/gauges")).stations.length, 2);
});

test("the Worker checks the admin's flood level", () => {
  assert.deepEqual(sanitizeFloodLevel({ code: "71527<" }), { code: "71527" });
  assert.deepEqual(sanitizeFloodLevel({ level_m: 2.345 }), { level_m: 2.35 });
  assert.equal(sanitizeFloodLevel({ level_m: 99 }), null);
  assert.equal(sanitizeFloodLevel({ level_m: "2" }), null);
  assert.deepEqual(sanitizeFloodLevel({}), {});
});
