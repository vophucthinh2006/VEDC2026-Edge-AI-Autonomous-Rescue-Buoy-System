import assert from "node:assert/strict";
import test from "node:test";
import { sanitizeBuoy } from "../src/telemetry.js";

const base = { id: "PHAO-01", fix: 1, lat: 10.762622, lon: 106.660172, rssi: -78, snr: 7.5, raw: "frame" };
const attitude = { roll: 2.1, pitch: -1.4, yaw: 278.5, target_yaw: 280, mode: "A", imu_ok: 1, calib: 3, seq: 12 };

test("accepts and preserves a complete attitude frame", () => {
  assert.deepEqual(sanitizeBuoy({ ...base, ...attitude }), { ...base, ...attitude });
});

test("keeps backward compatibility with GPS-only frames", () => {
  assert.deepEqual(sanitizeBuoy(base), base);
});

test("accepts NO_FIX while retaining valid attitude", () => {
  const result = sanitizeBuoy({ ...base, ...attitude, fix: 0, lat: undefined, lon: undefined });
  assert.equal(result.fix, 0);
  assert.equal(result.yaw, 278.5);
  assert.equal(Object.hasOwn(result, "lat"), false);
});

test("drops only the attitude group when it is incomplete or out of range", () => {
  for (const bad of [
    { ...attitude, yaw: 360 },
    { ...attitude, roll: 181 },
    { ...attitude, pitch: 91 },
    { roll: 1 },
    { ...attitude, calib: 4 },
    { ...attitude, mode: "X" },
  ]) {
    assert.deepEqual(sanitizeBuoy({ ...base, ...bad }), { ...base, attitude_bad: 1 });
  }
});

test("keeps a valid victim count and drops a bad one", () => {
  assert.equal(sanitizeBuoy({ ...base, ...attitude, victims: 2 }).victims, 2);
  assert.equal(sanitizeBuoy({ ...base, victims: 0 }).victims, 0);
  for (const bad of [-1, 256, 1.5, "2", null]) {
    assert.deepEqual(sanitizeBuoy({ ...base, victims: bad }), base);
  }
});

test("wraps headings that round up to 360", () => {
  const result = sanitizeBuoy({ ...base, ...attitude, yaw: 359.97, target_yaw: 359.96 });
  assert.equal(result.yaw, 0);
  assert.equal(result.target_yaw, 0);
  assert.equal(sanitizeBuoy({ ...base, ...attitude, target_yaw: -1 }).target_yaw, -1);
});

test("still rejects a frame without signal values", () => {
  assert.equal(sanitizeBuoy({ ...attitude, fix: 0 }), null);
});

const nav = { armed: 1, mode_name: "GUIDED", gs: 1.234, thr: 42.4, wp_dist: 18.37, wp_seq: 0,
              batt_v: 12.6, batt_pct: 100, sats: 10, hdop: 1.21 };
const mav = { id: "SIM-01", link: "mavlink", fix: 1, lat: 10.883438, lon: 106.796019, raw: "", ...attitude, nav };

test("accepts a MAVLink frame without RSSI/SNR and keeps the nav group", () => {
  const result = sanitizeBuoy(mav);
  assert.equal(result.link, "mavlink");
  assert.equal(Object.hasOwn(result, "rssi"), false);
  assert.equal(Object.hasOwn(result, "snr"), false);
  assert.equal(result.yaw, 278.5);
  assert.deepEqual(result.nav, { ...nav, gs: 1.23, thr: 42, wp_dist: 18.4 });
});

test("a LoRa frame never carries a link field and still needs RSSI/SNR", () => {
  assert.equal(Object.hasOwn(sanitizeBuoy(base), "link"), false);
  assert.equal(sanitizeBuoy({ ...base, link: "lora", rssi: undefined }), null);
  assert.equal(sanitizeBuoy({ ...base, link: "wifi" }), null);
});

test("drops only the nav group when it is malformed", () => {
  for (const bad of [
    { ...nav, armed: 2 },
    { ...nav, mode_name: "guided" },
    { ...nav, mode_name: "<script>" },
    { ...nav, gs: -1 },
    { ...nav, thr: 101 },
    { ...nav, wp_seq: 1.5 },
    { ...nav, batt_pct: 120 },
    "GUIDED",
  ]) {
    const result = sanitizeBuoy({ ...mav, nav: bad });
    assert.equal(Object.hasOwn(result, "nav"), false);
    assert.equal(result.nav_bad, 1);
    assert.equal(result.lat, mav.lat);
    assert.equal(result.yaw, 278.5);
  }
});

test("accepts the 'no waypoint' and 'no battery' markers", () => {
  const result = sanitizeBuoy({ ...mav, nav: { ...nav, wp_dist: -1, batt_pct: -1 } });
  assert.equal(result.nav.wp_dist, -1);
  assert.equal(result.nav.batt_pct, -1);
});
