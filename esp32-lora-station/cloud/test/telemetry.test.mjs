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

test("wraps headings that round up to 360", () => {
  const result = sanitizeBuoy({ ...base, ...attitude, yaw: 359.97, target_yaw: 359.96 });
  assert.equal(result.yaw, 0);
  assert.equal(result.target_yaw, 0);
  assert.equal(sanitizeBuoy({ ...base, ...attitude, target_yaw: -1 }).target_yaw, -1);
});

test("still rejects a frame without signal values", () => {
  assert.equal(sanitizeBuoy({ ...attitude, fix: 0 }), null);
});
