// Kiểm tra đầu vào của các lệnh đội tàu trong Worker (gửi lộ trình riêng, lệnh cả đội).
// Chạy:  node --test test/fleet.test.mjs
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
const { sanitizeSend, sanitizeFleet, sanitizePlan } = await import("../src/worker.js");

test("a send may carry the buoy's own route", () => {
  assert.deepEqual(sanitizeSend({ id: "PHAO-02", pts: [[10.123456789, 106.5], [10.2, 106.6]] }),
    { id: "PHAO-02", loop: false, clear: false, pts: [[10.1234568, 106.5], [10.2, 106.6]] });
  assert.deepEqual(sanitizeSend({}), { id: "PHAO-01", loop: false, clear: false });
  assert.equal(sanitizeSend({ id: "PHAO-02", pts: [[91, 106]] }), null);
  assert.equal(sanitizeSend({ id: "PHAO-02", pts: [{ lat: 10, lon: 106 }] }), null);
  assert.equal(sanitizeSend({ id: "PHAO-02", pts: Array(301).fill([10, 106]) }), null);
});

test("fleet commands take hold, resume or return, for everyone or the buoys named", () => {
  assert.deepEqual(sanitizeFleet({ action: "hold" }), { action: "hold" });
  assert.deepEqual(sanitizeFleet({ action: "return", ids: ["PHAO-02", "<b>"] }), { action: "return", ids: ["PHAO-02", "b"] });
  assert.equal(sanitizeFleet({ action: "explode" }), null);
  assert.equal(sanitizeFleet({ action: "hold", ids: "PHAO-01" }), null);
});

test("a plan needs an area of three points or more and one to eight buoys", () => {
  const poly = [{ lat: 10.88, lon: 106.79 }, { lat: 10.881, lon: 106.79 }, { lat: 10.881, lon: 106.791 }];
  const plan = sanitizePlan({ mode: "formation", poly, ids: ["PHAO-01", "PHAO-02", "PHAO-01"], shape: "v" });
  assert.deepEqual([plan.ids, plan.spacing, plan.shape, plan.dry], [["PHAO-01", "PHAO-02"], 10, "v", false]);
  assert.equal(sanitizePlan({ mode: "sectors", poly: poly.slice(0, 2), ids: ["PHAO-01"] }), null);
  assert.equal(sanitizePlan({ mode: "sectors", poly, ids: [] }), null);
  assert.equal(sanitizePlan({ mode: "swarm", poly, ids: ["PHAO-01"] }), null);
  assert.equal(sanitizePlan({ mode: "sectors", poly, ids: ["PHAO-01"], spacing: 1 }), null);
});
