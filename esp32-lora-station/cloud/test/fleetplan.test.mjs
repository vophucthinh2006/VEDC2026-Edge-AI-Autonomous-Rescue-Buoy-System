// Kiểm thử kế hoạch đội tàu: chia vùng phủ kín, không chồng nhau; đội hình đúng chỗ.
// Chạy:  node --test test/fleetplan.test.mjs
import assert from "node:assert/strict";
import test from "node:test";
import { formationOffsets, planFormation, planSectors, sweepAngle } from "../src/fleetplan.js";

// Hình chữ nhật 120 m (đông-tây) x 60 m (bắc-nam) quanh Hồ Đá
const O = { lat: 10.8834, lon: 106.7960 };
const kx = 111320 * Math.cos(O.lat * Math.PI / 180), ky = 111320;
const at = (x, y) => ({ lat: O.lat + y / ky, lon: O.lon + x / kx });
const RECT = [at(0, 0), at(120, 0), at(120, 60), at(0, 60)];
const xy = ([lat, lon]) => ({ x: (lon - O.lon) * kx, y: (lat - O.lat) * ky });

test("the sweep runs along the longest side", () => {
  assert.ok(Math.abs(sweepAngle(RECT) - 90) < 0.5);
});

test("three boats get three strips that cover the area once, nearest boat to each", () => {
  const boats = [{ id: "PHAO-01", ...at(0, 55) }, { id: "PHAO-02", ...at(0, 30) }, { id: "PHAO-03", ...at(0, 5) }];
  const plan = planSectors(RECT, boats, 10);
  assert.equal(plan.length, 3);
  const ys = {};
  for (const { id, pts } of plan) {
    assert.ok(pts.length >= 4 && pts.length % 2 === 0);
    ys[id] = [...new Set(pts.map(p => Math.round(xy(p).y)))];
    // Each lane runs the whole width
    for (let i = 0; i < pts.length; i += 2) assert.ok(Math.abs(Math.abs(xy(pts[i]).x - xy(pts[i + 1]).x) - 120) < 0.5);
    // Starts at the end near its boat (all boats on the west side)
    assert.ok(xy(pts[0]).x < 1);
  }
  const all = Object.values(ys).flat().sort((a, b) => a - b);
  assert.deepEqual(all, [5, 15, 25, 35, 45, 55]);                 // six lanes, each once
  assert.deepEqual(ys["PHAO-01"].sort(), [45, 55]);              // the northern strip to the northern boat
  assert.deepEqual(ys["PHAO-03"].sort((a, b) => a - b), [5, 15]);
});

test("more boats than lanes: the extra boats get nothing", () => {
  const plan = planSectors(RECT, ["A", "B", "C", "D", "E", "F", "G"].map(id => ({ id })), 20);
  assert.equal(plan.length, 3);
});

test("formation places: line left and right of the leader, V falling back, column behind", () => {
  assert.deepEqual(formationOffsets("line", ["B", "C", "D"], 10), { B: { right: 10, back: 0 }, C: { right: -10, back: 0 }, D: { right: 20, back: 0 } });
  assert.deepEqual(formationOffsets("v", ["B", "C"], 10), { B: { right: 10, back: 8 }, C: { right: -10, back: 8 } });
  assert.deepEqual(formationOffsets("column", ["B", "C"], 10), { B: { right: 0, back: 10 }, C: { right: 0, back: 20 } });
});

test("in line abreast the leader's lanes are as far apart as the whole line is wide", () => {
  const f = planFormation(RECT, { id: "A", ...at(0, 0) }, ["B", "C"], "line", 10);
  const lanesY = [...new Set(f.route.map(p => Math.round(xy(p).y)))].sort((a, b) => a - b);
  assert.deepEqual(lanesY, [15, 45]);
  assert.deepEqual(Object.keys(f.members), ["B", "C"]);
});

test("followers take the place nearest to them, so they do not cross the leader's path", () => {
  const B = at(0, 40), C = at(0, 20);
  const f = planFormation(RECT, { id: "A", ...at(0, 30) }, [{ id: "B", ...B }, { id: "C", ...C }], "line", 10);
  // Where each place is at the start of the route, the leader heading along its first leg
  const p0 = xy(f.route[0]), p1 = xy(f.route[1]), h = Math.atan2(p1.x - p0.x, p1.y - p0.y);
  const place = m => ({ x: p0.x + m.right * Math.cos(h), y: p0.y - m.right * Math.sin(h) });
  const d = (p, q) => Math.hypot(p.x - q.x, p.y - q.y);
  const b = xy([B.lat, B.lon]), c = xy([C.lat, C.lon]);
  assert.ok(d(b, place(f.members.B)) + d(c, place(f.members.C)) < d(b, place(f.members.C)) + d(c, place(f.members.B)));
});
