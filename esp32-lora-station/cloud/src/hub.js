import { DurableObject } from "cloudflare:workers";
import { json } from "./util.js";
import { Geocoder } from "./geocode.js";
import { formationOffsets, planFormation, planSectors } from "./fleetplan.js";

const MAX_BUOYS = 50;
const MAX_HISTORY = 300;   // số mẫu RSSI/SNR/vị trí giữ cho mỗi phao
// LoRa đến 5 giây một gói, cầu nối MAVLink thì 1-2 gói mỗi giây. Gói MAVLink vẫn được phát
// realtime, nhưng chỉ ghi kho và lịch sử tối đa mỗi PERSIST_MS một lần cho mỗi phao, để số
// lần ghi Durable Object không vượt hạn mức miễn phí (khoảng 100 nghìn dòng mỗi ngày).
// Gói LoRa luôn được ghi: ESP32 xếp hàng khi mất mạng rồi gửi dồn, mỗi gói là một mẫu thật.
const PERSIST_MS = 5000;
const EMPTY_MISSION = { rev: 0, waypoints: [], poly: [], spacing: 10, angle: null, speed: 1.5 };
const MAX_VICTIMS = 200;
const MAX_ALERTS = 50;
// Một người, một phao (docs/victim-claim.md): báo cáo của mọi phao cách nhau dưới CLAIM_M là cùng một người
// (hoặc một nhóm), phao báo đầu tiên là chủ. Trạm ESP32 phân xử ngay trong ACK; Hub giữ sổ và lời của admin.
const CLAIM_M = 6;
const CONFIRM_M = 10;                  // nút xác nhận của chủ cách người dưới chừng này: người đã được cứu
const CLAIM_KEEP_MS = { claimed: 10 * 60 * 1000, rescued: 30 * 60 * 1000, released: 2 * 60 * 1000 };
const CLAIMS_DOWN_MAX = 16;            // người gửi xuống trạm trong "claims", mới nhất trước
const CLAIM_CODE = { released: 0, claimed: 1, rescued: 2 };
const EVENT_DEDUP_MS = 10 * 60 * 1000; // phao gửi lặp sự kiện tới khi trạm ACK; phao khởi động lại thì seq đếm lại
const MAX_SEEN = 64;                   // số sự kiện gần nhất nhớ cho mỗi phao để bỏ bản lặp
const LOST_MS = 30 * 1000;             // phao im quá lâu này: cảnh báo mất liên lạc (beacon 5-8 s một lần)
const LOST_WINDOW_MS = 60 * 60 * 1000; // phao im quá 1 giờ là phao của lần chạy cũ: không cảnh báo nữa
const FLEET_RECENT_MS = 5 * 60 * 1000; // lệnh cả đội gửi cho các phao nghe được trong 5 phút gần nhất
const HOME_SPACING_M = 8;              // khoảng cách giữa hai điểm đỗ cạnh nhau dọc bờ
const HOME_AXIS_MOVE_M = 20;           // trạm dời xa hơn chừng này thì tính lại hướng ra nước

// Điểm cách p một đoạn d mét theo hướng bearing (độ, la bàn); xấp xỉ phẳng, đủ cho vài chục mét
function offsetM(p, bearing, d) {
  const rad = Math.PI / 180;
  const dn = d * Math.cos(bearing * rad), de = d * Math.sin(bearing * rad);
  return { lat: p.lat + dn / 111320, lon: p.lon + de / (111320 * Math.cos(p.lat * rad)) };
}

function bearingDeg(a, b) {
  const rad = Math.PI / 180;
  const y = Math.sin((b.lon - a.lon) * rad) * Math.cos(b.lat * rad);
  const x = Math.cos(a.lat * rad) * Math.sin(b.lat * rad) - Math.sin(a.lat * rad) * Math.cos(b.lat * rad) * Math.cos((b.lon - a.lon) * rad);
  return (Math.atan2(y, x) / rad + 360) % 360;
}

// Ô đỗ của phao: PHAO-01 -> 0 (ngay tại trạm, như khi chỉ có một phao), PHAO-02 -> 1, ...
export function homeSlot(id) {
  const m = /(\d+)$/.exec(id);
  if (m) return Math.max(0, parseInt(m[1], 10) - 1);
  let h = 0;
  for (const c of id) h = (h * 31 + c.charCodeAt(0)) % 997;
  return 8 + (h % 32);
}

// Ô k nằm trên đường dọc bờ qua trạm (vuông góc với hướng ra nước): 0 tại trạm, 1 bên phải 8 m, 2 bên trái
// 8 m, 3 bên phải 16 m... để các phao về bờ đỗ cạnh nhau, không dồn vào một điểm.
export function homePoint(station, waterBearing, slot) {
  if (slot === 0) return { lat: station.lat, lon: station.lon };
  const side = slot % 2 === 1 ? 90 : -90;
  return offsetM(station, (waterBearing + side + 360) % 360, Math.ceil(slot / 2) * HOME_SPACING_M);
}

// Khoảng cách mặt đất (m) giữa hai điểm {lat, lon}
function distM(a, b) {
  const R = 6371000, rad = Math.PI / 180;
  const dLat = (b.lat - a.lat) * rad, dLon = (b.lon - a.lon) * rad;
  const h = Math.sin(dLat / 2) ** 2 + Math.cos(a.lat * rad) * Math.cos(b.lat * rad) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(h));
}

// Bản tin chỉ chở sự kiện (không có nhóm tư thế) không phải beacon: không đổi vị trí, trạng thái hay lịch sử phao
const eventOnly = msg => (!!msg.event || !!msg.event_bad) && msg.yaw === undefined && !msg.attitude_bad;
// A diagnostics frame (dg=, telemetry.js sanitizeDiag) carries no beacon fields: kept beside the buoy, never
// in place of its last beacon
const diagOnly = msg => (!!msg.diag || !!msg.diag_bad) && msg.yaw === undefined && msg.lat === undefined && !msg.event;

// Bản tin cho trình duyệt; kèm tọa độ để bảng đội tàu vẽ lộ trình riêng của từng phao (chia vùng, đội hình)
const downlinkSummary = d => ({ type: "downlink", rev: d.rev, id: d.id, loop: d.loop, n: d.pts.length, pts: d.pts, mission_rev: d.mission_rev, by: d.by, at: d.at, delivered: d.delivered || null });

// Phao đích báo Pi đã nhận đủ bản gửi này: cùng rev, đủ điểm, pi = 1
const routeDelivered = (d, msg) => !!d.rev && !d.delivered && msg.id === d.id && !!msg.route &&
  msg.route.rev === d.rev && msg.route.have === d.pts.length && msg.route.pi === 1;

// Nominatim asks for an identifying User-Agent; GEOCODE_CONTACT (an email or URL) is optional
const geocodeUserAgent = env =>
  `mliot-redgescue/1.0 (rescue buoy shore station dashboard${env?.GEOCODE_CONTACT ? "; " + env.GEOCODE_CONTACT : ""})`;

// Bản gửi gọn cho trạm: chỉ thứ trạm cần phát qua LoRa
const routeForStation = d => ({ rev: d.rev, id: d.id, loop: d.loop, pts: d.pts });

// Một Durable Object duy nhất giữ vị trí mới nhất của từng phao, lộ trình chung, và phát cho mọi trình duyệt
// đang mở. Với nhiều phao nó là bộ quản lý đội tàu của trạm bờ: mỗi phao một bản gửi lộ trình, lệnh riêng
// (đèn, dừng, về bờ), điểm đỗ riêng, nhiệm vụ hiện tại, và cảnh báo khi một phao mất liên lạc.
export class Hub extends DurableObject {
  buoys = null;     // id -> { msg, t }
  mission = null;   // { rev, waypoints, poly, spacing, angle, speed, by?, at? }
  downlinks = null; // id -> lộ trình đang gửi xuống phao đó (docs/lora-route-downlink.md)
  dlRev = null;     // rev của lần gửi gần nhất, một bộ đếm chung cho mọi phao như trước
  station = undefined;   // { msg, t } vị trí GPS trạm bờ; undefined = chưa tải, null = chưa có
  hist = {};        // id -> [{ t, rssi, snr, lat?, lon? }], mỗi phao một khóa lưu trữ `h:<id>`
  persistedAt = {}; // id -> thời điểm ghi kho gần nhất (chỉ trong bộ nhớ)
  // Người phát hiện và cảnh báo: { victims: [{ vid, buoy, lat, lon, risk, conf, first_at, last_at }],
  // alerts: [{ type, buoy, seq, at, lat, lon, ... }], seen: id -> [{ seq, type, t }], vid }
  events = null;
  lights = null;    // id -> { want, by, at }: lệnh đèn của admin, tách khỏi bản gửi lộ trình
  hold = null;      // id -> { want, by, at }: lệnh DỪNG của admin (want true = F407 khóa motor), như đèn
  ret = null;       // id -> { want, by, at }: lệnh VỀ BỜ của admin (want true = Pi chạy về điểm đỗ), như đèn
  tasks = null;     // id -> { kind: idle|sector|return|formation|victim, ref, since }: phao đang làm gì
  lost = null;      // id -> lúc đã cảnh báo mất liên lạc (xóa khi nghe lại)
  homeAxis = null;  // { bearing, lat, lon }: hướng từ trạm ra nước, tính một lần cho các điểm đỗ
  // Đội hình đang chạy: { leader, shape, spacing, members: id -> { right, back }, route, loop, at }; null: không có.
  // Chỉ phao dẫn đầu nhận lộ trình; các phao còn lại nghe beacon của nó và giữ chỗ của mình (fol trong downlink).
  formation = undefined;
  diag = null;      // id -> { d, t }: khung chẩn đoán mới nhất của phao (lý do dừng, nhiệm vụ Pi, pin)
  geocoder = null;  // address search cache and rate limits (memory only, rebuilt after eviction)

  async loadBuoys() {
    if (!this.buoys) this.buoys = (await this.ctx.storage.get("buoys")) || {};
    return this.buoys;
  }
  async loadMission() {
    if (!this.mission) this.mission = (await this.ctx.storage.get("mission")) || { ...EMPTY_MISSION };
    return this.mission;
  }

  // Bản chụp lộ trình admin đã bấm "Gửi tới phao", mỗi phao một bản: id -> { rev, id, loop, pts: [[lat, lon]],
  // by, at, delivered? }. delivered: lúc phao báo Pi đã nhận đủ; từ đó trạm không được mời gửi lại bản này.
  // Kho cũ chỉ có một bản "downlink": chuyển sang dạng mới khi đọc.
  async loadDownlinks() {
    if (!this.downlinks) {
      this.downlinks = await this.ctx.storage.get("downlinks");
      if (!this.downlinks) {
        const old = await this.ctx.storage.get("downlink");
        this.downlinks = old?.rev ? { [old.id]: old } : {};
      }
      this.dlRev = (await this.ctx.storage.get("dlRev")) ?? Math.max(0, ...Object.values(this.downlinks).map(d => d.rev));
    }
    return this.downlinks;
  }

  async loadRet() {
    if (!this.ret) this.ret = (await this.ctx.storage.get("ret")) || {};
    return this.ret;
  }

  // have = phao đang về hoặc đã đỗ (nhiệm vụ R/H trong khung chẩn đoán), null khi chưa báo
  async retMessage(id) {
    const r = (await this.loadRet())[id];
    const mission = (await this.loadDiag())[id]?.d.mission;
    const have = mission === undefined ? null : mission === "R" || mission === "H";
    return { type: "ret", id, want: r ? r.want : null, have, by: r?.by ?? null, at: r?.at ?? null };
  }

  async loadTasks() {
    if (!this.tasks) this.tasks = (await this.ctx.storage.get("tasks")) || {};
    return this.tasks;
  }

  // Ghi nhiệm vụ hiện tại của phao và báo dashboard. Vấn đề 2 (nhiều phao thấy cùng một người) sẽ ghi
  // kind "victim" ở đây khi trạm giao một người cho đúng một phao.
  async setTask(id, kind, ref = null) {
    const all = await this.loadTasks();
    if (all[id]?.kind === kind && JSON.stringify(all[id].ref) === JSON.stringify(ref)) return;
    all[id] = { kind, ref, since: Date.now() };
    await this.ctx.storage.put("tasks", all);
    this.broadcast({ type: "task", id, ...all[id] });
  }

  // Nhiệm vụ khi phao không về bờ: lộ trình nó đang giữ, hoặc không có gì
  async routeTask(id) {
    const dl = (await this.loadDownlinks())[id];
    if (dl?.pts?.length) await this.setTask(id, "sector", { rev: dl.rev, n: dl.pts.length });
    else await this.setTask(id, "idle");
  }

  async loadLost() {
    if (!this.lost) this.lost = (await this.ctx.storage.get("lost")) || {};
    return this.lost;
  }

  // Phao im quá LOST_MS: một cảnh báo "L" (mất liên lạc) và cờ trên dashboard. Gọi mỗi lần trạm hỏi downlink
  // (5 s một lần) và mỗi gói đến, nên không cần hẹn giờ.
  async checkLinks(now) {
    const buoys = await this.loadBuoys();
    const lost = await this.loadLost();
    let changed = false;
    for (const [id, b] of Object.entries(buoys)) {
      const age = now - b.t;
      if (age <= LOST_MS || age > LOST_WINDOW_MS || lost[id] || b.msg.link === "mavlink") continue;
      lost[id] = now;
      changed = true;
      const ev = await this.loadEvents();
      const alert = { type: "L", buoy: id, seq: 0, at: now, lat: b.msg.lat ?? null, lon: b.msg.lon ?? null, silent_s: Math.round(age / 1000) };
      ev.alerts.push(alert);
      if (ev.alerts.length > MAX_ALERTS) ev.alerts.splice(0, ev.alerts.length - MAX_ALERTS);
      await this.ctx.storage.put("events", ev);
      this.broadcast({ type: "alert", alert });
      this.broadcast({ type: "link", id, lost: true, since: now });
    }
    if (changed) {
      await this.ctx.storage.put("lost", lost);
      for (const id of Object.keys(lost)) if (lost[id] === now) await this.releaseClaims(id, now);
      await this.electLeader(now);
    }
  }

  // Hướng từ trạm ra nước: từ trạm tới trọng tâm các phao có GPS. Giữ cố định tới khi trạm dời xa, để điểm đỗ
  // không xoay theo khi các phao chạy về.
  async waterBearing(station, now) {
    if (this.homeAxis === null) this.homeAxis = (await this.ctx.storage.get("homeAxis")) || undefined;
    const a = this.homeAxis;
    if (a && distM(a, station) < HOME_AXIS_MOVE_M) return a.bearing;
    const pts = Object.values(await this.loadBuoys())
      .filter(b => now - b.t < LOST_WINDOW_MS && b.msg.lat !== undefined && distM(b.msg, station) > 5)
      .map(b => b.msg);
    if (!pts.length) return null;
    const c = { lat: pts.reduce((s, p) => s + p.lat, 0) / pts.length, lon: pts.reduce((s, p) => s + p.lon, 0) / pts.length };
    this.homeAxis = { bearing: Math.round(bearingDeg(station, c) * 10) / 10, lat: station.lat, lon: station.lon };
    await this.ctx.storage.put("homeAxis", this.homeAxis);
    return this.homeAxis.bearing;
  }

  // id -> [lat, lon]: điểm đỗ của các phao nghe được trong giờ qua; {} khi trạm chưa có GPS
  async homes(now) {
    const st = await this.loadStation();
    if (!st || st.msg.lat === undefined) return {};
    const bearing = await this.waterBearing(st.msg, now);
    const out = {};
    for (const [id, b] of Object.entries(await this.loadBuoys())) {
      if (now - b.t > LOST_WINDOW_MS || b.msg.link === "mavlink") continue;
      const slot = homeSlot(id);
      if (slot !== 0 && bearing === null) continue;
      const p = homePoint(st.msg, bearing ?? 0, slot);
      out[id] = [+p.lat.toFixed(7), +p.lon.toFixed(7)];
    }
    return out;
  }

  // Bản gửi lộ trình mới cho phao id: rev mới, chưa giao; thay lệnh về bờ (phao đi lộ trình này)
  async sendRoute(id, pts, loop, missionRev, by) {
    const all = await this.loadDownlinks();
    // rev 1..65535 trên LoRa, 0 nghĩa là "chưa có"; một bộ đếm chung để một phao không bao giờ gặp lại rev cũ
    this.dlRev = (this.dlRev % 65535) + 1;
    all[id] = { rev: this.dlRev, id, loop: !!loop, pts, mission_rev: missionRev, by: by || null, at: Date.now() };
    await this.ctx.storage.put({ downlinks: all, dlRev: this.dlRev });
    const summary = downlinkSummary(all[id]);
    this.broadcast(summary);
    if ((await this.loadRet())[id]?.want) await this.setRet(id, false, by);
    else await this.routeTask(id);
    return summary;
  }

  async loadFormation() {
    if (this.formation === undefined) this.formation = (await this.ctx.storage.get("formation")) || null;
    return this.formation;
  }

  async saveFormation(f) {
    this.formation = f;
    await this.ctx.storage.put("formation", f);
    this.broadcast({ type: "formation", formation: f });
  }

  // Bắt đầu đội hình: lộ trình cho phao dẫn đầu, lộ trình rỗng cho các phao theo (bỏ dải quét cũ của chúng)
  async startFormation(f, poly, loop, by) {
    await this.saveFormation({ leader: f.leader, shape: f.shape, spacing: f.spacing, members: f.members, route: f.route, poly, loop, at: Date.now() });
    await this.sendRoute(f.leader, f.route, loop, null, by);
    await this.setTask(f.leader, "formation", { role: "leader", n: f.route.length });
    for (const id of Object.keys(f.members)) {
      await this.sendRoute(id, [], false, null, by);
      await this.setTask(id, "formation", { role: "follower", leader: f.leader, ...f.members[id] });
    }
  }

  // Các phao ids rời đội hình (phao dẫn đầu rời thì cả đội hình kết thúc); chúng đứng tại chỗ chờ lệnh
  async leaveFormation(ids, by) {
    const f = await this.loadFormation();
    if (!f) return;
    if (ids.includes(f.leader)) {
      for (const id of Object.keys(f.members)) if (!ids.includes(id)) await this.sendRoute(id, [], false, null, by);
      await this.saveFormation(null);
      return;
    }
    const members = { ...f.members };
    for (const id of ids) delete members[id];
    await this.saveFormation({ ...f, members });
  }

  // Phao dẫn đầu mất liên lạc: phao theo còn liên lạc có tên nhỏ nhất lên dẫn đầu, nhận phần lộ trình phao cũ chưa
  // đi (từ waypoint nó đang tới trong khung chẩn đoán), các phao còn lại xếp lại chỗ quanh phao mới.
  async electLeader(now) {
    const f = await this.loadFormation();
    const lost = await this.loadLost();
    if (!f || !lost[f.leader]) return;
    const alive = Object.keys(f.members).filter(id => !lost[id]).sort();
    if (!alive.length) return;
    const [leader, ...followers] = alive;
    const wp = (await this.loadDiag())[f.leader]?.d.wp ?? 0;
    const route = f.route.slice(Math.max(0, wp - 1));
    const members = formationOffsets(f.shape, followers, f.spacing);
    const old = f.leader;
    await this.saveFormation({ ...f, leader, members, route, at: now });
    await this.sendRoute(leader, route, f.loop, null, "auto");
    await this.setTask(leader, "formation", { role: "leader", n: route.length });
    for (const id of followers) await this.setTask(id, "formation", { role: "follower", leader, ...members[id] });
    await this.setTask(old, "idle");
    const ev = await this.loadEvents();
    const alert = { type: "E", buoy: leader, seq: 0, at: now, lat: null, lon: null, replaces: old };
    ev.alerts.push(alert);
    if (ev.alerts.length > MAX_ALERTS) ev.alerts.splice(0, ev.alerts.length - MAX_ALERTS);
    await this.ctx.storage.put("events", ev);
    this.broadcast({ type: "alert", alert });
  }

  async setHold(id, want, by) {
    const all = await this.loadHold();
    all[id] = { want, by: by || null, at: Date.now() };
    await this.ctx.storage.put("hold", all);
    const m = await this.holdMessage(id);
    this.broadcast(m);
    return m;
  }

  // Về bờ đổi nhiệm vụ của phao thành "return"; bỏ lệnh thì quay lại lộ trình đang giữ (nếu có)
  async setRet(id, want, by) {
    const all = await this.loadRet();
    all[id] = { want, by: by || null, at: Date.now() };
    await this.ctx.storage.put("ret", all);
    const m = await this.retMessage(id);
    this.broadcast(m);
    if (want) {
      await this.releaseClaims(id, Date.now());
      await this.setTask(id, "return");
    } else await this.routeTask(id);
    return m;
  }

  async loadStation() {
    if (this.station === undefined) this.station = (await this.ctx.storage.get("station")) || null;
    return this.station;
  }

  async loadHistory(id) {
    if (!this.hist[id]) this.hist[id] = (await this.ctx.storage.get("h:" + id)) || [];
    return this.hist[id];
  }

  async loadEvents() {
    if (!this.events) this.events = (await this.ctx.storage.get("events")) || { victims: [], alerts: [], seen: {}, vid: 0 };
    return this.events;
  }

  async loadLights() {
    if (!this.lights) this.lights = (await this.ctx.storage.get("lights")) || {};
    return this.lights;
  }

  // have = trạng thái đèn phao báo trong beacon gần nhất (null khi chưa báo)
  async lightsMessage(id) {
    const l = (await this.loadLights())[id];
    const have = (await this.loadBuoys())[id]?.msg.lights;
    return { type: "lights", id, want: l ? l.want : null, have: have ?? null, by: l?.by ?? null, at: l?.at ?? null };
  }

  // id -> { d, t }: the latest diagnostics frame of each buoy (telemetry.js sanitizeDiag)
  async loadDiag() {
    if (!this.diag) this.diag = (await this.ctx.storage.get("diag")) || {};
    return this.diag;
  }

  async loadHold() {
    if (!this.hold) this.hold = (await this.ctx.storage.get("hold")) || {};
    return this.hold;
  }

  // have = phao đang dừng theo lệnh hay không, từ beacon gần nhất (null khi chưa báo)
  async holdMessage(id) {
    const h = (await this.loadHold())[id];
    const have = (await this.loadBuoys())[id]?.msg.hold;
    return { type: "hold", id, want: h ? h.want : null, have: have === undefined ? null : have === 1, by: h?.by ?? null, at: h?.at ?? null };
  }

  // Nhiệm vụ của phao khi xong việc với một người: chỗ trong đội hình nếu nó đang ở đó, không thì lộ trình
  async restoreTask(id) {
    const f = await this.loadFormation();
    if (f && f.leader === id) await this.setTask(id, "formation", { role: "leader", n: f.route.length });
    else if (f?.members[id]) await this.setTask(id, "formation", { role: "follower", leader: f.leader, ...f.members[id] });
    else await this.routeTask(id);
  }

  // Đổi trạng thái một người (claimed / rescued / released) và chủ; cập nhật sổ nhiệm vụ của các phao.
  async setClaim(v, status, owner, now, by = null) {
    const before = v.owner;
    v.status = status;
    v.owner = owner;
    v.status_at = now;
    if (by) v.status_by = by;
    if (status === "claimed") await this.setTask(owner, "victim", { vid: v.vid });
    if (before && (before !== owner || status !== "claimed") && (await this.loadTasks())[before]?.ref?.vid === v.vid) {
      await this.restoreTask(before);
    }
  }

  // Nhả mọi người phao id đang giữ (phao mất liên lạc, bị gọi về bờ): phao khác thấy họ sẽ nhận
  async releaseClaims(id, now) {
    const ev = await this.loadEvents();
    const mine = ev.victims.filter(v => v.status === "claimed" && v.owner === id);
    for (const v of mine) {
      await this.setClaim(v, "released", null, now);
      this.broadcast({ type: "victim", victim: v, new: false });
    }
    if (mine.length) await this.ctx.storage.put("events", ev);
    return mine.length;
  }

  // Người gửi xuống trạm: [lat, lon, chủ, 1 có chủ | 2 đã cứu | 0 nhả], còn hiệu lực, mới nhất trước
  async claimsForStation(now) {
    const ev = await this.loadEvents();
    return ev.victims
      .filter(v => v.status && now - (v.status === "claimed" ? Math.max(v.last_at, v.status_at) : v.status_at) < CLAIM_KEEP_MS[v.status])
      .sort((a, b) => b.status_at - a.status_at)
      .slice(0, CLAIMS_DOWN_MAX)
      .map(v => [v.lat, v.lon, v.owner || "", CLAIM_CODE[v.status]]);
  }

  // Ghi một sự kiện mới; trả về false nếu là bản phao gửi lặp
  async recordEvent(id, e, now) {
    const ev = await this.loadEvents();
    const seen = ev.seen[id] || (ev.seen[id] = []);
    if (seen.some(s => s.seq === e.seq && s.type === e.type && now - s.t < EVENT_DEDUP_MS)) return false;
    seen.push({ seq: e.seq, type: e.type, t: now });
    if (seen.length > MAX_SEEN) seen.splice(0, seen.length - MAX_SEEN);

    let out, rescued = null;
    if (e.type === "V" && e.lat !== null) {
      // Báo gần một người đã có (phao nào báo cũng vậy): cùng một người, không thêm chấm mới.
      // Phao báo đầu tiên là chủ; người đã nhả thì phao báo kế tiếp nhận.
      let v = ev.victims.reduce((best, x) => {
        const d = distM(x, e);
        return d < CLAIM_M && (!best || d < best.d) ? { x, d } : best;
      }, null)?.x;
      const isNew = !v;
      if (v) {
        v.seen_by ??= [v.buoy];
        if (!v.seen_by.includes(id)) v.seen_by.push(id);
        if (id === (v.owner ?? v.buoy)) Object.assign(v, { risk: e.risk, conf: e.conf });
        v.last_at = now;
        if (!v.status || v.status === "released") await this.setClaim(v, "claimed", id, now);
      } else {
        v = { vid: ++ev.vid, buoy: id, lat: e.lat, lon: e.lon, risk: e.risk, conf: e.conf, first_at: now, last_at: now, seen_by: [id] };
        ev.victims.push(v);
        if (ev.victims.length > MAX_VICTIMS) ev.victims.splice(0, ev.victims.length - MAX_VICTIMS);
        await this.setClaim(v, "claimed", id, now);
      }
      out = { type: "victim", victim: v, new: isNew };
    } else {
      // Lật, xác nhận, hoặc phát hiện người khi phao chưa có GPS: chỉ là cảnh báo
      const { seq, type, ...rest } = e;
      const alert = { type, buoy: id, seq, at: now, ...rest };
      ev.alerts.push(alert);
      if (ev.alerts.length > MAX_ALERTS) ev.alerts.splice(0, ev.alerts.length - MAX_ALERTS);
      out = { type: "alert", alert };
      // Nút xác nhận của chủ cạnh người của nó (người nó nhận gần nhất khi không có GPS): đã cứu
      if (e.type === "C") {
        const mine = ev.victims.filter(v => v.status === "claimed" && v.owner === id);
        const v = e.lat !== null
          ? mine.filter(x => distM(x, e) < CONFIRM_M).sort((a, b) => distM(a, e) - distM(b, e))[0]
          : mine.sort((a, b) => b.status_at - a.status_at)[0];
        if (v) {
          await this.setClaim(v, "rescued", id, now);
          rescued = v;
        }
      }
    }
    await this.ctx.storage.put("events", ev);
    this.broadcast(out);
    if (rescued) this.broadcast({ type: "victim", victim: rescued, new: false });
    return true;
  }

  broadcast(obj) {
    const out = JSON.stringify(obj);
    for (const ws of this.ctx.getWebSockets()) {
      try { ws.send(out); } catch { /* client đã đóng */ }
    }
  }

  async fetch(request) {
    const url = new URL(request.url);

    if (url.pathname === "/ws") {
      if (request.headers.get("Upgrade") !== "websocket") return new Response("Cần WebSocket", { status: 426 });
      const [client, server] = Object.values(new WebSocketPair());
      this.ctx.acceptWebSocket(server);
      // Gửi trạng thái hiện có để trang mới mở không phải chờ gói kế tiếp
      const buoys = await this.loadBuoys();
      const now = Date.now();
      for (const b of Object.values(buoys)) {
        server.send(JSON.stringify({ ...b.msg, age_s: Math.round((now - b.t) / 1000) }));
      }
      // Lịch sử tín hiệu/vị trí để đồ thị và vệt di chuyển có dữ liệu ngay khi mở trang
      for (const id of Object.keys(buoys)) {
        const points = await this.loadHistory(id);
        if (points.length) server.send(JSON.stringify({ type: "history", id, points }));
      }
      const st = await this.loadStation();
      if (st) server.send(JSON.stringify({ type: "station", ...st.msg, age_s: Math.round((now - st.t) / 1000) }));
      server.send(JSON.stringify({ type: "mission", ...(await this.loadMission()) }));
      for (const dl of Object.values(await this.loadDownlinks())) {
        if (dl.rev) server.send(JSON.stringify(downlinkSummary(dl)));
      }
      const ev = await this.loadEvents();
      if (ev.victims.length || ev.alerts.length) server.send(JSON.stringify({ type: "victims", victims: ev.victims, alerts: ev.alerts }));
      for (const id of Object.keys(await this.loadLights())) server.send(JSON.stringify(await this.lightsMessage(id)));
      for (const id of Object.keys(await this.loadHold())) server.send(JSON.stringify(await this.holdMessage(id)));
      for (const id of Object.keys(await this.loadRet())) server.send(JSON.stringify(await this.retMessage(id)));
      for (const [id, t] of Object.entries(await this.loadTasks())) server.send(JSON.stringify({ type: "task", id, ...t }));
      for (const [id, since] of Object.entries(await this.loadLost())) server.send(JSON.stringify({ type: "link", id, lost: true, since }));
      const formation = await this.loadFormation();
      if (formation) server.send(JSON.stringify({ type: "formation", formation }));
      for (const [id, x] of Object.entries(await this.loadDiag())) {
        server.send(JSON.stringify({ type: "diag", id, ...x.d, age_s: Math.round((now - x.t) / 1000) }));
      }
      return new Response(null, { status: 101, webSocket: client });
    }

    // Address search (dashboard search box); one Geocoder here keeps Nominatim at 1 request/s overall
    if (url.pathname === "/geocode" && request.method === "GET") {
      this.geocoder ??= new Geocoder({ userAgent: geocodeUserAgent(this.env) });
      const r = await this.geocoder.search(url.searchParams.get("q"), url.searchParams.get("sid"));
      return json(r.body, r.status);
    }

    if (url.pathname === "/downlink") {
      if (request.method === "GET") {
        const now = Date.now();
        await this.checkLinks(now);
        // Đã giao thì không mời gửi lại: F407 khởi động lại báo rev 0, gửi lại bản cũ là thuyền tự chạy trên bờ
        const pending = Object.values(await this.loadDownlinks()).filter(d => d.rev && !d.delivered);
        // Trạm cũ chỉ đọc một bản ở gốc: bản gửi mới nhất. Trạm mới đọc mọi bản trong routes.
        const newest = pending.reduce((a, d) => (!a || d.at > a.at || (d.at === a.at && d.rev > a.rev) ? d : a), null);
        const out = newest ? { ...newest } : { rev: 0 };
        if (pending.length) out.routes = pending.map(routeForStation);
        // Lệnh đèn luôn đi kèm, kể cả khi không có lộ trình nào đang chờ giao
        const lights = Object.entries(await this.loadLights());
        if (lights.length) out.lights = Object.fromEntries(lights.map(([id, l]) => [id, l.want]));
        // Lệnh dừng cũng vậy: 1 = F407 khóa motor tới khi admin bấm "Chạy tiếp"
        const hold = Object.entries(await this.loadHold());
        if (hold.length) out.hold = Object.fromEntries(hold.map(([id, h]) => [id, h.want ? 1 : 0]));
        // Về bờ: 1 = Pi bỏ lộ trình, chạy về điểm đỗ của nó tới khi admin cho chạy tiếp
        const ret = Object.entries(await this.loadRet());
        if (ret.length) out.ret = Object.fromEntries(ret.map(([id, r]) => [id, r.want ? 1 : 0]));
        // Điểm đỗ riêng của từng phao: trạm gửi nó trong ACK thay cho GPS của trạm
        const home = await this.homes(now);
        if (Object.keys(home).length) out.home = home;
        // Đội hình: phao theo -> [phao dẫn đầu, sang phải m, lùi sau m]; trạm gửi trong ACK của phao đó
        const f = await this.loadFormation();
        if (f && Object.keys(f.members).length) {
          out.fol = Object.fromEntries(Object.entries(f.members).map(([id, m]) => [id, [f.leader, m.right, m.back]]));
        }
        // Nhận nạn nhân: lời của Hub (và của admin) về từng người; trạm sửa bảng phân xử của nó theo đó
        const claims = await this.claimsForStation(now);
        if (claims.length) out.claims = claims;
        return json(out);
      }
      if (request.method === "PUT") {
        const { id, loop, clear, pts: given, by } = await request.json();
        const mission = await this.loadMission();
        // pts: lộ trình riêng cho phao này (chia vùng trên dashboard); không có thì lộ trình chung
        const own = Array.isArray(given);
        const pts = clear ? [] : own ? given : mission.waypoints.map(p => [p.lat, p.lon]);
        // Gửi tay một lộ trình cho phao đang ở đội hình: phao đó rời đội hình
        if ((await this.loadFormation())?.members[id]) await this.leaveFormation([id], by);
        return json(await this.sendRoute(id, pts, loop, own ? null : mission.rev, by));
      }
    }

    // Kế hoạch cho cả đội (src/fleetplan.js): { mode: sectors|formation, poly, ids, spacing?, angle?, shape?,
    // leader?, loop?, dry?, by }. dry: chỉ trả kế hoạch để dashboard xem trước, không gửi gì.
    if (url.pathname === "/plan" && request.method === "PUT") {
      const req = await request.json();
      const buoys = await this.loadBuoys();
      const boats = req.ids.map(id => ({ id, ...(buoys[id]?.msg.lat !== undefined ? { lat: buoys[id].msg.lat, lon: buoys[id].msg.lon } : {}) }));
      if (req.mode === "formation") {
        const leaderId = req.leader && req.ids.includes(req.leader) ? req.leader : req.ids[0];
        const leader = boats.find(b => b.id === leaderId);
        const followers = boats.filter(b => b.id !== leaderId);
        const f = planFormation(req.poly, leader, followers, req.shape, req.spacing, req.angle);
        if (req.dry) return json({ mode: "formation", ...f });
        await this.startFormation(f, req.poly, !!req.loop, req.by);
        return json({ mode: "formation", ...f });
      }
      const routes = planSectors(req.poly, boats, req.spacing, req.angle);
      if (req.dry) return json({ mode: "sectors", routes });
      const formation = await this.loadFormation();
      if (formation) await this.leaveFormation(Object.keys(formation.members).concat(formation.leader), req.by);
      for (const r of routes) await this.sendRoute(r.id, r.pts, !!req.loop, null, req.by);
      return json({ mode: "sectors", routes });
    }

    // Kết thúc đội hình: mọi phao đứng tại chỗ (lộ trình rỗng) chờ lệnh mới
    if (url.pathname === "/formation" && request.method === "DELETE") {
      const { by } = await request.json().catch(() => ({}));
      const f = await this.loadFormation();
      if (f) await this.leaveFormation(Object.keys(f.members).concat(f.leader), by);
      return json({ ok: true });
    }

    // Admin bật/tắt hai đèn của phao: { id, lights: 0..3, by }. Không đụng tới bản gửi lộ trình.
    if (url.pathname === "/lights" && request.method === "PUT") {
      const { id, lights, by } = await request.json();
      const all = await this.loadLights();
      if (!all[id] && Object.keys(all).length >= MAX_BUOYS) return json({ error: "too many buoys" }, 429);
      all[id] = { want: lights, by: by || null, at: Date.now() };
      await this.ctx.storage.put("lights", all);
      const m = await this.lightsMessage(id);
      this.broadcast(m);
      return json(m);
    }

    // Admin dừng / cho chạy tiếp một phao: { id, hold: true|false, by }. Lộ trình giữ nguyên.
    if (url.pathname === "/hold" && request.method === "PUT") {
      const { id, hold, by } = await request.json();
      const all = await this.loadHold();
      if (!all[id] && Object.keys(all).length >= MAX_BUOYS) return json({ error: "too many buoys" }, 429);
      return json(await this.setHold(id, hold === true, by));
    }

    // Admin cho một phao về bờ / bỏ lệnh về bờ: { id, ret: true|false, by }
    if (url.pathname === "/return" && request.method === "PUT") {
      const { id, ret, by } = await request.json();
      const all = await this.loadRet();
      if (!all[id] && Object.keys(all).length >= MAX_BUOYS) return json({ error: "too many buoys" }, 429);
      return json(await this.setRet(id, ret === true, by));
    }

    // Lệnh cho cả đội: { action: hold|resume|return, ids?, by }. Không có ids: mọi phao nghe được trong 5 phút.
    // resume bỏ cả DỪNG lẫn VỀ BỜ.
    if (url.pathname === "/fleet" && request.method === "PUT") {
      const { action, ids, by } = await request.json();
      const now = Date.now();
      const buoys = await this.loadBuoys();
      const targets = Array.isArray(ids) ? ids
        : Object.keys(buoys).filter(id => now - buoys[id].t < FLEET_RECENT_MS && buoys[id].msg.link !== "mavlink");
      for (const id of targets) {
        if (action === "hold" || action === "resume") await this.setHold(id, action === "hold", by);
        if (action === "return") await this.setRet(id, true, by);
        if (action === "resume" && (await this.loadRet())[id]?.want) await this.setRet(id, false, by);
      }
      return json({ action, ids: targets });
    }

    // Admin về một người: { vid, action: rescued | release | assign, id?, by }. assign: giao cho phao id và gửi
    // cho nó lộ trình một điểm tới chỗ người đó (docs/victim-claim.md).
    if (url.pathname === "/victim" && request.method === "PUT") {
      const { vid, action, id, by } = await request.json();
      const ev = await this.loadEvents();
      const v = ev.victims.find(x => x.vid === vid);
      if (!v) return json({ error: "no such person" }, 404);
      const now = Date.now();
      if (action === "rescued") await this.setClaim(v, "rescued", v.owner, now, by);
      else if (action === "release") await this.setClaim(v, "released", null, now, by);
      else if (action === "assign" && id) {
        if ((await this.loadFormation())?.members[id]) await this.leaveFormation([id], by);
        await this.sendRoute(id, [[v.lat, v.lon]], false, null, by);
        await this.setClaim(v, "claimed", id, now, by);
      } else return json({ error: "bad request" }, 400);
      await this.ctx.storage.put("events", ev);
      const m = { type: "victim", victim: v, new: false };
      this.broadcast(m);
      return json(m);
    }

    if (url.pathname === "/victims") {
      const ev = await this.loadEvents();
      if (request.method === "GET") return json({ victims: ev.victims, alerts: ev.alerts });
      if (request.method === "DELETE") {
        // Xóa chấm người và cảnh báo trên bản đồ; vẫn nhớ sự kiện đã nhận để bản lặp không vẽ lại chấm
        ev.victims = []; ev.alerts = [];
        await this.ctx.storage.put("events", ev);
        const m = { type: "victims", victims: [], alerts: [] };
        this.broadcast(m);
        return json(m);
      }
    }

    if (url.pathname === "/state") {
      const now = Date.now();
      const buoys = await this.loadBuoys();
      return json(Object.values(buoys).map(b => ({ ...b.msg, age_s: Math.round((now - b.t) / 1000) })));
    }

    if (url.pathname === "/mission") {
      if (request.method === "GET") return json(await this.loadMission());
      if (request.method === "PUT") {
        const { mission, by } = await request.json();
        const prev = await this.loadMission();
        this.mission = { ...mission, rev: prev.rev + 1, by: by || null, at: Date.now() };
        await this.ctx.storage.put("mission", this.mission);
        this.broadcast({ type: "mission", ...this.mission });
        return json(this.mission);
      }
    }

    if (url.pathname === "/station" && request.method === "POST") {
      const msg = await request.json();
      const prev = await this.loadStation();
      // Mất fix thì giữ vị trí cuối trong kho (trạm đứng yên), vẫn phát bản tin mới để biết trạm còn sống
      const stored = msg.fix === 0 && prev && prev.msg.lat !== undefined ? { ...msg, lat: prev.msg.lat, lon: prev.msg.lon } : msg;
      this.station = { msg: stored, t: Date.now() };
      await this.ctx.storage.put("station", this.station);
      this.broadcast({ type: "station", ...msg, age_s: 0 });
      return json({ ok: true });
    }

    if (url.pathname === "/ingest" && request.method === "POST") {
      const msg = await request.json();
      const buoys = await this.loadBuoys();
      if (!buoys[msg.id] && Object.keys(buoys).length >= MAX_BUOYS) return json({ error: "too many buoys" }, 429);
      const now = Date.now();
      if (diagOnly(msg)) {
        if (!msg.diag) return json({ ok: true, diag: "bad" });
        const all = await this.loadDiag();
        all[msg.id] = { d: msg.diag, t: now };
        await this.ctx.storage.put("diag", all);
        this.broadcast({ type: "diag", id: msg.id, ...msg.diag, age_s: 0 });
        return json({ ok: true, diag: "stored" });
      }
      // Trạm ACK sự kiện cho phao; bản lặp (cùng id, seq) chỉ được ghi nhận một lần
      const fresh = msg.event ? await this.recordEvent(msg.id, msg.event, now) : null;
      const result = { ok: true, clients: this.ctx.getWebSockets().length };
      if (msg.event) result.event = fresh ? "new" : "dup";
      if (eventOnly(msg)) return json(result);
      delete msg.event;   // đã phát riêng (victim/alert), không lưu vào beacon để mở trang không phát lại

      // Bản tin NO_FIX không có tọa độ: giữ vị trí cũ trong kho nhưng vẫn phát bản tin mới
      const prev = buoys[msg.id]?.msg;
      const stored = msg.fix === 0 && prev && (prev.fix === 1 || prev.lat !== undefined)
        ? { ...msg, lat: prev.lat, lon: prev.lon, fix_lost: 1 } : msg;
      buoys[msg.id] = { msg: stored, t: now };
      if (msg.link !== "mavlink" || now - (this.persistedAt[msg.id] || 0) >= PERSIST_MS) {
        this.persistedAt[msg.id] = now;
        const points = await this.loadHistory(msg.id);
        const sample = { t: now };
        if (msg.rssi !== undefined) { sample.rssi = msg.rssi; sample.snr = msg.snr; }
        if (msg.fix === 1) { sample.lat = msg.lat; sample.lon = msg.lon; }
        points.push(sample);
        if (points.length > MAX_HISTORY) points.splice(0, points.length - MAX_HISTORY);
        await this.ctx.storage.put({ buoys, ["h:" + msg.id]: points });
      }
      this.broadcast({ ...msg, age_s: 0 });
      const all = await this.loadDownlinks();
      const dl = all[msg.id];
      if (dl && routeDelivered(dl, msg)) {
        all[msg.id] = { ...dl, delivered: now };
        await this.ctx.storage.put("downlinks", all);
        this.broadcast(downlinkSummary(all[msg.id]));
      }
      // Phao đã bị báo mất liên lạc nay nghe lại
      const lost = await this.loadLost();
      if (lost[msg.id]) {
        delete lost[msg.id];
        await this.ctx.storage.put("lost", lost);
        this.broadcast({ type: "link", id: msg.id, lost: false, since: now });
      }
      await this.checkLinks(now);
      // Đèn phao vừa đổi trạng thái thật: dashboard bỏ "đang gửi" khi khớp lệnh
      if (msg.lights !== undefined && msg.lights !== prev?.lights && (await this.loadLights())[msg.id]) {
        this.broadcast(await this.lightsMessage(msg.id));
      }
      if (msg.hold !== undefined && msg.hold !== prev?.hold && (await this.loadHold())[msg.id]) {
        this.broadcast(await this.holdMessage(msg.id));
      }
      return json(result);
    }

    console.log("hub: no route for", request.method, url.pathname);
    return new Response("Not found", { status: 404 });
  }

  webSocketMessage() { /* trình duyệt không gửi gì lên */ }
  webSocketClose(ws) { try { ws.close(); } catch { /* đã đóng */ } }
  webSocketError(ws) { try { ws.close(); } catch { /* đã đóng */ } }
}
