import { DurableObject } from "cloudflare:workers";
import { json } from "./util.js";
import { Geocoder } from "./geocode.js";

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
const VICTIM_MERGE_M = 5;              // phát hiện mới cách chấm cũ của cùng phao dưới 5 m: cùng một người
const EVENT_DEDUP_MS = 10 * 60 * 1000; // phao gửi lặp sự kiện tới khi trạm ACK; phao khởi động lại thì seq đếm lại
const MAX_SEEN = 64;                   // số sự kiện gần nhất nhớ cho mỗi phao để bỏ bản lặp

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

// Bản tin cho trình duyệt: không kèm tọa độ (đã có trong bản tin mission)
const downlinkSummary = d => ({ type: "downlink", rev: d.rev, id: d.id, loop: d.loop, n: d.pts.length, mission_rev: d.mission_rev, by: d.by, at: d.at, delivered: d.delivered || null });

// Phao đích báo Pi đã nhận đủ bản gửi này: cùng rev, đủ điểm, pi = 1
const routeDelivered = (d, msg) => !!d.rev && !d.delivered && msg.id === d.id && !!msg.route &&
  msg.route.rev === d.rev && msg.route.have === d.pts.length && msg.route.pi === 1;

// Nominatim asks for an identifying User-Agent; GEOCODE_CONTACT (an email or URL) is optional
const geocodeUserAgent = env =>
  `mliot-redgescue/1.0 (rescue buoy shore station dashboard${env?.GEOCODE_CONTACT ? "; " + env.GEOCODE_CONTACT : ""})`;

// Một Durable Object duy nhất giữ vị trí mới nhất của từng phao, lộ trình chung,
// và phát cả hai cho mọi trình duyệt đang mở.
export class Hub extends DurableObject {
  buoys = null;     // id -> { msg, t }
  mission = null;   // { rev, waypoints, poly, spacing, angle, speed, by?, at? }
  downlink = null;  // lộ trình đang gửi xuống phao (docs/lora-route-downlink.md)
  station = undefined;   // { msg, t } vị trí GPS trạm bờ; undefined = chưa tải, null = chưa có
  hist = {};        // id -> [{ t, rssi, snr, lat?, lon? }], mỗi phao một khóa lưu trữ `h:<id>`
  persistedAt = {}; // id -> thời điểm ghi kho gần nhất (chỉ trong bộ nhớ)
  // Người phát hiện và cảnh báo: { victims: [{ vid, buoy, lat, lon, risk, conf, first_at, last_at }],
  // alerts: [{ type, buoy, seq, at, lat, lon, ... }], seen: id -> [{ seq, type, t }], vid }
  events = null;
  lights = null;    // id -> { want, by, at }: lệnh đèn của admin, tách khỏi bản gửi lộ trình
  hold = null;      // id -> { want, by, at }: lệnh DỪNG của admin (want true = F407 khóa motor), như đèn
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

  // Bản chụp lộ trình admin đã bấm "Gửi tới phao": { rev, id, loop, pts: [[lat, lon]], by, at, delivered? }
  // delivered: lúc phao báo Pi đã nhận đủ; từ đó trạm không được mời gửi lại bản này
  async loadDownlink() {
    if (!this.downlink) this.downlink = (await this.ctx.storage.get("downlink")) || { rev: 0 };
    return this.downlink;
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

  // Ghi một sự kiện mới; trả về false nếu là bản phao gửi lặp
  async recordEvent(id, e, now) {
    const ev = await this.loadEvents();
    const seen = ev.seen[id] || (ev.seen[id] = []);
    if (seen.some(s => s.seq === e.seq && s.type === e.type && now - s.t < EVENT_DEDUP_MS)) return false;
    seen.push({ seq: e.seq, type: e.type, t: now });
    if (seen.length > MAX_SEEN) seen.splice(0, seen.length - MAX_SEEN);

    let out;
    if (e.type === "V" && e.lat !== null) {
      // Cùng phao báo lại gần chỗ cũ: cập nhật chấm cũ thay vì thêm chấm mới
      let v = ev.victims.find(x => x.buoy === id && distM(x, e) < VICTIM_MERGE_M);
      const isNew = !v;
      if (v) Object.assign(v, { risk: e.risk, conf: e.conf, last_at: now });
      else {
        v = { vid: ++ev.vid, buoy: id, lat: e.lat, lon: e.lon, risk: e.risk, conf: e.conf, first_at: now, last_at: now };
        ev.victims.push(v);
        if (ev.victims.length > MAX_VICTIMS) ev.victims.splice(0, ev.victims.length - MAX_VICTIMS);
      }
      out = { type: "victim", victim: v, new: isNew };
    } else {
      // Lật, xác nhận, hoặc phát hiện người khi phao chưa có GPS: chỉ là cảnh báo
      const { seq, type, ...rest } = e;
      const alert = { type, buoy: id, seq, at: now, ...rest };
      ev.alerts.push(alert);
      if (ev.alerts.length > MAX_ALERTS) ev.alerts.splice(0, ev.alerts.length - MAX_ALERTS);
      out = { type: "alert", alert };
    }
    await this.ctx.storage.put("events", ev);
    this.broadcast(out);
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
      const dl = await this.loadDownlink();
      if (dl.rev) server.send(JSON.stringify(downlinkSummary(dl)));
      const ev = await this.loadEvents();
      if (ev.victims.length || ev.alerts.length) server.send(JSON.stringify({ type: "victims", victims: ev.victims, alerts: ev.alerts }));
      for (const id of Object.keys(await this.loadLights())) server.send(JSON.stringify(await this.lightsMessage(id)));
      for (const id of Object.keys(await this.loadHold())) server.send(JSON.stringify(await this.holdMessage(id)));
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
        // Đã giao thì trả {rev: 0}: F407 khởi động lại báo rev 0, gửi lại bản cũ là thuyền tự chạy trên bờ
        const dl = await this.loadDownlink();
        const out = dl.delivered ? { rev: 0 } : { ...dl };
        // Lệnh đèn luôn đi kèm, kể cả khi không có lộ trình nào đang chờ giao
        const lights = Object.entries(await this.loadLights());
        if (lights.length) out.lights = Object.fromEntries(lights.map(([id, l]) => [id, l.want]));
        // Lệnh dừng cũng vậy: 1 = F407 khóa motor tới khi admin bấm "Chạy tiếp"
        const hold = Object.entries(await this.loadHold());
        if (hold.length) out.hold = Object.fromEntries(hold.map(([id, h]) => [id, h.want ? 1 : 0]));
        return json(out);
      }
      if (request.method === "PUT") {
        const { id, loop, clear, by } = await request.json();
        const mission = await this.loadMission();
        const prev = await this.loadDownlink();
        const pts = clear ? [] : mission.waypoints.map(p => [p.lat, p.lon]);
        // rev 1..65535 trên LoRa, 0 nghĩa là "chưa có"
        this.downlink = { rev: (prev.rev % 65535) + 1, id, loop: !!loop, pts, mission_rev: mission.rev, by: by || null, at: Date.now() };
        await this.ctx.storage.put("downlink", this.downlink);
        const summary = downlinkSummary(this.downlink);
        this.broadcast(summary);
        return json(summary);
      }
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
      all[id] = { want: hold === true, by: by || null, at: Date.now() };
      await this.ctx.storage.put("hold", all);
      const m = await this.holdMessage(id);
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
      const dl = await this.loadDownlink();
      if (routeDelivered(dl, msg)) {
        this.downlink = { ...dl, delivered: now };
        await this.ctx.storage.put("downlink", this.downlink);
        this.broadcast(downlinkSummary(this.downlink));
      }
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
