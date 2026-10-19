import { DurableObject } from "cloudflare:workers";
import { json } from "./util.js";

const MAX_BUOYS = 50;
const MAX_HISTORY = 300;   // số mẫu RSSI/SNR/vị trí giữ cho mỗi phao
// LoRa đến 5 giây một gói, cầu nối MAVLink thì 1-2 gói mỗi giây. Gói MAVLink vẫn được phát
// realtime, nhưng chỉ ghi kho và lịch sử tối đa mỗi PERSIST_MS một lần cho mỗi phao, để số
// lần ghi Durable Object không vượt hạn mức miễn phí (khoảng 100 nghìn dòng mỗi ngày).
// Gói LoRa luôn được ghi: ESP32 xếp hàng khi mất mạng rồi gửi dồn, mỗi gói là một mẫu thật.
const PERSIST_MS = 5000;
const EMPTY_MISSION = { rev: 0, waypoints: [], poly: [], spacing: 10, angle: null, speed: 1.5 };

// Bản tin cho trình duyệt: không kèm tọa độ (đã có trong bản tin mission)
const downlinkSummary = d => ({ type: "downlink", rev: d.rev, id: d.id, loop: d.loop, n: d.pts.length, mission_rev: d.mission_rev, by: d.by, at: d.at, delivered: d.delivered || null });

// Phao đích báo Pi đã nhận đủ bản gửi này: cùng rev, đủ điểm, pi = 1
const routeDelivered = (d, msg) => !!d.rev && !d.delivered && msg.id === d.id && !!msg.route &&
  msg.route.rev === d.rev && msg.route.have === d.pts.length && msg.route.pi === 1;

// Một Durable Object duy nhất giữ vị trí mới nhất của từng phao, lộ trình chung,
// và phát cả hai cho mọi trình duyệt đang mở.
export class Hub extends DurableObject {
  buoys = null;     // id -> { msg, t }
  mission = null;   // { rev, waypoints, poly, spacing, angle, speed, by?, at? }
  downlink = null;  // lộ trình đang gửi xuống phao (docs/lora-route-downlink.md)
  station = undefined;   // { msg, t } vị trí GPS trạm bờ; undefined = chưa tải, null = chưa có
  hist = {};        // id -> [{ t, rssi, snr, lat?, lon? }], mỗi phao một khóa lưu trữ `h:<id>`
  persistedAt = {}; // id -> thời điểm ghi kho gần nhất (chỉ trong bộ nhớ)

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
      return new Response(null, { status: 101, webSocket: client });
    }

    if (url.pathname === "/downlink") {
      if (request.method === "GET") {
        // Đã giao thì trả {rev: 0}: F407 khởi động lại báo rev 0, gửi lại bản cũ là thuyền tự chạy trên bờ
        const dl = await this.loadDownlink();
        return json(dl.delivered ? { rev: 0 } : dl);
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
      // Bản tin NO_FIX không có tọa độ: giữ vị trí cũ trong kho nhưng vẫn phát bản tin mới
      const prev = buoys[msg.id]?.msg;
      const stored = msg.fix === 0 && prev && (prev.fix === 1 || prev.lat !== undefined)
        ? { ...msg, lat: prev.lat, lon: prev.lon, fix_lost: 1 } : msg;
      const now = Date.now();
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
      return json({ ok: true, clients: this.ctx.getWebSockets().length });
    }

    return new Response("Not found", { status: 404 });
  }

  webSocketMessage() { /* trình duyệt không gửi gì lên */ }
  webSocketClose(ws) { try { ws.close(); } catch { /* đã đóng */ } }
  webSocketError(ws) { try { ws.close(); } catch { /* đã đóng */ } }
}
