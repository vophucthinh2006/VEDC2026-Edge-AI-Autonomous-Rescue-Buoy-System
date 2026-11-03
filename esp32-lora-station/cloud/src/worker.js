import { json, num, secretEquals } from "./util.js";
import { adminEmails, devSignInUrl, makeSession, originOk, readSession, sessionCookie, verifyGoogleToken } from "./auth.js";
import { sanitizeBuoy } from "./telemetry.js";

export { Hub } from "./hub.js";

const MAX_BODY = 2048;          // gói từ trạm
const MAX_MISSION_BODY = 32768; // lộ trình do admin gửi
const MAX_POINTS = 300;

const validPt = p => p && num(p.lat) !== null && num(p.lon) !== null && Math.abs(p.lat) <= 90 && Math.abs(p.lon) <= 180;
const cleanPts = arr => arr.map(p => ({ lat: +p.lat.toFixed(7), lon: +p.lon.toFixed(7) }));

// Chuẩn hóa vị trí trạm bờ (GPS của trạm); trả về null nếu không hợp lệ
function sanitizeStation(m) {
  if (!m || typeof m !== "object") return null;
  const out = { fix: m.fix === 1 ? 1 : 0 };
  if (out.fix === 1) {
    const lat = num(m.lat), lon = num(m.lon);
    if (lat === null || lon === null || Math.abs(lat) > 90 || Math.abs(lon) > 180) out.fix = 0;
    else {
      out.lat = lat; out.lon = lon;
      const sats = num(m.sats), hdop = num(m.hdop);
      if (sats !== null && sats >= 0 && sats <= 99) out.sats = Math.round(sats);
      if (hdop !== null && hdop >= 0 && hdop < 1000) out.hdop = Math.round(hdop * 10) / 10;
    }
  }
  return out;
}

// Chuẩn hóa lộ trình do admin gửi; trả về null nếu không hợp lệ
function sanitizeMission(m) {
  if (!m || typeof m !== "object") return null;
  if (!Array.isArray(m.waypoints) || m.waypoints.length > MAX_POINTS || !m.waypoints.every(validPt)) return null;
  const poly = m.poly ?? [];
  if (!Array.isArray(poly) || poly.length > MAX_POINTS || !poly.every(validPt)) return null;
  const spacing = m.spacing === undefined ? 10 : num(m.spacing);
  const speed = m.speed === undefined ? 1.5 : num(m.speed);
  const angle = m.angle === null || m.angle === undefined ? null : num(m.angle);
  if (spacing === null || spacing < 1 || spacing > 500) return null;
  if (speed === null || speed < 0.1 || speed > 20) return null;
  if (angle !== null && (angle < 0 || angle >= 360)) return null;
  const cid = typeof m.cid === "string" ? m.cid.replace(/[^\w-]/g, "").slice(0, 32) : null;
  return { waypoints: cleanPts(m.waypoints), poly: cleanPts(poly), spacing, angle, speed, cid };
}

// Yêu cầu gửi lộ trình xuống phao: { id?: tên phao, loop?: đi vòng, clear?: gửi lộ trình rỗng,
// pts?: [[lat, lon], ...] lộ trình riêng cho phao này (chia vùng), thay cho lộ trình chung đang lưu }
export function sanitizeSend(m) {
  if (!m || typeof m !== "object") return null;
  const id = m.id === undefined ? "PHAO-01" : typeof m.id === "string" ? m.id.replace(/[^\w.\-]/g, "").slice(0, 31) : "";
  if (!id) return null;
  if (m.loop !== undefined && typeof m.loop !== "boolean") return null;
  if (m.clear !== undefined && typeof m.clear !== "boolean") return null;
  const out = { id, loop: m.loop === true, clear: m.clear === true };
  if (m.pts !== undefined) {
    if (!Array.isArray(m.pts) || m.pts.length > MAX_POINTS) return null;
    const pts = m.pts.map(p => (Array.isArray(p) ? { lat: p[0], lon: p[1] } : null));
    if (!pts.every(validPt)) return null;
    out.pts = cleanPts(pts).map(p => [p.lat, p.lon]);
  }
  return out;
}

// Lệnh về bờ: { id: tên phao, ret: true (về điểm đỗ) | false (đi tiếp lộ trình) }
function sanitizeReturn(m) {
  if (!m || typeof m !== "object" || typeof m.id !== "string") return null;
  const id = m.id.replace(/[^\w.\-]/g, "").slice(0, 31);
  if (!id || typeof m.ret !== "boolean") return null;
  return { id, ret: m.ret };
}

// Admin về một người (docs/victim-claim.md): { action: rescued|release|assign, id? } (assign cần tên phao)
export function sanitizeVictimAction(m) {
  if (!m || typeof m !== "object" || !["rescued", "release", "assign"].includes(m.action)) return null;
  if (m.action !== "assign") return { action: m.action };
  if (typeof m.id !== "string") return null;
  const id = m.id.replace(/[^\w.\-]/g, "").slice(0, 31);
  return id ? { action: m.action, id } : null;
}

// Lệnh cả đội: { action: hold|resume|return, ids?: [tên phao] } (không có ids: mọi phao nghe được gần đây)
export function sanitizeFleet(m) {
  if (!m || typeof m !== "object" || !["hold", "resume", "return"].includes(m.action)) return null;
  const out = { action: m.action };
  if (m.ids !== undefined) {
    if (!Array.isArray(m.ids) || m.ids.length > 50 || !m.ids.every(id => typeof id === "string")) return null;
    out.ids = m.ids.map(id => id.replace(/[^\w.\-]/g, "").slice(0, 31)).filter(Boolean);
  }
  return out;
}

// Lệnh đèn: { id: tên phao, lights: 0..3 } (bit0 = đèn 1 PC3, bit1 = đèn 2 PC2)
function sanitizeLights(m) {
  if (!m || typeof m !== "object" || typeof m.id !== "string") return null;
  const id = m.id.replace(/[^\w.\-]/g, "").slice(0, 31);
  if (!id || !Number.isInteger(m.lights) || m.lights < 0 || m.lights > 3) return null;
  return { id, lights: m.lights };
}

// Lệnh dừng: { id: tên phao, hold: true (dừng) | false (chạy tiếp) }
// Mực nước cho lớp độ sâu ngập (docs/weather-map.md): { code } (một trạm thủy văn), { level_m } (nhập tay, -5..30 m)
// hoặc {} (trạm gần trạm bờ nhất)
export function sanitizeFloodLevel(m) {
  if (!m || typeof m !== "object") return null;
  if (typeof m.code === "string") {
    const code = m.code.replace(/[^\w.\-]/g, "").slice(0, 20);
    return code ? { code } : null;
  }
  if (m.level_m !== undefined) return Number.isFinite(m.level_m) && m.level_m >= -5 && m.level_m <= 30 ? { level_m: +m.level_m.toFixed(2) } : null;
  return {};
}

function sanitizeHold(m) {
  if (!m || typeof m !== "object" || typeof m.id !== "string") return null;
  const id = m.id.replace(/[^\w.\-]/g, "").slice(0, 31);
  if (!id || typeof m.hold !== "boolean") return null;
  return { id, hold: m.hold };
}

// Kế hoạch cho cả đội (src/fleetplan.js): { mode: "sectors" | "formation", poly: [{lat, lon}] (3..300 điểm),
// ids: [tên phao] (1..8), spacing? 2..200 m, angle? 0..359 độ, shape? line|v|column, leader?, loop?, dry? }
export function sanitizePlan(m) {
  if (!m || typeof m !== "object" || !["sectors", "formation"].includes(m.mode)) return null;
  if (!Array.isArray(m.poly) || m.poly.length < 3 || m.poly.length > MAX_POINTS || !m.poly.every(validPt)) return null;
  if (!Array.isArray(m.ids) || m.ids.length < 1 || m.ids.length > 8 || !m.ids.every(id => typeof id === "string")) return null;
  const ids = [...new Set(m.ids.map(id => id.replace(/[^\w.\-]/g, "").slice(0, 31)).filter(Boolean))];
  if (!ids.length) return null;
  const spacing = m.spacing === undefined ? 10 : num(m.spacing);
  const angle = m.angle === undefined || m.angle === null ? null : num(m.angle);
  if (spacing === null || spacing < 2 || spacing > 200) return null;
  if (angle !== null && (angle < 0 || angle >= 360)) return null;
  const shape = m.shape === undefined ? "line" : m.shape;
  if (!["line", "v", "column"].includes(shape)) return null;
  const leader = typeof m.leader === "string" ? m.leader.replace(/[^\w.\-]/g, "").slice(0, 31) : null;
  return { mode: m.mode, poly: cleanPts(m.poly), ids, spacing, angle, shape, leader, loop: m.loop === true, dry: m.dry === true };
}

// Lệnh admin: phiên admin từ trình duyệt (kèm Origin đúng), hoặc Bearer ADMIN_API_TOKEN cho script thử.
// Trả về { by } nếu được phép, ngược lại { error: Response }.
async function adminCaller(request, env, session) {
  const auth = request.headers.get("Authorization") || "";
  if (auth) {
    if (env.ADMIN_API_TOKEN && (await secretEquals(auth, "Bearer " + env.ADMIN_API_TOKEN))) return { by: "api" };
    return { error: json({ error: "unauthorized" }, 401) };
  }
  if (!session) return { error: json({ error: "unauthenticated" }, 401) };
  if (session.role !== "admin") return { error: json({ error: "Chỉ admin được ra lệnh này" }, 403) };
  if (!originOk(request)) return { error: json({ error: "forbidden origin" }, 403) };
  return { by: session.email };
}

export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    const path = url.pathname;
    const hub = () => env.HUB.get(env.HUB.idFromName("main"));

    // ---- Trạm bờ (ESP32) đẩy dữ liệu lên, xác thực bằng token riêng ----
    if (path === "/api/ingest") {
      if (request.method !== "POST") return json({ error: "method" }, 405);
      const auth = request.headers.get("Authorization") || "";
      if (!env.INGEST_TOKEN || !(await secretEquals(auth, "Bearer " + env.INGEST_TOKEN))) {
        return json({ error: "unauthorized" }, 401);
      }
      const body = await request.text();
      if (body.length > MAX_BODY) return json({ error: "too large" }, 413);
      let msg = null;
      try { msg = sanitizeBuoy(JSON.parse(body)); } catch { /* rơi xuống báo lỗi */ }
      if (!msg) return json({ error: "bad payload" }, 400);
      return hub().fetch("https://hub/ingest", { method: "POST", body: JSON.stringify(msg) });
    }

    // Trạm bờ báo vị trí GPS của chính nó (cùng token với /api/ingest)
    if (path === "/api/station") {
      if (request.method !== "POST") return json({ error: "method" }, 405);
      const auth = request.headers.get("Authorization") || "";
      if (!env.INGEST_TOKEN || !(await secretEquals(auth, "Bearer " + env.INGEST_TOKEN))) {
        return json({ error: "unauthorized" }, 401);
      }
      const body = await request.text();
      if (body.length > MAX_BODY) return json({ error: "too large" }, 413);
      let msg = null;
      try { msg = sanitizeStation(JSON.parse(body)); } catch { /* rơi xuống báo lỗi */ }
      if (!msg) return json({ error: "bad payload" }, 400);
      return hub().fetch("https://hub/station", { method: "POST", body: JSON.stringify(msg) });
    }

    // Trạm bờ lấy lộ trình admin đã bấm "Gửi tới phao" để phát LoRa (docs/lora-route-downlink.md)
    if (path === "/api/downlink") {
      if (request.method !== "GET") return json({ error: "method" }, 405);
      const auth = request.headers.get("Authorization") || "";
      if (!env.INGEST_TOKEN || !(await secretEquals(auth, "Bearer " + env.INGEST_TOKEN))) {
        return json({ error: "unauthorized" }, 401);
      }
      return hub().fetch("https://hub/downlink");
    }

    // ---- API công khai: cấu hình, trạng thái phiên, đăng nhập/đăng xuất ----
    if (path === "/api/config") return json({ googleClientId: env.GOOGLE_CLIENT_ID || "", devSignIn: devSignInUrl(env) });

    const session = await readSession(request, env);

    if (path === "/api/me") return json({ role: session?.role ?? null, email: session?.email ?? null });

    if (path.startsWith("/api/auth/")) {
      if (request.method !== "POST") return json({ error: "method" }, 405);
      if (!originOk(request)) return json({ error: "forbidden origin" }, 403);

      if (path === "/api/auth/guest") {
        const s = await makeSession(env, "guest", null);
        return json({ role: "guest" }, 200, { "Set-Cookie": sessionCookie(request, s.value, s.maxAge) });
      }
      if (path === "/api/auth/google") {
        let email;
        try {
          const body = await request.json();
          email = await verifyGoogleToken(body.credential, env);
        } catch { return json({ error: "Token Google không hợp lệ" }, 401); }
        if (!adminEmails(env).includes(email)) return json({ error: "Email này không có quyền admin" }, 403);
        const s = await makeSession(env, "admin", email);
        return json({ role: "admin", email }, 200, { "Set-Cookie": sessionCookie(request, s.value, s.maxAge) });
      }
      if (path === "/api/auth/logout") {
        return json({ ok: true }, 200, { "Set-Cookie": sessionCookie(request, "", 0) });
      }
      return json({ error: "not found" }, 404);
    }

    // ---- Gửi lộ trình xuống một phao qua trạm bờ: admin (hoặc token ADMIN_API_TOKEN cho script chạy thử) ----
    if (path === "/api/mission/send") {
      if (request.method !== "POST") return json({ error: "method" }, 405);
      const caller = await adminCaller(request, env, session);
      if (caller.error) return caller.error;
      const body = await request.text();
      if (body.length > MAX_MISSION_BODY) return json({ error: "too large" }, 413);
      let req = null;
      try { req = sanitizeSend(body ? JSON.parse(body) : {}); } catch { /* báo lỗi bên dưới */ }
      if (!req) return json({ error: "bad request" }, 400);
      return hub().fetch("https://hub/downlink", { method: "PUT", body: JSON.stringify({ ...req, by: caller.by }) });
    }

    // ---- Kế hoạch cho cả đội (chia vùng / đội hình), kết thúc đội hình: admin (hoặc token ADMIN_API_TOKEN) ----
    if (path === "/api/fleet/plan" || path === "/api/formation/end") {
      if (request.method !== "POST") return json({ error: "method" }, 405);
      const caller = await adminCaller(request, env, session);
      if (caller.error) return caller.error;
      if (path === "/api/formation/end") return hub().fetch("https://hub/formation", { method: "DELETE", body: JSON.stringify({ by: caller.by }) });
      const body = await request.text();
      if (body.length > MAX_MISSION_BODY) return json({ error: "too large" }, 413);
      let req = null;
      try { req = sanitizePlan(JSON.parse(body)); } catch { /* báo lỗi bên dưới */ }
      if (!req) return json({ error: "bad request" }, 400);
      return hub().fetch("https://hub/plan", { method: "PUT", body: JSON.stringify({ ...req, by: caller.by }) });
    }

    // ---- Bật/tắt đèn phao, dừng phao, về bờ, lệnh cả đội, xóa chấm người: admin (hoặc token ADMIN_API_TOKEN) ----
    const COMMANDS = { "/api/lights": [sanitizeLights, "lights"], "/api/hold": [sanitizeHold, "hold"], "/api/flood-level": [sanitizeFloodLevel, "flood-level"],
      "/api/return": [sanitizeReturn, "return"], "/api/fleet": [sanitizeFleet, "fleet"] };
    // Admin về một người (docs/victim-claim.md): đã cứu, nhả cho phao khác, giao cho một phao
    const victimPath = path.match(/^\/api\/victims\/(\d+)$/);
    if (victimPath) {
      if (request.method !== "POST") return json({ error: "method" }, 405);
      const caller = await adminCaller(request, env, session);
      if (caller.error) return caller.error;
      const body = await request.text();
      if (body.length > MAX_BODY) return json({ error: "too large" }, 413);
      let req = null;
      try { req = sanitizeVictimAction(JSON.parse(body)); } catch { /* báo lỗi bên dưới */ }
      if (!req) return json({ error: "bad request" }, 400);
      return hub().fetch("https://hub/victim", { method: "PUT", body: JSON.stringify({ ...req, vid: Number(victimPath[1]), by: caller.by }) });
    }

    if (COMMANDS[path] || path === "/api/victims/clear") {
      if (request.method !== "POST") return json({ error: "method" }, 405);
      const caller = await adminCaller(request, env, session);
      if (caller.error) return caller.error;
      if (path === "/api/victims/clear") return hub().fetch("https://hub/victims", { method: "DELETE" });
      const body = await request.text();
      if (body.length > MAX_BODY) return json({ error: "too large" }, 413);
      const [sanitize, target] = COMMANDS[path];
      let req = null;
      try { req = sanitize(JSON.parse(body)); } catch { /* báo lỗi bên dưới */ }
      if (!req) return json({ error: "bad request" }, 400);
      return hub().fetch("https://hub/" + target, { method: "PUT", body: JSON.stringify({ ...req, by: caller.by }) });
    }

    // ---- Address search for the map: guest or admin session, answered by Nominatim through the Hub ----
    // ---- Thời tiết, mưa, ngập cho bản đồ (docs/weather-map.md): khách hoặc admin ----
    if (path === "/api/weather" || path === "/api/gauges") {
      if (!session) return json({ error: "unauthenticated" }, 401);
      if (request.method !== "GET") return json({ error: "method" }, 405);
      if (path === "/api/gauges") return hub().fetch("https://hub/gauges");
      const p = new URLSearchParams({ lat: url.searchParams.get("lat") || "", lon: url.searchParams.get("lon") || "" });
      return hub().fetch("https://hub/weather?" + p);
    }

    if (path === "/api/geocode") {
      if (!session) return json({ error: "unauthenticated" }, 401);
      if (request.method !== "GET") return json({ error: "method" }, 405);
      const q = (url.searchParams.get("q") || "").slice(0, 400);
      return hub().fetch("https://hub/geocode?" + new URLSearchParams({ q, sid: session.sid }));
    }

    // ---- Dữ liệu: cần phiên (khách hoặc admin) ----
    if (path === "/ws" || path === "/api/state" || path === "/api/mission" || path === "/api/victims") {
      if (!session) return json({ error: "unauthenticated" }, 401);

      if (path === "/ws") return hub().fetch(request);
      if (path === "/api/state") return hub().fetch("https://hub/state");
      if (path === "/api/victims") return request.method === "GET" ? hub().fetch("https://hub/victims") : json({ error: "method" }, 405);

      if (request.method === "GET") return hub().fetch("https://hub/mission");
      if (request.method === "PUT") {
        if (session.role !== "admin") return json({ error: "Chỉ admin được sửa lộ trình" }, 403);
        if (!originOk(request)) return json({ error: "forbidden origin" }, 403);
        const body = await request.text();
        if (body.length > MAX_MISSION_BODY) return json({ error: "too large" }, 413);
        let mission = null;
        try { mission = sanitizeMission(JSON.parse(body)); } catch { /* báo lỗi bên dưới */ }
        if (!mission) return json({ error: "bad mission" }, 400);
        return hub().fetch("https://hub/mission", { method: "PUT", body: JSON.stringify({ mission, by: session.email }) });
      }
      return json({ error: "method" }, 405);
    }

    // ---- File web tĩnh: công khai (không chứa bí mật) ----
    return env.ASSETS.fetch(request);
  },
};
