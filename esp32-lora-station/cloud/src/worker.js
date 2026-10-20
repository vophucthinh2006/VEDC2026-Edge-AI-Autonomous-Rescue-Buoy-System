import { json, num, secretEquals } from "./util.js";
import { adminEmails, makeSession, originOk, readSession, sessionCookie, verifyGoogleToken } from "./auth.js";
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

// Yêu cầu gửi lộ trình xuống phao: { id?: tên phao, loop?: đi vòng, clear?: gửi lộ trình rỗng }
function sanitizeSend(m) {
  if (!m || typeof m !== "object") return null;
  const id = m.id === undefined ? "PHAO-01" : typeof m.id === "string" ? m.id.replace(/[^\w.\-]/g, "").slice(0, 31) : "";
  if (!id) return null;
  if (m.loop !== undefined && typeof m.loop !== "boolean") return null;
  if (m.clear !== undefined && typeof m.clear !== "boolean") return null;
  return { id, loop: m.loop === true, clear: m.clear === true };
}

// Lệnh đèn: { id: tên phao, lights: 0..3 } (bit0 = đèn 1 PC3, bit1 = đèn 2 PC2)
function sanitizeLights(m) {
  if (!m || typeof m !== "object" || typeof m.id !== "string") return null;
  const id = m.id.replace(/[^\w.\-]/g, "").slice(0, 31);
  if (!id || !Number.isInteger(m.lights) || m.lights < 0 || m.lights > 3) return null;
  return { id, lights: m.lights };
}

// Lệnh dừng: { id: tên phao, hold: true (dừng) | false (chạy tiếp) }
function sanitizeHold(m) {
  if (!m || typeof m !== "object" || typeof m.id !== "string") return null;
  const id = m.id.replace(/[^\w.\-]/g, "").slice(0, 31);
  if (!id || typeof m.hold !== "boolean") return null;
  return { id, hold: m.hold };
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
    if (path === "/api/config") return json({ googleClientId: env.GOOGLE_CLIENT_ID || "" });

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

    // ---- Gửi lộ trình đang lưu xuống phao qua trạm bờ: chỉ admin ----
    if (path === "/api/mission/send") {
      if (!session) return json({ error: "unauthenticated" }, 401);
      if (request.method !== "POST") return json({ error: "method" }, 405);
      if (session.role !== "admin") return json({ error: "Chỉ admin được gửi lộ trình" }, 403);
      if (!originOk(request)) return json({ error: "forbidden origin" }, 403);
      const body = await request.text();
      if (body.length > MAX_BODY) return json({ error: "too large" }, 413);
      let req = null;
      try { req = sanitizeSend(body ? JSON.parse(body) : {}); } catch { /* báo lỗi bên dưới */ }
      if (!req) return json({ error: "bad request" }, 400);
      return hub().fetch("https://hub/downlink", { method: "PUT", body: JSON.stringify({ ...req, by: session.email }) });
    }

    // ---- Bật/tắt đèn phao, dừng phao, xóa chấm người: admin (hoặc token ADMIN_API_TOKEN) ----
    if (path === "/api/lights" || path === "/api/hold" || path === "/api/victims/clear") {
      if (request.method !== "POST") return json({ error: "method" }, 405);
      const caller = await adminCaller(request, env, session);
      if (caller.error) return caller.error;
      if (path === "/api/victims/clear") return hub().fetch("https://hub/victims", { method: "DELETE" });
      const body = await request.text();
      if (body.length > MAX_BODY) return json({ error: "too large" }, 413);
      let req = null;
      const hold = path === "/api/hold";
      try { req = (hold ? sanitizeHold : sanitizeLights)(JSON.parse(body)); } catch { /* báo lỗi bên dưới */ }
      if (!req) return json({ error: "bad request" }, 400);
      return hub().fetch(hold ? "https://hub/hold" : "https://hub/lights", { method: "PUT", body: JSON.stringify({ ...req, by: caller.by }) });
    }

    // ---- Address search for the map: guest or admin session, answered by Nominatim through the Hub ----
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
