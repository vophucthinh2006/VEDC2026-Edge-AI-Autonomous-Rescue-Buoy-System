import { json, num, secretEquals } from "./util.js";
import { adminEmails, makeSession, originOk, readSession, sessionCookie, verifyGoogleToken } from "./auth.js";

export { Hub } from "./hub.js";

const MAX_BODY = 2048;          // gói từ trạm
const MAX_MISSION_BODY = 32768; // lộ trình do admin gửi
const MAX_POINTS = 300;

const validPt = p => p && num(p.lat) !== null && num(p.lon) !== null && Math.abs(p.lat) <= 90 && Math.abs(p.lon) <= 180;
const cleanPts = arr => arr.map(p => ({ lat: +p.lat.toFixed(7), lon: +p.lon.toFixed(7) }));

// Chuẩn hóa và kiểm tra bản tin từ trạm; trả về null nếu không hợp lệ
function sanitize(m) {
  if (!m || typeof m !== "object") return null;
  const rssi = num(m.rssi), snr = num(m.snr);
  if (rssi === null || snr === null) return null;
  const id = typeof m.id === "string" && m.id.trim()
    ? m.id.replace(/[^\w .\-]/g, "").slice(0, 32) || "PHAO-01"
    : "PHAO-01";
  const out = {
    id,
    fix: m.fix === 1 ? 1 : 0,
    rssi: Math.round(rssi),
    snr: Math.round(snr * 10) / 10,
    raw: typeof m.raw === "string" ? m.raw.slice(0, 100) : "",
  };
  if (out.fix === 1) {
    const lat = num(m.lat), lon = num(m.lon);
    if (lat === null || lon === null || Math.abs(lat) > 90 || Math.abs(lon) > 180) out.fix = 0;
    else { out.lat = lat; out.lon = lon; }
  }
  return out;
}

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
      try { msg = sanitize(JSON.parse(body)); } catch { /* rơi xuống báo lỗi */ }
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

    // ---- Dữ liệu: cần phiên (khách hoặc admin) ----
    if (path === "/ws" || path === "/api/state" || path === "/api/mission") {
      if (!session) return json({ error: "unauthenticated" }, 401);

      if (path === "/ws") return hub().fetch(request);
      if (path === "/api/state") return hub().fetch("https://hub/state");

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
