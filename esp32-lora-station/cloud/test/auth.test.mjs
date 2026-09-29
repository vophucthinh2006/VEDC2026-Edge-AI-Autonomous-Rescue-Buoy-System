// Kiểm thử đăng nhập/phân quyền của Worker đang chạy bằng `wrangler dev` (mặc định http://127.0.0.1:8787).
// Chạy:  node test/auth.test.mjs
// Yêu cầu .dev.vars: GOOGLE_CLIENT_ID=test-client-id, ADMIN_EMAILS chứa admin@example.com,
//                    GOOGLE_JWKS_URL=http://127.0.0.1:8799/jwks.json, INGEST_TOKEN=dev-token
import http from "node:http";
import crypto from "node:crypto";

const BASE = process.env.BASE || "http://127.0.0.1:8787";
const CLIENT_ID = "test-client-id";
const KID = "test-key-1";

const { publicKey, privateKey } = crypto.generateKeyPairSync("rsa", { modulusLength: 2048 });
const jwk = { ...publicKey.export({ format: "jwk" }), kid: KID, alg: "RS256", use: "sig" };
const other = crypto.generateKeyPairSync("rsa", { modulusLength: 2048 });

const server = http.createServer((req, res) => {
  res.setHeader("content-type", "application/json");
  res.end(JSON.stringify({ keys: [jwk] }));
}).listen(8799);

const b64u = b => Buffer.from(b).toString("base64url");
function mint(claims, { key = privateKey, alg = "RS256", kid = KID } = {}) {
  const now = Math.floor(Date.now() / 1000);
  const payload = { iss: "https://accounts.google.com", aud: CLIENT_ID, email: "admin@example.com", email_verified: true, iat: now, exp: now + 3600, ...claims };
  const head = b64u(JSON.stringify({ alg, kid, typ: "JWT" }));
  const body = b64u(JSON.stringify(payload));
  const sig = crypto.sign("RSA-SHA256", Buffer.from(`${head}.${body}`), key);
  return `${head}.${body}.${b64u(sig)}`;
}

let pass = 0, fail = 0;
function check(name, cond, extra = "") {
  if (cond) { pass++; console.log("  ok   " + name); } else { fail++; console.log("  FAIL " + name + " " + extra); }
}

async function call(method, path, { body, cookie, origin = BASE, raw } = {}) {
  const headers = {};
  if (cookie) headers.Cookie = cookie;
  if (origin) headers.Origin = origin;
  if (body !== undefined) headers["Content-Type"] = "application/json";
  const res = await fetch(BASE + path, { method, headers, body: raw ?? (body !== undefined ? JSON.stringify(body) : undefined) });
  const setCookie = res.headers.get("set-cookie");
  let data = null;
  try { data = await res.json(); } catch { /* không phải JSON */ }
  return { status: res.status, data, cookie: setCookie ? setCookie.split(";")[0] : null, setCookie };
}

const google = t => call("POST", "/api/auth/google", { body: { credential: t } });

console.log("== Đăng nhập Google (xác minh ID token)");
let r = await google(mint({}));
check("token hợp lệ + email trong danh sách -> 200 admin", r.status === 200 && r.data.role === "admin", JSON.stringify(r));
check("cookie HttpOnly + SameSite=Lax", /HttpOnly/.test(r.setCookie) && /SameSite=Lax/.test(r.setCookie));
const adminCookie = r.cookie;
r = await google(mint({ email: "Second@Example.com" }));
check("email thứ hai (khác hoa/thường) -> 200", r.status === 200, JSON.stringify(r));
r = await google(mint({ email: "stranger@example.com" }));
check("email ngoài danh sách -> 403", r.status === 403, JSON.stringify(r));
r = await google(mint({ aud: "other-client" }));
check("sai aud -> 401", r.status === 401);
r = await google(mint({ iss: "https://evil.example.com" }));
check("sai iss -> 401", r.status === 401);
r = await google(mint({ exp: Math.floor(Date.now() / 1000) - 10 }));
check("hết hạn -> 401", r.status === 401);
r = await google(mint({ email_verified: false }));
check("email chưa xác minh -> 401", r.status === 401);
r = await google(mint({}, { key: other.privateKey }));
check("chữ ký từ khóa lạ -> 401", r.status === 401);
r = await google(mint({}, { kid: "khong-ton-tai" }));
check("kid không có trong JWKS -> 401", r.status === 401);
r = await google(mint({}).slice(0, -6) + "AAAAAA");
check("chữ ký bị sửa -> 401", r.status === 401);
r = await google("khong.phai.token");
check("chuỗi rác -> 401", r.status === 401);
r = await call("POST", "/api/auth/google", { body: { credential: mint({}) }, origin: "https://evil.example.com" });
check("đăng nhập từ Origin lạ -> 403", r.status === 403);
r = await call("POST", "/api/auth/google", { body: { credential: mint({}) }, origin: null });
check("đăng nhập không có Origin -> 403", r.status === 403);

console.log("== Khách và không phiên");
r = await call("GET", "/api/me");
check("không phiên: /api/me role null", r.status === 200 && r.data.role === null);
for (const p of ["/api/state", "/api/mission", "/ws"]) {
  r = await call("GET", p);
  check(`không phiên: GET ${p} -> 401`, r.status === 401);
}
r = await call("PUT", "/api/mission", { body: { waypoints: [] } });
check("không phiên: PUT /api/mission -> 401", r.status === 401);
r = await call("GET", "/");
check("trang web tĩnh vẫn tải được không cần phiên", r.status === 200);
r = await call("POST", "/api/auth/guest");
check("vào khách -> 200", r.status === 200 && r.data.role === "guest");
const guestCookie = r.cookie;
r = await call("GET", "/api/me", { cookie: guestCookie });
check("phiên khách: /api/me role guest", r.data.role === "guest");
r = await call("GET", "/api/state", { cookie: guestCookie });
check("khách GET /api/state -> 200", r.status === 200);
r = await call("GET", "/api/mission", { cookie: guestCookie });
check("khách GET /api/mission -> 200", r.status === 200);
r = await call("PUT", "/api/mission", { cookie: guestCookie, body: { waypoints: [] } });
check("khách PUT /api/mission -> 403", r.status === 403);
r = await call("POST", "/api/auth/guest", { origin: "https://evil.example.com" });
check("vào khách từ Origin lạ -> 403", r.status === 403);

console.log("== Admin sửa lộ trình");
r = await call("GET", "/api/me", { cookie: adminCookie });
check("phiên admin: /api/me", r.data.role === "admin" && r.data.email === "admin@example.com");
const good = { waypoints: [{ lat: 10.76, lon: 106.66 }, { lat: 10.77, lon: 106.67 }], poly: [], spacing: 12, angle: null, speed: 2 };
const rev0 = (await call("GET", "/api/mission", { cookie: adminCookie })).data.rev;
r = await call("PUT", "/api/mission", { cookie: adminCookie, body: good });
check("admin PUT hợp lệ -> 200, rev tăng", r.status === 200 && r.data.rev === rev0 + 1 && r.data.by === "admin@example.com", JSON.stringify(r));
r = await call("GET", "/api/mission", { cookie: guestCookie });
check("khách thấy lộ trình admin vừa lưu", r.data.waypoints.length === 2 && r.data.speed === 2);
r = await call("PUT", "/api/mission", { cookie: adminCookie, body: good, origin: "https://evil.example.com" });
check("admin PUT từ Origin lạ -> 403", r.status === 403);
r = await call("PUT", "/api/mission", { cookie: adminCookie, body: { ...good, waypoints: [{ lat: 999, lon: 1 }] } });
check("lat 999 -> 400", r.status === 400);
r = await call("PUT", "/api/mission", { cookie: adminCookie, body: { ...good, waypoints: Array.from({ length: 301 }, () => ({ lat: 1, lon: 1 })) } });
check("301 điểm -> 400", r.status === 400);
r = await call("PUT", "/api/mission", { cookie: adminCookie, body: { ...good, spacing: 0 } });
check("spacing 0 -> 400", r.status === 400);
r = await call("PUT", "/api/mission", { cookie: adminCookie, raw: "{khong phai json" });
check("JSON hỏng -> 400", r.status === 400);
r = await call("PUT", "/api/mission", { cookie: adminCookie, raw: "x".repeat(40000) });
check("body quá lớn -> 413", r.status === 413);
const tampered = adminCookie.replace(/^session=[^.]+/, "session=" + b64u(JSON.stringify({ role: "admin", email: "admin@example.com", exp: 9999999999 })));
r = await call("PUT", "/api/mission", { cookie: tampered, body: good });
check("cookie bị giả payload -> 401", r.status === 401);
const guestToAdmin = guestCookie.replace(/^session=[^.]+/, "session=" + b64u(JSON.stringify({ role: "admin", email: "admin@example.com", exp: 9999999999 })));
r = await call("PUT", "/api/mission", { cookie: guestToAdmin, body: good });
check("nâng quyền khách -> admin bằng cách sửa cookie -> 401", r.status === 401);

console.log("== Realtime qua WebSocket");
function openWs(cookie) {
  return new Promise((resolve, reject) => {
    const ws = new WebSocket(BASE.replace("http", "ws") + "/ws", { headers: { Cookie: cookie } });
    const msgs = [];
    ws.onmessage = e => msgs.push(JSON.parse(e.data));
    ws.onopen = () => resolve({ ws, msgs });
    ws.onerror = () => reject(new Error("ws error"));
  });
}
const sleep = ms => new Promise(r => setTimeout(r, ms));
try {
  const g = await openWs(guestCookie);
  await sleep(500);
  check("WebSocket khách nhận lộ trình hiện tại khi vừa mở", g.msgs.some(m => m.type === "mission" && m.waypoints.length === 2));
  await call("PUT", "/api/mission", { cookie: adminCookie, body: { ...good, waypoints: [{ lat: 10.8, lon: 106.7 }] } });
  await sleep(500);
  const last = [...g.msgs].reverse().find(m => m.type === "mission");
  check("khách nhận realtime khi admin sửa", last && last.waypoints.length === 1 && last.rev > rev0 + 1);
  await fetch(BASE + "/api/ingest", { method: "POST", headers: { Authorization: "Bearer dev-token" }, body: JSON.stringify({ id: "T-1", fix: 1, lat: 10.5, lon: 106.5, rssi: -80, snr: 5, raw: "x" }) });
  await sleep(400);
  check("khách nhận bản tin phao", g.msgs.some(m => m.id === "T-1" && m.type === undefined));
  g.ws.close();
} catch (e) { check("WebSocket có thể mở với cookie", false, String(e)); }

console.log("== Lịch sử tín hiệu (history)");
try {
  const hid = "H-" + Date.now().toString(36);
  const post = (i) => fetch(BASE + "/api/ingest", { method: "POST", headers: { Authorization: "Bearer dev-token" }, body: JSON.stringify({ id: hid, fix: i % 5 === 0 ? 0 : 1, lat: 10 + i * 0.0001, lon: 106 + i * 0.0001, rssi: -80 - (i % 7), snr: 5, raw: "h" + i }) });
  for (let i = 1; i <= 305; i++) await post(i);
  const h = await openWs(guestCookie);
  await sleep(1200);
  const hist = h.msgs.find(m => m.type === "history" && m.id === hid);
  check("WebSocket gửi bản tin history cho phao vừa gửi 305 gói", !!hist);
  check("vòng đệm giữ đúng 300 mẫu", hist && hist.points.length === 300, hist ? String(hist.points.length) : "none");
  check("mẫu mới nhất ở cuối, mẫu cũ nhất đã bị bỏ", hist && hist.points[299].t >= hist.points[0].t);
  check("mẫu có rssi/snr; mẫu fix=0 không mang tọa độ", hist && hist.points.every(p => Number.isFinite(p.rssi) && Number.isFinite(p.snr)) && hist.points.some(p => p.lat === undefined) && hist.points.some(p => p.lat !== undefined));
  const iHist = h.msgs.findIndex(m => m.type === "history" && m.id === hid), iMis = h.msgs.findIndex(m => m.type === "mission");
  check("history được gửi trước bản tin mission", iHist >= 0 && iMis > iHist);
  h.ws.close();
  r = await call("GET", "/api/state", { cookie: guestCookie });
  check("phao mới có trong /api/state (giao thức cũ không đổi)", r.data.some(b => b.id === hid));
} catch (e) { check("kiểm thử history", false, String(e)); }

console.log("== Vị trí trạm bờ (/api/station)");
try {
  const st = (body, token = "dev-token") => fetch(BASE + "/api/station", { method: "POST", headers: { Authorization: "Bearer " + token }, body: JSON.stringify(body) });
  r = await fetch(BASE + "/api/station", { method: "POST", body: "{}" });
  check("station không token -> 401", r.status === 401);
  r = await st({ fix: 1, lat: 10.7, lon: 106.6 }, "sai-token");
  check("station sai token -> 401", r.status === 401);
  r = await fetch(BASE + "/api/station", { method: "POST", headers: { Cookie: adminCookie, Origin: BASE }, body: JSON.stringify({ fix: 1, lat: 10, lon: 106 }) });
  check("cookie admin không thay được token station -> 401", r.status === 401);
  r = await st("khong-phai-object");
  check("station payload sai -> 400", r.status === 400);
  const w = await openWs(guestCookie);
  await sleep(400);
  r = await st({ fix: 1, lat: 10.762622, lon: 106.660172, sats: 9, hdop: 0.94 });
  check("station hợp lệ -> 200", r.status === 200);
  await sleep(500);
  const live = w.msgs.filter(m => m.type === "station").pop();
  check("khách nhận bản tin station realtime", live && live.fix === 1 && live.lat === 10.762622 && live.sats === 9 && live.hdop === 0.9 && live.age_s === 0, JSON.stringify(live));
  await st({ fix: 1, lat: 999, lon: 1 });
  await sleep(300);
  const bad = w.msgs.filter(m => m.type === "station").pop();
  check("lat 999 bị hạ thành fix 0 (không tọa độ)", bad && bad.fix === 0 && bad.lat === undefined, JSON.stringify(bad));
  w.ws.close();
  const w2 = await openWs(guestCookie);
  await sleep(600);
  const snap = w2.msgs.find(m => m.type === "station");
  check("WebSocket mới nhận snapshot station với vị trí cuối còn giữ", snap && snap.fix === 0 && snap.lat === 10.762622 && Number.isFinite(snap.age_s), JSON.stringify(snap));
  w2.ws.close();
} catch (e) { check("kiểm thử station", false, String(e)); }

console.log("== Ingest không đổi");
r = await call("POST", "/api/ingest", { raw: "{}", origin: null });
check("ingest không token -> 401", r.status === 401);
r = await fetch(BASE + "/api/ingest", { method: "POST", headers: { Authorization: "Bearer dev-token" }, body: JSON.stringify({ fix: 0, rssi: -90, snr: 1, raw: "a" }) });
check("ingest đúng token -> 200 (không cần Origin/phiên)", r.status === 200);
r = await call("POST", "/api/ingest", { cookie: adminCookie, raw: "{}", origin: null });
check("cookie admin không thay được token ingest -> 401", r.status === 401);

console.log("== Đăng xuất");
r = await call("POST", "/api/auth/logout", { cookie: adminCookie });
check("logout xóa cookie (Max-Age=0)", r.status === 200 && /Max-Age=0/.test(r.setCookie));

console.log(`\n${pass} đạt, ${fail} lỗi`);
server.close();
process.exit(fail ? 1 : 0);
