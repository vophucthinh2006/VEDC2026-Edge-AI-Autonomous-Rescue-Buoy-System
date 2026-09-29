// Xác thực: ID token Google -> phiên (cookie ký HMAC) với vai trò admin/guest.

const enc = new TextEncoder();
const dec = new TextDecoder();

const ADMIN_TTL_S = 7 * 24 * 3600;
const GUEST_TTL_S = 24 * 3600;
const COOKIE = "session";
const GOOGLE_JWKS_URL = "https://www.googleapis.com/oauth2/v3/certs";
const GOOGLE_ISSUERS = ["accounts.google.com", "https://accounts.google.com"];

// ---- base64url ----
function b64uEncode(bytes) {
  let s = "";
  for (const b of new Uint8Array(bytes)) s += String.fromCharCode(b);
  return btoa(s).replace(/\+/g, "-").replace(/\//g, "_").replace(/=+$/, "");
}
function b64uDecode(str) {
  const pad = "=".repeat((4 - (str.length % 4)) % 4);
  const bin = atob(str.replace(/-/g, "+").replace(/_/g, "/") + pad);
  return Uint8Array.from(bin, c => c.charCodeAt(0));
}

export const adminEmails = env =>
  (env.ADMIN_EMAILS || "").split(",").map(s => s.trim().toLowerCase()).filter(Boolean);

// ---- Phiên ----
async function hmacKey(env) {
  if (!env.SESSION_SECRET) throw new Error("SESSION_SECRET chưa được đặt");
  return crypto.subtle.importKey("raw", enc.encode(env.SESSION_SECRET), { name: "HMAC", hash: "SHA-256" }, false, ["sign", "verify"]);
}

export async function makeSession(env, role, email) {
  const ttl = role === "admin" ? ADMIN_TTL_S : GUEST_TTL_S;
  const payload = b64uEncode(enc.encode(JSON.stringify({ role, email: email || null, exp: Math.floor(Date.now() / 1000) + ttl })));
  const sig = await crypto.subtle.sign("HMAC", await hmacKey(env), enc.encode(payload));
  return { value: `${payload}.${b64uEncode(sig)}`, maxAge: ttl };
}

function readCookie(request, name) {
  const h = request.headers.get("Cookie") || "";
  for (const part of h.split(";")) {
    const i = part.indexOf("=");
    if (i > 0 && part.slice(0, i).trim() === name) return part.slice(i + 1).trim();
  }
  return null;
}

// Trả về { role, email } hoặc null. Admin được kiểm tra lại với ADMIN_EMAILS mỗi lần,
// nên gỡ một email khỏi danh sách là mất quyền ngay dù cookie còn hạn.
export async function readSession(request, env) {
  const raw = readCookie(request, COOKIE);
  if (!raw) return null;
  const [payload, sig] = raw.split(".");
  if (!payload || !sig) return null;
  try {
    const ok = await crypto.subtle.verify("HMAC", await hmacKey(env), b64uDecode(sig), enc.encode(payload));
    if (!ok) return null;
    const s = JSON.parse(dec.decode(b64uDecode(payload)));
    if (!s || typeof s.exp !== "number" || s.exp < Date.now() / 1000) return null;
    if (s.role === "admin") {
      if (!s.email || !adminEmails(env).includes(String(s.email).toLowerCase())) return null;
      return { role: "admin", email: s.email };
    }
    if (s.role === "guest") return { role: "guest", email: null };
  } catch { /* cookie hỏng */ }
  return null;
}

export function sessionCookie(request, value, maxAge) {
  const secure = new URL(request.url).protocol === "https:" ? "; Secure" : "";   // http://localhost khi phát triển
  return `${COOKIE}=${value}; Path=/; HttpOnly; SameSite=Lax; Max-Age=${maxAge}${secure}`;
}

// Chống CSRF: yêu cầu ghi phải đến từ chính trang này
export function originOk(request) {
  const origin = request.headers.get("Origin");
  return !!origin && origin === new URL(request.url).origin;
}

// ---- ID token Google ----
let jwksCache = { keys: null, exp: 0 };

async function getJwks(env) {
  if (jwksCache.keys && jwksCache.exp > Date.now()) return jwksCache.keys;
  const res = await fetch(env.GOOGLE_JWKS_URL || GOOGLE_JWKS_URL);
  if (!res.ok) throw new Error("không tải được JWKS");
  const { keys } = await res.json();
  jwksCache = { keys, exp: Date.now() + 3600 * 1000 };
  return keys;
}

// Ném lỗi nếu token không hợp lệ; trả về email (chữ thường) nếu hợp lệ.
export async function verifyGoogleToken(credential, env) {
  if (!env.GOOGLE_CLIENT_ID) throw new Error("GOOGLE_CLIENT_ID chưa được cấu hình");
  if (typeof credential !== "string" || credential.length > 4096) throw new Error("token sai định dạng");
  const parts = credential.split(".");
  if (parts.length !== 3) throw new Error("token sai định dạng");
  const [h, p, s] = parts;

  const header = JSON.parse(dec.decode(b64uDecode(h)));
  if (header.alg !== "RS256") throw new Error("thuật toán không được hỗ trợ");
  let keys = await getJwks(env);
  let jwk = keys.find(k => k.kid === header.kid);
  if (!jwk) {                       // Google có thể vừa xoay khóa: tải lại một lần
    jwksCache.exp = 0;
    keys = await getJwks(env);
    jwk = keys.find(k => k.kid === header.kid);
  }
  if (!jwk) throw new Error("không có khóa phù hợp");

  const key = await crypto.subtle.importKey("jwk", jwk, { name: "RSASSA-PKCS1-v1_5", hash: "SHA-256" }, false, ["verify"]);
  const valid = await crypto.subtle.verify("RSASSA-PKCS1-v1_5", key, b64uDecode(s), enc.encode(`${h}.${p}`));
  if (!valid) throw new Error("chữ ký sai");

  const c = JSON.parse(dec.decode(b64uDecode(p)));
  const now = Date.now() / 1000;
  if (!GOOGLE_ISSUERS.includes(c.iss)) throw new Error("iss sai");
  if (c.aud !== env.GOOGLE_CLIENT_ID) throw new Error("aud sai");
  if (typeof c.exp !== "number" || c.exp < now) throw new Error("token hết hạn");
  if (typeof c.iat === "number" && c.iat > now + 300) throw new Error("iat ở tương lai");
  if (c.email_verified !== true && c.email_verified !== "true") throw new Error("email chưa xác minh");
  if (typeof c.email !== "string") throw new Error("thiếu email");
  return c.email.toLowerCase();
}
