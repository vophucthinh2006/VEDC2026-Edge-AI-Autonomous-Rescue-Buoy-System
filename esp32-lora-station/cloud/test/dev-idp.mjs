// IdP giả để thử đăng nhập admin trên trình duyệt khi chạy `wrangler dev`, không cần Google thật.
//   - :8799/jwks.json  khóa công khai (trùng GOOGLE_JWKS_URL trong .dev.vars)
//   - :8798/token?email=admin@example.com  trả một ID token đã ký (có CORS)
// Chạy:  node test/dev-idp.mjs
import http from "node:http";
import crypto from "node:crypto";

const CLIENT_ID = process.env.GOOGLE_CLIENT_ID || "test-client-id";
const KID = "test-key-1";
const { publicKey, privateKey } = crypto.generateKeyPairSync("rsa", { modulusLength: 2048 });
const jwk = { ...publicKey.export({ format: "jwk" }), kid: KID, alg: "RS256", use: "sig" };
const b64u = b => Buffer.from(b).toString("base64url");

http.createServer((req, res) => {
  res.setHeader("content-type", "application/json");
  res.end(JSON.stringify({ keys: [jwk] }));
}).listen(8799, "127.0.0.1");

http.createServer((req, res) => {
  const email = new URL(req.url, "http://x").searchParams.get("email") || "admin@example.com";
  const now = Math.floor(Date.now() / 1000);
  const head = b64u(JSON.stringify({ alg: "RS256", kid: KID, typ: "JWT" }));
  const body = b64u(JSON.stringify({ iss: "https://accounts.google.com", aud: CLIENT_ID, email, email_verified: true, iat: now, exp: now + 3600 }));
  const sig = crypto.sign("RSA-SHA256", Buffer.from(`${head}.${body}`), privateKey);
  res.setHeader("Access-Control-Allow-Origin", "*");
  res.setHeader("content-type", "text/plain");
  res.end(`${head}.${body}.${b64u(sig)}`);
}).listen(8798, "127.0.0.1");

console.log("dev IdP: jwks :8799, token :8798");
