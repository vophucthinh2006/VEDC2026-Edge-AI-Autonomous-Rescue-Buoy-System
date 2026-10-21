// Kiểm thử devSignInUrl: chỉ Worker chạy cục bộ với IdP giả (test/dev-idp.mjs) mới bật đăng nhập thử.
// Chạy:  node --test test/devsignin.test.mjs
import assert from "node:assert/strict";
import test from "node:test";
import { devSignInUrl } from "../src/auth.js";

test("wrangler dev with the test sign-in on this machine", () => {
  assert.equal(devSignInUrl({ GOOGLE_JWKS_URL: "http://127.0.0.1:8799/jwks.json" }), "http://127.0.0.1:8798/token");
  assert.equal(devSignInUrl({ GOOGLE_JWKS_URL: "http://localhost:8799/jwks.json" }), "http://localhost:8798/token");
});

test("never on the deployed Worker", () => {
  assert.equal(devSignInUrl({}), "");                                                       // Google's own keys
  assert.equal(devSignInUrl({ GOOGLE_JWKS_URL: "https://www.googleapis.com/oauth2/v3/certs" }), "");
  assert.equal(devSignInUrl({ GOOGLE_JWKS_URL: "https://127.0.0.1.evil.example/jwks.json" }), "");
  assert.equal(devSignInUrl({ GOOGLE_JWKS_URL: "not a url" }), "");
});
