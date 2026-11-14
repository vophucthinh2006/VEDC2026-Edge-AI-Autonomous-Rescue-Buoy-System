// Films the admin console (/admin) during a simulated run, for slides: a headless Edge driven over the
// DevTools protocol signs in with the test sign-in (npm run dev:idp), opens a tab, and takes a screenshot
// every --every seconds; ffmpeg then makes an mp4 at --fps (every 0.5 s at 8 fps: four times faster).
//
//   node sim/tools/record_admin.mjs --out D:\tmp\admin --duration 660 [--tab fleet] [--mp4 admin.mp4]
//        [--url http://127.0.0.1:8787/admin] [--every 0.5] [--fps 8] [--size 1600x900] [--zoom-in 3] [--pan -300,0]
// Needs the Worker (wrangler dev) and the test sign-in running, Edge, and ffmpeg on the PATH (Windows).
import { spawn, spawnSync } from "node:child_process";
import { mkdirSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";

const arg = (name, def) => { const i = process.argv.indexOf("--" + name); return i > 0 ? process.argv[i + 1] : def; };
const url = arg("url", "http://127.0.0.1:8787/admin");
const out = arg("out");
const duration = Number(arg("duration", "60")), every = Number(arg("every", "0.5")), fps = Number(arg("fps", "8"));
const [w, h] = arg("size", "1600x900").split("x").map(Number);
const tab = arg("tab", "fleet"), mp4 = arg("mp4");
if (!out) { console.error("--out is needed"); process.exit(2); }
mkdirSync(out, { recursive: true });
const sleep = ms => new Promise(r => setTimeout(r, ms));

const port = 9333;
const edge = spawn(arg("edge", "C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe"),
  ["--headless=new", `--remote-debugging-port=${port}`, `--user-data-dir=${mkdtempSync(join(tmpdir(), "vedc-edge-"))}`,
   `--window-size=${w},${h}`, "--hide-scrollbars", "about:blank"], { stdio: "ignore" });
let page;
for (let i = 0; i < 50 && !page; i++) {
  await sleep(200);
  page = await fetch(`http://127.0.0.1:${port}/json/list`).then(r => r.json()).then(l => l.find(t => t.type === "page")).catch(() => null);
}
if (!page) { console.error("Edge did not come up"); edge.kill(); process.exit(1); }

const ws = new WebSocket(page.webSocketDebuggerUrl);
await new Promise((ok, bad) => { ws.onopen = ok; ws.onerror = bad; });
let seq = 0;
const waiting = new Map();
ws.onmessage = e => { const m = JSON.parse(e.data); if (m.id && waiting.has(m.id)) { waiting.get(m.id)(m); waiting.delete(m.id); } };
const cdp = (method, params = {}) => new Promise(ok => { const id = ++seq; waiting.set(id, ok); ws.send(JSON.stringify({ id, method, params })); });
const js = async expr => (await cdp("Runtime.evaluate", { expression: expr, awaitPromise: true, returnByValue: true })).result?.result?.value;

await cdp("Emulation.setDeviceMetricsOverride", { width: w, height: h, deviceScaleFactor: 1, mobile: false });
await cdp("Page.navigate", { url });
// Signed in once the test sign-in button is there and pressed, and the login screen gone
for (let i = 0; i < 60; i++) {
  await sleep(500);
  if (await js("!!document.getElementById('btn-dev')")) { await js("document.getElementById('btn-dev').click()"); break; }
}
for (let i = 0; i < 40 && !(await js("document.getElementById('login').hidden")); i++) await sleep(500);
if (!(await js("document.getElementById('login').hidden"))) { console.error("not signed in (ADMIN_EMAILS has admin@example.com?)"); edge.kill(); process.exit(1); }
if (tab) await js(`document.querySelector('[data-m="${tab}"]')?.click()`);
await sleep(3000);
// The map: centred on PHAO-01 (its row in the fleet list), then --zoom-in clicks of +, then dragged by --pan dx,dy px
await js(`[...document.querySelectorAll('#panel-fleet *')].find(e => e.children.length === 0 && e.textContent.trim() === 'PHAO-01')?.click()`);
for (let i = 0; i < Number(arg("zoom-in", "0")); i++) { await js("document.querySelector('.leaflet-control-zoom-in').click()"); await sleep(400); }
const [dx, dy] = arg("pan", "0,0").split(",").map(Number);
if (dx || dy) {
  const mouse = (type, x, y) => cdp("Input.dispatchMouseEvent", { type, x, y, button: "left", buttons: type === "mouseReleased" ? 0 : 1, clickCount: 1 });
  const [x0, y0] = [w / 3, h / 2];
  await mouse("mousePressed", x0, y0);
  for (let k = 1; k <= 10; k++) await mouse("mouseMoved", x0 + dx * k / 10, y0 + dy * k / 10);
  await mouse("mouseReleased", x0 + dx, y0 + dy);
}
await sleep(3000);   // the map's tiles

const n = Math.round(duration / every);
const t0 = Date.now();
for (let i = 0; i < n; i++) {
  const shot = await cdp("Page.captureScreenshot", { format: "jpeg", quality: 85 });
  writeFileSync(join(out, `f${String(i).padStart(5, "0")}.jpg`), Buffer.from(shot.result.data, "base64"));
  await sleep(Math.max(0, t0 + (i + 1) * every * 1000 - Date.now()));
}
ws.close();
edge.kill();
console.log(`${n} frames in ${out}`);
if (mp4) {
  const r = spawnSync("ffmpeg", ["-y", "-loglevel", "error", "-framerate", String(fps), "-i", join(out, "f%05d.jpg"),
    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", mp4], { stdio: "inherit" });
  if (r.status !== 0) process.exit(1);
  console.log(mp4);
}
