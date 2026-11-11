// The console's instrument row (docs/admin-console-ui.md): what the buoy picked is doing, read at a glance as
// numbers and pictures instead of sentences. Only what the boats and the Worker really send:
//   - rain over the next 24 hours at the station (Open-Meteo through /api/weather, features/weather.js)
//   - roll and pitch (IMU), the bow now (yaw) against the heading the Pi asks for (target_yaw, -1 = none)
//   - battery % (beacon) with volts and amps (INA226, diagnostics frame), LoRa bars and the distance to the station
// Also the status pill in the top bar, the station's satellites, and the picked-buoy control at the bottom.
import { state, buoyStatus, STATUS, signalOf, SIGNAL, batteryOf, speedOf, attitudeAgeSec, ATTITUDE_STALE_SEC,
  isStale } from '../state.js';
import { distM, compassWord } from '../util/geo.js';
import { fmtAgo } from '../util/format.js';
import { forecast } from './weather.js';

const $ = id => document.getElementById(id);
const TILT_WARN_DEG = 20, TILT_BAD_DEG = 30;     // features/health.js
const HEAD_WARN_DEG = 10, HEAD_BAD_DEG = 30;     // features/health.js
const BATT_BAD = 20, BATT_WARN = 40;             // features/telemetry.js
const RAIN_HOURS = 24;
const TONE = { ok: 'ok', warn: 'warn', bad: 'bad', lost: 'lost', idle: 'idle' };

const num = (v, d = 1) => v.toLocaleString('vi-VN', { minimumFractionDigits: d, maximumFractionDigits: d });
const signed = v => `${v > 0 ? '+' : v < 0 ? '−' : ''}${num(Math.abs(v))}°`;
const signedDeg = d => ((d + 540) % 360) - 180;
const tiltTone = v => (Math.abs(v) >= TILT_BAD_DEG ? 'bad' : Math.abs(v) >= TILT_WARN_DEG ? 'warn' : 'ok');

// Set text only when it changed, so the browser (and a screen reader) is not told about every tick
function text(el, t) { if (el && el.textContent !== t) el.textContent = t; }
function tone(el, t) { if (el && el.dataset.tone !== t) el.dataset.tone = t; }

// A needle turns the short way round: 350° to 10° is +20°, not -340°
function turn(el, deg) {
  const prev = Number.isFinite(el._a) ? el._a : deg;
  el._a = prev + signedDeg(deg - prev);
  el.style.transform = `rotate(${el._a.toFixed(1)}deg)`;
}
function tilt(el, deg) { el.style.transform = `rotate(${deg.toFixed(1)}deg)`; }

// A word that changed plays a short swap animation (css/gauges.css .swap)
function swap(el, t) {
  if (!el || el.textContent === t) return;
  el.textContent = t;
  el.classList.remove('swap'); void el.offsetWidth; el.classList.add('swap');
}

let ctx = { onHome: () => {}, isHome: () => false, onStep: () => {} };

// ---- rain ------------------------------------------------------------------------------------
function renderRain() {
  const card = $('g-rain');
  const hours = forecast()?.hours?.slice(0, RAIN_HOURS) || [];
  card.dataset.empty = String(!hours.length);
  if (!hours.length) { text($('gr-mm'), '--'); text($('gr-prob').lastElementChild, '--'); return; }
  const total = hours.reduce((s, h) => s + (h.rain_mm || 0), 0);
  const prob = Math.max(...hours.map(h => h.prob ?? 0));
  text($('gr-mm'), total < 0.1 ? '0' : num(total, total < 10 ? 1 : 0));
  text($('gr-prob').lastElementChild, `${prob}%`);
  tone(card, total >= 50 || prob >= 80 ? 'warn' : 'ok');
  card.dataset.wet = String(total >= 0.1);   // the drops fall only when rain is forecast
  card.title = total < 0.1 ? 'Không mưa 24 giờ tới' : `Mưa 24 giờ tới: ${num(total)} mm, khả năng ${prob}%`;
  const bars = $('gr-bars');
  if (bars.children.length !== hours.length) bars.replaceChildren(...hours.map(() => document.createElement('i')));
  const peak = Math.max(2, ...hours.map(h => h.rain_mm || 0));
  hours.forEach((h, i) => {
    const bar = bars.children[i];
    bar.style.transform = `scaleY(${Math.max(0.08, (h.rain_mm || 0) / peak).toFixed(3)})`;
    bar.dataset.wet = String((h.rain_mm || 0) > 0);
  });
}

// ---- the buoy ----------------------------------------------------------------------------------
function renderAttitude(b) {
  const has = b.hasAttitude === true;
  const old = has && attitudeAgeSec(b) > ATTITUDE_STALE_SEC;
  for (const id of ['g-roll', 'g-pitch', 'g-head']) {
    const c = $(id);
    c.dataset.empty = String(!has);
    c.dataset.stale = String(old);
  }
  if (!has) {
    for (const id of ['gl-v', 'gp-v', 'gh-word', 'gh-pi-word']) text($(id), '--');
    for (const id of ['gh-deg', 'gh-pi-deg']) text($(id), '');
    text($('gh-off').lastElementChild, 'Chưa có dữ liệu IMU');
    for (const id of ['g-roll', 'g-pitch', 'g-head']) tone($(id), 'idle');
    $('g-head').dataset.want = 'false';
    return;
  }
  text($('gl-v'), signed(b.roll)); tone($('g-roll'), tiltTone(b.roll)); tilt($('gl-boat'), b.roll);
  text($('gp-v'), signed(b.pitch)); tone($('g-pitch'), tiltTone(b.pitch)); tilt($('gp-boat'), -b.pitch);

  const yaw = ((b.yaw % 360) + 360) % 360;
  swap($('gh-word'), compassWord(yaw));
  text($('gh-deg'), `${Math.round(yaw)}°`);
  turn($('gh-yaw'), yaw);
  const head = $('g-head');
  const want = b.targetYaw;
  head.dataset.want = String(want !== null);
  const off = $('gh-off');
  if (want === null) {
    swap($('gh-pi-word'), 'Chưa ra lệnh');
    text($('gh-pi-deg'), '');
    text(off.lastElementChild, 'Pi chưa yêu cầu hướng');
    off.dataset.dir = '';
    tone(head, 'idle');
    $('gh-arc').setAttribute('d', '');
  } else {
    const w = ((want % 360) + 360) % 360;
    swap($('gh-pi-word'), compassWord(w));
    text($('gh-pi-deg'), `${Math.round(w)}°`);
    turn($('gh-want'), w);
    const d = signedDeg(w - yaw), a = Math.abs(d);
    text(off.lastElementChild, a < 2 ? 'Đúng hướng' : `Lệch ${Math.round(a)}° ${d > 0 ? 'phải' : 'trái'}`);
    off.dataset.dir = a < 2 ? '' : d > 0 ? 'right' : 'left';
    tone(head, a <= HEAD_WARN_DEG ? 'ok' : a <= HEAD_BAD_DEG ? 'warn' : 'bad');
    // The arc on the rim, from the bow to the Pi's heading
    const pt = deg => { const r = (deg - 90) * Math.PI / 180; return `${(52 + 48 * Math.cos(r)).toFixed(1)} ${(52 + 48 * Math.sin(r)).toFixed(1)}`; };
    $('gh-arc').setAttribute('d', a < 2 ? '' : `M ${pt(yaw)} A 48 48 0 0 ${d > 0 ? 1 : 0} ${pt(w)}`);
  }
  $('gh-svg').setAttribute('aria-label', `La bàn: mũi tàu ${compassWord(yaw)} ${Math.round(yaw)}°, `
    + (want === null ? 'Pi chưa yêu cầu hướng' : `Pi yêu cầu ${compassWord(want)} ${Math.round(want)}°`));
}

function renderBattery(id, b) {
  const card = $('g-batt');
  const p = batteryOf(b);
  card.dataset.empty = String(p === null);
  text($('gb-v'), p === null ? '--' : String(p));
  tone(card, p === null ? 'idle' : p < BATT_BAD ? 'bad' : p < BATT_WARN ? 'warn' : 'ok');
  $('gb-ring').style.strokeDashoffset = String(100 - (p ?? 0));
  const d = state.diag.get(id);
  text($('gb-pack'), d && Number.isFinite(d.volts) ? `${num(d.volts, 2)} V · ${num(d.amps, 2)} A` : '');
}

function renderLink(b) {
  const card = $('g-link');
  const sig = signalOf(b);
  $('gk-bars').dataset.level = sig;
  tone(card, sig === 'lost' ? 'bad' : sig === 'weak' ? 'warn' : 'ok');
  const s = state.station;
  const km = s && s.hasPos && b.hasPos ? distM(s, b) / 1000 : null;
  text($('gk-km'), km === null ? '--' : num(km, km < 10 ? 1 : 0));
  card.title = `Tín hiệu ${SIGNAL[sig].text.toLowerCase()}${km === null ? '' : `, cách trạm bờ ${num(km)} km`}`;
}

function renderPill(id, b, st) {
  const pill = $('pill');
  pill.hidden = false;
  tone(pill, TONE[st.tone] || 'idle');
  text($('pill-name'), id);
  const d = state.diag.get(id);
  let extra = '';
  if (isStale(b)) extra = `gói cuối ${fmtAgo(b.lastSeen)}`;
  else if (d && d.wp_total) extra = `điểm ${d.wp}/${d.wp_total}`;
  swap($('pill-text'), extra ? `${st.text} · ${extra}` : st.text);
}

function renderCtl(id, b, st) {
  $('ctl').hidden = false;
  swap($('ctl-name'), id);
  const s = $('ctl-st');
  tone(s, TONE[st.tone] || 'idle');
  text(s.lastElementChild, st.text);
  const v = isStale(b) ? null : speedOf(b);
  text($('ctl-sog'), v === null ? '--' : num(v));
  const many = state.buoys.size > 1;
  $('ctl-prev').disabled = !many; $('ctl-next').disabled = !many;
  const home = $('ctl-home');
  const going = ctx.isHome(id);
  home.setAttribute('aria-pressed', String(going));
  const label = going ? `Bỏ lệnh về bờ cho ${id}` : `Gọi ${id} về điểm đỗ`;
  home.setAttribute('aria-label', label); home.title = label;
  home.disabled = state.role !== 'admin';
}

function renderDock(b) {
  const btn = $('dock-lights');
  btn.hidden = !Number.isInteger(b.lights);
  if (btn.hidden) return;
  const on = (b.lights & 1) + ((b.lights >> 1) & 1);
  text(btn.lastElementChild, `${on}/2`);
  btn.dataset.on = String(on > 0);
}

function renderStation() {
  const chip = $('st-sats');
  const s = state.station;
  chip.hidden = !s || !Number.isFinite(s.sats);
  if (!chip.hidden) {
    text(chip.lastElementChild, String(s.sats));
    chip.dataset.fix = String(s.fix === 1);
    chip.title = `Trạm bờ: ${s.sats} vệ tinh${s.fix === 1 ? '' : ', chưa có GPS'}`;
  }
}

export function renderGauges() {
  renderStation();
  renderRain();
  const id = state.selected;
  const b = id && state.buoys.get(id);
  $('gauges').hidden = !b;
  if (!b) { $('pill').hidden = true; $('ctl').hidden = true; return; }
  const st = STATUS[buoyStatus(id, b)];
  $('gauges').dataset.lost = String(isStale(b));
  renderAttitude(b);
  renderBattery(id, b);
  renderLink(b);
  renderPill(id, b, st);
  renderCtl(id, b, st);
  renderDock(b);
}

// onStep(+1|-1): the next or previous buoy; onHome(id, ret): call it home or cancel; isHome(id)
export function initGauges(c) {
  ctx = { ...ctx, ...c };
  $('ctl-prev').addEventListener('click', () => ctx.onStep(-1));
  $('ctl-next').addEventListener('click', () => ctx.onStep(1));
  $('ctl-home').addEventListener('click', () => {
    const id = state.selected;
    if (id && state.role === 'admin') ctx.onHome(id, !ctx.isHome(id));
  });
  // A rail badge that goes up bumps once, so a new person or event is noticed without reading
  for (const badge of document.querySelectorAll('#rail .badge')) {
    let last = Number(badge.textContent) || 0;
    new MutationObserver(() => {
      const n = Number(badge.textContent) || 0;
      if (n > last) { badge.classList.remove('bump'); void badge.offsetWidth; badge.classList.add('bump'); }
      last = n;
    }).observe(badge, { childList: true, characterData: true, subtree: true });
  }
  window.addEventListener('weatherupdate', renderRain);
  renderGauges();
}
