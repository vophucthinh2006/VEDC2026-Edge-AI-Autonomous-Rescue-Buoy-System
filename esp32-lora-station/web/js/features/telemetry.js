// Selected buoy card: name, one big status word, signal and battery as pictures, a short sentence,
// then the commands (stop / run, lights) and the route progress line.
import { state, buoyStatus, STATUS, signalOf, SIGNAL, batteryOf, speedOf } from '../state.js';
import { fmtDist, fmtAgo } from '../util/format.js';
import { distM } from '../util/geo.js';
import { renderHud } from './hud.js';
import { renderLights } from './lights.js';
import { renderHold } from './hold.js';

const $ = id => document.getElementById(id);
const SHOW_MAV_HUD = false;   // Pixhawk HUD stays in the code for the `pixhawk` boat but is not shown

let routeLine = () => null;   // app.js supplies the one-line route progress for a buoy

export function renderDetail() {
  const id = state.selected;
  const b = state.buoys.get(id);
  $('card-empty').hidden = !!b;
  $('card-body').hidden = !b;
  if (!b) return;

  $('c-name').textContent = id;
  const st = STATUS[buoyStatus(id, b)];
  $('c-status').dataset.tone = st.tone;
  $('c-status-ic').className = 'ph-fill ' + st.icon;
  $('c-status-text').textContent = st.text;

  const sig = signalOf(b);
  $('c-sig').dataset.level = sig;
  $('c-sig-text').textContent = SIGNAL[sig].text;

  const batt = batteryOf(b);
  $('c-batt-wrap').hidden = batt === null;
  if (batt !== null) {
    $('c-batt').textContent = batt + '%';
    const bar = $('c-batt-bar');
    bar.style.width = Math.max(4, Math.min(100, batt)) + '%';
    bar.parentElement.dataset.low = batt < 20 ? 'bad' : batt < 40 ? 'warn' : 'ok';
  }

  // "0.8 m/s · cách trạm 120 m"; when the data is old, say how old
  const parts = [];
  if (sig === 'lost') parts.push('Gói cuối ' + fmtAgo(b.lastSeen));
  else {
    const v = speedOf(b);
    if (v !== null) parts.push(v < 0.2 ? 'Đứng yên' : v.toFixed(1) + ' m/s');
  }
  const stn = state.station;
  if (stn && stn.hasPos && b.hasPos) parts.push('cách trạm ' + fmtDist(distM(stn, b)));
  if (b.hasPos && b.fix !== 1 && sig !== 'lost') parts.push('chưa có GPS, đang hiện vị trí cuối');
  if (!b.hasPos) parts.push('chưa có vị trí');
  const line = parts.join(' · ');
  $('c-line').textContent = line ? line[0].toUpperCase() + line.slice(1) : '';
  $('btn-copy').hidden = !b.hasPos;

  renderHold(b, id);
  renderLights(b, id);

  const r = routeLine(id);
  const el = $('c-route');
  el.hidden = !r;
  if (r) { el.dataset.tone = r.tone; el.textContent = r.text; }

  if (SHOW_MAV_HUD) renderHud(b);
}

export function initTelemetry({ toast, routeLine: fn }) {
  routeLine = fn;
  $('btn-copy').addEventListener('click', async () => {
    const b = state.buoys.get(state.selected);
    if (!b || !b.hasPos) { toast('Phao này chưa có toạ độ'); return; }
    const txt = b.lat.toFixed(6) + ', ' + b.lon.toFixed(6);
    try {
      await navigator.clipboard.writeText(txt);
    } catch {
      // Trang chạy trên http nên clipboard API có thể bị chặn: dùng cách dự phòng
      const ta = document.createElement('textarea');
      ta.value = txt; document.body.appendChild(ta); ta.select();
      try { document.execCommand('copy'); } catch { /* bỏ qua */ }
      ta.remove();
    }
    toast('Đã sao chép toạ độ ' + txt);
  });
}
