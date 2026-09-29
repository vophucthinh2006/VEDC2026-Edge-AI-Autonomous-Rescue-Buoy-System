// Thanh trạng thái trên cùng: máy chủ, số phao theo trạng thái, gói cuối, đồng hồ UTC và giờ địa phương.
import { state, statusOf, stationStatus } from '../state.js';
import { fmtAgeSec, hms, hmsUTC, NONE } from '../util/format.js';

const $ = id => document.getElementById(id);

function render() {
  const now = new Date();
  $('sb-utc').textContent = hmsUTC(now);
  $('sb-local').textContent = hms(now);

  const conn = $('sb-server');
  conn.dataset.state = state.connected ? 'up' : 'down';
  $('sb-server-val').textContent = state.connected ? 'Trực tuyến' : 'Mất kết nối';

  const c = { online: 0, stale: 0, nofix: 0 };
  for (const b of state.buoys.values()) c[statusOf(b)]++;
  $('sb-online').textContent = c.online;
  $('sb-stale').textContent = c.stale;
  $('sb-nofix').textContent = c.nofix;

  const ss = stationStatus();
  const sEl = $('sb-station');
  sEl.dataset.state = ss;
  $('sb-station-val').textContent = ss === 'none' ? NONE
    : ss === 'online' ? 'GPS ' + (state.station.sats != null ? state.station.sats + ' vệ tinh' : 'tốt')
    : ss === 'nofix' ? 'Chưa có GPS' : 'Mất tín hiệu';

  const rx = $('sb-rx-val');
  if (state.lastRx) {
    const s = (Date.now() - state.lastRx) / 1000;
    rx.textContent = fmtAgeSec(s);
    $('sb-rx').dataset.state = s > 60 ? 'warn' : 'ok';
  } else {
    rx.textContent = NONE;
    $('sb-rx').dataset.state = 'idle';
  }
}

export function initStatusbar() {
  render();
  setInterval(render, 1000);
}
export const refreshStatusbar = render;
