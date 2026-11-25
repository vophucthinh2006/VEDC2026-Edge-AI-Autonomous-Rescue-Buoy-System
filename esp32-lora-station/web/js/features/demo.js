// The judges' demo in a room: no GPS there, so the buoys are drawn at assumed positions, fix or not, said so on the
// map, and a board in large type tells what is happening. On with the admin's button (kept by the server, off by
// itself after 3 h; the Pis ask /api/demo) or, for this page only, with ?demo. The positions are those of
// Rasp_Pi/config/judge_demo.yaml (PHAO-01) and Phao_2/pi4/config.json (PHAO-02): keep them the same.
import { state, on, RISKS } from '../state.js';

const ASSUMED = { 'PHAO-01': [10.879264, 106.806386], 'PHAO-02': [10.879264, 106.806414] };
const BY_URL = new URLSearchParams(location.search).has('demo');
let on_ = BY_URL;
export const isDemo = () => on_;

// The buoy's frame with the assumed position, even when a real fix comes in: it must not move the demo.
export function assumePosition(m) {
  const at = on_ && ASSUMED[m.id];
  return at ? { ...m, lat: at[0], lon: at[1], assumed: true } : m;
}

let board, lines;
const said = [];
function say(text) {
  if (!on_ || said[0] === text) return;
  said.unshift(text);
  said.length = Math.min(said.length, 3);
  lines.replaceChildren(...said.map(t => Object.assign(document.createElement('li'), { textContent: t })));
}

const ownerOf = v => v.owner ?? (v.status ? null : v.buoy);
const names = v => (v.seen_by || []).length ? v.seen_by : [v.buoy];

// v: the victim now, before: {status, owner, seen} earlier, undefined the first time
export function tellVictim(v, before) {
  const risk = RISKS[v.risk].name.toLowerCase(), owner = ownerOf(v);
  if (!before) return say(`${v.buoy} thấy người (mức ${risk}). Trạm giao ${owner || v.buoy} nhận, dừng lại cứu.`);
  if (v.status === 'rescued' && before.status !== 'rescued') return say(`Đã cứu xong người do ${before.owner || v.buoy} nhận.`);
  const seen = names(v);
  if (seen.length > before.seen) say(`${seen.at(-1)} cũng thấy người đó. Trạm báo đã có ${owner}: ${seen.at(-1)} đi tiếp, không dừng.`);
  else if (v.owner !== before.owner && owner) say(`${owner} nhận người đó.`);
  else if (v.risk !== before.risk) say(`Mức nguy hiểm đổi thành ${risk}: ${owner || v.buoy} ưu tiên.`);
}

let onSet = () => {}, onChange = () => {};

// The phone's bottom sheet, the same test as the stylesheets (console.css)
const PHONE = window.matchMedia('(max-width:600px), (max-width:820px) and (orientation:portrait)');

// A phone on its side: no room between the gauges and the buoy control
const SHORT = window.matchMedia('(min-width:601px) and (max-height:520px) and (orientation:landscape)');

// Wide: over the map. Phone: in the sheet (upright) or the drawer (on its side), where it can lie over nothing.
function place() {
  if (PHONE.matches) document.getElementById('rail').before(board);
  else if (SHORT.matches) document.querySelector('#drawer .drawer-head').after(board);
  else document.getElementById('map-wrap').append(board);
}

function show() {
  board.hidden = !on_;
  document.getElementById('map-wrap').dataset.demo = String(on_);
  const btn = document.getElementById('btn-demo');
  btn.hidden = state.role !== 'admin';
  btn.setAttribute('aria-pressed', String(on_));
  if (on_ && !said.length) say('Đang chờ phao phát hiện người trong 10 m.');
}

// {type: 'demo', on} from the server, and after the role is known
export function setDemo(on) {
  const was = on_;
  on_ = BY_URL || on === true;
  show();
  if (was === on_) return;
  if (!on_) { said.length = 0; lines.replaceChildren(); }
  onChange(on_, ASSUMED);
}

// onSet(on): tell the server; onChange(on, positions): move the buoys on the map
export function initDemo(c) {
  ({ onSet, onChange } = c);
  board = document.createElement('section');
  board.id = 'demo-board';
  board.setAttribute('aria-live', 'polite');
  board.innerHTML = '<div class="demo-head"><h2>Đang xảy ra</h2><span class="demo-chip"><i class="ph ph-warning" aria-hidden="true"></i>GPS giả định</span>'
    + '<span class="demo-note">Vị trí phao do cấu hình. Nhận diện, LoRa và phân xử là thật.</span></div><ol></ol>';
  lines = board.querySelector('ol');
  place();
  for (const m of [PHONE, SHORT]) m.addEventListener('change', place);
  new ResizeObserver(() => document.getElementById('map-wrap').style.setProperty('--demo-h', `${board.offsetHeight}px`)).observe(board);
  document.getElementById('btn-demo').addEventListener('click', () => {
    const want = !on_;
    if (want && !confirm('Bật GPS giả định cho demo trong phòng? Phao báo người ở vị trí giả định, không phải vị trí thật. Tự tắt sau 3 giờ.')) return;
    onSet(want);
  });
  show();
  const held = new Map();
  on('buoy', ({ id, b }) => {
    if (b.hold && !held.get(id)) say(`${id} đang dừng theo lệnh từ dashboard.`);
    held.set(id, b.hold);
  });
}
