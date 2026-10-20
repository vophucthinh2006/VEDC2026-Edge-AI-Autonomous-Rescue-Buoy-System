// Nút DỪNG PHAO / CHẠY TIẾP (bản STM32). Admin bấm, máy chủ giữ lệnh, trạm phát xuống phao trong ACK;
// F407 khóa motor ngay khi nhận, kể cả khi Pi vẫn ra lệnh chạy. Lộ trình giữ nguyên: "Chạy tiếp" đi
// tiếp waypoint đang dở. want = lệnh admin, have = phao báo trong beacon (H=1 đang dừng).
// Lệnh tới phao sau 5 đến 10 s (trạm hỏi máy chủ 5 s một lần, phao phát beacon 5 s một lần).
import { state } from '../state.js';
import { log } from './timeline.js';

const $ = id => document.getElementById(id);
let onSet = () => {};

export function renderHold(b, id) {
  const box = $('d-hold');
  box.hidden = !b || b.link === 'mavlink';   // thuyền Pixhawk dừng bằng chế độ HOLD của autopilot
  if (box.hidden) return;
  const want = state.hold.has(id) ? state.hold.get(id) : null;
  const have = typeof b.hold === 'boolean' ? b.hold : null;
  const pending = want !== null && want !== have;
  const stopped = pending ? want : have === true;
  const btn = $('d-hold-btn');
  btn.textContent = stopped ? 'CHẠY TIẾP' : 'DỪNG PHAO';
  btn.className = 'btn hold-btn ' + (stopped ? 'resume' : 'stop');
  btn.disabled = state.role !== 'admin';
  const st = $('d-hold-state');
  st.textContent = pending ? (want ? 'Đang gửi lệnh dừng, phao nhận sau 5 đến 10 giây' : 'Đang gửi lệnh chạy, phao nhận sau 5 đến 10 giây')
    : have ? 'Motor đã khoá. Bấm Chạy tiếp để đi tiếp lộ trình.' : '';
  st.className = 'hold-state ' + (pending ? 'pending' : have ? 'stopped' : 'running');
}

// Bản tin {type: "hold", id, want, have} từ máy chủ
export function onHoldMessage(m) {
  const id = String(m.id);
  const prev = state.hold.has(id) ? state.hold.get(id) : null;
  if (typeof m.want === 'boolean') state.hold.set(id, m.want);
  const b = state.buoys.get(id);
  if (b && typeof m.have === 'boolean') b.hold = m.have;
  // Another admin's command (this tab logged its own on click). No email: guests read the timeline too
  if (typeof m.want === 'boolean' && prev !== null && m.want !== prev) log(m.want ? 'stop' : 'run', `Admin ra lệnh ${m.want ? 'dừng' : 'chạy tiếp'} ${id}`);
  noteHold(id);
  if (id === state.selected && b) renderHold(b, id);
}

// Phao báo đã dừng / đã chạy lại đúng lệnh: ghi nhật ký một lần. Lần đầu thấy (mở trang) chỉ ghi nhớ.
const seen = new Map();
export function noteHold(id) {
  const b = state.buoys.get(id);
  if (!b) return;
  const have = typeof b.hold === 'boolean' ? b.hold : null;
  const known = seen.has(id), before = seen.get(id);
  seen.set(id, have);
  if (known && have !== null && have !== before && have === state.hold.get(id)) {
    log(have ? 'stop' : 'run', have ? `${id} đã dừng, motor khoá` : `${id} chạy tiếp`);
  }
}

export function revertHold(id, want) {
  if (want === null) state.hold.delete(id); else state.hold.set(id, want);
  const b = state.buoys.get(id);
  if (id === state.selected && b) renderHold(b, id);
}

export function initHold({ onSet: fn }) {
  onSet = fn;
  $('d-hold-btn').addEventListener('click', () => {
    const id = state.selected;
    const b = state.buoys.get(id);
    if (state.role !== 'admin' || !b) return;
    const prev = state.hold.has(id) ? state.hold.get(id) : null;
    const stopped = prev !== null ? prev : b.hold === true;
    const want = !stopped;
    if (want && !confirm(`Dừng ${id}? Motor khoá tới khi bấm "Chạy tiếp" (lệnh tới phao sau 5 đến 10 giây).`)) return;
    state.hold.set(id, want);   // hiện "đang gửi…" ngay
    renderHold(b, id);
    log(want ? 'stop' : 'run', `Gửi lệnh ${want ? 'dừng' : 'chạy tiếp'} tới ${id}`);
    onSet(id, want, prev);
  });
}
