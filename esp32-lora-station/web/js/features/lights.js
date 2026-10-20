// Hai đèn trên phao (relay PC3 = đèn 1, PC2 = đèn 2). Chỉ admin bật/tắt từ dashboard (dùng ban đêm).
// want = lệnh admin (máy chủ giữ, trạm phát xuống phao), have = trạng thái thật phao báo trong beacon.
// Công tắc hiện "đang gửi…" tới khi have khớp want.
import { state } from '../state.js';
import { log } from './eventlog.js';

const $ = id => document.getElementById(id);
const LIGHTS = [{ bit: 0, name: 'Đèn 1 (PC3)' }, { bit: 1, name: 'Đèn 2 (PC2)' }];
let onSet = () => {};

const bitOf = (mask, bit) => (Number.isInteger(mask) ? (mask >> bit) & 1 : null);
const describe = mask => LIGHTS.map(l => `${l.name} ${bitOf(mask, l.bit) ? 'bật' : 'tắt'}`).join(', ');

export function renderLights(b, id) {
  const box = $('d-lights');
  box.hidden = !b || b.link === 'mavlink';   // thuyền Pixhawk không có hai relay này
  if (box.hidden) return;
  const want = state.lights.get(id) ?? null;
  const admin = state.role === 'admin';
  for (const l of LIGHTS) {
    const w = bitOf(want, l.bit), h = bitOf(b.lights, l.bit);
    const pending = w !== null && w !== h;
    const input = $('d-light' + (l.bit + 1));
    input.checked = (pending ? w : h) === 1;
    input.disabled = !admin;
    const st = $('d-light' + (l.bit + 1) + '-state');
    st.textContent = pending ? 'đang gửi…' : h === null ? 'chưa rõ' : h ? 'Bật' : 'Tắt';
    st.className = 'light-state ' + (pending ? 'pending' : h ? 'on' : 'off');
  }
}

// Bản tin {type: "lights", id, want, have} từ máy chủ
export function onLightsMessage(m) {
  const id = String(m.id);
  const prevWant = state.lights.get(id) ?? null;
  if (Number.isInteger(m.want)) state.lights.set(id, m.want);
  const b = state.buoys.get(id);
  if (b && Number.isInteger(m.have)) b.lights = m.have;
  // Lệnh của admin khác (lệnh của chính tab này đã ghi lúc bấm)
  if (Number.isInteger(m.want) && prevWant !== null && m.want !== prevWant) log('info', `${id}: ${m.by || 'admin'} gửi lệnh đèn: ${describe(m.want)}`);
  noteLights(id);
  if (id === state.selected && b) renderLights(b, id);
}

// Trạng thái thật có thể đến qua beacon hoặc bản tin lights, thứ tự nào cũng được:
// ghi nhật ký khi nó đổi sang đúng lệnh. Lần đầu thấy (mở trang) thì chỉ ghi nhớ.
const seenHave = new Map();   // id -> trạng thái thật lần trước
export function noteLights(id) {
  const b = state.buoys.get(id);
  if (!b) return;
  const have = Number.isInteger(b.lights) ? b.lights : null;
  const known = seenHave.has(id), before = seenHave.get(id);
  seenHave.set(id, have);
  if (known && have !== null && have !== before && have === state.lights.get(id)) log('ok', `${id} đã chuyển đèn: ${describe(have)}`);
}

// Gửi lệnh không thành công: trả lệnh về như trước khi bấm
export function revertLights(id, want) {
  if (want === null) state.lights.delete(id); else state.lights.set(id, want);
  const b = state.buoys.get(id);
  if (id === state.selected && b) renderLights(b, id);
}

export function initLights({ onSet: fn }) {
  onSet = fn;
  for (const l of LIGHTS) {
    $('d-light' + (l.bit + 1)).addEventListener('change', () => {
      const id = state.selected;
      const b = state.buoys.get(id);
      if (state.role !== 'admin' || !b) return;
      const prev = state.lights.get(id) ?? null;
      const mask = (prev ?? (Number.isInteger(b.lights) ? b.lights : 0)) ^ (1 << l.bit);
      state.lights.set(id, mask);   // hiện "đang gửi…" ngay, không chờ máy chủ trả lời
      renderLights(b, id);
      log('info', `${id}: gửi lệnh đèn: ${describe(mask)}`);
      onSet(id, mask, prev);
    });
  }
}
