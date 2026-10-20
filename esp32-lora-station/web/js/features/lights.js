// Hai đèn trên phao (relay PC3 = đèn 1, PC2 = đèn 2). Chỉ admin bật/tắt từ dashboard (dùng ban đêm).
// want = lệnh admin (máy chủ giữ, trạm phát xuống phao), have = trạng thái thật phao báo trong beacon.
// Công tắc hiện "đang gửi" tới khi have khớp want. Đèn không ghi vào dòng sự kiện.
import { state } from '../state.js';

const $ = id => document.getElementById(id);
const LIGHTS = [{ bit: 0, name: 'Đèn 1' }, { bit: 1, name: 'Đèn 2' }];   // relay PC3, PC2
let onSet = () => {};

const bitOf = (mask, bit) => (Number.isInteger(mask) ? (mask >> bit) & 1 : null);

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
    st.textContent = pending ? 'đang gửi' : h === null ? '' : h ? 'Bật' : 'Tắt';
    st.className = 'light-state ' + (pending ? 'pending' : h ? 'on' : 'off');
  }
}

// Bản tin {type: "lights", id, want, have} từ máy chủ
export function onLightsMessage(m) {
  const id = String(m.id);
  if (Number.isInteger(m.want)) state.lights.set(id, m.want);
  const b = state.buoys.get(id);
  if (b && Number.isInteger(m.have)) b.lights = m.have;
  if (id === state.selected && b) renderLights(b, id);
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
      state.lights.set(id, mask);   // hiện "đang gửi" ngay, không chờ máy chủ trả lời
      renderLights(b, id);
      onSet(id, mask, prev);
    });
  }
}
