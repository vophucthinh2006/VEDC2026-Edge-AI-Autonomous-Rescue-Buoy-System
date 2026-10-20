// Người phao phát hiện (chấm màu theo mức nguy hiểm) và cảnh báo lật phao / xác nhận nhận đồ cứu hộ.
// Máy chủ đã bỏ bản phao gửi lặp và gộp phát hiện gần nhau (5 m), ở đây chỉ vẽ và báo.
import { state } from '../state.js';
import { hms } from '../util/format.js';
import { log } from './eventlog.js';

export const RISKS = [
  { name: 'An toàn', color: '#22C55E' },
  { name: 'Trung bình', color: '#FACC15' },
  { name: 'Cao', color: '#F97316' },
  { name: 'Nguy kịch', color: '#EF4444' },
];

const dotLayer = L.layerGroup();
const alertLayer = L.layerGroup();
const dots = new Map();   // vid -> circleMarker
let toast = () => {};

const posText = (lat, lon) => `${lat.toFixed(5)}, ${lon.toFixed(5)}`;
const timeText = t => hms(new Date(t));

function row(dl, k, v) {
  const dt = document.createElement('dt'); dt.textContent = k;
  const dd = document.createElement('dd'); dd.textContent = v;
  dl.append(dt, dd);
}

function victimPopup(v) {
  const box = document.createElement('div');
  box.className = 'map-pop';
  const h = document.createElement('b');
  h.textContent = 'Người phát hiện: ' + RISKS[v.risk].name;
  h.style.color = RISKS[v.risk].color;
  const dl = document.createElement('dl');
  row(dl, 'Độ tin cậy', Math.round(v.conf * 100) + '%');
  row(dl, 'Thấy lần đầu', timeText(v.first_at));
  row(dl, 'Thấy lần cuối', timeText(v.last_at));
  row(dl, 'Phao', v.buoy);
  row(dl, 'Vị trí', posText(v.lat, v.lon));
  box.append(h, dl);
  return box;
}

function putVictim(v) {
  if (!RISKS[v.risk] || !Number.isFinite(v.lat) || !Number.isFinite(v.lon)) return;
  const style = { radius: 7 + v.risk, color: '#F8FAFC', weight: 2, fillColor: RISKS[v.risk].color, fillOpacity: .95 };
  const m = dots.get(v.vid);
  if (m) m.setLatLng([v.lat, v.lon]).setStyle(style).setRadius(style.radius).setPopupContent(() => victimPopup(v));
  else dots.set(v.vid, L.circleMarker([v.lat, v.lon], style).bindPopup(() => victimPopup(v), { className: 'pop' }).addTo(dotLayer));
}

function alertPopup(a) {
  const box = document.createElement('div');
  box.className = 'map-pop';
  const h = document.createElement('b');
  h.textContent = 'Phao lật / nghiêng lớn';
  h.style.color = 'var(--bad)';
  const dl = document.createElement('dl');
  row(dl, 'Phao', a.buoy);
  row(dl, 'Lúc', timeText(a.at));
  row(dl, 'Roll / Pitch', `${a.roll}° / ${a.pitch}°`);
  row(dl, 'Vị trí', posText(a.lat, a.lon));
  box.append(h, dl);
  return box;
}

// Chỉ cảnh báo lật có vị trí mới có dấu trên bản đồ
function putAlertMarker(a) {
  if (a.type !== 'T' || !Number.isFinite(a.lat) || !Number.isFinite(a.lon)) return;
  const icon = L.divIcon({ className: '', iconSize: [28, 28], iconAnchor: [14, 14],
    html: '<div class="alert-pin"><i class="ph-fill ph-warning" aria-hidden="true"></i></div>' });
  L.marker([a.lat, a.lon], { icon, zIndexOffset: 900 })
    .bindTooltip(`${a.buoy} lật lúc ${timeText(a.at)}`, { direction: 'top', offset: [0, -14], className: 'tip' })
    .bindPopup(() => alertPopup(a), { className: 'pop' })
    .addTo(alertLayer);
}

// Ảnh chụp toàn bộ lúc mở trang, hoặc sau khi admin xoá chấm: vẽ lại, không báo
export function onVictimsSnapshot(m) {
  dotLayer.clearLayers(); dots.clear(); alertLayer.clearLayers();
  for (const v of m.victims || []) putVictim(v);
  for (const a of m.alerts || []) putAlertMarker(a);
  if (!(m.victims || []).length && !(m.alerts || []).length) log('info', 'Đã xoá chấm người và cảnh báo trên bản đồ');
}

export function onVictimMessage(m) {
  const v = m.victim;
  if (!v || !RISKS[v.risk]) return;
  state.eventBuoys.add(v.buoy);
  const before = dots.get(v.vid);
  putVictim(v);
  const risk = RISKS[v.risk].name;
  const conf = Math.round(v.conf * 100);
  if (m.new || !before) {
    log('err', `${v.buoy} PHÁT HIỆN NGƯỜI: mức ${risk}, tin cậy ${conf}%, tại ${posText(v.lat, v.lon)}`);
    toast(`${v.buoy} phát hiện người (${risk})`, 'bad');
  } else {
    log('warn', `${v.buoy} thấy lại người tại ${posText(v.lat, v.lon)}: mức ${risk}, tin cậy ${conf}%`);
  }
}

export function onAlertMessage(m) {
  const a = m.alert;
  if (!a) return;
  state.eventBuoys.add(a.buoy);
  const where = Number.isFinite(a.lat) && Number.isFinite(a.lon) ? posText(a.lat, a.lon) : 'vị trí chưa rõ';
  if (a.type === 'T') {
    putAlertMarker(a);
    log('err', `${a.buoy} LẬT HOẶC NGHIÊNG LỚN (roll ${a.roll}°, pitch ${a.pitch}°) tại ${where}`);
    toast(`${a.buoy} bị lật hoặc nghiêng lớn`, 'bad');
  } else if (a.type === 'C') {
    log('ok', `${a.buoy}: người gặp nạn đã bấm xác nhận nhận đồ cứu hộ (${a.presses} lần)`);
    toast(`${a.buoy}: người gặp nạn đã nhận đồ cứu hộ`, 'ok');
  } else if (a.type === 'V' && RISKS[a.risk]) {
    log('err', `${a.buoy} PHÁT HIỆN NGƯỜI: mức ${RISKS[a.risk].name}, tin cậy ${Math.round(a.conf * 100)}%, phao chưa có GPS`);
    toast(`${a.buoy} phát hiện người (chưa có vị trí)`, 'bad');
  }
}

// Chú thích trên bản đồ + nút "Xoá chấm" cho admin
function legendControl(onClear) {
  const ctl = L.control({ position: 'bottomleft' });
  ctl.onAdd = () => {
    const box = L.DomUtil.create('div', 'map-legend');
    L.DomEvent.disableClickPropagation(box);
    L.DomEvent.disableScrollPropagation(box);
    // Bấm tiêu đề để thu gọn; trên điện thoại thu gọn sẵn cho đỡ che bản đồ
    const title = document.createElement('button');
    title.type = 'button'; title.className = 't';
    title.innerHTML = '<span>Người phát hiện</span><i class="ph ph-caret-down" aria-hidden="true"></i>';
    const collapse = on => { box.classList.toggle('collapsed', on); title.setAttribute('aria-expanded', String(!on)); };
    title.addEventListener('click', () => collapse(!box.classList.contains('collapsed')));
    collapse(window.matchMedia('(max-width: 600px), (max-width: 820px) and (orientation: portrait)').matches);
    box.appendChild(title);
    for (const r of RISKS) {
      const item = document.createElement('div');
      const dot = document.createElement('i'); dot.className = 'dot'; dot.style.background = r.color;
      item.append(dot, r.name);
      box.appendChild(item);
    }
    const extra = [['warn', 'Phao lật'], ['hdg', 'Hướng mũi (IMU)'], ['cog', 'Hướng đi (GPS)']];
    for (const [cls, text] of extra) {
      const item = document.createElement('div');
      const ic = document.createElement('i'); ic.className = cls;
      if (cls === 'warn') { ic.className = 'ph-fill ph-warning warn-ic'; ic.setAttribute('aria-hidden', 'true'); }
      item.append(ic, text);
      box.appendChild(item);
    }
    const btn = document.createElement('button');
    btn.type = 'button'; btn.className = 'btn admin-only'; btn.id = 'btn-clear-victims';
    btn.innerHTML = '<i class="ph ph-eraser" aria-hidden="true"></i>Xoá chấm';
    btn.addEventListener('click', () => {
      if (confirm('Xoá mọi chấm người và cảnh báo lật trên bản đồ? Mọi người xem cùng bị xoá.')) onClear();
    });
    box.appendChild(btn);
    return box;
  };
  return ctl;
}

export function initVictims(map, { toast: t, onClear }) {
  toast = t;
  alertLayer.addTo(map);
  dotLayer.addTo(map);
  legendControl(onClear).addTo(map);
}
