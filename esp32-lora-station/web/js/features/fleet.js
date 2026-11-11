// Đội tàu (docs/fleet-control.md): trạm bờ giao nhiệm vụ cho nhiều phao, không lái từng chiếc.
//   - bảng đội tàu: mỗi phao một màu, trạng thái, nhiệm vụ, lộ trình đã giao chưa, pin, mất liên lạc; về bờ từng chiếc
//   - lệnh cả đội: DỪNG / CHẠY TIẾP / VỀ BỜ tất cả
//   - kế hoạch: vẽ vùng tìm kiếm trên bản đồ, chọn phao, "Chia vùng" (mỗi phao một dải) hoặc "Đội hình" (một phao dẫn
//     đầu, các phao khác giữ chỗ quanh nó); xem trước rồi gửi. Worker tính kế hoạch (cloud/src/fleetplan.js).
//   - trên bản đồ: lộ trình đang giao của từng phao theo màu của nó, vùng tìm kiếm, đường nối đội hình
import { state, buoyStatus, STATUS, batteryOf, isStale } from '../state.js';
import { log } from './timeline.js';

const $ = id => document.getElementById(id);
export const PALETTE = ['#22D3EE', '#F59E0B', '#A78BFA', '#34D399', '#F472B6', '#60A5FA', '#FB923C', '#E879F9'];

// Màu cố định theo số cuối tên phao (PHAO-01 xanh như trước, PHAO-02 cam, ...), tên khác theo băm
export function colorOf(id) {
  const m = /(\d+)$/.exec(id);
  if (m) return PALETTE[(Math.max(1, +m[1]) - 1) % PALETTE.length];
  let h = 0;
  for (const c of id) h = (h * 31 + c.charCodeAt(0)) % 997;
  return PALETTE[h % PALETTE.length];
}

const fleet = {
  routes: new Map(),     // id -> bản gửi gần nhất { rev, n, pts, loop, delivered }
  tasks: new Map(),      // id -> { kind, ref, since }
  ret: new Map(),        // id -> { want, have }
  lost: new Map(),       // id -> since
  formation: null,
  area: [],              // vùng tìm kiếm đang vẽ: [{lat, lon}]
  preview: null,         // kế hoạch xem trước từ Worker
  picked: new Set(),     // phao được chọn cho kế hoạch; rỗng = mọi phao đang nghe được
};
let map = null, ctx = null;
const layer = L.layerGroup();

// The search area being drawn: a bold outline in the accent colour over a dark halo (it stands out on the dark and the
// light map alike), a clear fill, numbered corners that can be dragged, a dashed line from the last corner to the
// pointer and back to the first while drawing, and the area's size in the middle.
const areaLayer = L.layerGroup();
const areaHalo = L.polygon([], { color: '#020617', weight: 7, opacity: .55, fill: false, interactive: false });
const areaLine = L.polygon([], { weight: 3, fillOpacity: .16, interactive: false });
const rubber = L.polyline([], { weight: 2, dashArray: '6 6', opacity: .9, interactive: false });
const cssVar = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim() || '#22D3EE';
let pointer = null;   // where the pointer is over the map while drawing

// Square metres of a polygon of {lat, lon} (flat Earth round its first corner: right for a search area)
function areaM2(pts) {
  if (pts.length < 3) return 0;
  const k = Math.cos(pts[0].lat * Math.PI / 180) * 111320;
  const xy = pts.map(p => [(p.lon - pts[0].lon) * k, (p.lat - pts[0].lat) * 110540]);
  let s = 0;
  for (let i = 0; i < xy.length; i++) { const [x1, y1] = xy[i], [x2, y2] = xy[(i + 1) % xy.length]; s += x1 * y2 - x2 * y1; }
  return Math.abs(s) / 2;
}
const fmtArea = m2 => (m2 >= 1e6 ? `${(m2 / 1e6).toLocaleString('vi-VN', { maximumFractionDigits: 2 })} km²`
  : m2 >= 1e4 ? `${(m2 / 1e4).toLocaleString('vi-VN', { maximumFractionDigits: 1 })} ha` : `${Math.round(m2).toLocaleString('vi-VN')} m²`);

const cornerLayer = L.layerGroup();
const areaLabel = L.tooltip({ permanent: true, direction: 'center', className: 'area-label', interactive: false });
const isDrawing = () => document.body.dataset.mode === 'fleet' && state.role === 'admin';

// The outline, the line to the pointer and the size: cheap, redrawn as the pointer moves or a corner is dragged
function drawShape() {
  const accent = cssVar('--accent');
  const pts = fleet.area.map(a => [a.lat, a.lon]);
  areaHalo.setLatLngs(pts.length >= 2 ? pts : []);
  areaLine.setStyle({ color: accent, fillColor: accent }).setLatLngs(pts.length >= 2 ? pts : []);
  // While drawing: from the last corner to the pointer, and from the pointer back to the first corner
  if (isDrawing() && pointer && pts.length) {
    rubber.setStyle({ color: accent }).setLatLngs([pts[pts.length - 1], pointer, ...(pts.length >= 2 ? [pts[0]] : [])]);
    if (!areaLayer.hasLayer(rubber)) rubber.addTo(areaLayer);
  } else rubber.remove();
  if (pts.length >= 3) {
    areaLabel.setLatLng(areaLine.getBounds().getCenter()).setContent(`${fmtArea(areaM2(fleet.area))} · ${pts.length} góc`);
    if (!areaLayer.hasLayer(areaLabel)) areaLabel.addTo(areaLayer);
  } else areaLabel.remove();
}

// Everything, the numbered corners too: when a corner is added or taken away, or the mode changes
function drawArea() {
  if (!areaLayer.hasLayer(areaHalo)) { areaHalo.addTo(areaLayer); areaLine.addTo(areaLayer); cornerLayer.addTo(areaLayer); }
  cornerLayer.clearLayers();
  const drawing = isDrawing();
  fleet.area.forEach((a, i) => {
    const icon = L.divIcon({ className: '', iconSize: [24, 24], iconAnchor: [12, 12], html: `<span class="area-corner">${i + 1}</span>` });
    const m = L.marker([a.lat, a.lon], { icon, draggable: drawing, keyboard: false, title: `Góc ${i + 1}${drawing ? ' (kéo để chỉnh)' : ''}`, zIndexOffset: 800 })
      .addTo(cornerLayer);
    m.on('drag', e => { const p = e.target.getLatLng(); fleet.area[i] = { lat: p.lat, lon: p.lng }; drawShape(); });
    m.on('dragend', () => { fleet.preview = null; renderPlanForm(); drawLayer(); });
  });
  drawShape();
}

const TASK_TEXT = {
  idle: () => 'Chờ lệnh',
  sector: t => `Quét dải được giao (${t.ref?.n ?? '?'} điểm)`,
  return: () => 'Về điểm đỗ',
  formation: t => t.ref?.role === 'leader' ? 'Dẫn đầu đội hình'
    : `Theo ${t.ref?.leader ?? '?'} (${placeText(t.ref)})`,
  victim: t => `Đến người gặp nạn${t.ref?.vid ? ' #' + t.ref.vid : ''}`,
};

function placeText(p) {
  if (!p) return '';
  const side = p.right > 0 ? `phải ${p.right} m` : p.right < 0 ? `trái ${-p.right} m` : '';
  const back = p.back ? `sau ${p.back} m` : '';
  return [side, back].filter(Boolean).join(', ') || 'ngay sau';
}

function routeText(id) {
  const r = fleet.routes.get(id);
  if (!r || !r.rev) return 'chưa giao lộ trình';
  const b = state.buoys.get(id);
  const have = b?.route && b.route.rev === r.rev ? b.route : null;
  if (have && have.have === r.n && have.pi) return r.n ? `đã nhận ${r.n} điểm` : 'đã bỏ lộ trình';
  if (have) return `đang nhận ${have.have}/${r.n}`;
  return r.delivered ? 'khởi động lại, mất lộ trình' : `chờ nhận ${r.n} điểm`;
}

// Các phao kế hoạch dùng: những phao được tích, hoặc mọi phao còn liên lạc
function planIds() {
  const live = [...state.buoys.entries()].filter(([, b]) => !isStale(b) && b.link !== 'mavlink').map(([id]) => id).sort();
  const picked = [...fleet.picked].filter(id => state.buoys.has(id)).sort();
  return picked.length ? picked : live;
}

let sig = '';
export function renderFleet() {
  if (!ctx) return;
  const ids = [...state.buoys.keys()].filter(id => state.buoys.get(id).link !== 'mavlink').sort();
  const rows = ids.map(id => {
    const b = state.buoys.get(id), st = buoyStatus(id, b), t = fleet.tasks.get(id);
    return { id, st, batt: batteryOf(b), lost: fleet.lost.has(id) || st === 'lost', task: t ? (TASK_TEXT[t.kind] || TASK_TEXT.idle)(t) : 'Chờ lệnh',
      route: routeText(id), ret: fleet.ret.get(id)?.want === true, picked: planIds().includes(id) };
  });
  const s = JSON.stringify([state.selected, rows, !!fleet.formation]);
  if (s !== sig) {
    sig = s;
    const list = $('f-list');
    list.replaceChildren();
    $('f-count').textContent = rows.length;
    if (!rows.length) {
      const p = document.createElement('p'); p.className = 'quiet list-empty'; p.textContent = 'Chưa có phao nào trong đội.';
      list.appendChild(p);
    }
    for (const r of rows) {
      const el = document.createElement('div');
      el.className = 'f-row';
      if (r.id === state.selected) el.setAttribute('aria-current', 'true');
      el.style.setProperty('--boat', colorOf(r.id));
      const pick = document.createElement('input');
      pick.type = 'checkbox'; pick.className = 'f-pick admin-only'; pick.checked = r.picked;
      pick.setAttribute('aria-label', `Dùng ${r.id} cho kế hoạch`);
      pick.addEventListener('change', () => {
        if (!fleet.picked.size) planIds().forEach(i => fleet.picked.add(i));
        if (pick.checked) fleet.picked.add(r.id); else fleet.picked.delete(r.id);
        renderFleet(); renderPlanForm();
      });
      const main = document.createElement('button');
      main.type = 'button'; main.className = 'f-main';
      main.innerHTML = '<i class="f-dot" aria-hidden="true"></i>';
      const name = document.createElement('b'); name.textContent = r.id;
      const tag = document.createElement('span'); tag.className = 'tag'; tag.dataset.tone = r.lost ? 'bad' : STATUS[r.st].tone;
      tag.textContent = r.lost ? 'MẤT LIÊN LẠC' : STATUS[r.st].text;
      const task = document.createElement('small'); task.textContent = `${r.task} · ${r.route}${r.batt !== null ? ` · pin ${r.batt}%` : ''}`;
      const head = document.createElement('span'); head.className = 'f-head'; head.append(name, tag);
      main.append(head, task);
      main.addEventListener('click', () => ctx.onSelect(r.id));
      const ret = document.createElement('button');
      ret.type = 'button'; ret.className = 'btn small admin-only';
      ret.textContent = r.ret ? 'Chạy tiếp' : 'Về bờ';
      ret.title = r.ret ? `Bỏ lệnh về bờ cho ${r.id}` : `Gọi ${r.id} về điểm đỗ của nó`;
      ret.addEventListener('click', () => sendReturn(r.id, !r.ret));
      el.append(pick, main, ret);
      list.appendChild(el);
    }
    $('f-end').hidden = !fleet.formation;
  }
  drawLayer();
}

function drawLayer() {
  layer.clearLayers();
  // Lộ trình đang giao của từng phao, theo màu của nó
  for (const [id, r] of fleet.routes) {
    if (!r.pts?.length || !state.buoys.has(id)) continue;
    L.polyline(r.pts, { color: colorOf(id), weight: 3, opacity: .8, dashArray: r.delivered ? null : '8 6', interactive: false }).addTo(layer);
  }
  // Đội hình: đường mảnh từ phao dẫn đầu tới các phao theo
  const f = fleet.formation, lead = f && state.buoys.get(f.leader);
  if (lead?.hasPos) {
    for (const id of Object.keys(f.members)) {
      const b = state.buoys.get(id);
      if (b?.hasPos) L.polyline([[lead.lat, lead.lon], [b.lat, b.lon]], { color: colorOf(f.leader), weight: 1.5, opacity: .7, dashArray: '2 5', interactive: false }).addTo(layer);
    }
  }
  // Xem trước kế hoạch
  // Xem trước kế hoạch: đậm, trên một quầng tối, để thấy rõ trên bản đồ sáng lẫn tối
  const p = fleet.preview;
  const preview = (pts, color) => {
    L.polyline(pts, { color: '#020617', weight: 7, opacity: .45, interactive: false }).addTo(layer);
    L.polyline(pts, { color, weight: 4, opacity: .95, interactive: false }).addTo(layer);
  };
  if (p?.mode === 'sectors') {
    for (const r of p.routes) preview(r.pts, colorOf(r.id));
  } else if (p?.mode === 'formation' && p.route?.length) {
    preview(p.route, colorOf(p.leader));
  }
  drawArea();
}

function renderPlanForm() {
  if (!ctx) return;
  const mode = document.querySelector('input[name="f-mode"]:checked')?.value || 'sectors';
  $('f-form-opts').hidden = mode !== 'formation';
  const ids = planIds();
  const sel = $('f-leader');
  if (sel.dataset.ids !== ids.join()) {   // rebuilt only when the boats change: an open list keeps its place
    const keep = sel.value;
    sel.replaceChildren(...ids.map(id => Object.assign(document.createElement('option'), { value: id, textContent: id })));
    if (ids.includes(keep)) sel.value = keep;
    sel.dataset.ids = ids.join();
  }
  $('f-area-sum').textContent = fleet.area.length ? `${fleet.area.length} góc` : '';
  const ready = fleet.area.length >= 3 && ids.length >= 1;
  $('f-preview').disabled = !ready;
  $('f-send').disabled = !ready;
  $('f-area-undo').disabled = !fleet.area.length;
  $('f-area-clear').disabled = !fleet.area.length;
  $('f-hint').textContent = fleet.area.length < 3
    ? 'Chạm lên bản đồ để vẽ vùng tìm kiếm (ít nhất 3 góc).'
    : `${ids.length} phao: ${ids.join(', ')}`;
}

function planBody(dry) {
  const mode = document.querySelector('input[name="f-mode"]:checked')?.value || 'sectors';
  return { mode, poly: fleet.area, ids: planIds(), spacing: Number($('f-spacing').value) || 10, shape: $('f-shape').value,
    leader: $('f-leader').value, loop: $('f-loop').checked, dry };
}

async function post(path, body) {
  const res = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body ?? {}) });
  if (!res.ok) throw new Error(res.status === 401 || res.status === 403 ? 'Phiên hết hạn hoặc không có quyền' : 'Máy chủ từ chối lệnh');
  return res.json();
}

export async function sendReturn(id, ret) {
  if (ctx.mock) { window.mockServer?.setReturn?.(id, ret); return; }
  try { onRetMessage(await post('/api/return', { id, ret })); ctx.toast(ret ? `Đã gọi ${id} về bờ` : `${id} chạy tiếp`, 'info'); }
  catch (e) { ctx.toast(e.message || 'Mất kết nối'); }
}

async function fleetCommand(action) {
  const words = { hold: 'DỪNG tất cả phao', resume: 'cho tất cả phao chạy tiếp', return: 'gọi tất cả phao về bờ' };
  if (!confirm(`Xác nhận ${words[action]}?`)) return;
  if (ctx.mock) { window.mockServer?.fleet?.(action); return; }
  try {
    const r = await post('/api/fleet', { action });
    ctx.toast(`Đã ${words[action]} (${r.ids.length} phao)`, action === 'hold' ? 'bad' : 'info');
    log('fleet', `Lệnh cả đội: ${words[action]} (${r.ids.join(', ') || 'không có phao nào'})`);
  } catch (e) { ctx.toast(e.message || 'Mất kết nối'); }
}

async function plan(dry) {
  const body = planBody(dry);
  if (!dry && !confirm(body.mode === 'formation'
    ? `Gửi đội hình: ${body.leader} dẫn đầu, ${body.ids.length - 1} phao theo?`
    : `Chia vùng cho ${body.ids.length} phao và gửi lộ trình cho từng phao?`)) return;
  if (ctx.mock) { fleet.preview = window.mockServer?.plan?.(body) || null; drawLayer(); return; }
  try {
    const r = await post('/api/fleet/plan', body);
    fleet.preview = dry ? r : null;
    if (!dry) {
      ctx.toast(r.mode === 'formation' ? `Đã gửi đội hình, ${r.leader} dẫn đầu` : `Đã chia vùng cho ${r.routes.length} phao`, 'ok');
      log('fleet', r.mode === 'formation' ? `Đội hình ${r.shape}: ${r.leader} dẫn đầu, theo sau ${Object.keys(r.members).join(', ')}`
        : `Chia vùng tìm kiếm cho ${r.routes.map(x => x.id).join(', ')}`);
    }
    drawLayer();
  } catch (e) { ctx.toast(e.message || 'Mất kết nối'); }
}

// ---- tin nhắn từ Worker ----
export function onFleetDownlink(m) {
  fleet.routes.set(m.id, { rev: m.rev, n: m.n, pts: m.pts || fleet.routes.get(m.id)?.pts, loop: m.loop, delivered: m.delivered });
  renderFleet();
}
export function onTaskMessage(m) { fleet.tasks.set(m.id, { kind: m.kind, ref: m.ref, since: m.since }); renderFleet(); }
export function onRetMessage(m) { fleet.ret.set(m.id, { want: m.want, have: m.have }); renderFleet(); }
export function onLinkMessage(m, snapshot) {
  if (m.lost) fleet.lost.set(m.id, m.since); else fleet.lost.delete(m.id);
  if (!snapshot && !m.lost) ctx?.toast(`${m.id} đã có liên lạc lại`, 'ok');   // the timeline logs signal lost and back
  renderFleet();
}
export function onFormationMessage(m) { fleet.formation = m.formation; renderFleet(); }
export const retOf = id => fleet.ret.get(id)?.want === true;

export function initFleet(m, c) {
  map = m; ctx = c;
  layer.addTo(map); areaLayer.addTo(map);
  window.addEventListener('themechange', drawArea);
  map.on('mousemove', e => {
    if (document.body.dataset.mode !== 'fleet' || !fleet.area.length) return;
    pointer = e.latlng; drawShape();
  });
  map.on('mouseout', () => { if (pointer) { pointer = null; drawShape(); } });
  map.on('click', e => {
    if (document.body.dataset.mode !== 'fleet' || state.role !== 'admin') return;
    fleet.area.push({ lat: e.latlng.lat, lon: e.latlng.lng });
    fleet.preview = null;
    renderPlanForm(); drawLayer();
  });
  $('f-area-undo').addEventListener('click', () => { fleet.area.pop(); fleet.preview = null; renderPlanForm(); drawLayer(); });
  $('f-area-clear').addEventListener('click', () => { fleet.area = []; fleet.preview = null; renderPlanForm(); drawLayer(); });
  for (const r of document.querySelectorAll('input[name="f-mode"]')) r.addEventListener('change', () => { fleet.preview = null; renderPlanForm(); drawLayer(); });
  $('f-preview').addEventListener('click', () => plan(true));
  $('f-send').addEventListener('click', () => plan(false));
  $('f-hold-all').addEventListener('click', () => fleetCommand('hold'));
  $('f-resume-all').addEventListener('click', () => fleetCommand('resume'));
  $('f-return-all').addEventListener('click', () => fleetCommand('return'));
  $('f-end').addEventListener('click', async () => {
    if (!confirm('Kết thúc đội hình? Các phao đứng tại chỗ chờ lệnh mới.')) return;
    try { await post('/api/formation/end'); ctx.toast('Đã kết thúc đội hình', 'info'); } catch (e) { ctx.toast(e.message || 'Mất kết nối'); }
  });
  renderPlanForm();
  renderFleet();
  setInterval(() => { renderFleet(); renderPlanForm(); }, 1000);
}
