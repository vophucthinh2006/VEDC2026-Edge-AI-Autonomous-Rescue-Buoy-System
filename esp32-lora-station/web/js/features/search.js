// Address search for guests (admins too): type a place or use the browser location, then see which
// buoys are within 1/2/5 km, how far and in which direction, and how many people need rescue nearby.
// Geocoding goes through the Worker (/api/geocode -> Nominatim); distances are computed here from state.buoys.
import { state, buoyStatus, STATUS } from '../state.js';
import { distM, bearingDeg, compassWord } from '../util/geo.js';
import { fmtDist } from '../util/format.js';
import { victimsNear } from './victims.js';

const $ = id => document.getElementById(id);
const RADII_KM = [1, 2, 5];

let map = null;
let onShowBuoy = () => {};
let onOpen = () => {};
let place = null;          // { name, lat, lon }
let radiusKm = 2;
let lastSig = '';          // skip re-rendering the result list when nothing changed
const layer = L.layerGroup();

function panel() { return $('search-panel'); }
function open(content) {
  const p = panel();
  const was = !p.hidden;
  p.replaceChildren(...content);
  p.hidden = false;
  if (!was) { p.classList.remove('min'); onOpen(); }
}

// Fold the result panel to its title (phones, after "Xem trên bản đồ"); tapping the title unfolds it
export function minimizeSearch(on = true) {
  panel().classList.toggle('min', on && !panel().hidden);
}
function close() {
  panel().hidden = true;
  panel().replaceChildren();
  place = null; lastSig = '';
  layer.clearLayers();
}

function h(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

function closeBtn(label = 'Đóng') {
  const b = h('button', 'icon-btn close');
  b.type = 'button'; b.setAttribute('aria-label', label); b.title = label;
  b.innerHTML = '<i class="ph ph-x" aria-hidden="true"></i>';
  b.addEventListener('click', close);
  return b;
}

function message(text, tone = '') {
  const head = h('div', 'sp-head');
  const p = h('p', 'sp-msg' + (tone ? ' ' + tone : ''), text);
  head.append(p, closeBtn());
  open([head]);
}

function loading() {
  const wrap = h('div', 'sp-loading');
  wrap.setAttribute('aria-label', 'Đang tìm');
  for (let i = 0; i < 3; i++) wrap.appendChild(h('span', 'skel'));
  open([wrap]);
}

// Split "Bến Ninh Kiều, Ninh Kiều, Cần Thơ, Việt Nam" into a title and the rest
function splitName(name) {
  const parts = name.split(',').map(s => s.trim()).filter(Boolean);
  return { title: parts[0] || name, rest: parts.slice(1).filter(s => s !== 'Việt Nam').join(', ') };
}

function showChoices(results) {
  const head = h('div', 'sp-head');
  head.append(h('p', 'sp-title', 'Chọn đúng địa điểm'), closeBtn());
  const list = h('ul', 'sp-choices');
  for (const r of results) {
    const li = h('li');
    const b = h('button', 'choice'); b.type = 'button';
    const { title, rest } = splitName(r.name);
    b.innerHTML = '<i class="ph ph-map-pin" aria-hidden="true"></i>';
    const t = h('span', 'c-t'); t.append(h('b', '', title), h('small', '', rest));
    b.appendChild(t);
    b.addEventListener('click', () => choose({ name: r.name, lat: r.lat, lon: r.lon }));
    li.appendChild(b);
    list.appendChild(li);
  }
  open([head, list]);
}

function choose(p) {
  place = p;
  lastSig = '';
  layer.clearLayers();
  L.marker([p.lat, p.lon], {
    zIndexOffset: 1200, keyboard: false,
    icon: L.divIcon({ className: '', iconSize: [30, 38], iconAnchor: [15, 36],
      html: '<div class="place-pin"><i class="ph-fill ph-map-pin" aria-hidden="true"></i></div>' }),
  }).bindTooltip('Vị trí tìm kiếm', { direction: 'top', offset: [0, -34], className: 'tip' }).addTo(layer);
  L.circle([p.lat, p.lon], { radius: radiusKm * 1000, color: '#22D3EE', weight: 1.5, opacity: .9,
    fillColor: '#22D3EE', fillOpacity: .07, dashArray: '6 6', interactive: false }).addTo(layer);
  map.flyToBounds(L.latLng(p.lat, p.lon).toBounds(radiusKm * 2000 * 1.15), { duration: .6 });
  renderPlace();
}

function setRadius(km) {
  radiusKm = km;
  if (place) choose(place);
}

// Results for the chosen place, recomputed as buoys move
export function renderPlace() {
  if (!place) return;
  const r = radiusKm * 1000;
  const buoys = [...state.buoys.entries()]
    .filter(([, b]) => b.hasPos)
    .map(([id, b]) => ({ id, b, d: distM(place, b), brg: bearingDeg(place, b), st: STATUS[buoyStatus(id, b)] }))
    .sort((a, b) => a.d - b.d);
  const inside = buoys.filter(x => x.d <= r);
  const people = victimsNear(place, r).filter(v => v.risk > 0).length;
  const whereText = x => x.d < 30 ? 'Ngay tại đây' : `${fmtDist(x.d)} về hướng ${compassWord(x.brg)}`;
  const nearest = buoys[0];
  const summaryText = inside.length ? `${inside.length} phao trong bán kính ${radiusKm} km`
    : nearest ? `Chưa có phao trong bán kính ${radiusKm} km. Phao gần nhất: ${nearest.id} (${fmtDist(nearest.d)}).`
    : 'Chưa có phao nào gửi vị trí.';
  // Rebuild only when the set of buoys in range changes; distances, status tags and the people count
  // update in place, so a button is never swapped out under the pointer
  const peopleText = `${people} người cần cứu trong bán kính này`;
  const sig = JSON.stringify([place.name, radiusKm, !!people, inside.map(x => x.id), !inside.length && nearest?.id]);
  if (sig === lastSig) {
    const p = panel();
    const sum = p.querySelector('.sp-sum');
    if (sum) sum.textContent = summaryText;
    const ppl = p.querySelector('.sp-people span');
    if (ppl) ppl.textContent = peopleText;
    for (const x of inside) {
      const row = p.querySelector(`.sp-buoy[data-id="${CSS.escape(x.id)}"]`);
      if (!row) continue;
      row.querySelector('small').textContent = whereText(x);
      const tag = row.querySelector('.tag');
      tag.textContent = x.st.text; tag.dataset.tone = x.st.tone;
    }
    return;
  }
  lastSig = sig;

  const head = h('div', 'sp-head');
  const t = h('div', 'sp-place');
  const { title, rest } = splitName(place.name);
  t.append(h('b', '', title), h('small', '', rest || 'Vị trí tìm kiếm'));
  head.append(t, closeBtn());

  const radii = h('div', 'seg');
  radii.setAttribute('role', 'group'); radii.setAttribute('aria-label', 'Bán kính');
  for (const km of RADII_KM) {
    const b = h('button', '', km + ' km'); b.type = 'button';
    b.setAttribute('aria-pressed', String(km === radiusKm));
    b.addEventListener('click', () => setRadius(km));
    radii.appendChild(b);
  }

  const summary = h('p', 'sp-sum', summaryText);
  summary.classList.toggle('quiet', !inside.length);

  const nodes = [head, radii, summary];
  if (people) {
    const p = h('p', 'sp-people');
    p.innerHTML = '<i class="ph-fill ph-person-simple-swim" aria-hidden="true"></i>';
    p.append(h('span', '', peopleText));
    nodes.push(p);
  }
  if (inside.length) {
    const list = h('ul', 'sp-buoys');
    for (const x of inside) {
      const li = h('li', 'sp-buoy');
      li.dataset.id = x.id;
      const info = h('div', 'sb-info');
      const name = h('b', '', x.id);
      const tag = h('span', 'tag', x.st.text); tag.dataset.tone = x.st.tone;
      const where = h('small', '', whereText(x));
      info.append(name, tag, where);
      const go = h('button', 'btn small', 'Xem trên bản đồ'); go.type = 'button';
      go.setAttribute('aria-label', `Xem ${x.id} trên bản đồ`);
      go.addEventListener('click', ev => { ev.stopPropagation(); onShowBuoy(x.id); });
      li.append(info, go);
      list.appendChild(li);
    }
    nodes.push(list);
  }
  open(nodes);
}

async function search(q) {
  q = q.trim().replace(/\s+/g, ' ');
  if (q.length < 2) { message('Nhập ít nhất 2 ký tự.'); return; }
  loading();
  try {
    const res = await fetch('/api/geocode?' + new URLSearchParams({ q }));
    const data = await res.json().catch(() => ({}));
    if (res.status === 429) { message('Bạn tìm hơi nhanh. Đợi một giây rồi thử lại.', 'warn'); return; }
    if (res.status === 401) { message('Phiên đã hết hạn. Tải lại trang để vào lại.', 'warn'); return; }
    if (!res.ok) { message('Chưa tìm được lúc này. Thử lại sau ít phút.', 'warn'); return; }
    const results = Array.isArray(data.results) ? data.results : [];
    if (!results.length) { message('Không tìm thấy địa chỉ này. Thử ghi rõ hơn, ví dụ thêm quận hoặc tỉnh.'); return; }
    if (results.length === 1) choose(results[0]); else showChoices(results);
  } catch {
    message('Mất kết nối, chưa tìm được.', 'warn');
  }
}

function locate() {
  if (!navigator.geolocation) { message('Trình duyệt không hỗ trợ lấy vị trí.', 'warn'); return; }
  loading();
  navigator.geolocation.getCurrentPosition(
    pos => choose({ name: 'Vị trí của bạn', lat: pos.coords.latitude, lon: pos.coords.longitude }),
    err => message(err.code === 1 ? 'Bạn chưa cho phép lấy vị trí. Hãy nhập địa chỉ thay thế.' : 'Không lấy được vị trí. Hãy nhập địa chỉ thay thế.', 'warn'),
    { enableHighAccuracy: true, timeout: 10000, maximumAge: 60000 },
  );
}

export function initSearch(leafletMap, { onShowBuoy: fn, onOpen: fnOpen }) {
  map = leafletMap;
  onShowBuoy = fn;
  onOpen = fnOpen;
  panel().addEventListener('click', ev => {
    if (panel().classList.contains('min') && !ev.target.closest('.close')) minimizeSearch(false);
  });
  layer.addTo(map);
  const form = $('search');
  L.DomEvent.disableClickPropagation($('search-wrap'));
  L.DomEvent.disableScrollPropagation($('search-wrap'));
  form.addEventListener('submit', ev => { ev.preventDefault(); $('q').blur(); search($('q').value); });
  $('btn-locate').addEventListener('click', locate);
  $('q').addEventListener('search', () => { if (!$('q').value) close(); });   // the input's clear button
  document.addEventListener('keydown', ev => { if (ev.key === 'Escape' && !panel().hidden) close(); });
}
