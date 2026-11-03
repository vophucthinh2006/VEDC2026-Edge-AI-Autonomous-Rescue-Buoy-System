// Event timeline: only what matters to a rescue (people found, capsize, rescue confirmed, stop/run,
// routes, signal lost/back, returning to the station). No packet-level or connection log.
// Two lists: the people (found, who looks after them, rescued) and the boats and commands, each on
// its own so a person found is not lost among the routes and signal changes.
// Kept in this browser (localStorage) across a reload, a day at most; the people the Worker still
// holds come back from its snapshot on any browser (logPast, features/victims.js).
import { state, on, isStale, isReturning } from '../state.js';
import { hm } from '../util/format.js';

const MAX_ROWS = 200;           // per list
const KEEP_MS = 24 * 3600e3;    // rows older than this are not brought back on a reload
const STORE_KEY = 'timeline.v1';
const KINDS = {
  victim:  { icon: 'ph-fill ph-person-simple-swim', tone: 'bad',    alert: true, people: true },
  rescue:  { icon: 'ph-fill ph-hand-heart',         tone: 'ok',                  people: true },
  claim:   { icon: 'ph ph-hand-pointing',            tone: 'info',                people: true },
  capsize: { icon: 'ph-fill ph-warning',            tone: 'bad',    alert: true },
  stop:    { icon: 'ph-fill ph-hand-palm',          tone: 'bad' },
  run:     { icon: 'ph-fill ph-play',               tone: 'ok' },
  idle:    { icon: 'ph-fill ph-pause-circle',       tone: 'accent' },
  route:   { icon: 'ph-fill ph-path',               tone: 'accent' },
  home:    { icon: 'ph ph-arrow-u-up-left',         tone: 'warn',   alert: true },
  lost:    { icon: 'ph ph-cell-signal-slash',       tone: 'warn',   alert: true },
  back:    { icon: 'ph ph-cell-signal-full',        tone: 'ok' },
  battery: { icon: 'ph-fill ph-battery-warning',    tone: 'bad',    alert: true },
  fleet:   { icon: 'ph-fill ph-boat',               tone: 'accent' },
  leader:  { icon: 'ph-fill ph-crown-simple',       tone: 'warn',   alert: true },
};
const GROUPS = {
  people: { empty: 'Chưa có ai được phát hiện. Người phao thấy, phao nào lo họ và người đã được cứu sẽ hiện ở đây.' },
  ops:    { empty: 'Chưa có sự kiện nào. Phao lật, mất tín hiệu, lệnh dừng, về bờ và gửi lộ trình sẽ hiện ở đây.' },
};
const groupOf = kind => (KINDS[kind].people ? 'people' : 'ops');

const $ = id => document.getElementById(id);
const rows = { people: [], ops: [] };   // newest last
const unseen = { people: 0, ops: 0 };
let shown = 'people';

const isVisible = g => !$('log-view').hidden && shown === g;
// The same event twice (live, then again from the Worker's snapshot after a reload): by the id the
// Worker's record gives it, else by kind, text and second
const keyOf = r => r.id || `${r.kind}|${r.text}|${Math.round(r.time.getTime() / 1000)}`;

function save() {
  try {
    const all = [...rows.people, ...rows.ops].map(r => ({ k: r.kind, x: r.text, t: r.time.getTime(), i: r.id }));
    localStorage.setItem(STORE_KEY, JSON.stringify(all));
  } catch { /* private window, storage full: the log lives on this page only */ }
}

function load() {
  let all = [];
  try { all = JSON.parse(localStorage.getItem(STORE_KEY) || '[]'); } catch { all = []; }
  const since = Date.now() - KEEP_MS;
  // Through add(): rows the Worker's snapshot brought in first are not doubled
  for (const r of Array.isArray(all) ? all : []) {
    if (r && KINDS[r.k] && typeof r.x === 'string' && Number.isFinite(r.t) && r.t >= since) {
      add(r.k, r.x, r.t, typeof r.i === 'string' ? r.i : undefined);
    }
  }
}

function render() {
  const list = $('timeline');
  list.replaceChildren();
  for (const b of document.querySelectorAll('#tl-groups button')) {
    const g = b.dataset.g;
    b.setAttribute('aria-pressed', String(g === shown));
    b.querySelector('.count').textContent = rows[g].length;
  }
  const shownRows = rows[shown];
  if (!shownRows.length) {
    const e = document.createElement('li');
    e.className = 'tl-empty';
    e.textContent = GROUPS[shown].empty;
    list.appendChild(e);
    return;
  }
  const frag = document.createDocumentFragment();
  for (let i = shownRows.length - 1; i >= 0; i--) {
    const r = shownRows[i];
    const k = KINDS[r.kind];
    const li = document.createElement('li');
    li.className = 'tl-row';
    li.dataset.tone = k.tone;
    const ic = document.createElement('span'); ic.className = 'tl-ic';
    ic.innerHTML = `<i class="${k.icon}" aria-hidden="true"></i>`;
    const msg = document.createElement('span'); msg.className = 'tl-msg'; msg.textContent = r.text;
    const t = document.createElement('time'); t.dateTime = r.time.toISOString(); t.textContent = hm(r.time);
    // A row from an earlier day shows its date too
    if (r.time.toDateString() !== new Date().toDateString()) t.textContent = `${r.time.getDate()}/${r.time.getMonth() + 1} ${hm(r.time)}`;
    li.append(ic, msg, t);
    frag.appendChild(li);
  }
  list.appendChild(frag);
}

function updateBadge() {
  const total = unseen.people + unseen.ops;
  const b = $('tab-log-badge');
  b.hidden = total === 0;
  b.textContent = total > 99 ? '99+' : String(total);
  for (const btn of document.querySelectorAll('#tl-groups button')) {
    const n = unseen[btn.dataset.g];
    btn.classList.toggle('has-new', n > 0);
  }
}

// One row, kept in time order; false when the same row is there already (a replay after a reload)
function add(kind, text, at, id) {
  if (!KINDS[kind]) return false;
  const g = groupOf(kind), r = { kind, text, time: new Date(at), id }, key = keyOf(r);
  const list = rows[g];
  if (list.some(x => keyOf(x) === key)) return false;
  let i = list.length;
  while (i > 0 && list[i - 1].time > r.time) i--;
  list.splice(i, 0, r);
  if (list.length > MAX_ROWS) list.splice(0, list.length - MAX_ROWS);
  return true;
}

// Something that just happened: counted as new while its list is not on screen. id: the event's own
// name in the Worker's records (a person, an alert), so its replay after a reload is not a second row.
export function log(kind, text, at = Date.now(), id = undefined) {
  if (!add(kind, text, at, id)) return;
  const g = groupOf(kind);
  if (KINDS[kind].alert && !isVisible(g)) { unseen[g]++; updateBadge(); }
  save();
  render();
}

// Something that happened before this page was opened (the Worker's snapshot): not counted as new
export function logPast(kind, text, at, id = undefined) {
  if (!Number.isFinite(at) || !add(kind, text, at, id)) return;
  save();
  render();
}

// The events tab opened: what is on screen is seen
export function clearUnseen() {
  unseen[shown] = 0;
  updateBadge();
}

export function initTimeline() {
  for (const b of document.querySelectorAll('#tl-groups button')) {
    b.addEventListener('click', () => { shown = b.dataset.g; clearUnseen(); render(); });
  }
  $('tl-clear').addEventListener('click', () => {
    if (!confirm('Xoá nhật ký sự kiện trên trình duyệt này? Người đang cần cứu vẫn hiện lại từ máy chủ.')) return;
    rows.people.length = 0; rows.ops.length = 0;
    unseen.people = 0; unseen.ops = 0;
    save(); updateBadge(); render();
  });
  render();

  on('buoy', ({ id, prev, b, snapshot }) => {
    if (snapshot || !prev) return;   // a replay on page load is not a new event
    if (!prev.returning && isReturning(b)) log('home', `${id} mất liên lạc với trạm bờ, đang tự quay về`);
    if (prev.returning && b.hasAttitude && !isReturning(b)) log('back', `${id} nghe lại trạm bờ, chạy tiếp lộ trình`);
    // Count went up: the buoy found one more person. Buoys sending detailed events are logged by victims.js
    if (Number.isInteger(prev.victims) && b.victims > prev.victims && !state.eventBuoys.has(id)) {
      log('victim', `${id} phát hiện người (lần ${b.victims})`);
    }
  });

  // Signal lost / back: compare once a second
  const last = new Map();
  setInterval(() => {
    for (const [id, b] of state.buoys) {
      const stale = isStale(b);
      const before = last.get(id);
      last.set(id, stale);
      if (before === undefined) continue;
      if (!before && stale) log('lost', `${id} mất tín hiệu`);
      else if (before && !stale) log('back', `${id} có tín hiệu trở lại`);
    }
  }, 1000);
}

// The rows kept in this browser, before anything can log (and save) a new one over them
load();
