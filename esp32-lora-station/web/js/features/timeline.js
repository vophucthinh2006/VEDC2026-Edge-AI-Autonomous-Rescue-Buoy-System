// Event timeline: only what matters to a rescue (people found, capsize, rescue confirmed, stop/run,
// routes, signal lost/back, returning to the station). No packet-level or connection log.
import { state, on, isStale, isReturning } from '../state.js';
import { hm } from '../util/format.js';

const MAX_ROWS = 200;
const KINDS = {
  victim:  { icon: 'ph-fill ph-person-simple-swim', tone: 'bad',    alert: true },
  capsize: { icon: 'ph-fill ph-warning',            tone: 'bad',    alert: true },
  rescue:  { icon: 'ph-fill ph-hand-heart',         tone: 'ok' },
  stop:    { icon: 'ph-fill ph-hand-palm',          tone: 'bad' },
  run:     { icon: 'ph-fill ph-play',               tone: 'ok' },
  idle:    { icon: 'ph-fill ph-pause-circle',       tone: 'accent' },
  route:   { icon: 'ph-fill ph-path',               tone: 'accent' },
  home:    { icon: 'ph ph-arrow-u-up-left',         tone: 'warn',   alert: true },
  lost:    { icon: 'ph ph-cell-signal-slash',       tone: 'warn',   alert: true },
  back:    { icon: 'ph ph-cell-signal-full',        tone: 'ok' },
};

const $ = id => document.getElementById(id);
const rows = [];   // newest last
let unseen = 0;

const isVisible = () => !$('log-view').hidden;

function render() {
  const list = $('timeline');
  list.replaceChildren();
  if (!rows.length) {
    const e = document.createElement('li');
    e.className = 'tl-empty';
    e.textContent = 'Chưa có sự kiện nào. Phát hiện người, phao lật, lệnh dừng và gửi lộ trình sẽ hiện ở đây.';
    list.appendChild(e);
    return;
  }
  const frag = document.createDocumentFragment();
  for (let i = rows.length - 1; i >= 0; i--) {
    const r = rows[i];
    const k = KINDS[r.kind];
    const li = document.createElement('li');
    li.className = 'tl-row';
    li.dataset.tone = k.tone;
    const ic = document.createElement('span'); ic.className = 'tl-ic';
    ic.innerHTML = `<i class="${k.icon}" aria-hidden="true"></i>`;
    const msg = document.createElement('span'); msg.className = 'tl-msg'; msg.textContent = r.text;
    const t = document.createElement('time'); t.dateTime = r.time.toISOString(); t.textContent = hm(r.time);
    li.append(ic, msg, t);
    frag.appendChild(li);
  }
  list.appendChild(frag);
}

function updateBadge() {
  const b = $('tab-log-badge');
  b.hidden = unseen === 0;
  b.textContent = unseen > 99 ? '99+' : String(unseen);
}

export function log(kind, text, at = Date.now()) {
  if (!KINDS[kind]) return;
  rows.push({ kind, text, time: new Date(at) });
  if (rows.length > MAX_ROWS) rows.splice(0, rows.length - MAX_ROWS);
  if (KINDS[kind].alert && !isVisible()) { unseen++; updateBadge(); }
  render();
}

export function clearUnseen() { unseen = 0; updateBadge(); }

export function initTimeline() {
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
