// Giả lập máy chủ để xem thử trạm điều hành trên máy: mở admin.html?mock (admin) hoặc ?mock=guest (khách).
// Covers every status label: PHAO-01 patrols its route, PHAO-02 returns to the station in bursts and
// capsizes after ~18 s, PHAO-03 waits for a route then goes silent after ~36 s, PHAO-04 is stopped with
// a weak link and a low battery, SIM-01 is a Pixhawk over MAVLink. Address search is answered here too.
// KHÔNG được deploy (.assetsignore).
(() => {
  const BASE = { lat: 10.7626, lon: 106.6602 };
  const now = () => Date.now();
  const noise = (a) => (Math.random() - 0.5) * a;

  // Quỹ đạo tròn quanh một tâm; rssi giảm dần theo khoảng cách tới "trạm"
  const buoys = [
    { id: 'PHAO-01', cx: 10.7626, cy: 106.6602, r: 0.004, w: 0.05, ph: 0 },
    { id: 'PHAO-02', cx: 10.7700, cy: 106.6690, r: 0.006, w: -0.035, ph: 2 },
    { id: 'PHAO-03', cx: 10.7560, cy: 106.6540, r: 0.0006, w: 0.07, ph: 4, dies: true },
    { id: 'PHAO-04', cx: 10.7588, cy: 106.6668, r: 0, w: 0, ph: 0 },
  ];
  const pos = (b, t) => ({ lat: b.cx + b.r * Math.sin(b.w * t + b.ph), lon: b.cy + b.r * Math.cos(b.w * t + b.ph) });
  // Hướng đi thực (độ) theo đạo hàm quỹ đạo tròn
  const courseAt = (b, t) => {
    const a = b.w * t + b.ph;
    const dLat = b.r * b.w * Math.cos(a), dLon = -b.r * b.w * Math.sin(a) * Math.cos(b.cx * Math.PI / 180);
    return Math.round(((Math.atan2(dLon, dLat) * 180 / Math.PI) + 360) % 360 * 10) / 10;
  };

  // Người phát hiện, cảnh báo, đèn: giống bản tin máy chủ thật (đã bỏ bản phao gửi lặp, gộp phát hiện cách nhau dưới 5 m)
  let vid = 0;
  const victim = (buoy, risk, lat, lon, conf, agoS) => ({ vid: ++vid, buoy, lat, lon, risk, conf, first_at: now() - agoS * 1000, last_at: now() - agoS * 400 });
  const lights = { want: 1, have: 1, applyAt: 0 };   // PHAO-01: đèn 1 đang bật
  const holds = { 'PHAO-01': { want: false, have: false, applyAt: 0 }, 'PHAO-04': { want: true, have: true, applyAt: 0 } };
  // Route held by each buoy: { rev, n, have, pi }. PHAO-01 already runs rev 1, PHAO-04 was stopped on rev 1
  let dl = { rev: 1, id: 'PHAO-01', loop: false, n: 4, mission_rev: 1, by: 'mliotredgescue@gmail.com', at: now() - 600000, delivered: now() - 590000 };
  const routes = { 'PHAO-01': { rev: 1, n: 4, have: 4, pi: 1 }, 'PHAO-04': { rev: 1, n: 4, have: 4, pi: 1 } };
  const rssiAt = (p) => {
    const d = Math.hypot((p.lat - BASE.lat) * 111000, (p.lon - BASE.lon) * 109000);
    return Math.round(-68 - d / 55 + noise(5));
  };

  // Lịch sử 15 phút, mỗi 5 giây một mẫu
  const started = now();
  const history = id => {
    const b = buoys.find(x => x.id === id);
    const out = [];
    for (let s = 900; s > 0; s -= 5) {
      const t = 900 - s;   // liền mạch với thời gian thực (t = 900 + số giây đã chạy)
      const p = pos(b, t);
      out.push({ t: now() - s * 1000, lat: p.lat, lon: p.lon, rssi: rssiAt(p), snr: Math.round((8 + noise(8)) * 10) / 10 });
    }
    return out;
  };

  // Address search: a few places around the demo area, and one far away (no buoy in range)
  const PLACES = [
    { name: 'Chợ Lớn, Phường 11, Quận 5, Thành phố Hồ Chí Minh, Việt Nam', lat: 10.7536, lon: 106.6548 },
    { name: 'Công viên Lê Thị Riêng, Phường 15, Quận 10, Thành phố Hồ Chí Minh, Việt Nam', lat: 10.7861, lon: 106.6655 },
    { name: 'Nhà văn hoá Thanh niên, Bến Nghé, Quận 1, Thành phố Hồ Chí Minh, Việt Nam', lat: 10.7790, lon: 106.6986 },
    { name: 'Bến Ninh Kiều, Ninh Kiều, Cần Thơ, Việt Nam', lat: 10.0345, lon: 105.7884 },
  ];
  const fold = s => s.normalize('NFD').replace(/[̀-ͯ]/g, '').replace(/đ/gi, 'd').toLowerCase();
  const realFetch = window.fetch.bind(window);
  let lastGeocode = 0;
  window.fetch = async (url, init) => {
    const u = new URL(String(url), location.href);
    if (u.pathname !== '/api/geocode') return realFetch(url, init);
    await new Promise(r => setTimeout(r, 450));
    const body = (o, status = 200) => new Response(JSON.stringify(o), { status, headers: { 'content-type': 'application/json' } });
    if (now() - lastGeocode < 1000) return body({ error: 'rate', retry_after_ms: 1000 - (now() - lastGeocode) }, 429);
    lastGeocode = now();
    const q = fold(u.searchParams.get('q') || '');
    const words = q.split(' ').filter(Boolean);
    const hits = PLACES.filter(p => words.every(w => fold(p.name).includes(w)));
    return body({ results: hits.length || q.includes('khong co') ? hits : PLACES.slice(0, 3) });
  };

  window.WebSocket = class {
    constructor() {
      this._timers = [];
      setTimeout(() => {
        this.onopen && this.onopen();
        const send = o => this.onmessage && this.onmessage({ data: JSON.stringify(o) });

        // Lịch sử + trạng thái hiện tại + lộ trình, giống thứ tự máy chủ thật
        for (const b of buoys) {
          const h = history(b.id);
          const last = h[h.length - 1];
          send({ id: b.id, fix: 1, lat: last.lat, lon: last.lon, rssi: last.rssi, snr: last.snr, raw: '', age_s: 4,
                 ...(routes[b.id] ? { route: { rev: routes[b.id].rev, have: routes[b.id].have, pi: routes[b.id].pi } } : {}),
                 ...(holds[b.id] ? { hold: holds[b.id].have ? 1 : 0 } : {}) });
          send({ type: 'history', id: b.id, points: h });
        }
        // Trạm bờ có GPS, báo vị trí cố định
        send({ type: 'station', fix: 1, lat: 10.7626, lon: 106.6602, sats: 9, hdop: 0.9, age_s: 3 });
        send({
          type: 'mission', rev: 1, by: 'mliotredgescue@gmail.com', spacing: 10, angle: null, speed: 1.5, poly: [],
          waypoints: [{ lat: 10.7640, lon: 106.6620 }, { lat: 10.7665, lon: 106.6635 }, { lat: 10.7680, lon: 106.6655 }, { lat: 10.7672, lon: 106.6680 }],
        });
        send({ type: 'downlink', ...dl });
        // Đã có hai người từ trước (mức an toàn và trung bình), đèn 1 của PHAO-01 đang bật
        send({ type: 'victims', alerts: [], victims: [victim('PHAO-01', 0, 10.7652, 106.6589, 0.71, 400), victim('PHAO-02', 1, 10.7729, 106.6672, 0.64, 250)] });
        send({ type: 'lights', id: 'PHAO-01', want: lights.want, have: lights.have, by: 'mliotredgescue@gmail.com', at: now() - 600000 });
        for (const [id, h] of Object.entries(holds)) send({ type: 'hold', id, want: h.want, have: h.have, at: now() - 300000 });
        window.mockServer = {
          setLights(id, mask) {   // phao bật/tắt relay sau khoảng 2 beacon
            if (id !== 'PHAO-01') { send({ type: 'lights', id, want: mask, have: null }); return; }
            lights.want = mask; lights.applyAt = n + 2;
            send({ type: 'lights', id, want: lights.want, have: lights.have, by: 'mock@local', at: now() });
          },
          setHold(id, stop) {     // phao khóa / mở motor sau khoảng 2 beacon
            const h = holds[id];
            if (!h) { send({ type: 'hold', id, want: stop, have: null }); return; }
            h.want = stop; h.applyAt = n + 2;
            send({ type: 'hold', id, want: h.want, have: h.have, by: 'mock@local', at: now() });
          },
          // "Gửi tới phao": the buoy takes two points per beacon, then hands the route to the Pi
          sendRoute(id, count, loop, missionRev) {
            dl = { rev: dl.rev + 1, id, loop, n: count, mission_rev: missionRev, by: 'mock@local', at: now(), delivered: null };
            send({ type: 'downlink', ...dl });
            routes[id] = { rev: dl.rev, n: count, have: 0, pi: 0 };
          },
          clearVictims() { send({ type: 'victims', victims: [], alerts: [] }); },
        };

        // Gói mới mỗi 3 giây
        let n = 0;
        let critical = null;
        this._timers.push(setInterval(() => {
          const t = (now() - started) / 1000 + 900;
          n++;
          const p1 = pos(buoys[0], t), p2 = pos(buoys[1], t);
          if (lights.have !== lights.want && n >= lights.applyAt) {
            lights.have = lights.want;
            send({ type: 'lights', id: 'PHAO-01', want: lights.want, have: lights.have, at: now() });
          }
          for (const [id, h] of Object.entries(holds)) {
            if (h.have !== h.want && n >= h.applyAt) { h.have = h.want; send({ type: 'hold', id, want: h.want, have: h.have, at: now() }); }
          }
          for (const r of Object.values(routes)) {
            if (r.have < r.n) r.have = Math.min(r.n, r.have + 2);
            else if (!r.pi) { r.pi = 1; if (dl.rev === r.rev) dl.delivered = now(); }
          }
          // Sự kiện: người mức cao (~9 s), nguy kịch (~15 s), PHAO-02 lật (~18 s), thấy lại người nguy kịch (~24 s),
          // nút xác nhận (~27 s), người khi phao chưa có GPS (~33 s)
          if (n === 3) send({ type: 'victim', new: true, victim: victim('PHAO-02', 2, 10.7712, 106.6721, 0.82, 0) });
          if (n === 5) send({ type: 'victim', new: true, victim: critical = victim('PHAO-01', 3, p1.lat + 0.0002, p1.lon - 0.0001, 0.88, 0) });
          if (n === 6) send({ type: 'alert', alert: { type: 'T', buoy: 'PHAO-02', seq: 7, at: now(), lat: p2.lat, lon: p2.lon, roll: 118.5, pitch: 12.3 } });
          if (n === 8 && critical) send({ type: 'victim', new: false, victim: { ...critical, conf: 0.93, last_at: now() } });
          if (n === 9) send({ type: 'alert', alert: { type: 'C', buoy: 'PHAO-01', seq: 8, at: now(), lat: p1.lat, lon: p1.lon, presses: 1 } });
          if (n === 11) send({ type: 'alert', alert: { type: 'V', buoy: 'PHAO-02', seq: 3, at: now(), lat: null, lon: null, risk: 2, conf: 0.77, count: 2 } });
          for (const b of buoys) {
            if (b.dies && n > 12) continue;                      // PHAO-03 im lặng sau ~36 giây, để thấy trạng thái mất tín hiệu
            const p = pos(b, t);
            const snr = Math.round((8 + noise(8)) * 10) / 10;
            const r = routes[b.id];
            const route = r ? { route: { rev: r.rev, have: r.have, pi: r.pi } } : {};
            const att = { roll: 5 * Math.sin(t), pitch: 2 * Math.cos(t), imu_ok: 1, calib: 3, seq: n };
            if (b.id === 'PHAO-01') {
              // Mũi lệch hướng đi 20 độ (dòng nước đẩy ngang) để thấy hai mũi tên khác nhau
              const course = courseAt(b, t);
              send({ id: b.id, fix: 1, lat: p.lat, lon: p.lon, rssi: rssiAt(p), snr, raw: '', ...att,
                     yaw: Math.round((course + 20) % 360 * 10) / 10, target_yaw: course, mode: holds[b.id].have ? 'S' : 'A', age_s: 0,
                     course, speed: holds[b.id].have ? 0 : Math.round((0.8 + noise(0.2)) * 100) / 100, battery_pct: Math.max(5, 87 - Math.floor(n / 20)), lights: lights.have,
                     hold: holds[b.id].have ? 1 : 0, ...route, victims: Math.min(2, Math.floor(n / 4)) });
            } else if (b.id === 'PHAO-02') {
              // PHAO-02 quay về trạm (mode R) từng đợt 30 giây, mất GPS từng đợt
              const mode = n % 20 >= 10 ? 'R' : 'A';
              const fix = n % 10 >= 6 ? 0 : 1;
              send({ id: b.id, fix, ...(fix ? { lat: p.lat, lon: p.lon } : {}), rssi: rssiAt(p), snr, raw: '', ...att, yaw: (n * 7) % 360, target_yaw: 90, mode, age_s: 0,
                     speed: Math.round((1.1 + noise(0.2)) * 100) / 100, course: courseAt(b, t), battery_pct: 64, route: { rev: 1, have: 4, pi: 1 } });
            } else if (b.id === 'PHAO-03') {
              // Stands still (mode S) until a route sent from the dashboard reaches its Pi, then is driven (mode A)
              const driven = !!(r && r.pi && r.n);
              send({ id: b.id, fix: 1, lat: p.lat, lon: p.lon, rssi: rssiAt(p), snr, raw: '', ...att, yaw: 40, target_yaw: driven ? 40 : -1,
                     mode: driven ? 'A' : 'S', age_s: 0, speed: driven ? 0.9 : 0.05, battery_pct: 92, ...route });
            } else {
              send({ id: b.id, fix: 1, lat: p.lat, lon: p.lon, rssi: -112 + Math.round(noise(3)), snr: -2.5, raw: '', ...att, yaw: 300, target_yaw: -1,
                     mode: holds[b.id].have ? 'S' : 'A', age_s: 0, speed: 0, battery_pct: 17, hold: holds[b.id].have ? 1 : 0, ...route });
            }
          }
          if (n % 3 === 0) send({ type: 'station', fix: 1, lat: BASE.lat, lon: BASE.lon, sats: 9, hdop: 0.9, age_s: 0 });
        }, 3000));

        // SIM-01: phao báo qua cầu nối MAVLink (không RSSI/SNR, có nhóm nav), 2 gói/giây.
        // Chạy vòng tròn, đổi chế độ HOLD -> GUIDED -> AUTO -> RTL.
        const modes = ['HOLD', 'GUIDED', 'AUTO', 'RTL'];
        let k = 0;
        this._timers.push(setInterval(() => {
          const t = (now() - started) / 1000;
          k++;
          const mode = modes[Math.floor(t / 20) % modes.length];
          const armed = mode !== 'HOLD';
          const yaw = ((t * 6) % 360 + 360) % 360;
          const target = armed ? (yaw + 25 * Math.sin(t / 4) + 360) % 360 : -1;
          const p = { lat: 10.7600 + 0.0012 * Math.sin(t / 9.5), lon: 106.6560 + 0.0012 * Math.cos(t / 9.5) };
          send({
            id: 'SIM-01', link: 'mavlink', fix: 1, lat: p.lat, lon: p.lon, raw: `MAV ${mode}`,
            roll: 6 * Math.sin(t * 1.3), pitch: 3 * Math.cos(t * 0.9), yaw: Math.round(yaw * 10) / 10,
            target_yaw: Math.round(target * 10) / 10, mode: armed ? 'A' : 'S', imu_ok: 1, calib: 3, seq: k,
            nav: { armed: armed ? 1 : 0, mode_name: mode, gs: armed ? 1.4 + 0.3 * Math.sin(t) : 0.05,
                   thr: armed ? Math.round(45 + 30 * Math.sin(t / 3)) : 0, wp_dist: armed ? Math.max(0, 40 - (t % 20) * 2) : -1,
                   wp_seq: Math.floor(t / 20) % 4, batt_v: 12.6 - t * 0.002, batt_pct: Math.max(0, 100 - Math.round(t / 6)),
                   sats: 10, hdop: 1.2 },
            age_s: 0,
          });
        }, 500));
      }, 300);
    }
    close() { this._timers.forEach(clearInterval); this.onclose && this.onclose(); }
  };
})();
