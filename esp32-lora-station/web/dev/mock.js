// Giả lập máy chủ để xem thử giao diện trên máy: mở index.html?mock (admin) hoặc ?mock=guest (khách).
// Có phao chuyển động, RSSI dao động, phao mất GPS, phao mất tín hiệu và lịch sử 15 phút.
// KHÔNG được deploy (.assetsignore).
(() => {
  const BASE = { lat: 10.7626, lon: 106.6602 };
  const now = () => Date.now();
  const noise = (a) => (Math.random() - 0.5) * a;

  // Quỹ đạo tròn quanh một tâm; rssi giảm dần theo khoảng cách tới "trạm"
  const buoys = [
    { id: 'PHAO-01', cx: 10.7626, cy: 106.6602, r: 0.004, w: 0.05, ph: 0 },
    { id: 'PHAO-02', cx: 10.7700, cy: 106.6690, r: 0.006, w: -0.035, ph: 2 },
    { id: 'PHAO-03', cx: 10.7560, cy: 106.6540, r: 0.003, w: 0.07, ph: 4, dies: true },
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
  const hold = { want: false, have: false, applyAt: 0 };   // PHAO-01: đang được chạy
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
          send({ id: b.id, fix: 1, lat: last.lat, lon: last.lon, rssi: last.rssi, snr: last.snr, raw: `TRIGGER,0,${last.lat.toFixed(6)},${last.lon.toFixed(6)}`, age_s: 4 });
          send({ type: 'history', id: b.id, points: h });
        }
        // Trạm bờ có GPS, báo vị trí cố định
        send({ type: 'station', fix: 1, lat: 10.7626, lon: 106.6602, sats: 9, hdop: 0.9, age_s: 3 });
        // Đã có hai người từ trước (mức an toàn và trung bình), đèn 1 của PHAO-01 đang bật
        send({ type: 'victims', alerts: [], victims: [victim('PHAO-01', 0, 10.7652, 106.6589, 0.71, 400), victim('PHAO-02', 1, 10.7729, 106.6672, 0.64, 250)] });
        send({ type: 'lights', id: 'PHAO-01', want: lights.want, have: lights.have, by: 'mliotredgescue@gmail.com', at: now() - 600000 });
        window.mockServer = {
          setLights(id, mask) {   // phao bật/tắt relay sau khoảng 2 beacon
            if (id !== 'PHAO-01') { send({ type: 'lights', id, want: mask, have: null }); return; }
            lights.want = mask; lights.applyAt = n + 2;
            send({ type: 'lights', id, want: lights.want, have: lights.have, by: 'mock@local', at: now() });
          },
          setHold(id, stop) {     // phao khóa / mở motor sau khoảng 2 beacon
            if (id !== 'PHAO-01') { send({ type: 'hold', id, want: stop, have: null }); return; }
            hold.want = stop; hold.applyAt = n + 2;
            send({ type: 'hold', id, want: hold.want, have: hold.have, by: 'mock@local', at: now() });
          },
          clearVictims() { send({ type: 'victims', victims: [], alerts: [] }); },
        };
        send({
          type: 'mission', rev: 1, by: 'mliotredgescue@gmail.com', spacing: 10, angle: null, speed: 1.5, poly: [],
          waypoints: [{ lat: 10.7640, lon: 106.6620 }, { lat: 10.7665, lon: 106.6635 }, { lat: 10.7680, lon: 106.6655 }, { lat: 10.7672, lon: 106.6680 }],
        });

        // Gói mới mỗi 3 giây
        let n = 0;
        let critical = null;
        this._timers.push(setInterval(() => {
          const t = (now() - started) / 1000 + 900;
          n++;
          const p1 = pos(buoys[0], t);
          if (lights.have !== lights.want && n >= lights.applyAt) {
            lights.have = lights.want;
            send({ type: 'lights', id: 'PHAO-01', want: lights.want, have: lights.have, at: now() });
          }
          if (hold.have !== hold.want && n >= hold.applyAt) {
            hold.have = hold.want;
            send({ type: 'hold', id: 'PHAO-01', want: hold.want, have: hold.have, at: now() });
          }
          // Sự kiện: người mức cao (~9 s), nguy kịch (~15 s), lật phao (~18 s), thấy lại người nguy kịch (~24 s),
          // nút xác nhận (~27 s), người khi phao chưa có GPS (~33 s)
          if (n === 3) send({ type: 'victim', new: true, victim: victim('PHAO-02', 2, 10.7712, 106.6721, 0.82, 0) });
          if (n === 5) send({ type: 'victim', new: true, victim: critical = victim('PHAO-01', 3, p1.lat + 0.0002, p1.lon - 0.0001, 0.88, 0) });
          if (n === 6) send({ type: 'alert', alert: { type: 'T', buoy: 'PHAO-01', seq: 7, at: now(), lat: p1.lat, lon: p1.lon, roll: 118.5, pitch: 12.3 } });
          if (n === 8 && critical) send({ type: 'victim', new: false, victim: { ...critical, conf: 0.93, last_at: now() } });
          if (n === 9) send({ type: 'alert', alert: { type: 'C', buoy: 'PHAO-01', seq: 8, at: now(), lat: p1.lat, lon: p1.lon, presses: 1 } });
          if (n === 11) send({ type: 'alert', alert: { type: 'V', buoy: 'PHAO-02', seq: 3, at: now(), lat: null, lon: null, risk: 2, conf: 0.77, count: 2 } });
          for (const b of buoys) {
            if (b.dies && n > 12) continue;                      // PHAO-03 im lặng sau ~36 giây, để thấy trạng thái mất tín hiệu
            const p = pos(b, t);
            const rssi = rssiAt(p), snr = Math.round((8 + noise(8)) * 10) / 10;
            // PHAO-02 quay về trạm (mode R) từng đợt 30 giây
            const mode = b.id === 'PHAO-02' && n % 20 >= 10 ? 'R' : 'A';
            if (b.id === 'PHAO-02' && n % 10 >= 6) {             // PHAO-02 mất GPS từng đợt
              send({ id: b.id, fix: 0, rssi, snr, raw: `TRIGGER,${n},NO_FIX`, roll: 5 * Math.sin(t), pitch: 2 * Math.cos(t), yaw: (n * 7) % 360, target_yaw: 90, mode, imu_ok: 1, calib: 3, seq: n, battery_pct: 64 });
            } else if (b.id === 'PHAO-01') {
              // Mũi lệch hướng đi 20 độ (dòng nước đẩy ngang) để thấy hai mũi tên khác nhau
              const course = courseAt(b, t);
              send({ id: b.id, fix: 1, lat: p.lat, lon: p.lon, rssi, snr, raw: `TRIGGER,${n},${p.lat.toFixed(6)},${p.lon.toFixed(6)},sats=8,hdop=0.9`, roll: 5 * Math.sin(t), pitch: 2 * Math.cos(t),
                     yaw: Math.round((course + 20) % 360 * 10) / 10, target_yaw: course, mode, imu_ok: 1, calib: 3, seq: n, age_s: 0,
                     course, speed: Math.round((1.4 + noise(0.3)) * 100) / 100, battery_pct: Math.max(5, 87 - Math.floor(n / 20)), lights: lights.have, hold: hold.have ? 1 : 0,
                     victims: Math.min(2, Math.floor(n / 4)) });   // PHAO-01 báo phát hiện người sau ~12 và ~24 giây
            } else {
              send({ id: b.id, fix: 1, lat: p.lat, lon: p.lon, rssi, snr, raw: `TRIGGER,${n},${p.lat.toFixed(6)},${p.lon.toFixed(6)},sats=8,hdop=0.9`, roll: 5 * Math.sin(t), pitch: 2 * Math.cos(t), yaw: (n * 7) % 360, target_yaw: 90, mode, imu_ok: 1, calib: 3, seq: n, age_s: 0,
                     ...(b.id === 'PHAO-02' ? { battery_pct: 64 } : {}) });
            }
          }
          if (n % 3 === 0) send({ type: 'station', fix: 1, lat: BASE.lat, lon: BASE.lon, sats: 9, hdop: 0.9, age_s: 0 });
        }, 3000));

        // SIM-01: phao báo qua cầu nối MAVLink (không RSSI/SNR, có nhóm nav), 2 gói/giây.
        // Chạy vòng tròn, đổi chế độ HOLD -> GUIDED -> AUTO -> RTL để thấy HUD và nhật ký.
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
