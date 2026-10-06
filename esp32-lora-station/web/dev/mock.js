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
        send({
          type: 'mission', rev: 1, by: 'mliotredgescue@gmail.com', spacing: 10, angle: null, speed: 1.5, poly: [],
          waypoints: [{ lat: 10.7640, lon: 106.6620 }, { lat: 10.7665, lon: 106.6635 }, { lat: 10.7680, lon: 106.6655 }, { lat: 10.7672, lon: 106.6680 }],
        });

        // Gói mới mỗi 3 giây
        let n = 0;
        this._timers.push(setInterval(() => {
          const t = (now() - started) / 1000 + 900;
          n++;
          for (const b of buoys) {
            if (b.dies && n > 12) continue;                      // PHAO-03 im lặng sau ~36 giây, để thấy trạng thái mất tín hiệu
            const p = pos(b, t);
            const rssi = rssiAt(p), snr = Math.round((8 + noise(8)) * 10) / 10;
            if (b.id === 'PHAO-02' && n % 10 >= 6) {             // PHAO-02 mất GPS từng đợt
              send({ id: b.id, fix: 0, rssi, snr, raw: `TRIGGER,${n},NO_FIX`, roll: 5 * Math.sin(t), pitch: 2 * Math.cos(t), yaw: (n * 7) % 360, target_yaw: 90, mode: 'A', imu_ok: 1, calib: 3, seq: n });
            } else {
              send({ id: b.id, fix: 1, lat: p.lat, lon: p.lon, rssi, snr, raw: `TRIGGER,${n},${p.lat.toFixed(6)},${p.lon.toFixed(6)},sats=8,hdop=0.9`, roll: 5 * Math.sin(t), pitch: 2 * Math.cos(t), yaw: (n * 7) % 360, target_yaw: 90, mode: 'A', imu_ok: 1, calib: 3, seq: n, age_s: 0,
                     ...(b.id === 'PHAO-01' ? { victims: Math.min(2, Math.floor(n / 4)) } : {}) });   // PHAO-01 báo phát hiện người sau ~12 và ~24 giây
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
