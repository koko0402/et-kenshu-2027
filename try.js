/*
 * 第11回の「ブラウザで試す」。シミュレータ（C）の前半と同じ動きを JavaScript で書き直したもの。
 * コースの形は course.js（make_site.py がシミュレータを走らせて書き出す）の COURSE を使う。
 * world.c の数字を変えたら、ここも合わせること。
 */
(function () {
  "use strict";
  var C = window.COURSE;
  var canvas = document.getElementById("try-canvas");
  if (!C || !canvas) return;
  var chart = document.getElementById("try-chart");
  var g = canvas.getContext("2d");
  var gc = chart.getContext("2d");
  var pts = C.lines[0].concat(C.lines[1]);

  var TARGET = 47, CTRL_MS = 10, DEG_PER_POWER = 9, MOTOR_TAU = 0.06;
  var WHEEL_R = 50, TREAD = 140, SENSOR_AHEAD = 70;
  var REAR = C.robot[0], FRONT = C.robot[1], HALF_W = C.robot[2];
  var lap = C.gates[C.gates.length - 1];

  function random(seed) {
    var s = (seed * 2654435761) >>> 0 || 1;
    return function () {
      s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0;
      return s / 4294967296;
    };
  }

  function sense(x, y, rnd) {
    var best = 1e9, blue = 0;
    for (var i = 0; i < pts.length; i++) {
      var d = Math.hypot(pts[i][0] - x, pts[i][1] - y);
      if (d < best) { best = d; blue = pts[i][2]; }
    }
    var cover = Math.min(1, Math.max(0, (10 + 8 - best) / 16));
    var lineReflect = blue ? 35 : 5;
    return { reflect: Math.round(90 + (lineReflect - 90) * cover + (rnd() * 2 - 1) * 2), dist: best };
  }

  function side(o, a, b) { return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]); }
  function crosses(p1, p2, q1, q2) {
    return side(q1, q2, p1) * side(q1, q2, p2) < 0 && side(p1, p2, q1) * side(p1, p2, q2) < 0;
  }
  function clampPower(p, gain, dead) {
    p = Math.max(-100, Math.min(100, p));
    return Math.abs(p) < dead ? 0 : p * DEG_PER_POWER * gain;
  }

  function run(opt) {
    var rnd = random(opt.seed + 7);
    var gainL = 1, gainR = 1, dead = 5;
    if (opt.seed) { gainL = 0.96 + 0.08 * rnd(); gainR = 0.96 + 0.08 * rnd(); dead = 4 + 4 * rnd(); }
    var x = C.start[0], y = C.start[1], h = C.start[2] * Math.PI / 180;
    var vl = 0, vr = 0, lp = 0, rp = 0, prev = 0, farMs = 0, end = 40000;
    var r = { path: [], errors: [], passed: C.gates.map(function () { return false; }), lapMs: -1, pole: false, lost: false };

    for (var t = 0; t < end; t++) {
      if (t % CTRL_MS === 0) {
        var s = sense(x + SENSOR_AHEAD * Math.cos(h), y + SENSOR_AHEAD * Math.sin(h), rnd);
        var e = s.reflect - TARGET;
        var turn = opt.mode === "onoff" ? (e > 0 ? 20 : -20) : opt.kp * e + opt.kd * (e - prev);
        prev = e;
        lp = Math.round(opt.speed + turn);
        rp = Math.round(opt.speed - turn);
        if (t < 10000) r.errors.push(e);
        farMs = s.dist > 250 ? farMs + CTRL_MS : 0;
        if (farMs > 1500) { r.lost = true; break; }
      }
      vl += (clampPower(lp, gainL, dead) - vl) * 0.001 / MOTOR_TAU;
      vr += (clampPower(rp, gainR, dead) - vr) * 0.001 / MOTOR_TAU;
      var sl = vl * 0.001 * Math.PI / 180 * WHEEL_R, sr = vr * 0.001 * Math.PI / 180 * WHEEL_R;
      var f0 = [x + FRONT * Math.cos(h), y + FRONT * Math.sin(h)];
      var dh = (sr - sl) / TREAD, mid = h + dh / 2;
      x += (sl + sr) / 2 * Math.cos(mid);
      y += (sl + sr) / 2 * Math.sin(mid);
      h += dh;
      var f1 = [x + FRONT * Math.cos(h), y + FRONT * Math.sin(h)];

      for (var k = 0; k < C.gates.length && r.lapMs < 0; k++) {
        var gt = C.gates[k];
        var fx = gt.a[1] - gt.b[1], fy = gt.b[0] - gt.a[0];      /* ゲートを通る向き */
        if (!r.passed[k] && crosses(f0, f1, gt.a, gt.b) && (f1[0] - f0[0]) * fx + (f1[1] - f0[1]) * fy > 0) {
          r.passed[k] = true;
          if (k === C.gates.length - 1) { r.lapMs = t; end = t + 600; }
        }
      }
      [lap.a, lap.b].forEach(function (p) {
        var dx = p[0] - x, dy = p[1] - y;
        var u = dx * Math.cos(h) + dy * Math.sin(h), v = -dx * Math.sin(h) + dy * Math.cos(h);
        if (u > -REAR - 10 && u < FRONT + 10 && Math.abs(v) < HALF_W + 10) r.pole = true;
      });
      if (t % 20 === 0) r.path.push([x, y]);
      if (x < 0 || y < 0 || x > C.field[0] || y > C.field[1]) { r.lost = true; break; }
    }
    r.x = x; r.y = y; r.h = h;
    return r;
  }

  function color(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

  function draw(r) {
    var S = canvas.width / C.field[0], H = canvas.height;
    var X = function (v) { return v * S; }, Y = function (v) { return H - v * S; };
    g.fillStyle = color("--try-mat"); g.fillRect(0, 0, canvas.width, H);
    g.fillStyle = color("--try-floor");
    g.fillRect(X(150), Y(C.field[1] - 150), (C.field[0] - 300) * S, (C.field[1] - 300) * S);
    g.lineCap = "round"; g.lineWidth = 20 * S;
    C.lines.forEach(function (line) {
      for (var i = 1; i < line.length; i++) {
        g.strokeStyle = line[i][2] ? "#2f6fd0" : "#111";
        g.beginPath(); g.moveTo(X(line[i - 1][0]), Y(line[i - 1][1])); g.lineTo(X(line[i][0]), Y(line[i][1])); g.stroke();
      }
    });
    var gr = C.garage;
    g.strokeStyle = "#888"; g.lineWidth = 3;
    g.strokeRect(X(gr[0]), Y(gr[3]), (gr[2] - gr[0]) * S, (gr[3] - gr[1]) * S);
    C.gates.forEach(function (gt, i) {
      var isLap = i === C.gates.length - 1;
      g.strokeStyle = isLap ? "#c0392b" : "#2e7d32";
      g.lineWidth = r.passed[i] ? 4 : 2;
      g.setLineDash([8, 5]); g.beginPath();
      g.moveTo(X(gt.a[0]), Y(gt.a[1])); g.lineTo(X(gt.b[0]), Y(gt.b[1])); g.stroke(); g.setLineDash([]);
      g.fillStyle = g.strokeStyle; g.font = "bold 20px sans-serif";
      g.fillText(gt.name, X(gt.a[0]) + 6, Y(gt.a[1]) - 6);
    });
    g.strokeStyle = "#e08a00"; g.lineWidth = 3; g.beginPath();
    r.path.forEach(function (p, i) { if (i) g.lineTo(X(p[0]), Y(p[1])); else g.moveTo(X(p[0]), Y(p[1])); });
    g.stroke();
    g.save(); g.translate(X(r.x), Y(r.y)); g.rotate(-r.h);
    g.fillStyle = "rgba(47,111,208,.35)"; g.strokeStyle = "#2f6fd0"; g.lineWidth = 2;
    g.fillRect(-REAR * S, -HALF_W * S, (REAR + FRONT) * S, 2 * HALF_W * S);
    g.strokeRect(-REAR * S, -HALF_W * S, (REAR + FRONT) * S, 2 * HALF_W * S);
    g.restore();

    var W = chart.width, CH = chart.height;
    gc.fillStyle = color("--try-floor"); gc.fillRect(0, 0, W, CH);
    gc.strokeStyle = "#aaa"; gc.lineWidth = 1;
    gc.beginPath(); gc.moveTo(0, CH / 2); gc.lineTo(W, CH / 2); gc.stroke();
    gc.strokeStyle = "#2e7d32"; gc.lineWidth = 1.5; gc.beginPath();
    r.errors.forEach(function (e, i) {
      var px = i / 1000 * W, py = CH / 2 - e / 45 * (CH / 2 - 4);
      if (i) gc.lineTo(px, py); else gc.moveTo(px, py);
    });
    gc.stroke();
  }

  function report(r) {
    var cps = r.passed.slice(0, -1).filter(Boolean).length;
    var text;
    if (r.lapMs >= 0) {
      var points = Math.max(0, 30 - r.lapMs / 1000) + cps * 4 + (r.pole ? 0 : 5);
      text = "LAPまで " + (r.lapMs / 1000).toFixed(2) + " 秒、チェックポイント " + cps + " 本、" +
        (r.pole ? "LAPのポールにさわった（LAPの5点なし）" : "ポールにはさわっていない") +
        "。ここまでの点は " + points.toFixed(2) + " 点。";
    } else {
      text = (r.lost ? "線を外れて止まった" : "40秒でLAPに届かなかった") + "。チェックポイント " + cps + " 本。";
    }
    document.getElementById("try-result").textContent = text;
  }

  var form = document.getElementById("try-form");
  var timer = 0;
  function value(id) { return document.getElementById(id); }
  function update() {
    var opt = { mode: value("try-mode").value, speed: +value("try-speed").value, kp: +value("try-kp").value,
                kd: +value("try-kd").value, seed: +value("try-seed").value };
    value("try-speed-out").textContent = opt.speed;
    value("try-kp-out").textContent = opt.kp.toFixed(1);
    value("try-kd-out").textContent = opt.kd.toFixed(1);
    value("try-seed-out").textContent = opt.seed;
    value("try-kp").disabled = value("try-kd").disabled = opt.mode === "onoff";
    clearTimeout(timer);
    timer = setTimeout(function () { var r = run(opt); draw(r); report(r); }, 50);
  }
  form.addEventListener("input", update);
  form.addEventListener("submit", function (e) { e.preventDefault(); });
  update();
})();
