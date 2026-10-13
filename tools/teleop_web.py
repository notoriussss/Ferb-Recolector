#!/usr/bin/env python3
"""Control manual del recolector desde el navegador: WASD para moverlo y video de la cámara.

Herramienta provisional, sin ROS: habla directo con el HAT, así que NO pasa por
obstacle_guard. Úsala con las ruedas en el aire o con la mano cerca del interruptor.

Uso:
  python3 tools/teleop_web.py            # en la Pi, con el robot
  python3 tools/teleop_web.py --fake     # en la laptop: motores falsos y cámara de prueba
Luego abre http://localhost:8081 (8080 es el web_viewer de ROS)

Teclas: W/S adelante/atrás, A/D de lado, Q/E girar, espacio frenar, Shift = más rápido.
"""

import argparse
import json
import logging
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

log = logging.getLogger("teleop_web")

WHEELS = ("fl", "fr", "rl", "rr")
DEFAULT_CONFIG = Path(__file__).with_name("teleop_web.yaml")


# --- Lógica pura -------------------------------------------------------------


def mecanum_wheel_ratios(vx: float, vy: float, wz: float) -> dict[str, float]:
    """Cinemática inversa mecanum (rodillos en X), normalizada a [-1, 1].

    vx > 0 adelante, vy > 0 izquierda, wz > 0 antihorario (REP 103), cada uno en [-1, 1].
    Si alguna rueda pasa de 1, se escalan todas para conservar la dirección.
    """
    ratios = {
        "fl": vx - vy - wz,
        "fr": vx + vy + wz,
        "rl": vx + vy - wz,
        "rr": vx - vy + wz,
    }
    peak = max(1.0, *(abs(r) for r in ratios.values()))
    return {w: r / peak for w, r in ratios.items()}


def clamp(value: float, low: float = -1.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


# --- Motores -----------------------------------------------------------------


class FakeMotors:
    """Motores falsos: solo registra lo que haría."""

    def __init__(self):
        self._last = None

    def set(self, ratios: dict[str, float]) -> None:
        rounded = tuple(round(ratios[w], 2) for w in WHEELS)
        if rounded != self._last:
            self._last = rounded
            log.info("motores (falso) fl=%.2f fr=%.2f rl=%.2f rr=%.2f", *rounded)

    def close(self) -> None:
        pass


class HatMotors:
    """4 motores del HAT Cokoino (2× DRV8833) con gpiozero; en la Pi 5 usa lgpio."""

    def __init__(self, cfg: dict):
        from gpiozero import Motor  # solo existe en la Pi

        self._motors = {}
        self._inverted = {}
        for w in WHEELS:
            in1, in2 = cfg[f"{w}_pins"]
            self._motors[w] = Motor(forward=in1, backward=in2, pwm=True)
            self._motors[w].forward_device.frequency = cfg["pwm_frequency_hz"]
            self._motors[w].backward_device.frequency = cfg["pwm_frequency_hz"]
            self._inverted[w] = bool(cfg[f"{w}_inverted"])

    def set(self, ratios: dict[str, float]) -> None:
        for w, motor in self._motors.items():
            value = -ratios[w] if self._inverted[w] else ratios[w]
            motor.value = clamp(value)

    def close(self) -> None:
        for motor in self._motors.values():
            motor.stop()
            motor.close()


def pins_configured(cfg: dict) -> bool:
    return all(p is not None for w in WHEELS for p in cfg[f"{w}_pins"])


class Drive:
    """Guarda el último comando y frena si pasa cmd_timeout_s sin uno nuevo."""

    def __init__(self, motors, max_duty_ratio: float, cmd_timeout_s: float):
        self._motors = motors
        self._max_duty = max_duty_ratio
        self._timeout = cmd_timeout_s
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0, 0.0)  # vx, vy, wz, velocidad
        self._stamp = 0.0
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def command(self, vx: float, vy: float, wz: float, speed: float) -> None:
        with self._lock:
            self._cmd = (clamp(vx), clamp(vy), clamp(wz), clamp(speed, 0.0, 1.0))
            self._stamp = time.monotonic()

    def state(self) -> dict:
        with self._lock:
            vx, vy, wz, speed = self._cmd
            alive = time.monotonic() - self._stamp < self._timeout
        return {"vx": vx, "vy": vy, "wz": wz, "speed": speed, "alive": alive}

    def _loop(self) -> None:
        while self._running:
            with self._lock:
                vx, vy, wz, speed = self._cmd
                expired = time.monotonic() - self._stamp > self._timeout
            if expired:
                vx = vy = wz = 0.0
            scale = speed * self._max_duty
            ratios = mecanum_wheel_ratios(vx, vy, wz)
            self._motors.set({w: r * scale for w, r in ratios.items()})
            time.sleep(0.02)

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        self._motors.set(dict.fromkeys(WHEELS, 0.0))
        self._motors.close()


# --- Cámara ------------------------------------------------------------------


class Camera:
    """Captura en un hilo y guarda el último cuadro como JPEG."""

    def __init__(self, cfg: dict, fake: bool):
        self._cfg = cfg
        self._fake = fake
        self._cond = threading.Condition()
        self._jpeg = None
        self._seq = 0
        self.ok = False
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _open(self):
        """Devuelve una función que entrega un cuadro BGR, o None si no hay cámara."""
        import cv2
        import numpy as np

        w, h = self._cfg["width_px"], self._cfg["height_px"]
        if self._fake:
            t0 = time.monotonic()

            def fake_frame():
                frame = np.zeros((h, w, 3), np.uint8)
                frame[:] = (40, 40, 40)
                x = int((time.monotonic() - t0) * 120) % w
                cv2.rectangle(frame, (x, h // 2 - 20), (x + 40, h // 2 + 20), (0, 0, 255), -1)
                cv2.putText(frame, "camara falsa", (10, 30), 0, 0.8, (255, 255, 255), 2)
                return frame

            return fake_frame

        if self._cfg["backend"] == "picamera2":
            from picamera2 import Picamera2

            cam = Picamera2()
            cam.configure(cam.create_video_configuration(main={"size": (w, h), "format": "BGR888"}))
            cam.start()
            return cam.capture_array

        cap = cv2.VideoCapture(self._cfg["device_index"])
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if not cap.isOpened():
            return None

        def read_frame():
            ok, frame = cap.read()
            return frame if ok else None

        return read_frame

    def _loop(self) -> None:
        try:
            import cv2

            grab = self._open()
        except Exception as exc:  # cámara opcional: sin ella el control sigue funcionando
            log.warning("sin cámara: %s", exc)
            return
        if grab is None:
            log.warning("no se pudo abrir la cámara %s", self._cfg["device_index"])
            return
        self.ok = True
        period = 1.0 / self._cfg["fps"]
        params = [cv2.IMWRITE_JPEG_QUALITY, self._cfg["jpeg_quality"]]
        while self._running:
            t = time.monotonic()
            frame = grab()
            if frame is not None:
                ok, buf = cv2.imencode(".jpg", frame, params)
                if ok:
                    with self._cond:
                        self._jpeg = buf.tobytes()
                        self._seq += 1
                        self._cond.notify_all()
            time.sleep(max(0.0, period - (time.monotonic() - t)))

    def wait_frame(self, last_seq: int, timeout: float = 2.0):
        """Espera un cuadro más nuevo que last_seq; devuelve (seq, jpeg)."""
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq, timeout=timeout)
            return self._seq, self._jpeg

    def close(self) -> None:
        self._running = False


# --- Servidor web ------------------------------------------------------------

PAGE = """<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Recolector · control</title>
<style>
  body { margin: 0; font: 15px system-ui, sans-serif; background: #111; color: #eee;
         display: flex; flex-direction: column; align-items: center; gap: 12px; padding: 16px; }
  img { width: min(100%, 640px); aspect-ratio: 4 / 3; background: #222; border-radius: 8px; }
  .keys { display: grid; grid-template-columns: repeat(3, 56px); gap: 6px; }
  .k { height: 56px; border-radius: 8px; background: #2a2a2a; display: grid;
       place-items: center; font-weight: 700; user-select: none; touch-action: none; }
  .k.on { background: #2f7d32; }
  .row { display: flex; gap: 16px; align-items: center; flex-wrap: wrap; justify-content: center; }
  #st { font-family: ui-monospace, monospace; color: #aaa; }
  #st.off { color: #e57373; }
</style></head><body>
<h2 style="margin:0">Recolector</h2>
<img id="cam" src="/stream.mjpg" alt="cámara">
<div class="row">
  <div class="keys">
    <div class="k" data-k="q">Q</div><div class="k" data-k="w">W</div>
    <div class="k" data-k="e">E</div><div class="k" data-k="a">A</div>
    <div class="k" data-k="s">S</div><div class="k" data-k="d">D</div>
  </div>
  <label>Velocidad <input id="spd" type="range" min="10" max="100" value="50">
    <span id="spdv">50</span>%</label>
</div>
<div id="st">conectando…</div>
<div style="color:#888">
  W/S adelante·atrás · A/D lado · Q/E girar · espacio frenar · Shift rápido
</div>
<script>
const down = new Set();
const spd = document.getElementById("spd"), st = document.getElementById("st");
spd.oninput = () => document.getElementById("spdv").textContent = spd.value;
const KEYS = "wasdqe";
function paint() {
  document.querySelectorAll(".k").forEach(el => el.classList.toggle("on", down.has(el.dataset.k)));
}
addEventListener("keydown", ev => {
  const k = ev.key.toLowerCase();
  if (k === " ") { down.clear(); ev.preventDefault(); }
  else if (KEYS.includes(k)) down.add(k);
  if (ev.shiftKey) down.add("shift");
  paint();
});
addEventListener("keyup", ev => {
  down.delete(ev.key.toLowerCase());
  if (!ev.shiftKey) down.delete("shift");
  paint();
});
addEventListener("blur", () => { down.clear(); paint(); });
document.querySelectorAll(".k").forEach(el => {
  el.onpointerdown = () => { down.add(el.dataset.k); paint(); };
  el.onpointerup = el.onpointerleave = () => { down.delete(el.dataset.k); paint(); };
});
async function tick() {
  const vx = (down.has("w") ? 1 : 0) - (down.has("s") ? 1 : 0);
  const vy = (down.has("a") ? 1 : 0) - (down.has("d") ? 1 : 0);
  const wz = (down.has("q") ? 1 : 0) - (down.has("e") ? 1 : 0);
  const speed = down.has("shift") ? 1 : spd.value / 100;
  try {
    const r = await fetch("/cmd", { method: "POST", body: JSON.stringify({ vx, vy, wz, speed }) });
    const s = await r.json();
    st.className = "";
    st.textContent = `vx=${vx} vy=${vy} wz=${wz} vel=${Math.round(speed * 100)}% · ` +
      `motores ${s.motors} · cámara ${s.camera ? "sí" : "no"}`;
  } catch (e) { st.className = "off"; st.textContent = "sin conexión con el robot"; }
}
setInterval(tick, 100);
</script></body></html>
"""


def make_handler(drive: Drive, camera: Camera, motors_label: str):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            log.debug(fmt, *args)

        def do_GET(self):
            if self.path == "/":
                body = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/stream.mjpg":
                self._stream()
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path != "/cmd":
                self.send_error(404)
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                data = json.loads(self.rfile.read(length) or b"{}")
                drive.command(
                    float(data.get("vx", 0)),
                    float(data.get("vy", 0)),
                    float(data.get("wz", 0)),
                    float(data.get("speed", 0)),
                )
            except (ValueError, TypeError):
                self.send_error(400)
                return
            body = json.dumps({"motors": motors_label, "camera": camera.ok}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            seq = -1
            try:
                while True:
                    seq, jpeg = camera.wait_frame(seq)
                    if jpeg is None:
                        if not camera.ok:
                            time.sleep(1.0)
                        continue
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                        + jpeg
                        + b"\r\n"
                    )
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def load_config(path: Path) -> dict:
    import yaml

    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--fake", action="store_true", help="motores falsos y cámara de prueba")
    parser.add_argument("--host", help="0.0.0.0 para abrirlo desde otra máquina de la red")
    parser.add_argument("--port", type=int)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    # SIGTERM (kill, systemd) también frena los motores antes de salir
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    cfg = load_config(args.config)
    mcfg, ccfg, scfg = cfg["motors"], cfg["camera"], cfg["server"]

    if args.fake:
        motors, label = FakeMotors(), "falsos"
    elif not pins_configured(mcfg):
        log.warning("faltan pines en %s: motores en modo falso", args.config)
        motors, label = FakeMotors(), "falsos (faltan pines)"
    else:
        motors, label = HatMotors(mcfg), "HAT"

    drive = Drive(motors, mcfg["max_duty_ratio"], mcfg["cmd_timeout_s"])
    camera = Camera(ccfg, fake=args.fake)
    host = args.host or scfg["host"]
    port = args.port or scfg["port"]
    server = ThreadingHTTPServer((host, port), make_handler(drive, camera, label))
    server.daemon_threads = True
    shown = "localhost" if host == "127.0.0.1" else host
    log.info("abre http://%s:%d (Ctrl+C para salir)", shown, port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        log.info("frenando y cerrando")
        server.server_close()
        camera.close()
        drive.close()


if __name__ == "__main__":
    main()
