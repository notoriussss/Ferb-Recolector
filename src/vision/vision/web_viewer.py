"""Nodo web_viewer: página web para ver la cámara del robot y elegir los colores objetivo.

Abre un servidor HTTP (por defecto en el puerto 8080) con:
  /              página con el video, los cubos detectados y el estado del comportamiento
  /stream.mjpg   video MJPEG de cubes/debug_image (o de image_raw si no hay detector)
  /api/state     JSON con los cubos y el estado
  /api/params    POST JSON: cambia `enabled`, `target_colors` o `stop_distance_m` en behavior_fsm

Sirve igual en la laptop (simulación) y en la Pi: desde otra máquina se abre
http://<ip-de-la-pi>:8080 o se pasa por un túnel SSH (ver docs/despliegue-ssh.md).
"""

import json
import os
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from recolector_interfaces.msg import CubeDetectionArray
from vision.ros_image import image_to_bgr

DEFAULT_COLORS = ["rojo", "naranja", "amarillo", "verde", "azul", "morado"]


class FrameBuffer:
    """Último cuadro JPEG, compartido entre el hilo de ROS y los clientes HTTP."""

    def __init__(self):
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = 0

    def put(self, jpeg: bytes):
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()

    def wait_next(self, last_seq: int, timeout_s: float) -> tuple[int, bytes | None]:
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq, timeout=timeout_s)
            return self._seq, self._jpeg


def _param_msg(name: str, value) -> ParameterMsg:
    v = ParameterValue()
    if isinstance(value, bool):
        v.type, v.bool_value = ParameterType.PARAMETER_BOOL, value
    elif isinstance(value, float | int):
        v.type, v.double_value = ParameterType.PARAMETER_DOUBLE, float(value)
    elif isinstance(value, list):
        v.type, v.string_array_value = ParameterType.PARAMETER_STRING_ARRAY, [str(x) for x in value]
    else:
        raise ValueError(f"tipo no soportado para {name}")
    return ParameterMsg(name=name, value=v)


class WebViewerNode(Node):
    def __init__(self):
        super().__init__("web_viewer")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("max_fps_hz", 15.0)
        self.declare_parameter("jpeg_quality", 75)
        self.declare_parameter("color_names", DEFAULT_COLORS)
        self.declare_parameter("behavior_node", "behavior_fsm")

        self.frames = FrameBuffer()
        self._lock = threading.Lock()
        self._cubes: list[dict] = []
        self._behavior: dict | None = None
        self._last_debug_s = 0.0
        self._last_frame_s = 0.0
        self._fps = 0.0

        self.create_subscription(
            Image, "cubes/debug_image", self._on_debug_image, qos_profile_sensor_data
        )
        self.create_subscription(Image, "image_raw", self._on_raw_image, qos_profile_sensor_data)
        self.create_subscription(CubeDetectionArray, "cubes", self._on_cubes, 10)
        self.create_subscription(String, "behavior/state", self._on_behavior, 10)
        behavior = self.get_parameter("behavior_node").value
        self._param_client = self.create_client(SetParameters, f"{behavior}/set_parameters")

    # ---------- ROS ----------
    def _encode(self, msg: Image):
        now = time.monotonic()
        if now - self._last_frame_s < 1.0 / self.get_parameter("max_fps_hz").value:
            return
        dt = now - self._last_frame_s
        self._fps = 0.8 * self._fps + 0.2 * (1.0 / dt) if dt < 5 else 0.0
        self._last_frame_s = now
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        quality = int(self.get_parameter("jpeg_quality").value)
        ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            self.frames.put(jpg.tobytes())

    def _on_debug_image(self, msg: Image):
        self._last_debug_s = time.monotonic()
        self._encode(msg)

    def _on_raw_image(self, msg: Image):
        # solo si no hay detector publicando la imagen anotada
        if time.monotonic() - self._last_debug_s > 1.0:
            self._encode(msg)

    def _on_cubes(self, msg: CubeDetectionArray):
        cubes = [
            {
                "color": c.color,
                "distance_m": round(c.distance_m, 3),
                "bearing_rad": round(c.bearing_rad, 3),
                "u": c.u,
                "v": c.v,
            }
            for c in msg.cubes
        ]
        with self._lock:
            self._cubes = cubes

    def _on_behavior(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._behavior = data

    # ---------- para el servidor HTTP ----------
    def snapshot(self) -> dict:
        with self._lock:
            return {
                "cubes": list(self._cubes),
                "behavior": self._behavior,
                "fps": round(self._fps, 1),
                "color_names": list(self.get_parameter("color_names").value),
            }

    def set_behavior_params(self, values: dict, timeout_s: float = 2.0) -> tuple[bool, str]:
        allowed = {"enabled", "target_colors", "stop_distance_m"}
        unknown = set(values) - allowed
        if unknown:
            return False, f"parámetros no permitidos: {sorted(unknown)}"
        if not self._param_client.service_is_ready():
            return False, "behavior_fsm no está corriendo"
        req = SetParameters.Request()
        try:
            req.parameters = [_param_msg(k, v) for k, v in values.items()]
        except ValueError as e:
            return False, str(e)
        done = threading.Event()
        future = self._param_client.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout_s):
            return False, "behavior_fsm no respondió"
        results = future.result().results
        errors = [r.reason for r in results if not r.successful]
        return (not errors), "; ".join(errors) or "ok"


def make_handler(node: WebViewerNode, index_html: bytes):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # sin log por cada petición
            pass

        def _send(self, code, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, data):
            self._send(code, json.dumps(data).encode(), "application/json")

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(HTTPStatus.OK, index_html, "text/html; charset=utf-8")
            elif self.path.startswith("/api/state"):
                self._json(HTTPStatus.OK, node.snapshot())
            elif self.path.startswith("/stream.mjpg"):
                self._stream()
            elif self.path.startswith("/snapshot.jpg"):
                _, jpg = node.frames.wait_next(-1, 0.0)
                if jpg is None:
                    self._send(HTTPStatus.SERVICE_UNAVAILABLE, b"sin imagen", "text/plain")
                else:
                    self._send(HTTPStatus.OK, jpg, "image/jpeg")
            else:
                self._send(HTTPStatus.NOT_FOUND, b"no existe", "text/plain")

        def do_POST(self):
            if not self.path.startswith("/api/params"):
                self._send(HTTPStatus.NOT_FOUND, b"no existe", "text/plain")
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                values = json.loads(self.rfile.read(length) or b"{}")
                if not isinstance(values, dict):
                    raise ValueError("se esperaba un objeto JSON")
            except (ValueError, json.JSONDecodeError) as e:
                self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(e)})
                return
            ok, message = node.set_behavior_params(values)
            code = HTTPStatus.OK if ok else HTTPStatus.CONFLICT
            self._json(code, {"ok": ok, "message": message})

        def _stream(self):
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = -1
            try:
                while rclpy.ok():
                    seq, jpg = node.frames.wait_next(seq, 2.0)
                    if jpg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                    self.wfile.write(jpg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = WebViewerNode()
    share = get_package_share_directory("vision")
    with open(os.path.join(share, "web", "index.html"), "rb") as f:
        index_html = f.read()
    host = node.get_parameter("host").value
    port = node.get_parameter("port").value
    server = ThreadingHTTPServer((host, port), make_handler(node, index_html))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"visor web en http://{host}:{port}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
