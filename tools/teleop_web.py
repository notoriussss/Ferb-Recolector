#!/usr/bin/env python3
"""Browser control for the collector: WASD driving, camera video and an Auto mode.

Auto mode: find a target-colour cube, push it into the scoop, carry it to the green/blue
delivery line, leave it there and count it. The bottom line sensor stops the robot at the
black border (it backs up and turns inward).

Interim tool without ROS: it talks straight to the HAT, so it does NOT go through
obstacle_guard. Use it with the wheels off the ground or a hand near the switch.

Usage:
  python3 tools/teleop_web.py            # on the Pi, with the robot
  python3 tools/teleop_web.py --fake     # on the laptop: fake motors and test camera
Then open http://localhost:8081 (8080 is the ROS web_viewer)

Keys: W/S forward/back, A/D turn, Q/E strafe, P auto, R reset, space brake, Shift = faster.
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


# --- Pure logic --------------------------------------------------------------


def mecanum_wheel_ratios(vx: float, vy: float, wz: float) -> dict[str, float]:
    """Mecanum inverse kinematics (X-pattern rollers), normalised to [-1, 1].

    vx > 0 forward, vy > 0 left, wz > 0 counter-clockwise (REP 103), each in [-1, 1].
    If any wheel goes past 1, all are scaled down to keep the direction.
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


# --- Motors ------------------------------------------------------------------


class FakeMotors:
    """Fake motors: only logs what it would do."""

    def __init__(self):
        self._last = None

    def set(self, ratios: dict[str, float]) -> None:
        rounded = tuple(round(ratios[w], 2) for w in WHEELS)
        if rounded != self._last:
            self._last = rounded
            log.info("motors (fake) fl=%.2f fr=%.2f rl=%.2f rr=%.2f", *rounded)

    def close(self) -> None:
        pass


class HatMotors:
    """4 motors on the Cokoino HAT (2× DRV8833) via gpiozero; uses lgpio on the Pi 5."""

    def __init__(self, cfg: dict):
        from gpiozero import Motor, OutputDevice  # only available on the Pi

        self._nsleep = [
            OutputDevice(pin, active_high=True, initial_value=True)
            for pin in cfg.get("nsleep_pins", [12, 13])
            if pin is not None
        ]
        self._motors = {}
        self._inverted = {}
        self._last = {}
        for w in WHEELS:
            in1, in2 = cfg[f"{w}_pins"]
            self._motors[w] = Motor(forward=in1, backward=in2, pwm=True)
            self._motors[w].forward_device.frequency = cfg["pwm_frequency_hz"]
            self._motors[w].backward_device.frequency = cfg["pwm_frequency_hz"]
            self._inverted[w] = bool(cfg[f"{w}_inverted"])
            self._last[w] = None

    def set(self, ratios: dict[str, float]) -> None:
        for w, motor in self._motors.items():
            value = clamp(round(-ratios[w] if self._inverted[w] else ratios[w], 3))
            if value != self._last[w]:
                self._last[w] = value
                motor.value = value

    def close(self) -> None:
        for motor in self._motors.values():
            motor.stop()
            motor.close()
        for pin in self._nsleep:
            pin.off()
            pin.close()


class FusionHatMotors:
    """4 motors on the SunFounder Fusion HAT+ (MCU at I2C 0x17, PWM through registers).

    Each motor uses two PWM channels (a, b): a>0 forward, b>0 backward.
    Timers 1 (P4-P7) and 2 (P8-P11) are set to 100 Hz.
    """

    ADDR = 0x17
    ARR = 39999  # 72 MHz / 18 / 40000 = 100 Hz
    PSC = 17

    def __init__(self, cfg: dict):
        from smbus2 import SMBus  # only available on the Pi

        self._bus = SMBus(1)
        self._channels = {w: tuple(cfg["fusion_channels"][w]) for w in WHEELS}
        self._inverted = {w: bool(cfg[f"{w}_inverted"]) for w in WHEELS}
        self._last = dict.fromkeys(WHEELS)
        for timer in sorted({ch // 4 for pair in self._channels.values() for ch in pair}):
            self._write(0x50 + timer, self.ARR)
            self._write(0x40 + timer, self.PSC)
        self.set(dict.fromkeys(WHEELS, 0.0))

    def _write(self, reg: int, value: int) -> None:
        value &= 0xFFFF
        self._bus.write_word_data(self.ADDR, reg, ((value & 0xFF) << 8) | (value >> 8))

    def set(self, ratios: dict[str, float]) -> None:
        for w, (a, b) in self._channels.items():
            value = clamp(round(-ratios[w] if self._inverted[w] else ratios[w], 3))
            if value == self._last[w]:
                continue
            self._last[w] = value
            duty = int(abs(value) * self.ARR)
            self._write(0x60 + a, duty if value > 0 else 0)
            self._write(0x60 + b, duty if value < 0 else 0)

    def close(self) -> None:
        self.set(dict.fromkeys(WHEELS, 0.0))
        self._bus.close()


class LineSensor:
    """Bottom line sensor (TCRT5000, digital DO output) read with gpiozero.

    Power it from 3.3 V: at 5 V the DO output puts 5 V on the Pi's GPIO.
    """

    def __init__(self, cfg: dict):
        from gpiozero import DigitalInputDevice  # only available on the Pi

        self._dev = DigitalInputDevice(cfg["pin"], pull_up=False)
        self._black_value = int(cfg.get("black_value", 1))

    def is_black(self) -> bool:
        return self._dev.value == self._black_value

    def close(self) -> None:
        self._dev.close()


def pins_configured(cfg: dict) -> bool:
    if cfg.get("driver") == "fusion_hat":
        return True
    return all(p is not None for w in WHEELS for p in cfg[f"{w}_pins"])


class Drive:
    """Keeps the last command, brakes after cmd_timeout_s without a new one, runs Auto."""

    def __init__(
        self,
        motors,
        max_duty_ratio: float,
        cmd_timeout_s: float,
        auto_cfg: dict | None = None,
        line_sensor=None,
    ):
        self._motors = motors
        self._max_duty = max_duty_ratio
        self._timeout = cmd_timeout_s
        self._auto_cfg = auto_cfg or {}
        self._line = line_sensor  # None = no line sensor
        self.auto_enabled = bool(self._auto_cfg.get("enabled", False))
        self.auto_state = "PAUSED"
        self.target_info = None
        self.collected_count = 0  # cubes pushed into the scoop
        self.delivered_count = 0  # cubes left at the green/blue line
        self.carrying = False
        self.line_black = False
        self._phase_until = 0.0
        self._phase2_until = 0.0
        self._resume_state = "SEARCH"
        self._last_seen = 0.0
        self._search_start = time.monotonic()
        self._search_dir = 1.0
        self._camera = None
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0, 0.0)  # vx, vy, wz, speed
        self._stamp = 0.0
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set_camera(self, camera) -> None:
        self._camera = camera

    def set_auto(self, enabled: bool | None = None, reset: bool = False) -> None:
        with self._lock:
            now = time.monotonic()
            if enabled is not None:
                self.auto_enabled = bool(enabled)
            if reset or self.auto_enabled:
                if self.auto_state.startswith(("DONE", "NO CUBES", "PAUSED")) or reset:
                    self.auto_state = "CARRY" if self.carrying and not reset else "SEARCH"
                    self._search_start = now
                    if reset:
                        self.collected_count = 0
                        self.delivered_count = 0
                        self.carrying = False
            if not self.auto_enabled:
                self.auto_state = "PAUSED"

    def command(self, vx: float, vy: float, wz: float, speed: float) -> None:
        with self._lock:
            self._cmd = (clamp(vx), clamp(vy), clamp(wz), clamp(speed, 0.0, 1.0))
            self._stamp = time.monotonic()

    def _auto_step(self, now: float) -> tuple[float, float, float, float]:
        """Computes (vx, vy, wz, speed) in Auto mode using the robot's real axes:
        vy > 0 = forward (W), vx = -turn (A turns left -> vx < 0, D turns right -> vx > 0).
        """
        cfg = self._auto_cfg
        stop_d = float(cfg.get("stop_distance_m", 0.185))
        fwd_spd = float(cfg.get("forward_speed", 0.42))
        turn_spd = float(cfg.get("turn_speed", 0.38))
        align_tol = float(cfg.get("align_tolerance_rad", 0.22))
        lost_to = float(cfg.get("lost_timeout_s", 0.6))
        no_cubes_s = float(cfg.get("no_cubes_stop_s", 7.0))
        push_s = float(cfg.get("scoop_push_s", 0.65))
        line_backup_s = float(cfg.get("line_backup_s", 0.6))
        line_turn_s = float(cfg.get("line_turn_s", 1.2))
        line_spd = float(cfg.get("line_speed", 0.35))
        deliver_push_s = float(cfg.get("deliver_push_s", 0.5))
        deliver_backup_s = float(cfg.get("deliver_backup_s", 0.8))
        deliver_turn_s = float(cfg.get("deliver_turn_s", 2.0))
        zone_stop_d = float(cfg.get("zone_stop_distance_m", 0.20))
        zone_arrive_v = int(cfg.get("zone_arrive_v_px", 440))

        if self.auto_state.startswith(("DONE", "NO CUBES")):
            return 0.0, 0.0, 0.0, 0.0
        if self.auto_state in ("MANUAL", "PAUSED"):
            # back from WASD or pause: if it had a cube, keep carrying it
            self.auto_state = "CARRY" if self.carrying else "SEARCH"
            self._search_start = now

        # Black line underneath: brake, back up and turn inward, then resume what it was doing
        if self.line_black and not self.auto_state.startswith("LINE"):
            self._resume_state = "CARRY" if self.carrying else "SEARCH"
            self.auto_state = "LINE (back up)"
            self._phase_until = now + line_backup_s
            self._phase2_until = self._phase_until + line_turn_s
            self.target_info = None
        if self.auto_state.startswith("LINE"):
            if now < self._phase_until:
                return 0.0, -1.0, 0.0, line_spd
            if now < self._phase2_until:
                self.auto_state = "LINE (turn)"
                return -self._search_dir, 0.0, 0.0, turn_spd
            self.auto_state = self._resume_state
            self._search_start = now

        # Blind push through the last ~16.5 cm, where the scoop hides the cube from the camera
        if self.auto_state == "CATCH":
            if now < self._phase_until:
                return 0.0, 1.0, 0.0, fwd_spd
            self.collected_count += 1
            self.carrying = True
            self.auto_state = "CARRY"
            self._search_start = now
            return 0.0, 0.0, 0.0, 0.0

        # Delivery: push the cube onto the line, back up leaving it there and turn around
        if self.auto_state.startswith("DROP"):
            if now < self._phase_until:
                return 0.0, 1.0, 0.0, fwd_spd
            if self.carrying:
                self.carrying = False
                self.delivered_count += 1
            if now < self._phase_until + deliver_backup_s:
                self.auto_state = "DROP (back up)"
                return 0.0, -1.0, 0.0, line_spd
            if now < self._phase_until + deliver_backup_s + deliver_turn_s:
                self.auto_state = "DROP (turn)"
                return -self._search_dir, 0.0, 0.0, turn_spd
            self.auto_state = "SEARCH"
            self._search_start = now

        if self.auto_state.startswith("CARRY"):
            return self._carry_step(now, zone_stop_d, zone_arrive_v, deliver_push_s)

        det, _ = self._camera.latest_detection(now, lost_to) if self._camera else (None, 0)
        if det is None:
            self.target_info = None
            # If it lost sight of a cube a moment ago, wait briefly before turning
            if self._last_seen and (now - self._last_seen) < lost_to:
                return 0.0, 0.0, 0.0, 0.0
            # After searching too long without seeing any cube, stop
            if (now - self._search_start) >= no_cubes_s:
                if self.delivered_count > 0:
                    self.auto_state = f"DONE ({self.delivered_count} delivered)"
                else:
                    self.auto_state = "NO CUBES (stopped)"
                return 0.0, 0.0, 0.0, 0.0
            return self._roam(now, "EXPLORE")

        self.target_info = f"{det.color} {det.distance_m * 100:.0f}cm"
        self._last_seen = now
        self._search_start = now
        self._search_dir = 1.0 if det.bearing_rad >= 0 else -1.0

        # Cube at the scoop threshold (~18.5 cm or image bottom) and aligned:
        if (det.distance_m <= stop_d or det.v >= 445) and abs(det.bearing_rad) <= align_tol * 1.4:
            self.auto_state = "CATCH"
            self._phase_until = now + push_s
            return 0.0, 1.0, 0.0, fwd_spd

        return self._steer_to(det.bearing_rad, det.distance_m, stop_d, "APPROACH")

    def _roam(self, now: float, label: str) -> tuple[float, float, float, float]:
        """Free roaming: alternate driving forward (1.3 s) and turning to scan (1.7 s)."""
        cfg = self._auto_cfg
        if (now - self._search_start) % 3.0 < 1.3:
            self.auto_state = f"{label} (forward)"
            return 0.0, 1.0, 0.0, float(cfg.get("roam_forward_speed", 0.35))
        self.auto_state = f"{label} (turn)"
        return -self._search_dir, 0.0, 0.0, float(cfg.get("search_turn_speed", 0.32))

    def _steer_to(self, bearing: float, distance: float, stop_d: float, label: str):
        """Turn in place if far off-heading; otherwise drive forward while correcting."""
        cfg = self._auto_cfg
        align_tol = float(cfg.get("align_tolerance_rad", 0.22))
        if abs(bearing) > align_tol:
            self.auto_state = "ALIGN" if label == "APPROACH" else f"{label} (align)"
            turn_dir = 1.0 if bearing > 0 else -1.0
            return -turn_dir, 0.0, 0.0, float(cfg.get("turn_speed", 0.38))
        self.auto_state = label
        slow_d = float(cfg.get("slow_down_distance_m", 0.45))
        frac = clamp((distance - stop_d) / max(0.05, slow_d - stop_d), 0.35, 1.0)
        steer = clamp(bearing / align_tol, -0.7, 0.7)
        return -steer, 1.0, 0.0, float(cfg.get("forward_speed", 0.42)) * frac

    def _carry_step(self, now: float, zone_stop_d: float, zone_arrive_v: int, push_s: float):
        """With a cube in the scoop: find the green/blue line and drive to it."""
        lost_to = float(self._auto_cfg.get("lost_timeout_s", 0.6))
        zone = self._camera.latest_zone(now, lost_to) if self._camera else None
        if zone is None:
            self.target_info = "looking for zone"
            return self._roam(now, "CARRY")
        self.target_info = f"zone {zone.color} {zone.distance_m * 100:.0f}cm"
        self._search_start = now
        self._search_dir = 1.0 if zone.bearing_rad >= 0 else -1.0
        align_tol = float(self._auto_cfg.get("align_tolerance_rad", 0.22))
        near = zone.distance_m <= zone_stop_d or zone.v >= zone_arrive_v
        if near and abs(zone.bearing_rad) <= align_tol * 1.4:
            self.auto_state = "DROP"
            self._phase_until = now + push_s
            return 0.0, 1.0, 0.0, float(self._auto_cfg.get("forward_speed", 0.42))
        return self._steer_to(zone.bearing_rad, zone.distance_m, zone_stop_d, "CARRY")

    def _loop(self) -> None:
        while self._running:
            now = time.monotonic()
            with self._lock:
                vx, vy, wz, speed = self._cmd
                expired = now - self._stamp > self._timeout
                manual_active = not expired and (vx != 0.0 or vy != 0.0 or wz != 0.0)
                auto_on = self.auto_enabled

            self.line_black = self._line is not None and self._line.is_black()
            if manual_active:
                self.auto_state = "MANUAL"
            elif auto_on:
                vx, vy, wz, speed = self._auto_step(now)
            else:
                vx = vy = wz = 0.0
                self.auto_state = "PAUSED"

            scale = speed * self._max_duty
            ratios = mecanum_wheel_ratios(vx, vy, wz)
            self._motors.set({w: r * scale for w, r in ratios.items()})
            time.sleep(0.02)

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        self._motors.set(dict.fromkeys(WHEELS, 0.0))
        self._motors.close()


# --- Camera -------------------------------------------------------------------


class Camera:
    """Captures in a thread and keeps the latest frame as JPEG."""

    def __init__(self, cfg: dict, fake: bool):
        self._cfg = cfg
        self._fake = fake
        self._cond = threading.Condition()
        self._jpeg = None
        self._seq = 0
        self._det = None
        self._det_count = 0
        self._det_stamp = 0.0
        self._zone = None
        self._zone_stamp = 0.0
        self.ok = False
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def latest_detection(self, now: float, timeout: float = 0.6):
        with self._cond:
            if self._det is not None and (now - self._det_stamp) <= timeout:
                return self._det, self._det_count
            return None, 0

    def latest_zone(self, now: float, timeout: float = 0.6):
        """The delivery line (green/blue) seen less than `timeout` ago, or None."""
        with self._cond:
            if self._zone is not None and (now - self._zone_stamp) <= timeout:
                return self._zone
            return None

    def _open(self):
        """Returns a function that yields a BGR frame, or None if there is no camera."""
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
                cv2.putText(frame, "fake camera", (10, 30), 0, 0.8, (255, 255, 255), 2)
                return frame

            return fake_frame

        if self._cfg["backend"] == "picamera2":
            from picamera2 import Picamera2

            cams = Picamera2.global_camera_info()
            cam_num = next(
                (c["Num"] for c in cams if "i2c@80000" in c.get("Id", "")),
                self._cfg["device_index"],
            )
            cam = Picamera2(cam_num)
            # In Picamera2/DRM, "RGB888" gives [B, G, R] bytes (the BGR order OpenCV expects).
            cam.configure(cam.create_video_configuration(main={"size": (w, h), "format": "RGB888"}))
            cam.start()
            vflip = bool(self._cfg.get("vflip", True))
            hflip = bool(self._cfg.get("hflip", False))

            def grab_picam():
                frame = cam.capture_array()
                if vflip and hflip:
                    return cv2.flip(frame, -1)
                if vflip:
                    return cv2.flip(frame, 0)
                if hflip:
                    return cv2.flip(frame, 1)
                return frame

            return grab_picam

        cap = cv2.VideoCapture(self._cfg["device_index"])
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if not cap.isOpened():
            return None

        def read_frame():
            ok, frame = cap.read()
            return frame if ok else None

        return read_frame

    def _build_detector(self):
        colors = self._cfg.get("target_colors") or []
        if not colors:
            return None
        try:
            import sys

            import cv2
            import numpy as np
            import yaml

            root = Path(__file__).resolve().parent.parent
            sys.path.insert(0, str(root / "src" / "vision"))
            from vision.color_detection import (
                CameraModel,
                ColorDetector,
                DetectionFilter,
                draw_detections,
                parse_hsv_ranges,
            )

            hsv_cfg = yaml.safe_load(
                open(root / "src" / "vision" / "config" / "hsv_ranges.yaml", encoding="utf-8")
            )["cube_detector"]["ros__parameters"]
            ranges = {c: parse_hsv_ranges(hsv_cfg["hsv"][c]) for c in colors if c in hsv_cfg["hsv"]}
            cam_model = CameraModel(
                width_px=self._cfg["width_px"],
                height_px=self._cfg["height_px"],
                hfov_rad=hsv_cfg["camera"]["hfov_rad"],
                height_m=hsv_cfg["camera"]["height_m"],
                tilt_rad=hsv_cfg["camera"]["tilt_rad"],
            )
            filters = DetectionFilter(
                min_area_px=60,
                cube_size_m=hsv_cfg["filter"]["cube_size_m"],
                min_size_ratio=0.15,
                max_size_ratio=6.0,
                ignore_touching_bottom=False,
                max_distance_m=3.0,
            )
            detector = ColorDetector(ranges, cam_model, filters)
            zone_colors = self._cfg.get("zone_colors") or []
            zone_detector = ColorDetector(
                {c: parse_hsv_ranges(hsv_cfg["hsv"][c]) for c in zone_colors}, cam_model
            )
            zone_min_area = int(self._cfg.get("zone_min_area_px", 300))
            margin = int(self._cfg.get("zone_ignore_margin_px", 25))
            kernel = np.ones((2 * margin + 1, 2 * margin + 1), np.uint8)

            def annotate(frame):
                dets = detector.detect(frame)
                zone = None
                if zone_colors:
                    zone, zmask = zone_detector.find_region(frame, zone_min_area)
                    # cubes already at the delivery line are not picked up again
                    near = cv2.dilate(zmask, kernel)
                    h, w = near.shape
                    dets = [d for d in dets if not near[min(d.v, h - 1), min(d.u, w - 1)]]
                top = dets[0] if dets else None
                now = time.monotonic()
                with self._cond:
                    self._det = top
                    self._det_count = len(dets)
                    if top is not None:
                        self._det_stamp = now
                    self._zone = zone
                    if zone is not None:
                        self._zone_stamp = now
                out = draw_detections(frame, dets, top)
                if zone is not None:
                    x, y, bw, bh = zone.bbox
                    cv2.rectangle(out, (x, y), (x + bw, y + bh), (255, 255, 255), 2)
                    cv2.drawMarker(out, (zone.u, zone.v), (255, 255, 255), cv2.MARKER_CROSS, 18, 2)
                    label = f"zone {zone.color} {zone.distance_m * 100:.0f}cm"
                    cv2.putText(out, label, (x, max(12, y - 4)), 0, 0.5, (255, 255, 255), 1)
                return out

            return annotate
        except Exception as exc:
            log.warning("no colour detector: %s", exc)
            return None

    def _loop(self) -> None:
        try:
            import cv2

            grab = self._open()
        except Exception as exc:  # camera is optional: control keeps working without it
            log.warning("no camera: %s", exc)
            return
        if grab is None:
            log.warning("could not open camera %s", self._cfg["device_index"])
            return
        annotate = self._build_detector()
        self.ok = True
        period = 1.0 / self._cfg["fps"]
        params = [cv2.IMWRITE_JPEG_QUALITY, self._cfg["jpeg_quality"]]
        while self._running:
            t = time.monotonic()
            frame = grab()
            if frame is not None:
                if annotate is not None:
                    frame = annotate(frame)
                ok, buf = cv2.imencode(".jpg", frame, params)
                if ok:
                    with self._cond:
                        self._jpeg = buf.tobytes()
                        self._seq += 1
                        self._cond.notify_all()
            time.sleep(max(0.0, period - (time.monotonic() - t)))

    def wait_frame(self, last_seq: int, timeout: float = 2.0):
        """Waits for a frame newer than last_seq; returns (seq, jpeg)."""
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq, timeout=timeout)
            return self._seq, self._jpeg

    def close(self) -> None:
        self._running = False


# --- Web server -------------------------------------------------------------

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Collector · control</title>
<style>
  body { margin: 0; font: 15px system-ui, sans-serif; background: #111; color: #eee;
         display: flex; flex-direction: column; align-items: center; gap: 12px; padding: 16px; }
  img { width: min(100%, 640px); aspect-ratio: 4 / 3; background: #222; border-radius: 8px; }
  .keys { display: grid; grid-template-columns: repeat(3, 56px); gap: 6px; }
  .k { height: 56px; border-radius: 8px; background: #2a2a2a; display: grid;
       place-items: center; font-weight: 700; user-select: none; touch-action: none; }
  .k.on { background: #2f7d32; }
  .btn { padding: 10px 16px; border-radius: 8px; border: none; font: inherit; font-weight: 700;
         cursor: pointer; background: #2a2a2a; color: #eee; }
  .btn.on { background: #2f7d32; color: #fff; }
  .row { display: flex; gap: 16px; align-items: center; flex-wrap: wrap; justify-content: center; }
  #st { font-family: ui-monospace, monospace; color: #aaa; }
  #st.off { color: #e57373; }
</style></head><body>
<h2 style="margin:0">Collector</h2>
<img id="cam" src="/stream.mjpg" alt="camera">
<div class="row">
  <div class="keys">
    <div class="k" data-k="q">Q</div><div class="k" data-k="w">W</div>
    <div class="k" data-k="e">E</div><div class="k" data-k="a">A</div>
    <div class="k" data-k="s">S</div><div class="k" data-k="d">D</div>
  </div>
  <label>Speed <input id="spd" type="range" min="10" max="100" value="50">
    <span id="spdv">50</span>%</label>
  <button id="autoBtn" class="btn">Auto (P): OFF</button>
  <button id="nextBtn" class="btn">Reset (R)</button>
</div>
<div id="st">connecting…</div>
<div style="color:#888">
  W/S forward·back · A/D turn · Q/E strafe · P auto · R reset · space brake · Shift fast
</div>
<script>
const down = new Set();
const spd = document.getElementById("spd"), st = document.getElementById("st");
const autoBtn = document.getElementById("autoBtn"), nextBtn = document.getElementById("nextBtn");
let autoCmd = null, resetCmd = false;
spd.oninput = () => document.getElementById("spdv").textContent = spd.value;
autoBtn.onclick = () => { autoCmd = !autoBtn.classList.contains("on"); };
nextBtn.onclick = () => { autoCmd = true; resetCmd = true; };
const KEYS = "wasdqe";
function paint() {
  document.querySelectorAll(".k").forEach(el => el.classList.toggle("on", down.has(el.dataset.k)));
}
addEventListener("keydown", ev => {
  const k = ev.key.toLowerCase();
  if (k === " ") { down.clear(); autoCmd = false; ev.preventDefault(); }
  else if (k === "p" && !ev.repeat) { autoCmd = !autoBtn.classList.contains("on"); }
  else if (k === "r" && !ev.repeat) { autoCmd = true; resetCmd = true; }
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
  // What the keys ask for: forward (W/S), turn (A/D), strafe (Q/E).
  const fwd = (down.has("w") ? 1 : 0) - (down.has("s") ? 1 : 0);
  const turn = (down.has("a") ? 1 : 0) - (down.has("d") ? 1 : 0);
  const side = (down.has("q") ? 1 : 0) - (down.has("e") ? 1 : 0);
  // The robot's axes are swapped by how the motors are wired (measured on the robot):
  // what the server calls vx turns, vy drives forward and wz strafes.
  const vx = -turn, vy = fwd, wz = side;
  const speed = down.has("shift") ? 1 : spd.value / 100;
  const payload = { vx, vy, wz, speed };
  if (autoCmd !== null) { payload.auto = autoCmd; autoCmd = null; }
  if (resetCmd) { payload.reset_auto = true; resetCmd = false; }
  try {
    const r = await fetch("/cmd", { method: "POST", body: JSON.stringify(payload) });
    const s = await r.json();
    autoBtn.classList.toggle("on", !!s.auto);
    autoBtn.textContent = `Auto (P): ${s.auto ? "ON" : "OFF"}`;
    st.className = "";
    const tgt = s.target ? ` (${s.target})` : "";
    st.textContent = `state: ${s.state}${tgt} · delivered ${s.delivered} ` +
      `(picked ${s.collected}) · line ${s.line} · speed=${Math.round(speed * 100)}% · ` +
      `motors ${s.motors} · camera ${s.camera ? "yes" : "no"}`;
  } catch (e) { st.className = "off"; st.textContent = "no connection to the robot"; }
}
setInterval(tick, 100);
</script></body></html>
"""


def line_label(drive: Drive) -> str:
    if drive._line is None:
        return "no sensor"
    return "BLACK" if drive.line_black else "white"


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
                if "auto" in data or "reset_auto" in data:
                    drive.set_auto(data.get("auto"), bool(data.get("reset_auto", False)))
                drive.command(
                    float(data.get("vx", 0)),
                    float(data.get("vy", 0)),
                    float(data.get("wz", 0)),
                    float(data.get("speed", 0)),
                )
            except (ValueError, TypeError):
                self.send_error(400)
                return
            body = json.dumps(
                {
                    "motors": motors_label,
                    "camera": camera.ok,
                    "auto": drive.auto_enabled,
                    "state": drive.auto_state,
                    "target": drive.target_info,
                    "collected": drive.collected_count,
                    "delivered": drive.delivered_count,
                    "line": line_label(drive),
                }
            ).encode()
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
    parser.add_argument("--fake", action="store_true", help="fake motors and test camera")
    parser.add_argument("--host", help="0.0.0.0 to open it from another machine on the network")
    parser.add_argument("--port", type=int)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    # SIGTERM (kill, systemd) also stops the motors before exiting
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    cfg = load_config(args.config)
    mcfg, ccfg, scfg = cfg["motors"], cfg["camera"], cfg["server"]

    if args.fake:
        motors, label = FakeMotors(), "fake"
    elif not pins_configured(mcfg):
        log.warning("missing pins in %s: motors in fake mode", args.config)
        motors, label = FakeMotors(), "fake (missing pins)"
    elif mcfg.get("driver") == "fusion_hat":
        motors, label = FusionHatMotors(mcfg), "Fusion HAT+"
    else:
        motors, label = HatMotors(mcfg), "HAT"

    lcfg = cfg.get("line_sensor") or {}
    line = None
    if not args.fake and lcfg.get("pin") is not None:
        line = LineSensor(lcfg)
    elif not args.fake:
        log.warning("line_sensor.pin is empty in %s: black line not detected", args.config)

    drive = Drive(
        motors, mcfg["max_duty_ratio"], mcfg["cmd_timeout_s"], cfg.get("auto"), line_sensor=line
    )
    camera = Camera(ccfg, fake=args.fake)
    drive.set_camera(camera)
    host = args.host or scfg["host"]
    port = args.port or scfg["port"]
    server = ThreadingHTTPServer((host, port), make_handler(drive, camera, label))
    server.daemon_threads = True
    shown = "localhost" if host == "127.0.0.1" else host
    log.info("open http://%s:%d (Ctrl+C to quit)", shown, port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        log.info("stopping and closing")
        server.server_close()
        camera.close()
        drive.close()
        if line is not None:
            line.close()


if __name__ == "__main__":
    main()
