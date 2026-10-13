"""Conversión entre sensor_msgs/Image y arreglos de OpenCV, sin cv_bridge.

Solo maneja las codificaciones que usamos: bgr8, rgb8, bgra8, rgba8 y mono8.
"""

import numpy as np
from sensor_msgs.msg import Image

_CHANNELS = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1}


def image_to_bgr(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> arreglo BGR (alto, ancho, 3) de uint8."""
    enc = msg.encoding.lower()
    if enc not in _CHANNELS:
        raise ValueError(f"codificación no soportada: {msg.encoding}")
    ch = _CHANNELS[enc]
    row = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    img = row[:, : msg.width * ch].reshape(msg.height, msg.width, ch)
    if enc == "bgr8":
        return img.copy()
    if enc == "rgb8":
        return img[:, :, ::-1].copy()
    if enc == "bgra8":
        return img[:, :, :3].copy()
    if enc == "rgba8":
        return img[:, :, 2::-1].copy()
    return np.repeat(img, 3, axis=2)


def bgr_to_image(bgr: np.ndarray, header=None) -> Image:
    """Arreglo BGR de uint8 -> sensor_msgs/Image (bgr8)."""
    msg = Image()
    if header is not None:
        msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(bgr, dtype=np.uint8).tobytes()
    return msg
