"""
VAT - RGBD (RealSense D435i) relay:  ROS2 (DDS) -> Zenoh single frames.

Bridges the realsense2_camera node's depth/color topics to the client as ONE latest
frame at a time (no cloud, no accumulation), so the operator sees a panel in front of
the robot showing what the forward camera sees. The client picks the ACTIVE stream via
an RgbdRequest, so we only ever encode+send what is on screen:

  * depth : 16-bit depth (mm) -> 8-bit scaled over [0, max_range] (single channel, PNG).
            Out-of-range / invalid -> 0 (client renders black; colormap does near=warm).
  * color : RGB -> JPEG.

Client-configurable over Zenoh (RgbdRequest): kind, fps, max_range, range-gate. The
relay only publishes while a viewer is active (recent request) and can range-gate
(skip frames when nothing is within max_range) to save uplink.

Runs inside the robot container next to theta_camera/pose_fuser. It subscribes to the
realsense topics over CycloneDDS (same discovery as the Go2 bridge), so realsense2_camera
must be running (in this container via start.sh, or on the host).
"""

from __future__ import annotations

import math
import os
import threading
import time
import logging

import numpy as np
import cv2
import zenoh

import vat_protocol as proto

logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO"),
                    format="%(asctime)s [%(levelname)s] %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("rgbd")

ROBOT_NAME    = os.environ.get("ROBOT_NAME", "go2")
ZENOH_CONNECT = os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:7447")
SO_SNDBUF     = os.environ.get("RGBD_SO_SNDBUF", "262144").strip()

DEPTH_TOPIC   = os.environ.get("RGBD_DEPTH_TOPIC", "/camera/camera/depth/image_rect_raw")
COLOR_TOPIC   = os.environ.get("RGBD_COLOR_TOPIC", "/camera/camera/color/image_raw")
DEPTH_INFO    = os.environ.get("RGBD_DEPTH_INFO_TOPIC", "/camera/camera/depth/camera_info")
COLOR_INFO    = os.environ.get("RGBD_COLOR_INFO_TOPIC", "/camera/camera/color/camera_info")

SEND_WIDTH    = int(os.environ.get("RGBD_SEND_WIDTH", "424"))     # downscale long side; 0=native
JPEG_QUALITY  = int(os.environ.get("RGBD_JPEG_QUALITY", "70"))
VIEWER_TIMEOUT_S = float(os.environ.get("RGBD_VIEWER_TIMEOUT_S", "5.0"))
# Fallback intrinsics (deg) if camera_info hasn't arrived (D435: depth ~87x58, color ~69x42).
DEF_DEPTH_HFOV = float(os.environ.get("RGBD_DEPTH_HFOV", "87"))
DEF_DEPTH_VFOV = float(os.environ.get("RGBD_DEPTH_VFOV", "58"))
DEF_COLOR_HFOV = float(os.environ.get("RGBD_COLOR_HFOV", "69"))
DEF_COLOR_VFOV = float(os.environ.get("RGBD_COLOR_VFOV", "42"))


def _fov_from_info(width, height, fx, fy):
    hf = math.degrees(2 * math.atan(width / (2 * fx))) if fx else 0.0
    vf = math.degrees(2 * math.atan(height / (2 * fy))) if fy else 0.0
    return hf, vf


class RgbdRelay:
    def __init__(self):
        self._lock = threading.Lock()
        self._depth = None          # (H,W) uint16 mm
        self._color = None          # (H,W,3) rgb uint8
        self._depth_fov = (DEF_DEPTH_HFOV, DEF_DEPTH_VFOV)
        self._color_fov = (DEF_COLOR_HFOV, DEF_COLOR_VFOV)

        # request state (client-configurable over Zenoh)
        self._kind = proto.RGBD_KIND_OFF
        self._fps = 20
        self._max_range_mm = 4000
        self._range_gate = False
        self._last_req_t = 0.0
        self._out_seq = 0
        self._pub_count = 0
        self._last_stat_t = 0.0
        self._stop = threading.Event()

        self._z = self._open_session()
        self._pub = self._declare_pub()
        K = proto.keys(ROBOT_NAME)
        self._z.declare_subscriber(K["rgbd_request"], self._on_request)
        log.info(f"[rgbd] relay: req<-'{K['rgbd_request']}' frame->'{K['rgbd_frame']}' "
                 f"depth='{DEPTH_TOPIC}' color='{COLOR_TOPIC}'")

    # -- Zenoh ---------------------------------------------------------------
    def _open_session(self):
        conf = zenoh.Config()
        endpoint = ZENOH_CONNECT
        if SO_SNDBUF and SO_SNDBUF != "0":
            sep = ";" if "#" in endpoint else "#"
            endpoint = f"{endpoint}{sep}so_sndbuf={SO_SNDBUF}"
        conf.insert_json5("connect/endpoints", f'["{endpoint}"]')
        conf.insert_json5("mode", '"peer"')
        while not self._stop.is_set():
            try:
                return zenoh.open(conf)
            except Exception as e:
                log.warning(f"[rgbd] Zenoh connect failed: {e} - retry 5s")
                time.sleep(5)

    def _declare_pub(self):
        key = proto.keys(ROBOT_NAME)["rgbd_frame"]
        for kwargs in (
            dict(congestion_control=zenoh.CongestionControl.DROP,
                 reliability=zenoh.Reliability.BEST_EFFORT, priority=zenoh.Priority.DATA),
            dict(congestion_control=zenoh.CongestionControl.DROP, priority=zenoh.Priority.DATA),
            dict(congestion_control=zenoh.CongestionControl.DROP),
        ):
            try:
                return self._z.declare_publisher(key, **kwargs)
            except TypeError:
                continue
        return self._z.declare_publisher(key)

    def _on_request(self, sample):
        try:
            r = proto.unpack_rgbd_request(bytes(sample.payload))
        except proto.ProtocolError as e:
            log.warning(f"[rgbd] bad request: {e}")
            return
        with self._lock:
            self._kind = int(r.kind)
            self._fps = max(1, min(30, int(r.fps)))
            self._max_range_mm = max(200, int(r.max_range_mm))
            self._range_gate = bool(r.range_gate)
            self._last_req_t = time.time()

    # -- ROS2 intake (called from the rclpy thread) --------------------------
    def on_depth(self, arr_u16, hfov, vfov):
        with self._lock:
            self._depth = arr_u16
            if hfov and vfov:
                self._depth_fov = (hfov, vfov)

    def on_color(self, rgb_u8, hfov, vfov):
        with self._lock:
            self._color = rgb_u8
            if hfov and vfov:
                self._color_fov = (hfov, vfov)

    # -- publish loop --------------------------------------------------------
    def _viewer_active(self, now, last_req):
        if VIEWER_TIMEOUT_S <= 0:
            return last_req > 0.0
        return (now - last_req) < VIEWER_TIMEOUT_S

    @staticmethod
    def _downscale(img, interp):
        if SEND_WIDTH and img.shape[1] > SEND_WIDTH:
            h = int(round(img.shape[0] * SEND_WIDTH / img.shape[1]))
            return cv2.resize(img, (SEND_WIDTH, h), interpolation=interp)
        return img

    def _encode_depth(self, depth, max_mm):
        depth = self._downscale(depth, cv2.INTER_NEAREST)
        valid = (depth > 0) & (depth <= max_mm)
        v = np.zeros(depth.shape, np.uint8)
        if valid.any():
            scaled = depth.astype(np.float32) * (254.0 / float(max_mm)) + 1.0
            v[valid] = np.clip(scaled[valid], 1, 255).astype(np.uint8)
            min_mm = int(depth[valid].min())
        else:
            min_mm = 0
        ok, buf = cv2.imencode(".png", v)
        if not ok:
            return None
        return buf.tobytes(), v.shape[1], v.shape[0], min_mm, bool(valid.any())

    def _encode_color(self, rgb):
        rgb = self._downscale(rgb, cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", rgb[:, :, ::-1], [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if not ok:
            return None
        return buf.tobytes(), rgb.shape[1], rgb.shape[0]

    def run(self):
        log.info("[rgbd] publish loop started.")
        while not self._stop.is_set():
            now = time.time()
            with self._lock:
                kind, fps, max_mm = self._kind, self._fps, self._max_range_mm
                gate, last_req = self._range_gate, self._last_req_t
                depth, color = self._depth, self._color
                dfov, cfov = self._depth_fov, self._color_fov
            period = 1.0 / max(1, fps)
            if now - self._last_stat_t >= 10.0:
                self._last_stat_t = now
                log.info(f"[rgbd] published={self._pub_count} kind={kind} fps={fps} "
                         f"active={self._viewer_active(now, last_req)} "
                         f"depth={'y' if depth is not None else 'N'} color={'y' if color is not None else 'N'}")
            if kind == proto.RGBD_KIND_OFF or not self._viewer_active(now, last_req):
                time.sleep(min(0.2, period))
                continue
            try:
                self._encode_publish(kind, max_mm, gate, depth, color, dfov, cfov)
            except Exception:
                log.exception("[rgbd] encode/publish error")
                time.sleep(0.1)
            time.sleep(period)

    def _encode_publish(self, kind, max_mm, gate, depth, color, dfov, cfov):
        min_mm = 0
        if kind == proto.RGBD_KIND_DEPTH:
            if depth is None:
                return
            enc = self._encode_depth(depth, max_mm)
            if enc is None:
                return
            payload, w, h, min_mm, in_range = enc
            if gate and not in_range:
                return                         # nothing within range -> save uplink
            hfov, vfov = dfov
            codec = proto.RGBD_CODEC_PNG
        elif kind == proto.RGBD_KIND_COLOR:
            if color is None:
                return
            # range-gate on color uses the depth min if we have depth
            if gate and depth is not None:
                dv = depth[(depth > 0) & (depth <= max_mm)]
                if dv.size == 0:
                    return
                min_mm = int(dv.min())
            enc = self._encode_color(color)
            if enc is None:
                return
            payload, w, h = enc
            hfov, vfov = cfov
            codec = proto.RGBD_CODEC_JPEG
        else:
            return
        self._out_seq += 1
        f = proto.RgbdFrame(seq=self._out_seq, timestamp_ns=time.time_ns(), kind=kind,
                            codec=codec, width=w, height=h, hfov_deg=hfov, vfov_deg=vfov,
                            max_range_mm=max_mm, min_depth_mm=min_mm, payload=payload)
        buf = proto.pack_rgbd_frame(f)
        try:
            self._pub.put(buf, encoding=proto.ENC_RGBF)
        except TypeError:
            self._pub.put(buf)
        self._pub_count += 1

    def close(self):
        self._stop.set()
        try:
            self._z.close()
        except Exception:
            pass


# ── ROS2 node: subscribe realsense topics, feed the relay ────────────────────
def _decode_image(msg):
    """sensor_msgs/Image -> numpy. Returns (array, encoding_str)."""
    h, w = msg.height, msg.width
    enc = msg.encoding
    data = np.frombuffer(bytes(msg.data), np.uint8)
    if enc in ("16UC1", "mono16"):
        return data.view(np.uint16).reshape(h, w), enc
    if enc in ("rgb8", "bgr8"):
        img = data.reshape(h, w, 3)
        return (img if enc == "rgb8" else img[:, :, ::-1].copy()), enc
    if enc in ("mono8",):
        return data.reshape(h, w), enc
    # last resort: assume 3-channel
    try:
        return data.reshape(h, w, -1), enc
    except Exception:
        return None, enc


def main():
    relay = RgbdRelay()
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, CameraInfo
    except Exception as e:
        log.error(f"[rgbd] rclpy/sensor_msgs unavailable ({e}); is this the ROS2 container? Exiting.")
        relay.close()
        return

    class Sub(Node):
        def __init__(self):
            super().__init__("vat_rgbd_relay")
            self._dfov = (0.0, 0.0)
            self._cfov = (0.0, 0.0)
            self.create_subscription(Image, DEPTH_TOPIC, self._depth_cb, qos_profile_sensor_data)
            self.create_subscription(Image, COLOR_TOPIC, self._color_cb, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, DEPTH_INFO, self._dinfo_cb, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, COLOR_INFO, self._cinfo_cb, qos_profile_sensor_data)
            self.get_logger().info("vat_rgbd_relay subscribed to realsense topics")

        def _dinfo_cb(self, m):
            self._dfov = _fov_from_info(m.width, m.height, m.k[0], m.k[4])

        def _cinfo_cb(self, m):
            self._cfov = _fov_from_info(m.width, m.height, m.k[0], m.k[4])

        def _depth_cb(self, m):
            arr, enc = _decode_image(m)
            if arr is not None and arr.dtype == np.uint16:
                relay.on_depth(arr, *self._dfov)

        def _color_cb(self, m):
            arr, enc = _decode_image(m)
            if arr is not None and arr.ndim == 3:
                relay.on_color(np.ascontiguousarray(arr), *self._cfov)

    rclpy.init()
    node = Sub()
    pub_thread = threading.Thread(target=relay.run, name="rgbd-pub", daemon=True)
    pub_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        relay.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
