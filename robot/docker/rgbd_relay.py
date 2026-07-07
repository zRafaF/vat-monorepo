"""
VAT - RGBD (RealSense D435i) relay:  ROS2 (DDS) -> Zenoh voxel cloud.

The robot DEPROJECTS the latest depth frame to 3D points, clips to max_range,
voxel-downsamples, colorizes by distance (near=warm), and ships a compact
pack_pcd cloud (zlib + 16-bit quant) to the client. The client just transforms
the points by the live pose and renders them -- no client-side deprojection.

Single frame, no accumulation: each publish replaces the previous cloud. Points
are in the CAMERA-OPTICAL frame (+x right, +y down, +z forward); the client
applies the D435i mount extrinsics + robot pose. Client-configurable over Zenoh
(RgbdRequest): kind (on/off), fps, max_range, range-gate.
"""

from __future__ import annotations

import os
import threading
import time
import logging

import numpy as np
import cv2
import zenoh

import vat_protocol as proto

logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO"),
                    format="%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("rgbd")

ROBOT_NAME    = os.environ.get("ROBOT_NAME", "go2")
ZENOH_CONNECT = os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:7447")
SO_SNDBUF     = os.environ.get("RGBD_SO_SNDBUF", "262144").strip()

def _tlist(env, default):
    return [t.strip() for t in os.environ.get(env, default).split(",") if t.strip()]

DEPTH_TOPICS = _tlist("RGBD_DEPTH_TOPIC",
                      "/camera/camera/depth/image_rect_raw,/camera/depth/image_rect_raw")
DEPTH_INFOS  = _tlist("RGBD_DEPTH_INFO_TOPIC",
                      "/camera/camera/depth/camera_info,/camera/depth/camera_info")

SEND_WIDTH   = int(os.environ.get("RGBD_SEND_WIDTH", "320"))     # downscale depth before deproject
VOXEL_M      = float(os.environ.get("RGBD_VOXEL_M", "0.05"))     # voxel downsample size (m)
MAX_POINTS   = int(os.environ.get("RGBD_MAX_POINTS", "8000"))    # hard cap after downsample
VIEWER_TIMEOUT_S = float(os.environ.get("RGBD_VIEWER_TIMEOUT_S", "5.0"))
# Fallback intrinsics if camera_info hasn't arrived (D435 depth @ 848x480).
DEF_FX = float(os.environ.get("RGBD_DEPTH_FX", "425.0"))
DEF_FY = float(os.environ.get("RGBD_DEPTH_FY", "425.0"))


def _turbo(norm_u8):
    """norm_u8 (N,) uint8 -> (N,3) uint8 RGB, near=warm (input is 0..255 = near..far)."""
    inv = (255 - norm_u8).astype(np.uint8).reshape(-1, 1)
    try:
        cm = cv2.applyColorMap(inv, cv2.COLORMAP_TURBO)
    except Exception:
        cm = cv2.applyColorMap(inv, cv2.COLORMAP_JET)
    return cm.reshape(-1, 3)[:, ::-1]          # BGR->RGB


class RgbdRelay:
    def __init__(self):
        self._lock = threading.Lock()
        self._depth = None          # (H,W) uint16 mm
        self._K = None              # (fx, fy, cx, cy, W, H) at the depth image resolution
        self._depth_n = 0           # count of depth frames received (liveness)

        self._kind = proto.RGBD_KIND_OFF
        self._fps = 10
        self._max_range_mm = 2000
        self._range_gate = False
        self._last_req_t = 0.0
        self._out_seq = 0
        self._pub_count = 0
        self._last_pts = 0
        self._last_stat_t = 0.0
        self._stop = threading.Event()

        self._z = self._open_session()
        self._pub = self._declare_pub()
        K = proto.keys(ROBOT_NAME)
        self._z.declare_subscriber(K["rgbd_request"], self._on_request)
        log.info(f"[rgbd] relay(cloud): req<-'{K['rgbd_request']}' frame->'{K['rgbd_frame']}' "
                 f"depth={DEPTH_TOPICS} voxel={VOXEL_M}m")

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
                log.warning(f"[rgbd] Zenoh connect failed: {e} - retry 5s"); time.sleep(5)

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
            log.warning(f"[rgbd] bad request: {e}"); return
        with self._lock:
            self._kind = int(r.kind)
            self._fps = max(1, min(30, int(r.fps)))
            self._max_range_mm = max(200, int(r.max_range_mm))
            self._range_gate = bool(r.range_gate)
            self._last_req_t = time.time()

    # -- ROS2 intake ---------------------------------------------------------
    def on_depth(self, arr_u16, K):
        with self._lock:
            self._depth = arr_u16
            self._depth_n += 1
            if K is not None:
                self._K = K

    def _viewer_active(self, now, last_req):
        if VIEWER_TIMEOUT_S <= 0:
            return last_req > 0.0
        return (now - last_req) < VIEWER_TIMEOUT_S

    # -- deproject + voxel + colorize + pack --------------------------------
    def _make_cloud(self, depth, K, max_mm):
        """Return (pcd_bytes, n_points, min_depth_mm) or None. Points are in the
        camera-optical frame (metres); colour encodes distance (near=warm)."""
        H0, W0 = depth.shape
        # downscale to SEND_WIDTH (nearest, so depth edges stay crisp) + scale intrinsics
        if SEND_WIDTH and W0 > SEND_WIDTH:
            sc = SEND_WIDTH / float(W0)
            d = cv2.resize(depth, (SEND_WIDTH, int(round(H0 * sc))), interpolation=cv2.INTER_NEAREST)
        else:
            sc = 1.0
            d = depth
        H, W = d.shape
        if K is not None:
            fx, fy, cx, cy = K[0] * sc, K[1] * sc, K[2] * sc, K[3] * sc
        else:
            fx = fy = DEF_FX * sc; cx = W / 2.0; cy = H / 2.0
        valid = (d > 0) & (d <= max_mm)
        if not valid.any():
            return None
        vs, us = np.nonzero(valid)                    # rows (y), cols (x)
        z = d[valid].astype(np.float32) * 0.001       # mm -> m
        x = (us.astype(np.float32) - cx) / fx * z
        y = (vs.astype(np.float32) - cy) / fy * z
        xyz = np.stack([x, y, z], axis=1)             # optical: +x right, +y down, +z fwd
        # distance colour (near=warm) BEFORE downsample
        norm = np.clip(z / (max_mm * 0.001) * 255.0, 0, 255).astype(np.uint8)
        rgb = _turbo(norm)
        # voxel downsample: keep one point per voxel
        if VOXEL_M > 0:
            keys = np.floor(xyz / VOXEL_M).astype(np.int64)
            _, idx = np.unique(keys, axis=0, return_index=True)
            xyz = xyz[idx]; rgb = rgb[idx]
        # hard cap (random subsample if still too many)
        if MAX_POINTS and xyz.shape[0] > MAX_POINTS:
            sel = np.random.choice(xyz.shape[0], MAX_POINTS, replace=False)
            xyz = xyz[sel]; rgb = rgb[sel]
        min_mm = int(d[valid].min())
        buf = proto.pack_pcd(0, xyz.astype(np.float32), rgb, is_snapshot=True)
        return buf, int(xyz.shape[0]), min_mm

    def run(self):
        log.info("[rgbd] publish loop started.")
        while not self._stop.is_set():
            now = time.time()
            with self._lock:
                kind, fps, max_mm = self._kind, self._fps, self._max_range_mm
                gate, last_req = self._range_gate, self._last_req_t
                depth, K, dn = self._depth, self._K, self._depth_n
            period = 1.0 / max(1, fps)
            if now - self._last_stat_t >= 10.0:
                self._last_stat_t = now
                log.info(f"[rgbd] published={self._pub_count} pts={self._last_pts} kind={kind} "
                         f"fps={fps} active={self._viewer_active(now, last_req)} "
                         f"depth_rx={dn} depth={'y' if depth is not None else 'N'}")
            if kind == proto.RGBD_KIND_OFF or not self._viewer_active(now, last_req) or depth is None:
                time.sleep(min(0.2, period)); continue
            try:
                res = self._make_cloud(depth, K, max_mm)
                if res is not None:
                    buf, npts, min_mm = res
                    if not (gate and npts == 0):
                        self._out_seq += 1
                        try:
                            self._pub.put(buf, encoding=proto.ENC_PCD)
                        except TypeError:
                            self._pub.put(buf)
                        self._pub_count += 1
                        self._last_pts = npts
            except Exception:
                log.exception("[rgbd] make_cloud/publish error"); time.sleep(0.1)
            time.sleep(period)

    def close(self):
        self._stop.set()
        try:
            self._z.close()
        except Exception:
            pass


def _decode_depth(msg):
    enc = msg.encoding
    if enc not in ("16UC1", "mono16"):
        return None
    data = np.frombuffer(bytes(msg.data), np.uint8).view(np.uint16)
    return data.reshape(msg.height, msg.width)


def main():
    relay = RgbdRelay()
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, CameraInfo
    except Exception as e:
        log.error(f"[rgbd] rclpy/sensor_msgs unavailable ({e}); is this the ROS2 container? Exiting.")
        relay.close(); return

    class Sub(Node):
        def __init__(self):
            super().__init__("vat_rgbd_relay")
            self._K = None
            self._seen = set()
            for t in DEPTH_TOPICS:
                self.create_subscription(Image, t, self._mk_depth_cb(t), qos_profile_sensor_data)
            for t in DEPTH_INFOS:
                self.create_subscription(CameraInfo, t, self._info_cb, qos_profile_sensor_data)
            self.get_logger().info(f"vat_rgbd_relay(cloud) subscribed: depth={DEPTH_TOPICS}")

        def _info_cb(self, m):
            # K = [fx 0 cx; 0 fy cy; 0 0 1]
            self._K = (float(m.k[0]), float(m.k[4]), float(m.k[2]), float(m.k[5]),
                       int(m.width), int(m.height))

        def _mk_depth_cb(self, topic):
            def cb(m):
                arr = _decode_depth(m)
                if arr is not None:
                    if ("d", topic) not in self._seen:
                        self._seen.add(("d", topic))
                        self.get_logger().info(f"depth frames arriving on {topic} ({m.encoding})")
                    relay.on_depth(arr, self._K)
            return cb

    rclpy.init()
    node = Sub()
    pub_thread = threading.Thread(target=relay.run, name="rgbd-pub", daemon=True)
    pub_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        relay.close(); node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
