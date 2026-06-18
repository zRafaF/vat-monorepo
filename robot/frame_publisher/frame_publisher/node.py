"""
DEPRECATED — do not use.
This ROS2 node was superseded by robot/frame_decimator.py which runs inside
the bridge Docker container (Python 3.10+) and supports modern Zenoh.
The file is kept for historical reference only.

Original description:
VAT — PRISM Frame Publisher
============================
ROS2 node that sits between the equirectangular converter and the Zenoh bridge.
It subscribes to the equirectangular output topic, throttles it to a
configurable rate, JPEG-compresses each frame, and re-publishes on
/prism/camera/frame so the DynamicZenohBridge picks it up and streams it to
the server.

Throttle rate is configurable in three ways (in decreasing priority):
  1. Live Zenoh query  — a client writes to  go2/rt/prism/config/throttle_fps
     (any float string, e.g. "3.0") and the node reads it on the next cycle.
  2. ROS2 parameter    — --ros-args -p throttle_fps:=3.0
  3. Environment var   — THROTTLE_FPS=3.0
  4. Default           — 3.0 Hz

Topic map
---------
  Subscriptions:
    /equirectangular/image  (sensor_msgs/Image, BGR8 or RGB8)

  Publications:
    /prism/camera/frame     (sensor_msgs/CompressedImage, JPEG)

ROS2 launch (from robot bringup)
----------------------------------
  <node pkg="frame_publisher" exec="frame_publisher_node" name="frame_publisher">
    <param name="throttle_fps"   value="3.0"/>
    <param name="jpeg_quality"   value="85"/>
    <param name="zenoh_router"   value="tcp/server-ip:7447"/>
    <param name="robot_name"     value="go2"/>
  </node>
"""

from __future__ import annotations

import os
import time
import threading
import logging
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage

import zenoh

log = logging.getLogger(__name__)

# Sensor QoS: best-effort, volatile (matches camera driver)
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)

# Reliable QoS for the compressed output (for Zenoh bridge re-transmission)
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=2,
)


class FramePublisherNode(Node):
    """Throttle + JPEG-compress equirectangular frames for the PRISM server."""

    def __init__(self):
        super().__init__("frame_publisher")

        # ── Declare parameters ───────────────────────────────────────────────
        self.declare_parameter("throttle_fps",  float(os.environ.get("THROTTLE_FPS", "3.0")))
        self.declare_parameter("jpeg_quality",  int(os.environ.get("JPEG_QUALITY",   "85")))
        self.declare_parameter("zenoh_router",  os.environ.get("ZENOH_ROUTER",  "tcp/127.0.0.1:7447"))
        self.declare_parameter("robot_name",    os.environ.get("ROBOT_NAME",    "go2"))
        self.declare_parameter("input_topic",   "/equirectangular/image")
        self.declare_parameter("output_topic",  "/prism/camera/frame")

        self._read_params()

        # ── State ────────────────────────────────────────────────────────────
        self._last_published: float = 0.0
        self._frame_count = 0
        self._dropped_count = 0
        self._lock = threading.Lock()
        self._target_interval: float = 1.0 / self._throttle_fps

        # ── ROS2 subscriber / publisher ──────────────────────────────────────
        input_topic  = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._pub = self.create_publisher(CompressedImage, output_topic, RELIABLE_QOS)
        self._sub = self.create_subscription(
            Image, input_topic, self._on_image, SENSOR_QOS)
        self.get_logger().info(
            f"[FramePublisher] {input_topic} → {output_topic} "
            f"@ {self._throttle_fps:.1f} Hz (JPEG q={self._jpeg_quality})"
        )

        # ── Zenoh session for live config updates ────────────────────────────
        self._z: Optional[zenoh.Session] = None
        self._throttle_key = f"{self._robot_name}/rt/prism/config/throttle_fps"
        threading.Thread(target=self._init_zenoh, daemon=True).start()

        # ── Diagnostics timer ────────────────────────────────────────────────
        self.create_timer(10.0, self._log_stats)

    def _read_params(self):
        self._throttle_fps  = self.get_parameter("throttle_fps").value
        self._jpeg_quality  = self.get_parameter("jpeg_quality").value
        self._zenoh_router  = self.get_parameter("zenoh_router").value
        self._robot_name    = self.get_parameter("robot_name").value

    # ── Zenoh live config ────────────────────────────────────────────────────

    def _init_zenoh(self):
        """Connect to Zenoh so we can receive live config updates."""
        try:
            conf = zenoh.Config()
            conf.insert_json5("connect/endpoints", f'["{self._zenoh_router}"]')
            conf.insert_json5("mode", '"client"')
            self._z = zenoh.open(conf)
            self._z.declare_subscriber(self._throttle_key, self._on_throttle_config)
            self.get_logger().info(
                f"[FramePublisher] Zenoh live config on '{self._throttle_key}'"
            )
        except Exception as e:
            self.get_logger().warning(
                f"[FramePublisher] Zenoh init failed (live config disabled): {e}"
            )

    def _on_throttle_config(self, sample):
        """Handle a live throttle_fps update from Zenoh."""
        try:
            val = float(bytes(sample.payload).decode().strip())
            if 0.1 <= val <= 30.0:
                with self._lock:
                    self._throttle_fps = val
                    self._target_interval = 1.0 / val
                self.get_logger().info(
                    f"[FramePublisher] Throttle updated to {val:.2f} Hz via Zenoh"
                )
            else:
                self.get_logger().warning(
                    f"[FramePublisher] Throttle value {val} out of range [0.1, 30.0]"
                )
        except Exception as e:
            self.get_logger().warning(f"[FramePublisher] Bad throttle payload: {e}")

    # ── Image callback ────────────────────────────────────────────────────────

    def _on_image(self, msg: Image):
        now = time.monotonic()
        with self._lock:
            interval = self._target_interval
            quality  = self._jpeg_quality

        # Throttle
        if (now - self._last_published) < interval:
            self._dropped_count += 1
            return

        # Convert ROS2 Image → numpy BGR
        try:
            bgr = self._ros_image_to_bgr(msg)
        except Exception as e:
            self.get_logger().warning(f"[FramePublisher] Image conversion failed: {e}")
            return

        # JPEG encode
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
        ok, buf = cv2.imencode(".jpg", bgr, encode_params)
        if not ok or buf is None:
            self.get_logger().warning("[FramePublisher] JPEG encode failed")
            return

        # Publish
        out = CompressedImage()
        out.header = msg.header   # preserve original timestamp and frame_id
        out.format = "jpeg"
        out.data   = buf.tobytes()
        self._pub.publish(out)

        self._last_published = now
        self._frame_count += 1

    # ── Image format helpers ─────────────────────────────────────────────────

    @staticmethod
    def _ros_image_to_bgr(msg: Image) -> np.ndarray:
        """Convert sensor_msgs/Image to a numpy BGR uint8 array."""
        h, w = msg.height, msg.width
        enc  = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ("rgb8", "rgb"):
            rgb  = data.reshape(h, w, 3)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        elif enc in ("bgr8", "bgr"):
            return data.reshape(h, w, 3)
        elif enc in ("mono8", "8uc1"):
            grey = data.reshape(h, w)
            return cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR)
        elif enc in ("rgba8",):
            rgba = data.reshape(h, w, 4)
            return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
        elif enc in ("bgra8",):
            bgra = data.reshape(h, w, 4)
            return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        else:
            raise ValueError(f"Unsupported encoding: {enc}")

    # ── Diagnostics ──────────────────────────────────────────────────────────

    def _log_stats(self):
        self.get_logger().info(
            f"[FramePublisher] published={self._frame_count} "
            f"dropped={self._dropped_count} "
            f"rate={self._throttle_fps:.1f}Hz"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FramePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
