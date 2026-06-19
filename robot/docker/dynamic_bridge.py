import os
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from rosidl_runtime_py.utilities import get_message
import zenoh
import json
import time


class DynamicZenohBridge(Node):
    def __init__(self):
        super().__init__('dynamic_zenoh_bridge')

        # Configuration
        self.robot_prefix = os.environ.get('ROBOT_NAME', 'my_robot')
        zenoh_endpoint = os.environ.get('ZENOH_CONNECT', 'tcp/127.0.0.1:7447')
        log_level_str = os.environ.get('LOG_LEVEL', 'info').lower()

        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", f'["{zenoh_endpoint}"]')
        conf.insert_json5("mode", '"peer"')

        # Retry Loop for Zenoh Connection
        self.z_session = None
        while self.z_session is None:
            try:
                self.get_logger().info(f"Attempting to connect to Zenoh: {zenoh_endpoint}...")
                self.z_session = zenoh.open(conf)
            except Exception as e:
                self.get_logger().warn(f"Server unreachable: {e}. Retrying in 5 seconds...")
                time.sleep(5)

        self.get_logger().info("Successfully connected to Zenoh server!")

        # State Management and Liveliness
        self.zenoh_map = {}
        liveliness_key = f"{self.robot_prefix}/system/liveliness"
        self.liveliness_token = self.z_session.liveliness().declare_token(liveliness_key)

        # Setup Discovery and Timers
        self.z_session.declare_queryable(f"{self.robot_prefix}/system/get_topics", self.handle_topic_query)
        self.discovery_timer = self.create_timer(2.0, self.discover_topics)
        # Periodic forwarded-count so you can SEE whether data is actually flowing.
        self.stats_timer = self.create_timer(10.0, self.log_stats)

        self.get_logger().info(f"Smart Dynamic Bridge [{self.robot_prefix}] Ready.")

    def _match_qos(self, topic_name):
        """Build a subscription QoS compatible with the topic's publisher(s).

        This is the critical fix for camera/sensor topics: they are published
        BEST_EFFORT, and a default RELIABLE subscriber receives NOTHING from a
        best-effort publisher. A best-effort subscriber, by contrast, is
        compatible with BOTH reliable and best-effort publishers — so if any
        publisher offers best-effort we match it.
        """
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        try:
            infos = self.get_publishers_info_by_topic(topic_name)
            if infos:
                if any(i.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
                       for i in infos):
                    qos.reliability = ReliabilityPolicy.BEST_EFFORT
                # Only go transient-local if EVERY publisher is (else incompatible).
                if all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                       for i in infos):
                    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        except Exception as e:
            self.get_logger().debug(f"QoS probe failed for {topic_name}: {e}")
        return qos

    def discover_topics(self):
        """Polls ROS for topics and registers Zenoh publishers with MatchingListeners."""
        current_topics = dict(self.get_topic_names_and_types())

        for topic_name, types in current_topics.items():
            if topic_name not in self.zenoh_map:
                try:
                    msg_class = get_message(types[0])
                    zenoh_key = f"{self.robot_prefix}/rt{topic_name}"

                    z_pub = self.z_session.declare_publisher(
                        zenoh_key,
                        congestion_control=zenoh.CongestionControl.DROP,
                    )

                    self.zenoh_map[topic_name] = {
                        "z_pub": z_pub,
                        "ros_sub": None,
                        "msg_class": msg_class,
                        "type_str": types[0],
                        "count": 0,
                        "qos": None,
                    }

                    listener_cb = self.create_matching_callback(topic_name, msg_class, z_pub)
                    z_pub.declare_matching_listener(listener_cb)

                    self.get_logger().info(f"Registered Zenoh route for: {topic_name}")
                except Exception as e:
                    self.get_logger().debug(f"Could not register {topic_name}: {e}")

    def create_matching_callback(self, topic_name, msg_class, z_pub):
        """Creates a closure to handle matching events for a specific topic."""
        def on_matching_status_update(status: zenoh.MatchingStatus):
            entry = self.zenoh_map[topic_name]
            if status.matching:
                if entry["ros_sub"] is None:
                    qos = self._match_qos(topic_name)
                    entry["qos"] = qos
                    rel = "BEST_EFFORT" if qos.reliability == ReliabilityPolicy.BEST_EFFORT else "RELIABLE"
                    self.get_logger().info(
                        f"Client connected! Subscribing to {topic_name} (QoS={rel})")

                    def cb(msg, entry=entry, z_pub=z_pub):
                        z_pub.put(serialize_message(msg))
                        entry["count"] += 1

                    sub = self.create_subscription(msg_class, topic_name, cb, qos)
                    entry["ros_sub"] = sub
            else:
                if entry["ros_sub"] is not None:
                    self.get_logger().info(f"Clients disconnected. Stopping ROS subscription for {topic_name}")
                    self.destroy_subscription(entry["ros_sub"])
                    entry["ros_sub"] = None

        return on_matching_status_update

    def log_stats(self):
        """Log forwarded-message counts for active subscriptions (data-flow check)."""
        active = [(t, d["count"]) for t, d in self.zenoh_map.items() if d["ros_sub"] is not None]
        if not active:
            return
        summary = ", ".join(f"{t}={c}" for t, c in sorted(active))
        self.get_logger().info(f"[forwarded] {summary}")
        for t, c in active:
            if c == 0:
                self.get_logger().warn(
                    f"[no data] subscribed to {t} but received 0 msgs — check QoS/DDS "
                    f"(is the publisher running? right interface/domain?)")

    def handle_topic_query(self, query):
        """Replies with a list of topics currently capable of being forwarded."""
        available_topics = {topic: data["type_str"] for topic, data in self.zenoh_map.items()}
        payload = json.dumps(available_topics).encode('utf-8')
        query.reply(query.key_expr, payload)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicZenohBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.z_session.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
