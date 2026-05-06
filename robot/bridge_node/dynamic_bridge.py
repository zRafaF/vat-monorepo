import os
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
import zenoh
import json

class DynamicZenohBridge(Node):
    def __init__(self):
        # Initialize ROS 2 Node
        super().__init__('dynamic_zenoh_bridge')
        
        # 1. Configuration from Environment Variables[cite: 3]
        self.robot_prefix = os.environ.get('ROBOT_NAME', 'my_robot')
        zenoh_endpoint = os.environ.get('ZENOH_CONNECT', 'tcp/127.0.0.1:7447')
        log_level_str = os.environ.get('LOG_LEVEL', 'info').lower()
        
        # Set ROS logger level[cite: 3]
        numeric_level = getattr(rclpy.logging.LoggingSeverity, log_level_str.upper(), rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().set_level(numeric_level)

        # 2. Connect to Zenoh and Initialize Native Logging[cite: 3, 6]
        zenoh.init_log_from_env_or(log_level_str)
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", f'["{zenoh_endpoint}"]')
        
        self.get_logger().info(f"Connecting to Zenoh endpoint: {zenoh_endpoint}")
        self.z_session = zenoh.open(conf)
        
        # 3. State Management
        # Renamed to zenoh_map to avoid AttributeError with ROS Node.publishers[cite: 3]
        self.zenoh_map = {} 
        
        # 4. Declare Liveliness Token (Heartbeat for the robot)[cite: 6]
        liveliness_key = f"{self.robot_prefix}/system/liveliness"
        self.liveliness_token = self.z_session.liveliness().declare_token(liveliness_key)
        self.get_logger().info(f"Broadcasting Liveliness on: {liveliness_key}")

        # 5. Setup Discovery Queryable[cite: 3, 9]
        self.z_session.declare_queryable(f"{self.robot_prefix}/system/get_topics", self.handle_topic_query)
        
        # 6. Start ROS Topic Polling (Checks for new topics every 2 seconds)[cite: 3]
        self.discovery_timer = self.create_timer(2.0, self.discover_topics)
        
        self.get_logger().info(f"Smart Dynamic Bridge [{self.robot_prefix}] Ready.")

    def discover_topics(self):
        """Polls ROS for topics and registers Zenoh publishers with MatchingListeners."""
        current_topics = dict(self.get_topic_names_and_types())
        
        for topic_name, types in current_topics.items():
            # Check our local map instead of the restricted Node.publishers attribute[cite: 3]
            if topic_name not in self.zenoh_map:
                try:
                    # Resolve message type for dynamic subscription[cite: 3]
                    msg_class = get_message(types[0])
                    zenoh_key = f"{self.robot_prefix}/rt{topic_name}"
                    
                    # Declare Zenoh Publisher[cite: 11]
                    z_pub = self.z_session.declare_publisher(
                        zenoh_key,
                        congestion_control=zenoh.CongestionControl.DROP
                    )
                    
                    self.zenoh_map[topic_name] = {
                        "z_pub": z_pub,
                        "ros_sub": None,
                        "msg_class": msg_class,
                        "type_str": types[0]
                    }
                    
                    # Attach MatchingListener to automate ROS pub/sub based on Zenoh demand[cite: 11]
                    listener_cb = self.create_matching_callback(topic_name, msg_class, z_pub)
                    z_pub.declare_matching_listener(listener_cb)
                    
                    self.get_logger().info(f"Registered Zenoh route for: {topic_name}")
                except Exception as e:
                    self.get_logger().debug(f"Could not register {topic_name}: {e}")

    def create_matching_callback(self, topic_name, msg_class, z_pub):
        """Creates a closure to handle matching events for a specific topic[cite: 11]."""
        def on_matching_status_update(status: zenoh.MatchingStatus):
            if status.matching:
                # Start ROS subscription only when a Zenoh client is listening[cite: 3, 11]
                if self.zenoh_map[topic_name]["ros_sub"] is None:
                    self.get_logger().info(f"🟢 Client connected! Starting ROS subscription for {topic_name}")
                    cb = lambda msg: z_pub.put(serialize_message(msg))
                    sub = self.create_subscription(msg_class, topic_name, cb, 10)
                    self.zenoh_map[topic_name]["ros_sub"] = sub
            else:
                # Stop ROS subscription when no Zenoh clients are left[cite: 3, 11]
                if self.zenoh_map[topic_name]["ros_sub"] is not None:
                    self.get_logger().info(f"🔴 Clients disconnected. Stopping ROS subscription for {topic_name}")
                    self.destroy_subscription(self.zenoh_map[topic_name]["ros_sub"])
                    self.zenoh_map[topic_name]["ros_sub"] = None
                    
        return on_matching_status_update

    def handle_topic_query(self, query):
        """Replies with a list of topics currently capable of being forwarded[cite: 9]."""
        available_topics = {topic: data["type_str"] for topic, data in self.zenoh_map.items()}
        payload = json.dumps(available_topics).encode('utf-8')
        
        # Correct Zenoh 1.x reply signature[cite: 9]
        query.reply(query.key_expr, payload)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicZenohBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Liveliness token drops automatically when session closes[cite: 6]
        node.z_session.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()