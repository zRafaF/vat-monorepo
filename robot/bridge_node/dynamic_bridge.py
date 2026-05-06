import os
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
import zenoh
import json

class DynamicZenohBridge(Node):
    def __init__(self):
        super().__init__('dynamic_zenoh_bridge')
        
        # 1. Configuration from Environment Variables
        log_level_str = os.environ.get('LOG_LEVEL', 'INFO').upper()
        self.robot_prefix = os.environ.get('ROBOT_NAME', 'my_robot')
        zenoh_endpoint = os.environ.get('ZENOH_CONNECT', 'tcp/127.0.0.1:7447')
        self.debug_per_message = os.environ.get('ENABLE_PER_MESSAGE_DEBUG', 'false').lower() == 'true'
        
        # Set ROS logger level
        numeric_level = getattr(rclpy.logging.LoggingSeverity, log_level_str, rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().set_level(numeric_level)

        # 2. Connect to Zenoh
        conf = zenoh.Config()
        # Use provided endpoint environment variable
        conf.insert_json5("connect/endpoints", f'["{zenoh_endpoint}"]')
        
        self.get_logger().info(f"Connecting to Zenoh endpoint: {zenoh_endpoint}")
        self.z_session = zenoh.open(conf)
        
        self.active_streams = {} 

        # 3. Setup Discovery and Control
        self.z_session.declare_queryable(f"{self.robot_prefix}/system/get_topics", self.handle_topic_query)
        self.z_session.declare_subscriber(f"{self.robot_prefix}/system/control", self.handle_control_cmd)
        
        self.get_logger().info(f"Dynamic Bridge [{self.robot_prefix}] Ready.")

    def handle_topic_query(self, query):
        """Returns a JSON list of all currently available ROS 2 topics."""
        topics = self.get_topic_names_and_types()
        payload = json.dumps(dict(topics)).encode('utf-8')
        query.reply(zenoh.Sample(query.key_expr, payload))

    def handle_control_cmd(self, sample):
        """Handles start/stop commands via Zenoh."""
        try:
            data = json.loads(sample.payload.decode('utf-8'))
            action = data.get("action")
            topic = data.get("topic")

            if action == "start":
                self.start_stream(topic)
            elif action == "stop":
                self.stop_stream(topic)
        except Exception as e:
            self.get_logger().error(f"Control command error: {e}")

    def ros_callback(self, msg, pub, topic_name):
        if self.debug_per_message:
            self.get_logger().debug(f"Forwarding msg on {topic_name}")
        pub.put(serialize_message(msg))

    def start_stream(self, topic_name):
        if topic_name in self.active_streams:
            return

        topic_list = dict(self.get_topic_names_and_types())
        if topic_name not in topic_list:
            self.get_logger().error(f"Topic {topic_name} not found!")
            return
            
        topic_type_str = topic_list[topic_name][0]
        
        try:
            msg_class = get_message(topic_type_str)
            zenoh_key = f"{self.robot_prefix}/rt{topic_name}"
            z_pub = self.z_session.declare_publisher(zenoh_key)
            
            callback = lambda msg: self.ros_callback(msg, z_pub, topic_name)
            ros_sub = self.create_subscription(msg_class, topic_name, callback, 10)
            self.active_streams[topic_name] = (ros_sub, z_pub)
            
            self.get_logger().info(f"STARTED: {topic_name} -> {zenoh_key}")
        except Exception as e:
            self.get_logger().error(f"Failed to start {topic_name}: {str(e)}")

    def stop_stream(self, topic_name):
        if topic_name in self.active_streams:
            ros_sub, z_pub = self.active_streams.pop(topic_name)
            self.destroy_subscription(ros_sub)
            self.get_logger().info(f"Stopped forwarding: {topic_name}")

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