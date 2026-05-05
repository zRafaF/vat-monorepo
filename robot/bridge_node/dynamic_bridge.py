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
        
        # 1. Logging Configuration
        # Read from Environment Variables (standard for Docker)
        log_level_str = os.environ.get('LOG_LEVEL', 'INFO').upper()
        self.debug_per_message = os.environ.get('ENABLE_PER_MESSAGE_DEBUG', 'false').lower() == 'true'
        
        # Set ROS logger level
        numeric_level = getattr(rclpy.logging.LoggingSeverity, log_level_str, rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().set_level(numeric_level)

        # 2. Connect to Zenoh
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", '["tcp/100.125.156.19:7447"]')
        self.get_logger().info(f"Connecting to Zenoh with Log Level: {log_level_str}")
        self.z_session = zenoh.open(conf)
        
        self.active_streams = {} 
        self.robot_prefix = "my_robot"

        # 3. Setup Discovery and Control
        self.z_session.declare_queryable(f"{self.robot_prefix}/system/get_topics", self.handle_topic_query)
        self.z_session.declare_subscriber(f"{self.robot_prefix}/system/control", self.handle_control_cmd)
        
        self.get_logger().info("Dynamic Bridge Ready.")

    def ros_callback(self, msg, pub, topic_name):
        # 5kHz Optimization: The boolean check is much faster than the logger's internal check
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
            
            # Pass topic_name to the callback for specific logging
            callback = lambda msg: self.ros_callback(msg, z_pub, topic_name)
            
            ros_sub = self.create_subscription(msg_class, topic_name, callback, 10)
            self.active_streams[topic_name] = (ros_sub, z_pub)
            
            self.get_logger().info(f"STARTED: {topic_name} -> {zenoh_key}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to start {topic_name}: {str(e)}")


    def stop_stream(self, topic_name):
        if topic_name in self.active_streams:
            ros_sub, z_pub = self.active_streams.pop(topic_name)
            # Destroy the ROS subscription
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



"""
import zenoh
import json
import time

def main():
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", '["tcp/100.125.156.19:7447"]')
    session = zenoh.open(conf)
    
    robot_prefix = "my_robot"

    # 1. Ask the robot what topics it has
    print("Querying robot for available topics...")
    replies = session.get(f"{robot_prefix}/system/get_topics")
    
    available_topics = {}
    for reply in replies:
        try:
            available_topics = json.loads(reply.ok.payload.decode('utf-8'))
            print("--- Available ROS 2 Topics ---")
            for topic, types in available_topics.items():
                print(f" - {topic} ({types[0]})")
            print("------------------------------")
        except Exception:
            pass

    # 2. Tell the robot to START sending the Lidar data
    target_topic = "/utlidar/cloud"
    if target_topic in available_topics:
        print(f"\nRequesting stream for {target_topic}...")
        
        # Setup Zenoh subscriber BEFORE sending the start command
        zenoh_data_key = f"{robot_prefix}/rt{target_topic}"
        
        def data_callback(sample):
            print(f"[Received] {len(sample.payload)} bytes from {sample.key_expr}")
            
        sub = session.declare_subscriber(zenoh_data_key, data_callback)
        
        # Send Start Command
        cmd_pub = session.declare_publisher(f"{robot_prefix}/system/control")
        start_cmd = json.dumps({"action": "start", "topic": target_topic})
        cmd_pub.put(start_cmd.encode('utf-8'))
        
        try:
            print("Listening for data... Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            # Tell the robot to STOP sending data before quitting
            print("\nSending stop command...")
            stop_cmd = json.dumps({"action": "stop", "topic": target_topic})
            cmd_pub.put(stop_cmd.encode('utf-8'))
            time.sleep(0.5) # Give it a moment to send
    else:
        print(f"{target_topic} not found on the robot!")

    session.close()

if __name__ == '__main__':
    main()  
"""