import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import serialize_message
import zenoh

class ZenohUplinkNode(Node):
    def __init__(self):
        super().__init__('zenoh_custom_uplink')
        
        # 1. Connect to your Zenoh Router
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", '["tcp/100.125.156.19:7447"]')
        self.get_logger().info("Connecting to Zenoh router...")
        self.z_session = zenoh.open(conf)
        
        # 2. Define your namespace and keys
        self.topic_name = "/utlidar/cloud"
        self.zenoh_key = f"my_robot/rt{self.topic_name}" # e.g. my_robot/rt/utlidar/cloud
        
        # 3. Create Zenoh publisher
        self.z_pub = self.z_session.declare_publisher(self.zenoh_key)
        
        # 4. Subscribe to the local ROS 2 Lidar topic
        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic_name,
            self.ros_callback,
            10 # QoS depth
        )
        self.get_logger().info(f"Bridging local {self.topic_name} to Zenoh {self.zenoh_key}")

    def ros_callback(self, msg):
        # Convert the ROS message directly to raw binary (CDR) bytes
        raw_bytes = serialize_message(msg)
        
        # Blast it over Zenoh
        self.z_pub.put(raw_bytes)

def main(args=None):
    rclpy.init(args=args)
    node = ZenohUplinkNode()
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