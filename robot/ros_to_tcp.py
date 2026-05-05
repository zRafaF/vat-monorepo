import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import serialize_message
import socket
import struct

class RosTcpUplink(Node):
    def __init__(self):
        super().__init__('ros_tcp_uplink')
        
        # Connect to the local Python 3.9 Zenoh script
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Keep trying to connect until the Zenoh script is ready
        self.get_logger().info("Waiting for Zenoh TCP server on port 50000...")
        while True:
            try:
                self.sock.connect(('127.0.0.1', 50000))
                self.get_logger().info("Connected to local Zenoh bridge!")
                break
            except ConnectionRefusedError:
                pass

        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud',
            self.ros_callback,
            10
        )

    def ros_callback(self, msg):
        try:
            # 1. Serialize ROS message to raw CDR bytes
            raw_bytes = serialize_message(msg)
            
            # 2. Pack the size of the payload into a 4-byte integer header
            # '<I' means Little-Endian Unsigned Integer
            header = struct.pack('<I', len(raw_bytes))
            
            # 3. Send header, then payload
            self.sock.sendall(header + raw_bytes)
            
        except BrokenPipeError:
            self.get_logger().error("Lost connection to Zenoh script!")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RosTcpUplink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()