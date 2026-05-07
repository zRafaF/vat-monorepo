import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

class CameraGatewayNode(Node):
    def __init__(self, video_client):
        super().__init__('unitree_camera_gateway')
        
        # Use the client passed in from main()
        self.client = video_client
        
        # Create a CompressedImage publisher
        self.topic_name = '/utlidar/front_camera/compressed'
        self.pub = self.create_publisher(CompressedImage, self.topic_name, 10)
        
        # Run the poll loop on a fast timer (~30fps)
        self.timer = self.create_timer(0.033, self.poll_camera)
        self.get_logger().info(f"Camera Gateway Ready. Publishing to {self.topic_name}")

    def poll_camera(self):
        # ONLY fetch from the SDK if the dynamic bridge is listening
        if self.pub.get_subscription_count() > 0:
            code, data = self.client.GetImageSample()
            
            if code == 0:
                # The 'data' is already a compressed JPEG
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "front_camera_link"
                msg.format = "jpeg"
                msg.data = bytes(data) 
                
                self.pub.publish(msg)

def main(args=None):
    # 1. INITIALIZE UNITREE SDK FIRST (Before ROS 2!)
    # If it still crashes, try replacing the empty string with 'eth0' or 'lo'
    network_interface = "" 
    if len(sys.argv) > 1:
        network_interface = sys.argv[1]
        
    try:
        if network_interface:
            ChannelFactoryInitialize(0, network_interface)
        else:
            ChannelFactoryInitialize(0)
    except Exception as e:
        print(f"Failed to init Unitree SDK. Try running: python3 front_camera.py eth0")
        print(f"Error: {e}")
        return

    # Setup the Video Client
    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()

    # 2. INITIALIZE ROS 2 SECOND
    rclpy.init(args=args)
    
    # 3. Pass the working client into the node
    node = CameraGatewayNode(client)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()