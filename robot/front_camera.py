import sys
import time
import multiprocessing as mp

# ==========================================
# PROCESS 1: The Unitree SDK Worker
# ==========================================
def sdk_worker(pipe_conn, network_interface):
    # IMPORTANT: We import the SDK *inside* this function so it only 
    # loads into this specific process's memory space.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient

    try:
        if network_interface:
            ChannelFactoryInitialize(0, network_interface)
        else:
            ChannelFactoryInitialize(0)
    except Exception as e:
        print(f"[SDK Worker] Init error: {e}")
        return
        
    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()

    print("[SDK Worker] 🎥 Connected to Robot Camera. Streaming to ROS...")

    try:
        while True:
            code, data = client.GetImageSample()
            if code == 0:
                # Send the raw JPEG bytes through the memory pipe to the ROS process
                pipe_conn.send(bytes(data))
            
            # Cap at ~30fps to prevent overloading the pipe
            time.sleep(0.033) 
    except KeyboardInterrupt:
        pass
    finally:
        pipe_conn.close()

# ==========================================
# PROCESS 2: The ROS 2 Publisher (Main Process)
# ==========================================
def main():
    # IMPORTANT: Set spawn method to ensure totally clean memory spaces
    mp.set_start_method('spawn')

    # Create a high-speed memory pipe to connect the two processes
    ros_conn, sdk_conn = mp.Pipe(duplex=False)

    # Start the SDK in a completely separate background process
    network_interface = sys.argv[1] if len(sys.argv) > 1 else ""
    sdk_process = mp.Process(target=sdk_worker, args=(sdk_conn, network_interface))
    sdk_process.start()

    # Now we are safe to import ROS 2 in the main process
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage

    rclpy.init()
    node = rclpy.create_node('unitree_camera_gateway')
    pub = node.create_publisher(CompressedImage, '/utlidar/front_camera/compressed', 10)
    
    node.get_logger().info("ROS 2 Camera Node Ready. Waiting for video frames...")

    try:
        while rclpy.ok():
            # Check if there is a frame waiting in the pipe
            if ros_conn.poll(timeout=0.1):
                image_bytes = ros_conn.recv()
                
                # Only publish if your dynamic_bridge is actually listening!
                if pub.get_subscription_count() > 0:
                    msg = CompressedImage()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.header.frame_id = "front_camera_link"
                    msg.format = "jpeg"
                    msg.data = image_bytes
                    pub.publish(msg)
            
            # Spin once to process ROS callbacks (like subscription counts)
            rclpy.spin_once(node, timeout_sec=0)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        # Clean up both processes gracefully
        sdk_process.terminate()
        sdk_process.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()