import os
import time
import zenoh
from rosbags.typesys import Stores, get_typestore

# --- CONFIGURATION ---
ZENOH_ROUTER = os.environ.get('ZENOH_ROUTER', 'tcp/100.125.156.19:7447')
ROBOT_NAME = os.environ.get('ROBOT_NAME', 'jetson_robot')

# All potential odometry sources based on the robot's topics
ODOM_TOPICS = [
    "/lio_sam_ros2/mapping/odometry",
    "/utlidar/robot_odom",
    "/uslam/frontend/odom",
    "/uslam/localization/odom"
]

# Initialize the Typestore for ROS 2 Humble
typestore = get_typestore(Stores.ROS2_HUMBLE)

def create_odom_callback(topic_name):
    """Creates a custom callback to track which topic sent the data."""
    def callback(sample):
        try:
            # Deserialize the raw bytes into a ROS Odometry message
            msg = typestore.deserialize_cdr(bytes(sample.payload), "nav_msgs/msg/Odometry")
            pos = msg.pose.pose.position
            
            # Print the X, Y, Z coordinates rounded to 3 decimal places
            print(f"[{topic_name:30}] X: {pos.x: .3f} | Y: {pos.y: .3f} | Z: {pos.z: .3f}")
            
        except Exception as e:
            print(f"[{topic_name}] Failed to decode: {e}")
            
    return callback

def main():
    # Initialize Zenoh
    zenoh.init_log_from_env_or("error")
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
    session = zenoh.open(conf)
    
    print(f"Connected to Zenoh router at {ZENOH_ROUTER}")
    print(f"Sniffing Odometry on {ROBOT_NAME}...\n")
    
    subscribers = []
    
    # Subscribe to all 4 topics simultaneously
    for topic in ODOM_TOPICS:
        zenoh_key = f"{ROBOT_NAME}/rt{topic}"
        cb = create_odom_callback(topic)
        sub = session.declare_subscriber(zenoh_key, cb)
        subscribers.append(sub)
        print(f"Listening to: {topic}")

    print("\n>>> PUSH OR DRIVE THE ROBOT NOW <<<\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping sniffer...")
    finally:
        # Clean up Zenoh subscriptions
        for sub in subscribers:
            sub.undeclare()
        session.close()

if __name__ == '__main__':
    main()