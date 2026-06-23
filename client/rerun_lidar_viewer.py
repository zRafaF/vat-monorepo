import os
import rerun as rr
import zenoh
import numpy as np

# NEW: Import the Typestore system from rosbags
from rosbags.typesys import Stores, get_typestore

# --- CONFIGURATION ---
ZENOH_ROUTER = os.environ.get('ZENOH_ROUTER', 'tcp/100.125.156.19:7447')
ROBOT_NAME = os.environ.get('ROBOT_NAME', 'jetson_robot')

# Initialize the Typestore for ROS 2 Humble (this loads the standard ROS 2 message definitions)
typestore = get_typestore(Stores.ROS2_HUMBLE)

TOPICS = {
    "1": {"name": "Raw Cloud", "ros_topic": "/utlidar/cloud", "rr_path": "lidar/raw"},
    "2": {"name": "Height Map", "ros_topic": "/utlidar/height_map", "rr_path": "lidar/height_map"},
    "3": {"name": "Voxel Map", "ros_topic": "/utlidar/voxel_map", "rr_path": "lidar/voxel_map"}
}

current_subscriber = None

def pointcloud_callback(sample, rr_path):
    """Deserializes the payload using pure Python and logs it to Rerun."""
    try:
        payload_bytes = bytes(sample.payload)
        
        # 1. Deserialize the CDR payload using the typestore (NO ROS REQUIRED!)
        msg = typestore.deserialize_cdr(payload_bytes, "sensor_msgs/msg/PointCloud2")
        
        # 2. Extract XYZ using pure Numpy
        # PointCloud2 'data' is a flat byte array. We cast it to float32.
        raw_data = np.frombuffer(msg.data, dtype=np.float32)
        
        # Calculate how many floats make up a single point (float32 is 4 bytes)
        floats_per_point = msg.point_step // 4
        num_points = msg.width * msg.height
        
        # Reshape the 1D array into a 2D grid of points
        points = raw_data.reshape((num_points, floats_per_point))
        
        # Extract just the first 3 columns (X, Y, Z)
        xyz = points[:, :3]
        
        # Filter out invalid points (NaNs) which are very common in LiDAR
        valid_xyz = xyz[~np.isnan(xyz).any(axis=1)]
        
        if len(valid_xyz) > 0:
            # 3. Log to Rerun
            rr.log(rr_path, rr.Points3D(valid_xyz, radii=0.05))
            
    except Exception as e:
        print(f"[Error] Failed to process point cloud: {e}")

def switch_topic(session, choice_key):
    """Handles the toggling of Zenoh subscriptions to save bandwidth."""
    global current_subscriber
    
    if current_subscriber is not None:
        current_subscriber.undeclare()
        current_subscriber = None
        rr.log("lidar", rr.Clear.recursive())

    topic_info = TOPICS.get(choice_key)
    if topic_info:
        zenoh_key = f"{ROBOT_NAME}/rt{topic_info['ros_topic']}"
        rr_path = topic_info['rr_path']
        
        print(f"\n[Requesting] Telling robot to start streaming: {topic_info['name']}...")
        
        cb = lambda sample: pointcloud_callback(sample, rr_path)
        current_subscriber = session.declare_subscriber(zenoh_key, cb)
        print(f"Subscribed to Zenoh key: {zenoh_key}")
    else:
        print("\nInvalid choice. Stream paused.")

def main():
    # Initialize Rerun viewer
    rr.init("lidar_poc", spawn=True)
    rr.log("lidar", rr.ViewCoordinates.RIGHT_HAND_Z_UP, timeless=True)

    # Initialize Zenoh
    zenoh.init_log_from_env_or("error")
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
    session = zenoh.open(conf)
    print(f"Connected to Zenoh Router at {ZENOH_ROUTER}")

    switch_topic(session, "1")

    try:
        while True:
            print("\n--- View Toggle ---")
            print("1: Raw LiDAR Cloud")
            print("2: Height Map")
            print("3: Voxel Map")
            print("0: Pause Stream (Unsubscribe)")
            
            choice = input("Select a view (0-3): ").strip()
            
            if choice in ["1", "2", "3", "0"]:
                switch_topic(session, choice)
            else:
                print("Invalid input.")
                
    except KeyboardInterrupt:
        print("\nShutting down client...")
    finally:
        if current_subscriber:
            current_subscriber.undeclare()
        session.close()

if __name__ == '__main__':
    main()