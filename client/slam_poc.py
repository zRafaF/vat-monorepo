import os
import time
import copy
import numpy as np
import open3d as o3d
import rerun as rr
import zenoh
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation as R

# --- CONFIGURATION ---
ZENOH_ROUTER = os.environ.get('ZENOH_ROUTER', 'tcp/100.125.156.19:7447')
ROBOT_NAME = os.environ.get('ROBOT_NAME', 'jetson_robot')
LIDAR_TOPIC = "/utlidar/cloud"
IMU_TOPIC = "/utlidar/imu"  

typestore = get_typestore(Stores.ROS2_HUMBLE)

# --- STATE VARIABLES ---
latest_imu_quat = [0, 0, 0, 1]
current_pose = np.eye(4)
global_map = o3d.geometry.PointCloud()
start_time = time.time()

# Auto-Calibration & Tracking State
warmup_points = []
auto_rot_fix = None
trajectory_path = [] # Stores history for the trailing line

# Algorithm Params
VOXEL_SIZE = 0.15 
LOCAL_MAP_RADIUS = 10.0  # Match against walls within 10 meters

def calculate_floor_alignment(points_array):
    """Uses RANSAC to find the floor and calculates the rotation to level it."""
    print("\n--- Running Auto-Calibration ---")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_array)
    
    # 1. RANSAC Plane Segmentation (Finds the largest flat surface)
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,
                                             ransac_n=3,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model
    normal = np.array([a, b, c])

    # Ensure the normal is pointing UP, not DOWN
    if normal[2] < 0:
        normal = -normal

    print(f"Detected Floor Normal: [{normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f}]")

    # Safety Check: If it accidentally found a wall (Z is not dominant), use your manual fallback
    if abs(normal[2]) < 0.5:
        print("WARNING: RANSAC likely found a wall instead of the floor. Using -15 degree pitch fallback.")
        return R.from_euler('y', -15, degrees=True).as_matrix()

    # 2. Calculate the rotation required to snap the normal to [0, 0, 1] (Gravity)
    z_axis = np.array([0, 0, 1])
    rot, _ = R.align_vectors([z_axis], [normal])
    
    print("Auto-Calibration Successful! Floor is now mathematically flat.\n")
    return rot.as_matrix()

def process_imu(sample):
    """Tracks absolute orientation at 250Hz"""
    global latest_imu_quat
    try:
        msg = typestore.deserialize_cdr(bytes(sample.payload), "sensor_msgs/msg/Imu")
        q = msg.orientation
        if not (q.w == 0 and q.x == 0 and q.y == 0 and q.z == 0):
            latest_imu_quat = [q.x, q.y, q.z, q.w]
            
        # Visualize Raw IMU Axes (Madgwick View)
        imu_rot = R.from_quat(latest_imu_quat).as_matrix()
        rr.log("world/imu_raw", rr.Transform3D(mat3x3=imu_rot))
    except:
        pass

def process_lidar(sample):
    global global_map, current_pose, latest_imu_quat
    global warmup_points, auto_rot_fix, trajectory_path
    
    try:
        # 1. Deserialize
        msg = typestore.deserialize_cdr(bytes(sample.payload), "sensor_msgs/msg/PointCloud2")
        raw_data = np.frombuffer(msg.data, dtype=np.float32)
        points = raw_data.reshape((-1, msg.point_step // 4))[:, :3]
        
        valid_pts = points[~np.isnan(points).any(axis=1)].astype(np.float64)
        
        # --- THE WARMUP & CALIBRATION PHASE ---
        if time.time() - start_time < 3.0:
            # Collect points for 3 seconds to get a really good look at the room
            warmup_points.extend(valid_pts)
            return 
            
        if auto_rot_fix is None:
            # Time is up! Run the auto-calibration on the points we collected
            sample_pts = np.array(warmup_points)
            np.random.shuffle(sample_pts)
            # Use a subset (20,000 points) so it computes instantly
            auto_rot_fix = calculate_floor_alignment(sample_pts[:20000])
            
            # Clear memory
            warmup_points = []
            print("Commencing Frame-to-Map Tracking...")
        # ---------------------------------------

        # 2. Apply the RANSAC Auto-Correction
        points = valid_pts @ auto_rot_fix.T
        
        # Crop near-field (0.6m) to hide Ghost Dog and far-field (15m)
        dist = np.linalg.norm(points, axis=1)
        clean_pts = points[(dist > 0.6) & (dist < 15.0)]
        if len(clean_pts) < 50: return

        # 3. Setup Current Scan
        current_pcd = o3d.geometry.PointCloud()
        current_pcd.points = o3d.utility.Vector3dVector(clean_pts)
        current_pcd = current_pcd.voxel_down_sample(VOXEL_SIZE)
        current_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_SIZE*2, max_nn=30))

        # 4. FRAME-TO-MAP GICP
        if len(global_map.points) < 100:
            global_map += current_pcd
        else:
            robot_pos = current_pose[:3, 3]
            
            # Crop a Local Map (10m radius) to match against
            bbox = o3d.geometry.AxisAlignedBoundingBox(robot_pos - LOCAL_MAP_RADIUS, robot_pos + LOCAL_MAP_RADIUS)
            local_map = global_map.crop(bbox)

            if len(local_map.points) > 100:
                # Inject IMU for rotation hint
                imu_rot = R.from_quat(latest_imu_quat).as_matrix()
                init_guess = np.eye(4)
                init_guess[:3, :3] = imu_rot
                init_guess[:3, 3] = robot_pos

                reg = o3d.pipelines.registration.registration_generalized_icp(
                    current_pcd, local_map, VOXEL_SIZE * 2.5, init_guess,
                    o3d.pipelines.registration.TransformationEstimationForGeneralizedICP()
                )
                current_pose = reg.transformation
            
            # Map Maintenance
            transformed_pcd = copy.deepcopy(current_pcd).transform(current_pose)
            global_map += transformed_pcd
            global_map = global_map.voxel_down_sample(VOXEL_SIZE)

        # 5. RERUN LOGGING & VISUALIZATION
        robot_pos = current_pose[:3, 3]
        map_pts = np.asarray(global_map.points)
        
        # Record trajectory path
        trajectory_path.append(robot_pos)
        
        # Color by Elevation
        if len(map_pts) > 0:
            z = map_pts[:, 2]
            colors = np.zeros((len(map_pts), 3))
            colors[:, 0] = (z - np.min(z)) / (np.max(z) - np.min(z) + 1e-5)
            colors[:, 2] = 1.0 - colors[:, 0]
            rr.log("world/map", rr.Points3D(map_pts, colors=colors))

        # Draw Robot Body (Red Sphere)
        rr.log("world/robot/body", rr.Points3D([robot_pos], radii=0.25, colors=[255, 50, 50]))
        
        # Draw SLAM Axes (White)
        rr.log("world/robot/axes", rr.Transform3D(translation=robot_pos, mat3x3=current_pose[:3, :3]))
        
        # Draw Trajectory Line (Yellow)
        if len(trajectory_path) > 1:
            rr.log("world/trajectory", rr.LineStrips3D([trajectory_path], colors=[255, 255, 0]))

    except Exception as e:
        print(f"[SLAM Error]: {e}")

def main():
    rr.init("Auto_Calibrated_Go2_SLAM", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    zenoh.init_log_from_env_or("error")
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
    session = zenoh.open(conf)

    session.declare_subscriber(f"{ROBOT_NAME}/rt{IMU_TOPIC}", process_imu)
    session.declare_subscriber(f"{ROBOT_NAME}/rt{LIDAR_TOPIC}", process_lidar)

    print("Robust SLAM Initiated.")
    print(">>> DO NOT MOVE THE ROBOT FOR 3 SECONDS <<<")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        session.close()

if __name__ == "__main__":
    main()