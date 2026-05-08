import argparse
import numpy as np
import rerun as rr
import os

def visualize_slam_reconstruction(map_path, trajectory_path):
    print("🚀 Initializing Rerun Visualizer...")
    rr.init("Equi-360_SLAM_Viewer", spawn=True)
    
    # Set the world coordinate system (Right, Down, Forward is standard for OpenCV/Camera data)
    rr.log("world", rr.ViewCoordinates.RDF, timeless=True)

    # 1. Log the Global Point Cloud Map
    if os.path.exists(map_path):
        print(f"Loading Global Map from {map_path}...")
        rr.log("world/global_map", rr.Asset3D(path=map_path))
    else:
        print("❌ Could not find global map PLY.")
        return

    # 2. Log the Camera Trajectory
    if os.path.exists(trajectory_path):
        trajectory = np.load(trajectory_path)
        translations = []
        
        print(f"Logging {len(trajectory)} trajectory nodes...")
        for i, transform in enumerate(trajectory):
            # Extract Translation (x, y, z) and Rotation from the 4x4 matrix
            translation = transform[:3, 3]
            rotation_mat = transform[:3, :3]
            translations.append(translation)
            
            # Log the physical camera pose in space
            rr.log(
                f"world/trajectory/node_{i:04d}",
                rr.Transform3D(translation=translation, mat3x3=rotation_mat)
            )
        
        # 3. Draw a Red Line representing the robot's physical path
        rr.log(
            "world/trajectory_path",
            rr.LineStrips3D([translations], colors=[[255, 0, 0]], radii=0.01)
        )
    else:
        print("❌ Could not find trajectory NPY.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=str, default="output/FINAL_global_map.ply")
    parser.add_argument("--traj", type=str, default="output/FINAL_trajectory.npy")
    args = parser.parse_args()
    
    visualize_slam_reconstruction(args.map, args.traj)