import argparse
import numpy as np
import rerun as rr
import open3d as o3d
import os

def visualize_slam_reconstruction(map_path, trajectory_path):
    print("🚀 Initializing Rerun Visualizer...")
    rr.init("Equi-360_SLAM_Viewer", spawn=True)
    
    # Set the world coordinate system (Right, Down, Forward is standard for camera data)
    rr.log("world", rr.ViewCoordinates.RDF, timeless=True)

    # 1. Explicitly load and log the Point Cloud using Open3D
    if os.path.exists(map_path):
        print(f"Loading Global Map from {map_path}...")
        pcd = o3d.io.read_point_cloud(map_path)
        
        points = np.asarray(pcd.points)
        # Open3D loads colors as floats [0, 1]. Convert to uint8 [0, 255] for Rerun
        colors = (np.asarray(pcd.colors) * 255).astype(np.uint8) 
        
        print(f"Logging {len(points)} points to Rerun...")
        rr.log(
            "world/global_map", 
            rr.Points3D(positions=points, colors=colors, radii=0.02) # Adjust radii if points look too sparse/thick
        )
    else:
        print(f"❌ Could not find global map PLY at {map_path}")
        return

    # 2. Log the Camera Trajectory
    if os.path.exists(trajectory_path):
        trajectory = np.load(trajectory_path)
        translations = []
        
        print(f"Logging {len(trajectory)} trajectory nodes...")
        for i, transform in enumerate(trajectory):
            translation = transform[:3, 3]
            rotation_mat = transform[:3, :3]
            translations.append(translation)
            
            # Log the physical camera pose
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
        print(f"❌ Could not find trajectory NPY at {trajectory_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=str, default="output/FINAL_global_map.ply")
    parser.add_argument("--traj", type=str, default="output/FINAL_trajectory.npy")
    args = parser.parse_args()
    
    visualize_slam_reconstruction(args.map, args.traj)