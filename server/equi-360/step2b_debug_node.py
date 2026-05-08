import argparse
import numpy as np
import rerun as rr
import os

def debug_single_node(npz_path):
    print(f"🔍 Loading Single Spatial Node: {npz_path}")
    data = np.load(npz_path)
    
    # Handle both 'full' mode (single array) and 'quads' mode (chunked arrays)
    if 'points' in data:
        points = data['points']
        colors = data['colors']
        poses = data['poses']
    else:
        print("Detected chunked geometry. Assembling...")
        # Concatenate all chunks dynamically
        points = np.concatenate([data[k] for k in sorted(data.files) if k.startswith('points_')])
        colors = np.concatenate([data[k] for k in sorted(data.files) if k.startswith('colors_')])
        poses = np.concatenate([data[k] for k in sorted(data.files) if k.startswith('poses_')])
    
    print("🚀 Initializing Rerun Visualizer...")
    rr.init("Pi3X_Single_Node_Debug", spawn=True)
    rr.log("world", rr.ViewCoordinates.RDF, timeless=True)

    # Log the point cloud
    rr.log(
        "world/point_cloud", 
        rr.Points3D(positions=points, colors=colors, radii=0.01)
    )

    # Log where Pi3X *thinks* the cameras were placed
    for i, pose in enumerate(poses):
        translation = pose[:3, 3]
        rotation_mat = pose[:3, :3]
        
        rr.log(
            f"world/cameras/cam_{i}",
            rr.Transform3D(translation=translation, mat3x3=rotation_mat)
        )

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=str, required=True, help="Path to e.g., output/pcd_nodes/kf_0000_reconstruction.npz")
    args = parser.parse_args()
    
    debug_single_node(args.input)