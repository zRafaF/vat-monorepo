import os
import glob
import numpy as np
import open3d as o3d
import time

def load_point_cloud(npz_path, voxel_size=0.05):
    """Loads point cloud from .npz and downsamples it for faster processing."""
    data = np.load(npz_path)
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data['points'])
    # Open3D expects colors in [0, 1] range
    pcd.colors = o3d.utility.Vector3dVector(data['colors'] / 255.0)
    
    # Downsample to speed up ICP and normalize density
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    # Estimate normals (required for robust ICP)
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
    )
    return pcd_down

def register_point_clouds(source, target, voxel_size):
    """Aligns the source point cloud to the target using ICP."""
    # We assume the movement between consecutive 360 frames is relatively small
    # so we can use an Identity matrix as our initial guess.
    trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]])

    # Point-to-Plane ICP is much faster and more accurate than Point-to-Point for indoor spaces
    distance_threshold = voxel_size * 10
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
    )
    return reg_p2p.transformation

def build_global_map(input_dir, output_file, voxel_size=0.05):
    npz_files = sorted(glob.glob(os.path.join(input_dir, "*.npz")))
    if not npz_files:
        print(f"No .npz files found in {input_dir}")
        return

    print(f"Found {len(npz_files)} spatial nodes. Starting global stitching...")
    
    # 1. Initialize the global map with the first node (The Anchor)
    print(f"Loading Anchor Node: {os.path.basename(npz_files[0])}")
    global_map = load_point_cloud(npz_files[0], voxel_size)
    
    # The transformation of the current node relative to the global origin
    current_global_transform = np.identity(4)

    # 2. Iterate through the sequence and stitch
    for i in range(1, len(npz_files)):
        start_time = time.time()
        file_name = os.path.basename(npz_files[i])
        print(f"\nStitching Node {i}/{len(npz_files)-1}: {file_name}")
        
        # Load the new node
        source_pcd = load_point_cloud(npz_files[i], voxel_size)
        
        # Register the new node to the CURRENT state of the global map
        # (We align against the whole map to prevent pairwise drift)
        transformation = register_point_clouds(source_pcd, global_map, voxel_size)
        
        # Apply the transformation to move the source into the global coordinate system
        source_pcd.transform(transformation)
        
        # Merge the aligned source into the global map
        global_map += source_pcd
        
        # Re-downsample the global map to prevent RAM explosion over long videos
        global_map = global_map.voxel_down_sample(voxel_size)
        
        print(f"✅ Stitched in {time.time() - start_time:.2f} seconds. Map size: {len(global_map.points)} points.")

    # 3. Save the final global point cloud
    print("\n" + "="*40)
    print("💾 Saving Global Map...")
    o3d.io.write_point_cloud(output_file, global_map)
    print(f"🎉 Success! Saved to {output_file}")
    print("="*40)

    # 4. Open interactive visualizer
    print("Opening 3D Visualizer... (Press 'Q' or Esc to close)")
    o3d.visualization.draw_geometries([global_map], window_name="Equi-360 Global Map")

if __name__ == '__main__':
    # Define directories
    input_directory = "output/pcd_nodes"
    output_ply = "output/FINAL_global_map.ply"
    
    # Resolution of the map (0.05 = 5cm voxels). 
    # Decrease this number (e.g. 0.02) for a sharper map, increase for faster processing.
    resolution = 0.05 
    
    os.makedirs("output", exist_ok=True)
    build_global_map(input_directory, output_ply, resolution)