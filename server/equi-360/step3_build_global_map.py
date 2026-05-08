import os
import glob
import numpy as np
import open3d as o3d
import time

def assemble_spatial_node(npz_path, voxel_size):
    """Phase 1: Intra-Node Assembly. Snaps the 'quad' rings into a perfect 360 bubble."""
    data = np.load(npz_path)
    assembled_pcd = o3d.geometry.PointCloud()
    
    # Count how many chunks (rings) are in the file
    num_chunks = sum(1 for k in data.files if k.startswith('points_'))
    
    for i in range(num_chunks):
        chunk = o3d.geometry.PointCloud()
        chunk.points = o3d.utility.Vector3dVector(data[f'points_{i}'])
        chunk.colors = o3d.utility.Vector3dVector(data[f'colors_{i}'] / 255.0)
        chunk = chunk.voxel_down_sample(voxel_size)
        chunk.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))
        
        if i == 0:
            assembled_pcd = chunk # The first ring acts as the local anchor
        else:
            # ICP Snap the other rings to the local anchor ring
            reg = o3d.pipelines.registration.registration_icp(
                chunk, assembled_pcd, max_correspondence_distance=voxel_size * 5,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
            )
            chunk.transform(reg.transformation)
            assembled_pcd += chunk
            
    return assembled_pcd.voxel_down_sample(voxel_size)

def build_global_map(input_dir, output_dir, voxel_size=0.03):
    npz_files = sorted(glob.glob(os.path.join(input_dir, "*.npz")))
    if not npz_files:
        print(f"❌ No .npz files found in {input_dir}")
        return

    print(f"🌐 Equi-360: Two-Stage Assembly & Stitching ({len(npz_files)} nodes)...")
    
    # Initialize Map and Trajectory Tracker
    print(f"⚓ Initializing Global Anchor: {os.path.basename(npz_files[0])}")
    global_pcd = assemble_spatial_node(npz_files[0], voxel_size)
    trajectory = [np.eye(4)] # Frame 0 is at the Origin (Identity Matrix)
    
    for i in range(1, len(npz_files)):
        start_time = time.time()
        node_name = os.path.basename(npz_files[i])
        
        # Phase 1: Assemble the shattered 360 node
        source_pcd = assemble_spatial_node(npz_files[i], voxel_size)
        
        # Phase 2: Inter-Node SLAM (Snap the assembled bubble to the global map)
        reg = o3d.pipelines.registration.registration_icp(
            source_pcd, global_pcd, max_correspondence_distance=voxel_size * 10,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        
        # Apply transformation to point cloud
        source_pcd.transform(reg.transformation)
        
        # Calculate global pose by multiplying local transform with previous global pose
        current_global_pose = reg.transformation @ trajectory[-1]
        trajectory.append(current_global_pose)
        
        # Merge and downsample to maintain constant memory
        global_pcd += source_pcd
        global_pcd = global_pcd.voxel_down_sample(voxel_size)
        
        print(f"✅ Stitched {node_name} | Points: {len(global_pcd.points)} | Time: {time.time()-start_time:.2f}s")

    os.makedirs(output_dir, exist_ok=True)
    
    # Save the Map
    ply_path = os.path.join(output_dir, "FINAL_global_map.ply")
    o3d.io.write_point_cloud(ply_path, global_pcd)
    
    # Save the Trajectory Data for Step 4
    traj_path = os.path.join(output_dir, "FINAL_trajectory.npy")
    np.save(traj_path, np.array(trajectory))
    print(f"\n💾 Saved Map and Trajectory to {output_dir}")

    # Interactive Visualization
    print("🎥 Opening 3D Visualizer... (Press Q to close)")
    o3d.visualization.draw_geometries([global_pcd], window_name="Equi-360 Final Reconstruction")

if __name__ == "__main__":
    # Ensure directories match your setup
    build_global_map(
        input_dir="output/pcd_nodes", 
        output_dir="output", 
        voxel_size=0.03
    )