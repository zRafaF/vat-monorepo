import os
import glob
import numpy as np
import open3d as o3d
import time

def load_npz_as_pcd(npz_path, voxel_size=0.03):
    data = np.load(npz_path)
    pcd = o3d.geometry.PointCloud()
    
    # In 'full' mode, there is only one chunk, but it might be saved as points_0 
    # depending on how you left the step 2 script. Let's handle both dynamically:
    if 'points' in data:
        pts = data['points']
        cols = data['colors']
    else:
        pts = data['points_0']
        cols = data['colors_0']

    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(cols / 255.0)
    pcd = pcd.voxel_down_sample(voxel_size)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    return pcd

def build_global_map(input_dir, output_dir, voxel_size=0.03):
    npz_files = sorted(glob.glob(os.path.join(input_dir, "*.npz")))
    
    print(f"🌐 Equi-360: Full-Mode Stitching ({len(npz_files)} nodes)...")
    
    global_pcd = load_npz_as_pcd(npz_files[0], voxel_size)
    trajectory = [np.eye(4)] 
    
    for i in range(1, len(npz_files)):
        start_time = time.time()
        
        source_pcd = load_npz_as_pcd(npz_files[i], voxel_size)
        
        reg = o3d.pipelines.registration.registration_icp(
            source_pcd, global_pcd, max_correspondence_distance=voxel_size * 10,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        
        source_pcd.transform(reg.transformation)
        current_global_pose = reg.transformation @ trajectory[-1]
        trajectory.append(current_global_pose)
        
        global_pcd += source_pcd
        global_pcd = global_pcd.voxel_down_sample(voxel_size)
        
        print(f"✅ Stitched node {i} | Points: {len(global_pcd.points)} | Time: {time.time()-start_time:.2f}s")

    os.makedirs(output_dir, exist_ok=True)
    o3d.io.write_point_cloud(os.path.join(output_dir, "FINAL_global_map.ply"), global_pcd)
    np.save(os.path.join(output_dir, "FINAL_trajectory.npy"), np.array(trajectory))
    print(f"\n💾 Saved Map and Trajectory to {output_dir}")

if __name__ == "__main__":
    build_global_map("output/pcd_nodes", "output", voxel_size=0.03)