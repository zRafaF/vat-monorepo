import os
import sys
import glob
import time
import argparse
import torch
import numpy as np
import cv2

# Ensure Pi3 modules can be imported when running from the root directory
sys.path.append(os.path.join(os.path.dirname(__file__), 'Pi3'))

from Pi3.pi3.models.pi3x import Pi3X
from Pi3.pi3.utils.geometry import depth_edge

def load_spatial_node(kf_dir, device):
    """Loads and resizes the 12 views of a node to be compatible with DINOv2 patches."""
    image_paths = sorted(glob.glob(os.path.join(kf_dir, "*.jpg")))
    if len(image_paths) == 0:
        return None
        
    # Pi3X (DINOv2) requires dimensions to be multiples of 14
    TARGET_W, TARGET_H = 644, 476 
        
    imgs = []
    for p in image_paths:
        img = cv2.imread(p)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Resize to the nearest multiples of 14
        img = cv2.resize(img, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA)
        imgs.append(img)
    
    imgs_np = np.stack(imgs, axis=0)
    imgs_tensor = torch.from_numpy(imgs_np).permute(0, 3, 1, 2).float() / 255.0
    return imgs_tensor.unsqueeze(0).to(device)


def process_keyframes(input_dir, output_dir, device_name='cuda'):
    device = torch.device(device_name)
    
    print(f"Loading Pi3X model to {device_name}...")
    model = Pi3X.from_pretrained("yyfz233/Pi3X").to(device).eval()
    
    dtype = torch.bfloat16 if torch.cuda.is_available() and torch.cuda.get_device_capability()[0] >= 8 else torch.float16
    
    os.makedirs(output_dir, exist_ok=True)
    kf_dirs = sorted([d for d in glob.glob(os.path.join(input_dir, "kf_*")) if os.path.isdir(d)])
    
    total_frames = 0
    total_time = 0.0

    for kf_dir in kf_dirs:
        kf_name = os.path.basename(kf_dir)
        print(f"\nProcessing {kf_name}...")
        
        imgs = load_spatial_node(kf_dir, device)
        if imgs is None:
            continue
            
        N_views = imgs.shape[1]
        
        # --- Inference & Benchmarking ---
        torch.cuda.synchronize()
        start_time = time.time()
        
        with torch.no_grad():
            with torch.amp.autocast('cuda', dtype=dtype):
                res = model(imgs)
                
        torch.cuda.synchronize()
        end_time = time.time()
        
        inference_time = end_time - start_time
        total_time += inference_time
        total_frames += N_views
        fps = N_views / inference_time
        
        print(f"⏱️ Inference Time : {inference_time:.4f} sec | Views: {N_views} | Speed: {fps:.2f} FPS")

        # --- Data Extraction & Masking ---
        # 1. Calculate validity masks based on confidence and depth edges
        conf_logits = res['conf'][..., 0]
        masks = torch.sigmoid(conf_logits) > 0.1
        non_edge = ~depth_edge(res['local_points'][..., 2], rtol=0.03)
        final_mask = torch.logical_and(masks, non_edge)[0]

        # 2. Extract arrays
        # Shape: (N, H, W, 3) -> Flattened based on mask
        all_points = res['points'][0]
        all_colors = imgs[0].permute(0, 2, 3, 1) # Move channels to last dimension for RGB
        all_poses = res['camera_poses'][0]
        
        valid_points = all_points[final_mask].cpu().numpy()
        valid_colors = (all_colors[final_mask].cpu().numpy() * 255).astype(np.uint8)
        
        # Keep poses and raw confidence for the full 12 views for stitching/visualization
        poses_np = all_poses.cpu().numpy()
        conf_np = torch.sigmoid(conf_logits[0]).cpu().numpy()

        # --- Exporting ---
        out_file = os.path.join(output_dir, f"{kf_name}_reconstruction.npz")
        np.savez_compressed(
            out_file,
            points=valid_points,    # Filtered 3D points
            colors=valid_colors,    # Filtered RGB colors (uint8)
            poses=poses_np,         # Array of 12 (4x4) camera poses
            conf=conf_np,           # Confidence maps for error analysis
            inference_time=inference_time
        )
        print(f"💾 Saved to {out_file} ({len(valid_points)} valid points)")

    print("\n" + "="*40)
    print("🚀 FINAL BENCHMARK RESULTS")
    print("="*40)
    print(f"Total Spatial Nodes Processed : {len(kf_dirs)}")
    print(f"Total Views Processed         : {total_frames}")
    print(f"Average Speed                 : {(total_frames / total_time):.2f} FPS")
    print("="*40 + "\n")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_dir", type=str, default="output/keyframes", help="Directory containing kf_XXXX folders")
    parser.add_argument("--output_dir", type=str, default="output/pcd_nodes", help="Directory to save the .npz files")
    args = parser.parse_args()
    
    process_keyframes(args.input_dir, args.output_dir)