# --mode flag (full, quads, pairs)
# python step2_generate_pcd.py --mode full --device cpu

import os
import sys
import glob
import time
import argparse
import torch
import numpy as np
import cv2

# Ensure Pi3 modules can be imported
sys.path.append(os.path.join(os.path.dirname(__file__), 'Pi3'))
from pi3.models.pi3x import Pi3X
from pi3.utils.geometry import depth_edge

def process_elastic_node(kf_dir, model, device, mode):
    """Processes a 12-view node using dynamic batching to manage VRAM."""
    image_paths = sorted(glob.glob(os.path.join(kf_dir, "*.jpg")))
    if len(image_paths) == 0:
        return None, None, None, None
        
    # --- Elastic Configuration ---
    if mode == 'full':
        window_size = 12
        step_size = 12
        target_w, target_h = 546, 392 # Lowest resolution to try to survive 8GB
    elif mode == 'quads':
        window_size = 4
        step_size = 3 # 1 frame overlap between chunks
        target_w, target_h = 448, 336 # Medium resolution
    elif mode == 'pairs':
        window_size = 2
        step_size = 1 # 1 frame overlap between chunks
        target_w, target_h = 546, 392 # High resolution
    else:
        raise ValueError("Invalid mode selected.")

    print(f"  [Mode: {mode.upper()}] Window: {window_size}, Res: {target_w}x{target_h}")

    # Load and resize all images
    imgs = []
    for p in image_paths:
        img = cv2.imread(p)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (target_w, target_h), interpolation=cv2.INTER_AREA)
        imgs.append(img)
    
    node_points, node_colors, node_poses = [], [], []
    
    # Process using a sliding window
    for start_idx in range(0, len(imgs) - window_size + 1, step_size):
        chunk_imgs = imgs[start_idx : start_idx + window_size]
        
        # Format for Pi3X: (B, N, C, H, W)
        imgs_tensor = torch.from_numpy(np.stack(chunk_imgs)).permute(0, 3, 1, 2).float() / 255.0
        imgs_batch = imgs_tensor.unsqueeze(0).to(device)

        with torch.no_grad():
            if device.type == 'cuda':
                with torch.amp.autocast('cuda'):
                    res = model(imgs_batch)
            else:
                # Run on CPU in standard float32
                res = model(imgs_batch)
                
        # Filtering logic
        conf_mask = torch.sigmoid(res['conf'][0, ..., 0]) > 0.1
        edge_mask = ~depth_edge(res['local_points'][0, ..., 2], rtol=0.03)
        final_mask = torch.logical_and(conf_mask, edge_mask)

        # Extract data
        p = res['points'][0][final_mask].cpu().numpy()
        c = (imgs_batch[0].permute(0, 2, 3, 1)[final_mask].cpu().numpy() * 255).astype(np.uint8)
        
        node_points.append(p)
        node_colors.append(c)
        node_poses.append(res['camera_poses'][0].cpu().numpy())
        
    return node_points, node_colors, node_poses, None

def run_pipeline(input_dir, output_dir, mode):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Loading Pi3X model to {device}...")
    model = Pi3X.from_pretrained("yyfz233/Pi3X").to(device).eval()
    
    os.makedirs(output_dir, exist_ok=True)
    kf_dirs = sorted([d for d in glob.glob(os.path.join(input_dir, "kf_*")) if os.path.isdir(d)])
    
    for kf_dir in kf_dirs:
        kf_name = os.path.basename(kf_dir)
        print(f"\nProcessing {kf_name}...")
        
        torch.cuda.synchronize()
        start = time.time()
        
        pts, cols, poses, _ = process_elastic_node(kf_dir, model, device, mode)
        if pts is None: continue
        
        torch.cuda.synchronize()
        print(f"⏱️ Time: {time.time()-start:.2f}s | Points: {len(pts)}")
        
        out_file = os.path.join(output_dir, f"{kf_name}_reconstruction.npz")
        
        save_dict = {}
        for i in range(len(pts)):
            save_dict[f'points_{i}'] = pts[i]
            save_dict[f'colors_{i}'] = cols[i]
            save_dict[f'poses_{i}'] = poses[i]
        np.savez_compressed(out_file, **save_dict)
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_dir", type=str, default="output/keyframes")
    parser.add_argument("--output_dir", type=str, default="output/pcd_nodes")
    parser.add_argument("--mode", type=str, choices=['full', 'quads', 'pairs'], default='quads',
                        help="Batching mode to manage VRAM. Quads is recommended for 8GB GPUs.")
    args = parser.parse_args()
    
    run_pipeline(args.input_dir, args.output_dir, args.mode)