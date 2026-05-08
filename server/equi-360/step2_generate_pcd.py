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

def process_full_node(kf_dir, model, device, width, height):
    """Processes a 12-view node simultaneously."""
    image_paths = sorted(glob.glob(os.path.join(kf_dir, "*.jpg")))
    if len(image_paths) == 0:
        return None, None, None
        
    print(f"  [Mode: FULL] Views: {len(image_paths)}, Res: {width}x{height}, Device: {device.type.upper()}")

    # Load and resize all 12 images
    imgs = []
    for p in image_paths:
        img = cv2.imread(p)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
        imgs.append(img)
    
    # Format for Pi3X: (B, N, C, H, W)
    imgs_tensor = torch.from_numpy(np.stack(imgs)).permute(0, 3, 1, 2).float() / 255.0
    imgs_batch = imgs_tensor.unsqueeze(0).to(device)

    # --- Inference ---
    # We use inference_mode() which is even stricter on memory savings than no_grad()
    with torch.inference_mode():
        if device.type == 'cuda':
            # PyTorch 2.5+ GPU Optimizations
            # Note: sdpa_kernel replaces sdp_kernel to clear your deprecation warning
            with torch.nn.attention.sdpa_kernel(enable_flash=True, enable_math=False, enable_mem_efficient=True):
                with torch.amp.autocast('cuda'):
                    res = model(imgs_batch)
        else:
            # CPU Execution (Standard Float32, no autocast)
            res = model(imgs_batch)

    # --- Filtering Logic ---
    conf_mask = torch.sigmoid(res['conf'][0, ..., 0]) > 0.1
    edge_mask = ~depth_edge(res['local_points'][0, ..., 2], rtol=0.03)
    final_mask = torch.logical_and(conf_mask, edge_mask)

    # --- Data Extraction ---
    pts = res['points'][0][final_mask].cpu().numpy()
    cols = (imgs_batch[0].permute(0, 2, 3, 1)[final_mask].cpu().numpy() * 255).astype(np.uint8)
    poses = res['camera_poses'][0].cpu().numpy()
        
    return pts, cols, poses

def run_pipeline(input_dir, output_dir, width, height, compute_device):
    device = torch.device(compute_device)
    print(f"Loading Pi3X model to {device}...")
    model = Pi3X.from_pretrained("yyfz233/Pi3X").to(device).eval()
    
    os.makedirs(output_dir, exist_ok=True)
    kf_dirs = sorted([d for d in glob.glob(os.path.join(input_dir, "kf_*")) if os.path.isdir(d)])
    
    for kf_dir in kf_dirs:
        kf_name = os.path.basename(kf_dir)
        print(f"\nProcessing {kf_name}...")
        
        if device.type == 'cuda': torch.cuda.synchronize()
        start = time.time()
        
        pts, cols, poses = process_full_node(kf_dir, model, device, width, height)
        if pts is None: continue
        
        if device.type == 'cuda': torch.cuda.synchronize()
        print(f"⏱️ Time: {time.time()-start:.2f}s | Valid Points: {len(pts)}")
        
        out_file = os.path.join(output_dir, f"{kf_name}_reconstruction.npz")
        np.savez_compressed(out_file, points=pts, colors=cols, poses=poses)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_dir", type=str, default="output/keyframes")
    parser.add_argument("--output_dir", type=str, default="output/pcd_nodes")
    
    # Resolution arguments
    parser.add_argument("--width", type=int, default=336, help="Must be a multiple of 14")
    parser.add_argument("--height", type=int, default=224, help="Must be a multiple of 14")
    
    # Device selection
    parser.add_argument("--device", type=str, choices=['cuda', 'cpu'], default='cuda', 
                        help="Choose 'cuda' for speed or 'cpu' for high-resolution stability.")
    
    args = parser.parse_args()
    
    run_pipeline(args.input_dir, args.output_dir, args.width, args.height, args.device)