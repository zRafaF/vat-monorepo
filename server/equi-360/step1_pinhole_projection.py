import cv2
import numpy as np
import os

class EquirectangularToPinhole:
    def __init__(self, fov, output_size):
        self.fov = fov
        self.w, self.h = output_size
        self.f = 0.5 * self.w / np.tan(0.5 * np.radians(self.fov))
        
        u, v = np.meshgrid(np.arange(self.w), np.arange(self.h))
        self.x = u - self.w / 2
        self.y = v - self.h / 2
        self.z = self.f

    def get_view(self, equirect_img, yaw_deg, pitch_deg):
        yaw = np.radians(yaw_deg)
        pitch = np.radians(pitch_deg)
        
        R_yaw = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
        R_pitch = np.array([[1, 0, 0], [0, np.cos(pitch), -np.sin(pitch)], [0, np.sin(pitch), np.cos(pitch)]])
        R = R_yaw @ R_pitch

        xyz = np.stack([self.x, self.y, np.full_like(self.x, self.z)], axis=-1)
        xyz_rot = xyz @ R.T
        
        lon = np.arctan2(xyz_rot[..., 0], xyz_rot[..., 2])
        lat = np.arctan2(xyz_rot[..., 1], np.sqrt(xyz_rot[..., 0]**2 + xyz_rot[..., 2]**2))
        
        eq_h, eq_w = equirect_img.shape[:2]
        ui = (lon / (2 * np.pi) + 0.5) * eq_w
        vi = (lat / np.pi + 0.5) * eq_h
        
        return cv2.remap(equirect_img, ui.astype(np.float32), vi.astype(np.float32), cv2.INTER_LINEAR)

def process_360_spatial_nodes(video_path, output_dir, flow_threshold=12.0):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    projector = EquirectangularToPinhole(fov=90, output_size=(640, 480))
    
    prev_gray = None
    accumulated_motion = 0.0 # Track movement over time
    keyframe_count = 0
    
    camera_rig = [
        (0, 0, "mid_front"), 
        (60, 0, "mid_front_right"), 
        (120, 0, "mid_back_right"), 
        (180, 0, "mid_back"),
        (240, 0, "mid_back_left"), 
        (300, 0, "mid_front_left"),
        (0, 60, "up_ceiling"), 
        (0, -45, "down_floor") # -45 instead of -90 to avoid seeing the robot body
    ]

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret or frame is None: 
            break
        
        # 1. Prepare a low-res gray image for flow to speed things up
        h, w = frame.shape[:2]
        horizon_strip = frame[h//3:2*h//3, :] 
        # Resize to a constant size to prevent shape mismatch errors
        small_gray = cv2.cvtColor(horizon_strip, cv2.COLOR_BGR2GRAY)
        small_gray = cv2.resize(small_gray, (400, 200)) 
        
        is_keyframe = False
        
        if prev_gray is None:
            is_keyframe = True
        else:
            # 2. Calculate flow between CONSECUTIVE frames
            flow = cv2.calcOpticalFlowFarneback(prev_gray, small_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            
            # Accumulate motion so we catch slow-moving robots
            accumulated_motion += np.mean(mag)
            
            if accumulated_motion > flow_threshold:
                is_keyframe = True
                accumulated_motion = 0 # Reset after triggering
        
        if is_keyframe:
            kf_dir = os.path.join(output_dir, f"kf_{keyframe_count:04d}")
            os.makedirs(kf_dir, exist_ok=True)
            
            for yaw, pitch, name in camera_rig:
                view = projector.get_view(frame, yaw_deg=yaw, pitch_deg=pitch)
                cv2.imwrite(os.path.join(kf_dir, f"{name}.jpg"), view)
            
            print(f"Generated 360 Spatial Node: {keyframe_count}")
            keyframe_count += 1
            
        # Update prev_gray EVERY frame
        prev_gray = small_gray 

    cap.release()
    print("Processing complete.")

# Run the pipeline
process_360_spatial_nodes("input/robot_path.mp4", "output/keyframes", flow_threshold=3.0)