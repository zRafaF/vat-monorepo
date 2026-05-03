import zenoh
import time
import cv2
import numpy as np

latest_frame = None
# Variables for FPS calculation
prev_time = 0
current_fps = 0

def telemetry_listener(sample):
    print(f" [TELEMETRY] Received: {sample.payload.to_string()}")

def video_listener(sample):
    global latest_frame
    payload_bytes = sample.payload.to_bytes()
    np_arr = np.frombuffer(payload_bytes, np.uint8)
    latest_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

conf = zenoh.Config()
conf.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447"]')

if __name__ == "__main__":
    with zenoh.open(conf) as session:
        sub_telemetry = session.declare_subscriber('robot/telemetry', telemetry_listener)
        sub_video = session.declare_subscriber('robot/video', video_listener)
        
        print("Server running. FPS overlay active.")
        
        while True:
            if latest_frame is not None:
                # Calculate FPS
                current_time = time.time()
                # Avoid division by zero on the first frame
                if (current_time - prev_time) > 0:
                    current_fps = 1 / (current_time - prev_time)
                prev_time = current_time

                # Draw the FPS on the frame
                fps_text = f"FPS: {int(current_fps)}"
                cv2.putText(latest_frame, fps_text, (20, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                cv2.imshow("Zenoh Video Stream", latest_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    cv2.destroyAllWindows()