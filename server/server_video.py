import zenoh
import time
import cv2
import numpy as np

latest_frame = None
prev_time = 0
fps = 0

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
        # We subscribe to telemetry and video, but ignore the heavy point clouds for now
        session.declare_subscriber('robot/telemetry', telemetry_listener)
        session.declare_subscriber('robot/video', video_listener)
        
        print("Server running. Watch the video window for real-time FPS.")
        
        while True:
            if latest_frame is not None:
                # FPS Calculation logic
                curr_time = time.time()
                diff = curr_time - prev_time
                if diff > 0:
                    fps = 1 / diff
                prev_time = curr_time

                # Overlay the FPS counter
                cv2.putText(latest_frame, f"FPS: {int(fps)}", (20, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                cv2.imshow("Zenoh Video Stream", latest_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    cv2.destroyAllWindows()