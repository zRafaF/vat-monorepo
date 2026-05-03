import zenoh
import time
import random
import cv2

# Simulated 1MB Point Cloud payload
def get_fake_pointcloud():
    return bytearray(random.getrandbits(8) for _ in range(1024 * 1024))

conf = zenoh.Config()
# REPLACE with your server's actual IP
conf.insert_json5("connect/endpoints", '["tcp/192.168.1.10:7447"]')

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    # Attempt to set hardware to 30 FPS
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    with zenoh.open(conf) as session:
        pub_video = session.declare_publisher('robot/video')
        pub_telemetry = session.declare_publisher('robot/telemetry')
        pub_pc = session.declare_publisher('robot/pc')
        
        print("Robot online. Streaming at full speed...")
        
        frame_count = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # 1. Video (Full Speed)
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            pub_video.put(buffer.tobytes())

            # 2. Telemetry (Throttle to ~10Hz by sending every 3rd frame)
            if frame_count % 3 == 0:
                pub_telemetry.put(f"temp={random.randint(20, 30)}, battery=85%")
            
            # 3. Point Cloud (Throttle to every ~2 seconds)
            if frame_count % 60 == 0:
                pub_pc.put(get_fake_pointcloud())
            
            frame_count += 1

    cap.release()