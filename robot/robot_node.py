import zenoh
import time
import cv2

conf = zenoh.Config()
conf.insert_json5("connect/endpoints", '["tcp/192.168.1.10:7447"]')

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    # Set webcam to 30 FPS (if hardware supports it)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    with zenoh.open(conf) as session:
        pub_video = session.declare_publisher('robot/video')
        pub_telemetry = session.declare_publisher('robot/telemetry')
        
        print("Robot streaming...")
        
        count = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # 1. Video (Full Speed)
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            pub_video.put(buffer.tobytes())

            # 2. Telemetry (Throttle to ~10Hz by sending every 3rd frame)
            if count % 3 == 0:
                pub_telemetry.put("temp=24, battery=80%")
            
            count += 1
            # We remove the sleep(0.1) entirely to let the camera clock drive the loop[cite: 3]