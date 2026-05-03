import zenoh
import time
import numpy as np

def telemetry_listener(sample):
    # Consumes the string-based telemetry
    print(f" [ANALYTICS] Telemetry Update: {sample.payload.to_string()}")

def pc_listener(sample):
    # Convert the raw bytes from the point cloud into a NumPy array
    payload_bytes = sample.payload.to_bytes()
    # Interpret the bits as unsigned 8-bit integers
    data = np.frombuffer(payload_bytes, dtype=np.uint8)
    
    # Calculate the Standard Deviation
    std_dev = np.std(data)
    
    print(f" [ANALYTICS] Received Point Cloud ({len(payload_bytes)} bytes)")
    print(f"             Standard Deviation: {std_dev:.4f}")

conf = zenoh.Config()
# Listen on all interfaces at port 7447
conf.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447"]')

if __name__ == "__main__":
    with zenoh.open(conf) as session:
        # Subscribe only to the keys required for analytics
        sub_telemetry = session.declare_subscriber('robot/telemetry', telemetry_listener)
        sub_pc = session.declare_subscriber('robot/pc', pc_listener)
        
        print("Analytics Server is running. Press Ctrl+C to stop.")
        
        # Keep the script alive
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Shutting down Analytics Server.")