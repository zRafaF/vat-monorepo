import zenoh
import time
import random

# For the PoC, we'll simulate a 1MB Point Cloud
def get_fake_pointcloud():
    return bytearray(random.getrandbits(8) for _ in range(1024 * 1024))

conf = zenoh.Config()
# Replace with your server's actual IP
conf.insert_json5("connect/endpoints", '["tcp/192.168.1.10:7447"]')

if __name__ == "__main__":
    with zenoh.open(conf) as session:
        # Define our two keys
        key_telemetry = 'robot/telemetry'
        key_pc = 'robot/pc'
        
        pub_telemetry = session.declare_publisher(key_telemetry)
        pub_pc = session.declare_publisher(key_pc)
        
        print(f"Robot online. Publishing to {key_telemetry} and {key_pc}...")
        
        count = 0
        while True:
            # 1. Telemetry (String/JSON)
            temp = random.randint(20, 30)
            telemetry_buf = f"temp={temp}, battery=85%"
            pub_telemetry.put(telemetry_buf)
            
            # 2. Point Cloud (Raw Bytes) - every 2 seconds
            if count % 20 == 0:
                print("Sending Point Cloud data...")
                pc_buf = get_fake_pointcloud()
                pub_pc.put(pc_buf)
            
            count += 1
            time.sleep(0.1) # 10Hz loop