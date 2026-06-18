import zenoh
import time

def listener_callback(sample):
    # sample.payload contains the raw binary CDR bytes sent from the robot
    payload_size = len(sample.payload)
    print(f"[Zenoh] Received {payload_size} bytes on key: {sample.key_expr}")

if __name__ == '__main__':
    # 1. Connect to the Zenoh Router
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", '["tcp/100.125.156.19:7447"]')
    
    print("Connecting to Zenoh Router...")
    session = zenoh.open(conf)
    
    # 2. Subscribe to the exact key the robot is publishing
    zenoh_key = "my_robot/rt/utlidar/cloud"
    print(f"Subscribing to {zenoh_key}...")
    
    sub = session.declare_subscriber(zenoh_key, listener_callback)
    
    try:
        print("Listening for Lidar data... (Press Ctrl+C to quit)")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down client...")
    finally:
        session.close()