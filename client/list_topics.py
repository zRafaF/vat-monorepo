# run with `--topic /utlidar/cloud` 

import zenoh
import json
import os
import time
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', type=str, default=None, help="ROS topic to subscribe to (e.g., /chatter)")
    args = parser.parse_args()

    ZENOH_ROUTER = os.environ.get('ZENOH_ROUTER', 'tcp/100.125.156.19:7447')
    ROBOT_NAME = "jetson_robot"

    zenoh.init_log_from_env_or("error")
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
    session = zenoh.open(conf)

    print(f"Connected to Zenoh. Monitoring {ROBOT_NAME}...")

    # 1. Setup Liveliness Monitor
    liveliness_key = f"{ROBOT_NAME}/system/liveliness"
    
    def liveliness_callback(sample):
        if sample.kind == zenoh.SampleKind.PUT:
            print(f"\n[HEALTH] 🟢 Robot is ONLINE! ('{sample.key_expr}')")
        elif sample.kind == zenoh.SampleKind.DELETE:
            print(f"\n[HEALTH] 🔴 Robot went OFFLINE! Connection lost. ('{sample.key_expr}')")

    liveliness_sub = session.liveliness().declare_subscriber(liveliness_key, liveliness_callback)
    
    time.sleep(1) # Give it a second to detect the robot

    # 2. Query available topics
    print("\nQuerying available topics...")
    replies = session.get(f"{ROBOT_NAME}/system/get_topics")
    
    available_topics = {}
    for reply in replies:
        try:
            # FIX: Convert ZBytes to standard bytes before decoding
            payload_bytes = bytes(reply.ok.payload) 
            available_topics = json.loads(payload_bytes.decode('utf-8'))
            
            print("--- Available ROS 2 Topics ---")
            for topic, msg_type in available_topics.items():
                print(f" - {topic:30} ({msg_type})")
        except Exception as e:
            print(f"Error parsing topics: {e}")
    # 3. Dynamic Subscription
    target_topic = args.topic
    if target_topic:
        zenoh_data_key = f"{ROBOT_NAME}/rt{target_topic}"
        
        def data_callback(sample):
            print(f"[DATA] Received {len(sample.payload)} bytes on {sample.key_expr}")

        print(f"\nSubscribing to Zenoh Key: {zenoh_data_key}")
        print("This will automatically trigger the robot to start the ROS subscription.")
        
        # Declaring this subscriber triggers the MatchingListener on the Jetson!
        data_sub = session.declare_subscriber(zenoh_data_key, data_callback)

    try:
        print("\nListening... Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down client...")
        # When session.close() happens, the Jetson's MatchingListener will see the disconnect
        # and automatically destroy the ROS subscription.
    finally:
        session.close()

if __name__ == '__main__':
    main()