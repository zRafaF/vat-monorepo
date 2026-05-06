import zenoh
import json
import os

# Connect to the same router the robot is using
ZENOH_ROUTER = os.environ.get('ZENOH_ROUTER', 'tcp/100.125.156.19:7447')
ROBOT_NAME = "jetson_robot"

conf = zenoh.Config()
conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')

try:
    # Try to open a session
    session = zenoh.open(conf)
    print(f"✅ Successfully connected to Zenoh router at {ZENOH_ROUTER}")
except Exception as e:
    print(f"❌ Failed to connect to Zenoh router at {ZENOH_ROUTER}")
    print(f"Error: {e}")
    exit(1)  # Exit the script if connection fails

print(f"Querying topics for {ROBOT_NAME}...")

try:
    # Query the bridge's queryable
    replies = session.get(f"{ROBOT_NAME}/system/get_topics")
except Exception as e:
    print(f"❌ Failed to query topics from {ROBOT_NAME}")
    print(f"Error: {e}")
    session.close()
    exit(1)

found = False
for reply in replies:
    found = True
    try:
        topics = json.loads(reply.ok.payload.decode('utf-8'))
        print("\n--- Available ROS 2 Topics on Robot ---")
        for topic, types in topics.items():
            print(f" Topic: {topic:30} | Type: {types[0]}")
    except Exception as e:
        print(f"Error decoding reply: {e}")

if not found:
    print("⚠️ No response from robot. Is the bridge running and connected to the router?")

session.close()