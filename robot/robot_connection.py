import paho.mqtt.client as mqtt
import json
import time

# --- CONFIGURATION ---
RUNPOD_IP = "127.0.0.1"  # e.g., "74.12.34.56"
RUNPOD_PORT = 1883           # The external port you mapped to 1883
USER = "robot_user"
PASS = "robot_password_123"
# ---------------------

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected successfully to RunPod!")
    else:
        print(f"Connection failed with code {rc}")

client = mqtt.Client()
client.username_pw_set(USER, PASS)
client.on_connect = on_connect

try:
    client.connect(RUNPOD_IP, RUNPOD_PORT, 60)
    client.loop_start()

    while True:
        # Create your data payload
        payload = {
            "robot_id": "bot_01",
            "battery": 92,
            "position": {"x": 10.5, "y": 20.2},
            "timestamp": time.time()
        }
        
        # Publish to a specific topic
        client.publish("robot/telemetry", json.dumps(payload))
        print(f"Sent data: {payload}")
        
        time.sleep(2) # Send data every 2 seconds

except KeyboardInterrupt:
    print("Stopping robot...")
    client.disconnect()