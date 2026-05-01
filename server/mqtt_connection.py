import asyncio
import logging
import paho.mqtt.client as mqtt
import json
from amqtt.broker import Broker

# --- 1. Configuration ---
# Authentication removed for now
config = {
    'listeners': {
        'default': {'type': 'tcp', 'bind': '0.0.0.0:1883'}
    }
}

formatter = "[%(asctime)s] :: %(levelname)s :: %(name)s :: %(message)s"
logging.basicConfig(level=logging.INFO, format=formatter)

# --- 2. Paho Client Callbacks ---
def on_connect(client, userdata, flags, rc, properties=None):
    # Note: Paho v2.0 callbacks include an extra 'properties' argument
    if rc == 0:
        print("Internal Listener: Connected to local broker.")
        client.subscribe("robot/telemetry")
    else:
        print(f"Internal Listener: Connection failed with code {rc}")

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        print(f"\n[DATA RECEIVED] Topic: {msg.topic}")
        print(f"Payload: {data}")
    except Exception as e:
        print(f"Error parsing message: {e}")

# --- 3. Main Server Logic ---
async def run_server() -> None:
    # A. Start the Broker
    broker = Broker(config)
    await broker.start()
    print("MQTT Broker is live (Anonymous access)...")

    # B. Start the Paho Listener
    # FIX: Added CallbackAPIVersion.VERSION1 for Paho 2.0 compatibility
    paho_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, "InternalListener")
    
    paho_client.on_connect = on_connect
    paho_client.on_message = on_message
    
    # Connect to localhost
    try:
        paho_client.connect("127.0.0.1", 1883)
        paho_client.loop_start() 
    except Exception as e:
        print(f"Internal Listener failed to connect: {e}")

    try:
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        paho_client.loop_stop()
        await broker.shutdown()

def __main__():
    try:
        asyncio.run(run_server())
    except KeyboardInterrupt:
        print("\nServer exiting...")

if __name__ == "__main__":
    __main__()