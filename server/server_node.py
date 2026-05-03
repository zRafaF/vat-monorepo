import zenoh
import time

def telemetry_listener(sample):
    # .to_string() works great for text-based telemetry
    print(f" [TELEMETRY] Received: {sample.payload.to_string()}")

conf = zenoh.Config()
# Tell the server to listen on all interfaces at port 7447
conf.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447"]')

if __name__ == "__main__":
    with zenoh.open(conf) as session:
        # Only subscribe to the telemetry path
        sub = session.declare_subscriber('robot/telemetry', telemetry_listener)
        print("Telemetry Server is running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)