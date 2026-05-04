import zenoh
import time

conf = zenoh.Config()
conf.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447"]')

def data_callback(sample):
    print(f"\n[DATA ARRIVED] Topic: {sample.key_expr} | Bytes: {len(sample.payload.to_bytes())}")

def start_server():
    print("PC Server started. Listening for Robot...")
    with zenoh.open(conf) as session:
        # Subscribe to all topics to see what the bridge sends
        session.declare_subscriber("**", data_callback)
        
        while True:
            routers = session.info.routers_zid()
            if routers:
                print(f"Connected to Bridge: {routers}. Waiting for topics...", end="\r")
                
                # Check what the bridge has discovered in its own memory
                replies = session.get("@/*/ros2/route/pub/**")
                for reply in replies:
                    print(f"\n[BRIDGE ROUTE DETECTED] {reply.sample.key_expr}")
            else:
                print("Waiting for bridge connection...", end="\r")
            time.sleep(2)

if __name__ == "__main__":
    start_server()