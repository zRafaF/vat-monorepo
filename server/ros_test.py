import zenoh
import time

def listener(sample):
    # This will print the raw byte size of the incoming data
    # Once you see 'lowstate' here, you are successful!
    print(f">> Data on '{sample.key_expr}' | Size: {len(sample.payload)} bytes")

if __name__ == "__main__":
    # Your Dell Server Tailscale IP
    router_endpoint = "tcp/100.125.156.19:7447" 
    
    conf = zenoh.Config.from_json5(
        f'{{"connect": {{"endpoints": ["{router_endpoint}"]}}}}'
    )
    
    print(f"Connecting to Zenoh router at {router_endpoint}...")
    session = zenoh.open(conf)
    
    # We use '**' to listen to everything the bridge finds.
    # Based on your curl, it will catch 'imu/data', 'tf', etc.
    print("Subscribing to ALL topics ('**')...")
    sub = session.declare_subscriber("**", listener)
    
    print("Listening... (Press Ctrl+C to exit)")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nClosing session...")
        session.close()