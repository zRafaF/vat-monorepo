import zenoh
import json
import os

def main():
    # Connect to your server's Zenoh Router
    ZENOH_ROUTER = os.environ.get('ZENOH_ROUTER', 'tcp/100.125.156.19:7447')

    zenoh.init_log_from_env_or("error")
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
    session = zenoh.open(conf)

    print(f"Connected to Router: {ZENOH_ROUTER}")
    print("Querying the Zenoh Admin Space for bridged ROS 2 topics...\n")

    # The official bridge exposes its discovered routes under the Admin Space[cite: 14]
    # @/*/ros2/route/** gets the routes from ALL connected bridges[cite: 14]
    replies = session.get("@/*/ros2/route/**")

    found_topics = False
    
    print(f"{'ROS 2 Topic':<40} | {'Zenoh Key'}")
    print("-" * 80)

    for reply in replies:
        found_topics = True
        try:
            # The payload contains a JSON configuration of the route
            payload_bytes = bytes(reply.ok.payload)
            route_info = json.loads(payload_bytes.decode('utf-8'))
            
            # The Admin Space key looks like: @/<bridge_id>/ros2/route/<type>/<topic>
            admin_key = str(reply.ok.key_expr)
            
            # Extract standard fields the bridge provides
            ros_topic = route_info.get('ros2_name', 'Unknown')
            zenoh_key = route_info.get('zenoh_key_expr', 'Unknown')
            
            # Print cleanly
            print(f"{ros_topic:<40} | {zenoh_key}")
            
        except Exception as e:
            print(f"Error parsing route data: {e}")

    if not found_topics:
        print("No topics found!")
        print("Troubleshooting:")
        print(" 1. Is the bridge container running on the Jetson?")
        print(" 2. Are the Foxy nodes running on the Jetson?")
        print(" 3. Does the ROS_DOMAIN_ID match between the container and the host?")

    session.close()

if __name__ == '__main__':
    main()