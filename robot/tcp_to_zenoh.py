import socket
import struct
import zenoh

def recvall(sock, n):
    """Helper function to read exactly n bytes from the TCP socket."""
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data

def main():
    # 1. Setup Zenoh Connection to your Server
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", '["tcp/100.125.156.19:7447"]')
    print("[Zenoh] Connecting to remote router...")
    z_session = zenoh.open(conf)
    z_pub = z_session.declare_publisher("my_robot/rt/utlidar/cloud")

    # 2. Setup Local TCP Server to listen for the ROS script
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Allows the port to be reused immediately if you restart the script
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
    server_sock.bind(('127.0.0.1', 50000))
    server_sock.listen(1)
    
    print("[Local] Waiting for ROS 2 script to connect on port 50000...")
    conn, addr = server_sock.accept()
    print(f"[Local] Connected to ROS 2 script at {addr}")

    try:
        while True:
            # 1. Read the 4-byte header to find out the payload size
            raw_msglen = recvall(conn, 4)
            if not raw_msglen:
                break
            
            # Unpack the integer size
            msglen = struct.unpack('<I', raw_msglen)[0]
            
            # 2. Read the exact amount of bytes for the payload
            payload = recvall(conn, msglen)
            if not payload:
                break

            # 3. Blast the binary payload to the remote server via Zenoh
            z_pub.put(payload)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        conn.close()
        server_sock.close()
        z_session.close()

if __name__ == '__main__':
    main()