import socket
import json

HOST = "127.0.0.1"
PORT = 65432

MISSION_ID = "inspection_test_001"
DRONE_ID = 0  # Adjust to match your setup

def send_inspection_mission():
    mission = {
        "type": "add_mission",
        "mission_id": MISSION_ID,
        "drone_ids": [DRONE_ID],
        "waypoints": [
            [50.0, 0.0, 6.0],    # East
            [100.0, 0.0, 6.0],   # Further east
            [100.0, 50.0, 7.0],  # North
            [100.0, 100.0, 8.0], # Back west, lower  # Example waypoint 2 (slightly east, higher)
        ],
        "priority": 5,
        "description": "Test inspection mission via TCP client"
    }

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(json.dumps(mission).encode('utf-8'))
        print(f"[Client] Sent inspection mission: {json.dumps(mission, indent=2)}")

        # Optionally wait for server acknowledgment (if implemented)
        try:
            response = s.recv(1024)
            print(f"[Client] Received: {response.decode('utf-8')}")
        except Exception:
            pass

if __name__ == "__main__":
    print("Sending inspection mission...")
    send_inspection_mission()
