import socket
import json
import time

HOST = "127.0.0.1"
PORT = 65432

DRONE_ID = 0  # Adjust as needed

def send_tcp_message(payload):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(json.dumps(payload).encode('utf-8'))
        print(f"[Client] Sent: {payload}")

def send_normal_mission(mission_id, waypoints, description):
    mission = {
        "type": "add_mission",
        "mission_id": mission_id,
        "drone_ids": [DRONE_ID],
        "waypoints": waypoints,
        "priority": 5,
        "description": description
    }
    send_tcp_message(mission)

if __name__ == "__main__":
    print("Sending first normal mission...")
    send_normal_mission(
        mission_id="normal_test_001",
        waypoints=[
            [20.0, 0.0, 5.0],
            [0, 0, 5.0],
            [10.0, 20.0, 5.0],
        ],
        description="Test: Normal mission 1 via TCP client"
    )
    print("First mission sent. Press Enter to send a second mission (should queue if drone busy).")
    input()
    send_normal_mission(
        mission_id="normal_test_002",
        waypoints=[
            [30.0, 10.0, 5.0],
            [15.0, -5.0, 5.0],
            [5.0, 5.0, 5.0],
        ],
        description="Test: Normal mission 2 via TCP client"
    )
    print("Second mission sent!")
