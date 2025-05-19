import socket
import json
import time

HOST = "127.0.0.1"
PORT = 65432

MISSION_ID = "immediate_cancel_test_001"
DRONE_ID = 0  # Adjust as needed

def send_json_message(message):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(json.dumps(message).encode('utf-8'))
        print(f"Sent: {json.dumps(message)}")


def start_mission():
    msg = {
        "type": "add_mission",
        "mission_id": MISSION_ID,
        "drone_ids": [DRONE_ID],
        "waypoints": [
            [37.7749, -122.4194, 10.0],   # Example: San Francisco
            [37.7750, -122.4180, 10.0]
        ],
        "priority": 10,
        "description": "Test mission to be cancelled"
    }
    send_json_message(msg)

def cancel_mission():
    msg = {
        "type": "cancel_mission",
        "mission_id": MISSION_ID
    }
    send_json_message(msg)

if __name__ == "__main__":
    print("Starting immediate mission...")
    start_mission()
    print("Waiting 5 seconds before cancelling...")
    time.sleep(5)
    print("Cancelling mission...")
    cancel_mission()
