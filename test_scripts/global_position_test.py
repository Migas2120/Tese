from pymavlink import mavutil
import time

def send_gps_goal(lat, lon, alt):
    # Connect to SITL via TCP
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

    # Build and send the SET_POSITION_TARGET_GLOBAL_INT message
    master.mav.set_position_target_global_int_send(
        0,  # time_boot_ms (ignored)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # type_mask (only position)
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,  # velocities (ignored)
        0, 0, 0,  # accelerations (ignored)
        0, 0       # yaw, yaw_rate (ignored)
    )
    print(f"Sent SET_POSITION_TARGET_GLOBAL_INT to lat:{lat} lon:{lon} alt:{alt}")

if __name__ == "__main__":
    send_gps_goal(-40, 149.165337, 15.0)
