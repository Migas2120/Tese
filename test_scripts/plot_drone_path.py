import json
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

log_file = "/mnt/c/Users/mcaeiro/Desktop/Tese/logs/telemetry_drone0_20250521_124436.jsonl"

xs, ys, zs = [], [], []
battery_percents, battery_voltages = [], []
times = []
distances = [0.0]
flying_flags = []
modes = []

with open(log_file, "r") as f:
    last_pos = None
    for idx, line in enumerate(f):
        try:
            entry = json.loads(line)
            # Check all required keys are present and not None
            if (
                "pose" in entry and entry["pose"] is not None and
                "position" in entry["pose"] and entry["pose"]["position"] is not None and
                "battery" in entry and entry["battery"] is not None and
                "status" in entry and entry["status"] is not None
            ):
                pos = entry["pose"]["position"]
                batt = entry["battery"].get("percentage", None)
                volt = entry["battery"].get("voltage", None)
                t = entry.get("timestamp", None)
                mode = entry["status"].get("mode", None)
                flying = entry["status"].get("flying", None)

                # Only append if all values are valid (can be stricter if you want)
                if None not in (pos.get("x"), pos.get("y"), pos.get("z"), batt, volt, t, mode, flying):
                    x, y, z = pos["x"], pos["y"], pos["z"]
                    xs.append(x)
                    ys.append(y)
                    zs.append(z)
                    battery_percents.append(batt)
                    battery_voltages.append(volt)
                    times.append(t)
                    flying_flags.append(flying)
                    modes.append(mode)

                    # Cumulative distance
                    if last_pos is not None:
                        dx = x - last_pos[0]
                        dy = y - last_pos[1]
                        dz = z - last_pos[2]
                        d = (dx**2 + dy**2 + dz**2)**0.5
                        distances.append(distances[-1] + d)
                    last_pos = (x, y, z)
                else:
                    print(f"Skipped line {idx}: some fields are None")
            else:
                print(f"Skipped line {idx}: missing required keys")
        except Exception as e:
            print(f"Error parsing line {idx}: {e}")

# -------- Plot Battery % vs. Distance --------
plt.figure(figsize=(10, 6))
plt.plot(distances, battery_percents, marker='o', label="Battery %")
plt.xlabel("Cumulative Distance Traveled (meters)")
plt.ylabel("Battery Percentage")
plt.title("Battery % vs. Distance Traveled")
plt.grid(True)

# Also plot voltage if you want (second y-axis)
plt.twinx()
plt.plot(distances, battery_voltages, color='orange', linestyle='--', label="Voltage (V)")
plt.ylabel("Battery Voltage (V)")
plt.legend(loc='upper right')

plt.show()

# -------- Plot flying (shaded) regions --------
plt.figure(figsize=(10, 3))
plt.plot(times, battery_percents, label="Battery %")
plt.xlabel("Timestamp")
plt.ylabel("Battery %")
plt.title("Battery % over Time (Flying Regions Shaded)")
plt.grid(True)

# Shade regions where flying==True
for i in range(1, len(times)):
    if flying_flags[i]:
        plt.axvspan(times[i-1], times[i], color='green', alpha=0.2)
plt.show()

# -------- 3D Trajectory colored by battery % (optional/fancy) --------
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter(xs, ys, zs, c=battery_percents, cmap='viridis', s=30, label="Flight Path")
ax.plot(xs, ys, zs, color='gray', alpha=0.4)
ax.scatter(xs[0], ys[0], zs[0], color='green', s=60, label='Start')
ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=60, label='End')
fig.colorbar(sc, ax=ax, label="Battery %")
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_zlabel('Z (meters)')
ax.set_title('3D Flight Path (colored by battery %)')
ax.legend()
plt.show()

# -------- Efficiency: Meters per % battery --------
efficiency = []
for i in range(1, len(distances)):
    delta_dist = distances[i] - distances[i-1]
    delta_batt = battery_percents[i-1] - battery_percents[i]
    if delta_batt > 0:
        efficiency.append(delta_dist / delta_batt)
    else:
        efficiency.append(np.nan)  # Ignore for 0% change

plt.figure(figsize=(10,5))
plt.plot(distances[1:], efficiency, marker='x')
plt.xlabel("Cumulative Distance (m)")
plt.ylabel("Meters per % Battery")
plt.title("Efficiency: Meters per % Battery Used")
plt.grid(True)
plt.show()
