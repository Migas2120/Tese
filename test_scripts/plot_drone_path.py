import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Registers the 3D projection
import numpy as np

log_file = "/mnt/c/Users/mcaeiro/Desktop/Tese/logs/telemetry_drone0_20250520_123653.jsonl"  # Change to your file

xs, ys, zs, ts = [], [], [], []

with open(log_file, "r") as f:
    for line in f:
        try:
            entry = json.loads(line)
            pos = entry["pose"]["position"]
            xs.append(pos["x"])
            ys.append(pos["y"])
            zs.append(pos["z"])
            ts.append(entry["timestamp"])
        except Exception as e:
            print("Error parsing line:", e)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Path

ax.plot(xs, ys, zs, label='Drone Path', linewidth=2)

# Start/End markers
ax.scatter(xs[0], ys[0], zs[0], color='green', s=80, label='Start')
ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=80, label='End')

ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_title('Drone 3D Flight Path')
ax.legend()
plt.show()
