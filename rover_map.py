
import numpy as np
import matplotlib.pyplot as plt
import os

# =================================
# AUTONOMOUS SLAM ROVER MAPPING
# =================================

DATA_FILE = "rover_serial.txt"

MAP_SIZE = 61
CELL_SIZE = 10
CENTER = MAP_SIZE // 2
MAX_RANGE = 250

os.makedirs("output", exist_ok=True)

# =================================
# READ SENSOR DATA
# =================================

coordinates = []

with open(DATA_FILE, "r") as file:

    for line in file:

        parts = line.strip().split(",")

        if len(parts) == 5 and parts[0] == "COORD":

            try:
                angle = float(parts[1])
                distance = float(parts[2])
                x = float(parts[3])
                y = float(parts[4])

                if 0 < distance < MAX_RANGE:
                    coordinates.append(
                        [angle, distance, x, y]
                    )

            except ValueError:
                continue

print("Total sensor readings:", len(coordinates))

if not coordinates:
    raise ValueError("No valid COORD data found!")

# =================================
# CREATE MAP
# =================================

grid = np.full(
    (MAP_SIZE, MAP_SIZE),
    -1,
    dtype=int
)

obstacle_x = []
obstacle_y = []

for angle, distance, x, y in coordinates:

    gx = round(x / CELL_SIZE) + CENTER
    gy = round(y / CELL_SIZE) + CENTER

    if 0 <= gx < MAP_SIZE and 0 <= gy < MAP_SIZE:

        grid[gy, gx] = 1

        obstacle_x.append(x)
        obstacle_y.append(y)

# =================================
# VISUALIZE 2D MAP
# =================================

fig, ax = plt.subplots(figsize=(10, 8))

ax.set_facecolor("#333333")

ax.scatter(
    obstacle_x,
    obstacle_y,
    color="red",
    s=65,
    label="Detected obstacle endpoints"
)

ax.scatter(
    0,
    0,
    color="cyan",
    marker="^",
    s=180,
    label="Initial rover position"
)

# Draw sensor scan rays
for angle, distance, x, y in coordinates:

    ax.plot(
        [0, x],
        [0, y],
        color="orange",
        alpha=0.15,
        linewidth=0.8
    )

ax.set_xlim(-300, 300)
ax.set_ylim(-300, 300)

ax.set_xlabel("X Coordinate (cm)")
ax.set_ylabel("Y Coordinate (cm)")

ax.set_title(
    "Autonomous SLAM Rover - 2D Mapping"
)

ax.set_aspect("equal")

ax.grid(
    True,
    linestyle="--",
    alpha=0.3
)

ax.legend()

# =================================
# SAVE MAP
# =================================

plt.savefig(
    "output/final_2D_map.png",
    dpi=300,
    bbox_inches="tight"
)

print("2D Map generated successfully!")
print("Saved: output/final_2D_map.png")

plt.show()
