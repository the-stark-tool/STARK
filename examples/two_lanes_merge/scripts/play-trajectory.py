import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
from matplotlib.animation import FuncAnimation
import glob
import numpy as np
import sys
import argparse

parser = argparse.ArgumentParser(description="Visualize road simulation trajectories")
parser.add_argument("scenario", help="Scenario name")
parser.add_argument(
    "-v", "--violating", action="store_true", help="Show violating trajectories"
)
parser.add_argument(
    "-s", "--sampled", action="store_true", help="Show sampled trajectories"
)
parser.add_argument(
    "-saf", action="store_true", help="Violating: safety property (saf)"
)
parser.add_argument(
    "-tar", action="store_true", help="Violating: target property (tar)"
)
parser.add_argument(
    "-sil", action="store_true", help="Violating: silence property (sil)"
)
args = parser.parse_args()

if args.violating and args.sampled:
    print("Error: specify either -v or -s, not both")
    sys.exit(1)
if not args.violating and not args.sampled:
    print("Error: specify either -v or -s")
    sys.exit(1)

property_flags = [args.saf, args.tar, args.sil]
if args.violating:
    if sum(property_flags) == 0:
        print("Error: -v requires one of -saf, -tar, or -sil")
        sys.exit(1)
    if sum(property_flags) > 1:
        print("Error: specify only one of -saf, -tar, -sil")
        sys.exit(1)
if args.sampled and any(property_flags):
    print("Error: -saf, -tar, -sil are only valid with -v")
    sys.exit(1)

if args.violating:
    prop = "saf" if args.saf else "tar" if args.tar else "sil"
    subfolder = f"violating_trajectories/{prop}"
else:
    subfolder = "sampled_trajectories"
trajectory_dir = f"../generated_files/{args.scenario}/{subfolder}"

# --- Target sim lenght in s ---
TARGET_DURATION_S = 20

# --- Physical constants ---
VEHICLE_LENGTH = 5
VEHICLE_WIDTH = 2

# Typical values used for scaling reference
REF_SPEED = 15  # m/s  → arrow will be ~12% of visible x range
REF_ACC = 2  # m/s² → arrow will be ~6% of visible x range
LCC_SCALE = 1.5  # y-units per unit of laneChangeCommand (road is 8 wide)

# Vector colors (same across all cars, distinguish type not car)
SPEED_COLOR = "royalblue"
ACC_COLOR = "orangered"
LCC_COLOR = "forestgreen"

# --- Load data ---
files = sorted(glob.glob(f"{trajectory_dir}/*.csv"))
columns = ["carID", "x", "y", "speed", "acceleration", "laneChangeCommand", "finished"]

trajectories = []
for file in files:
    df = pd.read_csv(file, header=None, names=columns)
    trajectories.append(df)

if not trajectories:
    raise FileNotFoundError(f"No CSV files found in {trajectory_dir}")

n_cars = len(trajectories)
car_colors = plt.cm.Set1(np.linspace(0, 0.9, max(n_cars, 1)))

# --- Figure setup ---
fig, ax = plt.subplots(figsize=(16, 5))
fig.patch.set_facecolor("#f0f0f5")
ax.set_facecolor("#e4e4ec")

all_x = pd.concat([df["x"] for df in trajectories])
x_padding = 20
x_min = all_x.min() - x_padding
x_max = all_x.max() + x_padding
ax.set_xlim(x_min, x_max)
ax.set_ylim(-0.3, 8.3)

# --- Compute display scales from actual axis range ---
# The x range is large (e.g. 0-250) while y is only 8.
# All visual sizes that go along x must be scaled accordingly.
x_range = x_max - x_min

# Use real physical length — will look skinny due to axis aspect ratio,
# but preserves the true 5m x 2m footprint in data space.
DISP_LENGTH = VEHICLE_LENGTH

# Vector scales: speed=REF_SPEED -> 12% of x_range; acc=REF_ACC -> 6% of x_range
SPEED_SCALE = (x_range * 0.12) / REF_SPEED
ACC_SCALE = (x_range * 0.06) / REF_ACC

# --- Road visuals ---
ax.axhspan(0, 8, color="#c8c8d4", zorder=0)
ax.axhline(y=0, color="#555555", linewidth=2.5, zorder=1)
ax.axhline(y=8, color="#555555", linewidth=2.5, zorder=1)
ax.axhline(y=4, color="#888888", linewidth=1.5, linestyle="--", alpha=0.7, zorder=1)

if args.violating:
    prop = "saf" if args.saf else "tar" if args.tar else "sil"
    subfolder = f"violating_trajectories/{prop}"
    mode = f"Violating ({prop})"
else:
    subfolder = "sampled_trajectories"
    mode = "Sampled"
ax.set_title(
    f"{args.scenario} — {mode} Trajectories", color="#222222", fontsize=13, pad=10
)
ax.set_xlabel("X (m)", color="#444444")
ax.set_ylabel("Lane", color="#444444")
ax.set_yticks([2, 6])
ax.set_yticklabels(["Right Lane\n(y 0–4)", "Left Lane\n(y 4–8)"], color="#444444")
ax.tick_params(colors="#444444")
for spine in ax.spines.values():
    spine.set_edgecolor("#999999")

# --- Per-car persistent artists ---
car_rects = []
trail_lines = []

for i in range(n_cars):
    color = car_colors[i]
    rect = plt.Rectangle(
        (0, 0),
        DISP_LENGTH,
        VEHICLE_WIDTH,
        linewidth=1.5,
        edgecolor="#333333",
        facecolor=color,
        zorder=4,
        alpha=0.9,
    )
    ax.add_patch(rect)
    car_rects.append(rect)

    (trail,) = ax.plot([], [], "-", color=color, alpha=0.4, linewidth=1, zorder=2)
    trail_lines.append(trail)

# Dynamic quiver arrows — cleared and redrawn each frame
arrow_artists = []

# --- Legend ---
legend_handles = []

for i, df in enumerate(trajectories):
    car_id = int(df["carID"].iloc[0])
    legend_handles.append(
        mpatches.Patch(
            facecolor=car_colors[i], edgecolor="#333333", label=f"Car ID {car_id}"
        )
    )

legend_handles.append(mpatches.Patch(color="none", label=""))  # spacer

legend_handles.append(
    mlines.Line2D(
        [],
        [],
        color=SPEED_COLOR,
        linewidth=2.5,
        marker=">",
        markersize=7,
        label=f"Speed  (ref {REF_SPEED} m/s = 12% of x-range)",
    )
)
legend_handles.append(
    mlines.Line2D(
        [],
        [],
        color=ACC_COLOR,
        linewidth=2.5,
        marker=">",
        markersize=7,
        label=f"Acceleration  (ref {REF_ACC} m/s² = 6% of x-range)",
    )
)
legend_handles.append(
    mlines.Line2D(
        [],
        [],
        color=LCC_COLOR,
        linewidth=2.5,
        marker="^",
        markersize=7,
        label="Lane Change Cmd  (y-axis, hidden when 0)",
    )
)

ax.legend(
    handles=legend_handles,
    loc="upper right",
    fontsize=8,
    framealpha=0.7,
    facecolor="#ffffff",
    edgecolor="#999999",
    labelcolor="#222222",
)

max_frames = max(len(df) for df in trajectories)

frame_text = ax.text(
    0.01, 0.95, "", transform=ax.transAxes, color="#444444", fontsize=8, va="top"
)


# --- Animation update ---
def update(frame):
    for a in arrow_artists:
        a.remove()
    arrow_artists.clear()

    for i, df in enumerate(trajectories):
        if frame >= len(df):
            continue

        row = df.iloc[frame]
        x, y = row["x"], row["y"]

        # Hide car when finished
        if row["finished"] == 0.0:
            car_rects[i].set_visible(False)
            continue

        # Rectangle centered on (x, y)
        car_rects[i].set_xy((x - DISP_LENGTH / 2, y - VEHICLE_WIDTH / 2))

        # Trail
        trail_lines[i].set_data(
            df.iloc[: frame + 1]["x"],
            df.iloc[: frame + 1]["y"],
        )

        qkw = dict(
            scale=1,
            scale_units="xy",
            angles="xy",
            width=0.004,
            headwidth=4,
            headlength=5,
            zorder=5,
        )

        # Speed vector — slightly above centre
        q = ax.quiver(
            x, y + 0.3, row["speed"] * SPEED_SCALE, 0, color=SPEED_COLOR, **qkw
        )
        arrow_artists.append(q)

        # Acceleration vector — slightly below centre
        q = ax.quiver(
            x, y - 0.3, row["acceleration"] * ACC_SCALE, 0, color=ACC_COLOR, **qkw
        )
        arrow_artists.append(q)

        # Lane change command — vertical, only when non-zero
        lcc = row["laneChangeCommand"]
        if abs(lcc) > 0.01:
            q = ax.quiver(x, y, 0, lcc * LCC_SCALE, color=LCC_COLOR, **qkw)
            arrow_artists.append(q)

    frame_text.set_text(f"t = {frame}")
    return car_rects + trail_lines + arrow_artists + [frame_text]


ani = FuncAnimation(
    fig,
    update,
    frames=max_frames,
    interval=(TARGET_DURATION_S * 1000) / max_frames,
    blit=False,
    repeat=False,
)

plt.tight_layout()
plt.show()
