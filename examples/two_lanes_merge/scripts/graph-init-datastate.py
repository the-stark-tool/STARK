import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
import glob
import numpy as np
import sys
import argparse

parser = argparse.ArgumentParser(
    description="Visualize initial conditions of a scenario"
)
parser.add_argument("scenario", help="Scenario name")
parser.add_argument("-v", "--violating", action="store_true")
parser.add_argument("-s", "--sampled", action="store_true")
parser.add_argument("-saf", action="store_true")
parser.add_argument("-tar", action="store_true")
parser.add_argument("-sil", action="store_true")
parser.add_argument("--dpi", type=int, default=150)
parser.add_argument("--save", type=str, default=None)
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
    mode = f"Violating ({prop.upper()})"
else:
    subfolder = "sampled_trajectories"
    mode = "Sampled"

trajectory_dir = f"../generated_files/{args.scenario}/{subfolder}"

columns = ["carID", "x", "y", "speed", "acceleration", "laneChangeCommand", "finished"]
files = sorted(glob.glob(f"{trajectory_dir}/*.csv"))
rows = []
for file in files:
    df = pd.read_csv(file, header=None, names=columns)
    rows.append(df.iloc[0])

if not rows:
    raise FileNotFoundError(f"No CSV files found in {trajectory_dir}")

n_cars = len(rows)
cmap = plt.cm.tab10 if n_cars <= 10 else plt.cm.tab20
car_colors = [cmap(i / max(n_cars - 1, 1)) for i in range(n_cars)]

VEHICLE_LENGTH = 5
VEHICLE_WIDTH = 2

# Fixed x range
X_MIN, X_MAX = 0, 150
x_range = X_MAX - X_MIN

# Arrow scaling: speed ref=15 m/s → 10% of x_range; acc ref=2 m/s² → 5%
REF_SPEED = 15
REF_ACC = 2
SPEED_SCALE = (x_range * 0.10) / REF_SPEED
ACC_SCALE = (x_range * 0.05) / REF_ACC

SPEED_COLOR = "dodgerblue"
ACC_COLOR = "tomato"

fig, ax = plt.subplots(figsize=(8, 7), dpi=args.dpi)
fig.patch.set_facecolor("white")
ax.set_facecolor("white")

ax.set_xlim(X_MIN, X_MAX)
ax.set_ylim(-2, 11)

# --- Road ---
ax.axhspan(0, 8, color="#f0f0f0", zorder=0)
ax.axhline(y=0, color="#555555", linewidth=2, zorder=1)
ax.axhline(y=8, color="#555555", linewidth=2, zorder=1)
ax.axhline(y=4, color="#aaaaaa", linewidth=1.2, linestyle="--", alpha=0.8, zorder=1)

# --- Draw each car ---
qkw = dict(
    scale=1,
    scale_units="xy",
    angles="xy",
    width=0.003,
    headwidth=5,
    headlength=6,
    zorder=6,
)

for i, row in enumerate(rows):
    color = car_colors[i]
    car_id = int(row["carID"])
    x, y = row["x"], row["y"]
    spd = row["speed"]
    acc = row["acceleration"]

    # Car rectangle
    rect = plt.Rectangle(
        (x - VEHICLE_LENGTH / 2, y - VEHICLE_WIDTH / 2),
        VEHICLE_LENGTH,
        VEHICLE_WIDTH,
        linewidth=1.5,
        edgecolor="#333333",
        facecolor=color,
        zorder=4,
        alpha=0.9,
    )
    ax.add_patch(rect)

    # Car ID label above
    ax.text(
        x,
        y + VEHICLE_WIDTH / 2 + 0.25,
        f"Car {car_id}",
        ha="center",
        va="bottom",
        fontsize=8,
        color="#222222",
        fontweight="bold",
        zorder=5,
    )

    # Speed arrow (above centre, pointing right)
    ax.quiver(x, y + 0.3, spd * SPEED_SCALE, 0, color=SPEED_COLOR, **qkw)
    # Speed label at arrow tip
    ax.text(
        x + spd * SPEED_SCALE + 0.5,
        y + 0.3,
        f"{spd:.1f} m/s",
        va="center",
        fontsize=7,
        color=SPEED_COLOR,
    )

    # Acceleration arrow (below centre, pointing right; left if negative)
    ax.quiver(x, y - 0.3, acc * ACC_SCALE, 0, color=ACC_COLOR, **qkw)
    ax.text(
        x + acc * ACC_SCALE + 0.5,
        y - 0.3,
        f"{acc:+.1f} m/s²",
        va="center",
        fontsize=7,
        color=ACC_COLOR,
    )

# --- Axes ---
ax.set_xlabel("X (m)", color="#333333", fontsize=10)
ax.set_yticks([2, 6])
ax.set_yticklabels(["Right\n(0–4)", "Left\n(4–8)"], fontsize=8, color="#555555")
ax.tick_params(colors="#555555")
for spine in ax.spines.values():
    spine.set_edgecolor("#cccccc")

ax.set_title(
    f"{args.scenario}  ·  Initial Conditions",
    fontsize=12,
    fontweight="bold",
    color="#111111",
    pad=10,
)

# --- Legend ---
legend_handles = [
    mlines.Line2D(
        [], [], color=SPEED_COLOR, linewidth=2, marker=">", markersize=7, label="Speed"
    ),
    mlines.Line2D(
        [],
        [],
        color=ACC_COLOR,
        linewidth=2,
        marker=">",
        markersize=7,
        label="Acceleration",
    ),
]
ax.legend(
    handles=legend_handles,
    loc="upper right",
    fontsize=8,
    framealpha=0.6,
    edgecolor="#cccccc",
)

plt.tight_layout()

if args.save:
    plt.savefig(args.save, dpi=args.dpi, bbox_inches="tight", facecolor="white")
    print(f"Saved to {args.save}")
else:
    plt.show()
