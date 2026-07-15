import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import glob
import numpy as np
import sys
import argparse
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

parser = argparse.ArgumentParser(description="3D trajectory visualizer (time, x, y)")
parser.add_argument("scenario", help="Scenario name")
parser.add_argument(
    "-v", "--violating", action="store_true", help="Show violating trajectories"
)
parser.add_argument(
    "-s", "--sampled", action="store_true", help="Show sampled trajectories"
)
parser.add_argument("-saf", action="store_true", help="Violating: safety property")
parser.add_argument("-tar", action="store_true", help="Violating: target property")
parser.add_argument("-sil", action="store_true", help="Violating: silence property")
parser.add_argument("--dpi", type=int, default=150, help="Output DPI (default 150)")
parser.add_argument(
    "--save", type=str, default=None, help="Save figure to this path (e.g. fig.png)"
)
args = parser.parse_args()

# --- Validate flags ---
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
    mode_label = f"Violating ({prop.upper()})"
else:
    subfolder = "sampled_trajectories"
    mode_label = "Sampled"

trajectory_dir = f"../generated_files/{args.scenario}/{subfolder}"

# --- Load data ---
columns = ["carID", "x", "y", "speed", "acceleration", "laneChangeCommand", "finished"]
files = sorted(glob.glob(f"{trajectory_dir}/*.csv"))
trajectories = []
for file in files:
    df = pd.read_csv(file, header=None, names=columns)
    trajectories.append(df)

if not trajectories:
    raise FileNotFoundError(f"No CSV files found in {trajectory_dir}")

n_cars = len(trajectories)
cmap = plt.cm.tab10 if n_cars <= 10 else plt.cm.tab20
car_colors = [cmap(i / max(n_cars - 1, 1)) for i in range(n_cars)]

# --- Compute data ranges ---
all_x = pd.concat([df["x"] for df in trajectories])
t_max = (max(len(df) for df in trajectories) - 1) / 10.0
x_min, x_max = all_x.min(), all_x.max()
x_pad = (x_max - x_min) * 0.05
x_min -= x_pad
x_max += x_pad

# --- Figure ---
fig = plt.figure(figsize=(14, 8), dpi=args.dpi)
fig.patch.set_facecolor("white")

ax = fig.add_subplot(111, projection="3d")
ax.set_facecolor("white")


# --- Draw trajectories ---
for i, df in enumerate(trajectories):
    color = car_colors[i]
    car_id = int(df["carID"].iloc[0])

    active = df[df["finished"] == 1.0]
    if active.empty:
        active = df

    t = np.arange(len(active)) / 10.0
    x = active["x"].values
    z = active["y"].values  # lateral position → Z axis

    # Gradient: segments fade from dim to bright over time
    alphas = np.linspace(0.2, 1.0, max(len(t) - 1, 1))
    for j in range(len(t) - 1):
        ax.plot(
            x[j : j + 2],
            t[j : j + 2],
            z[j : j + 2],
            color=color,
            linewidth=2.0,
            alpha=alphas[j],
            solid_capstyle="round",
        )

    # Start dot
    ax.scatter(
        x[0],
        t[0],
        z[0],
        color=color,
        s=45,
        marker="o",
        edgecolors="white",
        linewidths=0.8,
        zorder=6,
        alpha=0.7,
    )
    # End: flat rectangle (5m long x 2m wide) in X-Z plane at final time
    VLEN, VWID = 5, 2
    cx, ct, cz = x[-1], t[-1], z[-1]
    corners = [
        [cx - VLEN / 2, ct, cz - VWID / 2],
        [cx + VLEN / 2, ct, cz - VWID / 2],
        [cx + VLEN / 2, ct, cz + VWID / 2],
        [cx - VLEN / 2, ct, cz + VWID / 2],
    ]
    poly = Poly3DCollection(
        [corners],
        facecolor=color,
        edgecolor="#333333",
        linewidth=1.0,
        alpha=0.9,
        zorder=7,
    )
    ax.add_collection3d(poly)

# --- Axis limits ---
ax.set_xlim(x_min, x_max)
ax.set_ylim(0, t_max * 1.1)
ax.set_zlim(0, 8)

# --- Axis labels & ticks ---
ax.set_xlabel("X  (m)", color="#333333", labelpad=12, fontsize=10)
ax.set_ylabel("Time  (s)", color="#333333", labelpad=12, fontsize=10)
ax.set_zlabel("Lane  (m)", color="#333333", labelpad=8, fontsize=10)

ax.set_zticks([2, 6])
ax.set_zticklabels(["Right\n(0–4)", "Left\n(4–8)"], color="#333333", fontsize=8)
ax.tick_params(colors="#333333", labelsize=8)

# --- Pane styling ---
for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
    pane.fill = False
    pane.set_edgecolor("#cccccc")
ax.grid(True, color="#cccccc", linewidth=0.5, alpha=0.6)

# --- Title ---
fig.suptitle(
    f"{args.scenario}  ·  Trajectory",
    color="#111111",
    fontsize=13,
    fontweight="bold",
    y=0.97,
)

# --- Legend ---
handles = []
for i, df in enumerate(trajectories):
    car_id = int(df["carID"].iloc[0])
    handles.append(
        mpatches.Patch(
            facecolor=car_colors[i],
            edgecolor="white",
            label=f"Car {car_id}",
            linewidth=0.8,
        )
    )

legend = ax.legend(
    handles=handles,
    loc="upper left",
    fontsize=8,
    framealpha=0.3,
    facecolor="white",
    edgecolor="#cccccc",
    labelcolor="#111111",
    title="Vehicles",
    title_fontsize=8,
)
legend.get_title().set_color("#333333")

# --- View angle: X runs left→right, Time runs into the scene, Z is height ---
ax.view_init(elev=20, azim=-60)

if args.save:
    plt.savefig(args.save, dpi=args.dpi, bbox_inches="tight", facecolor="white")
    print(f"Saved to {args.save}")
else:
    plt.show()
