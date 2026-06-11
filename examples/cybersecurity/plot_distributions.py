"""
plot_distributions.py

Produces four publication-quality plots from the per-run CSVs written by
TwoLanesTwoCarsLiDARAttack.java (runs_nominal.csv and runs_attacked.csv).

These plots visualise the distributions that STARK's RobTL formulas are
comparing under the hood:

    crash_probability.png    - cumulative crash fraction over time
    speed_distribution.png   - my_speed median + percentile bands per step
    distance_distribution.png - inter-car distance median + percentile bands
    position_over_time.png   - my_x median + percentile bands per step
                                (visualises the delayed-distance metric)

Requires:
    pip install pandas matplotlib

Usage:
    python plot_distributions.py [nominal_csv] [attacked_csv] [output_dir]

Defaults:
    nominal_csv  = runs_nominal.csv
    attacked_csv = runs_attacked.csv
    output_dir   = .
"""

import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ----- Visual style --------------------------------------------------------
NOMINAL_COLOR = "#2E7D32"      # green
ATTACKED_COLOR = "#C62828"     # red
GRID_COLOR = "#E0E0E0"
FIG_DPI = 150
FIG_SIZE = (10, 5.5)


def configure_style() -> None:
    """Single point of style configuration so all three plots match."""
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 11,
        "axes.titlesize": 14,
        "axes.titleweight": "bold",
        "axes.labelsize": 12,
        "axes.edgecolor": "#666666",
        "axes.grid": True,
        "grid.color": GRID_COLOR,
        "grid.linewidth": 0.8,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "legend.frameon": False,
        "legend.fontsize": 11,
        "figure.dpi": FIG_DPI,
        "savefig.dpi": FIG_DPI,
        "savefig.bbox": "tight",
    })


# ----- Data loading --------------------------------------------------------
def load_runs(nominal_path: str, attacked_path: str) -> pd.DataFrame:
    """Load and concatenate the two run CSVs into one tidy DataFrame."""
    nominal = pd.read_csv(nominal_path)
    if os.path.exists(attacked_path):
        attacked = pd.read_csv(attacked_path)
        df = pd.concat([nominal, attacked], ignore_index=True)
    else:
        print(f"Note: {attacked_path} not found - plotting nominal only.")
        df = nominal
    return df


# ----- Plot 1: cumulative crash probability --------------------------------
def plot_crash_probability(df: pd.DataFrame, output_path: str) -> None:
    """At each step, fraction of runs that have crashed by then."""
    fig, ax = plt.subplots(figsize=FIG_SIZE)

    for condition, color in [("nominal", NOMINAL_COLOR)] + sorted(
        [(c, ATTACKED_COLOR) for c in df["condition"].unique() if c != "nominal"]
    ):
        sub = df[df["condition"] == condition]
        # crash flag latches once set, so the per-step crash fraction is
        # already the cumulative crash probability
        prob = sub.groupby("step")["crash"].mean()
        label = "Nominal" if condition == "nominal" else f"Attacked ({condition})"
        ax.plot(prob.index, prob.values, color=color, linewidth=2.2, label=label)

    ax.set_xlabel("Simulation step")
    ax.set_ylabel("Cumulative crash probability")
    ax.set_title("Crash probability over time")
    ax.set_ylim(-0.02, 1.02)
    ax.legend(loc="best")

    fig.savefig(output_path)
    plt.close(fig)
    print(f"Wrote {output_path}")


# ----- Plot 2 & 3: distribution bands --------------------------------------
def plot_distribution_bands(df: pd.DataFrame, observable: str, ylabel: str,
                             title: str, output_path: str) -> None:
    """Median line plus 25-75% and 5-95% bands per step, one panel per condition."""
    conditions = list(df["condition"].unique())
    fig, axes = plt.subplots(1, len(conditions), figsize=(FIG_SIZE[0] * len(conditions) / 2,
                                                            FIG_SIZE[1]),
                              sharey=True)
    if len(conditions) == 1:
        axes = [axes]

    # Compute a shared y-axis range covering both conditions for fair comparison
    quantiles_global = df.groupby(["condition", "step"])[observable].quantile([0.05, 0.95])
    y_min = float(quantiles_global.min())
    y_max = float(quantiles_global.max())
    y_pad = (y_max - y_min) * 0.05
    y_lim = (y_min - y_pad, y_max + y_pad)

    for ax, condition in zip(axes, conditions):
        sub = df[df["condition"] == condition]
        color = NOMINAL_COLOR if condition == "nominal" else ATTACKED_COLOR
        label = "Nominal" if condition == "nominal" else f"Attacked ({condition})"

        # Compute per-step quantiles
        grouped = sub.groupby("step")[observable]
        median = grouped.median()
        q25 = grouped.quantile(0.25)
        q75 = grouped.quantile(0.75)
        q05 = grouped.quantile(0.05)
        q95 = grouped.quantile(0.95)
        steps = median.index

        ax.fill_between(steps, q05, q95, color=color, alpha=0.15, linewidth=0,
                        label="5-95% range")
        ax.fill_between(steps, q25, q75, color=color, alpha=0.30, linewidth=0,
                        label="25-75% range")
        ax.plot(steps, median, color=color, linewidth=2.0, label="Median")

        ax.set_title(label)
        ax.set_xlabel("Simulation step")
        ax.set_ylim(y_lim)
        ax.legend(loc="best")

    axes[0].set_ylabel(ylabel)
    fig.suptitle(title, fontsize=15, fontweight="bold", y=1.02)
    fig.savefig(output_path)
    plt.close(fig)
    print(f"Wrote {output_path}")


# ----- Entry point ---------------------------------------------------------
if __name__ == "__main__":
    nominal_path = sys.argv[1] if len(sys.argv) > 1 else "runs_nominal.csv"
    attacked_path = sys.argv[2] if len(sys.argv) > 2 else "runs_attacked.csv"
    output_dir = sys.argv[3] if len(sys.argv) > 3 else "."

    if not os.path.exists(nominal_path):
        print(f"Error: file not found: {nominal_path}")
        sys.exit(1)

    os.makedirs(output_dir, exist_ok=True)
    configure_style()

    df = load_runs(nominal_path, attacked_path)
    print(f"Loaded {len(df):,} rows ({df['run'].nunique()} runs per condition, "
          f"{df['step'].nunique()} steps)")

    plot_crash_probability(df, os.path.join(output_dir, "crash_probability.png"))
    plot_distribution_bands(df, "my_speed", "Ego car speed (units/step)",
                             "Ego speed distribution: nominal vs attacked",
                             os.path.join(output_dir, "speed_distribution.png"))
    plot_distribution_bands(df, "dist", "Inter-car distance (units)",
                             "Inter-car distance distribution: nominal vs attacked",
                             os.path.join(output_dir, "distance_distribution.png"))
    plot_distribution_bands(df, "my_x", "Ego car position (units)",
                             "Ego position over time: nominal vs attacked",
                             os.path.join(output_dir, "position_over_time.png"))