"""Plot the nominal and attacked CapSel run distributions.

Usage:
    python plot_distributions.py [runs_nominal.csv] [runs_attacked.csv] [output_dir]
"""

import os
import sys

import matplotlib.pyplot as plt
import pandas as pd


NOMINAL_COLOR = "#2E7D32"
ATTACKED_COLOR = "#C62828"
FIGURE_SIZE = (10, 5.5)


def configure_style() -> None:
    """Use one clear style for all plots."""
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 11,
            "axes.titlesize": 14,
            "axes.titleweight": "bold",
            "axes.labelsize": 12,
            "axes.grid": True,
            "grid.color": "#E0E0E0",
            "axes.spines.top": False,
            "axes.spines.right": False,
            "legend.frameon": False,
            "figure.dpi": 150,
            "savefig.dpi": 150,
            "savefig.bbox": "tight",
        }
    )


def load_runs(nominal_path: str, attacked_path: str) -> pd.DataFrame:
    """Read and join the two run files."""
    nominal = pd.read_csv(nominal_path)
    if not os.path.exists(attacked_path):
        print(f"Note: {attacked_path} was not found. Only nominal data will be shown.")
        return nominal
    return pd.concat([nominal, pd.read_csv(attacked_path)], ignore_index=True)


def condition_style(condition: str) -> tuple[str, str]:
    """Return the label and color for one condition."""
    if condition == "nominal":
        return "Nominal", NOMINAL_COLOR
    return f"Attacked ({condition})", ATTACKED_COLOR


def plot_probability(df: pd.DataFrame, value: str, ylabel: str, title: str, path: str) -> None:
    """Plot the fraction of runs that have a true state flag."""
    figure, axis = plt.subplots(figsize=FIGURE_SIZE)
    for condition in df["condition"].unique():
        label, color = condition_style(condition)
        values = df[df["condition"] == condition].groupby("step")[value].mean()
        axis.plot(values.index, values.values, color=color, linewidth=2.2, label=label)
    axis.set_xlabel("Simulation step")
    axis.set_ylabel(ylabel)
    axis.set_title(title)
    axis.set_ylim(-0.02, 1.02)
    axis.legend(loc="best")
    figure.savefig(path)
    plt.close(figure)
    print(f"Wrote {path}")


def plot_distribution(
    df: pd.DataFrame,
    value: str,
    ylabel: str,
    title: str,
    path: str,
) -> None:
    """Plot the median and percentile bands for one value."""
    conditions = list(df["condition"].unique())
    figure, axes = plt.subplots(
        1,
        len(conditions),
        figsize=(FIGURE_SIZE[0] * max(1, len(conditions)) / 2, FIGURE_SIZE[1]),
        sharey=True,
    )
    if len(conditions) == 1:
        axes = [axes]

    for axis, condition in zip(axes, conditions):
        label, color = condition_style(condition)
        grouped = df[df["condition"] == condition].groupby("step")[value]
        median = grouped.median()
        low = grouped.quantile(0.05)
        low_middle = grouped.quantile(0.25)
        high_middle = grouped.quantile(0.75)
        high = grouped.quantile(0.95)
        steps = median.index

        axis.fill_between(steps, low, high, color=color, alpha=0.15, linewidth=0)
        axis.fill_between(steps, low_middle, high_middle, color=color, alpha=0.30, linewidth=0)
        axis.plot(steps, median, color=color, linewidth=2.0)
        axis.set_title(label)
        axis.set_xlabel("Simulation step")

    axes[0].set_ylabel(ylabel)
    figure.suptitle(title, fontsize=15, fontweight="bold", y=1.02)
    figure.savefig(path)
    plt.close(figure)
    print(f"Wrote {path}")


if __name__ == "__main__":
    nominal_path = sys.argv[1] if len(sys.argv) > 1 else "runs_nominal.csv"
    attacked_path = sys.argv[2] if len(sys.argv) > 2 else "runs_attacked.csv"
    output_directory = sys.argv[3] if len(sys.argv) > 3 else "."

    if not os.path.exists(nominal_path):
        print(f"Error: file not found: {nominal_path}")
        sys.exit(1)

    os.makedirs(output_directory, exist_ok=True)
    configure_style()
    data = load_runs(nominal_path, attacked_path)

    plot_probability(
        data,
        "crash",
        "Cumulative crash probability",
        "Crash probability over time",
        os.path.join(output_directory, "crash_probability.png"),
    )
    plot_probability(
        data,
        "overtake_complete",
        "Overtake completion probability",
        "Overtake completion over time",
        os.path.join(output_directory, "overtake_completion.png"),
    )
    plot_distribution(
        data,
        "ego_speed",
        "Ego speed (units per step)",
        "Ego speed: nominal and attacked",
        os.path.join(output_directory, "speed_distribution.png"),
    )
    plot_distribution(
        data,
        "oncoming_ttc",
        "Time to oncoming car (steps)",
        "Oncoming time to collision: nominal and attacked",
        os.path.join(output_directory, "oncoming_ttc.png"),
    )
    plot_distribution(
        data,
        "ego_x",
        "Ego position (units)",
        "Ego position: nominal and attacked",
        os.path.join(output_directory, "position_over_time.png"),
    )
    plot_distribution(
        data,
        "abort_count",
        "Abort count",
        "Overtake aborts: nominal and attacked",
        os.path.join(output_directory, "abort_count.png"),
    )

