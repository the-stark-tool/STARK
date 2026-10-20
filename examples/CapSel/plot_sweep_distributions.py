"""Plot CapSel results across all attack angles.

Usage:
    python plot_sweep_distributions.py [output_root] [plot_directory]

Defaults:
    output_root    = output
    plot_directory = output/sweep_plots
"""

from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd


FIGURE_DPI = 150
ANGLE_TICKS = [0.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0]
NOMINAL_COLOR = "#2E7D32"
ATTACK_STYLES = {
    "spoof": ("Spoof", "#D97706"),
    "removal": ("Removal", "#C62828"),
}


def configure_style() -> None:
    """Use one clear style for all sweep plots."""
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.titlesize": 12,
            "axes.titleweight": "bold",
            "axes.labelsize": 11,
            "axes.grid": True,
            "grid.color": "#E0E0E0",
            "axes.spines.top": False,
            "axes.spines.right": False,
            "legend.frameon": False,
            "figure.dpi": FIGURE_DPI,
            "savefig.dpi": FIGURE_DPI,
            "savefig.bbox": "tight",
        }
    )


def first_event_step(frame: pd.DataFrame, column: str) -> pd.Series:
    """Return the first true step for each run."""
    events = frame[frame[column] > 0.5]
    return events.groupby("run")["step"].min()


def summarize_runs(path: Path) -> tuple[pd.DataFrame, float]:
    """Read one run file and return one row for each run."""
    columns = [
        "run",
        "step",
        "ego_y",
        "oncoming_ttc",
        "crash",
        "attack_success",
        "controller_mode",
        "overtake_complete",
        "abort_count",
    ]
    frame = pd.read_csv(path, usecols=columns)
    grouped = frame.groupby("run")
    runs = grouped.agg(
        crash=("crash", "max"),
        abort_count=("abort_count", "max"),
        overtake_complete=("overtake_complete", "max"),
    )
    runs["abort"] = (runs["abort_count"] > 0.5).astype(float)

    move_left = frame.assign(move_left=(frame["controller_mode"] == 1.0).astype(float))
    runs["first_overtake_step"] = first_event_step(move_left, "move_left")
    runs["completion_step"] = first_event_step(frame, "overtake_complete")

    left_lane = frame[frame["ego_y"] >= 5.0]
    runs["minimum_left_lane_ttc"] = left_lane.groupby("run")["oncoming_ttc"].min()

    observed_attack_rate = float(frame["attack_success"].mean())
    return runs.reset_index(), observed_attack_rate


def read_scenario_start(path: Path) -> dict[str, float]:
    """Read the initial positions and speeds for one result folder."""
    columns = [
        "ego_x",
        "ego_y",
        "ego_speed",
        "lead_x",
        "lead_y",
        "lead_speed",
        "oncoming_1_x",
        "oncoming_2_x",
        "oncoming_3_x",
        "oncoming_4_x",
    ]
    first_row = pd.read_csv(path, usecols=columns, nrows=1).iloc[0]
    return {f"initial_{column}": float(first_row[column]) for column in columns}


def load_sweep(output_root: Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Read all spoof and removal result folders."""
    condition_rows = []
    run_frames = []

    for result_directory in sorted(output_root.iterdir()):
        if not result_directory.is_dir():
            continue
        summary_path = result_directory / "attack_summary.csv"
        attacked_path = result_directory / "runs_attacked.csv"
        nominal_path = result_directory / "runs_nominal.csv"
        if not summary_path.exists():
            continue
        if not attacked_path.exists() or not nominal_path.exists():
            print(f"Skip incomplete result folder: {result_directory}")
            continue

        attack_summary = pd.read_csv(summary_path).iloc[0]
        attack = str(attack_summary["attack"]).lower()
        if attack not in ATTACK_STYLES:
            continue
        angle = float(attack_summary["attacker_bearing_degrees"])
        success_probability = float(attack_summary["success_probability"])

        attacked_runs, observed_rate = summarize_runs(attacked_path)
        nominal_runs, _ = summarize_runs(nominal_path)
        scenario_start = read_scenario_start(attacked_path)

        for condition, runs in (("attacked", attacked_runs), ("nominal", nominal_runs)):
            runs["attack"] = attack
            runs["angle_degrees"] = angle
            runs["condition"] = condition
            run_frames.append(runs)

        condition_rows.append(
            {
                "attack": attack,
                "angle_degrees": angle,
                "success_probability": success_probability,
                "observed_attack_rate": observed_rate,
                **scenario_start,
            }
        )

    if not condition_rows:
        raise ValueError(f"No complete sweep folders were found in {output_root}")

    conditions = pd.DataFrame(condition_rows).sort_values(["attack", "angle_degrees"])
    runs = pd.concat(run_frames, ignore_index=True)
    return conditions, runs


def report_scenario_mismatches(conditions: pd.DataFrame) -> bool:
    """Report result folders that do not use the same initial scenario."""
    scenario_columns = [column for column in conditions if column.startswith("initial_")]
    reference = conditions.iloc[0]
    mismatches = []

    for _, row in conditions.iloc[1:].iterrows():
        changed = [
            column
            for column in scenario_columns
            if row[column] != reference[column]
        ]
        if changed:
            changes = ", ".join(
                f"{column.removeprefix('initial_')}={row[column]:g}"
                for column in changed
            )
            mismatches.append(
                f"{row['attack']} {row['angle_degrees']:g} degrees: {changes}"
            )

    if not mismatches:
        return False

    print("Warning: some result folders use a different initial scenario.")
    print(
        "Reference: "
        f"{reference['attack']} {reference['angle_degrees']:g} degrees."
    )
    for mismatch in mismatches:
        print(f"  {mismatch}")
    print("Treat these points as a different test condition.")
    return True


def plot_attack_success(conditions: pd.DataFrame, path: Path) -> None:
    """Plot the model rate and the observed rate for each angle."""
    figure, axis = plt.subplots(figsize=(9, 5.5))
    for attack, (label, color) in ATTACK_STYLES.items():
        values = conditions[conditions["attack"] == attack].sort_values("angle_degrees")
        axis.plot(
            values["angle_degrees"],
            values["success_probability"],
            color=color,
            marker="o",
            linewidth=2.2,
            label=f"{label} model probability",
        )
        axis.plot(
            values["angle_degrees"],
            values["observed_attack_rate"],
            color=color,
            marker="x",
            linestyle="--",
            linewidth=1.7,
            label=f"{label} observed rate",
        )

    axis.set_title("LiDAR attack success by angle")
    axis.set_xlabel("Attack angle from the vehicle front (degrees)")
    axis.set_ylabel("Attack success rate")
    axis.set_xticks(ANGLE_TICKS)
    axis.set_ylim(-0.02, 1.02)
    axis.legend(loc="best")
    figure.savefig(path)
    plt.close(figure)
    print(f"Wrote {path}")


def plot_outcome_rates(runs: pd.DataFrame, path: Path) -> None:
    """Plot the main outcome rates for every attack angle."""
    metrics = [
        ("crash", "Crash probability"),
        ("abort", "Abort probability"),
        ("overtake_complete", "Overtake completion probability"),
    ]
    figure, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True, sharey=True)

    for row_index, (attack, (attack_label, attack_color)) in enumerate(ATTACK_STYLES.items()):
        attack_runs = runs[runs["attack"] == attack]
        for column_index, (metric, metric_label) in enumerate(metrics):
            axis = axes[row_index][column_index]
            for condition, color, line_style, label in (
                ("nominal", NOMINAL_COLOR, "--", "Nominal"),
                ("attacked", attack_color, "-", "Attacked"),
            ):
                values = (
                    attack_runs[attack_runs["condition"] == condition]
                    .groupby("angle_degrees")[metric]
                    .mean()
                    .sort_index()
                )
                axis.plot(
                    values.index,
                    values.values,
                    color=color,
                    linestyle=line_style,
                    marker="o",
                    linewidth=2.0,
                    label=label,
                )
            axis.set_title(f"{attack_label}: {metric_label}")
            axis.set_xticks(ANGLE_TICKS)
            axis.set_ylim(-0.02, 1.02)
            if row_index == 1:
                axis.set_xlabel("Attack angle (degrees)")
            if column_index == 0:
                axis.set_ylabel("Fraction of runs")
            if row_index == 0 and column_index == 0:
                axis.legend(loc="best")

    figure.suptitle("Controller outcomes across the attack-angle sweep", fontsize=15, fontweight="bold")
    figure.tight_layout()
    figure.savefig(path)
    plt.close(figure)
    print(f"Wrote {path}")


def plot_distribution_by_angle(
    runs: pd.DataFrame,
    value: str,
    ylabel: str,
    title: str,
    path: Path,
) -> None:
    """Plot median and percentile bands across the attack angles."""
    figure, axes = plt.subplots(1, 2, figsize=(13, 5.5), sharey=True)

    for axis, (attack, (attack_label, attack_color)) in zip(axes, ATTACK_STYLES.items()):
        attack_runs = runs[runs["attack"] == attack]
        for condition, color, line_style, label in (
            ("nominal", NOMINAL_COLOR, "--", "Nominal"),
            ("attacked", attack_color, "-", "Attacked"),
        ):
            selected = attack_runs[attack_runs["condition"] == condition].dropna(subset=[value])
            grouped = selected.groupby("angle_degrees")[value]
            median = grouped.median().sort_index()
            low = grouped.quantile(0.05).reindex(median.index)
            low_middle = grouped.quantile(0.25).reindex(median.index)
            high_middle = grouped.quantile(0.75).reindex(median.index)
            high = grouped.quantile(0.95).reindex(median.index)
            angles = median.index.to_numpy(dtype=float)

            axis.fill_between(angles, low.to_numpy(), high.to_numpy(), color=color, alpha=0.10)
            axis.fill_between(
                angles,
                low_middle.to_numpy(),
                high_middle.to_numpy(),
                color=color,
                alpha=0.22,
            )
            axis.plot(
                angles,
                median.to_numpy(),
                color=color,
                linestyle=line_style,
                marker="o",
                linewidth=2.0,
                label=label,
            )

        axis.set_title(attack_label)
        axis.set_xlabel("Attack angle (degrees)")
        axis.set_xticks(ANGLE_TICKS)
        axis.legend(loc="best")

    axes[0].set_ylabel(ylabel)
    figure.suptitle(title, fontsize=15, fontweight="bold")
    figure.text(
        0.5,
        0.015,
        "Line: median. Dark band: 25-75%. Light band: 5-95%.",
        ha="center",
        fontsize=9,
        color="#555555",
    )
    figure.tight_layout(rect=(0, 0.05, 1, 0.95))
    figure.savefig(path)
    plt.close(figure)
    print(f"Wrote {path}")


def write_summary(conditions: pd.DataFrame, runs: pd.DataFrame, path: Path) -> None:
    """Write one audit row for each angle and condition."""
    records = []
    for (attack, angle, condition), values in runs.groupby(
        ["attack", "angle_degrees", "condition"], sort=True
    ):
        attack_values = conditions[
            (conditions["attack"] == attack) & (conditions["angle_degrees"] == angle)
        ].iloc[0]
        records.append(
            {
                "attack": attack,
                "angle_degrees": angle,
                "condition": condition,
                "runs": len(values),
                "success_probability": attack_values["success_probability"],
                "observed_attack_rate": (
                    attack_values["observed_attack_rate"] if condition == "attacked" else ""
                ),
                "crash_rate": values["crash"].mean(),
                "abort_rate": values["abort"].mean(),
                "overtake_completion_rate": values["overtake_complete"].mean(),
                "median_first_overtake_step": values["first_overtake_step"].median(),
                "median_completion_step": values["completion_step"].median(),
                "median_minimum_left_lane_ttc": values["minimum_left_lane_ttc"].median(),
                **{
                    column: attack_values[column]
                    for column in conditions
                    if column.startswith("initial_")
                },
            }
        )
    pd.DataFrame(records).to_csv(path, index=False)
    print(f"Wrote {path}")


def main() -> None:
    """Load the sweep and write all angle-level plots."""
    script_directory = Path(__file__).resolve().parent
    output_root = Path(sys.argv[1]) if len(sys.argv) > 1 else script_directory / "output"
    plot_directory = (
        Path(sys.argv[2]) if len(sys.argv) > 2 else output_root / "sweep_plots"
    )

    if not output_root.exists():
        print(f"Error: folder not found: {output_root}")
        sys.exit(1)

    plot_directory.mkdir(parents=True, exist_ok=True)
    configure_style()
    conditions, runs = load_sweep(output_root)
    report_scenario_mismatches(conditions)
    print(
        f"Loaded {len(conditions)} attack-angle conditions and "
        f"{len(runs):,} run summaries."
    )

    plot_attack_success(conditions, plot_directory / "attack_success_by_angle.png")
    plot_outcome_rates(runs, plot_directory / "outcome_rates_by_angle.png")
    plot_distribution_by_angle(
        runs,
        "first_overtake_step",
        "First overtake step",
        "First overtake step across the attack-angle sweep",
        plot_directory / "first_overtake_step_by_angle.png",
    )
    plot_distribution_by_angle(
        runs,
        "completion_step",
        "Overtake completion step",
        "Overtake completion step across the attack-angle sweep",
        plot_directory / "completion_step_by_angle.png",
    )
    plot_distribution_by_angle(
        runs,
        "minimum_left_lane_ttc",
        "Minimum oncoming TTC (steps)",
        "Minimum left-lane TTC across the attack-angle sweep",
        plot_directory / "minimum_left_lane_ttc_by_angle.png",
    )
    write_summary(conditions, runs, plot_directory / "sweep_summary.csv")


if __name__ == "__main__":
    main()
