"""
plot_distributions.py

Produces four publication-quality plots from the per-run CSVs written by
ms\Main.java (runs_nominal.csv and runs_attacked.csv).

These plots visualise the distributions that STARK's RobTL formulas are
comparing under the hood:

    TEff_distribution.png - distribution over time of active effectory T cells
    TReg_distribution.png - distribution over time of active regulatory T cells
    ratio_distribution.png - distribution over time of TEff/TReg
    rev_damage_distribution.png - distribution over time of reversible damage
    irrev_damage_distribution.png - distribution over time of irreversible damage
                                

Requires:
    pip install pandas matplotlib

Usage:
    python plot_distributions.py [healthy_csv] [sick_csv] [output_dir]

Defaults:
    healthy_csv  = runs_ms_healthy.csv
    sick_csv = runs_ms_sick.csv
    sickuc_csv = runs_ms_sickuc.csv
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
def load_runs(healthy_path: str, sick_path: str, sickuc_path: str) -> pd.DataFrame:
    """Load and concatenate the two run CSVs into one tidy DataFrame."""
    healthy = pd.read_csv(healthy_path)
    if os.path.exists(sick_path):
        sick = pd.read_csv(sick_path)
        sickuc = pd.read_csv(sickuc_path)
        df = pd.concat([healthy, sick, sickuc], ignore_index=True)
    else:
        print(f"Note: {sick_path} not found - plotting healthy only.")
        df = healthy
    return df


# ----- Plot distribution bands --------------------------------------

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
        color = NOMINAL_COLOR if condition == "healthy" else ATTACKED_COLOR
        label = "Healthy" if condition == "healthy" else f"Sick({condition})"

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
        ax.set_yscale('log')
        ax.set_ylim(y_lim)
        ax.legend(loc="best")

    axes[0].set_ylabel(ylabel)
    fig.suptitle(title, fontsize=15, fontweight="bold", y=1.02)
    fig.savefig(output_path)
    plt.show(fig)
    print(f"Wrote {output_path}")


# ----- Entry point ---------------------------------------------------------
if __name__ == "__main__":
    healthy_path = sys.argv[1] if len(sys.argv) > 1 else "runs_ms_healthy.csv"
    
    sick7 = sys.argv[2] if len(sys.argv) > 2 else "runs_ms_s7.csv"
    sick8 = sys.argv[2] if len(sys.argv) > 2 else "runs_ms_s8.csv"
    sick9 = sys.argv[2] if len(sys.argv) > 2 else "runs_ms_s9.csv"
    sick10 = sys.argv[2] if len(sys.argv) > 2 else "runs_ms_s10.csv"
    sick11 = sys.argv[2] if len(sys.argv) > 2 else "runs_ms_s11.csv"
    sick12 = sys.argv[2] if len(sys.argv) > 2 else "runs_ms_s12.csv"
    
    sickuc_path = sys.argv[3] if len(sys.argv) > 3 else "runs_ms_sickuc.csv"
    output_dir = sys.argv[4] if len(sys.argv) > 4 else "."
    
    

    if not os.path.exists(healthy_path):
        print(f"Error: file not found: {healthy_path}")
        sys.exit(1)

    os.makedirs(output_dir, exist_ok=True)
    configure_style()

    df7 = load_runs(healthy_path, sick7, sickuc_path)
    print(f"Loaded {len(df7):,} rows ({df7['run'].nunique()} runs per condition, "
          f"{df7['step'].nunique()} steps)")


    plot_distribution_bands(df7, "E", "Active TEff cells",
                             "Active TEff cells distribution: healthy vs sick",
                             os.path.join(output_dir, "TEff_distrib7_aR24_cg7.png"))
    plot_distribution_bands(df7, "R", "Active TReg cells",
                             "Active TReg cells distribution: healthy vs sick",
                             os.path.join(output_dir, "TReg_distrib7_aR24_cg7.png"))
    plot_distribution_bands(df7, "ratio", "TEff-TReg ratio",
                             "TEff-TReg cells ratio distribution: healthy vs sick",
                             os.path.join(output_dir, "ratio_distrib7_aR24_cg7.png"))
    plot_distribution_bands(df7, "l", "Reversible damage",
                             "Reversible damage distribution: healthy vs sick",
                             os.path.join(output_dir, "rev_damage_distrib74_aR2_cg7.png"))
    plot_distribution_bands(df7, "L", "Irreversible damage",
                             "Irreverisible damage distribution: healthy vs sick",
                             os.path.join(output_dir, "irrev_damage_distrib74_aR2_cg7.png"))
    
    df8 = load_runs(healthy_path, sick8, sickuc_path)
    print(f"Loaded {len(df8):,} rows ({df8['run'].nunique()} runs per condition, "
          f"{df8['step'].nunique()} steps)")


    plot_distribution_bands(df8, "E", "Active TEff cells",
                             "Active TEff cells distribution: healthy vs sick",
                             os.path.join(output_dir, "TEff_distrib8_aR24_cg7.png"))
    plot_distribution_bands(df8, "R", "Active TReg cells",
                             "Active TReg cells distribution: healthy vs sick",
                             os.path.join(output_dir, "TReg_distrib8_aR24_cg7.png"))
    plot_distribution_bands(df8, "ratio", "TEff-TReg ratio",
                             "TEff-TReg cells ratio distribution: healthy vs sick",
                             os.path.join(output_dir, "ratio_distrib8_aR24_cg7.png"))
    plot_distribution_bands(df8, "l", "Reversible damage",
                             "Reversible damage distribution: healthy vs sick",
                             os.path.join(output_dir, "rev_damage_distrib8_aR24_cg7.png"))
    plot_distribution_bands(df8, "L", "Irreversible damage",
                             "Irreverisible damage distribution: healthy vs sick",
                             os.path.join(output_dir, "irrev_damage_distrib8_aR24_cg7.png"))
    
    df9 = load_runs(healthy_path, sick9, sickuc_path)
    print(f"Loaded {len(df9):,} rows ({df9['run'].nunique()} runs per condition, "
          f"{df9['step'].nunique()} steps)")


    plot_distribution_bands(df9, "E", "Active TEff cells",
                             "Active TEff cells distribution: healthy vs sick",
                             os.path.join(output_dir, "TEff_distrib9_aR24_cg7.png"))
    plot_distribution_bands(df9, "R", "Active TReg cells",
                             "Active TReg cells distribution: healthy vs sick",
                             os.path.join(output_dir, "TReg_distrib9_aR24_cg7.png"))
    plot_distribution_bands(df9, "ratio", "TEff-TReg ratio",
                             "TEff-TReg cells ratio distribution: healthy vs sick",
                             os.path.join(output_dir, "ratio_distrib9_aR24_cg7.png"))
    plot_distribution_bands(df9, "l", "Reversible damage",
                             "Reversible damage distribution: healthy vs sick",
                             os.path.join(output_dir, "rev_damage_distrib9_aR24_cg7.png"))
    plot_distribution_bands(df9, "L", "Irreversible damage",
                             "Irreverisible damage distribution: healthy vs sick",
                             os.path.join(output_dir, "irrev_damage_distrib9_aR24_cg7.png"))
    
    df10 = load_runs(healthy_path, sick10, sickuc_path)
    print(f"Loaded {len(df7):,} rows ({df10['run'].nunique()} runs per condition, "
          f"{df10['step'].nunique()} steps)")


    plot_distribution_bands(df10, "E", "Active TEff cells",
                             "Active TEff cells distribution: healthy vs sick",
                             os.path.join(output_dir, "TEff_distrib10_aR24_cg7.png"))
    plot_distribution_bands(df10, "R", "Active TReg cells",
                             "Active TReg cells distribution: healthy vs sick",
                             os.path.join(output_dir, "TReg_distrib10_aR24_cg7.png"))
    plot_distribution_bands(df10, "ratio", "TEff-TReg ratio",
                             "TEff-TReg cells ratio distribution: healthy vs sick",
                             os.path.join(output_dir, "ratio_distrib10_aR24_cg7.png"))
    plot_distribution_bands(df10, "l", "Reversible damage",
                             "Reversible damage distribution: healthy vs sick",
                             os.path.join(output_dir, "rev_damage_distrib10_aR24_cg7.png"))
    plot_distribution_bands(df10, "L", "Irreversible damage",
                             "Irreverisible damage distribution: healthy vs sick",
                             os.path.join(output_dir, "irrev_damage_distrib10_aR24_cg7.png"))
    
    df11 = load_runs(healthy_path, sick11, sickuc_path)
    print(f"Loaded {len(df11):,} rows ({df11['run'].nunique()} runs per condition, "
          f"{df11['step'].nunique()} steps)")


    plot_distribution_bands(df11, "E", "Active TEff cells",
                             "Active TEff cells distribution: healthy vs sick",
                             os.path.join(output_dir, "TEff_distrib11_aR24_cg7.png"))
    plot_distribution_bands(df11, "R", "Active TReg cells",
                             "Active TReg cells distribution: healthy vs sick",
                             os.path.join(output_dir, "TReg_distrib11_aR24_cg7.png"))
    plot_distribution_bands(df11, "ratio", "TEff-TReg ratio",
                             "TEff-TReg cells ratio distribution: healthy vs sick",
                             os.path.join(output_dir, "ratio_distrib11_aR24_cg7.png"))
    plot_distribution_bands(df11, "l", "Reversible damage",
                             "Reversible damage distribution: healthy vs sick",
                             os.path.join(output_dir, "rev_damage_distrib11_aR24_cg7.png"))
    plot_distribution_bands(df11, "L", "Irreversible damage",
                             "Irreverisible damage distribution: healthy vs sick",
                             os.path.join(output_dir, "irrev_damage_distrib11_aR24_cg7.png"))
    
    df12 = load_runs(healthy_path, sick12, sickuc_path)
    print(f"Loaded {len(df12):,} rows ({df12['run'].nunique()} runs per condition, "
          f"{df12['step'].nunique()} steps)")


    plot_distribution_bands(df12, "E", "Active TEff cells",
                             "Active TEff cells distribution: healthy vs sick",
                             os.path.join(output_dir, "TEff_distrib12_aR24_cg7.png"))
    plot_distribution_bands(df12, "R", "Active TReg cells",
                             "Active TReg cells distribution: healthy vs sick",
                             os.path.join(output_dir, "TReg_distrib12_aR24_cg7.png"))
    plot_distribution_bands(df12, "ratio", "TEff-TReg ratio",
                             "TEff-TReg cells ratio distribution: healthy vs sick",
                             os.path.join(output_dir, "ratio_distrib12_aR24_cg7.png"))
    plot_distribution_bands(df12, "l", "Reversible damage",
                             "Reversible damage distribution: healthy vs sick",
                             os.path.join(output_dir, "rev_damage_distrib12_aR24_cg7.png"))
    plot_distribution_bands(df12, "L", "Irreversible damage",
                             "Irreverisible damage distribution: healthy vs sick",
                             os.path.join(output_dir, "irrev_damage_distrib12_aR24_cg7.png"))
    