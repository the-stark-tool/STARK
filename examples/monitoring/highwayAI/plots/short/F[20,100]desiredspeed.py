import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# Optional: use LaTeX-style rendering without requiring a full TeX install
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.family'] = 'serif'

# ==============================
# Configuration
# ==============================
PLOT = "F[20,100]desiredspeed"
INPUT_FILE = PLOT + ".tsv"

OUTPUT_MON = PLOT + "_monitor.png"
OUTPUT_V0 = PLOT + "_v0.png"

# ==============================
# Load TSV data
# ==============================
df = pd.read_csv(INPUT_FILE, sep='\t', decimal=',', na_values=['u'])

x_column = df.columns[0]

latex_labels = {
    "mon": r"$m_p[\varphi_{s},0](\mathcal{S}[:t])$",
    "v0": r"$\mathtt{v_x}$ of ego at $t$ (avg.)",
}

# ==============================
# Shared figure dimensions
# Original: (7,4)
# Two stacked plots -> each gets half height
# ==============================
FIG_WIDTH = 7
FIG_HEIGHT = 2

# =========================================================
# Plot 1: Evaluation
# =========================================================
plt.figure(figsize=(FIG_WIDTH, FIG_HEIGHT))

plt.plot(
    df[x_column],
    df["mon"],
    linewidth=2,
    label=latex_labels["mon"]
)
plt.ylim(-0.25, -0.13)
plt.xlim(0, 160)

plt.xlabel(r"Simulation step $t$")
plt.ylabel(r"Eval.")
plt.grid(True)

plt.legend(
    frameon=True,
    loc='lower right'
)

plt.tight_layout()

plt.savefig(OUTPUT_MON, bbox_inches='tight')
plt.close()

# =========================================================
# Plot 2: Ego speed
# =========================================================
plt.figure(figsize=(FIG_WIDTH, FIG_HEIGHT))
plt.ylim(0.2, 0.9)
plt.xlim(0, 160)

# Speed series with custom color
plt.plot(
    df[x_column],
    df["v0"],
    linewidth=2,
    color='orange',
    label=latex_labels["v0"]
)

# Threshold lines
D = 0.65625
L = 0.8125

plt.axhline(
    y=D,
    linestyle='--',
    linewidth=1.5,
    color='darkgreen',
    label=r"$D_\mathtt{v_x}$"
)

plt.axhline(
    y=L,
    linestyle='--',
    linewidth=1.5,
    color='darkred',
    label=r"$L_\mathtt{v_x}$"
)

plt.xlabel(r"Simulation step $t$")
plt.ylabel(r"m/s (norm.)")
plt.grid(True)

plt.legend(
    frameon=True,
    loc='lower right',
    ncol=3
)

plt.tight_layout()

plt.savefig(OUTPUT_V0, bbox_inches='tight')
plt.close()