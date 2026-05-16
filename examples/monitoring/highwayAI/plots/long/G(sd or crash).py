import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# Optional: use LaTeX-style rendering without requiring a full TeX install
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.family'] = 'serif'

# ==============================
# Configuration
# ==============================
PLOT = "G(sd or crash)"
INPUT_FILE = PLOT+".tsv"
OUTPUT_FILE = PLOT+".png"

# ==============================
# Load TSV data
# ==============================
# decimal=',' handles values like -0,87
# sep='\t' reads tab-separated values

df = pd.read_csv(INPUT_FILE, sep='\t', decimal=',')

# Use first column as x-axis if it looks like a timestep/index column
x_column = df.columns[0]
monitor_columns = df.columns[1:]

# ==============================
# Create figure
# ==============================
plt.figure(figsize=(7, 4))
plt.ylim(-1.05, 0.05)
latex_labels = {
    "G (sg->nocrash)": r"$m_p[\psi_{\mathrm{sd}},0](\mathcal{L}[:t])$",
    "stepwise crash": r"$\text{c}$ stepwise eval.",
    "stepwise 5csd": r"$\text{5csd}$ stepwise eval."
}

for column in monitor_columns[::-1]:
    label = latex_labels.get(column, column)
    plt.plot(df[x_column], df[column], linewidth=2, label=label)

# ==============================
# Styling for conference papers
# ==============================
plt.xlabel(r"Simulation step $t$")
plt.ylabel(r"Evaluation")
plt.grid(True)
plt.legend(frameon=True)
plt.tight_layout()

# ==============================
# Save figure
# ==============================
plt.savefig(OUTPUT_FILE, bbox_inches='tight')

