import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as ticker
from matplotlib.lines import Line2D

# === Load the CSV ===
df = pd.read_csv('/Users/matasjones/Desktop/PDSII_plots/frequency_data.csv')  # Update path if needed

# === Prepare Data ===
period_labels = df.columns.tolist()
periods_ms = [int(p.strip().replace('ms', '')) for p in period_labels]
frequencies_hz = [1000 // p for p in periods_ms]

filtered_grouped = pd.Series({f"{f} Hz": df[col].dropna().tolist() for f, col in zip(frequencies_hz, df.columns)})
positions = np.arange(1, len(filtered_grouped) + 1)

# === Plot Setup ===
fig_height = 1.5 + 0.6 * len(filtered_grouped)
fig, ax = plt.subplots(figsize=(10, fig_height))

# Use normal (non-reversed) colormap
colors = plt.cm.plasma(np.linspace(0, 1, len(filtered_grouped)))

# === Draw violins ===
parts = ax.violinplot(
    filtered_grouped.values,
    positions=positions,
    vert=False,
    showmeans=False,
    showmedians=False,
    showextrema=False
)

for body, color in zip(parts['bodies'], colors):
    body.set_facecolor(color)
    body.set_edgecolor('black')
    body.set_alpha(0.8)

# === Overlay box plots ===
box_half_height = 0.05
for i, (values, color) in enumerate(zip(filtered_grouped.values, colors), start=1):
    data = np.array(values)
    q1 = np.percentile(data, 25)
    q3 = np.percentile(data, 75)
    median = np.median(data)
    whisker_low = np.min(data)
    whisker_high = np.max(data)

    ax.fill_betweenx([i - box_half_height, i + box_half_height], q1, q3, color='white', zorder=3)
    ax.plot([q1, q3], [i, i], color='black', linewidth=1.5, zorder=4)

    ax.plot([whisker_low, q1], [i, i], color='black', linewidth=1, zorder=3)
    ax.plot([q3, whisker_high], [i, i], color='black', linewidth=1, zorder=3)

    ax.plot([whisker_low, whisker_low], [i - 0.07, i + 0.07], color='black', linewidth=1.5, zorder=5)
    ax.plot([whisker_high, whisker_high], [i - 0.07, i + 0.07], color='black', linewidth=1.5, zorder=5)

    ax.plot([median, median], [i - box_half_height, i + box_half_height], color='black', linewidth=2.5, zorder=6)
    ax.text(median, i + 0.25, f"{median:.2f} ms", ha='center', va='bottom', fontsize=9, color='black')

# === Axis Config (now showing frequency) ===
ax.set_yticks(positions)
ax.set_yticklabels([f"{f} Hz" for f in frequencies_hz])
ax.set_ylim(0.5, len(filtered_grouped) + 0.8)

ax.set_xlim(1.2, df.max().max() * 1.1)
locator = ticker.MaxNLocator(nbins=20)
ax.xaxis.set_major_locator(locator)
ax.minorticks_on()
ax.xaxis.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)

ax.set_xlabel("Latency [ms]")
ax.set_ylabel("Frequency [Hz]")
ax.set_title("Latency Distribution | Same package size: 100 Bytes | Socket UDP | MAIN-HOLO")

# === Save / Show ===
plt.tight_layout()
output_path = '/Users/matasjones/Desktop/PDSII_plots/f_plot.pdf'
plt.savefig(output_path, format='pdf', bbox_inches='tight')
plt.show()
