import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as ticker
from matplotlib.lines import Line2D

# === Load and clean CSV ===
df = pd.read_csv('/Users/matasjones/Desktop/PDSII_plots/main_5ms_100B.csv', header=None, encoding='ISO-8859-1')
cleaned_df = df.applymap(lambda x: str(x).split('|')[-1].strip())

# === Extract labels and data ===
labels = cleaned_df.iloc[0].tolist()
data = cleaned_df.iloc[1:].apply(pd.to_numeric, errors='coerce')  # Convert to float with NaN fallback

# === Drop columns that are entirely or mostly invalid ===
valid_data = data.dropna(axis=1, how='all')
valid_data = valid_data.loc[:, valid_data.count() >= 2]

# === Re-align labels ===
valid_indices = valid_data.columns.tolist()
valid_labels = [labels[i] for i in valid_indices]
values = [valid_data[col].dropna().values for col in valid_data.columns]

# === Reverse for top-down display ===
values = values[::-1]
valid_labels = valid_labels[::-1]

# === Manually set y-axis labels (short labels for plot) ===
custom_labels = [
    "Socket UDP",
    "Socket TCP",
    "ROS2 #3",
    "ROS2 #2",
    "ROS2 #1",
][:len(valid_labels)]  # Trim if fewer violins than labels provided

# === Legend mapping: short label -> full description ===
ros_legend_map = {
    "ROS2 #1": "Default QOS, with logs",
    "ROS2 #2": "Default QOS, no logs",
    "ROS2 #3": "Best effort QOS, no logs",
}

# === Plot Setup ===
positions = np.arange(1, len(valid_labels) + 1)
fig_height = 1.5 + 0.6 * len(valid_labels)
fig, ax = plt.subplots(figsize=(10, fig_height))
colors = plt.cm.plasma(np.linspace(0, 1, len(valid_labels)))

parts = ax.violinplot(
    values,
    positions=positions,
    vert=False,
    showmeans=False,
    showmedians=False,
    showextrema=False
)

# === Style violins ===
for body, color in zip(parts['bodies'], colors):
    body.set_facecolor(color)
    body.set_edgecolor('black')
    body.set_alpha(0.8)

# === Overlay box plots ===
box_half_height = 0.05
for i, vals in enumerate(values, start=1):
    q1 = np.percentile(vals, 25)
    q3 = np.percentile(vals, 75)
    median = np.median(vals)
    whisker_low = np.min(vals)
    whisker_high = np.max(vals)

    ax.fill_betweenx([i - box_half_height, i + box_half_height], q1, q3, color='white', zorder=3)
    ax.plot([q1, q3], [i, i], color='black', linewidth=1.5, zorder=4)
    ax.plot([whisker_low, q1], [i, i], color='black', linewidth=1, zorder=3)
    ax.plot([q3, whisker_high], [i, i], color='black', linewidth=1, zorder=3)
    ax.plot([whisker_low, whisker_low], [i - 0.07, i + 0.07], color='black', linewidth=1.5, zorder=5)
    ax.plot([whisker_high, whisker_high], [i - 0.07, i + 0.07], color='black', linewidth=1.5, zorder=5)
    ax.plot([median, median], [i - box_half_height, i + box_half_height], color='black', linewidth=2.5, zorder=6)
    ax.text(median, i + 0.25, f"{median:.2f} ms", ha='center', va='bottom', fontsize=9, color='black')

# === Axes and labels ===
ax.set_yticks(positions)
ax.set_yticklabels(custom_labels)
ax.set_ylim(0.5, len(valid_labels) + 0.8)
ax.set_xlim(1.2, 11)

ax.set_xlabel("Latency [ms]")
ax.set_ylabel("Implementation")
ax.set_title("Latency Distribution Across Implementations")

ax.xaxis.set_major_locator(ticker.MaxNLocator(nbins=20))
ax.minorticks_on()
ax.xaxis.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)

# === Custom Legend for ROS2 Implementations ===
legend_lines = []
legend_names = []

for short_label, full_desc in ros_legend_map.items():
    if short_label in custom_labels:
        idx = custom_labels.index(short_label)
        color = colors[idx]
        legend_lines.append(Line2D([0], [0], color=color, lw=4))
        legend_names.append(f"{short_label}: {full_desc}")

if legend_lines:
    ax.legend(legend_lines, legend_names, title="ROS2 Variants", loc='lower right', fontsize=9, title_fontsize=10)

# === Save and show ===
plt.tight_layout()
output_path = '/Users/matasjones/Desktop/PDSII_plots/plot_main_5ms_100B.pdf'
plt.savefig(output_path, format='pdf', bbox_inches='tight')
plt.show()
