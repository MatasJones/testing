import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as ticker
import math
from matplotlib.lines import Line2D

# === Load and parse the CSV ===
file_path = '/Users/matasjones/Desktop/PDS_II/holo_tests/june_19/ROS_DEFAULT_NO_LOGS/holo_holo_no_logs_5ms.csv'
df = pd.read_csv(file_path, header=None, skiprows=1)
df[0] = df[0].astype(str)
split_df = df[0].str.split('|', expand=True).apply(lambda x: x.str.strip())
split_df.columns = ['MessageSize', 'MsgID', 'Latency_ms']
split_df = split_df.astype({'MessageSize': int, 'MsgID': int, 'Latency_ms': float})

# === Grouping, filtering, and loss calculation ===
filtered_groups = {}
retention_info = {}

for msg_size, group in split_df.groupby('MessageSize'):
    avg_latency = group['Latency_ms'].mean()
    filtered = group[group['Latency_ms'] <= avg_latency + 10]
    filtered_groups[msg_size] = filtered['Latency_ms'].tolist()
    retained_count = len(filtered)
    retention_pct = (retained_count / 50) * 100  # Assuming 50 packets expected
    retention_info[msg_size] = f"{retention_pct:.1f}%"  # Retention

# === Sort and prepare for plotting ===
filtered_grouped = pd.Series(filtered_groups).sort_index()
positions = np.arange(1, len(filtered_grouped) + 1)

# === Plot configuration ===
fig_height = 1.5 + 0.6 * len(filtered_grouped)
fig, ax = plt.subplots(figsize=(10, fig_height))

# Reversed plasma colormap for top-to-bottom alignment
colors = plt.cm.plasma(np.linspace(0, 1, len(filtered_grouped)))[::-1]

# === Draw violin plots (no medians, no extrema) ===
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

# === Overlay box plot, whiskers, end caps, and vertical median ===
box_half_height = 0.05  # Smaller box height

for i, (values, color) in enumerate(zip(filtered_grouped.values, colors), start=1):
    data = np.array(values)
    q1 = np.percentile(data, 25)
    q3 = np.percentile(data, 75)
    median = np.median(data)

    whisker_low = np.min(data)
    whisker_high = np.max(data)

    # IQR box
    ax.fill_betweenx([i - box_half_height, i + box_half_height], q1, q3, color='white', zorder=3)
    ax.plot([q1, q3], [i, i], color='black', linewidth=1.5, zorder=4)

    # Whiskers
    ax.plot([whisker_low, q1], [i, i], color='black', linewidth=1, zorder=3)
    ax.plot([q3, whisker_high], [i, i], color='black', linewidth=1, zorder=3)

    # Whisker end caps
    ax.plot([whisker_low, whisker_low], [i - 0.07, i + 0.07], color='black', linewidth=1.5, zorder=5)
    ax.plot([whisker_high, whisker_high], [i - 0.07, i + 0.07], color='black', linewidth=1.5, zorder=5)

    # Vertical median line (within the IQR box)
    ax.plot([median, median], [i - box_half_height, i + box_half_height], color='black', linewidth=1.5, zorder=6)

    # Median annotation
    ax.text(median, i + 0.25, f"{median:.2f} ms", ha='center', va='bottom', fontsize=9, color='black')

# === Axes and labels ===
power_labels = [f"$10^{{{int(math.log10(ms))}}}$" for ms in filtered_grouped.index]
ax.set_yticks(positions)
ax.set_yticklabels(power_labels)
ax.set_ylim(0.5, len(filtered_grouped) + 0.8)

ax.set_xlim(1, 20)
locator = ticker.MaxNLocator(nbins=20)
ax.xaxis.set_major_locator(locator)
ax.minorticks_on()
ax.xaxis.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)

ax.set_xlabel('Latency [ms]')
ax.set_ylabel('Message Size [bytes]')
ax.set_title('Latency Distribution | ROS2, default QOS, no logs | HOLO-HOLO | 5 ms interval')

# # === Custom legend with packet LOSS ===
# legend_labels = [
#     f"$10^{{{int(math.log10(ms))}}}$: {100 - float(retention_info[ms].strip('%')):.1f}%"
#     for ms in filtered_grouped.index
# ]
# legend_lines = [
#     Line2D([0], [0], color=color, lw=4)
#     for color in colors
# ]
# ax.legend(legend_lines, legend_labels, title="Packets Lost", loc='upper right', fontsize=9, title_fontsize=10)

# === Save and show ===
plt.tight_layout()
output_path = '/Users/matasjones/Desktop/PDSII_plots/size_plot.pdf'
plt.savefig(output_path, format='pdf', bbox_inches='tight')
plt.show()
