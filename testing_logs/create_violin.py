import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as ticker
import math

# === Load and parse the CSV ===
file_path = '/Users/matasjones/Desktop/PDS_II/holo_tests/june_19/UDP_HOLO/HOLO_UDP_socket_5ms.csv'
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
    loss_pct = (50 - retained_count) / 50 * 100  # Assuming 50 packets expected
    retention_info[msg_size] = f"{100 - loss_pct:.1f}%"  # Percentage retained

# === Sort and prepare for plotting ===
filtered_grouped = pd.Series(filtered_groups).sort_index()
legend_labels = [
    f"$10^{{{int(math.log10(ms))}}}$: {100 - float(retention_info[ms].strip('%')):.1f}%"
    for ms in filtered_grouped.index
]

# === Plot configuration ===
fig_height = 1.5 + 0.6 * len(filtered_grouped)
fig, ax = plt.subplots(figsize=(10, fig_height))
parts = ax.violinplot(filtered_grouped.values, vert=False, showmeans=True)

# Apply plasma color scheme
colors = plt.cm.plasma(np.linspace(0, 1, len(parts['bodies'])))
for body, color in zip(parts['bodies'], colors):
    body.set_facecolor(color)
    body.set_edgecolor('black')
    body.set_alpha(0.8)

# Set all candle components (mean, whiskers) to black
for partname in ['cmeans', 'cbars', 'cmins', 'cmaxes']:
    if partname in parts:
        vp = parts[partname]
        if isinstance(vp, list):
            for line in vp:
                line.set_color('black')
        else:
            vp.set_color('black')

# === Add average text annotations above each violin ===
for i, (msg_size, values) in enumerate(filtered_grouped.items(), start=1):
    avg = np.mean(values)
    ax.text(avg, i + 0.25, f"{avg:.2f} ms", ha='center', va='bottom', fontsize=9, color='black')

# Y-axis with powers of 10
y_positions = np.arange(1, len(filtered_grouped.index) + 1)
power_labels = [f"$10^{{{int(math.log10(ms))}}}$" for ms in filtered_grouped.index]
ax.set_yticks(y_positions)
ax.set_yticklabels(power_labels)
ax.set_ylim(0.5, len(filtered_grouped.index) + 0.8)

# X-axis grid and ticks
locator = ticker.MaxNLocator(nbins=20)
ax.xaxis.set_major_locator(locator)
ax.minorticks_on()
ax.xaxis.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)

# Labels and title
ax.set_xlabel('Latency [ms]')
ax.set_ylabel('Message Size [bytes]')
ax.set_title('Latency Distribution | UDP SOCKET | HOLO-HOLO | 5ms interval')

# Legend with packet loss percentages
ax.legend(legend_labels, title="Packets Lost", loc='upper right', fontsize=9, title_fontsize=10)

plt.tight_layout()
plt.show()
