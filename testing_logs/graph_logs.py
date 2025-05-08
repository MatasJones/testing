import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys

csv_file_path = sys.argv[1]
plot_name =sys.argv[2]

print(plot_name)

def plot_data_from_csv(csv_file_path):

    with open(csv_file_path, 'r') as f:
        first_line = f.readline().strip()

    # Read the CSV into a DataFrame
    df = pd.read_csv(csv_file_path, delimiter='|', header=None, skiprows=1)  # no header row

    # Add column names
    df.columns = ['msg_size', 'msg_nb', 'time']
    
    # Convert data types
    df['msg_size'] = pd.to_numeric(df['msg_size'], errors='coerce')
    df['time'] = pd.to_numeric(df['time'], errors='coerce')

    # Calculate log10 of msg_size (floor to get the power of 10)
    df['log10_msg_size'] = np.floor(np.log10(df['msg_size'])).astype(int)

    # Get unique powers of 10 for coloring
    unique_powers = sorted(df['log10_msg_size'].unique())
    print(f"Unique powers of 10 in msg_size: {unique_powers}")
    
    # Create a colormap for both scatter plot and histogram
    colormap = plt.get_cmap('rainbow')
    color_dict = {power: colormap(i / len(unique_powers)) for i, power in enumerate(unique_powers)}
    
    # Compute initial average value for each power of 10
    avg_values = df.groupby('log10_msg_size')['time'].mean()

    # Exclude values that are more than 20% above the average
    filtered_df = df[~df.apply(lambda row: row['time'] > 1.2 * avg_values[row['log10_msg_size']], axis=1)]

    # Recalculate the average after filtering
    filtered_avg_values = filtered_df.groupby('log10_msg_size')['time'].mean()

    # Print the filtered averages
    print("\nFiltered Average Value for Each Power of 10 in msg_size (Excluding Outliers):")
    for power, avg in filtered_avg_values.items():
        print(f"10^{power}: {avg:.4f}")

    # Create a figure with 2 subplots (scatter plot and histogram)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    ax1.set_xscale('log')
    # Scatter Plot (ax1)
    for power in unique_powers:
        subset = df[df['log10_msg_size'] == power]
        ax1.scatter(subset['msg_size'], subset['time'], color=color_dict[power], alpha=0.6, label=f"10^{power}")

    ax1.set_xlabel('Message size [bytes]')
    ax1.set_ylabel('Roundtrip time [ms]')
    ax1.set_title(f"Latency test holo to holo, {first_line} spacing, {plot_name}")
    #ax1.set_title(f"Latency test main to holohover, 100µs spacing, {plot_name}")
    ax1.legend(title="Message size [bytes]", title_fontsize='large', fontsize='small', loc='upper left')
    ax1.grid(True, linestyle='--', alpha=0.6)

    # Histogram (ax2)
    bars = ax2.bar(filtered_avg_values.index, filtered_avg_values.values, 
                   color=[color_dict[p] for p in filtered_avg_values.index], alpha=0.6)

    ax2.set_xlabel("Message size [bytes]")
    ax2.set_ylabel("Average roundtrip time [ms]")
    ax2.set_title("Average Roundtrip Time vs Message Size")
    ax2.set_xticks(filtered_avg_values.index)
    ax2.set_xticklabels([f"10^{int(x)}" for x in filtered_avg_values.index])

    # Add the values above each column of the histogram
    for bar in bars:
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width() / 2, height, f'{height:.4f}', ha='center', va='bottom', fontsize=10)

    count_lost_packets(csv_file_path, 50)
    
    # Capture printed output from count_lost_packets
    lost_summary_text = count_lost_packets(csv_file_path, 30)
    # Add the summary as a textbox at the bottom of the figure
    fig.text(0.3, 0.75, lost_summary_text, ha='center', va='bottom',
         fontsize=9, wrap=True, family='monospace',
         bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))


    # Show the figure with both plots and summary
    plt.tight_layout()
    plt.show()

def count_lost_packets(data_path, expected_messages, initial_size = 1):

    df = pd.read_csv(data_path, sep='|', header=None, engine='python', skiprows=1)
    df.columns = ['raw_size', 'message_number', 'time']

    # Clean and convert
    df_obj = df.select_dtypes(['object'])
    df[df_obj.columns] = df_obj.apply(lambda x: x.str.strip())

    df['message_number'] = df['message_number'].astype(int)

    # Infer actual size based on message_number
    df['package_size'] = df['message_number'].apply(lambda x: initial_size * (10 ** (x // 50)))

    summary_lines = []

    # Group by inferred size
    for size, group in df.groupby('package_size'):
        # Calculate message ID range for this size group
        size_index = group['message_number'].min() // 50
        start_id = size_index * 50
        expected_ids = set(range(start_id, start_id + 50))
        received_ids = set(group['message_number'])
        missing = expected_ids - received_ids
        loss_percent = (len(missing) / 50) * 100
        summary_lines.append(f"Size {size}: Lost {loss_percent:.1f}% packets")

    return "\n".join(summary_lines)



# Path to your CSV file
plot_data_from_csv(csv_file_path)

