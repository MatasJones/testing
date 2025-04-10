import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def plot_data_from_csv(csv_file_path):
    # Read the CSV into a DataFrame
    df = pd.read_csv(csv_file_path, delimiter='|', header=None)  # no header row
    
    # Add column names
    df.columns = ['msg_size', 'msg_nb', 'time']
    
    # Convert data types
    df['msg_size'] = pd.to_numeric(df['msg_size'], errors='coerce')
    df['time'] = pd.to_numeric(df['time'], errors='coerce')

    # Calculate log10 of msg_size (floor to get the power of 10)
    df['log10_msg_size'] = np.floor(np.log10(df['msg_size'])).astype(int)

    # Get unique powers of 10 for coloring
    unique_powers = sorted(df['log10_msg_size'].unique())
    
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

    # Scatter Plot (ax1)
    for power in unique_powers:
        subset = df[df['log10_msg_size'] == power]
        ax1.scatter(subset['msg_size'], subset['time'], color=color_dict[power], alpha=0.6, label=f"10^{power}")

    ax1.set_xlabel('Message size [bytes]')
    ax1.set_ylabel('Roundtrip time [ms]')
    ax1.set_title("Latency test 2 holos, 1ms spacing")
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

    count_lost_packets(csv_file_path, 30)
    
    # Show the figure with both plots
    plt.tight_layout()
    plt.show()

def count_lost_packets(data_path, expected_messages):
    # Load the data
    df = pd.read_csv(data_path, sep='|', header=None, engine='python')
    df.columns = ['package_size', 'message_number', 'time']
    
    # Clean whitespace
    df = df.applymap(lambda x: x.strip() if isinstance(x, str) else x)
    
    # Convert columns to int/float
    df['package_size'] = df['package_size'].astype(int)
    df['message_number'] = df['message_number'].astype(int)
    
    lost_messages = 0

    # Group by package size
    for (size), group in df.groupby(['package_size']):
        received_messages = set(group['message_number'].unique())
        expected = set(range(expected_messages))
        missing = expected - received_messages
        lost_messages += len(missing)

        print(f"Size {size} => Lost: {len(missing)} messages")

    print(f"\nTotal lost messages: {lost_messages}")
    return lost_messages

# Path to your CSV file
csv_file_path = '/Users/matasjones/Documents/Coding_projects/testing/testing_logs/logger1.csv'
plot_data_from_csv(csv_file_path)

