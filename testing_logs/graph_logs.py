import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def plot_data_from_csv(csv_file_path):
    # Read the CSV into a DataFrame
    df = pd.read_csv(csv_file_path, delimiter='|', header=None)  # no header row
    
    # Add column names
    df.columns = ['timestamp', 'column_2', 'value']
    
    # Convert data types
    df['column_2'] = pd.to_numeric(df['column_2'], errors='coerce')
    df['value'] = pd.to_numeric(df['value'], errors='coerce')

    # Calculate log10 of column_2 (floor to get the power of 10)
    df['log10_column_2'] = np.floor(np.log10(df['column_2'])).astype(int)

    # Get unique powers of 10 for coloring
    unique_powers = sorted(df['log10_column_2'].unique())
    
    # Create a colormap for both scatter plot and histogram
    colormap = plt.get_cmap('rainbow')
    color_dict = {power: colormap(i / len(unique_powers)) for i, power in enumerate(unique_powers)}
    
    # Compute initial average value for each power of 10
    avg_values = df.groupby('log10_column_2')['value'].mean()

    # Exclude values that are more than 20% above the average
    filtered_df = df[~df.apply(lambda row: row['value'] > 1.2 * avg_values[row['log10_column_2']], axis=1)]

    # Recalculate the average after filtering
    filtered_avg_values = filtered_df.groupby('log10_column_2')['value'].mean()

    # Print the filtered averages
    print("\nFiltered Average Value for Each Power of 10 in column_2 (Excluding Outliers):")
    for power, avg in filtered_avg_values.items():
        print(f"10^{power}: {avg:.4f}")

    # Create a figure with 2 subplots (scatter plot and histogram)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # Scatter Plot (ax1)
    for power in unique_powers:
        subset = df[df['log10_column_2'] == power]
        ax1.scatter(subset['column_2'], subset['value'], color=color_dict[power], alpha=0.6, label=f"10^{power}")

    ax1.set_xlabel('Message size [bytes]')
    ax1.set_ylabel('Roundtrip time [ms]')
    ax1.set_title(f"Experiment {pd.Timestamp.now().strftime('%Y-%m-%d')}")
    ax1.legend(title="Message size [bytes]", title_fontsize='large', fontsize='small', loc='upper left')
    ax1.grid(True, linestyle='--', alpha=0.6)

    # Histogram (ax2)
    bars = ax2.bar(filtered_avg_values.index, filtered_avg_values.values, 
                   color=[color_dict[p] for p in filtered_avg_values.index], alpha=0.6)

    ax2.set_xlabel("Message Size [bytes]")
    ax2.set_ylabel("Roundtrip Time [ms]")
    ax2.set_title("Roundtrip Time vs. Message Size")
    ax2.set_xticks(filtered_avg_values.index)
    ax2.set_xticklabels([f"10^{int(x)}" for x in filtered_avg_values.index])

    # Show the figure with both plots
    plt.tight_layout()
    plt.show()

# Path to your CSV file
csv_file_path = '/Users/matasjones/Documents/Coding_projects/testing/testing_logs/logger1.csv'
plot_data_from_csv(csv_file_path)
