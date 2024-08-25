import pandas as pd
import matplotlib.pyplot as plt

# Load the data from the Excel file
file_path = 'D:/FYP/Matlab/Results.xlsx'
df = pd.read_excel(file_path, sheet_name='Results')

algorithms = ['RRT', 'RRT*', 'Bidirectional RRT*']

for environment, env_group in df.groupby('Environment'):
    for algorithm in algorithms:
        # Filter for the current algorithm and its variations
        group_algo = env_group[env_group['Algorithm'] == algorithm]
        group_algo_1st_solution = env_group[env_group['Algorithm'] == f'{algorithm} 1st Solution'] if algorithm != 'RRT' else pd.DataFrame(columns=group_algo.columns)
        
        # Identify missing sampling methods in the 1st Solution
        missing_methods = set(group_algo['Sampling Method']) - set(group_algo_1st_solution['Sampling Method'])
        sampling_methods_order = group_algo['Sampling Method'].tolist()
        ordered_missing_methods = [method for method in sampling_methods_order if method in missing_methods]

        
        # Add rows with zero values for missing sampling methods in 1st Solution
        for method in ordered_missing_methods:
            group_algo_1st_solution = pd.concat([group_algo_1st_solution, pd.DataFrame({
                'Environment': [environment],
                'Algorithm': [f'{algorithm} 1st Solution'],
                'Sampling Method': [method],
                'Solutions Found': [0],
                'Average Time Taken': [0],
                'Time Taken Std Dev': [0],
                'Average Node Count': [0],
                'Node Count Std Dev': [0],  
                'Average Path Length': [0],
                'Path Length Std Dev': [0]
            })], ignore_index=True)
        
        # Merge data on the 'Sampling Method' to ensure alignment
        merged_group = pd.merge(group_algo_1st_solution, group_algo, on='Sampling Method', suffixes=('_1st', '_non_1st'), how='left')

        # Set up the figure and axis
        fig, ax1 = plt.subplots(figsize=(12, 6))
        
        # Bar positions
        x = range(len(merged_group))
        
        # Plotting Time Taken
        if algorithm != 'RRT':
            ax1.bar([p - 0.3 for p in x], merged_group['Average Time Taken_1st'], width=0.2, label=f'{algorithm} 1st Solution - Average Time Taken', align='center', color='blue')
        ax1.bar([p - 0.1 for p in x], merged_group['Average Time Taken_non_1st'], width=0.2, label=f'{algorithm} - Average Time Taken', align='center', color='red')
        ax1.set_xlabel('Sampling Method')
        ax1.set_ylabel('Average Time Taken (s)', color='blue')
        ax1.tick_params(axis='y', labelcolor='blue')

        # Plotting Path Length
        ax3 = ax1.twinx()
        if algorithm != 'RRT':
            ax3.bar([p + 0.1 for p in x], merged_group['Average Path Length_1st'], width=0.2, label=f'{algorithm} 1st Solution - Average Path Length', align='center', color='green')
        ax3.bar([p + 0.3 for p in x], merged_group['Average Path Length_non_1st'], width=0.2, label=f'{algorithm} - Average Path Length', align='center', color='cyan')
        ax3.set_ylabel('Average Path Length', color='green')
        ax3.tick_params(axis='y', labelcolor='green')

        # Setting the x-ticks to the sampling method
        xticks_labels = merged_group['Sampling Method'].tolist()
        ax1.set_xticks([p for p in x])
        ax1.set_xticklabels(xticks_labels, rotation=45, ha='right')

        # Adding titles and labels
        if algorithm != 'RRT':
            plt.title(f'Environment: {environment}, Algorithm: {algorithm} vs. {algorithm} 1st Solution')
        else:
            plt.title(f'Environment: {environment}, Algorithm: {algorithm}')
        ax1.legend(loc='upper left')
        ax3.legend(loc='upper right')
        
        # Show the plot
        plt.show()
