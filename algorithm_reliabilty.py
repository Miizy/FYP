import pandas as pd
import matplotlib.pyplot as plt

# Load the data from the Excel file
file_path = 'D:/FYP/Matlab/Results.xlsx'
df = pd.read_excel(file_path, sheet_name='Results')

# Exclude rows where the 'Algorithm' column contains '1st Solution'
df_filtered = df[~df['Algorithm'].str.contains('1st Solution')]

# Remove the word "Environment" from the 'Environment' column
df_filtered['Environment'] = df_filtered['Environment'].str.replace('Environment', '', regex=False).str.strip()

# Extract the unique environments and sampling methods in the order they appear
environments_order = df_filtered['Environment'].unique().tolist()
sampling_methods_order = df_filtered[df_filtered['Algorithm'] == 'Bidirectional RRT*']['Sampling Method'].unique().tolist()

# Convert 'Environment' and 'Sampling Method' columns to categorical types with the extracted orders
df_filtered['Environment'] = pd.Categorical(df_filtered['Environment'], categories=environments_order, ordered=True)
df_filtered['Sampling Method'] = pd.Categorical(df_filtered['Sampling Method'], categories=sampling_methods_order, ordered=True)

# Group by 'Environment' and 'Sampling Method', keeping the original order
grouped_df = df_filtered.groupby(['Environment', 'Sampling Method'])['Solutions Found'].sum().unstack()

# Create a grouped bar chart
ax = grouped_df.plot(kind='bar', figsize=(12, 6), width=0.8)

# Adding title and labels
plt.title('Reliability of Sampling Method')
plt.xlabel('Environment')
plt.ylabel('Solutions Found')
plt.xticks(rotation=45)
plt.grid(axis='y')

# Adjusting the legend to be outside the plot area and represent Sampling Methods
box = ax.get_position()
ax.set_position([box.x0, box.y0, box.width * 0.75, box.height])  # Resize plot area
ax.legend(title='Sampling Method', loc='center left', bbox_to_anchor=(1, 0.5))  # Place legend to the right

# Show the plot
plt.tight_layout()
plt.show()
