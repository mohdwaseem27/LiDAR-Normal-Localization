import os
import pandas as pd
import matplotlib.pyplot as plt

# Define the folder path
base_folder = '/home/waseem/Documents/HBRS/RnD/catkin_ws/src/experiments/direct_test/translational'
folders = ['als', 'amcl', 'optimal', 'optimal_2d']

# Initialize data storage
data = {
    'als': {'x': [], 'y': []},
    'amcl': {'x': [], 'y': []},
    'optimal': {'x': [], 'y': []},
    'optimal_2d': {'x': [], 'y': []},
}

# Function to get number of particles from filename
def get_num_particles(filename):
    return int(filename.split('_')[1])

# Loop through each folder and process CSV files
for folder in folders:
    folder_path = os.path.join(base_folder, folder)
    for file in os.listdir(folder_path):
        if file.endswith('.csv'):
            file_path = os.path.join(folder_path, file)
            print(file_path)
            df = pd.read_csv(file_path)
            
            # Find the RMSE row and calculate the average
            rmse_values = df[df.iloc[:, 0] == 'RMSE'].iloc[:, 1:].mean(axis=1).values
            if len(rmse_values) > 0:
                avg_rmse = rmse_values[0]
                num_particles = get_num_particles(file)
                
                # Store the data
                data[folder]['x'].append(num_particles)
                # data[folder]['y'].append(avg_rmse * 100)  # Convert to cm

# Plotting the results
fig = plt.figure(figsize=(16, 16))

for folder in folders:
    plt.plot(data[folder]['x'], data[folder]['y'], label=folder.capitalize())
    plt.scatter(data[folder]['x'], data[folder]['y'], marker='s')

# Adding equal-sized grids
plt.grid(True)
plt.yscale('log')
yticks = [1, 10, 100, 1000]
plt.yticks(yticks, [str(val) for val in yticks])

# Adding labels and title
plt.xlabel('Number of particles')
plt.ylabel('RMSE (cm)')
plt.title('Translational errors vs Number of particles')

# Adding legend
plt.legend()

# Show the plot
plt.show()
