import pandas as pd
import matplotlib.pyplot as plt

# Define file paths
file_paths = ["optimal_1_particles.csv", "optimal_3_particles.csv", "optimal_50_particles.csv", "optimal_100_particles.csv", "optimal_250_particles.csv", "optimal_500_particles.csv"]

# Define line colors and labels
line_colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown']
line_labels = ['1 particle', '3 particles', '50 particles', '100 particles', '250 particles', '500 particles']

# Plotting
plt.figure(figsize=(10, 6))

for i, file_path in enumerate(file_paths):
    # Read CSV file
    data = pd.read_csv(file_path)
    print(data)

    # Plot RMSE values using pandas plot
    data['RMSE'].plot(color=line_colors[i], label=line_labels[i])

# Add labels and legend
plt.xlabel('Index')
plt.ylabel('Errors (m)')
plt.legend()

# Show plot
plt.grid(True)
plt.show()
