import os
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Define your main folder path here
main_folder = "/home/waseem/Documents/HBRS/RnD/catkin_ws/src/experiments/new/direct_test"

# Data collection
all_data = []

# List of test types and algorithms
test_types = ['random', 'rotational', 'translational']
algorithms = ['amcl', 'optimal']

# Particle counts
particle_counts = [1, 3, 50, 100, 250, 500]

# Load data
for test_type in test_types:
    for alg in algorithms:
        for particles in particle_counts:
            for run in range(1, 6):  # 5 experiments per configuration
                file_name = f"{alg}_{particles}_{run}_particles.csv"
                file_path = os.path.join(main_folder, test_type, alg, file_name)
                # print(file_path)
                
                # Check if file exists
                if os.path.exists(file_path):
                    data = pd.read_csv(file_path)
                    
                    # Calculate the average RMSE for the experiment and convert to centimeters
                    avg_rmse = data['RMSE_Position'].median() * 100

                    # print("!!")
                    
                    # Append to all_data list
                    all_data.append({
                        'Experiment': test_type,
                        'Algorithm': alg,
                        'Particles': particles,
                        'AvgRMSE': avg_rmse
                    })

# Create DataFrame
all_data = pd.DataFrame(all_data)

# print("DataFrame columns:", all_data.columns)

# Plotting
for test_type in test_types:
    # Filter data for the specific test type
    test_data = all_data[all_data['Experiment'] == test_type]
    
    # Box Plot for the current test type
    plt.figure(figsize=(12, 6))
    sns.boxplot(
        data=test_data, 
        x='Particles', 
        y='AvgRMSE', 
        hue='Algorithm',
        palette='Set2'
    )
    plt.title(f'RMSE Distribution by Number of Particles and Algorithm - {test_type.capitalize()} Test')
    plt.xlabel('Number of Particles')
    plt.ylabel('RMSE (cm)')
    plt.legend(title='Algorithm')
    plt.grid(axis='y', linestyle='--', alpha=0.5)
    plt.show()
    
    # Line Chart for the current test type
    plt.figure(figsize=(12, 6))

    for alg in ['amcl', 'optimal']:
        alg_data = test_data[test_data['Algorithm'] == alg]
        color = 'orange' if alg == 'amcl' else 'blue'
        
        avg_rmse_points = []
        x_positions = list(range(len(particle_counts)))  # For equidistant spacing
        
        for i, particles in enumerate(particle_counts):
            particle_data = alg_data[alg_data['Particles'] == particles]
            
            # Plot individual RMSE averages per experiment with a light color
            plt.scatter(
                x=[i] * len(particle_data), 
                y=particle_data['AvgRMSE'], 
                color=color, 
                alpha=0.4
            )
            
            # Calculate and plot the mean of these 5 averaged RMSE values in a darker color
            overall_avg = particle_data['AvgRMSE'].mean()
            avg_rmse_points.append(overall_avg)
            plt.scatter(
                i, 
                overall_avg, 
                color=color, 
                edgecolor='k', 
                s=100
            )
        
        # Plot the line connecting the averaged points for each algorithm
        plt.plot(x_positions, avg_rmse_points, color=color, linestyle='-', marker='o', label=f'{alg.capitalize()} (Average Line)')

    # Set custom x-axis labels for equidistant plotting
    plt.xticks(x_positions, particle_counts)
    plt.title(f'Median RMSE by Number of Particles and Algorithm - {test_type.capitalize()} Test')
    plt.xlabel('Number of Particles')
    plt.ylabel('RMSE (cm)')
    plt.legend()
    plt.grid(axis='y', linestyle='--', alpha=0.5)
    plt.show()




#TIME COMPLEXITY

# import os
# import pandas as pd
# import matplotlib.pyplot as plt
# import seaborn as sns

# # Define your main folder path here
# main_folder = "/home/waseem/Documents/HBRS/RnD/catkin_ws/src/experiments/new/robustness/circular_room/time_computation_optimal"

# # Data collection
# all_data = []

# # Particle counts and experiments
# particle_counts = [1, 3, 50, 100, 250, 500]
# num_experiments = 5  # Number of runs per configuration

# # Load and process data
# for particles in particle_counts:
#     for run in range(1, num_experiments + 1):
#         file_name = f"particle_computation_times_{particles}_{run}.csv"
#         file_path = os.path.join(main_folder, file_name)
        
#         # Check if file exists
#         if os.path.exists(file_path):
#             data = pd.read_csv(file_path)
            
#             # Compute median or mean time for each particle
#             avg_times = data.median(axis=0)  # Median across rows for each column (time per particle)
#             avg_times = avg_times.values  # Convert to array
            
#             # Append data for plotting
#             for particle_idx, time in enumerate(avg_times):
#                 all_data.append({
#                     'ParticleCount': particles,
#                     'ParticleIndex': particle_idx + 1,
#                     'Time': time,
#                     'Run': run
#                 })
#         else:
#             print(f"File not found: {file_path} - Skipping this configuration.")

# # Create DataFrame
# all_data = pd.DataFrame(all_data)

# # Check if data exists
# if all_data.empty:
#     print("No data found. Please check the file paths or ensure the files exist.")
# else:
#     # Box Plot: Time complexity distribution across particle counts
#     plt.figure(figsize=(12, 6))
#     sns.boxplot(
#         data=all_data, 
#         x='ParticleCount', 
#         y='Time',
#         hue='ParticleCount',  # Assign x to hue to suppress the warning
#         palette='Set2',
#         legend=False  # Disable duplicate legends
#     )
#     plt.title('Time Complexity Distribution by Particle Count - Circular Room Test')
#     plt.xlabel('Number of Particles')
#     plt.ylabel('Time (ms)')
#     plt.grid(axis='y', linestyle='--', alpha=0.5)
#     plt.show()

#     # Line Plot: Median time scaling with particle count
#     plt.figure(figsize=(12, 6))

#     for run in range(1, num_experiments + 1):
#         run_data = all_data[all_data['Run'] == run]
#         # Filter for existing particle counts to avoid mismatches
#         valid_particle_counts = sorted(run_data['ParticleCount'].unique())
#         run_avg = run_data.groupby('ParticleCount')['Time'].mean()

#         plt.plot(
#             valid_particle_counts,  # Only include valid x values
#             run_avg, 
#             label=f'Run {run}', 
#             marker='o', 
#             linestyle='--', 
#             alpha=0.8
#         )

#     # Aggregate average across all runs
#     overall_avg = all_data.groupby('ParticleCount')['Time'].mean()
#     overall_particle_counts = sorted(overall_avg.index)

#     plt.plot(
#         overall_particle_counts, 
#         overall_avg, 
#         label='Overall Average', 
#         marker='o', 
#         color='black', 
#         linewidth=2
#     )

#     plt.title('Time Complexity Scaling with Particle Count')
#     plt.xlabel('Number of Particles')
#     plt.ylabel('Time (ms)')
#     plt.legend()
#     plt.grid(axis='y', linestyle='--', alpha=0.5)
#     plt.show()
