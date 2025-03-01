# Advancing 2D LiDAR-based Indoor Localization using Line Normal-constrained Particle Filters


## Overview
This repository contains the implementation of a 2D LiDAR-based indoor localization algorithm using line normals in a particle filter framework. 

Robot localization is a critical challenge in autonomous navigation, requiring accurate estimation of position and orientation in diverse and often ambiguous environments. Particle filters provide a robust probabilistic framework for localization, effectively handling multimodal uncertainty distributions. However, traditional approaches struggle with issues such as particle degeneracy and the trade-off between accuracy and robustness, particularly when faced with map discrepancies and feature-sparse environments.

While various enhancements to observation models have been proposed to improve robustness, they often come at the cost of reduced informativeness, potentially limiting overall accuracy. Additionally, conventional particle filters may require a high number of particles to maintain precision, leading to increased computational demands. There is a need for an approach that preserves accuracy while maintaining computational efficiency, particularly in 2D LiDAR-based localization systems.

This research introduces a method that integrates line normals from 2D LiDAR data into the particle filter framework to improve localization accuracy and reliability. By leveraging geometric features inherent in structured environments, this approach enhances the filter’s ability to align particles with the map, reducing localization errors while maintaining computational efficiency. The proposed methodology is systematically evaluated against standard particle filters, demonstrating significant improvements in accuracy and robustness, thereby advancing the effectiveness of 2D localization in indoor environments.

This repository includes:
- ROS implementation of the localization algorithm
- Map generation scripts using PCL
- Benchmarking against AMCL in simulation
- Real-world tests in dynamic and featureless environments

## Real-World Implementation
The following videos showcase the algorithm running on a real robot:

![Home Environment](videos/home_environment.gif) 
![Featureless Corridor](videos/long_corridor.gif)

**📌 Note:** These are GIF representations for quick viewing. For better quality, check out the full videos in the [`videos/`](videos) folder.
