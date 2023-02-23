# Motion_planning_of_Robots

## Overview
This repository implements different motion planning algorithms like Heuristic and Incremental Search based and Sampling based algorithms in a 2D environment for a point robot. All the algorithms are implemented in python. The environment is implemented using a priority queue and is made effecient using the concept of OOPs.

## Project Structure

```
    .
    ├── Search Based            
    │   ├── INN                 # Inverible Neural Networks
    │   ├── RBF                 # Radial Basis Functions
    │   ├── MLP                 # Multi-Layer Perceptron
    │   ├── ELM                 # Extreme Learning Machine
    │   └── Cascade_Correlatin  # Cascade Correlation
    |
    ├── Dataset                 # Contain the workspace generated using different models as a pkl file
    |
    ├── Models                  # Kinematic Robot Models
    │   ├── PCC                 # Piecewise Constant Curvature Model
    │   └── Static              # Static Model based on cosserat rod theory and string theory
    |
    ├── Results                 # Result Directory
    │   ├── Training            # All the learning models are stored as sav file
    │   ├── Trajectory          # Stores the trajectory output from learning models
    │   └── Workspace           # Snipet of workspace generated using Kinematic Models
    |
    ├── main.py.                # Main program to train different learning models
    └── trajectory.py           # Test the trained model on different trajectories
    
```

## Heuristic Search Based Algorithm

<p align="center">
<img src="./Results/RRT_Image/RRT.gif" width="300" alt="rrt_traj">
</p>

## Incremental Search Based Algorithm

<p align="center">
<img src="./Results/PRM_Image/PRM.gif" width="300" alt="prm_traj">
</p>

## Contact

If you have any questions, please let me know:

- Shaswat Garg {[sis_shaswat@outlook.com]()}

