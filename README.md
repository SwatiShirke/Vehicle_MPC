# Trajectory Tracking for Ackerman Steering using MPC and Acados

This repository implements trajectory tracking for an Ackerman steering system using **Model Predictive Control (MPC)** with the **Acados library**. The implementation focuses on tracking given reference trajectories while optimizing control inputs like steering angle and velocity.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Results](#results)
- [References](#references)

---

## Overview

The project demonstrates:
- Designing an OCP problem for MPC controller for an Ackerman steering model.
- Using the Acados library for efficient optimization.
- Integration with Carla Unreal Engine for reat-time testing. 

---

## Features

- Trajectory tracking for various reference paths.
- Visualization of:
  - 2D trajectory plots (vehicle vs. reference path).
  - Control inputs (steering angle and throttle).
- 

---

## Requirements

### Software:
- Python 3.x
- [Acados](https://github.com/acados/acados) library
- NumPy
- Matplotlib
- CasADi
- Carla Unreal Engine 

### Installation:
Install required Python libraries with:
```bash
pip install numpy matplotlib casadi
```

---

## Usage

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/ackerman-mpc-acados.git
   cd ackerman-mpc-acados
   ```

2. Run the main script:
   ```bash
   python main.py
   ```

3. Visualize results in the generated plots or refer to the images below.

---

## Results


Below are the plots for the run tested in Carla engine. The graps shows position, velocities as well as steering angle and 
accelerator input. 

---

### Trajectory and velocity
| 2D Trajectory Plot | Reference Vs Followed |
|---------------------|--------------------|
| ![Reference Vs Followed](Results/plots_10mps/trajectory_vs_followed_path.png) | ![Long. Velocity](Results/plots_10mps/velocity_over_time.png) |

---

### Control Input
|Throttle Input| Steering Input |
|---------------------|--------------------|
| ![Throttle Input](Results/plots_10mps/throttle_command.png) | ![Steering Input](Results/plots_10mps/steering_command.png) |

---

### Position
| X Position | Y Position |
|---------------------|--------------------|
| ![X Position](Results/plots_10mps/x_position_over_time.png) | ![Y Position](Results/plots_10mps/y_position_over_time.png) |

### Position
| Heading Angle
|---------------------
| ![Heading Angle](Results/plots_10mps/yaw_angle_over_time.png) 

---

Each plot shows the performance of the Model Predictive Controller (MPC) in tracking the desired path, while the control input plots illustrate the steering and velocity inputs used to achieve the trajectory tracking.


---

## References

- [Acados Documentation](https://docs.acados.org/)
- [Ackerman Steering Model](https://en.wikipedia.org/wiki/Ackermann_steering_geometry)

---

Phase 2: 
Dynamic VD model is built in MATLAB Simulink and tested.
Testing results are as follows:
# Velocity Control on 4-Wheel Driveline Model with PID Controller

## Case Studies
### Case 1 Torque in = 225 NM and Steering angle = 0 rad/s  
| **Longitudinal Velocity** | **Lateral Velocity** |
|---------------------------|-----------------------|
| ![Longitudinal Velocity](Results/MATLAB_Results/case1/Vx.jpg) | ![Lateral Velocity](Results/MATLAB_Results/case1/Vy.jpg) |

| **Pose (2D)**             | **Lateral Acceleration** |
|---------------------------|---------------------------|
| ![Pose (2D)](Results/MATLAB_Results/case1/pose.jpg) | ![Lateral Acceleration](Results/MATLAB_Results/case1/lat_accel.jpg) |

---

### Case 2 Torque in = 450 NM and Steering angle = 0 rad/s
| **Longitudinal Velocity** | **Lateral Velocity** |
|---------------------------|-----------------------|
| ![Longitudinal Velocity](Results/MATLAB_Results/Case-2/Vx.jpg) | ![Lateral Velocity](Results/MATLAB_Results/Case-2/Vy.jpg) |

| **Pose (2D)**             | **Lateral Acceleration** |
|---------------------------|---------------------------|
| ![Pose (2D)](Results/MATLAB_Results/Case-2/pose.jpg) | ![Lateral Acceleration](Results/MATLAB_Results/Case-2/Lat_acc.jpg) |

---

### Case 3 Torque in = 100 NM and Steering angle = 0.1 rad/s
| **Longitudinal Velocity** | **Lateral Velocity** |
|---------------------------|-----------------------|
| ![Longitudinal Velocity](Results/MATLAB_Results/case-3/Vx.jpg) | ![Lateral Velocity](Results/MATLAB_Results/case-3/Vy.jpg) |

| **Pose (2D)**             | **Lateral Acceleration** |
|---------------------------|---------------------------|
| ![Pose (2D)](Results/MATLAB_Results/case-3/pose.jpg) | ![Lateral Acceleration](Results/MATLAB_Results/case-3/lat_acc.jpg) |

---
### Case 4 Torque in = 100 NM and Steering angle = -0.1 rad/s Right Turn
| **Longitudinal Velocity** | **Lateral Velocity** |
|---------------------------|-----------------------|
| ![Longitudinal Velocity](Results/MATLAB_Results/Case-4/Vx.jpg) | ![Lateral Velocity](Results/MATLAB_Results/Case-4/Vy.jpg) |

| **Pose (2D)**             | **Lateral Acceleration** |
|---------------------------|---------------------------|
| ![Pose (2D)](Results/MATLAB_Results/Case-4/Pos.jpg) | ![Lateral Acceleration](Results/MATLAB_Results/Case-4/lat_accel.jpg) |

---

## PID Tuning
| **PID Tuning - Graph 1** | **PID Tuning - Graph 2** |
|--------------------------|--------------------------|
| ![PID Tuning 1](Results/MATLAB_Results/PID_tunning/Vel_graph.jpg) | ![PID Tuning 2](Results/MATLAB_Results/PID_tunning2/Vx.jpg) |

---
Feel free to contribute or suggest improvements! 😊
