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

Feel free to contribute or suggest improvements! 😊
