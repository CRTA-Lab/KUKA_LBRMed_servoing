# Cuboid_heatmap

This project evaluates and visualizes the workspace of a **KUKA iiwa14 robotic arm**, computing inverse kinematics and scoring positions based on manipulability, joint limits, and singularity metrics.

## Project Structure

- `main.py` – Main script: generates a 3D grid around a target, evaluates scores, visualizes results, and plots the robot configuration.
- `robot_utils.py` – Robot utilities: initialize robot chain, solve inverse kinematics, compute manipulability, joint limits, and singularity metrics.
- `sampling.py` – Sampling utilities: create 3D grids and sample orientations within a cone.
- `evaluation.py` – Evaluate scores for each position and orientation.
- `draw.py` – Visualization utilities (e.g., draw cuboids).

## Dependencies

- Python ≥ 3.8  
- `numpy`  
- `matplotlib`  
- `ikpy`  

## Installation

```bash
pip install numpy matplotlib ikpy
