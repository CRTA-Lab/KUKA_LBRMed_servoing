# Genetic_algo

This project implements a **Genetic Algorithm (GA)** to optimize the inverse kinematics of a KUKA iiwa14 robotic arm. It evaluates robot configurations based on **manipulability**, **joint limits**, and **singularity metrics**.

## Features
- Compute **inverse kinematics** for a target TCP position.
- Calculate **Jacobian**, **manipulability**, **joint limit distances**, and **singularity metrics**.
- Optimize robot pose using a **Genetic Algorithm**.

## Dependencies
- Python ≥ 3.8  
- `numpy`  
- `matplotlib`  
- `ikpy`

## Install dependencies
```bash
pip install numpy matplotlib ikpy
