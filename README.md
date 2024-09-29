# Drone Collision Detection Simulation

## Overview

This Python script simulates the movement and collision detection of 3D drone models placed on a NxN grid surface. It visualizes the drones, computes collision detection using various bounding volumes, and adjusts drone movement to avoid collisions. The script also includes a protocol for landing and takeoff to ensure that drones do not collide during these phases. Statictics are also calculated. Collisions of 2D projections.

## Features

### Part A

1. **3D Model Visualization**:  
   - Visualizes randomly placed 3D drone models on a landing/takeoff grid with NxN positions.
   - Models the environment with a surface for drone landing and takeoff.

2. **Bounding Volume Computations**:  
   - Computes **Convex Hull**, **Axis-Aligned Bounding Box (AABB)** for each 3D drone model.

3. **Collision Detection Algorithm A**:  
   - Implements collision detection based only on the calculated bounding volumes (AABB).

4. **Collision Detection Algorithm B**:  
   - Enhances the accuracy of **Algorithm A** by adding more precision. (CONVEX HULL)

5. **Collision Detection Algorithm C**:  
   - Implements a fully accurate collision detection algorithm to ensure the most precise collision results. (ACCURATE COLLISION)

6. **Random Initialization**:  
   - Randomly initializes the positions and orientations of drones, visualizes their movements, and shows detected collisions.

### Part B

7. **Dynamic Bounding Volume Update**:  
   - Uses the velocity of each drone to update its bounding volume in real-time.
   - Detects collisions during the time interval **dt** between frames and visualizes them.

8. **Velocity Adjustment for Collision Avoidance**:  
   - Modifies the velocity (direction and/or speed) of certain drones to prevent collisions during their movement.

9. **Landing-Takeoff Protocol**:  
   - Defines a protocol to ensure collision-free landing and takeoff of drones.
   - The number of drones, their appearance time, and speed are parametrized and randomized.

10. **Simulation and Statistics Visualization**:  
    - Displays interesting statistics.

## Installation

1. **Prerequisites**:  
   - Python 3.10.13
   - Environment: 
   ```bash
    conda create --name drone-sim python=3.10.13 
    conda activate drone-sim

2. **Installation**:
   ```bash
   pip install -r requirements.txt
