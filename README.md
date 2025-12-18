# (WIP) kalman-visualizer

State estimation visualizer built with egui + eframe.

## Components

### (WIP) Dynamic System Engine

- Data structures to represent continuous and discrete dynamic systems
- Integrator choices (Euler and RK4)
- Discretization for continous systems
- Step function
- Simulated measurements using gaussian noise

### (WIP) State Estimators

- Basic Kalman Filter
- Extended Kalman Filter
- Particle Filter
- Interacting Multiple Model (IMM) Builder

### (WIP) Estimator Visualization

- Plots estimate and covariance of selected states (2D)
- Displays error to true state
- Highlights current best model depending on conditions

### (WIP) Playable Environment

- Objects controllable through user input
- Contains the system that estimators attempt to track
- Simulates disturbances with gaussian noise

### (WIP) Core

- Common data structures
- Common implementation blocks
