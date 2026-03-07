# kalman-visualizer

State estimation visualizer built with egui + eframe.

## Currently Completed

**Non-Linear Time-Invariant system data structure and integrators**:
<img width="859" height="407" alt="image" src="https://github.com/user-attachments/assets/af7aec6c-e6ab-4dd8-b86e-909fe8b17843" />

Can be used to represent any arbitrary NLTI/LTI system with any set of inputs


Simple key controls by polling eframe input state and sending events through channels to input thread:

![kalman visualizer](https://github.com/user-attachments/assets/c7be2d0c-bc21-422c-93d9-4e3704d99123)

*Using W to go forward. Car naturally reaches top speed due to wind resistance.* 

## Components

### Dynamic System Engine (Core)

- Data structures to represent continuous and discrete dynamic systems
- Integrator choices (Euler and RK4)
- Discretization for continous systems
- Step function
- Simulated measurements using gaussian noise
- Common data structures
- Common implementation blocks

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

## References

- RK4: [https://lpsa.swarthmore.edu/NumInt/NumIntFourth.html]
- Vehicle Dynamics: Vehicle Dynamics and Control by Rajesh Rajamani, Second Edition
- Kalman Filter: [https://kalmanfilter.net/multiSummary.html]
