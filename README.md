# Semi-Active-Electromagnetic-Suspension

## Overview
This project investigates a Semi Active Electromagnetic Suspension system that combines conventional hydraulic damper with an electromagnetic actuator to improve vehicle ride comfort and stability. The system is evaluated using a quarter car model under a realistic road disturbance profile and compared against a passive suspension.

Three control algorithms are implemented and analysed:
- PD
- PID
- LQR

The study focuses on vibration attenuation, suspension travel, and actuator energy efficiency.

## System Model
A two degree of freedom quarter car model is used, consisting of:
- Body mass (vehicle body)
- Tire mass (tire and wheel)

The electromagnetic actuator operates in parallel with the mechanical spring and damper, providing a controllable force input.

The state variable and the body displacement and velocity, and the tire displacement and velocity.
The road input is modeled as a half-sine speed bump profile with height of 0.089 m, length of 0.375 m, and vehicle speed of 7.2 km/h.

## Control Strategy
The electromagnetic actuator force is regulated using feedback control to modify the effective damping of the suspension system.

- PD controller: Enhances damping and vibration attenuation
- PID controller: Improves steady-state error with integral gain
- LQR controller: Optimal control for balancing performance and effort

All controllers are implemented and simulated in MATLAB using the same operating conditions for dair comparison; however, they are all manually tuned.

## Key Results
### Ride Comfort (RMS Acceletation Reduction)
- **PD** : 20.0% Reduction
- **PID** : 17.9% Reduction
- **LQR** : 10.5% Reduction

PD controller delivered the best overall ride comfort while the PID reduced the steady-state error but introduced minor overshoot. The LQR controller required careful tuning and prioritized state regulation over comfort.

### Displacement and Stability
The LQR controller achieved the smallest body displacement amplitude with a peak body displacement of 0.0635 m. It had the fastest settling and smoother displacement decay compared to PD and PID controllers.

### Actuator Effort and Energy Efficiency
- **PD & PID**:
The actuator force ranges up to ±1500 N, and has higher and more erratic power consumption.
- **LQR**:
The actuator force ranges within ±800 N, and has more structured and energy-sufficient power usage.

The LQR controller is better suited for energy contraints, while the PD and PID controllers prioritize comfort.

## Running the project
1. Open MATLAB and navigate to the project folder.
2. Add required subfolders to MATLAB path:
   
   addpath('model');
   
   addpath('controllers');
   
   addpath('simulations');
4. Run the main script:
   
   run_simulation;

## Limitations
- Linearized system dynamics
- No actuator saturation or thermal effects
- No sensor noise or time delay
- Quarter car model (no roll or pitch dynamics)
- Manually tuned controllers

## Future Improvements
- Controller Enhancements: adaptive or self-tuning controllers, exploring nonlinear control strategies.
- Actuator Modeling: include saturation limits.
- Vehicle Model: full-car model to capture pitch, roll, and cross-coupled dynamics.
- Simulation and Visualization: integrate 3D vehicle visualization.

## Tools and Skills
- Vehicle dynamics modeling
- State-space systems
- Control system design
- MATLAB simulation and analysis

## Conclusion
The Semi-Active Electromagnetic Suspension system outperforms a passive suspension under realistic road disturbances. Among the tested controllers, the PD controller provides te mod balanced improvement in ride comfort, while the LQR controller offers energy efficiency and displacement control with proper tuning. The controller choice depends on whether comfort or efficiency is the primary design goal.
