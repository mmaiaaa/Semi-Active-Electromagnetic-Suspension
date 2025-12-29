## Simulation Setup

All simulations were performed in MATLAB using a linear state-space representation of the quarter-car suspension model.

### Road Profile
A half-sine speed bump was used to emulate realistic road conditions:
- Height: approximately 0.089 m
- Length: approximately 0.375 m
- Vehicle speed: approximately 7.2 km/h

This input produces a transient excitation representative of urban road disturbances.

### Simulation Parameters
- Time-domain simulation
- Identical initial conditions for all controllers
- Same road input applied to passive and semi-active systems

### Evaluation Metrics
- Body displacement
- Body acceleration
- Suspension deflection
- RMS body acceleration
- Actuator force and power consumption

This setup ensures objective performance comparison across control strategies.

## Control Methods

Three control strategies were implemented to regulate the electromagnetic actuator force in the semi-active suspension system.

### PD Controller
The PD controller uses proportional and derivative feedback to enhance damping and reduce vibration peaks. It is effective for transient vibration suppression and provides improved ride comfort with minimal complexity.

### PID Controller
The PID controller introduces integral action to reduce steady-state error and improve disturbance rejection. While effective, the integral term can introduce overshoot and reduce damping if not carefully tuned.

### LQR Controller
The Linear Quadratic Regulator is designed using a state-space approach to minimize a quadratic cost function balancing state deviation and control effort.


Each controller was tuned manually and evaluated under identical simulation conditions.
