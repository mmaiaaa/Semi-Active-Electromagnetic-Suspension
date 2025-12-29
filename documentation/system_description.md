## System Description
This project models a Semi-Active Electromagnetic Suspension system using a two degree of freedom quarter car representation. This system integrates a conventional hydraulic damper with an electromagnetic actuator to improve ride comfort and stability.

The model consists of a sprung mass (vehicle body) and an unsprung mass (wheel-tire assembly). The electromagnetic actuator operates in parallel with the mechanical spring-damper system.

### System Components
- **Sprung mass (mb)**
- **Unsprung mass (mt)**
- **Suspension spring (kb)**
- **Mechanical damper (cb)**
- **Electromagnetic actuator (Fem)**
- **Tire stiffness and damping (kt, ct)**
- **Body and Tire vertical displacement (zb, zt)**
- **Road displacement profile (zg)**

### Inputs and Outputs
**Input**
- Road displacement profile

**Outputs**
- Body displacement
- Body acceleration
- Suspension deflection
- Actuator force and power
