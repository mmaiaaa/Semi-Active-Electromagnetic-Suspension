## Modeling and Equations

The suspension dynamics are derived using Newton’s second law of motion and represented as a linear time-invariant system.

### Equations of Motion

The vertical dynamics of the tire and body are governed by:

Equation (1):
mt * z̈t + ct(żt − żg) + kt(zt − zg) − cb(żb − żt) − kb(zb − zt) = −Fem(t)

Equation (2):
mb * z̈b + cb(żb − żt) + kb(zb − zt) = Fem(t)

Where Fem represents the electromagnetic actuator force applied between the sprung and unsprung masses.

### State-Space Formulation
The state vector is defined as:
x = [zb, zt, żb, żt]ᵀ

The input vector includes:
u = [Fem, zg, żg]ᵀ

The system is represented in standard state-space form:
ẋ = A x + B u  
y = C x + D u

where,

A = [0,0,1,0; 0,0,0,1; -k_b/m_b,k_b/m_b,-c_b/m_b,c_b/m_b; k_b/m_t,-(k_b+k_t)/m_t,c_b/m_t,-(c_b+c_t)/m_t];

B = [0,0,0; 0,0,0; 1/m_b,0,0; -1/m_t,k_t/m_t,c_t/m_t];

C = [-k_b/m_b, k_b/m_b, -c_b/m_b, c_b/m_b];

D = [1/m_b, 0, 0];

This formulation enables systematic controller design and direct implementation of control strategies.

### Assumptions
- Linear spring and damping behavior
- Small displacement approximation
- No actuator saturation or thermal effects
- No roll or pitch dynamics
