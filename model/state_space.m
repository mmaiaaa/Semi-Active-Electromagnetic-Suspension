function [A, B, B_act, C_acc, D_acc] = state_space_quarter_car()
% State Space Of Linear Quarter Car Suspension Model
% States: x = [z_b; z_t; z_bdot; z_tdot]
% Inputs: u = [F_em; z_g; z_gdot]

% Load physical parameters
quarter_car_parameters;

%% State matrix
A = [
     0           0           1           0;
     0           0           0           1;
    -k_b/m_b     k_b/m_b    -c_b/m_b     c_b/m_b;
     k_b/m_t  -(k_b+k_t)/m_t c_b/m_t -(c_b+c_t)/m_t ];

%% Input matrix
B = [
     0           0           0;
     0           0           0;
     1/m_b       0           0;
    -1/m_t   k_t/m_t     c_t/m_t ];

% Actuator channel only
B_act = B(:,1);

%% Body acceleration output
C_acc = [-k_b/m_b, k_b/m_b, -c_b/m_b, c_b/m_b];
D_acc = [1/m_b, 0, 0];
end
