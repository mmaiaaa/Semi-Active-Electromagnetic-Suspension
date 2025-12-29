clc; close all;

addpath('../model');
addpath('../controllers/pid');

% SYSTEM
[A, B, ~, C_acc, D_acc] = state_space_quarter_car();
[T, z_g, z_gdot] = road_profile_half_sine();

% CONTROLLER PARAMETERS
Kpdef = 7500;
Kddef = 6300;
Ki    = 5300;
Kacc  = 500;
F_max = 2000;

% INITIAL CONDITION
x0 = zeros(5,1);

% SIMULATION
odefun = @(t,x) state_deriv_PID(t,x,A,B,T,z_g,z_gdot,Kpdef,Kddef,Ki,Kacc,F_max,C_acc,D_acc);
[tt, XX] = ode45(odefun, T, x0);

% PID controller
[F_hist, a_b_act] = pid_controller(XX, T, z_g, z_gdot, Kpdef, Kddef, Ki, Kacc, F_max, C_acc, D_acc);


z_b_act    = XX(:,1); 
z_t_act    = XX(:,2);
z_bdot_act = XX(:,3); 
z_tdot_act = XX(:,4);
defl_act   = z_b_act - z_t_act;
tire_defl_act = z_t_act - interp1(T,z_g,tt);
