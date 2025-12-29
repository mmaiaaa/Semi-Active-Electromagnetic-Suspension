clc; close all;

addpath('../model');
addpath('../controllers/pid');

[A, B, ~, C_acc, D_acc] = state_space_quarter_car();
[T, z_g, z_gdot] = road_profile_half_sine();

Kpdef = 7500;
Kddef = 6300;
Ki    = 5300;
Kacc  = 500;
F_max = 2000;

x0 = zeros(5,1);

odefun = @(t,x) state_deriv_pid(t,x,A,B,T,z_g,z_gdot,...
                                Kpdef,Kddef,Ki,Kacc,F_max,C_acc,D_acc);

[tt,XX] = ode45(odefun,T,x0);
