clc; close all;

[A, B, ~, C_acc, D_acc] = state_space_quarter_car();
[T, z_g, z_gdot] = road_profile_half_sine();

Kpdef = 8000;
Kddef = 6500;
Kacc  = 500;
F_max = 2000;

x0 = zeros(4,1);

odefun = @(t,x) state_deriv_pd(t,x,A,B,T,z_g,z_gdot,...
                               Kpdef,Kddef,Kacc,F_max,C_acc,D_acc);

[tt,XX] = ode45(odefun,T,x0);
