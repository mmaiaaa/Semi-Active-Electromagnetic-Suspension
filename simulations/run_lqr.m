clc; close all;

[A, ~, B_act, C_acc, D_acc] = state_space_quarter_car();
[T, z_g, z_gdot] = road_profile_half_sine();
quarter_car_parameters;

K_lqr_aug = lqr_design(A,B_act);
x0 = zeros(5,1);

odefun = @(t,x) state_deriv_lqr(t,x,A,B_act,T,z_g,z_gdot,...
                                K_lqr_aug,F_max,C_acc,D_acc,k_t,c_t,m_t);

[tt,XX] = ode45(odefun,T,x0);
