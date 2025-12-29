clc; close all;

addpath('../model');
addpath('../controllers/lqr');

% SYSTEM MATRICES
[A, B, ~, C_acc, D_acc] = state_space_quarter_car();
B_act = B(:,1);

% ROAD PROFILE
[T, z_g, z_gdot] = road_profile_half_sine();

% SYSTEM PARAMETERS
m_t = 13; k_t = 30000; c_t = 451;
F_max = 2000;

% AUGMENTED SYSTEM FOR INTEGRAL ACTION
C_defl = [1 -1 0 0];
A_aug = [A zeros(4,1); C_defl 0];
B_aug = [B_act; 0];

% LQR DESIGN
Q_aug = diag([1e6, 1e4, 100, 1e3, 5e5]);
R_aug = 0.001;
K_lqr_aug = lqr(A_aug, B_aug, Q_aug, R_aug);

% INITIAL CONDITION
x0_aug = zeros(5,1);

% SIMULATION
odefun_aug = @(t,x) state_deriv_lqr(t,x,A,B_act,T,z_g,z_gdot,K_lqr_aug,F_max,C_acc,D_acc,k_t,c_t,m_t);
[tt, XX_aug] = ode45(odefun_aug, T, x0_aug);

% LQR CONTROLLER (force & acceleration)
[F_lqr, a_b_lqr] = lqr_controller(XX_aug, T, z_g, z_gdot, K_lqr_aug, F_max, C_acc, D_acc);

% ACTIVE VARIABLES
z_b_lqr = XX_aug(:,1); z_t_lqr = XX_aug(:,2);
z_bdot_lqr = XX_aug(:,3); z_tdot_lqr = XX_aug(:,4);
defl_lqr = z_b_lqr - z_t_lqr;
tire_defl_lqr = z_t_lqr - interp1(T, z_g, tt);

% PASSIVE SYSTEM
sys = ss(A,B,eye(4),zeros(4,3));
u_passive = [zeros(length(T),1), z_g, z_gdot];
Yp = lsim(sys,u_passive,T);
z_b_pass = Yp(:,1); z_t_pass = Yp(:,2);
z_bdot_pass = Yp(:,3); z_tdot_pass = Yp(:,4);
defl_pass = z_b_pass - z_t_pass;
tire_defl_pass = z_t_pass - z_g;
a_b_pass = (C_acc*Yp.' + D_acc*u_passive.').';

% PLOTS
figure; plot(T,z_b_pass,'r',tt,z_b_lqr,'b'); ylabel('z_b (m)'); xlabel('Time (s)');
title('Body Displacement'); legend('Passive','LQR Active'); grid on;

figure; plot(T,z_bdot_pass,'r',tt,z_bdot_lqr,'b'); ylabel('v_b (m/s)'); xlabel('Time (s)');
title('Body Velocity'); legend('Passive','LQR Active'); grid on;

figure; plot(T,a_b_pass,'r',tt,a_b_lqr,'b'); ylabel('a_b (m/s^2)'); xlabel('Time (s)');
title('Body Acceleration'); legend('Passive','LQR Active'); grid on;

figure; plot(T,defl_pass,'r',tt,defl_lqr,'b'); ylabel('Suspension Deflection (m)'); xlabel('Time (s)');
title('Suspension Deflection'); legend('Passive','LQR Active'); grid on;

figure; plot(T,tire_defl_pass,'r',tt,tire_defl_lqr,'b'); ylabel('Tire Deflection (m)'); xlabel('Time (s)');
title('Tire Deflection'); legend('Passive','LQR Active'); grid on;

figure; plot(tt,F_lqr,'b'); ylabel('F_{em} (N)'); xlabel('Time (s)'); title('Actuator Force (LQR)'); grid on;

% METRICS
fprintf('RMS body accel: passive = %.3f m/s^2, LQR active = %.3f m/s^2 (%.1f%% reduction)\n',...
        rms(a_b_pass), rms(a_b_lqr), 100*(rms(a_b_pass)-rms(a_b_lqr))/rms(a_b_pass));
fprintf('Peak |a_b|: passive = %.3f, LQR active = %.3f\n', max(abs(a_b_pass)), max(abs(a_b_lqr)));
fprintf('Peak |z_b|: passive = %.4f m, LQR active = %.4f m\n', max(abs(z_b_pass)), max(abs(z_b_lqr)));

% ACTUATOR POWER
v_act = z_bdot_lqr - z_tdot_lqr;  
P_inst_W = F_lqr .* v_act;  
E_total = trapz(tt, max(P_inst_W,0));
P_avg_W = E_total / (tt(end) - tt(1));
P_avg_hp = P_avg_W / 745.7;

figure; plot(tt, P_inst_W,'b'); xlabel('Time (s)'); ylabel('Actuator Power (W)'); title('Actuator Power'); grid on;
fprintf('Total mechanical energy consumed by actuator: %.2f J\n', E_total);
fprintf('Average actuator power: %.2f W = %.4f hp\n', P_avg_W, P_avg_hp);
