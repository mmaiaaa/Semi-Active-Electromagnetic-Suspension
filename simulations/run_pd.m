clc; close all;

addpath('../model');
addpath('../controllers/pd');

% SYSTEM
[A, B, ~, C_acc, D_acc] = state_space_quarter_car();
[T, z_g, z_gdot] = road_profile_half_sine();

% CONTROLLER PARAMETERS
Kpdef = 8000;   % suspension deflection proportional gain
Kddef = 6500;   % suspension damping derivative gain
Kacc  = 500;    % small acceleration feedback
F_max = 2000;   % actuator force limit

% INITIAL CONDITION
x0 = zeros(4,1);

% SIMULATION
odefun = @(t,x) state_deriv_pd(t,x,A,B,T,z_g,z_gdot,Kpdef,Kddef,Kacc,F_max,C_acc,D_acc);
[tt, XX] = ode45(odefun, T, x0);

% PD controller computation (after ODE)
[F_hist, a_b_act] = pd_controller(XX, T, z_g, z_gdot, Kpdef, Kddef, Kacc, F_max, C_acc, D_acc);

% ACTIVE VARS
z_b_act    = XX(:,1); 
z_t_act    = XX(:,2);
z_bdot_act = XX(:,3); 
z_tdot_act = XX(:,4);
defl_act   = z_b_act - z_t_act;
tire_defl_act = z_t_act - interp1(T, z_g, tt);

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
figure; plot(T,z_b_pass,'r',tt,z_b_act,'b'); ylabel('z_b (m)'); xlabel('Time (s)');
title('Body Displacement'); legend('Passive','Active'); grid on;

figure; plot(T,z_bdot_pass,'r',tt,z_bdot_act,'b'); ylabel('v_b (m/s)'); xlabel('Time (s)');
title('Body Velocity'); legend('Passive','Active'); grid on;

figure; plot(T,a_b_pass,'r',tt,a_b_act,'b'); ylabel('a_b (m/s^2)'); xlabel('Time (s)');
title('Body Acceleration'); legend('Passive','Active'); grid on;

figure; plot(T,defl_pass,'r',tt,defl_act,'b'); ylabel('Suspension Deflection (m)'); xlabel('Time (s)');
title('Suspension Deflection'); legend('Passive','Active'); grid on;

figure; plot(T,tire_defl_pass,'r',tt,tire_defl_act,'b'); ylabel('Tire Deflection (m)'); xlabel('Time (s)');
title('Tire Deflection'); legend('Passive','Active'); grid on;

figure; plot(tt,F_hist,'b'); ylabel('F_{em} (N)'); xlabel('Time (s)');
title('Actuator Force (Active)'); grid on;

% METRICS
fprintf('RMS body accel: passive = %.3f m/s^2, active = %.3f m/s^2 (%.1f%% reduction)\n',...
        rms(a_b_pass), rms(a_b_act), 100*(rms(a_b_pass)-rms(a_b_act))/rms(a_b_pass));
fprintf('Peak |a_b|: passive = %.3f, active = %.3f\n', max(abs(a_b_pass)), max(abs(a_b_act)));
fprintf('Peak |z_b|: passive = %.4f m, active = %.4f m\n', max(abs(z_b_pass)), max(abs(z_b_act)));

% ACTUATOR POWER
v_act = z_bdot_act - z_tdot_act;  
P_inst_W = F_hist .* v_act;  
E_total = trapz(tt, max(P_inst_W,0));  
P_avg_W = E_total / (tt(end) - tt(1));  
P_avg_hp = P_avg_W / 745.7;

figure; plot(tt, P_inst_W,'b'); xlabel('Time (s)'); ylabel('Actuator Power (W)');
title('Actuator Power'); grid on;

fprintf('Total energy consumed by actuator: %.2f J\n', E_total);
fprintf('Average actuator power: %.2f W = %.4f hp\n', P_avg_W, P_avg_hp);
