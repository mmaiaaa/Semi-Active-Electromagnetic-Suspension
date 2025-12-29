x0 = zeros(5,1);  % [z_b; z_t; z_bdot; z_tdot; integral_defl]

%% ACTIVE SYSTEM
odefun = @(t,x) state_deriv_PID(t,x,A,B,T,z_g,z_gdot,Kpdef,Kddef,Ki,Kacc,F_max,C_acc,D_acc);
[tt,XX] = ode45(odefun,T,x0);

z_b_act    = XX(:,1); 
z_t_act    = XX(:,2);
z_bdot_act = XX(:,3); 
z_tdot_act = XX(:,4);
defl_act   = z_b_act - z_t_act;
tire_defl_act = z_t_act - interp1(T,z_g,tt);

%% ACTIVE VAR
F_hist = zeros(length(tt),1);
a_b_act = zeros(length(tt),1);
for i=1:length(tt)
    susp_defl = XX(i,1) - XX(i,2);
    defl_dot  = XX(i,3) - XX(i,4);
    int_defl  = XX(i,5);
    a_b = C_acc*XX(i,1:4)' + D_acc*[0; z_g(i); z_gdot(i)];
    F_raw = -Kpdef*susp_defl - Kddef*defl_dot - Ki*int_defl - Kacc*a_b;
    F_hist(i) = max(min(F_raw,F_max),-F_max);
    a_b_act(i) = a_b;
end
