function dx = state_deriv_PID(t,x,A,B,Tvec,zg_vec,zgdot_vec,Kpdef,Kddef,Ki,Kacc,Fm,C_acc,D_acc)
    % x = [z_b; z_t; z_bdot; z_tdot; integral_defl]

    % Find closest index in time vector
    [~, idx] = min(abs(Tvec - t));
    zg = zg_vec(idx); 
    zgdot = zgdot_vec(idx);

    % Suspension deflection and rate
    susp_defl = x(1) - x(2);
    defl_dot  = x(3) - x(4);

    % Integral of deflection
    int_defl = x(5);

    % Approximate body acceleration
    a_b = C_acc*x(1:4) + D_acc*[0; zg; zgdot];

    % PID + acceleration feedback
    F_raw = -Kpdef*susp_defl - Kddef*defl_dot - Ki*int_defl - Kacc*a_b;

    % Saturate actuator
    F_em = max(min(F_raw,Fm),-Fm);

    % System input
    u = [F_em; zg; zgdot];

    % State derivative
    dx_main = A*x(1:4) + B*u;

    % Integral derivative
    dx_int = susp_defl;

    % Combine derivatives
    dx = [dx_main; dx_int];
end

