function dx = state_deriv_pd(t,x,A,B,Tvec,zg_vec,zgdot_vec,Kpdef,Kddef,Kacc,Fm,C_acc,D_acc)
% Mixed-feedback PD + small acceleration

    % Find closest index in time vector
    [~, idx] = min(abs(Tvec - t));
    zg = zg_vec(idx); 
    zgdot = zgdot_vec(idx);

    % Suspension deflection and rate
    susp_defl = x(1) - x(2);
    defl_dot  = x(3) - x(4);

    % Approximate body acceleration
    a_b = C_acc*x + D_acc*[0; zg; zgdot];

    % Mixed PD + acceleration feedback
    F_raw = -Kpdef*susp_defl - Kddef*defl_dot - Kacc*a_b;

    % Saturate actuator
    F_em = max(min(F_raw,Fm),-Fm);

    % System input
    u = [F_em; zg; zgdot];

    % State derivative
    dx = A*x + B*u;
end
