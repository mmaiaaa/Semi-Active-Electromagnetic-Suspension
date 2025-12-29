function dx = state_deriv_lqr(t, x_aug, A, B_act, Tvec, zg_vec, zgdot_vec, K_lqr_aug, Fm, C_acc, D_acc, k_t, c_t, m_t)
    % x_aug = [z_b; z_t; z_bdot; z_tdot; int_defl]

    % Find closest index in time vector
    [~, idx] = min(abs(Tvec - t));
    zg = zg_vec(idx);
    zgdot = zgdot_vec(idx);

    % Body acceleration for feedforward
    a_b = C_acc*x_aug(1:4) + D_acc*[0; zg; zgdot];

    % LQR control (actuator)
    F_raw = -K_lqr_aug*x_aug - 0.5*a_b;  % include acceleration feedback
    F_em = max(min(F_raw,Fm),-Fm);

    % Actuator effect
    dx_main = A*x_aug(1:4) + B_act*F_em;

    % Integral derivative
    dx_int = x_aug(1) - x_aug(2);

    % Road feedforward
    dx_road = [0;0;0; k_t/m_t*zg + c_t/m_t*zgdot];

    % Combine derivatives
    dx = [dx_main + dx_road; dx_int];
end
