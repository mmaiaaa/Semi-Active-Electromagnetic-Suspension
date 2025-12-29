function [F_hist, a_b_act] = pid_controller(XX, T, z_g, z_gdot, Kpdef, Kddef, Ki, Kacc, F_max, C_acc, D_acc)
    N = size(XX,1);
    F_hist = zeros(N,1);
    a_b_act = zeros(N,1);

    for i = 1:N
        susp_defl = XX(i,1) - XX(i,2);
        defl_dot  = XX(i,3) - XX(i,4);
        int_defl  = XX(i,5);

        a_b = C_acc*XX(i,1:4)' + D_acc*[0; z_g(i); z_gdot(i)];
        F_raw = -Kpdef*susp_defl - Kddef*defl_dot - Ki*int_defl - Kacc*a_b;

        F_hist(i) = max(min(F_raw, F_max), -F_max);
        a_b_act(i) = a_b;
    end
end
