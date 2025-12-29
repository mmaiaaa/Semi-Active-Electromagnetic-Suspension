function [F_lqr, a_b_lqr] = lqr_controller(XX_aug, T, z_g, z_gdot, K_lqr_aug, F_max, C_acc, D_acc)
    n = length(T);
    F_lqr = zeros(n,1);
    a_b_lqr = zeros(n,1);

    for i=1:n
        x_aug = XX_aug(i,:)';
        a_b = C_acc*x_aug(1:4) + D_acc*[0; z_g(i); z_gdot(i)];
        F_raw = -K_lqr_aug*x_aug - 0.5*a_b;
        F_lqr(i) = max(min(F_raw, F_max), -F_max);
        a_b_lqr(i) = a_b;
    end
end

