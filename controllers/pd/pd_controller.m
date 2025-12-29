function F_em = pd_controller(x, zg, zgdot, Kpdef, Kddef, Kacc, Fm, C_acc, D_acc)
% Mixed-feedback PD + acceleration feedback

susp_defl = x(1) - x(2);
defl_dot  = x(3) - x(4);

a_b = C_acc*x + D_acc*[0; zg; zgdot];

F_raw = -Kpdef*susp_defl - Kddef*defl_dot - Kacc*a_b;
F_em  = max(min(F_raw,Fm),-Fm);
end

