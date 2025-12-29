function F_em = pid_controller(x, zg, zgdot, Kpdef, Kddef, Ki, Kacc, Fm, C_acc, D_acc)

susp_defl = x(1) - x(2);
defl_dot  = x(3) - x(4);
int_defl  = x(5);

a_b = C_acc*x(1:4) + D_acc*[0; zg; zgdot];

F_raw = -Kpdef*susp_defl - Kddef*defl_dot - Ki*int_defl - Kacc*a_b;

F_em = max(min(F_raw,Fm),-Fm);
end
