function dx = state_deriv_pid(t,x,A,B,Tvec,zg_vec,zgdot_vec,Kpdef,Kddef,Ki,Kacc,Fm,C_acc,D_acc)

[~, idx] = min(abs(Tvec - t));
zg    = zg_vec(idx);
zgdot = zgdot_vec(idx);

F_em = pid_controller(x, zg, zgdot, Kpdef, Kddef, Ki, Kacc, Fm, C_acc, D_acc);

u = [F_em; zg; zgdot];

dx_main = A*x(1:4) + B*u;
dx_int  = x(1) - x(2);

dx = [dx_main; dx_int];
end
