function dx = state_deriv_pd(t,x,A,B,Tvec,zg_vec,zgdot_vec,Kpdef,Kddef,Kacc,Fm,C_acc,D_acc)

[~, idx] = min(abs(Tvec - t));
zg    = zg_vec(idx);
zgdot = zgdot_vec(idx);

F_em = pd_controller(x, zg, zgdot, Kpdef, Kddef, Kacc, Fm, C_acc, D_acc);

u  = [F_em; zg; zgdot];
dx = A*x + B*u;
end
