function sim_atlas(model)

    q0 = zeros(model.NB, 1) ;
    dq0 = zeros(model.NB, 1) ;
    [q dq] = model.gamma_q(model, q0, dq0) ;
    x0 = [q; dq] ;

    [t_sol, x_sol] = ode45(@odefun, [0 1], x0, [], model) ;
end

function dx = odefun(t, x, model)
    q = x(1:model.NB) ;
    dq = x(model.NB+1 : end) ;
    tau = zeros(model.NB - 6, 1) ;
    dx = [dq; FDgq(model, q, dq, [zeros(6,1);tau])] ; 
end