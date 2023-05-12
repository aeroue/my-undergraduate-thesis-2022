function result = get_infinite_planning_horizon_result(Number_of_iterations,choice,a)
    global params
    A = params.A;
    B = params.B;
    Re = params.Re;
    Rp = params.Rp;
    Q = params.Qf;
    
    t0 = params.t0;
    tf = 2 * params.tf; 
    step = 2 * params.time_step;
    
    tau_int = t0:step:tf;
    num = numel(tau_int);
    
    X0 = params.X0(:,choice);
    Eva0 = params.Eva0(:,a);
    %Pur0 = params.Pur0(a,:);
    
    options = odeset('AbsTol', 1e-12, 'RelTol', 1e-8);
    
    P = get_Algebraic_Riccati_equation_results(Number_of_iterations);
    
    [t,X] = ode45(@(t,X)non_state_equation(t, X, P), tau_int, X0, options);
    
    [Ue, Up ,J] = gen_cost_and_control(X,P,1);

    [Xe, Xp, Ve, Vp] = gen_x_and_v(Ue,X,tau_int,step,Eva0);

    result = make_result(X, Xe, Xp, Ve, Vp, Ue, Up, J, tau_int);
end
    
    
    