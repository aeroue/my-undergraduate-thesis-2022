function [result,Pevt] = get_fix_duration_horizon_result(choice,a)
    global params
    A = params.A;
    B = params.B;
    Re = params.Re;
    Rp = params.Rp;
    Pf = params.Qf;
    
    t0 = params.t0;
    tf = params.tf; 
    step = params.time_step;
    
    tau_int = t0:step:tf;  
    num = numel(tau_int);
    
    X0 = params.X0(:,choice);
    Eva0 = params.Eva0(:,a);
    
    options = odeset('AbsTol', 1e-12, 'RelTol', 1e-8);
    %options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
    
    [tp,Ptau] = ode45(@martix_RDE,tau_int, Pf, options);

    Pevt = Ptau(end:-1:1,:);
    
    [t,X] = ode45(@(t,X)non_state_equation2(t, X, Pevt, tp), tau_int, X0, options);
    
    [Ue, Up ,J] = gen_cost_and_control(X,Pevt,2)

    [Xe, Xp, Ve, Vp] = gen_x_and_v(Ue,X,tau_int,step, Eva0);

    result = make_result(X, Xe, Xp, Ve, Vp, Ue, Up, J, tau_int);
end
    
    
    