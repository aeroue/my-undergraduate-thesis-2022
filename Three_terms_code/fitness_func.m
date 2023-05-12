function  f = fitness_func(pop)
    k1 = 1;
    k2 = 1;
    k3 = 1;
    k4 = 10;
    global CONSTANTS
    omg = CONSTANTS.refOmega;
    X0 = CONSTANTS.X0;
    step = CONSTANTS.stepT;
    
    lmdf = zeros(6,1);
    lmdf(1) = pop(2);
    lmdf(2) = pop(3);
    lmdf(3) = pop(4);
    tf = pop(1);
    
    trans = lamdatransMatrix(omg,tf);
    lmd0 = inv(trans) * lmdf;
    
    if abs(det(trans)) < 1e-10
        f = 1e13;
        c = [];
        ceq = [];
        return;
    else
        tspan = 0: step: tf;
        options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
        [TSPAN, XSPAN] = ode45(@(t,X)CW(t, X,lmd0), tspan, X0, options);
        Xf = XSPAN(end,:);

    Hf = lmdf(1)*Xf(4) + lmdf(2)*Xf(5) + lmdf(3)*Xf(6);

    F = zeros(4,1);
    F(1) = Xf(1);
    F(2) = Xf(2);
    F(3) = Xf(3);
    F(4) = Hf + 1;
    
    k1 = 1; 
    k2 = 1; 
    k3 = 1;
    k4 = 10;


    f = k1 * F(1)^2 + k2 * F(2)^2 + k3 * F(3)^2 + k4 * F(4)^2;
end
    
    
    
    
    
    
    
        
        