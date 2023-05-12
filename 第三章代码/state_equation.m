function X_dot = state_equation(t, X, P)

    global params
    
    A = params.A;
    B = params.B;
    Rp = params.Rp;
    Re = params.Re;
    t0 = params.t0;
    
    %Ue = [0;0;0];
    Ue = Re^(-1) * B'* P * X;
    %Ue = [cos(a*pi/100);cos(a*pi/100);cos(a*pi/100)];
    Up = Rp^(-1) * B'* P * X;
         
    for i = 1:3
        
        if Up(i)>=5
            Up(i)=5;
        else if Up(i)<=-5
                Up(i)=-5;
            end
        end
        
        if Ue(i)>=5
            Ue(i)=5;
        else if Ue(i)<=-5
                Ue(i)=-5;
            end
        end
    end
    
    U = Ue - Up;
    
    X_dot = A * X + B * U;

end
    