function X_dot = state_equation2(t, X, Pevt, tP)

    global params
    
    A = params.A;
    B = params.B;
    Rp = params.Rp;
    Re = params.Re;
    siz = params.siz;
    t0 = params.t0;
    step = params.time_step;

    for i = 1:36
        Pp(i) = interp1(tP, Pevt(:,i), t);
    end
    
    P = reshape(Pp,siz);

    
    Ue = Re^(-1) * B'* P * X;
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
    