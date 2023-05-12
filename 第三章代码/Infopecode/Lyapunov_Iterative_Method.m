function P = Lyapunov_Iterative_Method(Number_of_iterations)
    global params
    A = params.A;
    B = -params.B;
    Re = params.Re;
    Rp = params.Rp;
    R = params.R;
    Q = params.Qf;
    
    Sp = B * Rp^(-1) * B';
    Se = B * Re^(-1) * B';
    P = care(A , B, Q, Rp);
    
    N = Number_of_iterations;
    
    for i=1:1:N
        A1 = A - Sp * P;
        Q1 = Q + P * Sp * P + P * Se * P;
        P = lyap(A1', Q1);
    end
    
end