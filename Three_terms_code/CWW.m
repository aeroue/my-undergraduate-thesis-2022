function dX = CWW(t,X,lmd0,T)
    global CONSTANTS
    omg = CONSTANTS.refOmega;
    Mtx = lamdatransMatrix(omg,t);
    lmd = Mtx * lmd0;
    
    r = norm([lmd(4), lmd(5), lmd(6)]);
    if abs(r) < 1e-6
        alpha = 0;
        beta = 0;
    else
        calpha = lmd(4) / (norm([lmd(4), lmd(5)]));
        salpha = lmd(5) / (norm([lmd(4), lmd(5)]));
        alpha = atan2(salpha, calpha);
        beta = asin(lmd(6)/r);
    end
    
    dX = zeros(6,1);
    dX(1) = X(4);
    dX(2) = X(5);
    dX(3) = X(6);
    dX(4) = 2*omg*X(5) + 3*omg*omg*X(1) + (T) * cos(alpha)*cos(beta);
    dX(5) = -2*omg*X(4) + (T) * sin(alpha)*cos(beta);
    dX(6) = -omg*omg*X(3) + (T) * sin(beta);
end