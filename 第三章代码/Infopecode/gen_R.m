function R = gen_R(Rp, Re) 

% generate the R of the riccati inv(r£© = inv(rp) - inv(re)   

    r11 = Rp(1,1) * Re(1,1) / (Re(1,1) - Rp(1,1));
    r22 = Rp(2,2) * Re(2,2) / (Re(2,2) - Rp(2,2));
    r33 = Rp(3,3) * Re(3,3) / (Re(3,3) - Rp(3,3));  
    
    R = diag([r11 r22 r33]);
    
end