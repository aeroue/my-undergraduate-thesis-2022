function cir_point = polar2cartesian(rr,theta,phi)
    x=rr.*sin(phi).*cos(theta);
    y=rr.*sin(phi).*sin(theta);
    z=rr.*cos(phi);
    
    cir_point = [x;y;z];
end