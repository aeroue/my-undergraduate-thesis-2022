function phi = activateFun(x)
% phi = [0.5*x(1)*x(1);
%     0.5*x(2)*x(2);
%     0.5*x(3)*x(3);
%     0.5*x(4)*x(4);
%     0.5*x(5)*x(5);
%     0.5*x(6)*x(6);
%     x(1)*x(4);
%     x(2)*x(5);
%     x(3)*x(6)];
x1 = x(1);
x2 = x(2);
x3 = x(3);
x4 = x(4);
x5 = x(5);
x6 = x(6);

phi = [x1^2
x1*x2 
x1*x3 
x1*x4 
x1*x5 
x1*x6 
x2^2 
x2*x3 
x2*x4 
x2*x5 
x2*x6 
x3^2 
x3*x4 
x3*x5 
x3*x6
x4^2 
x4*x5 
x4*x6 
x5^2 
x5*x6 
x6^2 
x1^4
x2^4 
x3^4 
x4^4 
x5^4 
x6^4]';