function j = qiudao(t)
syms tttt
XX0=[-16400;-60300;7200;1.5;4.2;0];
w = 7.292124577897024e-05;
tauu = w*tttt;
s = sin(tauu);
c = cos(tauu);
phi = [4-3*c 0 0 s/w -2*(c-1)/w 0;
    6*(s-tauu) 1 0 2*(c-1)/w (4*s-3*tauu)/w 0;
    0 0 c 0 0 s/w;
    3*w*s 0 0 c 2*s 0;
    6*w*(c-1) 0 0 -2*s -3+4*c 0;
    0 0 -w*s 0 0 c];
phi = eval(phi(1:3,:));
dtf1 = phi*XX0;
dtf1 = vpa(diff(dtf1,tttt));
tttt = t;
j = eval(dtf1)
end
%j24 = subs(j24,tf,0)

% 
% tau = 0;
% s = sin(tau);
% c = cos(tau);
% phi2 = [s/w -2*(c-1)/w 0;
%     2*(c-1)/w (4*s-3*tau)/w 0;
%     0 0 s/w;
%     c 2*s 0;
%     -2*s -3+4*c 0;
%     0 0 c];
% g = 9.78;
% dtf2 =phi2*(0.0004*g-0.0006*g);
% dtf2 = dtf2(1:3,:)
