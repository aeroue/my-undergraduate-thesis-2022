function [x, y, z, vx, vy, vz] = gen_x_v_u(t_int, X)
len = length(t_int);
x = zeros(len,1);
y = zeros(len,1);
z = zeros(len,1);
vx = zeros(len,1);
vy = zeros(len,1);
vz = zeros(len,1);
for i = 1 : len
    x(i) = X(1,i)/1000; y(i) = X(2,i)/1000; z(i) = X(3,i)/1000;
    vx(i) = X(4,i); vy(i) = X(5,i); vz(i) = X(6,i);
end
end