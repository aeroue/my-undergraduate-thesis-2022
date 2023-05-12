function xk = xnolinear(x, u)
% 航天器非线性相对运动方程 欧拉离散形式
% wuyi
global CONSTANT
omega = CONSTANT.omega;
gravParam = CONSTANT.gravParam;
a = CONSTANT.Radius;
T = 0.2;
sigma = ((a + x(1)).^2 + x(2).^2 +x(3).^2).^(3/2);
ux  = u(1);
uy  = u(2);
uz  = u(3);
dotx = zeros(6,1);
dotx(1) = x(4);
dotx(2) = x(5);
dotx(3) = x(6);
dotx(4) = 2 * omega * x(5) + omega.^2 * (x(1) + a) ...
    - gravParam * (a + x(1))/sigma + gravParam / a.^2 + ux;
dotx(5) = -2 * omega * x(4) + omega.^2 * x(2) ...
    - gravParam * x(2)/((a + x(1)).^2 + x(2).^2 +x(3).^2).^(3/2) + uy;
dotx(6) = - gravParam * x(3)/((a + x(1)).^2 + x(2).^2 +x(3).^2).^(3/2) + uz;

xk(1) = x(1) + T*dotx(1);
xk(2) = x(2) + T*dotx(2);
xk(3) = x(3) + T*dotx(3);
xk(4) = x(4) + T*dotx(4);
xk(5) = x(5) + T*dotx(5);
xk(6) = x(6) + T*dotx(6);
xk = xk';
end