function dX = TDACW(t, X, P1, B, R1, R2)
% 构造CW方程

% 载入常数
global CONSTANTS
omg = CONSTANTS.refOmega;

% CW方程，控制量直接为推力加速度，采用反馈控制律获得
ue = -inv(R1) * B' * P1 * X;
up = -inv(R2) * B' * P1 * X;

% 状态微分方程
dX = zeros(6,1);
dX(1) = X(4);
dX(2) = X(5);
dX(3) = X(6);
dX(4) = 2*omg*X(5) + 3*omg*omg*X(1) + up(1) - ue(1);
dX(5) = -2*omg*X(4) + up(2) - ue(2);
dX(6) = -omg*omg*X(3) + up(3) - ue(3);

end