% @author:wuyi@gmail.com 
% ref:zhang.p
% 毕设:ADP-PI

%% CLEAN WORKSPACE
clc; 
close all; 
clear ; 

%% 参数配置
% 物理常数
global CONSTANT
earthRadius         = 6378.137 * 1000;                % 地球半径，m 
gravParam           = 3.986004418E14;                 % 地球引力常数，m3/s2
% 参考圆轨道参数
Height              = 2962 * 1000;                    % 参考轨道高度，m
Radius              = earthRadius + Height;           % 参考轨道半长轴，m
omega               = sqrt(gravParam / (Radius*Radius*Radius)); % 参考轨道角速度，rad/s
CONSTANT.omega      = omega;
CONSTANT.Radius     = Radius;
CONSTANT.gravParam  = gravParam;
% 追踪航天器与逃逸航天器初始轨道参数
Xe0 = [-4110.67053397794 -1996.33767225763 -2251.82355023388 5.40227842285007 ...
-1.52106912642709 0]';
Xp0 = [0; 0; 0; 0; 0; 0];
% 初始状态参数
X0 = Xp0 - Xe0;
xp(:,1) = Xp0;
xe(:,1) = Xe0;
x(:,1)  = X0;
% 仿真时间和积分步长
gameT               = 2*2*pi/omega;                   % s,一个绕飞周期
stepT               = 0.2;                            % s
N                   = gameT / stepT;
% 学习参数预置
B = zeros(6,3);
B(4,1) = 1;
B(5,2) = 1;
B(6,3) = 1;
Bp     = B;
Be     = -B;
Rp     = diag([78 32 53]);
Q      = 10*diag([58 15 20 200 300 200]);

W      = [0.899988149868883	5.84718619263320;
3.20941032647761	9.48108735396022;
5.11408938819178	0.610289291925092;
0.606063665682423	5.84641303355111;
7.25687923545844	2.85108085658642;
5.56555748561992	8.27732173448263;
5.29359902481257	1.90986440697398;
8.29982432033195	4.42529962202884;
8.58759034071804	3.93411506367576;
7.89028923313949	8.26573979042765;
3.17833053726229	6.76871093438419;
4.52207453762982	2.07603034379981;
7.52227970049942	3.18104726150263;
1.09861705750686	1.33810985356126;
1.09742368593904	6.71462889478031;
2.69883663704401	5.70991075462406;
5.24637345396311	1.69767066026489;
9.72651076977497	1.47655777151737;
7.10408685278170	4.76079718267456;
3.11859945147533	9.08102416506950;
2.91457127647727	5.52175026715835;
8.50357337374621	0.329398927498766;
9.11647424007853	0.538629264355561;
6.39276147276064	8.05063228558902;
2.55370297944443	4.51374854703448;
0.886658400322831	3.82646229559959;
8.38255587537226	7.89643703689691];
% 可自己随机生成一些初始值
a     = 0.2;
gamma = 9.585705312592099;
mabar = 60;

%% LOOP
%while eps >1e-4
N = 6500;
for k = 1:N
    j = 1;
    dphi            = dactivateFun(x(:,k));
    upbar(:,j)      = -0.5 * inv(Rp) * Bp' * dphi' * W(:,j);
    uebar(:,j)      = 0.5 * gamma^(-2)*inv(Rp) * Be' * dphi' * W(:,j);
    x_rpe           = xnolinear(xp(:,k),upbar(:,j)) - xnolinear(xe(:,k),uebar(:,j));
    vtheta(:,j)     = dphi * (x_rpe - x(:,k));
    WW              = W;
    for j = 2:200
        upbar(:,j)  = -0.5 * inv(Rp) * Bp' * dphi' * WW(:,j);
        uebar(:,j)  = 0.5 * gamma^(-2) * inv(Rp) * Be' * dphi' * WW(:,j);
        x_rpe       = xnolinear(xp(:,k),upbar(:,j)) - xnolinear(xe(:,k),uebar(:,j));
        vtheta(:,j) = dphi * (x_rpe - x(:,k));
        EV(j)       = x(:,k)'*Q*x(:,k) + upbar(:,j-1)'*Rp*upbar(:,j-1) - gamma^2*uebar(:,j-1)'*Rp*uebar(:,j-1);
        EtaHam      = EV(j) + WW(:,j)'*vtheta(:,j-1);
        dWW         = EtaHam * vtheta(:,j-1) / (vtheta(:,j-1)'*vtheta(:,j-1) + 1);
        WW(:,j+1)   = WW(:,j) - a * dWW;
     end    
    up_h(:,k)     =mabar * tanh(upbar(:,end)/mabar);
    ue_h(:,k)     =mabar * tanh(uebar(:,end)/mabar);

    xp(:,k+1)  = xnolinear(xp(:,k), up_h(:,k));
    xe(:,k+1)  = xnolinear(xe(:,k), ue_h(:,k));
    x(:,k+1)   = xp(:,k+1) - xe(:,k+1);
end

Int = 150;
t = (1:1:length(x(1,:))).*stepT;

figure(1)
plot(t,x(1,:),'b-.','linewidth',2)
hold on
plot(t,x(2,:),'r--','linewidth',2)
hold on
plot(t,x(3,:),'k','linewidth',2)
xlabel('time(s) ');
ylabel('Relative Distance(m) ');
defualtAxes;
grid on
legend('${X}_{1}$ ','${X}_{2}$ ','${X}_{3}$','Interpreter','latex','FontSize',18);


figure(2)
plot(t,x(4,:),'b-.','linewidth',2)
hold on
plot(t,x(5,:),'r--','linewidth',2)
hold on
plot(t,x(6,:),'k','linewidth',2)
xlabel('time(s) ');
ylabel('Relative Velocity(m/s) ');
defualtAxes;
grid on
legend('${X}_{4}$ ','${X}_{5}$ ','${X}_{6}$','Interpreter','latex','FontSize',18);

figure(3)
plot(t(1:end-1),up_h(1,:),'b-.','linewidth',2)
hold on
plot(t(1:end-1),up_h(2,:),'r--','linewidth',2)
hold on
plot(t(1:end-1),up_h(3,:),'k','linewidth',2)
grid on
defualtAxes;