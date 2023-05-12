% 生存型微分对策精确值求解
% renew
clear all
clc

%% 参数
% 基本参数
Re                  =   6378.137 * 1000;                
miu                 =   3.986004418E14;               
Href                =   2000 * 1000;                     
Rref                =   Re + Href;       
omega           =   sqrt(miu / Rref^3); 
% 初始参数
Xe = [4.5*1000; 15*1000; -2*1000; 3; -7.5; 0];
Xp = [0; 0; 0; 0; 0; 0];
g = 9.78;
TpMax = 0.0006 * g;
TeMax = 0.0004 * g;
X0 = Xe - Xp;
step = 10;

%% 常数参数
global CONSTANTS
CONSTANTS.refOmega  =   omega;
CONSTANTS.TpMax = TpMax;
CONSTANTS.TeMax = TeMax;
CONSTANTS.stepT     =   step;
CONSTANTS.X0 = X0;
CONSTANTS.X0(1:3) = X0(1:3);
CONSTANTS.X0(4:6) = X0(4:6);
CONSTANTS.Xp = Xp;
CONSTANTS.Xp(1:3) = Xp(1:3);
CONSTANTS.Xp(4:6) = Xp(4:6);
CONSTANTS.Xe = Xe;
CONSTANTS.Xe(1:3) = Xe(1:3);
CONSTANTS.Xe(4:6) = Xe(4:6);

A11 = zeros(3,3);
A12 = eye(3,3);
A21 = [ 3*omega^2    0       0;
             0       0       0;
             0       0       -omega^2];
A22 = [  0       2*omega     0;
        -2*omega     0       0;
             0       0       0 ];
CONSTANTS.A = [ A11      A12;
    A21      A22];
B11 = zeros(3,3);
B21 = eye(3,3);
CONSTANTS.B = [ B11;
    B21];

%% PSO求解粗略值
% 设置种群参数（此处）
dim = 4;        %变量个数
popmax = [10000,1,1,1];         %粒子最大位置
popmin = [1000,-1,-1,-1];       %粒子最小位置
vmax = [10,0.01,0.01,0.01];              %粒子最大运动速度
vmin = -[10,0.01,0.01,0.01];             %粒子最小运动速度
optPSO.npop = dim*50;         
optPSO.niter = 200;
optPSO.Plot = 1;
optPSO.c1 = 2; %认知学习因子
optPSO.c2 = 2; %社会学习因子
% 得粗略值
%[yopt, fval] = myPso(@fitness_func,dim,popmin,popmax,vmax,vmin,optPSO);

%% 拟牛顿法求精确值
y = [1809.38644367395;0.102635537813599;0.0286604207557571;-0.00950301199190852];

%% 相对状态求解
[F, Xf, XSPAN] = endGame(y);

%% 控制律重构
omg = CONSTANTS.refOmega;
tf = y(1);
lbd_f = zeros(6,1);
lbd_f(1) = y(2);
lbd_f(2) = y(3);
lbd_f(3) = y(4);
PhiMtx_t02tf = lamdatransMatrix(omg, tf);
lmd0 = inv(PhiMtx_t02tf) * lbd_f;    
Tp = CONSTANTS.TpMax;
Te = CONSTANTS.TeMax;
tspan = 0: CONSTANTS.stepT: tf;
num = numel(tspan);
dX = zeros(6,num);

for i = 1:num
    t = tspan(i);
    Mtx = lamdatransMatrix(omg,t);
    lmd = Mtx * lmd0;   
    r = norm([lmd(4), lmd(5), lmd(6)]);
    calpha = lmd(4) / (norm([lmd(4), lmd(5)]));
    salpha = lmd(5) / (norm([lmd(4), lmd(5)]));
    sbeta = lmd(6)/r;
    cbeta = sqrt(lmd(4)^2+lmd(5)^2)/r;
    alpha(i) =atan2(salpha, calpha)/pi*180;
    beta(i) = atan2(sbeta,cbeta)*180/pi;
end


%% 相对距离、相对速度大小
Xf = XSPAN';
DR = tspan;
DV = tspan;
for i = 1 : num
    DR(i) = norm(Xf(1:3,i))/1000;
    DV(i) = norm(Xf(4:6,i));
end

%% 状态重构
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[tSpan, Xp] = ode45(@(t,X)CWW(t, X, lmd0 ,TpMax), tspan, CONSTANTS.Xp, options);
%[tSpan, Xe] = ode45(@(t,X)CW(t, X, lmd0), tspan, CONSTANTS.Xe, options);
Xe = Xp + XSPAN;

%% 整理数据
Xp = Xp';
Xe = Xe';
[xp, yp, zp, vxp, vyp, vzp] = gen_x_v_u(tspan, Xp);
[xe, ye, ze, vxe, vye, vze] = gen_x_v_u(tspan, Xe);

%% 画图
t_int = tspan;
close all
figure
plot(t_int,Xf(1,:)/1000,'r','linewidth',2);
hold on
plot(t_int,Xf(2,:)/1000,'b--','linewidth',2);
hold on
plot(t_int,Xf(3,:)/1000,'k-.','linewidth',2);
hold on
%legend('相对距离')
xlabel('t/s');
ylabel('相对距离/km')
legend('x','y','z','Location','NorthEast')
grid on
box on
set(gca,'FontSize',14)

figure
plot(t_int,Xf(4,:),'r','linewidth',2);
hold on
plot(t_int,Xf(5,:),'b--','linewidth',2);
hold on
plot(t_int,Xf(6,:),'k-.','linewidth',2);
hold on
%legend('相对距离')
xlabel('t/s');
ylabel('相对速度(m/s)')
legend('x','y','z','Location','NorthEast')
grid on
box on
set(gca,'FontSize',14)

figure
plot3(xe,ye,ze,'r','linewidth',2)
hold on
plot3(xp,yp,zp,'b-.','linewidth',2)
hold on
plot3(xe(1),ye(1),ze(1),'s')
hold on
plot3(xp(1),yp(1),zp(1),'o')
quiver3(xe(1),ye(1),ze(1),xe(30)-xe(1),ye(30)-ye(1),ze(30)-ze(1),'Color',[1.0,0.0,0.0],'LineWidth',2.0,'Maxheadsize',17);
hold on
quiver3(xp(1),yp(1),zp(1),xp(40)-xp(1),yp(40)-yp(1),zp(40)-zp(1),...
    'Color',[0.0,0.0,1.0],'LineWidth',2.0,'Maxheadsize',17);
hold on
xlabel('x/km');
ylabel('y/km')
zlabel('z/km')
legend('追击者','防御者','追击者起始点','防御者起始点','Location','NorthEast')
grid on
box on
set(gca,'FontSize',14)
view(120,30)


figure
plot(t_int,DR,'r','linewidth',2);
hold on
%legend('相对距离')
xlabel('t/s');
ylabel('\deltaR/km')
grid on
box on
set(gca,'FontSize',14)

figure
plot(t_int,DV,'b','linewidth',2);
hold on
%legend('相对速度')
xlabel('t/s');
ylabel('\deltaV')
grid on
box on
set(gca,'FontSize',14)

% X
figure
plot(t_int,xp,'r-.','linewidth',1.5);
hold on
plot(t_int,xe,'b-','linewidth',1.5);
hold on
xlabel('t/s');
ylabel('X/km')
grid on
box on
legend('防御者','追击者')
set(gca,'FontSize',14)

% Y
figure
plot(t_int,yp,'r-.','linewidth',1.5);
hold on
plot(t_int,ye,'b-','linewidth',1.5);
hold on
xlabel('t/s');
ylabel('Y/km')
grid on
box on
legend('防御者','追击者')
set(gca,'FontSize',14)

% Z
figure
plot(t_int,zp,'r-.','linewidth',1.5);
hold on
plot(t_int,ze,'b-','linewidth',1.5);
hold on
xlabel('t/s');
ylabel('Z/km')
grid on
box on
legend('防御者','追击者')
set(gca,'FontSize',14)

% X
figure
plot(t_int,vxp,'r-.','linewidth',1.5);
hold on
plot(t_int,vxe,'b-','linewidth',1.5);
hold on
xlabel('t/s');
ylabel('V_x(m/s-2)')
grid on
box on
legend('防御者','追击者')
set(gca,'FontSize',14)

% Y
figure
plot(t_int,vyp,'r-.','linewidth',1.5);
hold on
plot(t_int,vye,'b-','linewidth',1.5);
hold on
xlabel('t/s');
ylabel('V_y(m/s-2)')
grid on
box on
legend('防御者','追击者')
set(gca,'FontSize',14)

% Z
figure
plot(t_int,vzp,'r-.','linewidth',1.5);
hold on
plot(t_int,vze,'b-','linewidth',1.5);
hold on
xlabel('t/s');
ylabel('V_z(m/s-2)')
grid on
box on
legend('防御者','追击者')
set(gca,'FontSize',14)

% U
figure
plot(t_int,alpha,'r','linewidth',1.5)
xlabel('t/s');
ylabel('\alpha/deg')
grid on
box on
set(gca,'FontSize',14)

figure
plot(t_int,beta,'r','linewidth',1.5)
xlabel('t/s');
ylabel('\beta/deg')
grid on
box on
set(gca,'FontSize',14)

h = playBox([xe,ye,ze],[xp,yp,zp],3);
movie(h,1,10);
    


