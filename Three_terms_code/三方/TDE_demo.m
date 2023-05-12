% ----------------- 尝试写一下航天器三方博弈，主要借鉴faruqi的书及代码
% 吴怡
% 2022.05.03
% ----------------- Target - Defender - Attacker -----------------

clear
clc
close all

%% 参数
% 基本参数
Re                  =   6378.137 * 1000;                
miu                 =   3.986004418E14;               
Href                =   2000 * 1000;                    
Rref                =   Re + Href;       
Omegaref            =   sqrt(miu / Rref^3); 
% 初始参数
Xa = [-0*1000; 0*1000; 30*1000; 2; 0; 0];
%Xa = [-4.5*1000; -15*1000; 2*1000; -3; 7.5; 0];
Xt = [12*1000; 16*1000; 0; 0; 0; 0];
%Xt = [0*1000; 0*1000; 0; 0; 0; 0];
%Xd = [1*1000; 20*1000; 1*1000; 0; 0; 0];
Xd = [6*1000; 8*1000; 10*1000; 0; 0; 0];
X310 = Xt - Xa;
X230 = Xa - Xd;
g = 9.78;
TpMax = 0.0006 * g;
TeMax = 0.0004 * g;
%X0 = Xe - Xp;
step = 1;
i_num = 3;
j_num = 3;
gameT = 3000%5*60

%% 常数参数
global CONSTANTS                           
CONSTANTS.refOmega  =   Omegaref;
CONSTANTS.TpMax = TpMax;
CONSTANTS.TeMax = TeMax;
CONSTANTS.gameT =  gameT;
CONSTANTS.stepT =   step;
CONSTANTS.X310 = X310;
CONSTANTS.X310(1:3) = X310(1:3);
CONSTANTS.X310(4:6) = X310(4:6);
CONSTANTS.X230 = X230;
CONSTANTS.X230(1:3) = X230(1:3);
CONSTANTS.X230(4:6) = X230(4:6);
CONSTANTS.Xa = Xa;
CONSTANTS.Xra = Xa(1:3);
CONSTANTS.Xva = Xa(4:6);
CONSTANTS.Xt = Xt;
CONSTANTS.Xrt = Xt(1:3);
CONSTANTS.Xvt = Xt(4:6);
CONSTANTS.Xd = Xd;
CONSTANTS.Xrd = Xd(1:3);
CONSTANTS.Xvd = Xd(4:6);
CONSTANTS.i_num = i_num;
CONSTANTS.j_num = j_num;

% 仿真参数
t0 = 0;
tf = gameT; 
t_int = t0:step:tf;
T_len=length(t_int);

%% 创建交战运动学模型
% 系数矩阵
A = zeros(6,6);
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;
A(4,1) = 3*CONSTANTS.refOmega*CONSTANTS.refOmega;
A(4,5) = 2*CONSTANTS.refOmega;
A(5,4) = -2*CONSTANTS.refOmega;
A(6,3) = -CONSTANTS.refOmega*CONSTANTS.refOmega;
B = zeros(6,3);
B(4,1) = 1;
B(5,2) = 1;
B(6,3) = 1;

%% 微分对策支付函数中的几个矩阵配置
% 状态量相应矩阵Q，可以分为两种情况，一种只考虑两航天器距离，一种同时还考虑速度要求
Q_T = zeros(6,6);
Q_T(1,1) = 1e-14;
Q_T(2,2) = 1e-14;
Q_T(3,3) = 1e-14;
%Q_T = eye(6);        % 这里来控制是否还需要速度达到一致
Q = Q_T;
% 控制量相应矩阵
R3p = 1e-5*eye(3);%0.00011 * eye(3);
R1e = 1.9e-4*sqrt(2) * eye(3);%0.0001 * eye(3);
R3e = 1e-4*sqrt(2) * eye(3);%0.0001 * eye(3);
R2p = 0.5*1e-5*eye(3);%0.00011 * eye(3);

% R3p = 0.00011 * eye(3);
% R2p = 0.0011 * eye(3);
% R1e = 0.01 * eye(3);
% R3e = 0.1 * eye(3);
% R3p = 1e-5;
% R2p = 1e-5;
% % a = 1e5;%2400
% % b = 1.9*1e5
% % a = 1e5;
% % b = 1.5
% a = 1.6;
% b = 1e5;
% R3e = b * R2p;
% R1e = a * R3p;
%% 矩阵黎卡提方程求解
% 计算一些中间矩阵
S1 = B * inv(R3p) * B';
S2 = B * inv(R1e) * B';
S11 = B * inv(R2p) * B';
S22 = B * inv(R3e) * B';
% 求解黎卡提微分方程
P0 = Q_T;
% 用ode45求解
tspan = 0: CONSTANTS.stepT: CONSTANTS.gameT;
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[TSPAN1, PSPAN1] = ode45(@(t,P)mRiccati(t, P, A, S1-S2, Q), tspan, P0, options);
[TSPAN2, PSPAN2] = ode45(@(t,P)mRiccati(t, P, A, S11-S22, Q), tspan, P0, options);
% 黎卡提方程的解P(t)
Pevt1 = PSPAN1(end:-1:1,:);
P1 =cell(size(PSPAN1,1),1);
for i = 1 : size(PSPAN1,1)
    P1{i} = reshape(Pevt1(i,:), size(A));
end

Pevt2 = PSPAN2(end:-1:1,:);
P2 =cell(size(PSPAN2,1),1);
for i = 1 : size(PSPAN2,1)
    P2{i} = reshape(Pevt2(i,:), size(A));
end
%% 对策状态轨迹和追逃控制量求解，可以直接采用状态转移矩阵的方法求解，也可用下面方法
% 追踪者与逃跑者
[t1,X31] = ode45(@(t,X) TDACW(t, X, TSPAN1, Pevt1, B, R1e, R3p), t_int, X310, options);
[t2,X23] = ode45(@(t,X) TDACW(t, X, TSPAN1, Pevt1, B, R3e, R2p), t_int, X230, options);
X31 = X31';
X23 = X23';
% 对追逃控制量求解
U3p = zeros(3,T_len);
U1e = zeros(3,T_len);
K311 = P1;
K312 = P1;

U2p = zeros(3,T_len);
U3e = zeros(3,T_len);
K231 = P1;
K232 = P1;
for i = 1 : T_len
    K311{i} = - inv(R3p) * B' * P1{i};
    K312{i} = - inv(R1e) * B' * P1{i};
    U3p(:,i) = - K311{i} * X31(:,i);
    U1e(:,i) = - K312{i} * X31(:,i);
    
    K231{i} = - inv(R2p) * B' * P2{i};
    K232{i} = - inv(R3e) * B' * P2{i};
    U2p(:,i) = - K231{i} * X23(:,i);
    U3e(:,i) = - K232{i} * X23(:,i);
end

% 三方控制量
UA = U3p + U3e;
UT = U1e;
UD = U2p;

%% 对追踪器和逃逸器的状态量分别进行求解
% 需要根据上面获得的对策控制量进行积分获取
% 积分获得两航天器的状态轨迹
aAxVec = t_int; aAyVec = t_int; aAzVec = t_int; 
aTxVec = t_int; aTyVec = t_int; aTzVec = t_int;
aDxVec = t_int; aDyVec = t_int; aDzVec = t_int;
for i = 1 : T_len
    aAxVec(i) = UA(1,i); aAyVec(i) = UA(2,i); aAzVec(i) = UA(3,i);
    aTxVec(i) = UT(1,i); aTyVec(i) = UT(2,i); aTzVec(i) = UT(3,i);
    aDxVec(i) = UD(1,i); aDyVec(i) = UD(2,i); aDzVec(i) = UD(3,i);
end
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[tASpan, Xa] = ode45(@(t,X)CW(t, X, t_int, aAxVec, aAyVec, aAzVec), t_int, CONSTANTS.Xa, options);
[tTSpan, Xt] = ode45(@(t,X)CW(t, X, t_int, aTxVec, aTyVec, aTzVec), t_int, CONSTANTS.Xt, options);
[tDSpan, Xd] = ode45(@(t,X)CW(t, X, t_int, aDxVec, aDyVec, aDzVec), t_int, CONSTANTS.Xd, options);
Xa = Xa';
%Xt = Xa + X31;
%Xd = Xa - X23;
Xt = Xt';
Xd = Xd';
X31 = Xt - Xa;
X23 = Xa - Xd;

%% 相对距离、相对速度大小
DR31 = TSPAN1;
DV31 = TSPAN1;
for i = 1 : T_len
    DR31(i) = norm(X31(1:3,i))/1000;
    DV31(i) = norm(X31(4:6,i));
end

DR23 = TSPAN2;
DV23 = TSPAN2;
for i = 1 : T_len
    DR23(i) = norm(X23(1:3,i))/1000;
    DV23(i) = norm(X23(4:6,i));
end

%% 得到对策支付函数关于时间的变化规律
J1 = t_int;
J_C1 = t_int;
for i = 1 : T_len
    % 每个时刻瞬时对应的待积分支付函数
    J_C1(i) = X31(:,i)'*Q*X31(:,i) + U3p(:,i)'*R3p*U3p(:,i) - U1e(:,i)'*R1e*U1e(:,i);
end
J1(1) = X31(:,1)'*Q_T*X31(:,1);
for i = 2 : T_len
    % 采用数值积分的形式
    t_temp = t_int(1:i);
    J_temp = J_C1(1:i);
    J1(i) = trapz(t_temp, J_temp);
    J1(i) = J1(i) + X31(:,i)'*Q*X31(:,i);       % 进一步加上末值型支付函数
end

%% 整理结果
[x31, y31, z31, vx31, vy31, vz31, ~, ~, ~] = gen_x_v_u(t_int, X31, U3p);
[x23, y23, z23, vx23, vy23, vz23, ~, ~, ~] = gen_x_v_u(t_int, X23, U3e);
[xa, ya, za, vxa, vya, vza, uxa, uya, uza] = gen_x_v_u(t_int, Xa, UA);
[xd, yd, zd, vxd, vyd, vzd, uxd, uyd, uzd] = gen_x_v_u(t_int, Xd, UD);
[xt, yt, zt, vxt, vyt, vzt, uxt, uyt, uzt] = gen_x_v_u(t_int, Xt, UT);

%% 画图
% 处理数据
% 找到追踪者和目标、防御者之间的最小距离及时间
dismin23 = min(DR23)
dismin23index = find(DR23 == min(DR23))*step
dismin31 = min(DR31)
dismin31index = find(DR31 == min(DR31))*step
if (dismin23 < 0.03 && dismin31 > 0.03)
    fprintf('防御者成功拦截追踪者\n')
elseif (dismin23 < 0.5 && dismin31 < 0.5)
        if dismin23index < dismin31index
            fprintf('防御者成功拦截追踪者\n')
        else
            fprintf('追踪者成功击中目标\n')
        end
elseif (dismin23 > 0.5 && dismin31 < 0.5)
        fprintf('追踪者成功击中目标\n')
elseif (dismin23 > 0.5 && dismin31 < 0.5)
            fprintf('追踪者未成功击中目标，并逃脱\n')
end
      
for i = 1:10
    eval(['c',num2str(i),'=','getColor(i,1,11)']);
end 

% 相对距离、速度图
figure
plot(t_int,DR31,'r','linewidth',2);
hold on
plot(t_int,DR23,'b--','linewidth',2);
hold on
% plot(dismin23index*ones(1,T_len),linspace(0,1.1*max(DR23),T_len),'Color',c4)
% hold on
% plot(dismin31index*ones(1,T_len),linspace(0,1.1*max(DR23),T_len),'Color',c8)
legend('追踪者-目标','追踪者-防御者')
xlabel('t/s');
ylabel('\deltaR/km')
grid on
box on
set(gca,'FontSize',14)

figure
plot(t_int,DV31,'r','linewidth',2);
hold on
plot(t_int,DV23,'b--','linewidth',2);
% hold on
% plot(dismin23index*ones(1,T_len),linspace(0,1.1*max(DV23),T_len),'Color',c4)
% hold on
% plot(dismin31index*ones(1,T_len),linspace(0,1.1*max(DV31),T_len),'Color',c8)
legend('追踪者-目标','追踪者-防御者')
xlabel('t/s');
ylabel('\deltaV')
grid on
box on

% X
figure
plot(t_int,xa,'linestyle','-.','Color',c1,'linewidth',1.5);
hold on
plot(t_int,xd,'linestyle','-','Color',c5,'linewidth',1.5);
hold on
plot(t_int,xt,'Color',c9,'linewidth',1.5);
xlabel('t/s');
ylabel('X/km')
grid on
box on
legend('追踪者','防御者','目标')

% Y
figure
plot(t_int,ya,'linestyle','-.','Color',c1,'linewidth',1.5);
hold on
plot(t_int,yd,'linestyle','-','Color',c5,'linewidth',1.5);
hold on
plot(t_int,yt,'Color',c9,'linewidth',1.5);
xlabel('t/s');
ylabel('Y/km')
grid on
box on
legend('追踪者','防御者','目标')

% Z
figure
plot(t_int,za,'linestyle','-.','Color',c1,'linewidth',1.5);
hold on
plot(t_int,zd,'linestyle','-','Color',c5,'linewidth',1.5);
hold on
plot(t_int,zt,'Color',c9,'linewidth',1.5);
xlabel('t/s');
ylabel('Z/km')
grid on
box on
legend('追踪者','防御者','目标')


% Ux
figure
plot(t_int,uxa,'linestyle','-.','Color',c1,'linewidth',1.5);
hold on
plot(t_int,uxd,'linestyle','-','Color',c5,'linewidth',1.5);
hold on
plot(t_int,uxt,'Color',c9,'linewidth',1.5);
xlabel('t/s');
ylabel('U_x')
grid on
box on
legend('追踪者','防御者','目标')

%Uy
figure
plot(t_int,uya,'linestyle','-.','Color',c1,'linewidth',1.5);
hold on
plot(t_int,uyd,'linestyle','-','Color',c5,'linewidth',1.5);
hold on
plot(t_int,uyt,'Color',c9,'linewidth',1.5);
xlabel('t/s');
ylabel('U_y')
grid on
box on
legend('追踪者','防御者','目标')

%Uz
figure
plot(t_int,uza,'linestyle','-.','Color',c1,'linewidth',1.5);
hold on
plot(t_int,uzd,'linestyle','-','Color',c5,'linewidth',1.5);
hold on
plot(t_int,uzt,'Color',c9,'linewidth',1.5);
xlabel('t/s');
ylabel('U_z')
grid on
box on
legend('追踪者','防御者','目标')

figure
plot(t_int,x31,'linestyle','-.','Color',c1,'linewidth',1.5);
hold on
plot(t_int,y31,'linestyle','-','Color',c5,'linewidth',1.5);
hold on
plot(t_int,z31,'Color',c9,'linewidth',1.5);
xlabel('t/s');
ylabel('R31/km')
grid on
box on
legend('x','y','z')

figure
plot(t_int,x23,'linestyle','-.','Color',c1,'linewidth',1.5);
hold on
plot(t_int,y23,'linestyle','-','Color',c5,'linewidth',1.5);
hold on
plot(t_int,z23,'Color',c9,'linewidth',1.5);
xlabel('t/s');
ylabel('R23/km')
grid on
box on
legend('x','y','z')

%3D
figure
plot3(xa,ya,za,'b--','linewidth',2);
hold on
plot3(xd,yd,zd,'g--','linewidth',2);
hold on
plot3(xt,yt,zt,'r','linewidth',2);
hold on
plot3(xa(1),ya(1),za(1),'p');
hold on
plot3(xd(1),yd(1),zd(1),'s');
hold on
plot3(xt(1),yt(1),zt(1),'o');
grid on
box on
view(80,70)
legend('追踪者','防御者','目标','追踪者','防御者','目标' ,'Location','NorthWest')

%X轴
figure
plot(xa,ya,'b-','linewidth',2);
hold on
plot(xd,yd,'g-.','linewidth',2);
hold on
plot(xt,yt,'r','linewidth',2);
hold on
plot(xa(1),ya(1),'p');
hold on
plot(xd(1),yd(1),'s');
hold on
plot(xt(1),yt(1),'o');
hold on 
plot(xa(dismin23index/step),ya(dismin23index/step),'*','Markersize',12);
hold on 
plot(xa(dismin31index/step),ya(dismin31index/step),'*','Markersize',12);
hold on
arrowPlot(xa,ya, 'number',2,'color','b');
hold on
arrowPlot(xd,yd, 'number',2,'color','g');
hold on
arrowPlot(xt,yt, 'number',2,'color','r')
%%annotation('arrow','x',[0.41,0.41],'Y',[0.6,0.5]);
grid on
box on
xlabel('x/km');
ylabel('y/km')
legend('追踪者','防御者','目标','追踪者起始点','防御者起始点','目标起始点'...
    ,'Location','NorthWest')
set(gca,'FontSize',14)

figure
plot(xa,za,'b--','linewidth',2);
hold on
plot(xd,zd,'g--','linewidth',2);
hold on
plot(xt,zt,'r-.','linewidth',2);
hold on
plot(xa(1),za(1),'p');
hold on
plot(xd(1),zd(1),'s');
hold on
plot(xt(1),zt(1),'o');
hold on
plot(xa(dismin23index/step),za(dismin23index/step),'*','Markersize',12);
hold on 
plot(xa(dismin31index/step),za(dismin31index/step),'*','Markersize',12);
hold on
arrowPlot(xa,za, 'number',2,'color','b');
hold on
arrowPlot(xd,zd, 'number',2,'color','g');
hold on
arrowPlot(xt,zt, 'number',2,'color','r')
xlabel('x/km');
ylabel('z/km')
grid on
box on
legend('追踪者','防御者','目标','追踪者起始点','防御者起始点','目标起始点','Location','NorthWest')
set(gca,'FontSize',14)

figure
plot(ya,za,'b--','linewidth',2);
hold on
plot(yd,zd,'g--','linewidth',2);
hold on
plot(yt,zt,'r-.','linewidth',2);
hold on
plot(ya(1),za(1),'p');
hold on
plot(yd(1),zd(1),'s');
hold on
plot(yt(1),zt(1),'o');
hold on
plot(ya(dismin23index/step),za(dismin23index/step),'*','Markersize',12);
hold on 
plot(ya(dismin31index/step),za(dismin31index/step),'*','Markersize',12);
hold on
arrowPlot(ya,za, 'number',2,'color','b');
hold on
arrowPlot(yd,zd, 'number',2,'color','g');
hold on
arrowPlot(yt,zt, 'number',2,'color','r')
xlabel('z/km');
ylabel('y/km');
grid on
box on
legend('追踪者','防御者','目标','追踪者起始点','防御者起始点','目标起始点','Location','NorthWest')
set(gca,'FontSize',14)