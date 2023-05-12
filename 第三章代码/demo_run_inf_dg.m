clear
clc
close all
%% 初值
%动力学模型
mu = 3.986*10^14; %地球引力常数
rp=8378136;
omega = gen_omega(mu,rp);
X0 = [6*1000;16*1000;-4*1000;-3;7.5;0];
A11 = zeros(3,3);
A12 = eye(3,3);
A21 = [3*omega^2  0      0;
          0       0      0;
          0       0   -omega^2];
A22 = [   0     2*omega  0;
        -2*omega  0      0;
          0       0      0];
A = [    A11     A12;
         A21     A22];
     
B = [zeros(3,3);
       eye(3,3)];
% 控制参数
Q = diag([1,1,1,1,1,1])*1e-17;
Rp = diag([1,1,1])*1e-9;
Re = 1.5*diag([1,1,1])*1e-9;

R = gen_R(Rp, Re);
    
P = care(A,B,Q,R);

dt = 0.1;
tf = 300;
tau_int = 0:dt:tf;

options = odeset('AbsTol', 1e-12, 'RelTol', 1e-8);

%% 主函数
[~,X] = ode45(@(t,x)xdot(t,x,A,B,P,Rp,Re),tau_int,X0,options);
Up = gen_U(X,B,P,Rp);
%Ue = gen_U(X,B,P,Re);
%Ue = zeros(3,1);
draw(tau_int,X',Up)

%% 局部函数
function omega = gen_omega(mu,rp)
    omega =sqrt(mu/rp^3);
end
function R = gen_R(Rp, Re) 

    r11 = Rp(1,1) * Re(1,1) / (Re(1,1) - Rp(1,1));
    r22 = Rp(2,2) * Re(2,2) / (Re(2,2) - Rp(2,2));
    r33 = Rp(3,3) * Re(3,3) / (Re(3,3) - Rp(3,3));  
    
    R = diag([r11 r22 r33]);
    
end

function x = xdot(t,x,A,B,P,Rp,Re)
    Up = Rp^(-1) * B' * P * x;
    %Ue = zeros(3,1);
    Ue = Re^(-1) * B' * P * x;
    %Ue = [cos(t*pi/100);cos(t*pi/100);cos(t*pi/100)];
    
    for i = 1:3
        if Up(i)>=5
            Up(i)=5;
        elseif Up(i)<=-5
            Up(i)=-5;  
        end
    end 
    U = Ue - Up;
    x = A * x + B * U;
end

function U = gen_U(x,B,P,R)
    U = R^(-1) * B' * P * x';
end

function draw(t,x,up)
    figure
    plot(t,x(1,:),'r:',t,x(2,:),'k--',t,x(3,:),'b-','linewidth',1.5)
    xlabel('t/s')
    ylabel('相对距离/m')
    legend('x','y','z')   
    grid on
    box on

    figure
    plot(t,x(4,:),'r:',t,x(5,:),'k--',t,x(6,:),'b-','linewidth',1.5)
    xlabel('t/s')
    ylabel('相对速度/（m/s)')
    legend('x','y','z')
    grid on
    box on
    
        
    figure
    subplot(3,1,1)
    plot(t,up(1,:))
    box on
    grid on
    title('x轴控制力加速度')

    subplot(3,1,2)
    plot(t,up(2,:))
    title('y轴控制力加速度')
    box on
    grid on
    ylabel('控制加速度曲线/（m/s^2）')

    subplot(3,1,3)
    plot(t,up(3,:))
    title('z轴控制力加速度')
    grid on
    hold on
    xlabel('t/s')
    
%     figure(4)
%     subplot(3,1,1)
%     plot(t,ue(1,:))
%     box on
%     grid on
%     title('x轴控制力加速度')
% 
%     subplot(3,1,2)
%     plot(t,ue(2,:))
%     title('y轴控制力加速度')
%     box on
%     grid on
%     ylabel('控制加速度曲线/（m/s^2）')
% 
%     subplot(3,1,3)
%     plot(t,ue(3,:))
%     title('z轴控制力加速度')
%     grid on
%     hold on
%     xlabel('t/s')
end

