%run_LQR
clear
clc
close all
%% 初值
%动力学模型
mu = 3.986*10^14; %地球引力常数
rp = 8378136; %追踪航天器的半长轴
omega = gen_omega(mu,rp);
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

X0 = [6*1000;16*1000;-4*1000;-3;7.5;0];

% 控制参数
Q = diag([1,1,1,1,1,1])*1e-17;
Rp = diag([1,1,1])*1e-4;

P=care(A,B,Q,Rp);

dt = 10;
tf = 15000;
tau_int = 0:dt:tf;

options = odeset('AbsTol', 1e-12, 'RelTol', 1e-8);
[~,X] = ode45(@(t,x)xdot(t,x,A,B,P,Rp),tau_int,X0,options);
Up = gen_U(X,B,P,Rp);

draw(tau_int,X',Up)

function omega = gen_omega(mu,rp)
    omega =sqrt(mu/rp^3);
end

function x = xdot(t,x,A,B,P,Rp)
    Up = Rp^(-1) * B' * P * x;
    Ue = zeros(3,1);
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

function U = gen_U(x,B,P,Rp)
    U = -Rp^(-1) * B' * P * x';
end

function draw(t,x,up)
     m = load('result_inf_non.mat')
     m = m.result.inf_res(2)
    num = numel(t)
    figure
    plot(t,x(1,:),'r:',t,x(2,:),'k--',t,x(3,:),'b-','linewidth',1.5)
    xlabel('t/s')
    ylabel('相对距离/m')
    legend('x','y','z')  
    set(gca,'FontSize',14)
    grid on
    box on

%     figure
%     plot(t,x(4,:),'r:',t,x(5,:),'k--',t,x(6,:),'b-','linewidth',1.5)
%     xlabel('t/s')
%     ylabel('相对速度/（m/s)')
%     legend('x','y','z')
%     grid on
%     box on
%     
%         
%     figure
%     subplot(3,1,1)
%     plot(t,up(1,:))
%     box on
%     grid on
%     title('x轴控制力加速度')
% 
%     subplot(3,1,2)
%     plot(t,up(2,:))
%     title('y轴控制力加速度')
%     box on
%     grid on
%     ylabel('控制加速度曲线/（m/s^2）')
% 
%     subplot(3,1,3)
%     plot(t,up(3,:))
%     title('z轴控制力加速度')
%     grid on
%     hold on
%     xlabel('t/s')
%     
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

    
        figure
    plot(t,up(1,:),'r:',t,up(2,:),'k--',t,up(3,:),'b-','linewidth',1.5)
    xlabel('t/s')
    ylabel('控制力加速度大小(m/（s^2）)')
    legend('x','y','z')  
    set(gca,'FontSize',14)
    grid on
    box on

    figure
    %subplot(3,1,1)
    plot(t,x(1,:)/1000,'k--','linewidth',2)
    hold on
    plot(t,m.X(1,1:num)','m','linewidth',2)
    box on
    grid on
    xlabel('时间/s')
    ylabel('x/（km）')
    set(gca,'FontSize',14)
    legend('LQR','博弈控制')
    
        figure
    %subplot(3,1,1)
    plot(t,x(2,:)/1000,'k--','linewidth',2)
    hold on
    plot(t,m.X(2,1:num)','m','linewidth',2)
    box on
    grid on
    xlabel('时间/s')
    set(gca,'FontSize',14)
    ylabel('y/（km）')
     legend('LQR','博弈控制')
        %subplot(3,1,1)
        figure
    plot(t,x(3,:)/1000,'b--','linewidth',2)
    hold on
    plot(t,m.X(3,1:num)','m','linewidth',2)
    box on
    grid on
    set(gca,'FontSize',14)
    xlabel('时间/s')
    ylabel('z/（km）')
    legend('LQR','博弈控制')
    
    figure
    %subplot(3,1,1)
    plot(t,up(1,:),'b--','linewidth',2)
    hold on
    plot(t,m.control(1:num,1)','m','linewidth',2)
    box on
    grid on
    xlabel('时间/s')
    ylabel('U_x/（m/s^2）')
    %title('x轴控制力加速度')
    legend('LQR','博弈控制')
    set(gca,'FontSize',14)
    %subplot(3,1,2)
    figure
    %subplot(3,1,1)
    plot(t,up(2,:),'b--','linewidth',1.5)
    hold on
    plot(t,m.control(1:num,2)','m','linewidth',1.5)
    box on
    grid on
    xlabel('时间/s')
    ylabel('U_y/（m/s^2）')
    set(gca,'FontSize',14)
        legend('LQR','博弈控制')

    %subplot(3,1,3)
    figure
    %subplot(3,1,1)
    plot(t,up(3,:),'b--','linewidth',2)
    hold on
    plot(t,m.control(1:num,3)','m','linewidth',2)
    box on
    grid on
    xlabel('时间/s')
    ylabel('U_z/（m/s^2）')
    set(gca,'FontSize',14)
        legend('LQR','博弈控制')
%         
%     figure
%     plot(t,sum(sqrt(up.^2)))
end

