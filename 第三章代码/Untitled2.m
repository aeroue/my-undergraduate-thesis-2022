clear
clc
%%
% %仿真初值
% Q=1e-16*eye(6);
% Rp=1e-6*eye(3);
% Re=1.5*1e-6*eye(3);
% x(:,1)=[300;150;-100;0;0;0];
% up(:,1)=zeros(3,1);
% ue(:,1)=zeros(3,1);
% %%工况三仿真初值
% Q=eye(6);%1e-5*eye(6);
% Rp=eye(3);%1e-2*eye(3);
% Re=eye(3)*1.5;%0.008*eye(3);
Q=1e-5*eye(6);
Rp=1e-2*eye(3);
Re=0.008*eye(3);
x(:,1)=[6000;-16000;4000;3;-7.5;0];
%x(:,1)=[3000;1500;-1000;0;0;0];
up(:,1)=zeros(3,1);
ue(:,1)=zeros(3,1);

%%
%动力学模型
mu=3.986*10^14;
%rp=6720804;
rp = 8378136;
w0=sqrt(mu/rp^3);

A11=zeros(3);
A12=eye(3);
A21=[3*w0^2 0 0;0 0 0;0 0 -w0^2];
A22=[0 2*w0 0;-2*w0 0 0;0 0 0];
A=[A11 A12;A21 A22]
Bp=[zeros(3);-eye(3)];
Be=[zeros(3);eye(3)];
%%
%控制策略初值求解
j=1;
%dt=0.1;
dt=0.1;
t(1)=0;
J=0;

Sp=Bp*Rp^(-1)*Bp'
Se=Be*Re^(-1)*Be'
P=care(A,Bp,Q,Rp)
for i=1:1:20
A1=A-Sp*P;
Q1=Q+P*Sp*P+P*Se*P;
P=lyap(A1',Q1);
end
P
%     r11 = Rp(1,1) * Re(1,1) / (Re(1,1) - Rp(1,1));
%     r22 = Rp(2,2) * Re(2,2) / (Re(2,2) - Rp(2,2));
%     r33 = Rp(3,3) * Re(3,3) / (Re(3,3) - Rp(3,3));  
%     
%     R = diag([r11 r22 r33]);
%     
%     P=care(A,Bp,Q,R)


%%
%控制策略更新
for j=2:100000
    t(j)=t(j-1)+dt;
    a=fix(t(j)/10);
    x(:,j)=x(:,j-1)+dt*(A*x(:,j-1)+Bp*up(:,j-1)+Be*ue(:,j-1));
    up(:,j)=-Rp^(-1)*Bp'*P*x(:,j);
%     ue(:,j)=zeros(3,1);%死目标
    %ue(:,j)=Re^(-1)*Be'*P*x(:,j);%纳什均衡解
    ue(:,j)=[cos(a*pi/100);cos(a*pi/100);cos(a*pi/100)];
    for a=1:3
        if up(a,j)>=5
            up(a,j)=5;
        else if up(a,j)<=-5
                up(a,j)=-5;
            end
        end
       if ue(a,j)>=2
            ue(a,j)=2;
        else if ue(a,j)<=-2
                ue(a,j)=-2;
            end
        end
    end
  
    J=J+x(:,j)'*Q*x(:,j)+up(:,j)'*Rp*up(:,j)+ue(:,j)'*Re*ue(:,j);
end

Xi(:,1) = zeros(6,1);
    for i = 2:100000
        dx = dt * (A * Xi(:,i-1)+Be * ue(:,i));
        Xi(:,i) = Xi(:,i-1) + dx;
    end
    
    Xp(:,1) = zeros(6,1);
    for i = 2:100000
        dx = dt * (A * Xp(:,i-1)+Bp * up(:,i));
        Xp(:,i) = Xp(:,i-1) + dx;
    end
    
 %%
 %下一步的复现关键在三个点，运动方程UE会变化，给出UE的控制策略，黎卡提方程加入UE项 并通过李雅普诺夫迭代求解P，指标函数加入ue.
%绘图
figure(1)
plot(t,x(1,:),'r:',t,x(2,:),'k--',t,x(3,:),'b-','linewidth',3)
xlabel('t/s')
ylabel('相对距离/m')
legend('x','y','z')
grid on
box on

figure(2)
plot(t,x(4,:),'r:',t,x(5,:),'k--',t,x(6,:),'b-','linewidth',3)
xlabel('t/s')
ylabel('相对速度/（m/s)')
legend('x','y','z')
grid on
box on



figure(3)
subplot(3,1,1)
plot(t,up(1,:),'linewidth',3)
box on
grid on
title('x轴控制力加速度')

subplot(3,1,2)
plot(t,up(2,:),'linewidth',3)
title('y轴控制力加速度')
box on
grid on
ylabel('控制加速度曲线/（m/s^2）')

subplot(3,1,3)
plot(t,up(3,:),'linewidth',3)
title('z轴控制力加速度')
box on
grid on
xlabel('t/s')

figure(4)
plot(t,ue,'linewidth',3)
xlabel('t/s')
ylabel('ue')
legend('ue_x','ue_y','ue_z')


figure(5)
plot(Xi(1,:)/1000,Xi(2,:)/1000,'b-.','linewidth',1.5)
hold on
plot(Xp(1,:)/1000,Xp(2,:)/1000,'r','linewidth',1.5)
xlabel('x(km)')
ylabel('y(km)')

%legend('ue_x','ue_y','ue_z')
