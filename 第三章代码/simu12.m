%轨道追逃博弈（状态相关里卡提方程）
%以追踪航天器为参考轨道,圆轨道

clear
clc
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%动力学模型%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mu=3.986*10^14; %地球引力常数
rrp=8378136; %追踪航天器的半长轴
w=sqrt(mu/rrp^3);%追踪航天器的轨道偏心率是小量，则轨道角速度接近轨道平均角速度
A11=zeros(3,3);
A12=eye(3,3);
A21=[3*w^2 0 0;
    0 0 0;
    0 0 -w^2];
A22=[0 2*w 0;
    -2*w 0 0;
    0 0 0];
A1=[A11 A12;
    A21 A22];%动力学方程A矩阵
Bp=-[zeros(3,3);
    eye(3,3)];%动力学方程BP、BE
Be=[zeros(3,3);
    eye(3,3)];
%%
%初始值
j=1;
dt=0.1;
t(1)=0;
x(:,1)=[6000;-16000;4000;3;-7.5;0];%初始时刻追踪航天器相对于目标的位置；位置、速度
up(:,1)=zeros(3,1);
ue(:,1)=zeros(3,1);


%控制参数
Qpe=diag([1,1,1,1,1,1])*1e-16;%Q矩阵
Rp=diag([1,1,1])*1e-5;%追踪航天器的的R矩阵，这是因为二次型目标函数涉及Q和R  求解下面的Riccati
J=0;
P=care(A1,Bp,Qpe,Rp)%李雅普诺夫迭代中求初值
%%若是P复杂些、需要李雅普诺夫迭代，这里缺少
for j=2:2000
    t(j)=t(j-1)+dt;  
    x(:,j)=x(:,j-1)+dt*(A1*x(:,j-1)+Bp*up(:,j-1)+Be*ue(:,j-1));%相对运动方程
    a=fix(t(j)/10);
    %控制更新
    up(:,j)=-Rp^(-1)*Bp'*P*x(:,j);
    ue(:,j)=[cos(a*pi/100);cos(a*pi/100);cos(a*pi/100)];
    
    %%up约束
    if up(1,j)>=5
        up(1,j)=5;
    else if up(1,j)<=-5
         up(1,j)=-5;  
        end
    end
    
    if up(2,j)>=5
        up(2,j)=5;
    else if up(2,j)<=-5
         up(2,j)=-5;  
        end
    end
    
    if up(3,j)>=5
        up(3,j)=5;
    else if up(3,j)<=-5
         up(3,j)=-5;  
        end
    end
    
  ue(:,j)=zeros(3,1);%ue固定 也就是不采取机动
     J=J+x(:,j)'*Qpe*x(:,j)+up(:,j)'*Rp*up(:,j); %每一步的指标求和就是总时长的积分
end

figure(1)
plot(t,x(1,:),'r:',t,x(2,:),'k--',t,x(3,:),'b-','linewidth',3)
xlabel('t/s')
ylabel('相对距离/m')
legend(['x'],['y'],['z'])
grid on
box on

figure(2)
plot(t,x(4,:),'r:',t,x(5,:),'k--',t,x(6,:),'b-','linewidth',3)
xlabel('t/s')
ylabel('相对速度/（m/s)')
legend(['x'],['y'],['z'])
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
grid on
hold on
xlabel('t/s')
