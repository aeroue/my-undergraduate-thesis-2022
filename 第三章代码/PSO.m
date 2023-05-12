clc
clear
initialize;

%% 设置种群参数
N = 50; %种群规模
iter_num = 100; %最大迭代次数
dim = 4; %变量个数
%w = 0.729; %惯性权重
c1 = 2; %认知学习因子
c2 = 2; %社会学习因子
popmax = [1,1,1,10000]; %粒子最大位置
popmin = [-1,-1,-1,1000]; %粒子最小位置
vmax = [1,1,1,10]; %粒子最大运动速度
vmin = -[1,1,1,10]; %粒子最小运动速度
best_fitness = 9e-10;

%% 种群初始化
% 初始化种群各个粒子位置
pop = ones(N,dim).*popmin + abs(rand(N,dim)).*(popmax-popmin);
% 初始化种群各个粒子速度
V = ones(N,dim).*vmin + abs(rand(N,dim)).*(vmax-vmin);
% 计算种群各个粒子初始最优适应值
for i=1:N
    fpi(i)=fitness_func(pop(i,:)); 
    pi(i,1:4)=pop(i,:); 
end
    %fprintf('fit完成')
% 计算种群初始最优适应值
pg=pop(1,:);
fpg=fitness_func(pop(1,:));
for i=2:N
    if fitness_func(pop(i,:))<fitness_func(pg)
        fpg=fitness_func(pop(i,:));
        pg=pop(i,:);
    end
end
% 初始的个体最优位置和种群最优位置
% pbest = pi;
% gbest = pg;
% %% 绘制初始状态图
% plot(pi,fpi,'ro','MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',5)
% title('粒子初始分布图')
% xlabel('X')
% ylabel('Y')

%% 迭代寻优
for t=1:iter_num
    %t
    %pause(1)
    w = 0.9 - 0.4 * t/iter_num; %惯性权重
    V = velocity_update(V,pop,pi,pg,c1,c2,w,vmax,vmin);%
    %pause(1)%更新粒子速度
    pop = position_update(pop,V,popmax,popmin);%更新粒子位置
    % 更新每个粒子的历史最优位置
    for i=1:N
        i
        %pt = pop(i,:)
        %ft = fpi(i)
        %pause(1)
        if fitness_func(pop(i,:))<fpi(i)
            fpi(i)=fitness_func(pop(i,:));
            pi(i,:)=pop(i,:);
        end
        %pot = pi(i,:)
        %pause(0.1)
        % 更新群体的历史最优位置
        if fpi(i)<fpg
            fpg=fpi(i);
            pg=pi(i,:);
        end
    end
    yy(t)=fpg;
end
% for t=1:iter_num
%     for i=1:N
%         V(i,:) = velocity_update(V(i,:),pi(i,:),pbest(i,:),gbest,c1,c2,w,vmax,vmin); %更新粒子速度
%         pop(i,:) = position_update(pop(i,:),V(i,:));    %更新粒子位置
%         % 更新每个粒子的历史最优位置
%         if fitness_func(pop(i,:))>fpi(i)
%             fpi(i)=fitness_func(pop(i,:));
%             pi(i,:)=pop(i,:);
%         end
%         % 更新群体的历史最优位置
%         if fpi(i)>fpg
%             fpg=fpi(i);
%             pg=pi(i,:);
%         end
%     end
%     yy(t)=fpg;
% end

%% 输出结果 绘制适应度变化曲线
disp('***************************************')
disp(['目标函数的最大值为:',num2str(fpg)]);
disp(['取最大值时自变量取值为:',num2str(pg)]);
disp('***************************************')
f2=figure('color',[1 1 1]);
%mesh(x,y,z)
hold on
plot(pg,fpg,'ro','MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',5) %绘制定位结果图
title('粒子寻优结果')
xlabel('X轴')
ylabel('Y轴')
%zlabel('Z轴')
f3=figure('color',[1 1 1]);
plot(yy,'Linewidth',3)
title('适应值变化曲线')
xlabel('迭代次数')
ylabel('适应值')

%计算粒子的适应值（目标函数值）
function J = fitness_func(pop)
    fprintf('测试:\n')
    pop
    pause(0.1)
    global params
    
    A = params.A;
    B = params.B;
    
    k1 = 1; 
    k2 = 1; 
    k3 = 1;
    k4 = 1;
    
    g = 9.78;
    Tei = 0.0004*g;
    Tpi = 0.0006*g;
    xx =[-16400 -60300 7200 1.5 4.2 0]'; 
    step = 20;
    tau_int = 0:step:pop(4);
    lamda_tf = [pop(1);pop(2);pop(3);0;0;0];
    
    options = odeset('AbsTol', 1e-12, 'RelTol', 1e-8);
    [~,lamda_aut] = ode45(@(t,lamda)lamda_dot(t,lamda),tau_int,lamda_tf,options);
    lamda_t = lamda_aut(end:-1:1,:);
    
    num = size(lamda_t,1);
    
    for i = 1:num
    lamdat = lamda_t(i,:);
    
    salphap = lamdat(5)/sqrt(lamdat(4)^2+lamdat(5)^2);
    calphap = lamdat(4)/sqrt(lamdat(4)^2+lamdat(5)^2);
    sbetap = lamdat(6)/sqrt(lamdat(4)^2+lamdat(5)^2+lamdat(6)^2);
    cbetap = sqrt(lamdat(4)^2+lamdat(5)^2)/sqrt(lamdat(4)^2+lamdat(5)^2+lamdat(6)^2);
    salphae = salphap;
    calphae = calphap;
    sbetae = sbetap;
    cbetae = cbetap;
    
    ae = Tei .* [cbetae*calphae;cbetae*salphae;sbetae];
    ap = Tpi .* [cbetap*calphap;cbetap*salphap;sbetap];

    U = ae - ap;
    
    if isnan(U)
        U = (Tei - Tpi).*ones(3,1);
    end
    
    dx = A * xx + B * U;
    xx = xx + step*dx;
    
    end
    
    x_tf = xx;
    
    Htf = Hamilton_tf(pop(1),pop(2),pop(3),x_tf);%!!!!pop（4）时间？？;
    J = k1 * x_tf(1)^2 + k2 * x_tf(2)^2 + k3 * x_tf(3)^2 + k4 * (Htf+1)^2
end
function Htf = Hamilton_tf(v1,v2,v3,x)
    x_tf = x;
    Htf = v1*x_tf(4) + v2*x_tf(5) + v3*x_tf(6);
end
function dlamda = lamda_dot(t,lamda)
    global params
    A = params.A;
    dlamda = A' * lamda;
end
function vv = velocity_update(V,pop,pi,pg,c1,c2,w,vmax,vmin)
    si = size(pop)
    r1 = rand(si);
    r2 = rand(si);
    vv = w * V + c1 * r1 .*(pi - pop) + c2 * r2 .* (pg - pop);
    % 防止越界处理
     for i = 1:si(1)
        
        for j = 1:si(2)
            if vv(i,j)>vmax(j)
                fprintf('yunxing1:%8.8f %8.8f\n',vv(i,j),vmax(j))
                vv(i,j)=vmax(j);
            elseif vv(i,j)<vmin(j)      
                fprintf('yunxing2:%8.8f %8.8f\n',vv(i,j),vmin(j))
                vv(i,j)=vmin(j);
            end
            %X(i,:)
            
        end
%        fprintf('X完成')
%         X(i,4)
%         %pause(0.1)
    end
end
function Xw = position_update(pop,V,popmax,popmin)
    si = size(pop);
    Xw = pop + V;
    % 防止越界处理
    for i = 1:si(1)
        %i
        %X(i,:)
        %pause(1)
        
        for j = 1:si(2)
            %j
            %X(i,j)
            %popmax(j)
            %pause(1)
            if Xw(i,j)>popmax(j)
                %fprintf('yunxing1:%8.8f %8.8f\n',Xw(i,j),popmax(j))
                %pause(5)
                Xw(i,j)=popmax(j);
                %Xw(i,j)
            elseif Xw(i,j)<popmin(j)
                %Xw(i,j)=popmin(j);
                fprintf('yunxing2:%8.8f %8.8f\n',Xw(i,j),popmin(j))
                                %pause(5)
                %Xw(i,j)
                Xw(i,j)=popmin(j);
            end
            %X(i,:)
            
        end
%        fprintf('X完成')
%         X(i,4)
%         %pause(0.1)
    end
end
    

 
