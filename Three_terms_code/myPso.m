function [pg,fpg] = myPso(fitness_func,dim,popmin,popmax,vmax,vmin,optPSO)

    N = optPSO.npop;       
    iter_num = optPSO.niter;
    Plotflag = optPSO.Plot;
    c1 = optPSO.c1;
    c2 = optPSO.c2;
    % 种群初始化 
    pop = ones(N,dim).*popmin + abs(rand(N,dim)).*(popmax-popmin);
    % 初始化种群各个粒子速度
    V = ones(N,dim).*vmin + abs(rand(N,dim)).*(vmax-vmin);
    % 计算种群各个粒子初始最优适应值
    for i=1:N
        fpi(i)=fitness_func(pop(i,:)); 
        pi(i,1:4)=pop(i,:); 
    end
    % 计算种群初始最优适应值
    pg=pop(1,:);
    %fpg=fitness_func(pop(1,:));
    for i=2:N
        if fitness_func(pop(i,:))<fitness_func(pg)
            fpg=fitness_func(pop(i,:));
            pg=pop(i,:);
        end
    end
    % 迭代寻优
    disp('***************************************')
for t=1:iter_num
    w = 0.9 - 0.4 * t/iter_num; %惯性权重
    V = velocity_update(V,pop,pi,pg,c1,c2,w,vmax,vmin);%更新粒子速度
    pop = position_update(pop,V,popmax,popmin);%更新粒子位置
    % 更新每个粒子的历史最优位置
    for i=1:N
        if fitness_func(pop(i,:))<fpi(i)
            fpi(i)=fitness_func(pop(i,:));
            pi(i,:)=pop(i,:);
        end
        % 更新群体的历史最优位置
        if fpi(i)<fpg
            fpg=fpi(i);
            pg=pi(i,:);
        end
    end
    yy(t)=fpg;
    fprintf('---------------------------------------------\n');
    fprintf('迭代次数：%d\n',t); 

end    
    disp('***************************************')
    disp(['目标函数的最大值为:',num2str(fpg)]);
    disp(['取最大值时自变量取值为:',num2str(pg)]);
    disp('***************************************')

    f3=figure('color',[1 1 1]);
    plot(yy,'*-')
    title('适应值变化曲线')
    xlabel('迭代次数')
    ylabel('适应值')

end
function vv = velocity_update(V,pop,pi,pg,c1,c2,w,vmax,vmin)
    si = size(pop);
    r1 = rand(si);
    r2 = rand(si);
    vv = w * V + c1 * r1 .*(pi - pop) + c2 * r2 .* (pg - pop);
    % 防止越界处理
     for i = 1:si(1)
        
        for j = 1:si(2)
            if vv(i,j)>vmax(j)
                vv(i,j)=vmax(j);
            elseif vv(i,j)<vmin(j)      
                vv(i,j)=vmin(j);
            end            
        end
    end
end
function Xw = position_update(pop,V,popmax,popmin)
    si = size(pop);
    Xw = pop + V;
    % 防止越界处理
    for i = 1:si(1)
        
        for j = 1:si(2)
            if Xw(i,j)>popmax(j)

                Xw(i,j)=popmax(j);

            elseif Xw(i,j)<popmin(j)


                Xw(i,j)=popmin(j);
            end

        end
    end
end

