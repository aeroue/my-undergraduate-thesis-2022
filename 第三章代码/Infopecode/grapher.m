% draw2.m
function grapher(result)
    %close all
    tau_int = result.tau_int;
    deltX_tol = sqrt(result(1).relative_X(1,:).^2 + ...
        result(1).relative_X(2,:).^2 + ...
        result(1).relative_X(3,:).^2);
    deltV_tol = sqrt(result(1).relative_V(1,:).^2 + ...
        result(1).relative_V(2,:).^2 + ...
        result(1).relative_V(3,:).^2);
    
    for i = 1:10
        eval(['c',num2str(i),'=','getColor(i,1,11)']);  
    end

    figure;
    plot3(result(1).X(1,:),result(1).X(2,:) ,...
        result(1).X(3,:),'b-.','linewidth',2);
    hold on
    plot3(result(2).X(1,:),...
    result(2).X(2,:) , ...
    result(2).X(3,:),'r','linewidth',2);
    hold on
    plot3(result(1).X(1,1),result(1).X(2,1) ,...
        result(1).X(3,1),'p');
    hold on
    plot3(result(2).X(1,1),...
    result(2).X(2,1), ...
    result(2).X(3,1),'o');
    hold on
%     plot3(result(2).X(1,100),result(2).X(2,100) ,...
%         result(2).X(3,100),'>');
%     hold on
%     p0 = [result(1).X(1,1),result(1).X(2,1) ,...
%         result(1).X(3,1)];
%     p1 = [result(1).X(1,15),result(1).X(2,15) ,...
%         result(1).X(3,15)]
%     vectarrow(p0,p1)
%     hold on
%     p0 = [result(2).X(1,1),result(2).X(2,1) ,...
%         result(2).X(3,1)];
%     p1 = [result(2).X(1,2),result(2).X(2,2) ,...
%         result(2).X(3,2)]
%     vectarrow(p0,p1)
%    annotation('arrow',result(1).X(1,1),result(1).X(2,1),result(1).X(3,1));
   quiver3(result(1).X(1,1),result(1).X(2,1) ,...
         result(1).X(3,1),result(1).X(1,100)-result(1).X(1,1),result(1).X(2,100)-result(1).X(2,1) ,...
        result(1).X(3,100)-result(1).X(3,1),'Color',[0.0,0.0,1.0],'LineWidth',2.0,'Maxheadsize',17);
   hold on
   
      quiver3(result(2).X(1,1),result(2).X(2,1) ,...
         result(2).X(3,1),result(2).X(1,100)-result(2).X(1,1),result(2).X(2,100)-...,
         result(2).X(2,1),result(2).X(3,100)-result(2).X(3,1),...
         'Color',[1.0,0.0,0.0],'LineWidth',2.0,'Maxheadsize',17);
   hold on
    %set(h,'maxheadsize',5); 
    hold on    
    grid on
    xlabel('x(km）');
    ylabel('y(km）');
    zlabel('z(km）');
    view(10,0)
    box on
    legend('目标','追踪航天器','目标起始点','追踪航天器起始点','Location','NorthEast')
    axis equal
    set(gca,'FontSize',14)
    %title('固定时域追逃三维空间轨迹')

    figure;
    %subplot(2, 1, 1);
    plot(tau_int,deltX_tol,'Color',c3,'linewidth',2)
    xmin = min(tau_int);
    xmax = max(tau_int);
    ymin = 1.1 * min(min(deltX_tol));
    ymax = 1.1 * max(max(deltX_tol));
    axis([xmin xmax ymin ymax]);
    xlabel('时间(s)');
    ylabel('相对距离(km)');
    %title('相对距离');  
    grid on
    box on
    %set(gca,'FontName','Times New Roman','FontSize',14)
    set(gca,'FontSize',14)
    
    %subplot(2, 1, 2);
    figure
    plot(tau_int,deltV_tol,'Color',c4,'linewidth',2)
    xmin = min(tau_int);
    xmax = max(tau_int);
    ymin = 1.1 * min(min(deltV_tol));
    ymax = 1.1 * max(max(deltV_tol));
    axis([xmin xmax ymin ymax]);
    xlabel('时间(s)');
    ylabel('相对速度(m/s)');
    %title('相对速度');  
    grid on
    box on
    %set(gca,'FontName','Times New Roman','FontSize',14)
    set(gca,'FontSize',14)
    
    figure;
    hold all
    %subplot(2, 1, 1);
    plot(tau_int,result(1).relative_V(1,:),'--','Color',c1,'linewidth',2)
     plot(tau_int,result(1).relative_V(2,:),'-.','Color',c5,'linewidth',2)
      plot(tau_int,result(1).relative_V(3,:),'Color',c9,'linewidth',2)
%     %plot(result(2).X(4,:),result(2).X(5,:) ,...
%         result(2).X(6,:),'r-','linewidth',2)
    xlabel('时间(s)');
    ylabel('相对速度(m/s)');
    %title('相对速度');  
    legend('x','y','z')
    grid on
    box on
    %set(gca,'FontName','Times New Roman','FontSize',14)
    set(gca,'FontSize',14)
    
        figure;
    hold all
    %subplot(2, 1, 1);
    plot(tau_int,result(1).relative_X(1,:),'--','Color',c1,'linewidth',2)
     plot(tau_int,result(1).relative_X(2,:),'-.','Color',c5,'linewidth',2)
      plot(tau_int,result(1).relative_X(3,:),'Color',c9,'linewidth',2)
%     %plot(result(2).X(4,:),result(2).X(5,:) ,...
%         result(2).X(6,:),'r-','linewidth',2)
    xlabel('时间(s)');
    ylabel('相对距离(km)');
    %title('相对距离');  
    legend('x','y','z')
    grid on
    box on
    %set(gca,'FontName','Times New Roman','FontSize',14)
    set(gca,'FontSize',14)

    num = numel(tau_int);
    figure;
    %subplot(3, 1, 1);
    plot(tau_int,result(1).control(1:num,1),'b-.','linewidth',1.5)
    hold on
    plot(tau_int,result(2).control(1:num,1),'r','linewidth',1.5)
    hold off
    xlabel('时间(s)');
    ylabel('U_x(m/s^2)');
    grid on
    legend('目标','追踪航天器');
    box on
    %set(gca,'FontName','Times New Roman','FontSize',14)
    set(gca,'FontSize',14)
    
    %set(h,'FontName','Times New Roman','FontSize',11,'FontWeight','normal','Orientation','horizon','Box','on')
    %subplot(3, 1, 2);
    figure
    plot(tau_int,result(1).control(1:num,2),'b-.','linewidth',1.5)
    hold on
    plot(tau_int,result(2).control(1:num,2),'r','linewidth',1.5)
    hold off
    xlabel('时间(s)');
    ylabel('U_y(m/s^2)');
    grid on
    legend('目标','追踪航天器');
    box on
    %set(gca,'FontName','Times New Roman','FontSize',14)
    set(gca,'FontSize',14)
    
    figure
    %subplot(3, 1, 3);
    plot(tau_int,result(1).control(1:num,3),'b-.','linewidth',1.5)
    hold on
    plot(tau_int,result(2).control(1:num,3),'r','linewidth',1.5)
    hold off
    xlabel('时间(s)');
    ylabel('U_z(m/s^2)');
    legend('目标','追踪航天器');
    grid on
    box on
    %set(gca,'FontName','Times New Roman','FontSize',14)
    set(gca,'FontSize',14)
    
    figure
    hold all
    %subplot(3, 1, 3);
    plot(tau_int,sqrt(sum(result(1).control(1:num,:).*result(1).control(1:num,:),2)),'b-.','linewidth',2)
    hold on
    plot(tau_int,sqrt(sum(result(2).control(1:num,:).*result(2).control(1:num,:),2)),'r','linewidth',2)
    xlabel('时间(s)');
    ylabel('U(m/s^2)');
    legend('目标','追踪航天器');
    %title('控制加速度随时间变化曲线')
    grid on
    box on
    %set(gca,'FontName','Times New Roman','FontSize',14)
    set(gca,'FontSize',14)
    
    figure;
    %subplot(3, 1, 1);
    plot(tau_int,result(1).V(1,1:num),'b-.','linewidth',1.5)
    hold on
    plot(tau_int,result(2).V(1,1:num),'r','linewidth',1.5)
    hold off
    xlabel('时间(s)');
    ylabel('V_x(m/s)');
    legend('目标','追踪航天器');
     grid on
     box on
    %set(h,'FontName','Times New Roman','FontSize',11,'FontWeight','normal','Orientation','horizon','Box','on')
    %subplot(3, 1, 2);
    %set(gca,'FontName','Times New Roman','FontSize',14)
    set(gca,'FontSize',14)
    
    figure
    plot(tau_int,result(1).V(2,1:num),'b-.','linewidth',1.5)
    hold on
    plot(tau_int,result(2).V(2,1:num),'r','linewidth',1.5)
    hold off
    xlabel('时间(s)');
    ylabel('V_y(m/s)');
    legend('目标','追踪航天器');
     grid on
     box on
     %set(gca,'FontName','Times New Roman','FontSize',14)
    %subplot(3, 1, 3);
    set(gca,'FontSize',14)
    
    figure
    plot(tau_int,result(1).V(3,1:num),'b-.','linewidth',1.5)
    hold on
    plot(tau_int,result(2).V(3,1:num),'r','linewidth',1.5)
    hold off
    xlabel('时间(s)');
    ylabel('V_z(m/s)');
    grid on
    legend('目标','追踪航天器');
    box on
    set(gca,'FontSize',14)
    
    figure
    plot(tau_int,sqrt(sum(result(1).V(:,1:num).*result(1).V(:,1:num))),'b-.','linewidth',1.5)
    hold on
    plot(tau_int,sqrt(sum(result(2).V(:,1:num).*result(2).V(:,1:num))),'r','linewidth',1.5)
    hold off
    xlabel('时间(s)');
    ylabel('V(m/s)');
    grid on
    legend('目标','追踪航天器')
    box on;
        %title('速度')
    set(gca,'FontSize',14)
        
    figure
    plot(tau_int,result(1).X(1,1:num),'b-.','linewidth',1.5)
    hold on
    plot(tau_int,result(2).X(1,1:num),'r','linewidth',1.5)
    hold off
    xlabel('时间(s)');
    ylabel('X_x(km)');
    legend('目标','追踪航天器');
    grid on
    box on
    %set(gca,'FontName','Times New Roman','FontSize',14)
     set(gca,'FontSize',14)
     
    figure
    plot(tau_int,result(1).X(2,1:num),'b-.','linewidth',1.5)
    hold on
    plot(tau_int,result(2).X(2,1:num),'r','linewidth',1.5)
    hold off
    xlabel('时间(s)');
    ylabel('X_y(km)');
    legend('目标','追踪航天器');
    grid on
    box on
    %subplot(3, 1, 3);
    set(gca,'FontSize',14)
    
    figure
    plot(tau_int,result(1).X(3,1:num),'b-.','linewidth',1.5)
    hold on
    plot(tau_int,result(2).X(3,1:num),'r','linewidth',1.5)
    hold off
    xlabel('时间(s)');
    ylabel('X_z(m/s)');
    grid on
    legend('目标','追踪航天器');
    box on
    %set(gca,'FontName','Times New Roman','FontSize',14)
   set(gca,'FontSize',14)
    
    figure
    plot(tau_int,sqrt(sum(result(1).X(:,1:num).*result(1).X(:,1:num))),'b-.','linewidth',1.5)
    hold on
    plot(tau_int,sqrt(sum(result(2).X(:,1:num).*result(2).X(:,1:num))),'r','linewidth',1.5)
    hold off
    xlabel('时间(s)');
    ylabel('X(km)');
    grid on
    legend('目标','追踪航天器')
    box on;
   %set(gca,'FontName','Times New Roman','FontSize',14)
    set(gca,'FontSize',14)
    
    figure;
    hold all
    plot(result(1).X(1,1:num),result(1).X(2,1:num),'b-.','linewidth',2)
    plot(result(2).X(1,1:num),result(2).X(2,1:num),'r','linewidth',2)
    plot(result(1).X(1,1),result(1).X(2,1),'p')
    plot(result(2).X(1,1),result(2).X(2,1),'o')
    arrowPlot(result(2).X(1,1:num),result(2).X(2,1:num), 'number',3,'color','r')
    hold on
    arrowPlot(result(1).X(1,1:num),result(1).X(2,1:num), 'number',3,'color','b')
    xlabel('x(km)');
    ylabel('y(km)');
    legend('目标','追踪航天器','目标起始点','追踪航天器起始点','Location','Northwest')
    %set(h,'FontName','Times New Roman','FontSize',11,'FontWeight','normal','Orientation','horizon','Box','on')
    %title('二维平面')
    axis equal
    grid on
    %set(gca,'FontName','Times New Roman','FontSize',14)
    set(gca,'FontSize',12)
    
    figure;
    plot(tau_int,result(1).cost(1:num),'b','linewidth',2)
    xlabel('时间(s)');
    ylabel('J');
    %title('微分对策支付函数');
    grid on
    %set(gca,'FontName','Times New Roman','FontSize',14)
set(gca,'FontSize',14)
end
