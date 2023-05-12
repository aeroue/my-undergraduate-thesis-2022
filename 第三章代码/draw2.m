% draw2.m
function grapher(result)
    tau_int = result.inf_res.tau_int;
    deltX_tol = sqrt(result.inf_res(1).relative_X(1,:)^2 + ...
        result.inf_res(1).relative_X(2,:)^2 + ...
        result.inf_res(1).relative_X(3,:)^2);
    deltV_tol = sqrt(result.inf_res(1).relative_V(1,:)^2 + ...
        result.inf_res(1).relative_X(V,:)^2 + ...
        result.inf_res(1).relative_X(V,:)^2);
    
    
    figure(1);
    subplot(2, 1, 1);
    plot(tau_int,deltX_tol)
    xmin = min(tau_int);
    xmax = max(tau_int);
    ymin = 1.1 * min(min(deltX_tol));
    ymax = 1.1 * max(max(deltX_tol));
    axis([xmin xmax ymin ymax]);
    xlabel('时间(s)');
    ylabel('相对距离(km)');
    title('相对距离');    
    subplot(2, 1, 2);
    plot(tau_int,deltV_tol)
    xmin = min(tau_int);
    xmax = max(tau_int);
    ymin = 1.1 * min(min(deltV_tol));
    ymax = 1.1 * max(max(deltV_tol));
    axis([xmin xmax ymin ymax]);
    xlabel('时间(s)');
    ylabel('相对速度(m/s)');
    title('相对距离');  


    figure(2);
    subplot(3, 1, 1);
    plot(tau_int,result.inf_res(1).control(:,1))
    xlabel('时间(s)');
    ylabel('追踪者Ue_x');
    subplot(3, 1, 2);
    plot(tau_int,result.inf_res(1).control(:,2))
    xlabel('时间(s)');
    ylabel('追踪者Ue_y');
    subplot(3, 1, 3);
    plot(tau_int,result.inf_res(1).control(:,3))
    xlabel('时间(s)');
    ylabel('追踪者Ue_z');
    title('追踪者控制力加速度')
    
    figure(3);
    subplot(3, 1, 1);
    plot(tau_int,result.inf_res(2).control(:,1))
    xlabel('时间(s)');
    ylabel('追踪者Ue_x');
    subplot(3, 1, 2);
    plot(tau_int,result.inf_res(2).control(:,2))
    xlabel('时间(s)');
    ylabel('追踪者Ue_y');
    subplot(3, 1, 3);
    plot(tau_int,result.inf_res(2).control(:,3))
    xlabel('时间(s)');
    ylabel('追踪者Ue_z');
    title('逃跑者控制力加速度')
    
    figure(4);
    plot(tau_int,result.inf_res(2).cost)
    xlabel('时间(s)');
    ylabel('微分对策支付函数');
end
