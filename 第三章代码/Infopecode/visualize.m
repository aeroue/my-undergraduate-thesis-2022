% 吴怡
function h = visualize(data)

    figure; 
    plots = [subplot(3, 2, 1:4), subplot(3, 2, 5), subplot(3, 2, 6)];
    subplot(plots(1));
    
    axis([-200 200 -200 200 -200 200]);
    zlabel('距离（km）');
    title('航天器追逃博弈');
    
    animate(data, plots);
end

function animate(data, plots)
    pause(1)
    for t = 1:5:length(data(1).tau_int)
        subplot(plots(1));
        muti3plot(data(1).X, data(2).X, t);
        xlabel('x(km)');
        ylabel('y(km)');
        zlabel('z(km)');
        title('航天器追逃博弈');
        view(30,20)
        drawnow;
               
        subplot(plots(2));
        multiplot(data, data(1).relative_V, t);
        xlabel('时间(s)');
        ylabel('相对速度(m/s)');
        title('相对速度');

        subplot(plots(3));
        multiplot(data, data(1).relative_X, t);
        xlabel('时间(s)');
        ylabel('相对距离(km)');
        title('相对距离');
    end
end

function multiplot(data,values,ind)

    times = data(1).tau_int(:, 1:ind);
    values = values(:, 1:ind);

    plot(times, values(1, :), 'r-', times, values(2, :), 'g.', times, values(3, :), 'b-.');
    
    xmin = min(data(1).tau_int);
    xmax = max(data(1).tau_int);
    ymin = 1.1 * min(min(values));
    ymax = 1.1 * max(max(values));
    axis([xmin xmax ymin ymax]);
end

function muti3plot(values1,values2, ind)
    %times = data(1).tau_int(:, 1:ind);
    values11 = values1(:, 1:ind);
    values22 = values2(:, 1:ind);

    plot3(values11(1, :), values11(2, :), values11(3, :), 'b-.');
    hold on;
    plot3(values22(1, :), values22(2, :), values22(3, :), 'r.');
    
    view(5,30)
    xmin = 1.1 * min(values1(1, :));
    xmax = 1.1 * max(values1(1, :));
    ymin = 1.1 * min(values1(2, :));
    ymax = 1.1 * max(values1(2, :));
    zmin = 1.1 * min(values1(3, :));
    zmax = 1.1 * max(values1(3, :));
    axis([xmin xmax ymin ymax zmin zmax]);
end
