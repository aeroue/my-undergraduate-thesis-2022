function h = playBox(data1,data2,d)
n = size(data1,1)
if d == 2
    for j = 1:n
        plot(data1(1:j,1),data1(1:j,2),'r','LineWidth',2)
        hold on
        plot(data2(1:j,1),data2(1:j,2),'b-','LineWidth',2)
        %axis equal
        h(:,j) = getframe;
    end
else
    for j = 1:n
        plot3(data1(1:j,1),data1(1:j,2),data1(1:j,3),'r','LineWidth',2)
        hold on
        plot3(data2(1:j,1),data2(1:j,2),data1(1:j,3),'b-.','LineWidth',2)
        view(10,30)
        box on
        grid on
        %axis equal
        h(:,j) = getframe;
        xmin = 1.1 * min(data1(:, 1));
        xmax = 1.1 * max(data1(:, 1));
        ymin = 1.1 * min(data1(:, 2));
        ymax = 1.1 * max(data1(:, 2));
        zmin = 1.1 * min(data1(:, 3));
        zmax = 1.1 * max(data1(:, 3));
        axis([xmin xmax ymin ymax zmin zmax]);
    end
end
end
