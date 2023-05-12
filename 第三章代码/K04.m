clear 
clc
close all


% 绘制带箭头图形
t = [0:0.01:20];
x = t.*cos(t);    
y = t.*sin(t);
arrowPlot(x, y, 'number', 3)   % 'number', 3 表示曲线上均匀分布3个箭头标识

% 绘制带箭头图形
t = [0:0.01:10];
x = cos(t);
arrowPlot(t, x, 'number', 6)


% 绘制带箭头图形 + 箭头坐标轴
plot_with_arrow();     % 使用plot_with_arrow绘制箭头坐标轴
hold on
t = [0:0.01:20];
x = t.*cos(t);
y = t.*sin(t);
arrowPlot(x, y, 'number', 5, 'color', 'r', 'LineWidth', 1, 'scale', 0.8, 'ratio', 'equal');
Arrow_Xlabel([],'X/m','pp');    % x轴标签
Arrow_Ylabel([],'Y/m','pp');    % y轴标签

% 绘制带箭头图形 + 箭头坐标轴
plot_with_arrow([],[],[],[],'aa');     % 使用plot_with_arrow绘制箭头坐标轴
hold on
t = [0:0.01:20];
x = t.*cos(t);
y = t.*sin(t);
arrowPlot(x, y, 'number', 5, 'color', 'r', 'LineWidth', 1, 'scale', 0.8, 'ratio', 'equal');


% 绘制带箭头图形 + 箭头坐标轴
plot_with_arrow([],[],[],[],'aa');     % 使用plot_with_arrow绘制箭头坐标轴
hold on
t = linspace(0,2*pi,600);
x = cos(t);
y = sin(t);
plot(x,y,'r')       % 绘制直径1圆
plot(3*x,3*y,'r')   % 绘制直径3圆
ind = 25 + 0:100:600;
for ii = 1:length(ind)   % 循环绘制斜线
    hold on
    arrowPlot([3*x(ind(ii)) x(ind(ii))], [3*y(ind(ii)) y(ind(ii))], 'number', 2, 'color', 'r');
end
axis equal

