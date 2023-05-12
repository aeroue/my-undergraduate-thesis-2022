% draw1.m
S1 = [30 30 30];
S2 = [100 100 100];
theta = 0:1:360;
th_rad = deg2rad(theta);
phi = ones(1,361).*90;
phi_rad = deg2rad(phi);
circle_point = polar2cartesian(data.nearby_dist,th_rad,phi_rad);


figure
scatter(data.circle_point(1,:),data.circle_point(2,:),S1,[0 1 1],'filled')
hold on
scatter(0,0,100,[1 0 0],'filled')
hold on
plot(data.position_p0(1,:),data.position_p0(2,:),'bs')
hold on
plot(circle_point(1,:),circle_point(2,:))
hold on
xlabel('X(m)');
ylabel('Y(m)');
legend('Î§²¶µã','ÌÓÒÝº½ÌìÆ÷','×·×Ùº½ÌìÆ÷','30km¹Û²âÈ¦')
axis([-60,60,-60,60])
set(gca,'XTick',-60:10:60,'YTick',-60:10:60)
set(gcf,'position',[500,50,500,500])
