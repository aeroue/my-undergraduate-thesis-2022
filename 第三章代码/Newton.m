%拟牛顿法的主函数
initialize
x0=[-0.003 -0.205 0.033 6472.2]';
[x,n,data]=broyden(x0);
disp('计算结果为')
x
disp('迭代次数为')
n
%抽取data1中第一个变量数据 画出曲线
subplot(3,1,1)
plot(data(1,:)),title('x1在迭代中的变化')
%抽取data中的第二个变量数据 画出其变化曲线
subplot(3,1,2)
%v=[1 n -0.05 0.05];  %用于控制坐标轴范围 使图象更清晰
%plot(data(2,:)),axis(v),title('x2在迭代中的变化')
plot(data(2,:)),title('x2在迭代中的变化')
%抽取第三个变量数据
subplot(3,1,3)
plot(data(3,:)),title('x3在迭代中的变化')

%以下为数据存储部分 
num=(1:n)';
a=[num data'];
save data1.txt a -ascii

%拟牛顿法（broyden） 计算非线性方程组
%输入 x0为迭代初值
%tol为误差容限  如果缺省 默认为10的-10次方
%data用来存放计算的中间数据便于计算收敛情况分析
function  [x, n, data] = broyden(x0, tol)
    if nargin == 1
        tol = 1e-5;
    end

    
% 第一次
    [F,H0] = fun(x0);
    H0 = inv(H0)
    
    x1 = x0 - H0 * F;

    n=1;

%设置初始误差 使之可以进入循环
    wucha=0.1;
%循环迭代
    while   (wucha>tol)&(n<20)&(n<500)

        wucha=norm(x1-x0);

        dx=x1-x0;
        [F1,~] = fun(x1);
        [F2,~] = fun(x0);
        y = F1 - F2;

        fenzi = dx'*H0.*y; %是标量
        H1 = H0+(dx-H0*y)*(dx)'*H0/fenzi;

        temp_x0 = x0;  
        x0 = x1;
        Ff = fun(temp_x0);
        x1 = temp_x0-H1* Ff;   %x1的更新

%更新H矩阵
        H=H1;
        n=n+1;
%data用来存放中间数据
        data(:,n)=x1;
        end
    x=x1;
end

function [F,J] = fun(x0)
    global params
    A = params.A;
    B = params.B;
    v1 = x0(1);
    v2 = x0(2);
    v3 = x0(3);
    tf = x0(4);
    
    g = 9.78;
    Tei = 0.0004*g;
    Tpi = 0.0006*g;
    xx =[-16400;-60300;7200;1.5;4.2;0]; 
    step = 1;
    tau_int = 0:step:tf;
    lamda_tf = [v1;v2;v3;0;0;0];
    
    options = odeset('AbsTol', 1e-12, 'RelTol', 1e-8);
    [~,lamda_aut] = ode45(@(t,lamda)lamda_dot(t,lamda),tau_int,lamda_tf,options);
    lamda_t = lamda_aut(end:-1:1,:);
    num = size(lamda_t,1);
    
    for i = 1:num-1
        
    lamdat = lamda_t(i,:);
    
    salphap = lamdat(5)/sqrt(lamdat(4)^2+lamdat(5)^2);
    calphap = lamdat(4)/sqrt(lamdat(4)^2+lamdat(5)^2);
    sbetap = lamdat(6)/sqrt(lamdat(4)^2+lamdat(5)^2+lamdat(6)^2);
    cbetap = sqrt(lamdat(4)^2+lamdat(5)^2)/sqrt(lamdat(4)^2+lamdat(5)^2+lamdat(6)^2);
    salphae = salphap;
    calphae = calphap;
    sbetae = sbetap;
    cbetae = cbetap;
    alhpa(i)=180+atan2(salphae,calphae)*180/pi;
    beta(i) = atan2(sbetae,cbetae)*180/pi;
    ae = Tei .* [cbetae*calphae;cbetae*salphae;sbetae];
    ap = Tpi .* [cbetap*calphap;cbetap*salphap;sbetap];

    U = ae - ap;
    
    if isnan(U)
        U = (Tei - Tpi).*ones(3,1);
    end
    
    dd = A * xx;
    dx = A * xx + B * U;
    
    xx = xx + step*dx;
    end
    
    x_tf = xx;
    xx(1:3)
    uu = dx;
    
    
    f1 = Hamilton_tf(v1,v2,v3,x_tf) + 1
    f2 = x_tf(1);
    f3 = x_tf(2);
    f4 = x_tf(3);
%     f2 = 4-3*cos(tau)*xx(1);
%     f3 = 
    F=[f1;f2;f3;f4]
    X0=[-0.003 -0.205 0.033 6472.2]';
    omega = 7.292124577897024e-05;
    tau = omega * tf;
    j11 = x_tf(4);
    j12 = x_tf(5);
    j13 = x_tf(6);
    j14 = uu(4)*v1 + uu(5)*v2 + uu(6)*v3;   
    j14 = 0;
    j21 = 0;
    j22 = 0;
    j23 = 0;
    jj = qiudao(tf);
    j24 = uu(4);
    j34 = uu(5);
    j44 = uu(6);
    j31 = 0;
    j32 = 0;
    j33 = 0;
    j41 = 0;
    j42 = 0;
    j43 = 0;
   
    %j24 = 3*X0(1)*omega*sin(tau)+X0(4)*omega*cos(tau)+2*sin(tau)*X0(5)/omega;
    %j34 = ;
    
    J = [j11 j12 j13 j14;
         j21 j22 j23 j24;
         j31 j32 j33 j34;
         j41 j42 j43 j44]
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

