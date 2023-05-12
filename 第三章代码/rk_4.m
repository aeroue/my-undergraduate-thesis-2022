function  [t,xx] = rk_4(f,X0,a,b,h)
% 定步长四阶 Runge-Kutta 方法
% x0初值
% (a,b)为迭代区间
% N迭代次数
    if a > b
            h = -h;
    end
    
    N = floor((b - a) / h);%求步数
    xx = zeros(length(X0),N);
    xx(:,1) = X0;
    tN = a + N * h;
    t = a:h:tN
    
    for i = 1:N
        tn = t(i);
        xn = xx(:,i);
        K1 = f(tn,xn);
        K2 = f(tn + h/2 , xn + (h / 2) * K1);
        K3 = f(tn + h/2 , xn + (h / 2) * K2);
        K4 = f(tn + h , xn + h * K3);
        xx(:,i+1) = xx(:,i)+ (h / 6) * (K1 + 2 * K2 + 2 * K3 + K4);
    end
    
    xx(:,N+1) = xx(:,N)+((xx(:,N+1)-xx(:,N))/(t(N+1)-t(N)))*(b-t(N));
    
    xx = xx';
end