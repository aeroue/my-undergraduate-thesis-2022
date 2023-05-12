function  [x,n,data]=nyNew(x0,tol,tol2)
if nargin==1
tol=8.3e-5;
tol2=3.14;
end

H0=genJacc(x0);
H0=inv(H0);
x1=x0-0.1*H0*endGame(x0);

n=1;
F = endGame(x0);
%设置初始误差 使之可以进入循环
wucha = 100;
wucha2 = 100; 
%循环迭代
while   (wucha>tol)||(wucha2>tol2)
%while  norm(F(1:3))<1 
%wucha=norm(x1-x0);
H1=inv(genJacc(x1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%这一段相当重要 技巧性也比较强  请体会
temp_x0=x1;  
x1=temp_x0-0.1*H1*endGame(temp_x0)   %x1的更新
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%更新H矩阵
H=H1;
n=n+1;
%data用来存放中间数据
F = endGame(x1);
wucha = norm(F(4))
wucha2 = norm([F(1),F(2),F(3)])
data(:,n)=x1;
end
x=x1;
end