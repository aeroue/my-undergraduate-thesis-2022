function Jaccobi = genJacc(y0)
% clc
% clear
% y0 = [4850,-0.3964,0.1027,0.5413];
global CONSTANTS
% 拟牛顿迭代法求解精确值
% 首先求第一次的脱靶量
y0 = y0';
[F,Xf,XSPAN] = endGame(y0);
% if F(1) * F(2) * F(3) < 1 && F(4) < 1e-7
%     yf = y0;
% end
% 雅可比矩阵
af = XSPAN(end,4:6) - XSPAN(end-1,4:6);
dRdtf = Xf(4:6);
dHdtf = y0(2:4)*af';
tf = y0(1);

h = 1e-14;
keshi1 = y0(2:4) + h*[1,0,0]; 
yy1 = [tf,keshi1];
keshi2 = y0(2:4) + h*[0,1,0];
yy2 = [tf,keshi2];
keshi3 = y0(2:4) + h*[0,0,1];
yy3 = [tf,keshi3];
[~,xff1,~] = endGame(yy1);
[~,xff2,~] = endGame(yy2);
[~,xff3,~] = endGame(yy3);
dxdlmd1 = (xff1 - Xf)/h;
dxdlmd2 = (xff2 - Xf)/h;
dxdlmd3 = (xff3 - Xf)/h;

dvdlmdtf = [dxdlmd1(4:6);dxdlmd2(4:6);dxdlmd3(4:6)]';
dRdlmdtf = [dxdlmd1(1:3);dxdlmd2(1:3);dxdlmd3(1:3)]';

dHdlmdtf = Xf(4:6) + y0(2:4) * dvdlmdtf;

Jaccobi = zeros(4,4);
Jaccobi(1:3,1) = dRdtf';
Jaccobi(1:3,2:4) = dRdlmdtf;
Jaccobi(4,1) = dHdtf;
Jaccobi(4,2:4) = dHdlmdtf
end
    




    