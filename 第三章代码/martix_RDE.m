function dXdt = martix_RDE(tau, X)

%   martix_RDE.m
%   Made by Wuyi,SCU
%
%   martix Riccati Differential Equation for PE game
%   构造追逃对策的矩阵黎卡提微分方程
%
%   INPUT
%   tau: 时间变量
%   X：黎卡提矩阵
%   params:
%       - params.A = 状态矩阵
%       - params.Qf = 权重矩阵
%       - params.B = 输入矩阵
%       - params.size = A的size
%       - params.R = 权重函数
%
%   注意黎卡提微分方程的变量为末导时间即 params.tf - t backwards in
%
%   LATEX
%   \dot{\boldsymbol{P}}(t)=\boldsymbol{P}(t)\left(\boldsymbol{B}_{P} \boldsymbol{R}_{P}^{-1} \boldsymbol{B}_{P}^{\mathrm{T}}-\boldsymbol{B}_{E} \boldsymbol{R}_{E}^{-1} \boldsymbol{B}_{E}^{\mathrm{T}}\right) \boldsymbol{P}(t)
%   -\boldsymbol{A}^{\mathrm{T}} \boldsymbol{P}(t)-\boldsymbol{P}(t) \boldsymbol{A}-\boldsymbol{Q}
%   终端条件
%   \boldsymbol{P}\left(t_{f}\right)=\boldsymbol{Q}_{f}

    global params

    A = params.A;
    Q = params.Qf;
    B = params.B;
    Re = params.Re;
    Rp = params.Rp;
    siz = params.siz;
%    tf = params.tf;
    
%    t = tf - tau;

    X = reshape(X,siz); 
    dXdt = (A' * X + X * A - X *( B * Rp^(-1) * B' - B * Re^(-1) * B')* X) + Q;
    dXdt = dXdt(:);
 
end
