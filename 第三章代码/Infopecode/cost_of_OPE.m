function J = cost_of_OPE(X,Qf,Up,Ue,Rp,Re,num)
% 微分对策支付函数
% Qf 和 Q 均为半正定矩阵
% Qf 是衡量博弈结束时刻两航天器相对状态之差的权重矩阵
% Q 是衡量博弈过程中两航天器相对状态之差的权重矩阵；
% Rp 和 Re 是正定矩阵，分别表征了追踪器和逃逸器能量消耗在支付函数中所占权重
    %X_T = X(step);
    %n = 1;
    J = 0;
    
    for n = 1:num
        X_t = X(n);
        Up_t = Up(n);
        Ue_t = Ue(n);
        dJ = X_t' * Qf * X_t + Up_t' * Rp * Up_t - Ue_t * Re * Ue_t;
        J = J + dJ;
    end
    
end