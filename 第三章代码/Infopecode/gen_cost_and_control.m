function [Ue, Up ,J] = gen_cost_and_control(X,P,a)

% 微分对策支付函数
% Qf 和 Q 均为半正定矩阵
% Qf 是衡量博弈结束时刻两航天器相对状态之差的权重矩阵
% Q 是衡量博弈过程中两航天器相对状态之差的权重矩阵；
% Rp 和 Re 是正定矩阵，分别表征了追踪器和逃逸器能量消耗在支付函数中所占权重
    global params
    A = params.A;
    B = params.B;
    Re = params.Re;
    Rp = params.Rp;
    Qf = params.Qf;
    siz = params.siz;    
    num = size(X,1);

    %X_T = X(step);
    %n = 1;
    Up = zeros(3,num);
    Ue = zeros(3,num);
    J = zeros(1,num);
    J(1) = 0;  
    
    for n = 1:num
        X_t = X(n,:)';
        if a == 1
            Up(:,n+1) = Rp^(-1) * B'* P * X_t;
            Ue(:,n+1) = 0;
            %Ue(:,n+1) = Re^(-1) * B'* P * X_t;
            %Ue(:,n+1) = [cos((n+1)*pi/10);cos((n+1)*pi/10);cos((n+1)*pi/10)];
            dJ = X_t' * Qf * X_t + Up(:,n+1)' * Rp * Up(:,n+1) - Ue(:,n+1)' * Re * Ue(:,n+1);
            J(n+1) = J(n) + dJ;
        else
                    P1 = reshape(P(n,:),siz);
                    Up(:,n+1) = Rp^(-1) * B'* P1 * X_t;
                    %Ue(:,n+1) = [cos((n+1)*pi/10);cos((n+1)*pi/10);cos((n+1)*pi/10)];
                    Ue(:,n+1) = 0;
                    %Ue(:,n+1) = Re^(-1) * B'* P1 * X_t;
                    dJ = X_t' * Qf * X_t + Up(:,n+1)' * Rp * Up(:,n+1) - Ue(:,n+1)' * Re * Ue(:,n+1);
                    J(n+1) = J(n) + dJ;
                    
        end
    end
    
end