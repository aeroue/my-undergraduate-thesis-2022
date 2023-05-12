function [Xe, Xp, Ve, Vp] = gen_x_and_v(U,re_X,tau_int,time_step,Eva0)
    global params
    num = size(tau_int,2);
    [Xe, Ve] = gen_final_x_and_v(U,tau_int,time_step,Eva0);
    Xp = Xe - re_X(1:num,1:3)';
    Vp = Ve - re_X(1:num,4:6)';
end


function [Xf, Vf] = gen_final_x_and_v(U,tau_int,time_step,Eva0)
    global params
    global earth
    A = params.AA;
    B = params.B;
    num = size(tau_int,2);
    dt = time_step;
    Xi(:,1) = Eva0';

    for i = 2:num
        dx = dt * (A * Xi(:,i-1)+B * U(:,i));
        Xi(:,i) = Xi(:,i-1) + dx;
    end
    
    Xf = Xi(1:3,1:num);
    Vf = Xi(4:6,1:num);
end

% function X_dot = state(t, X, U)
%     global params
%     A = params.A;
%     B = params.B;
%     X_dot = A * X + B * U
% end
 