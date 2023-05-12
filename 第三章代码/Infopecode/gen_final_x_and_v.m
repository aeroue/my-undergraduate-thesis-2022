function [Xe, Xp, Ve, Vp] = gen_x_and_v(U,re_X)
    [Xe, Ve] = gen_final_x_and_v(U);
    [Xp, Vp] = [Xe, Ve] + re_X;
end


function [X, V] = gen_final_x_and_v(U)
    global params
    A = params.A;
    B = params.B;
    tau_int = params.tau_int;
    num = size(tau_int);
    dt = params.time_step;
    X(1,:) = zeros(1,6);
    for i = 2:num
        X(i,:) = X(i-1,:) + dt * (A * X(i-1,:)+B * U(i-1));
    end
    
    X = X(1:num,1:3);
    V = X(1:num,1:3);
end
 