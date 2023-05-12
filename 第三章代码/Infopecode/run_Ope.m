% 凸优化第三大节
% 作者：吴怡
% 无限时域微分对策
% 双方信息完全已知

%% Clean workspace
clear
close all
clearvars
clearvars -global
clc

%% Initialization
initialize;

%% Orbital Pursuit Escape
[choice1,choice2] = gen_match_result(data.conv);
a = 1;
% 无限时域微分对策鞍点求解 （代数黎卡提方程）
gen_params(1e-17,1e-17)
result.inf_res = get_infinite_planning_horizon_result(params.Number_of_iterations,choice1(a),a);
grapher(result.inf_res)
save('test1.mat','result')

%% Draw
draw1;
grapher(result.inf_res)
%visualize(result.inf_res)

