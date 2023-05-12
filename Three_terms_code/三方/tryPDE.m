%% 尝试写一下航天器三方博弈，主要借鉴faruqi的书及代码
%% 吴怡
%% 2022.05.03
%% Target - Defender - Attacker 

clear all
clc

%% 参数
% 基本参数
Re                  =   6378.137 * 1000;                
miu                 =   3.986004418E14;               
Href                =   2000 * 1000;                    
Rref                =   Re + Href;       
Omegaref            =   sqrt(miu / Rref^3); 
% 初始参数
Xp = [-6*1000; -16*1000; 4*1000; -9; 13.6; 0];
Xe = [0; 0; 0; 0; 0; 0];
g = 9.78;
TpMax = 0.0006 * g;
TeMax = 0.0004 * g;
X0 = Xe - Xp;
step = 10;
i_num = 3;
j_num = 3;

%% 常数参数
global CONSTANTS
CONSTANTS.t0 = 0;
CONSTANTS.tf = 8000;                            %s
CONSTANTS.refOmega  =   Omegaref;
CONSTANTS.TpMax = TpMax;
CONSTANTS.TeMax = TeMax;
CONSTANTS.stepT     =   step;
CONSTANTS.X0 = X0;
CONSTANTS.X0(1:3) = X0(1:3);
CONSTANTS.X0(4:6) = X0(4:6);
CONSTANTS.Xp = Xp;
CONSTANTS.Xp(1:3) = Xp(1:3);
CONSTANTS.Xp(4:6) = Xp(4:6);
CONSTANTS.Xe = Xe;
CONSTANTS.Xe(1:3) = Xe(1:3);
CONSTANTS.Xe(4:6) = Xe(4:6);
CONSTANTS.t_int=CONSTANTS.t0:step:CONSTANTS.tf;
CONSTANTS.T_index=length(CONSTANTS.t_int);
CONSTANTS.i_num = i_num;
CONSTANTS.j_num = j_num;

%% 创建交战运动学模型

