% initialize.m
% made by wuyi

%% Set Path
% global
filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end

addpath(genpath(currentPath));
addpath(genpath('constants'));
load('earth_constants.mat');

%% global var&&constant
%format long;
global earth unit data params pi
pi = 3.1415926;

%% 正则单位
Href = 2000000;
Rearth = earth.R;
unit.du = Rearth + Href; % 距离正则单位
[unit.vu,unit.tu,unit.au,unit.wu] = gen_RefGEO_params(earth.mu,unit.du);% 速度正则单位

%% 围捕点分配
data.Np = 3;
data.Ne = 1;

%初始位置、速度
data.position_e0 = [0,0,0];
data.velocity_e0 = [0,0,0];
data.nearby_dist = 30;  %km
data.theta = [0 120 240];
th_rad = deg2rad(data.theta);
data.phi = [90 90 90];
phi_rad = deg2rad(data.phi);
data.circle_point = polar2cartesian(data.nearby_dist,th_rad,phi_rad);
data.relative_dist0 = [-6 -3 3;-16.6 7 -15;4 4 4];
data.position_p0 = data.circle_point + data.relative_dist0;
data.ralative_velocity0 = [3;-7.5;0];
data.velocity_p0 = data.velocity_e0 .* ones(3,3) + data.ralative_velocity0 .* ones(3,3);
[data.conv,data.dis0] = gen_pe_distance(data.circle_point,data.position_p0);

%% 相对运动方程
omega = unit.wu;
A11 = zeros(3,3);
A12 = eye(3,3);
A21 = [ 3*omega^2    0       0;
             0       0       0;
             0       0       -omega^2];
A22 = [  0       2*omega     0;
        -2*omega     0       0;
             0       0       0 ];
params.A = [ A11      A12;
    A21      A22];
B11 = zeros(3,3);
B21 = eye(3,3);
params.B = [ B11;
    B21];
params.X0 = [data.dis0*1000;-data.ralative_velocity0.*ones(3,9)];
params.Pur0 = [data.position_p0*1000;-data.ralative_velocity0.*ones(3,3)];
params.Eva0 = [data.circle_point*1000;[0;0;0].*ones(3,3)];
params.siz = size(params.A);

%% 微分对策方程
% 初始值
params.t0 = 0;
params.tf = unit.tu; %5553.63s(另外写一个轨道参数函数)
params.tf2 = 2 * unit.tu; %5553.63s(另外写一个轨道参数函数)
params.time_step = 5;
params.riccati_time_step = 5;
params.riccati_tf = params.tf + params.riccati_time_step; %5553.63s(另外写一个轨道参数函数)
params.N = numel(params.t0:params.riccati_time_step:params.tf) + 1;
params.tau_int = params.t0:params.riccati_time_step:params.tf;
params.tau_int2 = params.t0:params.riccati_time_step:params.tf2;
% 加权矩阵
% s1 = 1e-16;
% s2 = 0;
% s3 = 0;
% rp11 = 1e-5;
% rp22 = 1e-5;
% rp33 = 1e-5;
% [params.Qf,params.Pf,params.Rp,params.Re,params.R,...
%     params.Number_of_iterations] = gen_Weighting(s1,s2,s3,rp11,rp22,rp33);
% 运动方程
AA11 = zeros(3,3);
AA12 = eye(3,3);
AA21 = [  -omega^2    0        0;
             0   -omega^2     0;
             0       0       -omega^2];
params.AA = [ AA11      AA12;
    AA21      AA11];





