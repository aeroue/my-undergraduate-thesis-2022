
%%*************************faruqi_dgt_DEMO*********************************
% ************AIR-AIR MULTI PARTY GAME SIMULTION **************************
% Creator: F.Faruqi
% Single Run Version
% Includes Autopilot:
% Intelligent Weapons Group.
% Version-1: October 2012; Updated: July 2015; Data for IPHS DEMO SEM
% While the Author has verified the accuracy of the program; no
% guarantee/warranty is expressed or implied. The user should verify the
% correctness of this simulation for his particular application.
% *************************************************************************
% *************************************************************************

%%=========================================================================
% 10.10.10: Simulation Parameters Values ==============================+===
% =========================================================================
t0=0; % Simulation start time
tf=30; % Simulation final time.Test 1-7.
del_t=.0005; % Simulation integration step (Trapeziodal Rule Implemented).
t=t0:del_t:tf; % Simulation time.
T_index=length(t);
%==========================================================================

%%=========================================================================
% 10.10.20: Number of Vehicles Values ========+============================
% +++++++++++==============================================================
i_num=3; % Number of vehicles involved in the engagement.
j_num=3; % Number of vehicles involved in the engagement.
% For this simulation (3-party Game Simuulation), the following definition
% is used:
% VEHICLE INDEX 1: High Value (Aircraft) Target;
% VEHICLE INDEX 2: Defender (Missile) or Pursuer; with the purpose of
% intercepting the attcker-2.
% VEHICLE INDEX 3: Attacker (Missile) evader against the defender-2, while 
% attacking the high value target-1, while evading the defender.
%==========================================================================

%%=========================================================================
% 10.10.30: Set Up Vehicle Body-Axis States ===============================
%==========================================================================

%10.10.30.10: Position Vector:
x_b=zeros(i_num,T_index);
y_b=zeros(i_num,T_index);
z_b=zeros(i_num,T_index);

% 10.10.30.20: Velocity Vector:
u_b=zeros(i_num,T_index);
v_b=zeros(i_num,T_index);
w_b=zeros(i_num,T_index);

% 10.10.30.30: Acceleration Vector:
ax_b=zeros(i_num,T_index);
ay_b=zeros(i_num,T_index);
az_b=zeros(i_num,T_index);

% 10.10.30.40: Acceleration Deritivr Vector:
ax_b_dot=zeros(i_num,T_index);
ay_b_dot=zeros(i_num,T_index);
az_b_dot=zeros(i_num,T_index);

% 10.10.30.50: Demanded Acceleration Vector(Autopilot Input):
ax_b_dem=zeros(i_num,T_index);
ay_b_dem=zeros(i_num,T_index);
az_b_dem=zeros(i_num,T_index);

% Body-Axis Rotation Rate Vector:
p_b=zeros(i_num,T_index);
q_b=zeros(i_num,T_index);
r_b=zeros(i_num,T_index);

% Total-Body Axis Velocity and Acceleration:
V_b=zeros(i_num,T_index);
V_b_sq=zeros(i_num,T_index);

A_b_sq=zeros(i_num,T_index);
A_b=zeros(i_num,T_index);

%% INPUT VALUES=============================================================
% Body-Axis Velocity Vector Values:
 u_b(1,1) = 660.0000; %Baseline
 u_b(2,1) = 990.0000; %Baseline
 u_b(3,1) = 990.0000; %Baseline


v_b(1,1) = 0.0;
v_b(2,1) = 0.0;
v_b(3,1) = 0.0;

w_b(1,1) = 0.0;
w_b(2,1) = 0.0;
w_b(3,1) = 0.0;

% Body-Axis Acceleration Vector Values:
ax_b(1,1)=0.0;
ax_b(2,1)=0.0;
ax_b(3,1)=0.0;

ay_b(1,1)=0.0;
ay_b(2,1)=0.0;
ay_b(3,1)=0.0;

az_b(1,1)=0.0;
az_b(2,1)=0.0;
az_b(3,1)=0.0;

% Body-AxisRotation Vector Values:
p_b(1,1)=0.0;
q_b(2,1)=0.0;
r_b(3,1)=0.0;
%==========================================================================

% Compute Body Axis Total Velocity, Acceleration & Body Rates:
for i=1:i_num;
    V_b(i,1)=sqrt(u_b(i,1)*u_b(i,1)+v_b(i,1)*v_b(i,1)+w_b(i,1)*w_b(i,1));
    V_b_sq(i,1)=V_b(i,1)*V_b(i,1);
    
    A_b_sq(i,1)=(ax_b(i,1)*ax_b(i,1)+ay_b(i,1)*ay_b(i,1)+az_b(i,1)*az_b(i,1));
    A_b(i,1)=sqrt(A_b_sq(i,1));
end

for i=1:i_num;
    p_b(i,1)=(v_b(i,1)*az_b(i,1)-w_b(i,1)*ay_b(i,1))/V_b_sq(i,1);
    q_b(i,1)=(w_b(i,1)*ax_b(i,1)-u_b(i,1)*az_b(i,1))/V_b_sq(i,1);
    r_b(i,1)=(u_b(i,1)*ay_b(i,1)-v_b(i,1)*ax_b(i,1))/V_b_sq(i,1);
end

%% 10.30. Vehicle Heading (Euler) Angles and Quaternions*******************
phi=zeros(i_num,T_index);
theta=zeros(i_num,T_index);
psi=zeros(i_num,T_index);

% INPUT VALUES============================================================+
phi(1,1)=0*pi/180;
phi(2,1)=0*pi/180;
phi(3,1)=0*pi/180;

psi(1,1)=0*pi/180;
theta(1,1)=0*pi/180;

psi(2,1)=0*pi/180;
theta (2,1)=0*pi/180;

psi(3,1)=0*pi/180;
theta(3,1)=0*pi/180;


%% 10.40. Vehicle Fixed_Axis States ***************************************
% Fixed-Axis Position Vector:
x_i=zeros(i_num,T_index);
y_i=zeros(i_num,T_index);
z_i=zeros(i_num,T_index);

% Fixed-Axis Velocity Vector:
u_i=zeros(i_num,T_index);
v_i=zeros(i_num,T_index);
w_i=zeros(i_num,T_index);

% Fixed-Axis Acceleration Vector:
ax_i=zeros(i_num,T_index);
ay_i=zeros(i_num,T_index);
az_i=zeros(i_num,T_index);

% Fixed Axis Demanded Acceleration (Guidance Demands)
ax_i_dem=zeros(i_num,T_index);
ay_i_dem=zeros(i_num,T_index);
az_i_dem=zeros(i_num,T_index);

% Fixed-Axis Rotation Rate Vector:
p_i=zeros(i_num,T_index);
q_i=zeros(i_num,T_index);
r_i=zeros(i_num,T_index);

% Fixed-Axis Total Range, Velocity and Acceleration:
R_i_sq=zeros(i_num,T_index);
R_i=zeros(i_num,T_index);

V_i_sq=zeros(i_num,T_index);
V_i=zeros(i_num,T_index);

A_i_sq=zeros(i_num,T_index);
A_i=zeros(i_num,T_index);

% INPUT VALUES=============================================================
x_i(1,1)=5000.0000; %target Baseline
x_i(2,1)=5000.0000; %defender Baseline
x_i(3,1)=15000.0000; % target Baseline

y_i(1,1)=5000.0000;
y_i(2,1)=5000.0000;
y_i(3,1)=5000.0000;

z_i(1,1)=-8000.0000;
z_i(2,1)=-8000.0000;
z_i(3,1)=-0000.0000;
%==========================================================================

%% 10.50. Vehicle Fixed-Axis Relative States ******************************
rel_x_i=zeros(i_num,j_num,T_index);
rel_y_i=zeros(i_num,j_num,T_index);
rel_z_i=zeros(i_num,j_num,T_index);

rel_u_i=zeros(i_num,j_num,T_index);
rel_v_i=zeros(i_num,j_num,T_index);
rel_w_i=zeros(i_num,j_num,T_index);

rel_ax_i=zeros(i_num,j_num,T_index);
rel_ay_i=zeros(i_num,j_num,T_index);
rel_az_i=zeros(i_num,j_num,T_index);

rel_R1_i_sq=zeros(i_num,j_num,T_index);
rel_R1_i=zeros(i_num,j_num,T_index);

rel_R_i_sq=zeros(i_num,j_num,T_index);
rel_R_i=zeros(i_num,j_num,T_index);

rel_V_i_sq=zeros(i_num,j_num,T_index);
rel_V_i=zeros(i_num,j_num,T_index);

rel_A_i_sq=zeros(i_num,j_num,T_index);
rel_A_i=zeros(i_num,j_num,T_index);

rel_R1_i_dot=zeros(i_num,j_num,T_index);
rel_R_i_dot=zeros(i_num,j_num,T_index);

% Relative Azimuth, Elevation LOS Angles & Closing Velocity:
rel_theta_los_i=zeros(i_num,j_num,T_index);
rel_psi_los_i=zeros(i_num,j_num,T_index);

rel_theta_los_i_dot=zeros(i_num,j_num,T_index);
rel_psi_los_i_dot=zeros(i_num,j_num,T_index);

clos_vel=zeros(i_num,j_num,T_index);

% Compute Relative Positons & LOS Angles:
for i = 1:i_num;
    for j =1:j_num;
        if(i~=j);
            rel_x_i(i,j,1) = x_i(i,1)-x_i(j,1);
            rel_y_i(i,j,1) = y_i(i,1)-y_i(j,1);
            rel_z_i(i,j,1) = z_i(i,1)-z_i(j,1);
            
            rel_R1_i_sq(i,j,1)=(rel_x_i(i,j,1)*rel_x_i(i,j,1)+...
                rel_y_i(i,j,1)*rel_y_i(i,j,1));
            rel_R1_i(i,j,1)=sqrt(rel_R1_i_sq(i,j,1));
            
            rel_R_i_sq(i,j,1)=(rel_R1_i_sq(i,j,1)+rel_z_i(i,j,1)*rel_z_i(i,j,1));
            rel_R_i(i,j,1)=sqrt(rel_R_i_sq(i,j,1));
            
            rel_psi_los_i(i,j,1) = atan2(rel_y_i(i,j,1),rel_x_i(i,j,1));
            rel_theta_los_i(i,j,1) = atan2(-rel_z_i(i,j,1),rel_R1_i(i,j,1));
        end
    end
end

%% 10.60. Collision Course Headings ***************************************
beta = zeros(i_num,j_num,T_index);
beta_cc = zeros(i_num,j_num,T_index);

cos_cos_cc = zeros(i_num,j_num,T_index);
cos_sin_cc = zeros(i_num,j_num,T_index);

VC_cc = zeros(i_num,j_num,T_index);

theta_cc =  zeros(i_num,j_num,T_index);
psi_cc =  zeros(i_num,j_num,T_index);

theta_he=zeros(i_num,T_index);
psi_he=zeros(i_num,T_index);

% INPUT VALUES
% =========================================================================
defender_cc_override=0;
%==========================================================================

% Heading Error Values for Computing Collision Course:

 theta_he(3,1)=5*pi/180; %Baseline
 psi_he(3,1)=5*pi/180;   %Baseline
 theta_he(2,1)=5*pi/180;  %Baseline
 psi_he(2,1)=5*pi/180;   %Baseline

% Compute Collision Course Headings (Attacker):
for i_target= 1:2;
    if(i_target==1);
        j=1; % target
        i=3; % attacker
    end
    % Compute Collision Course Headings (Defender):
    if(i_target==2);
        j=3;
        i=2;
        %Compute Defender Heading Based on Attacker's HE
        psi(3,1) = psi_cc(3,1,1)+psi_he(3,1);
        theta(3,1) = theta_cc(3,1,1)+theta_he(3,1);
    end
    A=cos(theta(j,1))*cos(psi(j,1))*cos(rel_theta_los_i(j,i,1))*...
        cos(rel_psi_los_i(j,i,1));
    B=cos(theta(j,1))*sin(psi(j,1))*cos(rel_theta_los_i(j,i,1))*...
        sin(rel_psi_los_i(j,i,1));
    C=sin(theta(j,1))*sin(rel_theta_los_i(j,i,1));
    D=A+B+C;
    beta(j,i,1)=acos(D);
    beta_cc(i,j,1) = asin(u_b(j,1)*sin(beta(j,i,1))/u_b(i,1));
    VC_cc(i,j,1) = u_b(i,1)*cos(beta_cc(i,j,1))-u_b(j,1)*...
        cos(beta(j,i,1));
    theta_cc(i,j,1) = asin((VC_cc(i,j,1)/u_b(i,1))*...
        sin(rel_theta_los_i(j,i,1))+(u_b(j,1)/u_b(i,1))*...
        sin(theta(j,1)));
    cos_cos_cc(i,j,1) = (VC_cc(i,j,1)/u_b(i,1))*...
        cos(rel_theta_los_i(j,i,1))*cos(rel_psi_los_i(j,i,1))+...
        (u_b(j,1)/u_b(i,1))*cos(theta(j,1))*cos(psi(j,1));
    cos_sin_cc(i,j,1) = (VC_cc(i,j,1)/u_b(i,1))*...
        cos(rel_theta_los_i(j,i,1))*sin(rel_psi_los_i(j,i,1))+...
        (u_b(j,1)/u_b(i,1))*cos(theta(j,1))*sin(psi(j,1));
    psi_cc(i,j,1)=atan2(cos_sin_cc(i,j,1),cos_cos_cc(i,j,1));
    xxx=1;
end

% Heading Error Values for Defender Post-Collision Course Computation:
theta(2,1) = theta_cc(2,3,1)+theta_he(2,1);
psi(2,1) = psi_cc(2,3,1)+psi_he(2,1);

if(defender_cc_override==1);
    theta(2,1) = theta(1,1)+theta_he(2,1);
    psi(2,1) = psi(1,1)+psi_he(2,1);
end

%% 10.70. Compute Quaternions:*********************************************
% Compute Quaternion Definition and Transformation Matrix (DCM):
quat1=zeros(i_num,T_index);
quat2=zeros(i_num,T_index);
quat3=zeros(i_num,T_index);
quat4=zeros(i_num,T_index);
quat_sq=zeros(i_num,T_index);
quat=zeros(i_num,T_index);

t11_bi=zeros(i_num,T_index);
t12_bi=zeros(i_num,T_index);
t13_bi=zeros(i_num,T_index);
t21_bi=zeros(i_num,T_index);
t22_bi=zeros(i_num,T_index);
t23_bi=zeros(i_num,T_index);
t31_bi=zeros(i_num,T_index);
t32_bi=zeros(i_num,T_index);
t33_bi=zeros(i_num,T_index);

for i=1:i_num;
    quat1(i,1)=cos(phi(i,1)/2)*cos(theta(i,1)/2)*cos(psi(i,1)/2)...
        +sin(phi(i,1)/2)*sin(theta(i,1)/2)*sin(psi(i,1)/2);
    quat2(i,1)=sin(phi(i,1)/2)*cos(theta(i,1)/2)*cos(psi(i,1)/2)...
        -cos(phi(i,1)/2)*sin(theta(i,1)/2)*sin(psi(i,1)/2);
    quat3(i,1)=cos(phi(i,1)/2)*sin(theta(i,1)/2)*cos(psi(i,1)/2)...
        +sin(phi(i,1)/2)*cos(theta(i,1)/2)*sin(psi(i,1)/2);
    quat4(i,1)=cos(phi(i,1)/2)*cos(theta(i,1)/2)*sin(psi(i,1)/2)...
        -sin(phi(i,1)/2)*sin(theta(i,1)/2)*cos(psi(i,1)/2);
    
    quat_sq(i,1)=quat1(i,1)*quat1(i,1)+quat2(i,1)*quat2(i,1)+...
        quat3(i,1)*quat3(i,1)+quat4(i,1)*quat4(i,1);
    quat(i,1)=sqrt(quat_sq(i,1));
    
    quat1(i,1)=quat1(i,1)/quat(i,1);
    quat2(i,1)=quat2(i,1)/quat(i,1);
    quat3(i,1)=quat3(i,1)/quat(i,1);
    quat4(i,1)=quat4(i,1)/quat(i,1);
end

% Compute Transformation Matrix form Body to Fixed (DCM):
for i = 1:i_num
    t11_bi(i,1)=quat1(i,1)*quat1(i,1)+quat2(i,1)*quat2(i,1)...
        -quat3(i,1)*quat3(i,1)-quat4(i,1)*quat4(i,1);
    t12_bi(i,1)=2*(quat2(i,1)*quat3(i,1)-quat1(i,1)*quat4(i,1));
    t13_bi(i,1)=2*(quat2(i,1)*quat4(i,1)+quat1(i,1)*quat3(i,1));
    t21_bi(i,1)=2*(quat2(i,1)*quat3(i,1)+quat1(i,1)*quat4(i,1));
    t22_bi(i,1)=quat1(i,1)*quat1(i,1)-quat2(i,1)*quat2(i,1)...
        +quat3(i,1)*quat3(i,1)-quat4(i,1)*quat4(i,1);
    t23_bi(i,1)=2*(quat3(i,1)*quat4(i,1)-quat1(i,1)*quat2(i,1));
    t31_bi(i,1)=2*(quat2(i,1)*quat4(i,1)-quat1(i,1)*quat3(i,1));
    t32_bi(i,1)=2*(quat3(i,1)*quat4(i,1)+quat1(i,1)*quat2(i,1));
    t33_bi(i,1)=quat1(i,1)*quat1(i,1)-quat2(i,1)*quat2(i,1)...
        -quat3(i,1)*quat3(i,1)+quat4(i,1)*quat4(i,1);
end

%% 10.80. Compute Other Fixed Axis States ****************************************
% Compute Fixed-Axis velocity & Acceleration:
for i=1:i_num;
    u_i(i,1)=t11_bi(i,1)*u_b(i,1)+t12_bi(i,1)*v_b(i,1)+t13_bi(i,1)*w_b(i,1);
    v_i(i,1)=t21_bi(i,1)*u_b(i,1)+t22_bi(i,1)*v_b(i,1)+t23_bi(i,1)*w_b(i,1);
    w_i(i,1)=t31_bi(i,1)*u_b(i,1)+t32_bi(i,1)*v_b(i,1)+t33_bi(i,1)*w_b(i,1);
    
    ax_i(i,1)=t11_bi(i,1)*ax_b(i,1)+t12_bi(i,1)*ay_b(i,1)+t13_bi(i,1)*az_b(i,1);
    ay_i(i,1)=t21_bi(i,1)*ax_b(i,1)+t22_bi(i,1)*ay_b(i,1)+t23_bi(i,1)*az_b(i,1);
    az_i(i,1)=t31_bi(i,1)*ax_b(i,1)+t32_bi(i,1)*ay_b(i,1)+t33_bi(i,1)*az_b(i,1);
    
    % Fixed_Axis Total Range,Velocity, Acceleration & Body Rates:
    R_i_sq(i,1)=(x_i(i,1)*x_i(i,1)+y_i(i,1)*y_i(i,1)+z_i(i,1)*z_i(i,1));
    R_i(i,1)=sqrt(R_i_sq(i,1));
    
    V_i_sq(i,1)=(u_i(i,1)*u_i(i,1)+v_i(i,1)*v_i(i,1)+w_i(i,1)*w_i(i,1));
    V_i(i,1)=sqrt(V_i_sq(i,1));
    
    A_i_sq(i,1)=(ax_i(i,1)*ax_i(i,1)+ay_i(i,1)*ay_i(i,1)+az_i(i,1)*az_i(i,1));
    A_i=sqrt(A_i_sq(i,1));
    
    p_i(i,1)=(v_i(i,1)*az_i(i,1)-w_i(i,1)*ay_i(i,1))/V_i_sq(i,1);
    q_i(i,1)=(w_i(i,1)*ax_i(i,1)-u_i(i,1)*az_i(i,1))/V_i_sq(i,1);
    r_i(i,1)=(u_i(i,1)*ay_i(i,1)-v_i(i,1)*ax_i(i,1))/V_i_sq(i,1);
end

%% Compute Relative Positons, LOS Angles & Closing:
for i = 1:i_num;
    for j =1:j_num;
        if(i~=j);
            
            rel_u_i(i,j,1) = u_i(i,1)-u_i(j,1);
            rel_v_i(i,j,1) = v_i(i,1)-v_i(j,1);
            rel_w_i(i,j,1) = w_i(i,1)-w_i(j,1);
            
            rel_ax_i(i,j,1) = ax_i(i,1)-ax_i(j,1);
            rel_ay_i(i,j,1) = ay_i(i,1)-ay_i(j,1);
            rel_az_i(i,j,1) = az_i(i,1)-az_i(j,1);
            
            rel_R1_i_dot(i,j,1)=(rel_x_i(i,j,1)*rel_u_i(i,j,1)...
                +rel_y_i(i,j,1)*rel_v_i(i,j,1))/rel_R1_i(i,j,1);
            rel_R_i_dot(i,j,1)=(rel_x_i(i,j,1)*rel_u_i(i,j,1)...
                +rel_y_i(i,j,1)*rel_v_i(i,j,1)...
                +rel_z_i(i,j,1)*rel_w_i(i,j,1))/rel_R_i(i,j,1);
            
            rel_V_i_sq(i,j,1)=(rel_u_i(i,j,1)*rel_u_i(i,j,1)+...
                rel_v_i(i,j,1)*rel_v_i(i,j,1)+...
                rel_w_i(i,j,1)*rel_w_i(i,j,1));
            rel_V_i(i,j,1)=sqrt(rel_V_i_sq(i,j,1));
            
            rel_A_i_sq(i,j,1)=(rel_ax_i(i,j,1)*rel_ax_i(i,j,1)+...
                rel_ay_i(i,j,1)*rel_ay_i(i,j,1)+...
                rel_az_i(i,j,1)*rel_az_i(i,j,1));
            rel_A_i(i,j,1)=sqrt(rel_A_i_sq(i,j,1));
            
            rel_psi_los_i_dot(i,j,1)=(rel_x_i(i,j,1)*rel_v_i(i,j,1)...
                -rel_y_i(i,j,1)*rel_u_i(i,j,1))/rel_R1_i_sq(i,j,1);
            rel_theta_los_i_dot(i,j,1)=(rel_w_i(i,j,1)*rel_R1_i(i,j,1)...
                -rel_z_i(i,j,1)*rel_R1_i_dot(i,j,1))/rel_R_i_sq(i,j,1);
            
        end
    end
end


%% 10.90. Autopilot Parameters ********************************************
% Autopilot Bandwidth & Input Limit Values:
bw_ax=zeros(i_num);
bw_ay=zeros(i_num);
bw_az=zeros(i_num);

limit_ax=zeros(i_num);
limit_ay=zeros(i_num);
limit_az=zeros(i_num);

bw_ax(1)=.1; bw_ax(2)=.1; bw_ax(3)=.1;
bw_ay(1)=3; bw_ay(2)=3; bw_ay(3)=3;
bw_az(1)=3; bw_az(2)=3; bw_az(3)=3;

%Set values for g-limits
lim_max_x=zeros(i_num);
lim_min_x=zeros(i_num);
lim_max_y=zeros(i_num);
lim_min_y=zeros(i_num);
lim_max_z=zeros(i_num);
lim_min_z=zeros(i_num);

%INPUT VALUES==============================================================
lim_max_x(1)=0; lim_min_x(1)=0;
lim_max_x(2)=0; lim_min_x(2)=0;
lim_max_x(3)=0; lim_min_x(3)=0;

lim_max_y(1)=80; lim_min_y(1)=-80;
lim_max_y(2)=400; lim_min_y(2)=-400;
lim_max_y(3)=400; lim_min_y(3)=-400;

lim_max_z(1)=80; lim_min_z(1)=-80;
lim_max_z(2)=400; lim_min_z(2)=-400;
lim_max_z(3)=400; lim_min_z(3)=-400;

%==========================================================================

%% 10.100 PN. APN Guidance-Law Parameters *********************************

nav_const_los=zeros(i_num);
nav_const_ax=zeros(i_num);
nav_const_ay=zeros(i_num);
nav_const_az=zeros(i_num);

miss_dist=zeros(i_num,j_num);
miss_flag=zeros(i_num,j_num);
flight_time=zeros(i_num,j_num);

for i=1:i_num;
    for j=1:j_num;
        if(i~=j);
            miss_dist(i,j)=rel_R_i(i,j,1);
        end
    end
end

%% 10.110. Optimum Guidance Parameters*************************************

T_1=zeros(1,T_index);
T_2=zeros(1,T_index);

rand_x=zeros(i_num,T_index);
rand_y=zeros(i_num,T_index);
rand_z=zeros(i_num,T_index);

ax_b_bias=zeros(i_num,T_index);
ay_b_bias=zeros(i_num,T_index);
az_b_bias=zeros(i_num,T_index);

sigma_1=0;
sigma_2=0;
sigma_3=0;
mean_1=0;
mean_2=0;
mean_3=0;


%% 10.120. TRIAL PARAMETER VALUES *****************************************
factor1=1;
factor2=1;
factor3=1;
factor4=1;

T_1_factor=1;
T_2_factor=1;

del_T_1=0;
del_T_2=0;

% INPUT VALUES=============================================================
% TEST 2 values
% s1 =1; s2=1; s3=1; s4=0; s5=0; s6=0; 
% r1_bar=0.1001; r3=0.1;r3_bar=0.1001; r2=0.1; 

%**************************************************************************
% PN TEST Values
% s1 =10; s2=10; s3=10; s4=0; s5=0; s6=0;%  PN Test
% r1_bar=1000; r2=.0001; r3_bar=1000; r3=.0001; % PN Test

%**************************************************************************
%TEST 1 Values
% r1_bar=1.001; r3=1.0;r3_bar=1.001; r2=1.0; %exp. r values
% s1 =1; s2=1; s3=1; s4=0; s5=0; s6=0; %experiment 3.1

%**************************************************************************
%TEST BASELINE 
 s1 =1; s2=1; s3=1; s4=0; s5=0; s6=0; %BASELINE
 r1_bar=0.00011; r2=.0001; r3_bar=0.00011; r3=.0001; %BASELINE

%==========================================================================

r_diff_1=(r1_bar*r3)/(r1_bar-r3);
r_diff_2=(r3_bar*r2)/(r3_bar-r2);

% Calculate Guidance Gains:
T_1(1,1)=T_1_factor*abs(rel_R_i(3,1,1)/rel_R_i_dot(3,1,1))+del_T_1;
T_2(1,1)=T_2_factor*abs(rel_R_i(2,3,1)/rel_R_i_dot(2,3,1))+del_T_2;

T_1_sq=T_1(1,1)*T_1(1,1);
T_1_cube=T_1_sq*T_1(1,1);
T_1_fourth=T_1_cube*T_1(1,1);

T_2_sq=T_2(1,1)*T_2(1,1);
T_2_cube=T_2_sq*T_2(1,1);
T_2_fourth=T_2_cube*T_2(1,1);

% Gains for vehicle 3 against 1:
den_1=(12.0*r_diff_1*r_diff_1+12.0*s4*r_diff_1*T_1(1,1)+...
    4.0*s1*r_diff_1*T_1_cube+s1*s4*T_1_fourth);
den_2=(12.0*r_diff_1*r_diff_1+12.0*s5*r_diff_1*T_1(1,1)+...
    4.0*s2*r_diff_1*T_1_cube+s2*s5*T_1_fourth);
den_3=(12.0*r_diff_1*r_diff_1+12.0*s6*r_diff_1*T_1(1,1)+...
    4.0*s3*r_diff_1*T_1_cube+s3*s6*T_1_fourth);

num_14=6.0*s1*r_diff_1*T_1(1,1)*(2.0*r_diff_1+s4*T_1(1,1));
num_25=6.0*s2*r_diff_1*T_1(1,1)*(2.0*r_diff_1+s5*T_1(1,1));
num_36=6.0*s3*r_diff_1*T_1(1,1)*(2.0*r_diff_1+s6*T_1(1,1));
num_44=4.0*r_diff_1*(3.0*s4*r_diff_1+3.0*s1*r_diff_1*T_1_sq+s1*s4*T_1_cube);
num_55=4.0*r_diff_1*(3.0*s5*r_diff_1+3.0*s2*r_diff_1*T_1_sq+s2*s5*T_1_cube);
num_66=4.0*r_diff_1*(3.0*s6*r_diff_1+3.0*s3*r_diff_1*T_1_sq+s3*s6*T_1_cube);

% Interceptor (3) intercept gains against Target (1)
g31_1=(num_14/den_1)/r3;
g31_2=(num_25/den_2)/r3;
g31_3=(num_36/den_3)/r3;
g31_4=(num_44/den_1)/r3;
g31_5=(num_55/den_2)/r3;
g31_6=(num_66/den_3)/r3;

% target (1) evasion gains against attacker (3)
g13_1=(num_14/den_1)/r1_bar;
g13_2=(num_25/den_2)/r1_bar;
g13_3=(num_36/den_3)/r1_bar;
g13_4=(num_44/den_1)/r1_bar;
g13_5=(num_55/den_2)/r1_bar;
g13_6=(num_66/den_3)/r1_bar;

% Gains for vehicles 2 against 3:
den_1=(12.0*r_diff_2*r_diff_2+12.0*s4*r_diff_2*T_2(1,1)+...
    4.0*s1*r_diff_2*T_2_cube+s1*s4*T_2_fourth);
den_2=(12.0*r_diff_2*r_diff_2+12.0*s5*r_diff_2*T_2(1,1)+...
    4.0*s2*r_diff_2*T_2_cube+s2*s5*T_2_fourth);
den_3=(12.0*r_diff_2*r_diff_2+12.0*s6*r_diff_2*T_2(1,1)+...
    4.0*s3*r_diff_2*T_2_cube+s3*s6*T_2_fourth);

num_14=6.0*s1*r_diff_2*T_2(1,1)*(2.0*r_diff_2+s4*T_2(1,1));
num_25=6.0*s2*r_diff_2*T_2(1,1)*(2.0*r_diff_2+s5*T_2(1,1));
num_36=6.0*s3*r_diff_2*T_2(1,1)*(2.0*r_diff_2+s6*T_2(1,1));
num_44=4.0*r_diff_2*(3.0*s4*r_diff_2+3.0*s1*r_diff_2*T_2_sq+s1*s4*T_2_cube);
num_55=4.0*r_diff_2*(3.0*s5*r_diff_2+3.0*s2*r_diff_2*T_2_sq+s2*s5*T_2_cube);
num_66=4.0*r_diff_2*(3.0*s6*r_diff_2+3.0*s3*r_diff_2*T_2_sq+s3*s6*T_2_cube);

% defender (2) intercept gains against attacker (3)
g23_1=(num_14/den_1)/r2;
g23_2=(num_25/den_2)/r2;
g23_3=(num_36/den_3)/r2;
g23_4=(num_44/den_1)/r2;
g23_5=(num_55/den_2)/r2;
g23_6=(num_66/den_3)/r2;

% attacker (3) evasion gains against defender (2)
g32_1=(num_14/den_1)/r3_bar;
g32_2=(num_25/den_2)/r3_bar;
g32_3=(num_36/den_3)/r3_bar;
g32_4=(num_44/den_1)/r3_bar;
g32_5=(num_55/den_2)/r3_bar;
g32_6=(num_66/den_3)/r3_bar;

%Guidance demands in Fixed Axis:

ax_i_dem(1,1)=factor1*(g13_1*rel_x_i(1,3,1)+g13_4*rel_u_i(1,3,1));
ax_i_dem(2,1)=factor2*(g23_1*rel_x_i(3,2,1)+g23_4*rel_u_i(3,2,1));
ax_i_dem(3,1)=factor3*(g31_1*rel_x_i(1,3,1)+g31_4*rel_u_i(1,3,1)...
    +factor4*(g32_1*rel_x_i(3,2,1)+g32_4*rel_u_i(3,2,1)));

ay_i_dem(1,1)=factor1*(g13_2*rel_y_i(1,3,1)+g13_5*rel_v_i(1,3,1));
ay_i_dem(2,1)=factor2*(g23_2*rel_y_i(3,2,1)+g23_5*rel_v_i(3,2,1));
ay_i_dem(3,1)=factor3*(g31_2*rel_y_i(1,3,1)+g31_5*rel_v_i(1,3,1)...
    +factor4*(g32_2*rel_y_i(3,2,1)+g32_5*rel_v_i(3,2,1)));

az_i_dem(1,1)=factor1*(g13_3*rel_z_i(1,3,1)+g13_6*rel_w_i(1,3,1));
az_i_dem(2,1)=factor2*(g23_3*rel_z_i(3,2,1)+g23_6*rel_w_i(3,2,1));
az_i_dem(3,1)=factor3*(g31_3*rel_z_i(1,3,1)+g31_6*rel_w_i(1,3,1)...
    +factor4*(g32_3*rel_z_i(3,2,1)+g32_6*rel_w_i(3,2,1)));

%Convert to Demands in Body Axis
for i=1:i_num;
    ax_b_dem(i,1)=t11_bi(i,1)*ax_i_dem(i,1)+t21_bi(i,1)*ay_i_dem(i,1)+...
        t31_bi(i,1)*az_i_dem(i,1);
    ay_b_dem(i,1)=t12_bi(i,1)*ax_i_dem(i,1)+t22_bi(i,1)*ay_i_dem(i,1)+...
        t32_bi(i,1)*az_i_dem(i,1);
    az_b_dem(i,1)=t13_bi(i,1)*ax_i_dem(i,1)+t23_bi(i,1)*ay_i_dem(i,1)+...
        t33_bi(i,1)*az_i_dem(i,1);
end
%% g-constraints
ax_b_dem(1,1)=0;
ax_b_dem(2,1)=0;
ax_b_dem(3,1)=0;

if ay_b_dem(1,1)<lim_min_y(1); ay_b_dem(1,1)=lim_min_y(1); end
if ay_b_dem(1,1)>lim_max_y(1); ay_b_dem(1,1)=lim_max_y(1); end
if ay_b_dem(2,1)<lim_min_y(2); ay_b_dem(2,1)=lim_min_y(2); end
if ay_b_dem(2,1)>lim_max_y(2); ay_b_dem(2,1)=lim_max_y(2); end
if ay_b_dem(3,1)<lim_min_y(3); ay_b_dem(3,1)=lim_min_y(3); end
if ay_b_dem(3,1)>lim_max_y(3); ay_b_dem(3,1)=lim_max_y(3); end
if az_b_dem(1,1)<lim_min_z(1); az_b_dem(1,1)=lim_min_z(1); end
if az_b_dem(1,1)>lim_max_z(1); az_b_dem(1,1)=lim_max_z(1); end
if az_b_dem(2,1)<lim_min_z(2); az_b_dem(2,1)=lim_min_z(2); end
if az_b_dem(2,1)>lim_max_z(2); az_b_dem(2,1)=lim_max_z(2); end
if az_b_dem(3,1)<lim_min_z(3); az_b_dem(3,1)=lim_min_z(3); end
if az_b_dem(3,1)>lim_max_z(3); az_b_dem(3,1)=lim_max_z(3); end

% Vehicles Additional Manoeuvres
ax_b_bias(1,1)=0;
ay_b_bias(1,1)=0;
az_b_bias(1,1)=0;

ax_b_bias(2,1)=0;
ay_b_bias(2,1)=0;
az_b_bias(2,1)=0;

ax_b_bias(3,1)=0;
ay_b_bias(3,1)=0;
az_b_bias(3,1)=0;

%% ************************ END INITIALISATION BLOCK **********************

%% ************************************************************************
%  20. START MAIN SIMULATION LOOP::
%  ************************************************************************
%% 20.10. Update Inertial_Axis Position, Velocity & Acceleration:

for T=1:T_index-1;
    for i=1:i_num;
        [x_i(i,T+1),y_i(i,T+1),z_i(i,T+1),u_i(i,T+1),v_i(i,T+1),...
            w_i(i,T+1),ax_i(i,T+1),ay_i(i,T+1),az_i(i,T+1),...
            quat1(i,T+1),quat2(i,T+1),quat3(i,T+1),quat4(i,T+1),...
            t11_bi(i,T+1),t12_bi(i,T+1),t13_bi(i,T+1),t21_bi(i,T+1),...
            t22_bi(i,T+1),t23_bi(i,T+1),t31_bi(i,T+1),t32_bi(i,T+1),...
            t33_bi(i,T+1)]=...
            kinematics3(x_i(i,T),y_i(i,T),z_i(i,T),u_i(i,T),v_i(i,T),...
            w_i(i,T),quat1(i,T),quat2(i,T),quat3(i,T),quat4(i,T),...
            p_b(i,T),q_b(i,T),r_b(i,T),ax_b(i,T),ay_b(i,T),az_b(i,T),del_t);
        
        R_i_sq(i,T+1)=(x_i(i,T+1)*x_i(i,T+1)+y_i(i,T+1)*y_i(i,T+1)+...
            z_i(i,T+1)*z_i(i,T+1));
        R_i(i,T+1)=sqrt(R_i_sq(i,T+1));
        
        V_i_sq(i,T+1)=(u_i(i,T+1)*u_i(i,T+1)+v_i(i,T+1)*v_i(i,T+1)+...
            w_i(i,T+1)*w_i(i,T+1));
        V_i(i,T+1)=sqrt(V_i_sq(i,T+1));
        
        A_i_sq(i,T+1)=(ax_i(i,T+1)*ax_i(i,T+1)+ay_i(i,T+1)*ay_i(i,T+1)+...
            az_i(i,T+1)*az_i(i,T+1));
        A_i(i,T+1)=sqrt(A_i_sq(i,T+1));
        
        p_i(i,T+1)=(v_i(i,T+1)*az_i(i,T+1)-w_i(i,T+1)*ay_i(i,T+1))/V_i_sq(i,T+1);
        q_i(i,T+1)=(w_i(i,T+1)*ax_i(i,T+1)-u_i(i,T+1)*az_i(i,T+1))/V_i_sq(i,T+1);
        r_i(i,T+1)=(u_i(i,T+1)*ay_i(i,T+1)-v_i(i,T+1)*ax_i(i,T+1))/V_i_sq(i,T+1);
    end
    
    %%  20.20. Update Inertial_Axis Relative States************************
    for i = 1:i_num;
        for j =1:j_num;
            if(i~=j);
                rel_x_i(i,j,T+1) = x_i(i,T+1)-x_i(j,T+1);
                rel_y_i(i,j,T+1) = y_i(i,T+1)-y_i(j,T+1);
                rel_z_i(i,j,T+1) = z_i(i,T+1)-z_i(j,T+1);
                
                rel_u_i(i,j,T+1) = u_i(i,T+1)-u_i(j,T+1);
                rel_v_i(i,j,T+1) = v_i(i,T+1)-v_i(j,T+1);
                rel_w_i(i,j,T+1) = w_i(i,T+1)-w_i(j,T+1);
                
                rel_ax_i(i,j,T+1) = ax_i(i,T+1)-ax_i(j,T+1);
                rel_ay_i(i,j,T+1) = ay_i(i,T+1)-ay_i(j,T+1);
                rel_az_i(i,j,T+1) = az_i(i,T+1)-az_i(j,T+1);
                
                rel_R1_i_sq(i,j,T+1)=(rel_x_i(i,j,T+1)*rel_x_i(i,j,T+1)+...
                    rel_y_i(i,j,T+1)*rel_y_i(i,j,T+1));
                rel_R1_i(i,j,T+1)=sqrt(rel_R1_i_sq(i,j,T+1));
                
                rel_R_i_sq(i,j,T+1)=(rel_R1_i_sq(i,j,T+1)+...
                    rel_z_i(i,j,T+1)*rel_z_i(i,j,T+1));
                rel_R_i(i,j,T+1)=sqrt(rel_R_i_sq(i,j,T+1));
                
                rel_V_i_sq(i,j,T+1)=(rel_u_i(i,j,T+1)*rel_u_i(i,j,T+1)+...
                    rel_v_i(i,j,T+1)*rel_v_i(i,j,T+1)+...
                    rel_w_i(i,j,1)*rel_w_i(i,j,1));
                rel_V_i(i,j,T+1)=sqrt(rel_V_i_sq(i,j,T+1));
                
                rel_A_i_sq(i,T+1)=(rel_ax_i(i,j,T+1)*rel_ax_i(i,j,T+1)+...
                    rel_ay_i(i,j,T+1)*rel_ay_i(i,j,T+1)+...
                    rel_az_i(i,j,T+1)*rel_az_i(i,j,T+1));
                rel_A_i(i,j,T+1)=sqrt(rel_A_i_sq(i,j,T+1));
                
                % Update Range, LOS Angle and Rates:
                rel_R1_i_dot(i,j,T+1)=(rel_x_i(i,j,T+1)*rel_u_i(i,j,T+1)...
                    +rel_y_i(i,j,T+1)*rel_v_i(i,j,T+1))/rel_R1_i(i,j,T+1);
                rel_R_i_dot(i,j,T+1)=(rel_x_i(i,j,T+1)*rel_u_i(i,j,T+1)...
                    +rel_y_i(i,j,T+1)*rel_v_i(i,j,T+1)...
                    +rel_z_i(i,j,T+1)*rel_w_i(i,j,T+1))/rel_R_i(i,j,T+1);
                
                rel_psi_los_i_dot(i,j,T+1)=(rel_x_i(i,j,T+1)*...
                    rel_v_i(i,j,T+1)-rel_y_i(i,j,T+1)*rel_u_i(i,j,T+1))...
                    /rel_R1_i_sq(i,j,T+1);
                rel_theta_los_i_dot(i,j,T+1)=(rel_w_i(i,j,T+1)*...
                    rel_R1_i(i,j,T+1)-rel_z_i(i,j,T+1)*...
                    rel_R1_i_dot(i,j,T+1))/rel_R_i_sq(i,j,T+1);
                
                rel_psi_los_i(i,j,T+1) = atan2(rel_y_i(i,j,T+1),...
                    rel_x_i(i,j,T+1));
                rel_theta_los_i(i,j,T+1) = atan2(-rel_z_i(i,j,T+1),...
                    rel_R1_i(i,j,T+1));
            end
        end
    end
    
    %% 20.30. Autopilot Loop Dynamics**************************************
    % Update Body-Axis Velocities & Accelerations
    for i=1:i_num
        ax_b_dot(i,T+1)=-bw_ax(i)*ax_b(i,T)+bw_ax(i)*ax_b_dem(i,T);
        ay_b_dot(i,T+1)=-bw_ay(i)*ay_b(i,T)+bw_ay(i)*ay_b_dem(i,T);
        az_b_dot(i,T+1)=-bw_az(i)*az_b(i,T)+bw_az(i)*az_b_dem(i,T);
        
        ax_b(i,T+1)=ax_b(i,T)+ax_b_dot(i,T+1)*del_t;
        ay_b(i,T+1)=ay_b(i,T)+ay_b_dot(i,T+1)*del_t;
        az_b(i,T+1)=az_b(i,T)+az_b_dot(i,T+1)*del_t;
        
        u_b(i,T+1)=u_b(i,T);
        v_b(i,T+1)=v_b(i,T);
        w_b(i,T+1)=w_b(i,T);
        
    end
    
    % Update Body_Axis Velociy, Acceleration and Rates:
    for i=1:i_num;
        V_b(i,T+1)=sqrt(u_b(i,T+1)*u_b(i,T+1)+v_b(i,T+1)*v_b(i,T+1)+...
            w_b(i,T+1)*w_b(i,T+1));
        V_b_sq(i,T+1)=V_b(i,T+1)*V_b(i,T+1);
        A_b(i,T+1)=sqrt(ax_b(i,T+1)*ax_b(i,T+1)+ay_b(i,T+1)*ay_b(i,T+1)+...
            az_b(i,T+1)*az_b(i,T+1));
        p_b(i,T+1)=(v_b(i,T+1)*az_b(i,T+1)-w_b(i,T+1)*ay_b(i,T+1))/V_b_sq(i,T+1);
        q_b(i,T+1)=(w_b(i,T+1)*ax_b(i,T+1)-u_b(i,T+1)*az_b(i,T+1))/V_b_sq(i,T+1);
        r_b(i,T+1)=(u_b(i,T+1)*ay_b(i,T+1)-v_b(i,T+1)*ax_b(i,T+1))/V_b_sq(i,T+1);
    end
    
    %% 20.40. Guidance Law Implementation**********************************
    T_1(1,T+1)=T_1_factor*abs(rel_R_i(3,1,T+1)/rel_R_i_dot(3,1,T+1))+del_T_1;
    T_2(1,T+1)=T_2_factor*abs(rel_R_i(2,3,T+1)/rel_R_i_dot(2,3,T+1))+del_T_2;
    
    if(T_1(1,T+1)>T_1(1,T));T_1(1,T+1)=T_1(1,T);
    end
    if(T_2(1,T+1)>T_2(1,T));T_2(1,T+1)=T_2(1,T);
    end
    
    T_1_sq=T_1(1,T+1)*T_1(1,T+1);
    T_1_cube=T_1_sq*T_1(1,T+1);
    T_1_fourth=T_1_cube*T_1(1,T+1);
    
    T_2_sq=T_2(1,T+1)*T_2(1,T+1);
    T_2_cube=T_2_sq*T_2(1,T+1);
    T_2_fourth=T_2_cube*T_2(1,T+1);
    
    time=T*del_t;
    if(time>6);
        factor4=0;
    end
    
    % Guidance Gains - Target(1)/Attacker(3):
    den_1=(12.0*r_diff_1*r_diff_1+12.0*s4*r_diff_1*T_1(1,T+1)+...
        4.0*s1*r_diff_1*T_1_cube+s1*s4*T_1_fourth);
    den_2=(12.0*r_diff_1*r_diff_1+12.0*s5*r_diff_1*T_1(1,T+1)+...
        4.0*s2*r_diff_1*T_1_cube+s2*s5*T_1_fourth);
    den_3=(12.0*r_diff_1*r_diff_1+12.0*s6*r_diff_1*T_1(1,T+1)+...
        4.0*s3*r_diff_1*T_1_cube+s3*s6*T_1_fourth);
    
    num_14=6.0*s1*r_diff_1*T_1(1,T+1)*(2.0*r_diff_1+s4*T_1(1,T+1));
    num_25=6.0*s2*r_diff_1*T_1(1,T+1)*(2.0*r_diff_1+s5*T_1(1,T+1));
    num_36=6.0*s3*r_diff_1*T_1(1,T+1)*(2.0*r_diff_1+s6*T_1(1,T+1));
    num_44=4.0*r_diff_1*(3.0*s4*r_diff_1+3.0*s1*r_diff_1*T_1_sq+s1*s4*T_1_cube);
    num_55=4.0*r_diff_1*(3.0*s5*r_diff_1+3.0*s2*r_diff_1*T_1_sq+s2*s5*T_1_cube);
    num_66=4.0*r_diff_1*(3.0*s6*r_diff_1+3.0*s3*r_diff_1*T_1_sq+s3*s6*T_1_cube);
    
    g31_1=num_14/den_1/r3;
    g31_2=num_25/den_2/r3;
    g31_3=num_36/den_3/r3;
    g31_4=num_44/den_1/r3;
    g31_5=num_55/den_2/r3;
    g31_6=num_66/den_3/r3;
    
    g13_1=num_14/den_1/r1_bar;
    g13_2=num_25/den_2/r1_bar;
    g13_3=num_36/den_3/r1_bar;
    g13_4=num_44/den_1/r1_bar;
    g13_5=num_55/den_2/r1_bar;
    g13_6=num_66/den_3/r1_bar;
    
    
    % Guidance Gains - Target(3)/Defender(2):
    den_1=(12.0*r_diff_2*r_diff_2+12.0*s4*r_diff_2*T_2(1,T+1)+...
        4.0*s1*r_diff_2*T_2_cube+s1*s4*T_2_fourth);
    den_2=(12.0*r_diff_2*r_diff_2+12.0*s5*r_diff_2*T_2(1,T+1)+...
        4.0*s2*r_diff_2*T_2_cube+s2*s5*T_2_fourth);
    den_3=(12.0*r_diff_2*r_diff_2+12.0*s6*r_diff_2*T_2(1,T+1)+...
        4.0*s3*r_diff_2*T_2_cube+s3*s6*T_2_fourth);
    
    num_14=6.0*s1*r_diff_2*T_2(1,T+1)*(2.0*r_diff_2+s4*T_2(1,T+1));
    num_25=6.0*s2*r_diff_2*T_2(1,T+1)*(2.0*r_diff_2+s5*T_2(1,T+1));
    num_36=6.0*s3*r_diff_2*T_2(1,T+1)*(2.0*r_diff_2+s6*T_2(1,T+1));
    num_44=4.0*r_diff_2*(3.0*s4*r_diff_2+3.0*s1*r_diff_2*T_2_sq+s1*s4*T_2_cube);
    num_55=4.0*r_diff_2*(3.0*s5*r_diff_2+3.0*s2*r_diff_2*T_2_sq+s2*s5*T_2_cube);
    num_66=4.0*r_diff_2*(3.0*s6*r_diff_2+3.0*s3*r_diff_2*T_2_sq+s3*s6*T_2_cube);
    
    % Attacker Gains
    g23_1=num_14/den_1/r2;
    g23_2=num_25/den_2/r2;
    g23_3=num_36/den_3/r2;
    g23_4=num_44/den_1/r2;
    g23_5=num_55/den_2/r2;
    g23_6=num_66/den_3/r2;
    
    % Evader gains
    g32_1=num_14/den_1/r3_bar;
    g32_2=num_25/den_2/r3_bar;
    g32_3=num_36/den_3/r3_bar;
    g32_4=num_44/den_1/r3_bar;
    g32_5=num_55/den_2/r3_bar;
    g32_6=num_66/den_3/r3_bar;
    
    % Guidance Acceleration Demands in Fixed_Axis:
    
    ax_i_dem(1,T+1)=factor1*(g13_1*rel_x_i(1,3,T+1)+g13_4*rel_u_i(1,3,T+1));
    ax_i_dem(2,T+1)=factor2*(g23_1*rel_x_i(3,2,T+1)+g23_4*rel_u_i(3,2,T+1));
    ax_i_dem(3,T+1)=factor3*(g31_1*rel_x_i(1,3,T+1)+g31_4*rel_u_i(1,3,T+1))...
        +factor4*(g32_1*rel_x_i(3,2,T+1)+g32_4*rel_u_i(3,2,T+1));
    
    ay_i_dem(1,T+1)=factor1*(g13_2*rel_y_i(1,3,T+1)+g13_5*rel_v_i(1,3,T+1));
    ay_i_dem(2,T+1)=factor2*(g23_2*rel_y_i(3,2,T+1)+g23_5*rel_v_i(3,2,T+1));
    ay_i_dem(3,T+1)=factor3*(g31_2*rel_y_i(1,3,T+1)+g31_5*rel_v_i(1,3,T+1))...
        +factor4*(g32_2*rel_y_i(3,2,T+1)+g32_5*rel_v_i(3,2,T+1));
    
    az_i_dem(1,T+1)=factor1*(g13_3*rel_z_i(1,3,T+1)+g13_6*rel_w_i(1,3,T+1));
    az_i_dem(2,T+1)=factor2*(g23_3*rel_z_i(3,2,T+1)+g23_6*rel_w_i(3,2,T+1));
    az_i_dem(3,T+1)=factor3*(g31_3*rel_z_i(1,3,T+1)+g31_6*rel_w_i(1,3,T+1))...
        +factor4*(g32_3*rel_z_i(3,2,T+1)+g32_6*rel_w_i(3,2,T+1));
    
    %Convert Demands to Body_Axis
    for i=1:i_num;
        ax_b_dem(i,T+1)=t11_bi(i,T+1)*ax_i_dem(i,T+1)+t21_bi(i,T+1)*...
            ay_i_dem(i,T+1)+t31_bi(i,T+1)*az_i_dem(i,T+1);
        ay_b_dem(i,T+1)=t12_bi(i,T+1)*ax_i_dem(i,T+1)+t22_bi(i,T+1)*...
            ay_i_dem(i,T+1)+t32_bi(i,T+1)*az_i_dem(i,T+1);
        az_b_dem(i,T+1)=t13_bi(i,T+1)*ax_i_dem(i,T+1)+t23_bi(i,T+1)*...
            ay_i_dem(i,T+1)+t33_bi(i,T+1)*az_i_dem(i,T+1);
    end
    
    %Additional Vehicle Manoeuvres:
    for i = 1:i_num;
        ax_b_dem(i,T+1)=ax_b_dem(i,T+1)+ax_b_bias(i,T+1);
        ay_b_dem(i,T+1)=ay_b_dem(i,T+1)+ay_b_bias(i,T+1);
        az_b_dem(i,T+1)=az_b_dem(i,T+1)+az_b_bias(i,T+1);
    end
    
    %% g-constraints
    ax_b_dem(1,T+1)=0;
    ax_b_dem(2,T+1)=0;
    ax_b_dem(3,T+1)=0;
    
    if ay_b_dem(1,T+1)<lim_min_y(1); ay_b_dem(1,T+1)=lim_min_y(1); end
    if ay_b_dem(1,T+1)>lim_max_y(1); ay_b_dem(1,T+1)=lim_max_y(1); end
    if ay_b_dem(2,T+1)<lim_min_y(2); ay_b_dem(2,T+1)=lim_min_y(2); end
    if ay_b_dem(2,T+1)>lim_max_y(2); ay_b_dem(2,T+1)=lim_max_y(2); end
    if ay_b_dem(3,T+1)<lim_min_y(3); ay_b_dem(3,T+1)=lim_min_y(3); end
    if ay_b_dem(3,T+1)>lim_max_y(3); ay_b_dem(3,T+1)=lim_max_y(3); end
    if az_b_dem(1,T+1)<lim_min_z(1); az_b_dem(1,T+1)=lim_min_z(1); end
    if az_b_dem(1,T+1)>lim_max_z(1); az_b_dem(1,T+1)=lim_max_z(1); end
    if az_b_dem(2,T+1)<lim_min_z(2); az_b_dem(2,T+1)=lim_min_z(2); end
    if az_b_dem(2,T+1)>lim_max_z(2); az_b_dem(2,T+1)=lim_max_z(2); end
    if az_b_dem(3,T+1)<lim_min_z(3); az_b_dem(3,T+1)=lim_min_z(3); end
    if az_b_dem(3,T+1)>lim_max_z(3); az_b_dem(3,T+1)=lim_max_z(3); end
    
    % Check for Miss Distance *********************************************
    if(rel_R_i(2,3,T+1)<miss_dist(2,3));
        miss_dist(2,3)=rel_R_i(2,3,T+1);
        miss_flag(2,3)=0;
    else
        if(miss_flag(2,3)==0);
            miss_flag(2,3)=1;
            Miss23=miss_dist(2,3)
            flight_time(2,3)=(T+1)*del_t;
            Flight_time23=flight_time(2,3)
        end
    end
    
    if(rel_R_i(3,1,T+1)<miss_dist(3,1));
        miss_dist(3,1)=rel_R_i(3,1,T+1);
        miss_flag (3,1)=0;
    else
        if(miss_flag(3,1)==0);
            miss_flag(3,1)=1;
            Miss31=miss_dist(3,1)
            flight_time(3,1)=(T+1)*del_t;
            Flight_time31=flight_time(3,1)
        end
    end
    %     if(miss_flag(2,3)==1 && miss_flag(3,1)==1);
    %         break
    %     end
    
    if T==1     % Only for the first simulation step
        %% Miss distances
        decreasing_3_1 = true;
        decreasing_2_3 = true;
        
        misses23 = [];
        misses31 = [];
        
        %% Incremental plotting during run
        res = 500;      %Plot every "res" simulation steps
        rescount = 1;
        
        % Calculate locations for 3 figures in top half of screen
        ss = get(0,'ScreenSize');
        windw = ss(3)/3;
        windh = (ss(4)-28)/2;
        
        % Format figures and plot first point
        f25 = figure(25); hold on
        set(f25, 'OuterPosition', [1 29+windh  windw windh], 'MenuBar', ' none', 'Toolbar', 'figure');
        f25p1 = plot(x_i(1,1),-z_i(1,1),'k');
        f25p2 = plot(x_i(2,1),-z_i(2,1),':k');
        f25p3 = plot(x_i(3,1),-z_i(3,1),'--k');
        a25 = gca;
        title('Z vs. X; 1=blk, 2=..., 3=- - -');
        xlabel('Down-Range (m)');
        ylabel('Altitude (m)');
        
        f26 = figure(26); hold on
        set(f26, 'OuterPosition', [1+windw 29+windh  windw windh], 'MenuBar', ' none', 'Toolbar', 'figure');
        f26p1 = plot(y_i(1,1),-z_i(1,1),'k');
        f26p2 = plot(y_i(2,1),-z_i(2,1),':k');
        f26p3 = plot(y_i(3,1),-z_i(3,1),'--k');
        a26 = gca;
        title('Z vs. Y; 1=blk, 2=..., 3=- - -');
        xlabel('Cross-Range (m)');
        ylabel('Altitude (m)');
        
        f27 = figure(27); hold on
        set(f27, 'OuterPosition', [1+2*windw 29+windh  windw windh], 'MenuBar', ' none', 'Toolbar', 'figure');
        f27p1 = plot(x_i(1,1),y_i(1,1),'k');
        f27p2 = plot(x_i(2,1),y_i(2,1),':k');
        f27p3 = plot(x_i(3,1),y_i(3,1),'--k');
        a27 = gca;
        title('Y vs. X; 1=blk, 2=..., 3=- - -');
        xlabel('Down-Range (m)');
        ylabel('Cross Range (m)');
        
        f36 = figure(36); hold on
        set(f36, 'OuterPosition', [1+2*windw 29 windw windh], 'MenuBar', ' none', 'Toolbar', 'figure');
        set(gca, 'xlim', [0,tf]);
        rel_R_i_3_1 = zeros(length(rel_R_i),1);
        rel_R_i_3_1(1) = rel_R_i(3,1,1);
        rel_R_i_3_1(2) = rel_R_i(3,1,2);
        f36p1 = plot(t(1),rel_R_i_3_1(1),'--k');
        rel_R_i_2_3 = zeros(length(rel_R_i),1);
        rel_R_i_2_3(1) = rel_R_i(2,3,1);
        rel_R_i_2_3(2) = rel_R_i(2,3,2);
        f36p2 = plot(t(1),rel_R_i_2_3(1),':k');
        a36 = gca;
        y_lim = get(gca, 'ylim');
        set(gca, 'ylim', [0 y_lim(2)]);
        y_lim = get(gca, 'ylim');
        title('Range-to-go vs. Time');
        xlabel('Time (s)');
        ylabel('Range-to-go (m)');
        
        %% Pause and quit buttons
        choice=0;
        hd = dialog('WindowStyle', 'normal', 'Name', '', 'OuterPosition', [1 29+windh-100  270 90]);
        but1=uicontrol(hd,'Style','pushbutton','String','Pause','Callback','choice=1;');
        but2=uicontrol(hd,'Style','pushbutton','String','Continue','Position', [100 20 60 20],'Callback','choice=2;');
        but3=uicontrol(hd,'Style','pushbutton','String','Quit','Position', [180 20 60 20],'Callback','choice=3;');
    else        % For every simulation step except the first
        %% Miss distances
        if (rel_R_i(2,3,T+1) <= rel_R_i_2_3(T))     % Decreasing range
            if(~decreasing_2_3)
                decreasing_2_3 = true;              % Change to decreasing
            end
        else                                        % Increasing range
            if(decreasing_2_3)
                decreasing_2_3 = false;              % Change to increasing
                misses23 = [misses23; t(T) rel_R_i_2_3(T)];
                plot(a25, x_i(3,T+1),-z_i(3,T+1),'*b');
                plot(a25, x_i(2,T+1),-z_i(2,T+1),'sb');
                plot(a26, y_i(3,T+1),-z_i(3,T+1),'*b');
                plot(a26, y_i(2,T+1),-z_i(2,T+1),'sb');
                plot(a27, x_i(3,T+1),y_i(3,T+1),'*b');
                plot(a27, x_i(2,T+1),y_i(2,T+1),'sb');
                plot(a36, [t(T), t(T)], y_lim, '-b');
            end
        end
        if (rel_R_i(3,1,T+1) <= rel_R_i_3_1(T))     % Decreasing range
            if(~decreasing_3_1)
                decreasing_3_1 = true;              % Change to decreasing
            end
        else                                        % Increasing range
            if(decreasing_3_1)
                decreasing_3_1 = false;              % Change to increasing
                misses31 = [misses31; t(T) rel_R_i_3_1(T)];
                plot(a25, x_i(1,T+1),-z_i(1,T+1),'or');
                plot(a25, x_i(3,T+1),-z_i(3,T+1),'*r');
                plot(a26, y_i(1,T+1),-z_i(1,T+1),'or');
                plot(a26, y_i(3,T+1),-z_i(3,T+1),'*r');
                plot(a27, x_i(1,T+1),y_i(1,T+1),'or');
                plot(a27, x_i(3,T+1),y_i(3,T+1),'*r');
                plot(a36, [t(T), t(T)], y_lim, '-r');
            end
        end
        rel_R_i_3_1(T+1) = rel_R_i(3,1,T+1);
        rel_R_i_2_3(T+1) = rel_R_i(2,3,T+1);
        
        %% Incremental plotting during run
        rescount = rescount+1;
        if rescount>=res
            rescount = 0;
            set(f25p1,'xdata',x_i(1,1:T),'ydata',-z_i(1,1:T));
            set(f25p2,'xdata',x_i(2,1:T),'ydata',-z_i(2,1:T));
            set(f25p3,'xdata',x_i(3,1:T),'ydata',-z_i(3,1:T));
            set(f26p1,'xdata',y_i(1,1:T),'ydata',-z_i(1,1:T));
            set(f26p2,'xdata',y_i(2,1:T),'ydata',-z_i(2,1:T));
            set(f26p3,'xdata',y_i(3,1:T),'ydata',-z_i(3,1:T));
            set(f27p1,'xdata',x_i(1,1:T),'ydata',y_i(1,1:T));
            set(f27p2,'xdata',x_i(2,1:T),'ydata',y_i(2,1:T));
            set(f27p3,'xdata',x_i(3,1:T),'ydata',y_i(3,1:T));
            set(f36p1,'xdata',t(1:T),'ydata',rel_R_i_3_1(1:T));
            set(f36p2,'xdata',t(1:T),'ydata',rel_R_i_2_3(1:T));
            drawnow;
        end
    end
    
    %% Pause and quit buttons
    while choice==1
        set(but1,'String','Step');
        waitforbuttonpress;
        choice=2;
        if choice==2
            set(but1,'String','Pause');
        end
    end
    if choice==3
        delete(hd);
        clear('hd');
        break
    end
end
%% Pause and quit buttons. Delete buttons if they still exist
if exist('hd', 'var')
    delete(hd);
    clear('hd');
end

%% Incremental plotting
% Plot last point on graphs
set(f25p1,'xdata',x_i(1,1:T),'ydata',-z_i(1,1:T));
set(f25p2,'xdata',x_i(2,1:T),'ydata',-z_i(2,1:T));
set(f25p3,'xdata',x_i(3,1:T),'ydata',-z_i(3,1:T));
set(f26p3,'xdata',y_i(3,1:T),'ydata',-z_i(3,1:T));
set(f27p1,'xdata',x_i(1,1:T),'ydata',y_i(1,1:T));
set(f27p2,'xdata',x_i(2,1:T),'ydata',y_i(2,1:T));
set(f27p3,'xdata',x_i(3,1:T),'ydata',y_i(3,1:T));
set(f36p1,'xdata',t(1:T),'ydata',rel_R_i_3_1(1:T));
set(f36p2,'xdata',t(1:T),'ydata',rel_R_i_2_3(1:T));

% Show 2 minimum miss distances on graph 36
figure(36);
% Stretch series to fill X axis
set(gca, 'xlim', [0,ceil(t(T))]);
% Plot vertical near miss lines
% yl = get(gca, 'ylim');

empty = true;
str={};
str{1} = '               Time        Distance';
if ~isempty(misses31)
    empty = false;
    misses31 = sortrows(misses31,2);
    for i=1:size(misses31,1)
%         plot([misses31(i,1),misses31(i,1)], yl, '-r');
        str = [str; 'Miss31 :  ', num2str(misses31(i,:))];
        if (i==2)
            break
        end
    end
end
if ~isempty(misses23)
    empty = false;
    misses23 = sortrows(misses23,2);
    for i=1:size(misses23,1)
%         plot([misses23(i,1),misses23(i,1)], yl, '-b');
        str = [str; 'Miss23 :  ', num2str(misses23(1,:))];
        if (i==2)
            break
        end
    end
end
if ~empty
    text(.55,.95,str,'EdgeColor','black','VerticalAlignment','top','units','normalized');
    drawnow;
end






