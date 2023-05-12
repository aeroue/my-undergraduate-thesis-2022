%% ************************************************************************
%% This subroutine updates the direction cosine matrix using quaternions
%% Transform the vehicle acceleration from body to fixed axis
%% Updates the fixed axis vehicle position and velocities
%% ************************************************************************

function[x_i,y_i,z_i,u_i,v_i,w_i,ax_i,ay_i,az_i,quat1,quat2,quat3,quat4,...
    t11_bi,t12_bi,t13_bi,t21_bi,t22_bi,t23_bi,t31_bi,t32_bi,t33_bi]=...
    kinematics3(x_i,y_i,z_i,u_i,v_i,w_i,quat1,quat2,quat3,quat4,p,q,r,...
    ax_b,ay_b,az_b,del_t)

%% 1. Position,Velocity & Acceleration Vectors Update:
%% 1.1. Update Quaternions:
quat1_dot= -0.5*(quat2*p+quat3*q+quat4*r);
quat2_dot= 0.5*(quat1*p-quat4*q+quat3*r);
quat3_dot= 0.5*(quat4*p+quat1*q-quat2*r);
quat4_dot= -0.5*(quat3*p-quat2*q-quat1*r);

quat1=quat1+quat1_dot*del_t;
quat2=quat2+quat2_dot*del_t;
quat3=quat3+quat3_dot*del_t;
quat4=quat4+quat4_dot*del_t;

quat_sq=quat1*quat1+quat2*quat2+quat3*quat3+quat4*quat4;
quat=sqrt(quat_sq);

quat1=quat1/quat;
quat2=quat2/quat;
quat3=quat3/quat;
quat4=quat4/quat;

%% 1.2. Construct DCM;
t11_bi=quat1*quat1+quat2*quat2-quat3*quat3-quat4*quat4;
t12_bi=2*(quat2*quat3-quat1*quat4);
t13_bi=2*(quat2*quat4+quat1*quat3);
t21_bi=2*(quat2*quat3+quat1*quat4);
t22_bi=quat1*quat1-quat2*quat2+quat3*quat3-quat4*quat4;
t23_bi=2*(quat3*quat4-quat1*quat2);
t31_bi=2*(quat2*quat4-quat1*quat3);
t32_bi=2*(quat3*quat4+quat1*quat2);
t33_bi=quat1*quat1-quat2*quat2-quat3*quat3+quat4*quat4;

%% 1.3. Construct:ax_i,ay_i,az_i, from ax_b,ay_b,az_b:
ax_i=t11_bi*ax_b+t12_bi*ay_b+t13_bi*az_b;
ay_i=t21_bi*ax_b+t22_bi*ay_b+t23_bi*az_b;
az_i=t31_bi*ax_b+t32_bi*ay_b+t33_bi*az_b;

%% 1.4. Update Position & Velocity;

u_i=u_i+ax_i*del_t;
v_i=v_i+ay_i*del_t;
w_i=w_i+az_i*del_t;

x_i =x_i+u_i*del_t;
y_i =y_i+v_i*del_t;
z_i =z_i+w_i*del_t;


    

