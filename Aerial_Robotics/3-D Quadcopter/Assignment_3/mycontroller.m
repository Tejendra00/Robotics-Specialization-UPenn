function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   control

% =================== Your code goes here ===================

% Thrust
F = 0;

% Moment
M = zeros(3,1);

m=params.mass;
g=params.gravity;

% U1 PD controller to control thrust
r3=state.pos(3);
r3_dot=state.vel(3);
r3T=des_state.pos(3);
r3T_dot=des_state.vel(3);
r3T_dotdot=des_state.acc(3);
kp3=100;
kd3=20;
u1=m*g+m*(r3T_dotdot+kp3*(r3T-r3)+kd3*(r3T_dot-r3_dot));
F=u1;

%--------U2 Controller-----


% r1_dotdot_des PD
r1=state.pos(1);
r1_dot=state.vel(1);
r1T=des_state.pos(1);
r1T_dot=des_state.vel(1);
r1T_dotdot=des_state.acc(1);
kp1=200;
kd1=40;
r1_dotdot_des=r1T_dotdot+kp1*(r1T-r1)+kd1*(r1T_dot-r1_dot);

%r2_dotdot_des PD
r2=state.pos(2);
r2_dot=state.vel(2);
r2T=des_state.pos(2);
r2T_dot=des_state.vel(2);
r2T_dotdot=des_state.acc(2);
kp2=200;
kd2=40;
r2_dotdot_des=r2T_dotdot+kp2*(r2T-r2)+kd2*(r2T_dot-r2_dot);

% phi_des and theta_des
yaw_des=des_state.yaw;

phi_des=(r1_dotdot_des*sin(yaw_des)-r2_dotdot_des*cos(yaw_des))/g;
theta_des=(r1_dotdot_des*cos(yaw_des)+r2_dotdot_des*sin(yaw_des))/g;

%U2 
phi=state.rot(1);
p=state.omega(1);
p_des=0;
kp_phi=100;
kd_phi=2;
Mb1=kp_phi*(phi_des-phi)+kd_phi*(p_des-p);

theta=state.rot(2);
q=state.omega(3);
q_des=0;
kp_theta=100;
kd_theta=2;
Mb2=kp_theta*(theta_des-theta)+kd_theta*(q_des-q);

psi=state.rot(1);
r=state.omega(1);
r_des=des_state.yawdot;
kp_psi=100;
kd_psi=2;
Mb3=kp_psi*(yaw_des-psi)+kd_psi*(r_des-r);

u2=[Mb1;Mb2;Mb3];
M=u2;

% =================== Your code ends here ===================

end
