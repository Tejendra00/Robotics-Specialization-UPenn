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

r=state.pos;
r_dot=state.vel;
rT=des_state.pos;
rT_dot=des_state.vel;
rT_dotdot=des_state.acc;
kpr=[110;110;110];
kdr=[21;21;21];

r_dotdot_des=rT_dotdot+kpr.*(rT-r)+kdr.*(rT_dot-r_dot);
u1=m*g+m*r_dotdot_des(3);
% u1=m*g;
F=u1;


%--------U2 Controller-----

% phi_des and theta_des
yaw_des=des_state.yaw;

phi_des=(r_dotdot_des(1)*sin(yaw_des)-r_dotdot_des(2)*cos(yaw_des))/g;
theta_des=(r_dotdot_des(1)*cos(yaw_des)+r_dotdot_des(2)*sin(yaw_des))/g;

%U2 
% For tunning
% phi_des=0;
% theta_des=0;
% yaw_des=1;
kp=[50;50;50];
kd=[5;5;5];
angle=state.rot;
angle_des=[phi_des;theta_des;yaw_des];
omega_des=[0;0;0];
omega=state.omega;
Mb=kp.*(angle_des-angle)+kd.*(omega_des-omega);


u2=[Mb(1);Mb(2);Mb(3)];
M=u2;

% =================== Your code ends here ===================

end
