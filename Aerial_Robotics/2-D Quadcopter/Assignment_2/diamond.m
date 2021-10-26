function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;

% FILL IN YOUR CODE HERE
z_ddot=des_state.acc(2,1);
kdz=10;
kpz=1000;
etz=des_state.pos(2,1)-state.pos(2,1);
e_dottz=des_state.vel(2,1)-state.vel(2,1);


u1=min(params.mass*(z_ddot+kpz*etz+kdz*e_dottz+params.gravity),params.maxF);
if u1<params.minF
    u1=params.minF;
end

y_ddot=des_state.acc(1,1);
y_dddot=0;
kdy=2;
kpy=50;
ety=des_state.pos(1,1)-state.pos(1,1);
e_dotty=des_state.vel(1,1)-state.vel(1,1);
phi_c=-(y_ddot+kpy*ety+kdy+e_dotty)/params.gravity;

phi_c_dot=-(y_dddot+kpy*(e_dotty)+kdy*(y_ddot-(-params.gravity*state.rot)));
% phi_c_dot=0;
phi_c_ddot=0;


kdphi=50;
kpphi=100;
etphi=phi_c-state.rot(1,1);
e_dottphi=phi_c_dot-state.omega(1,1);

u2=params.Ixx*(phi_c_ddot+kpphi*etphi+kdphi*e_dottphi);





end

