function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;

% FILL IN YOUR CODE HERE
e_t=s_des(1,1)-s(1,1);
e_dot_t=s_des(2,1)-s(2,1);
kp=220;
kd=30;
% Feedforward Term is zero as final position is hover
z_dotdot=0;


if s_des(1,1)==0
    % For hovering at z=0 we simply need thrust equal to weight of quadcopter.
    u=params.mass*params.gravity;
    if u>params.u_max
        disp('Thrust/Weight ratio of motors not enough');
    end
% For z_des=1m we use PD Controller
else 
    u=min(params.mass*(z_dotdot+kd*e_dot_t+kp*e_t+params.gravity),params.u_max);
    if u<params.u_min 
       u=params.u_min;
    end
    
    
end
    
end

