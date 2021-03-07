function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters
z_dotdot_des = 0;
e = s_des(1) - s(1);
e_dot = s_des(2) - s(2);
m = params.mass;
g = params.gravity;
K_p = 60;
K_v = 10;

u = m*(z_dotdot_des + K_p*e + K_v*e_dot + g);


% FILL IN YOUR CODE HERE


end

