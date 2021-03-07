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
m = params.mass;
g = params.gravity;
Ixx = params.Ixx;

k_vz = 10; k_pz = 50;
k_vphi = 20; k_pphi = 80;
k_vy = 10; k_py = 50;

y = state.pos(1); z = state.pos(2);
y_dot = state.vel(1); z_dot = state.vel(2);
phi = state.rot; phi_dot = state.omega;

y_des = des_state.pos(1); z_des = des_state.pos(2);
y_dot_des = des_state.vel(1); z_dot_des = des_state.vel(2);
y_ddot_des = des_state.acc(1); z_ddot_des = des_state.acc(2);


phi_c = -(y_ddot_des + k_vy*(y_dot_des - y_dot) + k_py*(y_des - y)) / g;
phi_c_dot = -(k_vy*(y_ddot_des + g*phi) + k_py*(y_dot_des - y_dot)) / g;
u1 = m*(g + z_ddot_des + k_vz*(z_dot_des - z_dot) + k_pz*(z_des - z));
u2 = Ixx*(k_vphi*(phi_c_dot - phi_dot) + k_pphi*(phi_c - phi));

end

