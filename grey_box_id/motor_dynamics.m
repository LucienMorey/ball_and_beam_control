function [A,B,C,D] = motor_dynamics(k_theta, k_theta_dot, k_v, Ts)

%disp(k_theta, k_theta_dot, k_v);
A = [0, 1;
     k_theta, k_theta_dot];

B = [0;
     k_v];

C = [1, 0];

D = 0;
end

