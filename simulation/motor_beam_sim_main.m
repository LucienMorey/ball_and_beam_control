%% Ball and Beam System
%% 48580 Control Studio B% University of Technology Sydney, Australia
%% George Sheslow & Lucien Morey
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear
%% Simulation Config
Tsim=10;
%Total simulation time
fs = 1000;
Ts = 1/fs;
% if obs = 1 then the kalman filter will be used for control calculations
OBS = 1;
%% Parameters
length = 0.91; %m
height = 0.32; %M
m = 0.03299; %kg
r = 0.01; %m
g = 9.81; %m/s^2
J_b = 2/5*m*r^2; %kg*m^2
a = -13.3794; % TODO FIND THIS
b = 747.4732; % TODO FIND THIS

position_noise = 0.1;
angle_noise = 0.1;
voltage_noise =1e-6;
  
p_0 = 1.0; %m
p_dot_0 = 0.0; %m/s
theta_0 = 0.0; %rad
theta_dot_0 = 0.0; %rad/s

p_hat_0 = 0.8; %m
p_dot_hat_0 = 0.0; %m/s
theta_hat_0 = 0* pi/180; %rad
theta_dot_hat_0 = 0.0; %rad/s
%% Continuous-time Model 
% = [p p_dot theta theta_dot]'
Ac = [0, 1, 0, 0;
      0, 0, -(m*g)/((J_b/(r^2))+m), 0;
      0, 0, 0 1;
      0, 0, 0 a];
Bc = [0 0 0 b]';
Cc = [1 0 0 0;
      0 0 1 0];
Dc = 0;
%Continuos-time model in a compact form
c_sys=ss(Ac,Bc,Cc,Dc);
%% Controllability
disp('Controllability Matrix')
CM= ctrb(Ac,Bc);
disp(' ')
if (rank(CM)==size(CM,1))
    disp('System is controllable')
else 
    disp('System is NOT controllable')
end
disp(' ')
%% Observability
disp('Observability Matrix')
OM=obsv(Ac,Cc);
if (rank(OM)==size(OM,2))
    disp('System is observable')
else 
    disp('System is NOT observable')
end
disp(' ')
%% Equilibrium Point
% Ac is non-sigular
%disp('--------------------------------')
%disp('Equilibrium Point')
%Mo = -Cc*pinv(Ac)*Bc;
%Nu = inv(Mo)
%Nx = -pinv(Ac)*Bc*Nu
%disp(' ');
%% Discrete-Time Model
%    x(k+1)=A·x(k)+B·u(k)
%    y(k)=C·x(k)+D·u(k)
d_sys = c2d(c_sys,Ts);
disp('--------------------------------')
disp('Discrete-Time Matrices:')
disp('x(k+1)=Ax(k)=Bu(k)')
disp('y(k)=Cx(k)')

A=d_sys.A
B=d_sys.B
C=d_sys.C
D=d_sys.D;
disp(' ')
%% State Feedback
%modify these to be less than the unit circle if we want to use the
%dsicrete time system 
poles = [-0.7547 + 1.2059i, -0.7547 - 1.2059i, -10, -11]';
poles_discrete = exp(poles*Ts);
K = place(A,B,poles_discrete);
K = place(Ac,Bc, poles);

%% Observer
Q = 1e-10* eye(4);
R = diag([position_noise^2, angle_noise^2]);
P_0 = 0*eye(4,4);
