%% Ball and Beam System
%% 48580 Control Studio B% University of Technology Sydney, Australia
%% George Sheslow & Lucien Morey
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear
%% Simulation Config
Tsim=10;
%Total simulation time
fs = 100;
Ts = 1/fs;
% if obs = 1 then the kalman filter will be used for control calculations
OBS = 1;
CL = 1;
%% Parameters
length = 0.91; %m
height = 0.32; %M
m = 0.03299; %kg
r = 0.01; %m
g = 9.81; %m/s^2
J_b = 2/5*m*r^2; %kg*m^2
k_theta = -0;
k_theta_dot = -109.9;
k_v = 27.8;

position_variance = 1.1212 / 100.0;
angle_variance = 0.0045;
voltage_noise =1e-6;
  
p_0 = 0.2; %m
p_dot_0 = 0.0; %m/s
theta_0 = 0.0; %rad
theta_dot_0 = 0.0; %rad/s

p_hat_0 = 0.3; %m
p_dot_hat_0 = 0.0; %m/s
theta_hat_0 = 0* pi/180; %rad
theta_dot_hat_0 = 0.0; %rad/s
%% Continuous-time Model 
% = [p p_dot theta theta_dot]'
Ac = [0, 1, 0, 0;
      0, 0, -(m*g)/((J_b/(r^2))+m), 0;
      0, 0, 0, 1;
      0, 0, k_theta, k_theta_dot];
Bc = [0 0 0 k_v]';
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
overshoot = 0.14;
settling_time = 5;

zeta = sqrt((log(overshoot)^2)/((pi^2)+log(overshoot)^2))
%zeta= 1.2*zeta;
w_n = 4/(settling_time*zeta)

p1 = -zeta*w_n + w_n*sqrt(1-zeta^2)*1i;
p2 = -zeta*w_n - w_n*sqrt(1-zeta^2)*1i;

p_cont = [p1; p2; 10*real(p1); 10.1*real(p1)]

po_cont = 5*p_cont;

po_discrete = exp(po_cont * Ts)


disp('LO DT')
L = place(A',C',po_discrete)'
eig(A-L*C)


disp('SFC DT')
p_discrete = exp(p_cont * Ts)
K = place(A,B,p_discrete);
eig(A-B*K)

%K = place(Ac,Bc, p_cont);

%% Observer
Q = 1e-10* eye(4);
R = diag([position_variance^2, angle_variance^2]);
P_0 = 0*eye(4,4);



[Kf,Pf]=dlqr(A',C',Q,R)
Kf = Kf'

eig(Ac-Kf*C)
