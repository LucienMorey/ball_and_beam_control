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

% voltage input dead zone for system [upper, lower]
dead_zone = [0.0, -0.0];

% voltage input limit [upper, lower]
input_limit = [10, -10];

CTLR = 2; % 0: Open Loop
          % 1: State Feedback
          % 2: State Feedback + integral action
          % 3: LQR
          % 4: LQR + integral action
          % 5: Sliding Mode Control

OBS = 1; % 0: No observer
         % 1: observer

sensor_noise = 1; %set 0 or 1, nothing else!
input_noise = 1;

%% Parameters
length = 0.91; %m
height = 0.32; %M
m = 0.03299; %kg
r = 0.01; %m
g = 9.81; %m/s^2
J_b = 2/5*m*r^2; %kg*m^2
k_theta = 0;
k_theta_dot = -84.45;
k_v = 20.83;

position_variance = 1.1212 / 100.0;
angle_variance = 0.0045;
voltage_noise =1e-6;

% initial states [p (m), p_dot (m/s), theta (rads), theta_dot (rads/s)]
xo = [0.4 0.0 0.0 0.0]';
xo_hat = [0.4 0.0 0.0 0.0]'; 

%% Continuous-time Model 
% x = [p p_dot theta theta_dot]'
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

%% State Feedback + Luenberger

%modify these to be less than the unit circle if we want to use the
%dsicrete time system
overshoot = 0.07;
settling_time = 4;

[p1, p2] = second_order_poles(overshoot, settling_time);

p_cont = [p1; p2; 10*real(p1); 10.1*real(p1)];
p_discrete = exp(p_cont * Ts);

po_cont = 5*p_cont;
po_discrete = exp(po_cont * Ts);

disp('SFC:')

K = place(A,B,p_discrete)

% disp('poles before placement');
% disp(po_discrete);
% disp('poles after placement');
% disp(eig(A-B*K));

disp('LO DT')

L = place(A',C',po_discrete)'

% disp('poles before placement');
% disp(p_discrete);
% disp('poles after placement');
% disp(eig(A-L*C));

%% LQR + Kalman Filter

%LQR
Q_lqr = diag([15,20,40,1]);
R_lqr = 0.1;
[Kf,S,P] =dlqr(A, B, Q_lqr, R_lqr);

% Kalman Filter
Q = 1e-10* eye(4);
R = diag([position_variance^2, angle_variance^2]);
P_0 = 0*eye(4,4);

%% Integral Action

A_augmented = [A zeros(4,1);
               [1 0 0 0] 1]; % integral action only for ball position error

B_augmented = [B;
               0];

%LQR
Q_lqr = diag([20, 8, 10, 1, 0.00001]);
R_lqr = 0.01;
[Kf_and_Ki,S,P] =dlqr(A_augmented, B_augmented, Q_lqr, R_lqr);

%% SFC with integral action

p_cont_integral = [p1; p2; 10*real(p1); 10.1*real(p1); -0.01];
p_discrete_integral = exp(p_cont_integral * Ts);

disp('SFC with integral action:')

K_sfc_integral_action = place(A_augmented,B_augmented,p_discrete_integral)


%% Sliding Mode Control

Cs = [-1 -1.5 5 1];
gamma = 0.4;

%% Run Simulation
% sim('motor_beam_model_sim.slx');