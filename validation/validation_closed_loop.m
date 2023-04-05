clear
clc
%%
fs = 100;
Ts = 1/fs;
length = 0.91; %m
height = 0.32; %M
m = 0.03299; %kg
r = 0.01; %m
g = 9.81; %m/s^2
J_b = 2/5*m*r^2; %kg*m^2
k_theta = -0;
k_theta_dot = -47.66;
k_v = 13.6;

%%
Ac = [0, 1, 0, 0;
      0, 0, -(m*g)/((J_b/(r^2))+m), 0;
      0, 0, 0, 1;
      0, 0, k_theta, k_theta_dot];
Bc = [0 0 0 k_v]';
Cc = [1 0 0 0;
      0 0 1 0];
Dc = 0;

%%
overshoot = 0.14;
settling_time = 5.3;

zeta = sqrt((log(overshoot)^2)/((pi^2)+log(overshoot)^2));
% zeta= 1.2*zeta;
w_n = 4/(settling_time*zeta);
p1 = -zeta*w_n + w_n*sqrt(zeta^2-1);
p2 = -zeta*w_n - w_n*sqrt(zeta^2-1);


p_cont = [p1; p2; 10*real(p1); 10.1*real(p1)];
p_discrete = exp(p_cont * Ts);
po_cont = 5*p_cont;
po_discrete = exp(po_cont * Ts);


%% Model
K = place(Ac,Bc,p_cont);
L = place(Ac',Cc',po_cont)';


AA = [Ac -Bc*K; L*Cc Ac-Bc*K-L*Cc];
BB = zeros(8,1);
CC = [Cc zeros(2,4)];
DD = 0;

poles_whole_cl = eig(AA)

sys_cl_whole=ss(AA,BB,CC,DD);

%% Discrete Sim

sys_cl_whole=ss(AA,BB,CC,DD);
sys_d = c2d(sys_cl_whole,Ts);

% simulation of the whole closed loop system
t = 0:Ts:100;
u = 0*ones(size(t));
x0 = [0.2; 0; 0; 0; 0.2; 0; 0; 0];
[Y,T,X]=lsim(sys_d,u,t,x0);

figure(1)
plot(t,Y)
title("Closed Loop SFC and Observer Simulation")
axis([0 10 -0.3 0.3])
xlabel('time')
ylabel('output')
ylabel("Ball Position (m) and Beam Angle (rads)")
xlabel("Time (s)")
legend("BallPos","BeamAngle")
stepinfo(Y(:,1),t,0,0.2)