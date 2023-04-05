close all
clear
clc
%% Sim Parameters
fs = 200;
ts = 1/fs;
%% Parameters
length = 0.91; %m
height = 0.32; %M
m = 0.03299; %kg, 0.033
r = 0.01; %m
g = 9.81; %m/s^2
J_b = 2/5*m*r^2; %kg*m^2

a = 47.66;
b = 13.6;
%% Continuous-time Model 
% = [p p_dot theta theta_dot]'
Ac = [0, 1, 0, 0;
      0, 0, -(m*g)/((J_b/(r^2))+m), 0;
      0, 0, 0, 1;
      0, 0, 0, -a];
Bc = [0 0 0 b]';
Cc = [1 0 0 0;
      0 0 1 0];
Dc = 0;
%Continuos-time model in a compact form
sys_c = ss(Ac,Bc,Cc,Dc);

%%

data = readtable('scope_31.csv');

data = data(4:end,:);

real_ts = data.Var1(2) - data.Var1(1);
real_input = data.Var2;

inputVoltage_offset = mean(real_input(1:50));
real_input = real_input - inputVoltage_offset;

real_ball = data.Var3;
real_ball = 16.838518 * real_ball + 2.763498; %Ball Position = 16.838518 * Signal + 2.763498
real_ball = real_ball/100;
real_ball_offset = mean(real_ball(1:50));
real_ball = real_ball - real_ball_offset;

real_theta= data.Var4;
real_theta =  0.018152* real_theta + 0.012149; % Beam Angle = 0.018152 * Signal + 0.012149 
real_theta_offset = mean(real_theta(1:50));
real_theta = real_theta - real_theta_offset;

%%
input_duration = 10;
voltage = 0.8;
on_duration = 0.1;
off_duration = 0.1;
input = zeros(1,input_duration/ts);


for i = 1:size(input,2)
    time = i*ts;
    if(time > 4.4)
        time = time - 4.4;
    end
    if(time > 0.98) && ( time < 1.08)
        input(i) = 0.85;
    end
    if(time > 2.08) && (time < 2.18)
        input(i) = -0.94;
    end
    if(time > 3.18) && (time < 3.28)
        input(i) = -0.91;
    end
    if(time > 4.28) && (time < 4.38)
        input(i) = 0.85;
    end
end

time = (0:size(input,2)-1)*ts;
ball_sim = lsim(sys_c,input,time,[0 0 0 0]');

%% Plots
real_time = (0:size(real_input,1)-1)*ts;

figure(1)
subplot(3,1,1)
plot(time,input,'r')
hold on
plot(real_time,real_input,'b')
title("Input")
legend('simInput','realInput')
axis([0 6 -1 1])
xlabel("time (s)")
ylabel("Voltage (v)")

subplot(3,1,2)
plot(time,ball_sim(:,2),'r')
hold on
plot(real_time,real_theta,'b')
title("Theta")
legend('simTheta','realTheta')
axis([0 6 -0.05 0.05])
xlabel("time (s)")
ylabel("Theta (rads)")

subplot(3,1,3)
plot(time,ball_sim(:,1),'r')
hold on
plot(real_time,real_ball,'b')
title("BallPos")
legend('simBallPos','realBallPos')
axis([0 6 -0.4 0.4])
xlabel("time (s)")
ylabel("Position (m)")

%% Stability, Controlability and Observerability

stability = eig(Ac)

controllability = rank(ctrb(Ac,Bc))

observability = rank(obsv(Ac,Cc))