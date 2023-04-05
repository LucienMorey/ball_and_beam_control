%% Determine Beam Magic Numbers via 1st order approx:
close all
clear
clc
%%
data = readtable('scope_29.csv');

data = data(4:300,1:3);

ts = data.Var1(2) - data.Var1(1);

inputVoltage = data.Var2;

inputVoltage_offset = mean(inputVoltage(1:50));
inputVoltage = inputVoltage - inputVoltage_offset;

theta = data.Var3;

theta_rads =  0.018152* theta + 0.012149; % Beam Angle = 0.018152 * Signal + 0.012149 

theta_offset = mean(theta_rads(1:50)); %% determine offset from average of steady state

theta_rads = theta_rads - theta_offset;

theta_dot = diff(theta_rads)/ts;

%% 1st order approx
test_data = iddata(theta_dot,inputVoltage(1:end-1),ts);
opt = tfestOptions('InitializeMethod','n4sid','InitialCondition','zero');
tf = tfest(test_data,1,0,opt)

%% Beam State Space Model x = [theta theta_dot]'

b = tf.Numerator(1);
a = tf.Denominator(2);
Ac = [0 1; 
      0 -a];
Bc = [0 b]';
Cc = [1 0];
Dc = 0;

sys_c = ss(Ac,Bc,Cc,Dc);

%% Validation

u = inputVoltage(1:end-1);
time = (0:length(u)-1)*ts;

theta_sim = lsim(sys_c,u,time,[0 0]');

figure(1)
subplot(2,1,1)
plot(time,theta_sim,'b',time,theta_rads(1:end-1),'r')
title("Theta Sim vs Theta Real")
legend("thetaSim","thetaReal")

subplot(2,1,2)
plot(time,u)
title("Input")

