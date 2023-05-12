clear 
clc

data = readtable("scope_44.csv");
data = data(3:end-1,:);

% time
time = data.Var1;

% conversion
ballPos = (16.838518 * data.Var3 + 2.763498)/100;

% averaging/filtering
ballPos = movmean(ballPos,20);

ts = data.Var1(2) - data.Var1(2);

inital_pos = mean(ballPos(1:5));
finish_pos = -0.2;

stepinfo(ballPos,time,finish_pos,inital_pos)

figure(1)

plot(time,ballPos)
hold on
plot(time, ones(size(time))*finish_pos)
title("Experimental Data Closed Loop Validation")
ylabel("Ball Position (Voltage)")
xlabel("Time (s)")
legend("Ball Position","Final Reference")