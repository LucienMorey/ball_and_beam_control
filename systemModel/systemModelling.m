% load data
clear
clc
data = load ('data/sysid_02_03_23.mat');

%% plot

time = data.sysid020323.time;
input = data.sysid020323.v1;
output = data.sysid020323.v2;

%% conversion to physical parameters

%% plot, sanity check

plot(time, input);
hold on
plot(time, output);

%% filtering

filtered_input = medfilt1(input,10,'truncate');

plot(time,input);
hold on 
plot(time,filtered_input);

%%

filtered_output = medfilt1(output,3,'truncate');

plot(time,output);
hold on 
plot(time,filtered_output);


%%

time = scope2.xaxis;
v1 = scope2.VarName2;
v2 = scope2.VarName3;

plot(time,v1);
hold on;
plot(time,v2);