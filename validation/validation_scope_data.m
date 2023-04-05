clear 
clc

data = readtable("scope_4.csv");
data = data(3:end-1,:);

data.Var3 = movmean(data.Var3,20);

ts = data.Var1(2) - data.Var1(2);

inital_pos = mean(data.Var3(1:5))

finish_pos = mean(data.Var3(end-5:end))

stepinfo(data.Var3,data.Var1,finish_pos,inital_pos)

figure(1)

plot(data.Var1,data.Var3)