%% Load data from O.scope
data = readtable('scope_29.csv');

data = data(4:end,1:3);

ts = data.Var1(2) - data.Var1(1);
Fs = 1/ts;
inputVoltage = data.Var2;
beamAngle = data.Var3;
plot(beamAngle)
%%
beamAngle_rads =  0.0171* beamAngle + 0.0120; % Beam Angle = 0.0171* Signal + 0.0120

first_bit = beamAngle_rads(1:100);

beam_avg = mean(first_bit);

beamAngle_avg = beamAngle_rads - beam_avg;

plot(beamAngle_avg)
hold on
beamAngle_avg_dot = diff(beamAngle_avg);
plot(beamAngle_avg_dot)
legend('beamAngle','beamAngularVelocity')
%% 1st order approx
test_data = iddata(beamAngle_avg_dot,inputVoltage(1:end-1));
tf = tfest(test_data,1)
compare(test_data,tf)

%% 2nd order approx
test_data = iddata(beamAngle_avg,inputVoltage(1:end));
tf = tfest(test_data,2,1)
compare(test_data,tf)

