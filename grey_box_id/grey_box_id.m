%% Initialise estimate system
k_theta_0 = -0;
k_theta_dot_0 = -113.1;
k_v_0 = 28.31;

function_params = {'k_theta',     k_theta_0;
                   'k_theta_dot', k_theta_dot_0;
                   'k_v',         k_v_0};

function_type = 'c';

sys = idgrey(@motor_dynamics,function_params,function_type, {},0);
sys.Structure.Parameters(1).Free = false;
sys.Structure.Parameters(2).Free = true;
sys.Structure.Parameters(3).Free = true;

%% Prepare test data
data = readtable('scope_29.csv');
data = data(4:300,1:3);

inputVoltage = data.Var2;
beamAngle = data.Var3;

ts = data.Var1(2) - data.Var1(1);
Fs = 1/ts;

% convert to radians
beamAngle_rads =  0.018152* beamAngle + 0.012149 ; % Beam Angle = 0.018152 * Signal + 0.012149 

%extract small set of sample to account for offset from 0
first_bit = beamAngle_rads(1:50);
beam_avg = mean(first_bit);
beamAngle_avg = beamAngle_rads - beam_avg;

beamAngle_avg_dot = diff(beamAngle_avg)/ts;

% create id data
test_data = iddata(beamAngle_avg,inputVoltage(1:end), ts);

%% Estimate system
opt1 = greyestOptions('InitialState','estimate','Display','on');
opt1.EnforceStability = true;

est_sys = greyest(test_data, sys, opt1)

opt = compareOptions('InitialCondition','zero');
compare(test_data,est_sys,Inf,opt)