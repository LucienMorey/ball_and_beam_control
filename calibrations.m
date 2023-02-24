%% Calibration using regression for ball position on beam

adc_resolution = 10;

beam_position = [-40 -35 -30 -25 -20 -15 -10  -5   0   5  10  15  20  25  30  35  40];
adc_reading   = [400 420 430 440 455 470 480 500 512 525 537 550 565 580 590 605 620];

% Linear Regression
mdl = fitlm(beam_position, adc_reading)

plot(beam_position, adc_reading)
hold on
plotAdded(mdl)

%% Calibration using regression for beam angle

beam_angle = [-20 -15 -10 -5 0 5 10 15 20];
adc_reading = [];

% convert degrees to rads
beam_angle = beam_angle.*(pi/180);

