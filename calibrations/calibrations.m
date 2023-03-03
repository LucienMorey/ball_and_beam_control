%% Load Data
clear
clc
load('data/2_03_23.mat')
%% ADC -> Beam Angle

[p1, p2] = fitData(data.BeamAngleADC, data.BeamAngleRads);
fprintf("ADC -> Beam Angle (Rads): Beam Angle = %f * ADC + %f \r\n", p2, p1);

[p1, p2] = fitData(data.BeamAngleADC, data.BeamAngleDegrees);
fprintf("ADC -> Beam Angle (Degs): Beam Angle = %f * ADC + %f \r\n", p2, p1);

%plot
% plot(data.BeamAngleADC, data.BeamAngleRads);

%% BeamAngleSignal -> Beam Angle

[p1, p2] = fitData(data.BeamAngleSignal, data.BeamAngleDegrees);
fprintf("Beam Angle Signal -> Beam Angle (Rads): Beam Angle = %f * Signal + %f \r\n", p2, p1);

[p1, p2] = fitData(data.BeamAngleDegrees, data.BeamAngleDegrees);
fprintf("Beam Angle Signal -> Beam Angle (Degs): Beam Angle = %f * Signal + %f \r\n", p2, p1);

%% ADC -> Ball Position

[p1, p2] = fitData(data.BallPosADC, data.BallPosMeters);
fprintf("ADC -> Ball Position (Meters): Beam Position = %f * ADC + %f \r\n", p2, p1);

%% BallPositionSignal -> Ball Position

[p1, p2] = fitData(data.BallPosSignal, data.BallPosMeters);
fprintf("Ball Position Signal -> Ball Position (Meters): Ball Position = %f * Signal + %f \r\n", p2, p1);

%% DriveVoltage -> ADC

[p1, p2] = fitData(data.DriveVoltageADC, data.DriveVoltageVoltage);
fprintf("Drive Voltage -> ADC (12 Bit): ADC = (Drive Voltage - %f) / %f \r\n", p1, p2);
