%%% SSY345 - PROJECT
% Name 1:    Nicholas Granlund
% Name 2:    Lousie Olssom

close all
clear
clc

addpath(genpath('./Sensorfusion smartphone'));
addpath(genpath('./functions'));
addpath(genpath('./data'));

%% =====================================================
%%% Task 2

%%% load the calibration data to assertain sensor bias and noise
calibrationData = load('logStatic2.mat');
Hz = 10;

%%% Calculate and plot necessary information from calibration data
[structAcc, structGyr, structMag] = plotCalibrationData(calibrationData, Hz);



%% ======================================================
%%% Connecting to mobile device
close all
clear all
clc

% Connect to device
devName = 'iPhone - Nicholas iPhone';
m = connectiOS(devName);

% Filter the recieved measurements
[xhat, meas] = filterTemplateiOS(m);


% Plot the estimates OUR vs MATLAB/GOOGLE
plotFilterEstimate(xhat,meas)






