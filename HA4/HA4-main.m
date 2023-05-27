%%% ASSIGNMENT 4
% Nicholas Granlund

close all
clear
clc

addpath(genpath('./functions'));
%rng(5)
rng(11)

%% 1. Smoothing

% Generate the trajectory.
[X,T,Tvec] = generateTrueTrack();

% Set the intial prior
x0 = [0; 0; 0; 0; 0];
P0 = diag([10^2, 10^2, 10^2, ((5*pi)/180)^2, ((1*pi)/180)^2]);

% Sensor position
s1 = [300; -100];
s2 = [300; -300];
S = [s1 s2];


% Measurement noise
sigma_phi_1 = pi/180;
sigma_phi_2 = pi/180;
R = diag([(sigma_phi_1).^2, (sigma_phi_2).^2]).*T;

% Process noise covariance (TUNE HERE)
sigma_v = 1;
sigma_w = pi/180;
Q = diag([0 0 (sigma_v)^2 0 (sigma_w)^2]).*T;


% Generate measurements
h = @(x,T) dualBearingMeasurement(x,s1,s2);
Y = genNonLinearMeasurementSequence(X, h, R);
[xpos, ypos] = getPosFromMeasurement(Y, s1, s2);

% Dynamics model
proc_f = @(x,T) coordinatedTurnMotion(x, T);
sigmaPoints = @sigmaPoints;

% Dummy variable for sensor pos
S = zeros(2,length(Y));

% Filter the position with non-linear Kalman filter
type = 'CKF';
[xs, Ps, xf, Pf, xp, Pp] = nonLinRTSsmoother(Y, x0, P0, proc_f, T, Q, S, h, R, sigmaPoints, type);


% Plotting
plotFilterSmoother(X,xs, Ps, xf, Pf, xp, Pp, xpos, ypos, s1, s2, type, Tvec)


%b) Modify a measurement at k=300
Y(:,300) = Y(:,300) + rand();
[xpos, ypos] = getPosFromMeasurement(Y, s1, s2);

% Filter the position with non-linear Kalman filter
[xs, Ps, xf, Pf, xp, Pp] = nonLinRTSsmoother(Y, x0, P0, proc_f, T, Q, S, h, R, sigmaPoints, type);

% Plot everything
plotFilterSmoother(X,xs, Ps, xf, Pf, xp, Pp, xpos, ypos, s1, s2, type, Tvec);







%% 2. Particle filters for linear/Gaussian systems
close all
clear
clc
rng(5)

% Process noise
proc_Q = 1.5;
% Measurement noise
meas_R = 3;

% Initial prior and cov
x0 = 2;
P0 = 8;

% Timesteps
K = 30;

% Dynamics model & Measurement model
proc_f = @(x) x;
meas_h = @(x) x;

% Generate state and measurement sequences
X = genLinearStateSequence(x0,P0,proc_f,proc_Q,K);
Y = genLinearMeasurementSequence(X, meas_h, meas_R);

% Incorrect prior UNCOMMENT FOR 2b
x0 = -20;
P0 = 2;

% Filter with kalman filter
[xf, Pf] = linearKalmanFilter(Y, x0, P0, 1, proc_Q, 1, meas_R);

% Filter with particle filter
N = 1000;
[xfp, Pfp, Xp, Wp] = pfFilter(x0, P0, Y, proc_f, proc_Q, meas_h, meas_R, N, false);
[xfp_resam, Pfp_resam, Xp_resam, Wp_resam] = pfFilter(x0, P0, Y, proc_f, proc_Q, meas_h, meas_R, N, true);


% Plot everything
plotParticleFilter(X,Y,K,xf,Pf,xfp,Pfp,xfp_resam,Pfp_resam);

% Plot posterior approx
time_instances = [1, 15, 30];
plotPosteriorApproximation(xf, Pf, xfp, Pfp, xfp_resam, Pfp_resam, time_instances);



% Calculate MSE
err_KF = immse( X(2:end) , xf )
err_PF = immse( X(2:end) , xfp)
err_PFresam = immse( X(2:end) , xfp_resam)

% Plot errobar
plotParticleFilterErrorbar(X,Y,K,xf,Pf,xfp,Pfp,xfp_resam,Pfp_resam)



% Filter with particle filter
N = 100;
bResampl = true;
[xfp, Pfp, Xp, Wp] = pfFilter(x0, P0, Y, proc_f, proc_Q, meas_h, meas_R, N, bResampl);
plotParticleTrajectory(Xp,Wp,K,N,X,xfp,Pfp,bResampl)







%% 2. Bicycle tracking in a village
close all;
clear;
clc;
rng(5)



