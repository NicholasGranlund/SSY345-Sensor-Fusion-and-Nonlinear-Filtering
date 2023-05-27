%% ASSIGNMENT 2

close all
clear all
clc


% Add path to functions folder
addpath('./functions');



%% Scenario 1 - A First Kalman filter and its properties
% a) Generate state sequence and measurements sequence
close all
clc
rng(2)

% Sequence length
N = 35;

% initial prior
x_0 = 2;
P_0 = 8;

% Measurments noise covariance and process disturbance covariance
Q = 1.5;
R = 3;

% Dynamics matrix
A = 1;
H = 1;


% Generate state sequence
x_seq = genLinearStateSequence(x_0, P_0, A, Q, N);

% Generate state measurements
y_seq = genLinearMeasurementSequence(x_seq, H, R);

% Plot the results
figure()
hold on
grid on
plot(0:N, x_seq,'Linewidth', 2)
plot(1:N, y_seq,'linewidth', 2)
legend('state sequence x','state measurements y')
title('Linear Gaussian state space model')
xlabel('N');





% b) Filter the measurements through the Kalman filter
[x_kalmf, P] = kalmanFilter(y_seq, x_0, P_0, A, Q, H, R);

std = sqrt(reshape(P,1,35));


% Plot the results
figure()
hold on
grid on
plot(0:N, x_seq,'Linewidth', 2)
plot(1:N, y_seq,'linewidth', 2)
plot(1:N, x_kalmf,'linewidth', 2)
plot(1:N, x_kalmf+(3*std),':','linewidth', 2,'color','k')
plot(1:N, x_kalmf-(3*std),':','linewidth', 2,'color','k')
legend('state sequence x','state measurements y','Kalman filtered measurements' ...
    ,'kalmf + 3\sigma','kalmf - 3\sigma')
title('Linear Gaussian state space model')
xlabel('N');


% Plot error densities
% figure()
% subplot(4,1,1)
% counter = 0;
% 
% for i = [1, 2, 4, 30]
%     counter = counter + 1;
%     err = (x_seq(i+1) - x_kalmf(i))^2;
% 
% 
%     a = mvnrnd(0,err,10000);
%     subplot(4,1,counter)
%     histogram(a,75,'Normalization','pdf')
%     hold on
% end








% c) Filter the measurements through the Kalman filter (x0 = 12)
[x_kalmf_err, P] = kalmanFilter(y_seq, 12, P_0, A, Q, H, R);


% Plot the results
figure()
hold on
grid on
plot(1:N, x_kalmf,'linewidth', 2)
plot(1:N, x_kalmf_err,':','linewidth', 2)
legend('correct Kalman','Incorrect Kalman')
title('Kalman filters for right/wrong assumption on x_0')
xlabel('N');








% d) Plot yada ydada...
% We choose k = 23

k = 23;


figure()
ticks = -15:0.01:0

% Prior
one = normpdf(ticks,x_kalmf(k-1),std(k-1));
plot(ticks,one,'linewidth',2)
hold on
grid on

% Prediction
[x_pred, P_pred] = linearPrediction(x_kalmf(k-1), std(k-1)^2, A, Q);
two = normpdf(ticks,x_pred,sqrt(P_pred));
plot(ticks,two,'linewidth',2)

% Measurement
three = normpdf(ticks,y_seq(k),R);
plot(ticks,three,'linewidth',2)

% Update
[x_upd, P_upd] = linearUpdate(x_pred, P_pred, y_seq(k), H, R);
four = normpdf(ticks,x_upd,sqrt(P_upd));
plot(ticks,four,'linewidth',2)


% Plot details
legend('prior: P(x_{k-1} | y_{1:k-1})','prediction: P(x_{k} | y_{1:k-1})','measurements: y','update: P(x_{k} | y_{1:k})')
title('PDF for the different Kalman steps')






%% Scenario 2 - Tuning a Kalman filter
close all
clear all
clc

% Load the data
trainMeasurements = load('SensorMeasurements.mat');

% Extract
pureNoise = trainMeasurements.CalibrationSequenceVelocity_v0;
trainSpeed10 = trainMeasurements.CalibrationSequenceVelocity_v10;
trainSpeed20 = trainMeasurements.CalibrationSequenceVelocity_v20;

% Determine C
C1 = pureNoise / pureNoise;
C2 = trainSpeed10 / (10 + pureNoise);
C3 = trainSpeed20 / (20 + pureNoise);


C2 = (mean(trainSpeed10) - mean(pureNoise)) / 10;
C2 = (mean(trainSpeed20) - mean(pureNoise)) / 20;
C = mean([C2 C3])


% Calculate variance
var2 = var(trainSpeed10 / C);
var3 = var(trainSpeed20 / C);

% Try to replicate the noisy behaviour
fakeTrainSpeed10 = C2*(10 + mvnrnd(mean(pureNoise), var2, 2000)');
fakeTrainSpeed20 = C3*(20 + mvnrnd(mean(pureNoise), var3, 2000)');

figure()
subplot(2,1,1)
plot(trainSpeed10)
hold on
plot(fakeTrainSpeed10)
legend('Actual readings','generated readings')

subplot(2,1,2)
plot(trainSpeed20)
hold on
plot(fakeTrainSpeed20)
legend('Actual readings','generated readings')


% =========================================================================
% Call Kalman filter (CONSTANT VEL)
h = 2*0.1;
alpha = 0.0001;
Y = Generate_y_seq;
x_0 = [0;0];
P_0 = [0.5 0.5; 
       0.5 0.5];

A_cv = [1  h; 
        0  1];

% Process disturbances
Q = alpha*[0   0
     0   1];

H = [1  0;
     0  1];

% Measurement noise
R = [1   0
    0  mean([var2, var3])];

% Remove every other measurement. Let the Kalman filter work on the
% instances where there is data available from both sensors. (double h)
Y = Y(:,1:2:end);
Y(2,:) = Y(2,:)./C;
time = h:h:200;


[X_cv, P] = kalmanFilter(Y, x_0, P_0, A_cv, Q, H, R);

figure()
subplot(2,1,1)
plot(time, X_cv(1,:),'linewidth',3)
title('Motion model CV')
grid on
hold on
plot(time, Y(1,:),':k','linewidth',2)
legend('x position of train CV','Positional measurements y','location','southeast')
xlabel('time [s]'); ylabel('Position of train [m]')

subplot(2,1,2)
plot(time, X_cv(2,:),'linewidth',3)
grid on
hold on
plot(time, Y(2,:),':k','linewidth',2)
legend('x Velocity of train CV','Velocity measurements y')
xlabel('time [s]'); ylabel('Velocity of train [m/s]')







% =========================================================================
% Call Kalman filter (CONSTANT ACC)
h = 2*0.1;
Y = Generate_y_seq;
x_0 = [0;0;0];
P_0 = eye(3);

A_ca = [1   h   h*h/2; 
        0   1   h;
        0   0   1];

% Process disturbances
Q = alpha*[0   0   0
     0   0   0
     0   0   1];

H = [1  0  0;
     0  1  0];

% Measurement noise
R = [1   0
    0  mean([var2, var3])];


Y = Y(:,1:2:end);
Y(2,:) = Y(2,:)./C;
time = h:h:200;
[X_ca, P] = kalmanFilter(Y, x_0, P_0, A_ca, Q, H, R);

figure()
subplot(2,1,1)
plot(time, X_ca(1,:),'linewidth',3)
title('Motion model CA')
grid on
hold on
plot(time, Y(1,:),':k','linewidth',2)
legend('x position of train CA','Positional measurements y','location','southeast')
xlabel('time [s]'); ylabel('Position of train [m]')

subplot(2,1,2)
plot(time, X_ca(2,:),'linewidth',3)
grid on
hold on
plot(time, Y(2,:),':k','linewidth',2)
legend('x Velocity of train CA','Velocity measurements y')
xlabel('time [s]'); ylabel('Velocity of train [m/s]')

