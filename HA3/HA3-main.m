%%% HA3
% BY: Nicholas Granlund

close all
clear all
clc

addpath('./functions');
rng(1)



%% =========================================================================
% QUESTION 1
% Approximations of mean and covariance


% Task a) 
close all
clc

% Number of samples to estimate mean and covariance of the transformed gaussian
N=100000;

figure()
for i=1:3

%Choose between prior distributions
state_dens = i; % 1, 2 or 3


% Set distributions
if state_dens == 1
    x = [125; 125];
    P  = diag([10^2, 5^2]);
elseif state_dens == 2
    x = [-25; 125];
    P  = diag([10^2, 5^2]);
elseif state_dens == 3
    x = [60; 60];
    P  = diag([10^2, 5^2]);
else
    error('State density must be choosen from 1, 2 or 3.')
end


% Set sensor positions
s1 = [0; 100];
s2 = [100; 0];

% Set measurement noise covariance
R = diag([(0.1*pi)/180, (0.1*pi)/180]).^2;

% Create handle for measurement model y = h(x) + R
h = @(x) dualBearingMeasurement(x,s1,s2);


% approximate the transformed gaussian distribution as a gaussian
[y_mean, P_y, y_s] = approxGaussianTransform( x, P, @(x)genNonLinearMeasurementSequence(x,h,R) , N );


% Generate 2D elipse
level = 3;
[xy] = sigmaEllipse2D( y_mean, P_y,level, N );


% Plot the samples, approximated mean and cov
subplot(1,3,i)
hold on
grid on
scatter(y_s(1,:), y_s(2,:),1,'MarkerEdgeColor',[0 .5 .5],...
              'MarkerFaceColor',[0 .7 .7],...
              'LineWidth',1.5)
scatter(y_mean(1),y_mean(2),'white','filled')
scatter(xy(1,:), xy(2,:),1)
legend('Samples','Approximated mean','3\sigma-curve')
title('Approximated mean and covariance for state density ',state_dens)


y_mean
P_y
end







%% 
% Task b)
close all
clc

for i=1:3
    
    % choose between two prior distributions
    state_dens = i; % 1, 2 or 3


    % type of filter update
    type = 'EKF';   % 'EKF', 'UKF' or 'CKF'
    
    
    % Set distributions
    if state_dens == 1
        x = [125; 125];
        P  = diag([10^2, 5^2]);
    elseif state_dens == 2
        x = [-25; 125];
        P  = diag([10^2, 5^2]);
    elseif state_dens == 3
        x = [60; 60];
        P  = diag([10^2, 5^2]);
    else
        error('State density must be choosen from 1, 2 or 3.')
    end

    % Compute the approimated mean and covariances analytically
    [mean_y, P_y] = analyticalDensity(x, P, h, R, type)

end








%% 
% Task c)
close all
clc


% Choose between prior distributions
state_dens = 3; % 1, 2 or 3


% Set distributions
if state_dens == 1
    x = [125; 125];
    P  = diag([10^2, 5^2]);
elseif state_dens == 2
    x = [-25; 125];
    P  = diag([10^2, 5^2]);
elseif state_dens == 3
    x = [60; 60];
    P  = diag([10^2, 5^2]);
else
    error('State density must be choosen from 1, 2 or 3.')
end


% Set sensor positions
s1 = [0; 100];
s2 = [100; 0];

% Set measurement noise covariance
R = diag([(0.1*pi)/180, (0.1*pi)/180]).^2;

% Create handle for measurement model y = h(x) + R
h = @(x) dualBearingMeasurement(x,s1,s2);


% approximate the transformed gaussian distribution as a gaussian
[y_mean, P_y, y_s] = approxGaussianTransform( x, P, @(x)genNonLinearMeasurementSequence(x,h,R) , N );

% Generate 2D elipse
level = 3;
[xy] = sigmaEllipse2D( y_mean, P_y,level, N );

% Plot the samples, approximated mean and cov
figure()
hold on
grid on
scatter(y_s(1,:), y_s(2,:),1,'MarkerEdgeColor',[0 .5 .5],...
              'MarkerFaceColor',[0 .7 .7],...
              'LineWidth',1.5)
scatter(y_mean(1),y_mean(2),'white','filled')
scatter(xy(1,:), xy(2,:))
title('Approximated mean and covariance for state density ',state_dens)

% Analytical approximation
types = {'EKF','UKF','CKF'};
for i = 1:3
   
    % Choose type of KF
    type = types{i};

    % Compute the approimated mean and covariances analytically
    [mean_y, P_y] = analyticalDensity(x, P, h, R, type)

    % Generate 2D elipse
    level = 3;
    [xy] = sigmaEllipse2D( mean_y, P_y,level, N );

    % Plot the analytical mean
    scatter(mean_y(1), mean_y(2),'filled')

    % Plot sigma curve
    scatter(xy(1,:), xy(2,:), 2)

end

% Add legend
legend('Samples','Approximated mean','3\sigma-curve approx','EKF mean',...
    '3\sigma-curve EKF','UKF mean','3\sigma-curve UKF','CKF mean','3\sigma-curve CKF','Location','best')















%% =========================================================================
% QUESTION 2
% Non-linear Kalman filtering

close all
clear all
clc
rng(1)

% Task a/b)

% The initial prior
x0 = [0;  0;  20;  0;  (5*pi)/180];
P0 = diag([10^2, 10^2, 2^2, (pi/180)^2, (pi/180)^2]);

% Sensor location
s1 = [-200;  100];
s2 = [-200; -100];

% Sampling time
T = 1;

% Time steps
N = 100;
n = length(x0);

% Set disturbance case
sigma_v = 1;
sigma_w = pi/180;
disturbance = 2;        % 1, 2 or 3

if disturbance == 1
    sigma_phi_1 = (2*pi)/180;
    sigma_phi_2 = (2*pi)/180;

elseif disturbance == 2
     sigma_phi_1 = (2*pi)/180;
     sigma_phi_2 = (0.1*pi)/180;

elseif disturbance == 3
     sigma_phi_1 = (0.1*pi)/180;
     sigma_phi_2 = (0.1*pi)/180;
else
    error('Disturbance case must be choosen from 1, 2 or 3.')
end


% Process disturbances
Q = diag([0 0 T*(sigma_v) 0 T*(sigma_w)]).^2;

% Measurement noise
R = diag([sigma_phi_1^2 sigma_phi_2^2]);



% Create handle to coordinated turn motion & Generate state sequence
f = @(x) coordinatedTurnMotion(x,T);
X = genNonLinearStateSequence(x0,P0,f,Q,N);

% Create handle to measurement model & generate measurement sequence
h = @(x) dualBearingMeasurement(x,s1,s2);
Y = genNonLinearMeasurementSequence(X,h,R);

% Get the position from the measurements
[xpos, ypos] = getPosFromMeasurement(Y, s1, s2);



% Plot the state trajectory
    
figure()
hold on
grid on
plot(X(1,:),X(2,:),'k','LineWidth',2)
plot([0, xpos],[0, ypos],':','color',[0 0.7 0.7],'LineWidth',2)


% Filter the measurements through the non-linear kalman filter
type = 'EKF';
[xf, Pf, ~, ~] = nonLinearKalmanFilter(Y, x0, P0, f, Q, h, R, type);

% Plot filtered position
plot([0, xf(1,:)], [0, xf(2,:)],'LineWidth',2)

% Plot camera position
scatter(s1(1),s1(2),'k','filled','square')
scatter(s2(1),s2(2),'k','filled','square')


% Plot 3sigma at every fifth instant
for i = 5:5:100

    P_i = Pf(1:2,1:2,i);

    %Generate 2D elipse
    level = 3;
    [xy] = sigmaEllipse2D( [xf(1,i); xf(2,i)], P_i,level, 1000 );

    % Plot sigma curve
    scatter(xy(1,:), xy(2,:), 2,'k','filled')

end

legend('True position','Measured position','Filtered position','Cam 1 pos','Cam 2 pos','3\sigma-curves (5th k)')
title('State trajectory, measured position and filtered position, using ', type)
xlabel('x position'); ylabel('y position');



















%% =============================================================
% Task c)
close all
clc

figure()
for i=1:3
% Set disturbance case
sigma_v = 1;
sigma_w = pi/180;
disturbance = i;        % 1, 2 or 3

if disturbance == 1
    sigma_phi_1 = (2*pi)/180;
    sigma_phi_2 = (2*pi)/180;

elseif disturbance == 2
     sigma_phi_1 = (2*pi)/180;
     sigma_phi_2 = (0.1*pi)/180;

elseif disturbance == 3
     sigma_phi_1 = (0.1*pi)/180;
     sigma_phi_2 = (0.1*pi)/180;
else
    error('Disturbance case must be choosen from 1, 2 or 3.')
end


% Process disturbances
Q = diag([0, 0, T*(sigma_v), 0, T*(sigma_w)]).^2;

% Measurement noise
R = diag([sigma_phi_1, sigma_phi_2]).^2;

% initilalize vectors
MC = 150;
N = 100;
error_mean = zeros(2,MC*N);
error_cov = zeros(2,2,MC);
type = 'UKF';

for imc = 1:MC
    
    % Simulate state sequence
    X = genNonLinearStateSequence(x0, P0, f, Q, N);

    % Simulate measurements
    Y = genNonLinearMeasurementSequence(X, h, R);

    % Run Kalman filter (you need to run all three, for comparison)
    [xf,Pf,xp,Pp] = nonLinearKalmanFilter(Y,x0,P0,f,Q,h,R,type);

    % Determine performance of Kalman filter
    pos_error = xf(1:2,:) - X(1:2,2:end);

    error_mean(:,N*(imc-1)+1:N*imc) = pos_error;

end

% Compute estimation error mean and cov
pos_error = error_mean;
pos_error_mean = mean(pos_error,2)
pos_error_cov = cov(pos_error(:,1), pos_error(:,2))


% Plot histogram of estimation errors
subplot(3,2,(2*i)-1)
hold on
grid on
histogram(pos_error(1,:),'Normalization','pdf')
range = linspace(pos_error_mean(1) - 4*sqrt(pos_error_cov(1,1)), pos_error_mean(1) + 4*sqrt(pos_error_cov(1,1)),1000);
yy = normpdf(range, pos_error_mean(1), sqrt(pos_error_cov(1,1)));
plot(range,yy,'Linewidth',2)
xline(pos_error_mean(1),':','Linewidth',2)
title('Estimation error mean and covariance. Filter type: ',type)
legend('Error distribution bins', 'PDF', 'Estimation error mean')
xlabel('Error in x');
%xlim([range(1), range(end)])

subplot(3,2,2*i)
hold on
grid on
histogram(pos_error(2,:),'Normalization','pdf')
range = linspace(pos_error_mean(2) - 4*sqrt(pos_error_cov(2,2)), pos_error_mean(2) + 4*sqrt(pos_error_cov(2,2)),1000);
yy = normpdf(range, pos_error_mean(2), sqrt(pos_error_cov(2,2)));
plot(range,yy,'Linewidth',2)
xline(pos_error_mean(2),':','Linewidth',2)
title('Estimation error mean and covariance. Filter type: ',type)
legend('Error distribution bins', 'PDF', 'Estimation error mean')
xlabel('Error in y')
%xlim([range(1), range(end)])


end
















%% =============================================================
% QUESTION 3
% Task a)
close all
clear all
clc
rng(6)



% True track
% Sampling period
T = 0.1;
% Length of time sequence
K = 600;
% Allocate memory
omega = zeros(1,K+1);
% Turn rate
omega(150:450) = -pi/301/T;
% Initial state
x0 = [0 0 20 0 omega(1)]';
% Allocate memory
X = zeros(length(x0),K+1);
X(:,1) = x0;
% Create true track
for i=2:K+1
    % Simulate
    X(:,i) = coordinatedTurnMotion(X(:,i-1), T);
    % Set turn−rate
    X(5,i) = omega(i);
end


% Set the intial prior
x0 = [0; 0; 0; 0; 0];
P0 = diag([10^2, 10^2, 10^2, ((5*pi)/180)^2, ((1*pi)/180)^2]);

% Sensor position
s1 = [300; -100];
s2 = [300; -300];

% Measurement noise
sigma_phi_1 = pi/180;
sigma_phi_2 = pi/180;
R = diag([sigma_phi_1, sigma_phi_2]).^2;

% Generate measurements
h = @(x) dualBearingMeasurement(x,s1,s2);
Y = genNonLinearMeasurementSequence(X, h, R);
[xpos, ypos] = getPosFromMeasurement(Y, s1, s2);

f = @(x) coordinatedTurnMotion(x, T);


% Filter the position with non-linear Kalman filter
type = 'EKF';

% Tune process noise covariance
sigma_v = 1;
sigma_w = pi/180;

%Q = diag([0 0 (sigma_v)^2 0 (sigma_w)^2]).*T;
%Q = diag([0 0 (0.01*sigma_v)^2 0 (0.01*sigma_w)^2]).*T;
Q = diag([0 0 (10*sigma_v)^2 0 (10*sigma_w)^2]).*T;

Q

[xf,Pf,xp,Pp] = nonLinearKalmanFilter(Y,x0,P0,f,Q,h,R,type);





% Plot to see the turn
figure()
grid on
hold on
plot(X(1,:),X(2,:),'k','linewidth',2)
plot(xpos,ypos,':','linewidth',2,'color',[0 0.7 0.7])

% Plot filtered position
plot([0, xf(1,:)], [0, xf(2,:)],'LineWidth',2)


% Plot camera position
scatter(s1(1),s1(2),'k','filled','square')
scatter(s2(1),s2(2),'k','filled','square')



% Plot 3sigma at every fifth instant
for i = 5:5:K

    P_i = Pf(1:2,1:2,i);

    %Generate 2D elipse
    level = 3;
    [xy] = sigmaEllipse2D( [xf(1,i); xf(2,i)], P_i,level, 1000 );

    % Plot sigma curve
    scatter(xy(1,:), xy(2,:), 2,'k','filled')

end


% Set title/legend etc
legend('True position','Measured position','Filtered position','Cam 1 pos','Cam 2 pos','3\sigma-curve (5th)')
title('Corrdinated turn motion model filtered with ',type)
xlabel('x pos'); ylabel('y pos')



% ==================================
% Task c)

figure()

for n =1:3
    Pos_error = zeros(1,K);
    if n==1
        Q = diag([0 0 (sigma_v)^2 0 (sigma_w)^2]).*T;
    elseif n == 2
        Q = diag([0 0 (0.01*sigma_v)^2 0 (0.01*sigma_w)^2]).*T;
    else
        Q = diag([0 0 (10*sigma_v)^2 0 (10*sigma_w)^2]).*T;
    end

    [xf,Pf,xp,Pp] = nonLinearKalmanFilter(Y,x0,P0,f,Q,h,R,type);

    for i=1:K
        err = [xf(1,i); xf(2,i)] - [xp(1,i); xp(2,i)];
        Pos_error(i) = sqrt(sum( err.^2));
    end

   
    subplot(3,1,n)
    grid on
    hold on
    plot(1:K, Pos_error,':','LineWidth',2)
    legend('||P_k - P_{k|k} ||_{2}')
    xlabel('time k'); ylabel('Positional error')
    ylim([0 15]);


end








