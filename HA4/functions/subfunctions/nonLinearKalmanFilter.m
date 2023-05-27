function [xf, Pf, xp, Pp] = nonLinearKalmanFilter(Y, x_0, P_0, f, Q, h, R, type, T)
%NONLINEARKALMANFILTER Filters measurement sequence Y using a 
% non-linear Kalman filter. 
%
%Input:
%   Y           [m x N] Measurement sequence for times 1,...,N
%   x_0         [n x 1] Prior mean for time 0
%   P_0         [n x n] Prior covariance
%   f                   Motion model function handle
%                       [fx,Fx]=f(x) 
%                       Takes as input x (state) 
%                       Returns fx and Fx, motion model and Jacobian evaluated at x
%   Q           [n x n] Process noise covariance
%   h                   Measurement model function handle
%                       [hx,Hx]=h(x,T) 
%                       Takes as input x (state), 
%                       Returns hx and Hx, measurement model and Jacobian evaluated at x
%   R           [m x m] Measurement noise covariance
%
%Output:
%   xf          [n x N]     Filtered estimates for times 1,...,N
%   Pf          [n x n x N] Filter error convariance
%   xp          [n x N]     Predicted estimates for times 1,...,N
%   Pp          [n x n x N] Filter error convariance
%
%

% Your code here. If you have good code for the Kalman filter, you should re-use it here as
% much as possible.
%
% EDITED BY: Nicholas Granlund

% Number of measurements
N = size(Y,2);

N

% Number of states
n = length(x_0);

% Initialize outputs
xf = zeros(n,N);
Pf = zeros(n,n,N);
xp = zeros(n,N);
Pp = zeros(n,n,N);

% Iterate through the measurements
for i =1:N
    
    % Prediction
    [x_pred, P_pred] = nonLinKFprediction(x_0, P_0, f, T, Q, type);
    xp(:,i) = x_pred;
    Pp(:,:,i) = P_pred;
    
    % Update
    [x_f, P_f] = nonLinKFupdate(x_pred, P_pred, Y(:,i), h, T, R, type);
    xf(:,i) = x_f;
    Pf(:,:,i) = P_f;
    
    % update x_0 and P_0 for next time instant k (x_1 x_2 x_3 ... x_N) (P_1 P_2 P_3 ... P_N)
    x_0 = xf(:,i);
    P_0 = Pf(:,:,i);

end