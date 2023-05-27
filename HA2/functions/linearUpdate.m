function [x, P] = linearUpdate(x, P, y, H, R)
%LINEARPREDICTION calculates mean and covariance of predicted state
%   density using a linear Gaussian model.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%   y           [m x 1] Measurement
%   H           [m x n] Measurement model matrix
%   R           [m x m] Measurement noise covariance
%
%Output:
%   x           [n x 1] updated state mean
%   P           [n x n] updated state covariance
%

% Your code here
% EDITED BY: Nicholas Granlund

   % Kalman gain
    K = P * H' * inv(H * P * H' + R);
    
    % Update state mean and covariance
    x = x + K * (y - H * x);
    P = P - K * H * P;

end