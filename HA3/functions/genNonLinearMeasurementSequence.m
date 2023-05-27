function Y = genNonLinearMeasurementSequence(X, h, R)
%GENNONLINEARMEASUREMENTSEQUENCE generates ovservations of the states 
% sequence X using a non-linear measurement model.
%
%Input:
%   X           [n x N+1] State vector sequence
%   h           Measurement model function handle
%   h           Measurement model function handle
%               [hx,Hx]=h(x) 
%               Takes as input x (state) 
%               Returns hx and Hx, measurement model and Jacobian evaluated at x
%   R           [m x m] Measurement noise covariance
%
%Output:
%   Y           [m x N] Measurement sequence
%

% Your code here
% EDITED BY: Nicholas Granlund

% Get dimensions
n = size(X, 1);
N = size(X, 2)-1;
m = size(R, 1);

% Initialize output
Y = zeros(m, N);

% Generate measurements for each state
for k = 1:N
    
    % Extract the states for time instant k
    x_k = X(:, k+1);

    
    % Propagate state through measurement model function
    [hx_k, ~] = h(x_k);
    
    % Add to measurements sequence
    r = mvnrnd(zeros(1,m),[R(1,1), R(2,2)],1)';

    Y(:, k) = hx_k + r;

end

% return

end