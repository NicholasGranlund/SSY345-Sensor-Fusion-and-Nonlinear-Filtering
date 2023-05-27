function Y = genLinearMeasurementSequence(X, H, R)
%GENLINEARMEASUREMENTSEQUENCE generates a sequence of observations of the state 
% sequence X using a linear measurement model. Measurement noise is assumed to be 
% zero mean and Gaussian.
%
%Input:
%   X           [n x N+1] State vector sequence. The k:th state vector is X(:,k+1)
%   H           [m x n] Measurement matrix
%   R           [m x m] Measurement noise covariance
%
%Output:
%   Y           [m x N] Measurement sequence
%
%
% EDITED BY: Nicholas Granlund

    % Extract the number of time steps
    N = size(X, 2) - 1;
    
    % Extract the dimensions of the states and the measurements (m measurements and n states)
    [m, n] = size(H);
    
    % Initialize the measurement sequence
    Y = zeros(m, N);
    
    % Generate the measurement sequence
    for k = 1:N
        
        % Measurement
        mu_k = H * X(:, k+1);
        
        % Measurement noise sample
        noise_k = mvnrnd(zeros(length(R),1), R)';
        
        % Compute the measurement at time k
        Y(:, k) = mu_k + noise_k;
    end
    

    
end