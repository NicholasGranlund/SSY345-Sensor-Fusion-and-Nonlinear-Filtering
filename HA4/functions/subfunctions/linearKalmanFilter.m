function [X, P] = linearKalmanFilter(Y, x_0, P_0, f, Q, h, R)
    % KALMANFILTER Filters measurements sequence Y using a Kalman filter. 
    %
    % Input:
    %   Y           [m x N] Measurement sequence
    %   x_0         [n x 1] Prior mean
    %   P_0         [n x n] Prior covariance
    %   A           [n x n] State transition matrix
    %   Q           [n x n] Process noise covariance
    %   H           [m x n] Measurement model matrix
    %   R           [m x m] Measurement noise covariance
    %
    % Output:
    %   x           [n x N] Estimated state vector sequence
    %   P           [n x n x N] Filter error convariance
    %

    % Parameters
    N = size(Y,2);

    n = length(x_0);
    m = size(Y,1);

    % Data allocation
    x = zeros(n,   N +1);
    P = zeros(n,n, N +1);

    % filter
    x(:,1)   = x_0;
    P(:,:,1) = P_0;
    
    for i=1:N
        % prediction step: compute p(x_k | y_1:k-1) from p(x_k-1 | y_1:k-1)
        [x(:,i+1), P(:,:,i+1)] = linearKalmanPrediction(x(:,i), P(:,:,i), f, Q);

        % update step: compute p(x_k | y_1:k) from p(x_k | y_1:k-1)
        [x(:,i+1), P(:,:,i+1)] = linearKalmanUpdate(x(:,i+1), P(:,:,i+1), Y(:,i), h, R);
    end
    
    % exclude prior x0~N(x_0, P_0) from posterior
    X = x(:,2:end);
    P = P(:,:,2:end);
    
end
