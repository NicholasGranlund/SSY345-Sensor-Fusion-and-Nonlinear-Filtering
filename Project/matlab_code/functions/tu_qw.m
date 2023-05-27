function [x, P] = tu_qw(x, P, omega, T, Rw)
    % TU_QW: Time update for quaternion given 
    % gyroscope measurements.
    %
    % By: Nicholas Granlund
    %     Louise Olsson
    %
    %Input:
    %   - x             [4x1]    State vector (quaternion)  
    %   - P             [4x4]    State covariance matrix
    %   - omega         [3x1]    Angular velocity vector
    %   - T                      Time step
    %   - Rw            [4x4]    Process noise covariance matrix
    %
    %Output:
    %   - x             [3x1]    State vector (quaternion)
    %   - P             [4x4]    State covariance matrix

    % Compute x^{k} = F(w^{k-1))q^{k-1} + G(q^{k-1})v^{k-1} 
    F = eye(4) + (T/2)*(Somega(omega));
    G = (T/2)*Sq(x);

    % Predict x and P
    x = F*x;
    P = F*P*F' + G*Rw*G';

    % Normalize x and P, (only x is normalized, must have unit length)
    [x, P] = mu_normalizeQ(x, P);

    % return

end

