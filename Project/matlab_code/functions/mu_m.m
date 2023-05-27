function [x, P] = mu_m(x, P, mag, m0, Rm)
    % MU_M 
    %
    % By: Nicholas Granlund
    %     Louise Olsson
    %
    %Input:
    %   - x             [4x1]      State vector (quaternions)
    %   - P             [4x4]      Covariance
    %   - mag           [3x1]      Magnetometer measurement
    %   - Rm            [3x3]      Measurement noise
    %   - m0            [3x1]      Nominal magnetic vector
    %
    %Output:
    %   - x             [3x1]    State vector (quaternion)
    %   - P             [4x4]    State covariance matrix


    % Make sure vectors has right dimension
    m0 = reshape(m0, [3,1]);
    fm = zeros(size(m0,1),1); 

    % Measurement model
    hx = Qq(x)'*(m0+fm);
   
    % dH/dq
    [Q0, Q1, Q2, Q3] = dQqdq(x);

    % Measurement model jacobian Hx
    Hx(:,1) = Q0'*(m0+fm);
    Hx(:,2) = Q1'*(m0+fm);
    Hx(:,3) = Q2'*(m0+fm);
    Hx(:,4) = Q3'*(m0+fm);

    % Inovation covariance matrix
    S = Hx*P*Hx' + Rm;

    % Kalman gain
    K = P*Hx'*inv(S);

    % Update state estimate
    x = x + K*(mag-hx);

    % Update covariance estimate
    P = P - K*S*K';
 
    % Normalize the quaternion
    [x, P] = mu_normalizeQ(x, P);

    % return
end

