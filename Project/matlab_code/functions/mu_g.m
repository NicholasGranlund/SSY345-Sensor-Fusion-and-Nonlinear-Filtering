function [x, P] = mu_g(x, P, yacc, Ra, g0)
    % MU_G Summary of this function goes here
    %
    % By: Nicholas Granlund
    %     Louise Olsson
    %
    %Input:
    %   - x             [4x1]      State vector (quaternions)
    %   - P             [4x4]      Covariance
    %   - yacc          [3x1]      Accelerometer measurement
    %   - Ra            [3x3]      Measurement noise
    %   - g0            [3x1]      Nominal gravity vector
    %
    %Output:
    %   - x             [3x1]    State vector (quaternion)
    %   - P             [4x4]    State covariance matrix
    
    % Make sure vectors has right dimension
    g0 = reshape(g0, [3,1]);
   
    fa = zeros(size(g0,1),1); 

    % Measurement model
    hx = Qq(x)'*(g0+fa);
   
    % dH/dq
    [Q0, Q1, Q2, Q3] = dQqdq(x);

    % Measurement model jacobian Hx
    Hx(:,1) = Q0'*(g0+fa);
    Hx(:,2) = Q1'*(g0+fa);
    Hx(:,3) = Q2'*(g0+fa);
    Hx(:,4) = Q3'*(g0+fa);
  
    % Innovation Covariance S
    S = Hx*P*Hx'+Ra;

    % Kalman Gain K
    K = P*Hx'*inv(S);

    % Update state estimate
    x = x + K*(yacc-hx);

    % Update covariance estimate
    P = P - K*S*K';

    % Normalize the quaternion
    [x, P] = mu_normalizeQ(x, P);

    % return

end

