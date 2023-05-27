function [SP,W] = sigmaPoints(x, P, type)
% SIGMAPOINTS computes sigma points, either using unscented transform or
% using cubature.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%
%Output:
%   SP          [n x 2n+1] UKF, [n x 2n] CKF. Matrix with sigma points
%   W           [1 x 2n+1] UKF, [1 x 2n] UKF. Vector with sigma point weights 
%
% EDITED BY: Nicholas Granlund

    % Determine n
    n = length(x);

    switch type        
        case 'UKF'
    
            % your code
        
            % Compute sigma points using Unscented Transform
             SP = zeros(n,2*n+1);
             W = zeros(1,2*n+1);
             
             % Sigma point 0 & weight 0
             SP(:,1) = x;
             W(1) = 1 - (n/3);
             
             % Sigma point i & weight i
             Pchol = chol(P,'lower');
             for i = 2:(n+1)
                SP(:,i) = x + (sqrt(n / (1 - W(1)))*Pchol(:,i-1));
                SP(:,i+n) = x - (sqrt(n / (1 - W(1)))*Pchol(:,i-1));
             end
             
             W(2:end) = (1 - W(1)) / (2*n);
                
             
                
        case 'CKF'
            
            % your code
        
            % Compute sigma points using Cubature Rule
             SP = zeros(n,2*n);
             W = zeros(1,2*n);
             
             % Sigma point i & weight i
             Pchol = chol(P,'lower');
             for i = 1:n
                SP(:,i)   = x + sqrt(n)*Pchol(:,i);
                SP(:,i+n) = x - sqrt(n)*Pchol(:,i);
             end
            
             W(:) = 1/(2*n);
        
            
        otherwise
            error('Incorrect type of sigma point')
    end

end