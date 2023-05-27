function [x, P] = nonLinKFprediction(x, P, f, Q, type)
%NONLINKFPREDICTION calculates mean and covariance of predicted state
%   density using a non-linear Gaussian model.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%   f           Motion model function handle
%               [fx,Fx]=f(x) 
%               Takes as input x (state), 
%               Returns fx and Fx, motion model and Jacobian evaluated at x
%               All other model parameters, such as sample time T,
%               must be included in the function
%   Q           [n x n] Process noise covariance
%   type        String that specifies the type of non-linear filter
%
%Output:
%   x           [n x 1] predicted state mean
%   P           [n x n] predicted state covariance
%
% EDITED BY: Nicholas Granlund
    
    n = length(x);

    switch type
        case 'EKF'
            
            % Your EKF code here
            
            % 1. Linearize the system
            [fx,Fx]=f(x); 
              
            % 2. Return as it would be linear
            x = fx;
            P = Fx*P*Fx' + Q;
            
            
            
            
            
        case 'UKF'
    
            % Your UKF code here
            
            % 1. Form a set of 2n+1 sigma points
            [SP,W] = sigmaPoints(x, P, 'UKF');
            
            % 2. Compute the predicted moments
            x_pred = zeros(n,1);
            P_pred = Q;
            
            % Iterate through and sum
            for i=1:length(SP)
                [fx,]=f(SP(:,i));
                x_pred = x_pred + fx*W(i);
                
                [fx,]=f(SP(:,i))-x; 
                P_pred = P_pred + fx*fx'*W(i);
            end
            
            % 3. Return prediction mean and covariance
            x = x_pred;
            P = P_pred;
           
            
            % 4. Make sure the covariance matrix is semi-definite
            if min(eig(P))<=0
                [v,e] = eig(P, 'vector');
                e(e<0) = 1e-4;
                P = v*diag(e)/v;
            end
            
            
            
            
            
            
        case 'CKF'
            
            % Your CKF code here
            
            % 1. Form a set of 2n sigma points
            [SP,W] = sigmaPoints(x, P, 'CKF');
            
            % 2. Compute the predicted moments
            x_pred = zeros(n,1);
            P_pred = Q;
            
           for i=1:length(SP)
                [fx,]=f(SP(:,i)); 
                x_pred = x_pred + fx*W(i);
                
                [fx,]=f(SP(:,i))-x; 
                P_pred = P_pred + fx*fx'*W(i);
            end
            
            % 3. Return prediction mean and covariance
            x = x_pred;
            P = P_pred;
            
        otherwise
            error('Incorrect type of non-linear Kalman filter')
    end

end