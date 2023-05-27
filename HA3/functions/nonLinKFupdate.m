function [x, P] = nonLinKFupdate(x, P, y, h, R, type)
%NONLINKFUPDATE calculates mean and covariance of predicted state
%   density using a non-linear Gaussian model.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%   y           [m x 1] measurement vector
%   h           Measurement model function handle
%               [hx,Hx]=h(x) 
%               Takes as input x (state), 
%               Returns hx and Hx, measurement model and Jacobian evaluated at x
%               Function must include all model parameters for the particular model, 
%               such as sensor position for some models.
%   R           [m x m] Measurement noise covariance
%   type        String that specifies the type of non-linear filter
%
%Output:
%   x           [n x 1] updated state mean
%   P           [n x n] updated state covariance
%
% EDITED BY: Nicholas Granlund

n = length(x);

    switch type
        case 'EKF'
            
            % Your EKF update here
            
            % 1. Calculate S_k and kalman gain K_k
            [hx,Hx]=h(x);
            S = Hx*P*Hx' + R;
            K = P*Hx'*inv(S);
             
            % 2. Calculate the updated mean and covariance
            x = x + K*(y - hx);
            P = P - K*S*K';
            
            % 3. Return
            
            
            
            
            
            
            
        case 'UKF'
    
            % Your UKF update here
            
            % 1. Form a set of 2n+1 sigma points
            [SP,W] = sigmaPoints(x, P, 'UKF');
            
            % 2. Compute the desired moments
            y_hat = 0;
            for i=1:(2*n)+1
                [hx,]=h(SP(:,i));
                y_hat = y_hat + hx*W(i);
            end
            
            P_xy = 0;
            for i=1:(2*n)+1
                [hx,]=h(SP(:,i));
                P_xy = P_xy + ((SP(:,i) - x) * (hx - y_hat).' * W(i));
            end
            
            S = R;
            for i=1:(2*n)+1
                [hx,]=h(SP(:,i));
                S = S + (hx - y_hat)*(hx - y_hat).'*W(i);
            end
            
            % 3. Calculate the updated mean and covariance
            x = x + P_xy*inv(S)*(y - y_hat);
            P = P - P_xy*inv(S)*P_xy';
            
              
            % Make sure the covariance matrix is semi-definite
            if min(eig(P))<=0
                [v,e] = eig(P, 'vector');
                e(e<0) = 1e-4;
                P = v*diag(e)/v;
            end
            
            
            
            
            
            
            
            
        case 'CKF'
    
            % Your CKF update here
            
            % 1. Form a set of 2n sigma points
            [SP,W] = sigmaPoints(x, P, 'CKF');
            
            % 2. Compute the desired moments
            y_hat = 0;
            for i=1:(2*n)
                [hx,]=h(SP(:,i));
                y_hat = y_hat + hx*W(i);
            end
            
            P_xy = 0;
            for i=1:(2*n)
                [hx,]=h(SP(:,i));
                P_xy = P_xy + ((SP(:,i) - x) * (hx - y_hat).' * W(i));
            end
            
            S = R;
            for i=1:(2*n)
                [hx,]=h(SP(:,i));
                S = S + (hx - y_hat)*(hx - y_hat).'*W(i);
            end
            
            % 3. Calculate the updated mean and covariance
            x = x + P_xy*inv(S)*(y - y_hat);
            P = P - P_xy*inv(S)*P_xy';
            
        otherwise
            error('Incorrect type of non-linear Kalman filter')
    end

end

