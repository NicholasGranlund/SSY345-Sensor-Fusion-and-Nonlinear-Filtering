function [xs, Ps] = nonLinRTSSupdate(xs_kplus1, ...
                                     Ps_kplus1, ...
                                     xf_k, ... 
                                     Pf_k, ...
                                     xp_kplus1, ...
                                     Pp_kplus1, ...
                                     f, ...
                                     T, ...
                                     sigmaPoints, ...
                                     type)
%NONLINRTSSUPDATE Calculates mean and covariance of smoothed state
% density, using a non-linear Gaussian model.
%
%Input:
%   xs_kplus1   Smooting estimate for state at time k+1
%   Ps_kplus1   Smoothing error covariance for state at time k+1
%   xf_k        Filter estimate for state at time k
%   Pf_k        Filter error covariance for state at time k
%   xp_kplus1   Prediction estimate for state at time k+1
%   Pp_kplus1   Prediction error covariance for state at time k+1
%   f           Motion model function handle
%   T           Sampling time
%   sigmaPoints Handle to function that generates sigma points.
%   type        String that specifies type of non-linear filter/smoother
%
%Output:
%   xs          Smoothed estimate of state at time k
%   Ps          Smoothed error convariance for state at time k

% Your code here.
% Edited by: Nicholas Granlund

    % The RTS algorithm
    % 1. Run a kalman filter for k = 1,2,..,K

    % EKF model
    if all(type == 'EKF')
        % Propagate motion model at x_{k|k}
        [~, F_xk] = f(xf_k,T);
        
        % Approximate covariance P_{k,k+1|k}
        Pkp1 = Pf_k * F_xk';
      
    % UKF or CKF model
    else
        % Generate sigma points
        [SP,W] = sigmaPoints(xf_k, Pf_k, type);
        
        % Approximate covariance P_{k,k+1|k}
        Pkp1 = zeros(size(xf_k,1));
        for i=1:numel(W)
            Pkp1 = Pkp1 + (SP(:,i)-xf_k)*(f(SP(:,i),T)-xp_kplus1).' * W(i);
        end
    end

    % 2. For k = K-1,...,1, compute;
    % (Lecture 7)

    % Smoothing gain G_k = P_{k|k} * A_{k}^T * inv( P_{k+1|k} )
    %                G_k = P_{k,k+1|k} * inv( P_{k+1|k} )
    G_k =  Pkp1 * inv(Pp_kplus1);

    % Smoothed estimate and covariance at time k
    xs = xf_k + G_k*( xs_kplus1 - xp_kplus1 );
    Ps = Pf_k - G_k*( Pp_kplus1 - Ps_kplus1 )*G_k';
    
    % return

end