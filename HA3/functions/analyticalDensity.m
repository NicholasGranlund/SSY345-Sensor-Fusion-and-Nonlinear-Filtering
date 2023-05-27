function [mean_y, P_y] = analyticalDensity(x, P, h, R, type)
%ANALYTICALDENSITY Summary of this function goes here
%   Detailed explanation goes here


    % estimate transformed mean and covariance
    if all(type == 'UKF') || all(type == 'CKF')
    
        % Determine som sigma points around x
        [SP,W] = sigmaPoints(x,P,type);
        
        % Propagate sigma points through non-linear function
        [mean_y, P_y]  = propagateSP(h, R, SP, W);
    
    elseif all(type == 'EKF')
        [hx, dhx] = h(x);
        mean_y = hx;
        P_y = (dhx * P * dhx') + R;
    end



function [x, P] = propagateSP(f, R, SP, W)
    n = size(SP,1);
    x = zeros(n,1);
    for i=1:numel(W)
        x = x + f(SP(:,i)) * W(i);
    end   
    P = R; %zeros(n,n);
    for i=1:numel(W)
        P = P + (f(SP(:,i))-x)*(f(SP(:,i))-x).' * W(i);
    end

end
end
