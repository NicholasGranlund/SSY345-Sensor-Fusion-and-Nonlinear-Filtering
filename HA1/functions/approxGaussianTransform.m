function [mu_y, Sigma_y, y_s] = approxGaussianTransform(mu_x, Sigma_x, f, N)
%approxGaussianTransform takes a Gaussian density and a transformation 
%function and calculates the mean and covariance of the transformed density.
%
%Inputs
%   MU_X        [m x 1] Expected value of x.
%   SIGMA_X     [m x m] Covariance of x.
%   F           [Function handle] Function which maps a [m x 1] dimensional
%               vector into another vector of size [n x 1].
%   N           Number of samples to draw. Default = 5000.
%
%Output
%   MU_Y        [n x 1] Approximated mean of y.
%   SIGMA_Y     [n x n] Approximated covariance of y.
%   ys          [n x N] Samples propagated through f


if nargin < 4
    N = 5000;
end

%Your code here
% Edited by: Nicholas Granlund

% 1. Draw N samples from the Gaussian density
x_s = mvnrnd(mu_x, Sigma_x, N)';

% 2. Apply the non-linear function
y_s = f(x_s);

% 3. Calculate the mean and covariance of y_s
mu_y = mean(y_s,2);
Sigma_y = cov(y_s');

% return

end

