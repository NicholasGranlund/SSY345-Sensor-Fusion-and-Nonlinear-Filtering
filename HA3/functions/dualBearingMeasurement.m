function [hx, Hx] = dualBearingMeasurement(x, s1, s2)
%DUOBEARINGMEASUREMENT calculates the bearings from two sensors, located in 
%s1 and s2, to the position given by the state vector x. Also returns the
%Jacobian of the model at x.
%
%Input:
%   x           [n x 1] State vector, the two first element are 2D position
%   s1          [2 x 1] Sensor position (2D) for sensor 1
%   s2          [2 x 1] Sensor position (2D) for sensor 2
%
%Output:
%   hx          [2 x 1] measurement vector
%   Hx          [2 x n] measurement model Jacobian
%
% NOTE: the measurement model assumes that in the state vector x, the first
% two states are X-position and Y-position.

% Your code here

% % Extract the position from the state vector
% pos = x(1:2);
% 
% % Calculate the bearings to the position from equation (2)
% b1 = atan2(pos(2)-s1(2), pos(1)-s1(1));
% b2 = atan2(pos(2)-s2(2), pos(1)-s2(1));
% 
% % Create the measurement vector
% hx = [b1; 
%       b2];
% 
% % Calculate the Jacobian of the measurement model
% dx1 = pos(1) - s1(1);
% dy1 = pos(2) - s1(2);
% q1 = dx1^2 + dy1^2;
% 
% dx2 = pos(1) - s2(1);
% dy2 = pos(2) - s2(2);
% q2 = dx2^2 + dy2^2;
% 
% Hx = [-dy1/q1, dx1/q1, 0, 0, 0;
%       -dy2/q2, dx2/q2, 0, 0, 0];

%Hx = [-dy1/q1, dx1/q1;
%      -dy2/q2, dx2/q2];

  % initialize outputs with correct sizes
    n  = size(x,1);
    N = size(x,2);
    hx = zeros(2,N);
    Hx = zeros(2,n);

    % calculate readings from the two sensors
    ang1 = atan2( x(2,:)-s1(2), x(1,:)-s1(1) );
    ang2 = atan2( x(2,:)-s2(2), x(1,:)-s2(1) );
    
    % output is the concatenation of the two readings
    hx(1:2,:) = [ang1; 
                 ang2];
    
    % jacobian of hx, as calculated using the symbolic toolbox
    Hx(1:2,1:2) = [-(x(2)-s1(2)) / ( (x(1)-s1(1))^2 + (x(2)-s1(2))^2 ), (x(1)-s1(1)) / ((x(1)-s1(1))^2 + (x(2)-s1(2))^2);
                   -(x(2)-s2(2)) / ( (x(1)-s2(1))^2 + (x(2)-s2(2))^2 ), (x(1)-s2(1)) / ((x(1)-s2(1))^2 + (x(2)-s2(2))^2)];




end