function [X,T,Tvec] = generateTrueTrack()
% True track
% Sampling period
T = 0.1;

% Length of time sequence
K = 600;

% Allocate memory
omega = zeros(1,K+1);

% Turn rate
omega(150:450) = -pi/301/T;

% Initial state
x0 = [0 0 20 0 omega(1)]';

% Allocate memory
X = zeros(length(x0),K+1);
X(:,1) = x0;

% Create true track
for i=2:K+1
    % Simulate
    X(:,i) = coordinatedTurnMotion(X(:,i-1), T);
    % Set turn−rate
    X(5,i) = omega(i);
end

Tvec = 0:T:(length(X)-1)*T;

end

