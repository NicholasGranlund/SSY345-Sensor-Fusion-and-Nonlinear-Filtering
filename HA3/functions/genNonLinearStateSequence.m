function X = genNonLinearStateSequence(x_0, P_0, f, Q, N)
%GENNONLINEARSTATESEQUENCE generates an N+1-long sequence of states using a 
%    Gaussian prior and a nonlinear Gaussian process model
%
%Input:
%   x_0         [n x 1] Prior mean
%   P_0         [n x n] Prior covariance
%   f           Motion model function handle
%               [fx,Fx]=f(x) 
%               Takes as input x (state), 
%               Returns fx and Fx, motion model and Jacobian evaluated at x
%               All other model parameters, such as sample time T,
%               must be included in the function
%   Q           [n x n] Process noise covariance
%   N           [1 x 1] Number of states to generate
%
%Output:
%   X           [n x N+1] State vector sequence
%

% Your code here

% Initialize state sequence X with size [n x N+1]
X = zeros(length(x_0), N+1);

% Set the initial state
X(:,1) = x_0;

% Propagate the state forward through the nonlinear process model
for k = 2:N+1
    % Compute the motion model and its Jacobian at the previous state
    [f_k, F_k] = f(X(:,k-1));
    
    % Propagate the state forward with the motion model
    X(:,k) = f_k + mvnrnd(zeros(length(x_0),1), Q)';
    
    % THIS IS NOT NEEDED TO FIND X
    % Update the covariance with the motion model Jacobian
    % P_k = F_k*P_0*F_k' + Q;
    
    % Set the current state as the new prior for the next iteration
    % x_0 = X(:,k);
    % P_0 = P_k;
end

end