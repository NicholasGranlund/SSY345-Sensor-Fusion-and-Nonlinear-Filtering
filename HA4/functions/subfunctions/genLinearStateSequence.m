function X = genLinearStateSequence(x_0, P_0, f, Q, K)
    % GENLINEARSTATESEQUENCE generates an N-long sequence of states using a 
    %    Gaussian prior and a linear Gaussian process model
    %
    % Input:
    %   x_0         [n x 1] Prior mean
    %   P_0         [n x n] Prior covariance
    %   f                   Function handle for 
    %   Q           [n x n] Process noise covariance
    %   K           [1 x 1] Number of states to generate
    %
    % Output:
    %   X           [n x N+1] State vector sequence
    
    % Number of states
    n = length(x_0);

    % Initialize X output
    X = zeros(n,K);
    
    % Sample initial state from the prior distribution
    X(:,1) = mvnrnd(x_0, P_0)';

    % Iterate to generate K samples
    for i=1:K
        % Propagate through motion model
        X(:,i+1) =  f(X(:,i)) + mvnrnd(zeros(n,1), Q)';
    end

end