function [xfp, Pfp, Xp, Wp] = pfFilter(x_0, P_0, Y, proc_f, proc_Q, meas_h, meas_R, ...
                             N, bResample, plotFunc)
%PFFILTER Filters measurements Y using the SIS or SIR algorithms and a
% state-space model.
%
% Input:
%   x_0         [n x 1] Prior mean
%   P_0         [n x n] Prior covariance
%   Y           [m x K] Measurement sequence to be filtered
%   proc_f      Handle for process function f(x_k-1)
%   proc_Q      [n x n] process noise covariance
%   meas_h      Handle for measurement model function h(x_k)
%   meas_R      [m x m] measurement noise covariance
%   N           Number of particles
%   bResample   boolean false - no resampling, true - resampling
%   plotFunc    Handle for plot function that is called when a filter
%               recursion has finished.
% Output:
%   xfp         [n x K] Posterior means of particle filter
%   Pfp         [n x n x K] Posterior error covariances of particle filter
%   Xp          [n x N x K] Non-resampled Particles for posterior state distribution in times 1:K
%   Wp          [N x K] Non-resampled weights for posterior state x in times 1:K

% Your code here
% Edited by: Nicholas Granlund

    % Get dimensions of task
    n = size(x_0,1);
    K = size(Y,2);

    % Initiallize output
    xfp = zeros(n,K);
    Pfp = zeros(n,n,K);
    Xp = zeros(n,N,K);
    Wp = zeros(N,K);
    
    % Insert prior distribution
    Xp(:,:,1) = mvnrnd(x_0,P_0,N)';
    Wp(:,1)  = 1/N * ones(1,N);

    
    % Iterate through the timesteps k=1,2,...,.K
    for k=2:K+1
        
        % Extract the posterior from previous tiestep k-1
        Xp_previous = Xp(:,:,k-1);
        Wp_previous = Wp(:,k-1)';        % <-- Transpose!
        

%         % Resample if bResample == True
%         if bResample
%             [Xp_previous, Wp_previous,] = resampl(Xp_previous, Wp_previous);   
%         end
            
        % Particle filter step
        [Xp(:,:,k), Wp(:,k)] = pfFilterStep( Xp_previous, Wp_previous, Y(:,k-1), proc_f, proc_Q, meas_h, meas_R);

        % Resample if bResample == True
        if bResample
            [Xp(:,:,k), Wp(:,k)] = resampl(Xp(:,:,k), Wp(:,k)');   
        end


        % estimate mean and covariance given the particles
        xfp(:,k)   = sum( Xp(:,:,k).*Wp(:,k)' ,2 );
        Pfp(:,:,k) = Wp(:,k)'.*(Xp(:,:,k) - xfp(:,k))*(Xp(:,:,k) - xfp(:,k))';
        
        % repeat
        
    end
    
    % Remove prior
    xfp(:,1)   = [];
    Pfp(:,:,1) = [];
    Xp(:,:,1)  = [];
    Wp(:,1)    = [];
    
    % return
    
end
