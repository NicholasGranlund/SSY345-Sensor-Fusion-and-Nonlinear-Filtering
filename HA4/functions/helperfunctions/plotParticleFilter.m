function plotParticleFilter(X,Y,K,xf,Pf,xfp,Pfp,xfp_resam, Pfp_resam)
%PLOTPARTICLEFILTER Summary of this function goes here
%   Detailed explanation goes here


    figure()
    grid on; hold on; axis on

    % Plot true state
    plot(0:K,X,'k','LineWidth',2);

    % Plot measured
    plot(1:K,Y,':','LineWidth',2,'color','#808080');

    % Plot kalman filtered estimate
    plot(1:K,xf,'LineWidth',2,'color','#EDB120');

    % Plot particle filtered estimate NOT RESAMPLED
    plot(1:K,xfp,'LineWidth',2,'color','#A2142F');

    % Plot particle filtered estimate RESAMPLED
    plot(1:K,xfp_resam,'LineWidth',1,'color','#77AC30');

    % Show axis
    xline(0); yline(0);

    % Legend Title etc.
    legend('True state','Measured state',...
        'Kalman filtered state trajectory',...
        'Particle filtered','Particle filtered w/ resampling');
    xlabel('Timesteps k'); ylabel('x_k');
    ylabel('x_k');
    ylim([-15 10])

    % Return
    
end

