function plotParticleFilterErrorbar(X,Y,K,xf,Pf,xfp,Pfp,xfp_resam,Pfp_resam)
%PLOTPARTICLEFILTERERRORBAR Summary of this function goes here
%   Detailed explanation goes here


    figure()
    grid on; hold on

    % Plot true state
    plot(0:K,X,'k','LineWidth',2);

    % Plot Kalman filtered estimate w/ errorbar
    errorbar(1:K,xf,reshape(Pf,[1,length(Pf)]),'LineWidth',3,'color','#EDB120')

    % Plot Partice filtered estimate w/ errorbar
    errorbar(1:K,xfp,reshape(Pfp,[1,length(Pfp)]),'LineWidth',2,'color','#A2142F')

    % Plot Partice filtered estimate w/ errorbar w/ resampling
    errorbar(1:K,xfp_resam,reshape(Pfp_resam,[1,length(Pfp_resam)]),'LineWidth',1,'color','#77AC30')

    % Show axis
    xline(0); yline(0);

    % Legend Title etc.
    legend('True state',...
        'Kalman filtered state trajectory',...
        'Particle filtered','Particle filtered w/ resampling')
    xlabel('Timesteps k'); ylabel('x_k');
       ylim([-15 10])


end

