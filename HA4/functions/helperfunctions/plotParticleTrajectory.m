function plotParticleTrajectory(Xp,Wp,K,N,X,xfp,Pfp,bResampl)
    %PLOTPARTICLETRAJECTORY Summary of this function goes here
    %   Detailed explanation goes here
    
    
    figure()
    hold on; grid on

    % Plot true state
    a = plot(1:K,X(1:end-1),'k','LineWidth',2);

    % Decide color
    if ~bResampl
        customColor = '#A2142F';
    else
        customColor = '#77AC30';
    end

    % Plot Partice filtered estimate w/ errorbar
    b = errorbar(1:K,xfp,reshape(Pfp,[1,length(Pfp)]),'LineWidth',2,'color',customColor);

    % Plot particle traj
    for i=1:N
        p1 = plot(1:K,reshape(Xp(:,i,:),[1,K]),'-','color',customColor);
        p1.Color(4) = 0.2;
        hold on
    end


    % Show axis
    xline(0); yline(0);

    % plot attributes
    legend('True state','PF filtered','Particle paths')
    uistack(b, 'top')
    uistack(a, 'top')
    xlabel('Timesteps k'); ylabel('x_k');
    xlim([1 length(xfp)]);
    ylim([-15 15]);

end

