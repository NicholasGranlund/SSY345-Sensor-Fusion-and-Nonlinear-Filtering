function plotPosteriorApproximation(xf, Pf, xfp, Pfp, xfp_resam, Pfp_resam, time_instances);

    %PLOTPOSTERIORAPPROXIMATION Summary of this function goes here
    %   Detailed explanation goes here

    % Create figure
    figure()

    for i=1:3

        % change plot handle
        subplot(3,1,i)
        hold on; grid on

        mean = xf(time_instances(i));
        sigma = sqrt(Pf(:,:,time_instances(i)));
        x = linspace(mean-(4*sigma), (mean+4*sigma));
        y = pdf('Normal',x,mean,sigma);
        xlim([x(10) x(end-10)]);
     
        % Plot posterior from Kalman filter
        plot(x,y,'color','#EDB120','Linewidth',2)

        mean = xfp(time_instances(i));
        sigma = sqrt(Pfp(:,:,time_instances(i)));
        x = linspace(mean-(4*sigma), mean+(4*sigma));
        y = pdf('Normal',x,mean,sigma);

        plot(x,y,'color','#A2142F','LineWidth',2)

        mean = xfp_resam(time_instances(i));
        sigma = sqrt(Pfp_resam(:,:,time_instances(i)));
        x = linspace(mean-(4*sigma), mean+(4*sigma));
        y = pdf('Normal',x,mean,sigma);

        plot(x,y,'color','#77AC30','LineWidth',2)

        % Legend Title etc.
        legend('Kalman filtered posterior',...
        'Particle filtered posterior','Particle filtered w/ resampling posterior');
        xlabel('posterior mean');
        
    end



    




end

