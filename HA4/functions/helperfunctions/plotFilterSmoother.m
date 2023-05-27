function plotFilterSmoother(X,xs, Ps, xf, Pf, xp, Pp, xpos, ypos, s1, s2, type, Tvec)
    %PLOTFILTERSMOOTHER Summary of this function goes here
    %   Helper function for plotting of the filter coordinated turn motion
    %   model


    figure()
    grid on; hold on
    plot(X(1,:),X(2,:),'k','LineWidth',2)

    plot(xpos,ypos,':','color','#808080','LineWidth',2)
    plot(xf(1,:),xf(2,:),'LineWidth',2)
    plot(xs(1,:),xs(2,:),'LineWidth',2)
    
    % Plot camera position
    scatter(s1(1),s1(2),'k','filled','square')
    scatter(s2(1),s2(2),'k','filled','square')

    
    % Plot 3sigma at every fifth instant
    for i = 5:5:length(X)-1
    
        P_i = Pf(1:2,1:2,i);
    
        %Generate 2D elipse
        [xy] = sigmaEllipse2D( [xf(1,i); xf(2,i)], P_i, 3, 1000);
    
        % Plot sigma curve
        %scatter(xy(1,:), xy(2,:), 2,'k','filled')
        plot(xy(1,:),xy(2,:),'k')
    
    end

    % Plot 3sigma at every fifth instant
    for i = 5:5:length(X)-1
    
        P_i = Ps(1:2,1:2,i);
    
        %Generate 2D elipse
        [xy] = sigmaEllipse2D( [xs(1,i); xs(2,i)], P_i, 3, 1000);
   
    
        % Plot sigma curve
        plot(xy(1,:), xy(2,:),'color','#A2142F')
    
    end
    
    % Set title/legend etc
    legend('True position','Measured position','Filtered position','Smoothed position','Cam 1 pos','Cam 2 pos','3\sigma-contour filter','3\sigma-contour smoother')
    title('Corrdinated turn motion model filtered with ',type)
    xlabel('x pos'); ylabel('y pos')
    
    
    figure()
    subplot(1,3,1)
    grid on; hold on
    plot(Tvec,X(3,:),'k','LineWidth',2)
    plot(Tvec(2:end),xf(3,:),'Color','#EDB120')
    plot(Tvec(2:end),xs(3,:),'Color','#7E2F8E','LineWidth',2)
    ylabel('Velocity [m/s]'); xlabel('time [s]')
    legend('True state trajectory','Filtered state trajectory','Smoothed state trajectory')
    
    subplot(1,3,2)
    grid on; hold on
    plot(Tvec,X(4,:),'k','LineWidth',2)
    plot(Tvec(2:end),xf(4,:),'Color','#EDB120')
    plot(Tvec(2:end),xs(4,:),'Color','#7E2F8E','LineWidth',2)
    ylabel('Turn (heading) [rad]'); xlabel('time [s]')
    legend('True state trajectory','Filtered state trajectory','Smoothed state trajectory')
    
    subplot(1,3,3)
    grid on; hold on
    plot(Tvec,X(5,:),'k','LineWidth',2)
    plot(Tvec(2:end),xf(5,:),'Color','#EDB120')
    plot(Tvec(2:end),xs(5,:),'Color','#7E2F8E','LineWidth',2)
    ylabel('Turn rate [rad/s]'); xlabel('time [s]')
    legend('True state trajectory','Filtered state trajectory','Smoothed state trajectory')

end

