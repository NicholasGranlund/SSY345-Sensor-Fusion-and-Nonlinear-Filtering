function plotFilterEstimate(xhat, meas)
    

    anglesOUR = rad2deg(q2euler(xhat.x));
    anglesGOOGLE = rad2deg(q2euler(meas.orient));


    figure()

    
    subplot(3,1,1);
    hold on; grid on; title('YAW  (z-axis)')
    plot(xhat.t(1:end-1),anglesOUR(1,1:end-1),'Linewidth',2)
    plot(meas.t(1:end-1),anglesGOOGLE(1,1:end-1),':k','Linewidth',2)
    legend('Our estimate','MATLAB estimate')
    ylim([-140, 140])
    
    subplot(3,1,2);
    hold on; grid on; title('ROLL (y-axis)')
    plot(xhat.t(1:end-1),anglesOUR(2,1:end-1),'Linewidth',2)
    plot(meas.t(1:end-1),anglesGOOGLE(2,1:end-1),':k','Linewidth',2)
    legend('Our estimate','MATLAB estimate')
    ylim([-140, 140])

    subplot(3,1,3);
    hold on; grid on; title('PITCH (x-axis)')
    plot(xhat.t(1:end-1),anglesOUR(3,1:end-1),'Linewidth',2)
    plot(meas.t(1:end-1),anglesGOOGLE(3,1:end-1),':k','Linewidth',2)
    legend('Our estimate','MATLAB estimate')
    ylim([-140, 140])

    clc




end

