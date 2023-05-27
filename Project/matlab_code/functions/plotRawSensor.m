function plotRawSensor(acc,gyr,Hz)
    % PLOTRAWSENSOR
    %
    % By: Nicholas Granlund
    %     Louise Olsson
    %
    % 
    
    % Create figure
    figure()


    timeAcc = 0:(1/Hz):((length(acc.X)-1)/Hz);
    timeGyr = 0:(1/Hz):((length(gyr.X)-1)/Hz);

    g = 9.82;

    subplot(2,1,1)
    hold on; grid on
    plot(timeAcc,acc.X./g,'LineWidth',2)
    plot(timeAcc,acc.Y./g,'LineWidth',2)
    plot(timeAcc,acc.Z./g,'LineWidth',2)
    xlabel("Time [s]"); ylabel("Acceleration [m/s^2,    1/g]");
    legend('x-acc','y-acc','z-acc');
    
    subplot(2,1,2)
    hold on; grid on
    plot(timeGyr,gyr.X,'LineWidth',2)
    plot(timeGyr,gyr.Y,'LineWidth',2)
    plot(timeGyr,gyr.Z,'LineWidth',2)
    xlabel("Time [s]"); ylabel("Angular velocity [rad/s]");
    legend('x-gyro', 'y-gyro', 'z-gyro');

end

