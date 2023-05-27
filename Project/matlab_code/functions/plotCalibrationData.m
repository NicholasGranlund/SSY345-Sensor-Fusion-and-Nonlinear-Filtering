function [structAcc, structGyr, structMag] = plotCalibrationData(calibrationData, Hz)
    %PLOTCALIBRATIONDATA Computes the mean/bias from
    % the callibration data and plots the histograms to
    % illustrate the noise distribution.
    %
    % By: Nicholas Granlund
    %     Louise Olsson
    %
    %Input:
    %   - calibrationData         [3x1]x[Kx3] Struct with fields
    %   - Hz                      [1x1]       Sampling frequency
    %
    %Output:
    %   - structAcc               [3x1]       Struct with fields
    %   - structGyr               [3x1]       Struct with fields
    %   - structMag               [3x1]       Struct with fields

   
    % Set parameters for function
    nbinsAcc = 15;
    nbinsGyr = 15;
    nbinsMag = 15;

    xColor = "#0072BD";
    yColor = "#D95319";
    zColor = "#7E2F8E";

    % Extract the data from log
    accX = calibrationData.Acceleration.X;
    accY = calibrationData.Acceleration.Y;
    accZ = calibrationData.Acceleration.Z;

    gyrX = calibrationData.AngularVelocity.X;
    gyrY = calibrationData.AngularVelocity.Y;
    gyrZ = calibrationData.AngularVelocity.Z;

    magX = calibrationData.MagneticField.X;
    magY = calibrationData.MagneticField.Y;
    magZ = calibrationData.MagneticField.Z;

    % Compute at which timesteps the observations where made
    timeAcc = 0:(1/Hz):(length(accX)-1)*(1/Hz);
    timeGyr = 0:(1/Hz):(length(gyrX)-1)*(1/Hz);
    timeMag = 0:(1/Hz):(length(magX)-1)*(1/Hz);

    


    % Compute mean/bias and covariance from accelerometers
    meanAccX = mean(accX); covAccX = cov(accX);
    meanAccY = mean(accY); covAccY = cov(accY);
    meanAccZ = mean(accZ); covAccZ = cov(accZ);

    % Create struct
    structAcc = struct('meanX',meanAccX,...
                       'meanY',meanAccY,...
                       'meanZ',meanAccZ,...
                       'covX', covAccX,...
                       'covY', covAccY,...
                       'covZ', covAccZ);


    % Plot accelerometer calibration data
    figure()

    % x - data
    subplot(3,2,1)
    grid on; hold on
    histogram(accX,nbinsAcc,'FaceColor',xColor,'Normalization','pdf');
    x = linspace(meanAccX - (5*sqrt(covAccX)), meanAccX + (5*sqrt(covAccX)),1000);
    y = pdf('Normal',x,meanAccX,sqrt(covAccX));
    plot(x,y,'Color','k')
    xlim([x(1), x(end)])
    title('Histograms from Accelerometer')
   
    subplot(3,2,2)
    grid on; hold on
    plot(timeAcc,accX,'color',xColor)
    xlim([timeAcc(1), timeAcc(end)])
    ylim([meanAccX - 5*sqrt(covAccX), meanAccX + 5*sqrt(covAccX)])
    yline(meanAccX);
    title('Accelerometer calibration data')


    % y - data
    subplot(3,2,3)
    grid on; hold on
    histogram(accY,nbinsAcc,'FaceColor',yColor,'Normalization','pdf')
    x = linspace(meanAccY - (5*sqrt(covAccY)), meanAccY + (5*sqrt(covAccY)),1000);
    y = pdf('Normal',x,meanAccY,sqrt(covAccY));
    plot(x,y,'Color','k')
    xlim([x(1), x(end)])
    ylabel('Number of occurances')

    subplot(3,2,4)
    grid on; hold on
    plot(timeAcc,accY,'color',yColor)
    xlim([timeAcc(1), timeAcc(end)]);
    ylim([meanAccY - 5*sqrt(covAccY), meanAccY + 5*sqrt(covAccY)])
    yline(meanAccY);
    ylabel('Value [m/s^2]')


    % z - data
    subplot(3,2,5)
    grid on; hold on
    histogram(accZ,nbinsAcc,'FaceColor',zColor,'Normalization','pdf');
    x = linspace(meanAccZ - (5*sqrt(covAccZ)), meanAccZ + (5*sqrt(covAccZ)),1000);
    y = pdf('Normal',x,meanAccZ,sqrt(covAccZ));
    plot(x,y,'Color','k')
    xlim([x(1), x(end)])
    xlabel('Value [m/s^2]')

    subplot(3,2,6)
    grid on; hold on
    plot(timeAcc,accZ,'color',zColor)
    xlim([timeAcc(1), timeAcc(end)])
    ylim([meanAccZ - 5*sqrt(covAccZ), meanAccZ + 5*sqrt(covAccZ)])
    yline(meanAccZ);
    xlabel('Time [s]')




    % Compute mean/bias and covariance from gyroscope
    meanGyrX = mean(gyrX); covGyrX = cov(gyrX);
    meanGyrY = mean(gyrY); covGyrY = cov(gyrY);
    meanGyrZ = mean(gyrZ); covGyrZ = cov(gyrZ);

    % Create struct
    structGyr = struct('meanX',meanGyrX,...
                       'meanY',meanGyrY,...
                       'meanZ',meanGyrZ,...
                       'covX', covGyrX,...
                       'covY', covGyrY,...
                       'covZ', covGyrZ);


    % Plot gyroscope calibration data
    figure()

    % x - data
    subplot(3,2,1)
    grid on; hold on
    histogram(gyrX,nbinsGyr,'FaceColor',xColor,'Normalization','pdf');
    x = linspace(meanGyrX - (5*sqrt(covGyrX)), meanGyrX + (5*sqrt(covGyrX)),1000);
    y = pdf('Normal',x,meanGyrX,sqrt(covGyrX));
    plot(x,y,'Color','k')
    xlim([x(1), x(end)])
    title('Histograms from Gyroscope')

    subplot(3,2,2)
    grid on; hold on
    plot(timeGyr,gyrX,'color',xColor)
    xlim([timeGyr(1), timeGyr(end)])
    ylim([meanGyrX - 5*sqrt(covGyrX), meanGyrX + 5*sqrt(covGyrX)])
    yline(meanGyrX);
    title('Gyroscope calibration data')


    % y - data
    subplot(3,2,3)
    grid on; hold on
    histogram(gyrY,nbinsGyr,'FaceColor',yColor,'Normalization','pdf');
    x = linspace(meanGyrY - (5*sqrt(covGyrY)), meanGyrY + (5*sqrt(covGyrY)),1000);
    y = pdf('Normal',x,meanGyrY,sqrt(covGyrY));
    plot(x,y,'Color','k')
    xlim([x(1), x(end)])
    ylabel('Number of occurances')

    subplot(3,2,4)
    grid on; hold on
    plot(timeGyr,gyrY,'color',yColor)
    xlim([timeGyr(1), timeGyr(end)])
    ylim([meanGyrY - 5*sqrt(covGyrY), meanGyrY + 5*sqrt(covGyrY)])
    yline(meanGyrY);
    ylabel('Value [rad/s]')


    % z - data
    subplot(3,2,5)
    grid on; hold on
    histogram(gyrZ,nbinsGyr,'FaceColor',zColor,'Normalization','pdf');
    x = linspace(meanGyrZ - (5*sqrt(covGyrZ)), meanGyrZ + (5*sqrt(covGyrZ)),1000);
    y = pdf('Normal',x,meanGyrZ,sqrt(covGyrZ));
    plot(x,y,'Color','k')
    xlim([x(1), x(end)])
    xlabel('Value [rad/s]')

    subplot(3,2,6)
    grid on; hold on
    plot(timeGyr,gyrZ,'color',zColor)
    xlim([timeGyr(1), timeGyr(end)])
    ylim([meanGyrZ - 5*sqrt(covGyrZ), meanGyrZ + 5*sqrt(covGyrZ)])
    yline(meanGyrZ);
    xlabel('Time [s]')




    % Compute mean/bias and covariance from magnetometer
    meanMagX = mean(magX); covMagX = cov(magX);
    meanMagY = mean(magY); covMagY = cov(magY);
    meanMagZ = mean(magZ); covMagZ = cov(magZ);

    % Create struct
    structMag = struct('meanX',meanMagX,...
                       'meanY',meanMagY,...
                       'meanZ',meanMagZ,...
                       'covX', covMagX,...
                       'covY', covMagY,...
                       'covZ', covMagZ);

    % Plot magnetometer calibration data
    figure()

    % x - data
    subplot(3,2,1)
    grid on; hold on
    histogram(magX,nbinsMag,'FaceColor',xColor,'Normalization','pdf');
    x = linspace(meanMagX - (5*sqrt(covMagX)), meanMagX + (5*sqrt(covMagX)),1000);
    y = pdf('Normal',x,meanMagX,sqrt(covMagX));
    plot(x,y,'Color','k')
    xlim([x(1), x(end)])
    title('Histograms from Magnetometer')

    subplot(3,2,2)
    grid on; hold on
    plot(timeMag,magX,'color',xColor)
    xlim([timeMag(1), timeMag(end)]);
    ylim([meanMagX - 5*sqrt(covMagX), meanMagX + 5*sqrt(covMagX)])
    yline(meanMagX);
    title('Magnetometer calibration data')


    % y - data
    subplot(3,2,3)
    grid on; hold on
    histogram(magY,nbinsMag,'FaceColor',yColor,'Normalization','pdf');
    x = linspace(meanMagY - (5*sqrt(covMagY)), meanMagY + (5*sqrt(covMagY)),1000);
    y = pdf('Normal',x,meanMagY,sqrt(covMagY));
    plot(x,y,'Color','k')
    xlim([x(1), x(end)])
    ylabel('Number of occurances')

    subplot(3,2,4)
    grid on; hold on
    plot(timeMag,magY,'color',yColor)
    xlim([timeMag(1), timeMag(end)])
    ylim([meanMagY - 5*sqrt(covMagY), meanMagY + 5*sqrt(covMagY)])
    yline(meanMagY);
    ylabel('Value [\muT]')
    


    % z - data
    subplot(3,2,5)
    grid on; hold on
    histogram(magZ,nbinsMag,'FaceColor',zColor,'Normalization','pdf');
    x = linspace(meanMagZ - (5*sqrt(covMagZ)), meanMagZ + (5*sqrt(covMagZ)),1000);
    y = pdf('Normal',x,meanMagZ,sqrt(covMagZ));
    plot(x,y,'Color','k')
    xlim([x(1), x(end)])
    xlabel('Value [\muT]')

    subplot(3,2,6)
    grid on; hold on
    plot(timeMag,magZ,'color',zColor)
    xlim([timeMag(1), timeMag(end)])
    ylim([meanMagZ - 5*sqrt(covMagZ), meanMagZ + 5*sqrt(covMagZ)])
    yline(meanMagZ);
    xlabel('Time [s]')


end

