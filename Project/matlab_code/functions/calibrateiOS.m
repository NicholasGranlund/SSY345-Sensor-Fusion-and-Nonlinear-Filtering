function [calAcc, calGyr, calMag] = calibrateiOS(m)
    %CALIBRATEIOS function for gathering of calibration 
    % data for iOS sensors.
    %
    %Inputs:
    %   - m                   mobile device object
    %
    %Outputs:
    %   - calAcc        [Kx3] accelerometer measurements
    %   - calGyr        [Kx3] gyroscope measurements
    %   - calMag        [Kx3] magnetometer measurements


    % Assert that all needed sensors are anbled
    if ~m.AccelerationSensorEnabled
        error('Error in calibrateiOS: Acceleration sensor not enabled. Please turn on acceleration sensor and try again.')
    elseif ~m.AngularVelocitySensorEnabled
        error('Error in calibrateiOS: Gyroscope sensor not enabled. Please turn on gyroscope sensor and try again.')
    elseif ~m.MagneticSensorEnabled
        error('Error in calibrateiOS: Magnetometer sensor not enabled. Please turn on magnetometer sensor and try again.')
    elseif ~m.OrientationSensorEnabled
        error('Error in calibrateiOS: Orientation estimate not enabled. Please turn on orientation estimate and try again.')
    end
        
    % Print information to user
    for i = 1:6
        clc
        fprintf('Calibrating.')
        pause(0.3)
        fprintf('.')
        pause(0.3)
        fprintf('.')
        pause(0.3)

    end

    % Print information to user
    clc
    fprintf('Calibrated!')
    pause(1)
          
    % Gather the data obtained by the sensors so far
    calAcc = accellog(m);
    calGyr = angvellog(m);
    calMag = magfieldlog(m);

    clc
    fprintf('Initiating data filtering')
    pause(1.5)

    % return
end

