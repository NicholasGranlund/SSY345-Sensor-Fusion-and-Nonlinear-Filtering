function [xhat, meas] = filterTemplateiOS(m, calAcc, calGyr, calMag)
% FILTERTEMPLATEIOS  Filter template adapted for iOS capability
%
% This is a template function for how to collect and filter data
% sent from a smartphone live.  Calibration data for the
% accelerometer, gyroscope and magnetometer assumed available as
% structs with fields m (mean) and R (variance).
%
% The function returns xhat as an array of structs comprising t
% (timestamp), x (state), and P (state covariance) for each
% timestamp, and meas an array of structs comprising t (timestamp),
% acc (accelerometer measurements), gyr (gyroscope measurements),
% mag (magnetometer measurements), and orint (orientation quaternions
% from the phone).  Measurements not availabe are marked with NaNs.
%
% As you implement your own orientation estimate, it will be
% visualized in a simple illustration.  If the orientation estimate
% is checked in the Sensor Fusion app, it will be displayed in a
% separate view.
%
% Note that it is not necessary to provide inputs (calAcc, calGyr, calMag).

%
%Inputs:
%   - m                 Mobile device object 
%   - calAcc    [Kx3]   Accelerometer measurements used for calibration
%   - calGyr    [Kx3]   Gyroscope measurements used for calibration
%   - calMag    [Kx3]   Magnetometer measurements used for calibration
% 
%Outputs:
%   - xhat              state estimate struct
%   - meas              sensor measurements struct
%
%
% Edited by:    Nicholas Granlund
%               Louise Olsson


  %% Calibration

  % If calibration data is not provided - calibrate
  if nargin < 2
      % Do someting
    
  end


  %% Filter settings
  t0 = [];  % Initial time (initialize on first data received)
  nx = 4;   % Assuming that you use q as state variable.
  % Add your filter settings here.


  % Current filter state.
  x = [1; 0; 0 ;0];
  P = eye(nx, nx);

  % Saved filter states.
  xhat = struct('t', zeros(1, 0),...
                'x', zeros(nx, 0),...
                'P', zeros(nx, nx, 0));

  meas = struct('t', zeros(1, 0),...
                'acc', zeros(3, 0),...
                'gyr', zeros(3, 0),...
                'mag', zeros(3, 0),...
                'orient', zeros(4, 0));


     % Used for visualization.
     figure(1);
     subplot(1, 2, 1);
     ownView = OrientationView('Own filter', gca);  % Used for visualization.
     googleView = [];
     counter = 0;  % Used to throttle the displayed frame rate.




    %% Filter loop

    % Repeat while mobile device 'm' is connected and while at least one
    % sensor is transmitting data
    while m.Connected && m.Logging 
      % Get the next measurement set, assume all measurements
      % within the next 5 ms are concurrent (suitable for sampling
      % in 100Hz).


      % ------------------ SMARTPHONE SENSORS --------------------

      % Gather information from the sensors
      [t, acc, gyr, mag, dataOrient] = getiOSData(m);

      % ----------------------------------------------------------



      if isempty(t0)  % Initialize t0
        t0 = t;
      end



      % --------------------- ACCELEROMETER ------------------------
 
      % Acc measurements are available / sensor is enabled.
      if m.AccelerationSensorEnabled
           % Do something
      end
      % ------------------------------------------------------------
    





      % ----------------------- GYROSCOPE --------------------------

      % Gyro measurements are available / sensor is enabled.
      if m.AngularVelocitySensorEnabled
            % Do something
      end
      % ------------------------------------------------------------




      

     % ---------------------- MAGNETOMETER ------------------------
     
     % Mag measurements are available / sensor is enabled.
     if  m.MagneticSensorEnabled
        % Do something
      end
      % ------------------------------------------------------------





      % --------------------- VISUALIZE RESULT ---------------------
     
      % Google's/Matlabs orientation estimate.
      orientation = eul2quat([-dataOrient(1), dataOrient(3), -dataOrient(2)].*(pi/180),"ZYX");
     
      % Visualize result
      if rem(counter, 10) == 0
        setOrientation(ownView, x(1:4));
        title(ownView, 'OWN', 'FontSize', 16);
        if ~any(isnan(orientation))
          if isempty(googleView)
            subplot(1, 2, 2);
            % Used for visualization.
            googleView = OrientationView('Google filter', gca);
          end
          setOrientation(googleView, orientation);
          title(googleView, 'GOOGLE', 'FontSize', 16);
        end
      end
      % Increment the counter
      counter = counter + 1;
      % ------------------------------------------------------------



    
      % ------------------------ SAVE DATA -------------------------
      % Save estimates
      xhat.x(:, end+1) = x;
      xhat.P(:, :, end+1) = P;
      xhat.t(end+1) = t - t0;

      meas.t(end+1) = t - t0;
      meas.acc(:, end+1) = acc;
      meas.gyr(:, end+1) = gyr;
      meas.mag(:, end+1) = mag;
      meas.orient(:, end+1) = orientation;

      % -----------------------------------------------------------


    end


end

