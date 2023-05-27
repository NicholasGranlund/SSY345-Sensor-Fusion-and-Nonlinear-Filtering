function [t, acc, gyr, mag, dataOrient] = getiOSData(m)
    %GETIOSDATA This functions uses the connected iOS device
    % and gathers data from its sensors at the time instance when the function is
    % called.
    %
    %Input:
    %   m                   Mobile device object
    %
    %Output:
    %   t                   Timestamp
    %   acc         [3x1]   Acceleration measurements
    %   gyr         [3x1]   Gyroscope measurements
    %   mag         [3x1]   Magnetmoeter measurements
    %   dataOrient  [3x1]   Phone estimated orientation
    %
    % By: Nicholas Granlund
    %     Louise Olsson
    
    % Extract the data from the sensors
    acc= m.Acceleration';
    gyr = m.AngularVelocity';
    mag = m.MagneticField';
    [dataOrient, timestamp] = orientlog(m);
    dataOrient = dataOrient(end,:);
    
    % Determine how much time has passed since last measurement
    t = timestamp(end);

    % return

end

