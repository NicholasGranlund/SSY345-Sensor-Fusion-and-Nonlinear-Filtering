function q = euler2q(roll,pitch,yaw)
    %EULER2Q This function converst euler angles to
    % quaternions.
    %
    %Input:
    %   roll        roll angle
    %   pitch       pitch angle
    %   yaw         yaw angle
    %
    %Output:
    %   q           [4x1] quaternion
    %
    % By: Nicholas Granlund
    %     Louise Olsson

    % Convert degrees to radians
    roll = deg2rad(roll);
    pitch = deg2rad(pitch);
    yaw = deg2rad(yaw);

    % Compute the corresponding orientation as quaternion
    q = [cos(roll/2)*cos(pitch/2)*cos(yaw/2)  +  sin(roll/2)*sin(pitch/2)*sin(yaw/2);
         sin(roll/2)*cos(pitch/2)*cos(yaw/2)  -  cos(roll/2)*sin(pitch/2)*sin(yaw/2);
         cos(roll/2)*sin(pitch/2)*cos(yaw/2)  +  sin(roll/2)*cos(pitch/2)*sin(yaw/2);
         cos(roll/2)*cos(pitch/2)*sin(yaw/2)  -  sin(roll/2)*sin(pitch/2)*cos(yaw/2)];


    % Normalize the quaternion
   [q, ~] = mu_normalizeQ(q, q);

    % return
    
end

