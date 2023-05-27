function m = connectiOS(devName)
    %CONNECTIOS Function that connects the matlab client with iOS
    % Connect to mobile device and creat object m
    %
    %Input:
    %   - devName     string of device name to connect to
    %
    %Output:
    %   - m           mobile device object
    %
    % By: Nicholas Granlund
    %     Louise Olsson

    % Print information to user
    fprintf('Connecting to device...\n\n')

    % Assert connection with device
    m = mobiledev(devName);

    % Clear command window and print information to user
    clc
    fprintf(['Connection established!\n\n' ...
             'Connected to:  %s\n'...
             'Start streaming data from smartphone \n'],devName)

    fprintf(['\nMake sure that the following sensors are active:\n' ...
             '1.    Acceleration\n' ...
             '2.    Magnetic Field\n' ...
             '3.    Orientation\n' ...
             '4.    Angular Velocity\n'])
    fprintf('\nStart filtering by pressing ENTER\n')
   
    % pause before continuing
    pause;
    
end

