function finalRad= ControlProgram1(serPort)
% Simple program for autonomously control the iRobot Create on either the
% physical Create or the simulated version. This will simply spiral outward
% and turn away from obstacles that detects with the bump sensors.
%
% For the physical Create only, it is assumed that the function call
% serPort= RoombaInit(comPort) was done prior to running this program.
% Calling RoombaInit is unnecessary if using the simulator.
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% finalRad - Double, final turning radius of the Create (m)


%distance sensor rumba - check total distance from Start
%check edge cases

    % Set constants for this program
    maxDuration= 1200;  % Max time to allow the program to run (s)
    maxDistSansBump= 5; % Max distance to travel without obstacles (m)
    maxFwdVel= 0.5;     % Max allowable forward velocity with no angular 
                        % velocity at the time (m/s)
    maxVelIncr= 0.005;  % Max incrementation of forward velocity (m/s)
    maxOdomAng= pi/4;   % Max angle to move around a circle before 
                        % increasing the turning radius (rad)
    
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    distSansBump= 0;    % Distance traveled without hitting obstacles (m)
    angTurned= 0;       % Angle turned since turning radius increase (rad)
    %v= 0;               % Forward velocity (m/s)
    %w= v2w(v);          % Angular velocity (rad/s )
    v = .5;
    w = 0;

    % Start robot moving
    SetFwdVelAngVelCreate(serPort,v,w);
    initialHit = 1;  
    forward = 0;
    turnAngle = 0;
    while (toc(tStart) < maxDuration)
        bumped = bumpChecks(serPort);
        frontBump = frontBumpCheck(serPort);

        display(bumped);
        %display(frontBump);
        %display(turnAngle);
        %waiting for first hit
        if initialHit
            initialHit = firstBump(serPort);
        end
        
        if ~initialHit 
            if ~forward
                %bump check ~ reset
                forward = bumpCheck(serPort)
                turnAngle = 0;
            end

            if forward
                if bumped && ~frontBump
                    SetFwdVelAngVelCreate(serPort,v,w);
                    turnAngle = 0;
                elseif bumped && frontBump
                    bumpCheck(serPort);
                    turnAngle = 0;
                else ~bumped
                    SetFwdVelAngVelCreate(serPort, 0, -pi/8);
                    turnAngle = turnAngle + pi/8;
                    %if(turnAngle >= pi/2)
                    %    SetFwdVelAngVelCreate(serPort, 1, -pi/8);
                    %end
                    %check for 180 deg turn, move over + down
                    %forward = 0;
                    % Wait for turn to complete
                    %angTurned= 0;
                    %ang = pi/4;
                    %AngleSensorRoomba(serPort);
                    %while angTurned < ang
                    %    angTurned= angTurned+abs(AngleSensorRoomba(serPort));
                    %    pause(0.1)
                end
                    %SetFwdVelAngVelCreate(serPort,v,w);
                %end
            end


        end

        
        %no hit
        %if bumped
        %    SetFwdVelAngVelCreate(serPort,v,w);
        %    initialHit = 0;
        %end
       
        %forward hit
        %if bumped == 2
        %    Reset(serPort);
        %end

        %if bumped == 0 && ~initialBump
        %    turnAngle(serPort, .5, -pi/8);
        %    SetFwdVelAngVelCreate(serPort,v,w);
        %    initialBump = 0;
        %end
        %else ~bumped && ~initialBump
        %    SetFwdVelAngVelCreate(serPort,v,w);
        %end
        
        %force right bump
        %if a
        %    turnAngle(serPort, .5, -pi/8);
        %end
        %rightbumped = rightCheck(serPort);
        %while rightbumped
        %    SetFwdVelAngVelCreate(serPort,v,w);
        %    rightbumped = rightCheck(serPort);
        %end
        
        %while leftbump
        %while not bumped
        %turnAngle(serPort, .5, -pi/8);
        %end
        %rightbump = forwardCheck(serPort);
        %while rightbump
        %    SetFwdAngVelCreate(serPort, 0, 0);
        %end
    pause(0.1);
    end
    
    % turnAngle(serPort, .5, -pi/8);
    %
    %leftbump = forwardCheck(serPort)
    %while(leftbump = forwardCheck(serPort))
    %    SetFwdAngVelCreate(serPort, 0, 0);

    
    % Specify output parameter
    finalRad= v/w;
    
    % Stop robot motion
    v= 0;
    w= 0;
    SetFwdVelAngVelCreate(serPort,v,w)
    
    % If you call RoombaInit inside the control program, this would be a
    % good place to clean up the serial port with...
    % fclose(serPort)
    % delete(serPort)
    % clear(serPort)
    % Don't use these if you call RoombaInit prior to the control program
end

function initialHit = firstBump(serPort)
% Check bump sensors and steer the robot away from obstacles if necessary
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% leftbump - Boolean, true if bump sensor is activated

    % Check bump sensors (ignore wheel drop sensors)
    [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
        BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    if BumpFront || BumpRight || BumpLeft
        initialHit = 0;
    else
        SetFwdVelAngVelCreate(serPort,.5,0);
        initialHit = 1;
    end
end

function bumped = bumpChecks(serPort)
% Check bump sensors and steer the robot away from obstacles if necessary
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% bumped - Boolean, true if bump sensor is activated

    % Check bump sensors (ignore wheel drop sensors)
    [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
        BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    bumped= BumpRight || BumpLeft || BumpFront;

end

function frontBump = frontBumpCheck(serPort)
% Check bump sensors and steer the robot away from obstacles if necessary
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% bumped - Boolean, true if bump sensor is activated

    % Check bump sensors (ignore wheel drop sensors)
    [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
        BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    frontBump= BumpFront;

end

function forward = bumpCheck(serPort, initialHit)
% Check bump sensors and steer the robot away from obstacles if necessary
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% leftbump - Boolean, true if bump sensor is activated

    % Check bump sensors (ignore wheel drop sensors)
    [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
        BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    if BumpFront
        turnAngle(serPort, .2, pi/5);
        forward = 0;
    elseif BumpRight
        turnAngle(serPort, .2, pi/5);
        forward = 0;
    else
        turnAngle(serPort, .2, -pi/5);
        forward = 1;
    end

end

function w= v2w(v)
% Calculate the maximum allowable angular velocity from the linear velocity
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)
    
    % Robot constants
    maxWheelVel= 0.5;   % Max linear velocity of each drive wheel (m/s)
    robotRadius= 0.2;   % Radius of the robot (m)
    
    % Max velocity combinations obey rule v+wr <= v_max
    w= (maxWheelVel-v)/robotRadius;
end

function Reset(serPort)
% Calculate the maximum allowable angular velocity from the linear velocity
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)
    bumped = bumpCheck(serPort);
    while bumped
        turnAngle(serPort, .2, pi/8);
        bumped = bumpCheck(serPort);
    end
end

function findWall(serPort)
% Calculate the maximum allowable angular velocity from the linear velocity
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)
    bumped = bumpCheck(serPort);
    while bumped == 0
        SetFwdVelAngVelCreate(serPort,0,0); 
        bumped = bumpCheck(serPort); 
    end  
end

function moveForward(serPort)
% Calculate the maximum allowable angular velocity from the linear velocity
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)
    turnAngle(serPort, .2, -pi/8);
    bumped = bumpCheck(serPort);
    while bumped == 1
     SetFwdVelAngVelCreate(serPort,0,0);       
     bumped = bumpCheck(serPort);
    end
end


