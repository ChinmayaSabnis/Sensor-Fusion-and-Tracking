   function mostImportantObject = findMostImportantObject(confirmedTracks,egoLane,positionSelector,velocitySelector)

        % Initialize outputs and parameters
        MIO = [];               % By default, there is no MIO
        trackID = [];           % By default, there is no trackID associated with an MIO
        FCW = 3;                % By default, if there is no MIO, then FCW is 'safe'
        threatColor = 'green';  % By default, the threat color is green
        maxX = 1000;  % Far enough forward so that no track is expected to exceed this distance
        gAccel = 9.8; % Constant gravity acceleration, in m/s^2
        maxDeceleration = 0.4 * gAccel; % Euro NCAP AEB definition
        delayTime = 1.2; % Delay time for a driver before starting to brake, in seconds

        positions = getTrackPositions(confirmedTracks, positionSelector);
        velocities = getTrackVelocities(confirmedTracks, velocitySelector);

        for i = 1:numel(confirmedTracks)
            x = positions(i,1);
            y = positions(i,2);

            relSpeed = velocities(i,1); % The relative speed between the cars, along the lane

            if x < maxX && x > 0 % No point checking otherwise
                yleftLane  = polyval(egoLane.left,  x);
                yrightLane = polyval(egoLane.right, x);
                if (yrightLane <= y) && (y <= yleftLane)
                    maxX = x;
                    trackID = i;
                    MIO = confirmedTracks(i).TrackID;
                    if relSpeed < 0 % Relative speed indicates object is getting closer
                        % Calculate expected braking distance according to
                        % Euro NCAP AEB Test Protocol
                        d = abs(relSpeed) * delayTime + relSpeed^2 / 2 / maxDeceleration;
                        if x <= d % 'warn'
                            FCW = 1;
                            threatColor = 'red';
                        else % 'caution'
                            FCW = 2;
                            threatColor = 'yellow';
                        end
                    end
                end
            end
        end
        mostImportantObject = struct('ObjectID', MIO, 'TrackIndex', trackID, 'Warning', FCW, 'ThreatColor', threatColor);
    end