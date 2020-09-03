function jacobian = fcwmeasjac(state, sensorID)
% The example measurements depend on the sensor type, which is reported by
% the MeasurementParameters property of the objectDetection. We choose
% sensorID=1 for video objects and sensorID=2 for radar objects.  The
% following two sensorID values are used:
%   sensorID=1: video objects, the measurement is [x;vx;y].
%   sensorID=2: radar objects, the measurement is [x;vx;y;vy].
% The state is:
%   Constant velocity       state = [x;vx;y;vy]
%   Constant turn           state = [x;vx;y;vy;omega]
%   Constant acceleration   state = [x;vx;ax;y;vy;ay]

    numStates = numel(state);
    jacobian = zeros(4, numStates);

    if numel(state) < 6 % Constant turn or constant velocity
        switch sensorID
            case 1 % video
                jacobian(1,1) = 1;
                jacobian(2,2) = 1;
                jacobian(3,3) = 1;
            case 2 % radar
                jacobian(1,1) = 1;
                jacobian(2,2) = 1;
                jacobian(3,3) = 1;
                jacobian(4,4) = 1;
        end
    else % Constant acceleration
        switch sensorID
            case 1 % video
                jacobian(1,1) = 1;
                jacobian(2,2) = 1;
                jacobian(3,4) = 1;
            case 2 % radar
                jacobian(1,1) = 1;
                jacobian(2,2) = 1;
                jacobian(3,4) = 1;
                jacobian(4,5) = 1;
        end
    end
end