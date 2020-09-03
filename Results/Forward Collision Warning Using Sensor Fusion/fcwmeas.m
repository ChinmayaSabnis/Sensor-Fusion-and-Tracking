function measurement = fcwmeas(state, sensorID)
% The example measurements depend on the sensor type, which is reported by
% the MeasurementParameters property of the objectDetection. The following
% two sensorID values are used:
%   sensorID=1: video objects, the measurement is [x;vx;y].
%   sensorID=2: radar objects, the measurement is [x;vx;y;vy].
% The state is:
%   Constant velocity       state = [x;vx;y;vy]
%   Constant turn           state = [x;vx;y;vy;omega]
%   Constant acceleration   state = [x;vx;ax;y;vy;ay]

    if numel(state) < 6 % Constant turn or constant velocity
        switch sensorID
            case 1 % video
                measurement = [state(1:3); 0];
            case 2 % radar
                measurement = state(1:4);
        end
    else % Constant acceleration
        switch sensorID
            case 1 % video
                measurement = [state(1:2); state(4); 0];
            case 2 % radar
                measurement = [state(1:2); state(4:5)];
        end
    end
end