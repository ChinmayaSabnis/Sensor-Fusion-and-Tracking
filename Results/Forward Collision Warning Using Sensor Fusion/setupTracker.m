 function [tracker, positionSelector, velocitySelector] = setupTracker()
        tracker = multiObjectTracker(...
            'FilterInitializationFcn', @initConstantAccelerationFilter, ...
            'AssignmentThreshold', 35, 'ConfirmationThreshold', [2 3], ...
            'DeletionThreshold', 5);

        % The State vector is:
        %   In constant velocity:     State = [x;vx;y;vy]
        %   In constant acceleration: State = [x;vx;ax;y;vy;ay]

        % Define which part of the State is the position. For example:
        %   In constant velocity:     [x;y] = [1 0 0 0; 0 0 1 0] * State
        %   In constant acceleration: [x;y] = [1 0 0 0 0 0; 0 0 0 1 0 0] * State
        positionSelector = [1 0 0 0 0 0; 0 0 0 1 0 0];

        % Define which part of the State is the velocity. For example:
        %   In constant velocity:     [x;y] = [0 1 0 0; 0 0 0 1] * State
        %   In constant acceleration: [x;y] = [0 1 0 0 0 0; 0 0 0 0 1 0] * State
        velocitySelector = [0 1 0 0 0 0; 0 0 0 0 1 0];
    end