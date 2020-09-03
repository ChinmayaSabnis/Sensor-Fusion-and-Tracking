function filter = initConstantAccelerationFilter(detection)
% This function shows how to configure a constant acceleration filter. The
% input is an objectDetection and the output is a tracking filter.
% For clarity, this function shows how to configure a trackingKF,
% trackingEKF, or trackingUKF for constant acceleration.
%
% Steps for creating a filter:
%   1. Define the motion model and state
%   2. Define the process noise
%   3. Define the measurement model
%   4. Initialize the state vector based on the measurement
%   5. Initialize the state covariance based on the measurement noise
%   6. Create the correct filter

    % Step 1: Define the motion model and state
    % This example uses a constant acceleration model, so:
    STF = @constacc;     % State-transition function, for EKF and UKF
    STFJ = @constaccjac; % State-transition function Jacobian, only for EKF
    % The motion model implies that the state is [x;vx;ax;y;vy;ay]
    % You can also use constvel and constveljac to set up a constant
    % velocity model, constturn and constturnjac to set up a constant turn
    % rate model, or write your own models.

    % Step 2: Define the process noise
    dt = 0.05; % Known timestep size
    sigma = 1; % Magnitude of the unknown acceleration change rate
    % The process noise along one dimension
    Q1d = [dt^4/4, dt^3/2, dt^2/2; dt^3/2, dt^2, dt; dt^2/2, dt, 1] * sigma^2;
    Q = blkdiag(Q1d, Q1d); % 2-D process noise

    % Step 3: Define the measurement model
    MF = @fcwmeas;       % Measurement function, for EKF and UKF
    MJF = @fcwmeasjac;   % Measurement Jacobian function, only for EKF

    % Step 4: Initialize a state vector based on the measurement
    % The sensors measure [x;vx;y;vy] and the constant acceleration model's
    % state is [x;vx;ax;y;vy;ay], so the third and sixth elements of the
    % state vector are initialized to zero.
    state = [detection.Measurement(1); detection.Measurement(2); 0; detection.Measurement(3); detection.Measurement(4); 0];

    % Step 5: Initialize the state covariance based on the measurement
    % noise. The parts of the state that are not directly measured are
    % assigned a large measurement noise value to account for that.
    L = 100; % A large number relative to the measurement noise
    stateCov = blkdiag(detection.MeasurementNoise(1:2,1:2), L, detection.MeasurementNoise(3:4,3:4), L);

    % Step 6: Create the correct filter.
    % Use 'KF' for trackingKF, 'EKF' for trackingEKF, or 'UKF' for trackingUKF
    FilterType = 'EKF';

    % Creating the filter:
    switch FilterType
        case 'EKF'
            filter = trackingEKF(STF, MF, state,...
                'StateCovariance', stateCov, ...
                'MeasurementNoise', detection.MeasurementNoise(1:4,1:4), ...
                'StateTransitionJacobianFcn', STFJ, ...
                'MeasurementJacobianFcn', MJF, ...
                'ProcessNoise', Q ...
                );
        case 'UKF'
            filter = trackingUKF(STF, MF, state, ...
                'StateCovariance', stateCov, ...
                'MeasurementNoise', detection.MeasurementNoise(1:4,1:4), ...
                'Alpha', 1e-1, ...
                'ProcessNoise', Q ...
                );
        case 'KF' % The ConstantAcceleration model is linear and KF can be used
            % Define the measurement model: measurement = H * state
            % In this case:
            %   measurement = [x;vx;y;vy] = H * [x;vx;ax;y;vy;ay]
            % So, H = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0]
            %
            % Note that ProcessNoise is automatically calculated by the
            % ConstantAcceleration motion model
            H = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0];
            filter = trackingKF('MotionModel', '2D Constant Acceleration', ...
                'MeasurementModel', H, 'State', state, ...
                'MeasurementNoise', detection.MeasurementNoise(1:4,1:4), ...
                'StateCovariance', stateCov);
    end
end