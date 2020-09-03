function [Vx,Vy] = calculateGroundSpeed(Vxi,Vyi,egoSpeed)
% Inputs
%   (Vxi,Vyi) : relative object speed
%   egoSpeed  : ego vehicle speed
% Outputs
%   [Vx,Vy]   : ground object speed

    Vx = Vxi + egoSpeed;    % Calculate longitudinal ground speed
    theta = atan2(Vyi,Vxi); % Calculate heading angle
    Vy = Vx * tan(theta);   % Calculate lateral ground speed

end