function realRadarObjects = findNonClutterRadarObjects(radarObject, numRadarObjects, egoSpeed, laneBoundaries)
% The radar objects include many objects that belong to the clutter.
% Clutter is defined as a stationary object that is not in front of the
% car. The following types of objects pass as nonclutter:
%
% # Any object in front of the car
% # Any moving object in the area of interest around the car, including
%   objects that move at a lateral speed around the car

    % Allocate memory
    normVs = zeros(numRadarObjects, 1);
    inLane = zeros(numRadarObjects, 1);
    inZone = zeros(numRadarObjects, 1);

    % Parameters
    LaneWidth = 3.6;            % What is considered in front of the car
    ZoneWidth = 1.7*LaneWidth;  % A wider area of interest
    minV = 1;                   % Any object that moves slower than minV is considered stationary
    for j = 1:numRadarObjects
        [vx, vy] = calculateGroundSpeed(radarObject(j).velocity(1),radarObject(j).velocity(2),egoSpeed);
        normVs(j) = norm([vx,vy]);
        laneBoundariesAtObject = computeBoundaryModel(laneBoundaries, radarObject(j).position(1));
        laneCenter = mean(laneBoundariesAtObject);
        inLane(j) = (abs(radarObject(j).position(2) - laneCenter) <= LaneWidth/2);
        inZone(j) = (abs(radarObject(j).position(2) - laneCenter) <= max(abs(vy)*2, ZoneWidth));
    end
    realRadarObjectsIdx = union(...
        intersect(find(normVs > minV), find(inZone == 1)), ...
        find(inLane == 1));

    realRadarObjects = radarObject(realRadarObjectsIdx);
end