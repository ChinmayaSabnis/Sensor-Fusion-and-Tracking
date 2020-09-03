function postProcessedDetections = processRadar(postProcessedDetections, realRadarObjects, t)
% Process the radar objects into objectDetection objects
numRadarObjects = numel(realRadarObjects);
if numRadarObjects
    classToUse = class(realRadarObjects(1).position);
    radarMeasCov = cast(diag([2,2,2,100]), classToUse);
    % Process Radar Objects:
    for i=1:numRadarObjects
        object = realRadarObjects(i);
        postProcessedDetections{i} = objectDetection(t, ...
            [object.position(1); object.velocity(1); object.position(2); object.velocity(2)], ...
            'SensorIndex', 2, 'MeasurementNoise', radarMeasCov, ...
            'MeasurementParameters', {2}, ...
            'ObjectAttributes', {object.id, object.status, object.amplitude, object.rangeMode});
    end
end
end