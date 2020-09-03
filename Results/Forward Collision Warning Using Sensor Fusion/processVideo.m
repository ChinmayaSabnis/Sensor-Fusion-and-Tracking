function postProcessedDetections = processVideo(postProcessedDetections, visionFrame, t)
% Process the video objects into objectDetection objects
numRadarObjects = numel(postProcessedDetections);
numVisionObjects = visionFrame.numObjects;
if numVisionObjects
    classToUse = class(visionFrame.object(1).position);
    visionMeasCov = cast(diag([2,2,2,100]), classToUse);
    % Process Vision Objects:
    for i=1:numVisionObjects
        object = visionFrame.object(i);
        postProcessedDetections{numRadarObjects+i} = objectDetection(t,...
            [object.position(1); object.velocity(1); object.position(2); 0], ...
            'SensorIndex', 1, 'MeasurementNoise', visionMeasCov, ...
            'MeasurementParameters', {1}, ...
            'ObjectClassID', object.classification, ...
            'ObjectAttributes', {object.id, object.size});
    end
end
end