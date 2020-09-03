 function [detections,laneBoundaries, egoLane] = processDetections...
            (visionFrame, radarFrame, IMUFrame, laneFrame, egoLane, time)
        % Inputs:
        %   visionFrame - objects reported by the vision sensor for this time frame
        %   radarFrame  - objects reported by the radar sensor for this time frame
        %   IMUFrame    - inertial measurement unit data for this time frame
        %   laneFrame   - lane reports for this time frame
        %   egoLane     - the estimated ego lane
        %   time        - the time corresponding to the time frame

        % Remove clutter radar objects
        [laneBoundaries, egoLane] = processLanes(laneFrame, egoLane);
        realRadarObjects = findNonClutterRadarObjects(radarFrame.object,...
            radarFrame.numObjects, IMUFrame.velocity, laneBoundaries);

        % Return an empty list if no objects are reported

        % Counting the total number of objects
        detections = {};
        if (visionFrame.numObjects + numel(realRadarObjects)) == 0
            return;
        end

        % Process the remaining radar objects
        detections = processRadar(detections, realRadarObjects, time);

        % Process video objects
        detections = processVideo(detections, visionFrame, time);
    end