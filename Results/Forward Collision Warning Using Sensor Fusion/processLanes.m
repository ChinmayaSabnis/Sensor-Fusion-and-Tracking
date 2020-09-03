function [laneBoundaries, egoLane] = processLanes(laneReports, egoLane)
% Lane boundaries are updated based on the laneReports from the recordings.
% Since some laneReports contain invalid (isValid = false) reports or
% impossible parameter values (-1e9), these lane reports are ignored and
% the previous lane boundary is used.
leftLane    = laneReports.left;
rightLane   = laneReports.right;

% Check the validity of the reported left lane
cond = (leftLane.isValid && leftLane.confidence) && ...
    ~(leftLane.headingAngle == -1e9 || leftLane.curvature == -1e9);
if cond
    egoLane.left = cast([leftLane.curvature, leftLane.headingAngle, leftLane.offset], 'double');
end

% Update the left lane boundary parameters or use the previous ones
leftParams  = egoLane.left;
leftBoundaries = parabolicLaneBoundary(leftParams);
leftBoundaries.Strength = 1;

% Check the validity of the reported right lane
cond = (rightLane.isValid && rightLane.confidence) && ...
    ~(rightLane.headingAngle == -1e9 || rightLane.curvature == -1e9);
if cond
    egoLane.right  = cast([rightLane.curvature, rightLane.headingAngle, rightLane.offset], 'double');
end

% Update the right lane boundary parameters or use the previous ones
rightParams = egoLane.right;
rightBoundaries = parabolicLaneBoundary(rightParams);
rightBoundaries.Strength = 1;

laneBoundaries = [leftBoundaries, rightBoundaries];
end