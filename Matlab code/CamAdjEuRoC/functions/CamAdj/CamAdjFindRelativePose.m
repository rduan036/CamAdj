function [orient, loc, epipolarInliers] = ...
    CamAdjFindRelativePose(matchedPoints1, matchedPoints2, cameraParams0, cameraParams1)

% Estimate the fundamental matrix
[E, epipolarInliers] = estimateEssentialMatrix(...
    matchedPoints1, matchedPoints2, cameraParams0, cameraParams1, 'Confidence', 90);

% Find epipolar inliers
inlierPoints1 = matchedPoints1(epipolarInliers, :);
inlierPoints2 = matchedPoints2(epipolarInliers, :);

[orient, loc] = relativeCameraPose(E, cameraParams0, cameraParams1, inlierPoints1, inlierPoints2);
return