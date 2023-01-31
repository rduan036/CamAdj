function [tracked_points, validIdx] = CamAdjPointTracker(points, lastframe, currentframe, minimumPoint, featurePara)
frame_hist = imhist(lastframe);
currentframe = histeq(currentframe, frame_hist);
tracker = vision.PointTracker('NumPyramidLevels', 5);
initialize(tracker, points, lastframe);
[tracked_points, validIdx] = step(tracker, currentframe);
if sum(validIdx) < minimumPoint
    tracked_points = points;
    validIdx = false(size(points,1),1);
    fast_points1 = cornerPoints(points);
    [features1, validPoints] = extractFeatures(lastframe, fast_points1);
    points = CamAdjDetectFeature(currentframe, featurePara);
    fast_points2 = cornerPoints(points);
    [features2, validPoints] = extractFeatures(currentframe, fast_points2);
    indexPairs = matchFeatures(features1, features2);
    trackes = fast_points2.Location;
    tracked_points(indexPairs(:,1),:) = trackes(indexPairs(:,2),:);
    validIdx(indexPairs(:,1)) = 1;
end
return