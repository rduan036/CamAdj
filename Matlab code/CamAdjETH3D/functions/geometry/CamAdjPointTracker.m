function [tracked_points, validIdx] = CamAdjPointTracker(points, lastframe, currentframe)
    frame_hist = imhist(lastframe);
    currentframe = histeq(currentframe, frame_hist);
    tracker = vision.PointTracker('NumPyramidLevels', 5);
    initialize(tracker, points, lastframe);
    [tracked_points, validIdx] = step(tracker, currentframe);
return