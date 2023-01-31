function indix = CamAdjPointsRefine(point3d)
N = size(point3d,2);
mean_points = mean(point3d, 2);
dist_vector = vecnorm(point3d - repmat(mean_points,1,N));
indix = find(dist_vector < 3 * mean(dist_vector));
return