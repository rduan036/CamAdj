function outliers = CamAdjFindFarPoints(points3D)

N = size(points3D,2);
cost = zeros(N,1);
points3D = points3D';
ptCloud = pointCloud(points3D);
K = 5;
for ii = 1:N
    point = points3D(ii,:);
    [indices,dists] = findNearestNeighbors(ptCloud,point,K);
    mean_dist = max(dists);
    cost(ii) = mean_dist;
end
median_cost = median(cost);
outliers = find(cost > 2*median_cost);

return