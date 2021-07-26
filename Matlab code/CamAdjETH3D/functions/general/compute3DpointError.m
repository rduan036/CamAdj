function error3D = compute3DpointError(ptCloud_gt, point3D_estimate, max_error)
    N = length(point3D_estimate);
    error3D = 0;
    for ii = 1:N
        point = point3D_estimate(ii,:);
        [indices,dists] = findNearestNeighbors(ptCloud_gt, point, 1);
        error3D = error3D + min(dists,max_error);
    end
    error3D = error3D/N;
return