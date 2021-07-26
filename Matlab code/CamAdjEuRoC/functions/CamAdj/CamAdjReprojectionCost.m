% \roi g2
function error_vector = CamAdjReprojectionCost(point2D, point3D, orient, loc, K)
    point2D = point2D';
    point2D(3,:) = 1;
    point2D = K\point2D;
    point2D = point2D./repmat(point2D(3,:),3,1);
    point3D = point3D';
    c = cayley_R2c(orient);
    skew_c = skew(c);
    x0 = [c;(eye(3) - skew_c)*loc'];
    nPoints = size(point2D,2);
    error_vector = zeros(nPoints,1);
    for ii = 1:nPoints
        p1 = point3D(:,ii);
        p2 = point2D(:,ii);
        G1 = compute_G1(p1(1),p1(2),p1(3),p2(1),p2(2));
        G2 = compute_G1(p1(1),p1(2),p1(3),p2(1),p2(2));
        G3 = compute_G3(p1(1),p1(2),p1(3),p2(1),p2(2));
        error_vector(ii,1) = norm([[x0;1]'*G1*[x0;1], [x0;1]'*G2*[x0;1], [x0;1]'*G3*[x0;1]]);
    end
end