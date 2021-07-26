% inpute point_camera = pinv(K) * point_pixcel
function point_vector = pattern_pix2metric( point_camera )
N = size(point_camera,2);
point_vector = zeros(2*N,1);
point_vector(1:2:2*N-1) =  point_camera(1,:)';
point_vector(2:2:2*N) =  point_camera(2,:)';
return