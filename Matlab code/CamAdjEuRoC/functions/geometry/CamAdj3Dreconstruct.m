% =====================================================================

% =====================================================================
function points3D = CamAdj3Dreconstruct(T, inlierPoints0, inlierPoints1, cameraParams0, cameraParams1)
Tm = T';
orient = Tm(1:3,1:3);
loc = Tm(4,1:3);
[Re, te] = cameraPoseToExtrinsics(orient, loc);
camMatrix0 = cameraMatrix(cameraParams0, eye(3), [0 0 0]);
camMatrix1 = cameraMatrix(cameraParams1, Re, te);
points3D = triangulate(inlierPoints0, inlierPoints1, camMatrix0, camMatrix1);
% points3D = points3D';
return