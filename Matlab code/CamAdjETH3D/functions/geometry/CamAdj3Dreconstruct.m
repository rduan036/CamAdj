function points3D = CamAdj3Dreconstruct(R0, t0, R1, t1, inlierPoints0, inlierPoints1, cameraParams0, cameraParams1)
camMatrix0 = cameraMatrix(cameraParams0, R0, t0);
camMatrix1 = cameraMatrix(cameraParams1, R1, t1);
points3D = triangulate(inlierPoints0, inlierPoints1, camMatrix0, camMatrix1);
return