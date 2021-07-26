function [mp0, mp1, point3D0, point3D1] = CamAdjStereoView3dReconstruction(...
    img0, img1, cameraParams0, cameraParams1, T_c0c1, T_c1c0,...
    featurePara, minimumPoint)
% stereo vision
Points0 = CamAdjDetectFeature(img0, featurePara);
[Points0_tracked, validIdx0] = CamAdjPointTracker(Points0, img0, img1, minimumPoint, featurePara);
validIdx0 = CamAdjRemoveLargeShift(Points0, Points0_tracked, validIdx0);
mp0 = Points0(validIdx0,:);
mp0_tracked = Points0_tracked(validIdx0,:);
points3D_cam0 = CamAdj3Dreconstruct(T_c0c1, mp0, mp0_tracked, cameraParams0, cameraParams1);
points3D_cam0 = points3D_cam0';
% filter bad 3D points
outliers_negdepth = find(points3D_cam0(3,:) < 0);
outliers_farpoint = CamAdjFindFarPoints(points3D_cam0);
outliers = [outliers_negdepth'; outliers_farpoint];
points3D_cam0(:,outliers) = [];
mp0(outliers,:) = [];
mp0_tracked(outliers,:) = [];
mp1 = mp0_tracked;
R = T_c1c0(1:3,1:3);
t = T_c1c0(1:3,4);
K1 = cameraParams1.IntrinsicMatrix';
error_vector = CamAdjReprojectionCost(mp1, points3D_cam0', R', t', K1);
CamAdj_check_idx = find(error_vector > 0.15);
if ~isempty(CamAdj_check_idx)
    rejection_idx = CamAdjFiltering(mp1(CamAdj_check_idx,:)', ...
        points3D_cam0(:,CamAdj_check_idx), R, t, K1, [0.00001, 0.05]);
    if ~isempty(rejection_idx)
        mp0(CamAdj_check_idx(rejection_idx),:) = [];
        mp1(CamAdj_check_idx(rejection_idx),:) = [];
        points3D_cam0(:,CamAdj_check_idx(rejection_idx)) = [];
        disp(['CamAdj rejected ', num2str(length(rejection_idx)), ' points.']);
    end
end

points3D_cam1 = R * points3D_cam0 + repmat(t,1,length(points3D_cam0));
point3D0 = points3D_cam0';
point3D1 = points3D_cam1';

return
