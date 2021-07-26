function CamAdjVO = CamAdj_VO_init(img0, img1, feature_para, system_para,... 
    cameraParams0, cameraParams1, T_BS_cam0, T_BS_cam1, UAV_pose)
% initial camera and UAV pose 
CamAdjVO.cameraParams0 = cameraParams0;
CamAdjVO.cameraParams1 = cameraParams1;
CamAdjVO.pose = UAV_pose;
CamAdjVO.lastFrame0 = img0;
CamAdjVO.lastFrame1 = img1;
% CamAdjVO.lastFrame0 = imsharpen(img0);
% CamAdjVO.lastFrame1 = imsharpen(img1);
% T_cam = pinv(T_uav2cam)
% T_cam * T_BS_cam = I and pinv(R) = R', so T_cam0 = [R', - R' * t; 0 0 0 1]
CamAdjVO.T_body2cam0 = T_BS_cam0;
CamAdjVO.T_body2cam1 = T_BS_cam1;
CamAdjVO.T_cam2body0 = CamAdjTransformInverse(T_BS_cam0);
CamAdjVO.T_cam2body1 = CamAdjTransformInverse(T_BS_cam1);
% transform from cam0 to cam1,  T_BS_cam0 * T_BS_c0c1 = T_BS_cam1
CamAdjVO.T_c0c1 = CamAdjTransformInverse(T_BS_cam0) * T_BS_cam1;
% transform from cam1 to cam0
CamAdjVO.T_c1c0 = CamAdjTransformInverse(CamAdjVO.T_c0c1);
% load parameters
CamAdjVO.featurePara = feature_para;
CamAdjVO.keyframeMaximumMotion = system_para(1);
CamAdjVO.minimumPoint = system_para(2);
CamAdjVO.bufferSize = system_para(3);
CamAdjVO.maxMotion = system_para(4);
CamAdjVO.xyOnly = system_para(5);
% initial VO system
CamAdjVO.keyframeID = 1; % reserve for future work
CamAdjVO.scaleFactor = 1; % scale the estimated loc to real distance
CamAdjVO.stage = 0;
CamAdjVO.mapAvailable = 0;
% initial keyframe
disp('-----------------init keyframe--------------------');
[mp0, mp1, point3D0, point3D1] = CamAdjStereoView3dReconstruction(...
    img0, img1, cameraParams0, cameraParams1, CamAdjVO.T_c0c1, ...
    CamAdjVO.T_c1c0, feature_para, CamAdjVO.minimumPoint);
% update data
CamAdjVO.tracks0 = mp0;
CamAdjVO.tracks1 = mp1;
CamAdjVO.fp0_valid = true(length(mp0),1);
CamAdjVO.fp1_valid = true(length(mp1),1);
CamAdjVO.pre_T = eye(4);
CamAdjVO.motion_buff = [];
% initial keyframe
CamAdjVO.keyframe{CamAdjVO.keyframeID}.fpt0 = mp0;
CamAdjVO.keyframe{CamAdjVO.keyframeID}.fpt1 = mp1;
CamAdjVO.keyframe{CamAdjVO.keyframeID}.img0 = img0;
CamAdjVO.keyframe{CamAdjVO.keyframeID}.img1 = img1;
CamAdjVO.keyframe{CamAdjVO.keyframeID}.X3D_cam0 = point3D0;
CamAdjVO.keyframe{CamAdjVO.keyframeID}.X3D_cam1 = point3D1;
CamAdjVO.keyframe{CamAdjVO.keyframeID}.pose = CamAdjVO.pose;
return