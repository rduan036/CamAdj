function CamAdjVO = CamAdj_VO_reset(CamAdjVO, img0, img1)
disp('-----------------reset keyframe--------------------');
CamAdjVO.lastFrame0 = img0;
CamAdjVO.lastFrame1 = img1;
CamAdjVO.stage = 0;
CamAdjVO.mapAvailable = 0;
cameraParams0 = CamAdjVO.cameraParams0;
cameraParams1 = CamAdjVO.cameraParams1;
feature_para = CamAdjVO.featurePara;
sos_para = CamAdjVO.sos_para;
[mp0, mp1, point3D0, point3D1] = CamAdjStereoView3dReconstruction(...
    img0, img1, cameraParams0, cameraParams1, CamAdjVO.T_c0c1, ...
    CamAdjVO.T_c1c0, feature_para, sos_para, CamAdjVO.minimumPoint);
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