function CamAdjVO = CamAdj_VO_step(CamAdjVO, currentFrame0, currentFrame1)
disp('-----------------nonkeyframe--------------------');
% update image frames
lastFrame0 = CamAdjVO.lastFrame0;
lastFrame1 = CamAdjVO.lastFrame1;
% load camera params
cameraParams0 = CamAdjVO.cameraParams0;
cameraParams1 = CamAdjVO.cameraParams1;
K0 = cameraParams0.IntrinsicMatrix';
K1 = cameraParams1.IntrinsicMatrix';
% points tracking
Points0 = CamAdjVO.tracks0;
Points1 = CamAdjVO.tracks1;
[Points0_tracked, validIdx0] = CamAdjPointTracker(Points0, lastFrame0, currentFrame0, CamAdjVO.minimumPoint, CamAdjVO.featurePara);
[Points1_tracked, validIdx1] = CamAdjPointTracker(Points1, lastFrame1, currentFrame1, CamAdjVO.minimumPoint, CamAdjVO.featurePara);
validIdx0 = CamAdjRemoveLargeShift(Points0, Points0_tracked, validIdx0);
validIdx1 = CamAdjRemoveLargeShift(Points1, Points1_tracked, validIdx1);
CamAdjVO.fp0_valid(CamAdjVO.fp0_valid == 1) = validIdx0;
CamAdjVO.fp1_valid(CamAdjVO.fp1_valid == 1) = validIdx1;
% filtering points
mp0_tracked = Points0_tracked(validIdx0,:);
mp1_tracked = Points1_tracked(validIdx1,:);
mp0_keyframe = CamAdjVO.keyframe{CamAdjVO.keyframeID}.fpt0(CamAdjVO.fp0_valid,:);
mp1_keyframe = CamAdjVO.keyframe{CamAdjVO.keyframeID}.fpt1(CamAdjVO.fp1_valid,:);
point3D0 = CamAdjVO.keyframe{CamAdjVO.keyframeID}.X3D_cam0(CamAdjVO.fp0_valid,:);
point3D1 = CamAdjVO.keyframe{CamAdjVO.keyframeID}.X3D_cam1(CamAdjVO.fp1_valid,:);
% tracking check
% figure(2); 
% subplot(2,1,1);
% showMatchedFeatures(lastFrame0,currentFrame0,matchedPoints1,matchedPoints2);
% subplot(2,1,2);
% showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2);

bad_distribution0 = CamAdjDistributionCheck(mp0_tracked, currentFrame0);
bad_distribution1 = CamAdjDistributionCheck(mp1_tracked, currentFrame1);
motion_flag0 = CamAdjMotionDetection(mp0_keyframe, mp0_tracked, CamAdjVO.keyframeMaximumMotion);
motion_flag1 = CamAdjMotionDetection(mp1_keyframe, mp1_tracked, CamAdjVO.keyframeMaximumMotion);
% update keyframe when the condition matched
if size(mp0_tracked,1) < CamAdjVO.minimumPoint || ...
   size(mp1_tracked,1) < CamAdjVO.minimumPoint || ...
   bad_distribution0 || bad_distribution1 || motion_flag0 || motion_flag1
    disp('-----------------updating keyframe--------------------');
    [mp0, mp1, point3D0, point3D1] = CamAdjStereoView3dReconstruction(...
        lastFrame0, lastFrame1, cameraParams0, cameraParams1, CamAdjVO.T_c0c1, ...
        CamAdjVO.T_c1c0, CamAdjVO.featurePara, CamAdjVO.minimumPoint);
    % update point tracking data
    CamAdjVO.tracks0 = mp0;
    CamAdjVO.tracks1 = mp1;
    CamAdjVO.fp0_valid = true(length(mp0),1);
    CamAdjVO.fp1_valid = true(length(mp1),1);
    CamAdjVO.pre_T = eye(4);
    CamAdjVO.motion_buff = [];
    % initial keyframe
    CamAdjVO.keyframe{CamAdjVO.keyframeID}.fpt0 = mp0;
    CamAdjVO.keyframe{CamAdjVO.keyframeID}.fpt1 = mp1;
    CamAdjVO.keyframe{CamAdjVO.keyframeID}.img0 = currentFrame0;
    CamAdjVO.keyframe{CamAdjVO.keyframeID}.img1 = currentFrame1;
    CamAdjVO.keyframe{CamAdjVO.keyframeID}.X3D_cam0 = point3D0;
    CamAdjVO.keyframe{CamAdjVO.keyframeID}.X3D_cam1 = point3D1;
    CamAdjVO.keyframe{CamAdjVO.keyframeID}.pose = CamAdjVO.pose;
    % points tracking
    Points0 = CamAdjVO.tracks0(CamAdjVO.fp0_valid,:);
    Points1 = CamAdjVO.tracks1(CamAdjVO.fp1_valid,:);
    [Points0_tracked, validIdx0] = CamAdjPointTracker(Points0, lastFrame0, currentFrame0, CamAdjVO.minimumPoint, CamAdjVO.featurePara);
    [Points1_tracked, validIdx1] = CamAdjPointTracker(Points1, lastFrame1, currentFrame1, CamAdjVO.minimumPoint, CamAdjVO.featurePara);
    validIdx0 = CamAdjRemoveLargeShift(Points0, Points0_tracked, validIdx0);
    validIdx1 = CamAdjRemoveLargeShift(Points1, Points1_tracked, validIdx1);
    CamAdjVO.fp0_valid(CamAdjVO.fp0_valid == 1) = validIdx0;
    CamAdjVO.fp1_valid(CamAdjVO.fp1_valid == 1) = validIdx1;
    % filtering points
    mp0_tracked = Points0_tracked(validIdx0,:);
    mp1_tracked = Points1_tracked(validIdx1,:);
    mp0_keyframe = CamAdjVO.keyframe{CamAdjVO.keyframeID}.fpt0(CamAdjVO.fp0_valid,:);
    mp1_keyframe = CamAdjVO.keyframe{CamAdjVO.keyframeID}.fpt1(CamAdjVO.fp1_valid,:);
    point3D0 = CamAdjVO.keyframe{CamAdjVO.keyframeID}.X3D_cam0(CamAdjVO.fp0_valid,:);
    point3D1 = CamAdjVO.keyframe{CamAdjVO.keyframeID}.X3D_cam1(CamAdjVO.fp1_valid,:);
end
% load pose of last keyframe
UAV_pose_keyframe = CamAdjVO.keyframe{CamAdjVO.keyframeID}.pose;
% load transform of stereo cameras to uav body 
T_cam2body0 = CamAdjVO.T_cam2body0;
T_cam2body1 = CamAdjVO.T_cam2body1;
T_body2cam0 = CamAdjVO.T_body2cam0;
T_body2cam1 = CamAdjVO.T_body2cam1;
% load previous pose estimations
pre_T = CamAdjVO.pre_T;
pre_T_cam0 = T_cam2body0 * pre_T * T_body2cam0;
pre_T_cam1 = T_cam2body1 * pre_T * T_body2cam1;
pre_loc_r2f0 = pre_T_cam0(1:3,4)';
pre_loc_r2f1 = pre_T_cam1(1:3,4)';
% figure(2);
% subplot(1,2,1);
% showMatchedFeatures(lastFrame0, currentFrame0, mp0_lastframe, mp0_tracked);
% xlabel('cam 0');
% subplot(1,2,2);
% showMatchedFeatures(lastFrame1, currentFrame1, mp1_lastframe, mp1_tracked);
% xlabel('cam 1');
try
    [orient0, loc0, inlierIdx0] = estimateWorldCameraPose(mp0_tracked, point3D0, cameraParams0);
%     orient_r2f0 = orient0;
%     loc_r2f0 = loc0;
    [orient_r2f0, loc_r2f0] = CamAdjVisualServoing(mp0_tracked(inlierIdx0,:)', point3D0(inlierIdx0,:)', orient0, loc0, K0);
catch
    [orient0, loc, inlierIdx0] = ...
        CamAdjFindRelativePose(mp0_keyframe, mp0_tracked, cameraParams0, cameraParams1);
    loc0 = pre_loc_r2f0;
    [orient_r2f0, loc_r2f0] = CamAdjVisualServoing(mp0_tracked(inlierIdx0,:)', point3D0(inlierIdx0,:)', orient0, loc0, K0);
end
try
    [orient1, loc1, inlierIdx1] = estimateWorldCameraPose(mp1_tracked, point3D1, cameraParams1);
%     orient_r2f1 = orient1;
%     loc_r2f1 = loc1;
    [orient_r2f1, loc_r2f1] = CamAdjVisualServoing(mp1_tracked(inlierIdx1,:)', point3D1(inlierIdx1,:)', orient1, loc1, K1);
catch
    [orient1, loc, inlierIdx1] = ...
        CamAdjFindRelativePose(mp1_keyframe, mp1_tracked, cameraParams0, cameraParams1);
    loc1 = pre_loc_r2f1;
    [orient_r2f1, loc_r2f1] = CamAdjVisualServoing(mp1_tracked(inlierIdx1,:)', point3D1(inlierIdx1,:)', orient1, loc1, K1);
end
if CamAdjVO.xyOnly
    eul0 = rotm2eul(orient_r2f0);
    eul1 = rotm2eul(orient_r2f1);
    orient_r2f0 = eul2rotm([0 eul0(2) 0]);
    orient_r2f1 = eul2rotm([0 eul1(2) 0]);
end
estimate_T0 = [orient_r2f0',loc_r2f0'; 0 0 0 1];
estimate_T1 = [orient_r2f1',loc_r2f1'; 0 0 0 1];
T_uav0 = T_body2cam0 * estimate_T0 * T_cam2body0;
T_uav1 = T_body2cam1 * estimate_T1 * T_cam2body1;
confidence0 = sum(inlierIdx0)/length(inlierIdx0);
confidence1 = sum(inlierIdx1)/length(inlierIdx1);
T_uav = CamAdjPoseFusion(T_uav0, T_uav1, confidence0, confidence1);

% abrapt motion rejection
current_pose = UAV_pose_keyframe * T_uav;
current_ang = rotm2eul(current_pose(1:3,1:3));
current_ang = current_ang';
if size(CamAdjVO.motion_buff,2) > CamAdjVO.bufferSize
    mean_motion = mean(CamAdjVO.motion_buff, 2);
    if CamAdjVO.xyOnly
        last_rotation = CamAdjVO.motion_buff(1:3,end);
        current_rotate = max(abs(current_ang - last_rotation));
        if current_rotate > pi/6
            current_pose(1:3,1:3) = CamAdjVO.pose(1:3,1:3);
            disp('rotation rejected...');
        end
    end
    mean_position = mean_motion(4:6);
    current_shift = norm(current_pose(1:3,4) - mean_position);
    if current_shift > CamAdjVO.maxMotion
        current_pose(1:3,4) = CamAdjVO.pose(1:3,4);
        disp('translation rejected...');
    end
    CamAdjVO.motion_buff(:,1) = [];
end

% update data
CamAdjVO.tracks0 = mp0_tracked;
CamAdjVO.tracks1 = mp1_tracked;
CamAdjVO.lastFrame0 = currentFrame0;
CamAdjVO.lastFrame1 = currentFrame1;
CamAdjVO.pose = current_pose;
CamAdjVO.pre_T = T_uav;
CamAdjVO.motion_buff = [CamAdjVO.motion_buff, [current_ang;CamAdjVO.pose(1:3,4)]];

return