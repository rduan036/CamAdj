%% CamAdj for long term visual odometry:
% Please download EuRoc rosbag into ./euroc
% download link:
% https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc; close all;
addpath(genpath('./functions'));
%% load rosbag of EuRoc Vicon Room
dataset_path = fullfile('.\euroc'); % modify the path for your test
dataset_name = 'V1_01_easy.bag';
% dataset_name = 'V1_02_medium.bag';
% dataset_name = 'V1_03_difficult.bag';
% dataset_name = 'V2_01_easy.bag';
% dataset_name = 'V2_02_medium.bag';
bag = rosbag([dataset_path,'\',dataset_name]);
cam0_msg = select(bag,'Topic','/cam0/image_raw');
cam1_msg = select(bag,'Topic','/cam1/image_raw');
vicon_msg = select(bag,'Topic','/vicon/firefly_sbx/firefly_sbx');
%% Camera Info
cam_hight = 480;
cam_width = 752;
imageSize = [cam_hight, cam_width];
% camera 0
fx = 458.654; % focal length (mm) / pixel_width_x (mm) = pixels
fy = 457.296; % focal length (mm) / pixel_height_y (mm) = pixels
cx = 367.215; % image center x-coordinate
cy = 248.375; % image center y-coordinate
radialDistortion = [-0.28340811, 0.07395907];
tangentialDistortion = [0.00019359, 1.76187114e-05];
intrinsics  = [fx 0 0; 0 fy 0; cx cy 1];
cameraParams0 = cameraParameters('IntrinsicMatrix',intrinsics ,...
    'RadialDistortion',radialDistortion,...
    'TangentialDistortion',tangentialDistortion);
% camera 1
fx = 457.587; % focal length (mm) / pixel_width_x (mm) = pixels
fy = 456.134; % focal length (mm) / pixel_height_y (mm) = pixels
cx = 379.999; % image center x-coordinate
cy = 255.238; % image center y-coordinate¡¤
radialDistortion = [-0.28368365,  0.07451284];
tangentialDistortion = [-0.00010473, -3.55590700e-05];
intrinsics  = [fx 0 0; 0 fy 0; cx cy 1];
cameraParams1 = cameraParameters('IntrinsicMatrix',intrinsics ,...
    'RadialDistortion',radialDistortion,...
    'TangentialDistortion',tangentialDistortion);
%% VO paras
% Feature detection para:
% [feature MinQuality, grid number [H, W], number of feature in each grid,
% edge(pixcel)]
feature_para = [0.02, 8, 14, 2, 5];
% System para
% [keyframe maximum motion, minimum tracked points, trace filter buffer, 
% maximum movement(in meter), 2D map VO flag]
system_para = [35, 20, 5, 0.4, 0]; 
% UAV body frame to camera frame
T_BS_cam0 = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975;
    0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768;
    -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949;
    0.0, 0.0, 0.0, 1.0];
T_BS_cam1 = [0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556;
    0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024;
    -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038;
    0.0, 0.0, 0.0, 1.0];
% VICON to bady frame
T_BS_v2b = [ 0.33638, -0.01749,  0.94156,  0.06901;
    -0.02078, -0.99972, -0.01114, -0.02781;
    0.94150, -0.01582, -0.33665, -0.12395;
    0.0,      0.0,      0.0,      1.0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization
frame_counter = 1;
vicon_counter = 1;
total_frame = min(cam0_msg.NumMessages, cam1_msg.NumMessages);
current_time = cam0_msg.MessageList(frame_counter,1).Time;
% get image data
img_msg = readMessages(cam0_msg,frame_counter);
img0 = readImage(img_msg{1,1});
img_msg = readMessages(cam1_msg,frame_counter);
img1 = readImage(img_msg{1,1});
vicon_time = vicon_msg.MessageList(vicon_counter,1).Time;
% find the sync vicon pose
while vicon_time < current_time
    if vicon_counter < vicon_msg.NumMessages - 1
        vicon_counter = vicon_counter + 1;
        vicon_time = vicon_msg.MessageList(vicon_counter,1).Time;
    else
        break;
    end
end
vicon_pose = readMessages(vicon_msg, vicon_counter);
pose_q = vicon_pose{1,1}.Transform.Rotation;
pose_t = vicon_pose{1,1}.Transform.Translation;
q_VICON = [pose_q.W, pose_q.X, pose_q.Y, pose_q.Z];
R_VICON = quat2rotm(q_VICON);
t_VICON = [pose_t.X; pose_t.Y; pose_t.Z];
Rt_VICON = [R_VICON, t_VICON ; 0 0 0 1];
UAV_pose =  Rt_VICON*T_BS_v2b;
% load images
cur_I0 = undistortImage(img0, cameraParams0);
cur_I1 = undistortImage(img1, cameraParams1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% init VO
rotation = rotm2eul(UAV_pose(1:3,1:3));
vicon_traces = UAV_pose(1:3,4);
our_traces = UAV_pose(1:3,4);
our_rotation = rotm2eul(UAV_pose(1:3,1:3));
time = 0;
tic;
CamAdjVO = CamAdj_VO_init(cur_I0, cur_I1, feature_para, system_para,...
    cameraParams0, cameraParams1, T_BS_cam0, T_BS_cam1, UAV_pose);
time = time + toc;
while frame_counter < total_frame
    frame_counter = frame_counter + 1;
    current_time = cam0_msg.MessageList(frame_counter,1).Time;
    img_msg = readMessages(cam0_msg,frame_counter);
    img0 = readImage(img_msg{1,1});
    img_msg = readMessages(cam1_msg,frame_counter);
    img1 = readImage(img_msg{1,1});
    vicon_time = vicon_msg.MessageList(vicon_counter,1).Time;
    while vicon_time < current_time
        if vicon_counter < vicon_msg.NumMessages - 1
            vicon_counter = vicon_counter + 1;
            vicon_time = vicon_msg.MessageList(vicon_counter,1).Time;
        else
            break;
        end
    end
    vicon_pose = readMessages(vicon_msg, vicon_counter);
    pose_q = vicon_pose{1,1}.Transform.Rotation;
    pose_t = vicon_pose{1,1}.Transform.Translation;
    q_VICON = [pose_q.W, pose_q.X, pose_q.Y, pose_q.Z];
    R_VICON = quat2rotm(q_VICON);
    t_VICON = [pose_t.X; pose_t.Y; pose_t.Z];
    Rt_VICON = [R_VICON, t_VICON ; 0 0 0 1];
    UAV_pose = Rt_VICON*T_BS_v2b;
    % display pose
    vicon_traces = [ vicon_traces, UAV_pose(1:3,4) ]; % the trajectory of the camera frame
    rotation = [rotation; rotm2eul(UAV_pose(1:3,1:3))];
    cur_I0 = undistortImage(img0, cameraParams0);
    cur_I1 = undistortImage(img1, cameraParams1);
    tic;
    CamAdjVO = CamAdj_VO_step(CamAdjVO, cur_I0, cur_I1);
    time = time + toc;
    disp(['FPS = ', num2str(frame_counter/time)]);
    our_traces = [our_traces, CamAdjVO.pose(1:3,4)];
    our_rotation = [our_rotation; rotm2eul(CamAdjVO.pose(1:3,1:3))];
    % normally, x = K(RX + t), where X is 3 * N
    % in matlab, raw is prior dimension because of the memory operation
    % it uses x = (XR' + t')*K', where X is N * 3
    % therefore, the transpose of UAV pose is used
%     UAV_pose_T = UAV_pose';
    figure(1);
    clf;
    subplot(2,1,1);
    hold on;
    plot3( vicon_traces(1,:), vicon_traces(2,:), vicon_traces(3,:), 'k' );
    plot3( our_traces(1,:), our_traces(2,:), our_traces(3,:), 'r' );
    plot_pose(UAV_pose,'g');
    plot_pose(double(CamAdjVO.pose),'r');
    view( -50, 30 );
    title([' Frame: ' num2str(frame_counter)] );
    hold off;
    xlabel('x(m)');
    ylabel('y(m)');
    zlabel('z(m)');
    legend({'VICON','CamAdj'},'Location', 'north');
    % show keypoints
    points_loc0 = CamAdjVO.tracks0;
    points_loc1 = CamAdjVO.tracks1;
    points_loc1(:,1) = points_loc1(:,1) + size(cur_I0,2)*ones(size(points_loc1(:,1)));
    keypoints = [points_loc0; points_loc1];
    image_views = [cur_I0, cur_I1];
    subplot(2,1,2);
    imshow(image_views);
    hold on;
    plot(keypoints(:,1), keypoints(:,2), '.g');
    hold off;
    set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.2, 0.1, 0.4, 0.8]);
    drawnow;
end

%% result evaluation
gt_t = vicon_traces;
our_t = double(our_traces);
[R, t] = umeyama(our_t, gt_t);
our_t_aligned = R * our_t + repmat(t,1,length(our_t));
gt_t = gt_t';
our_t_aligned = our_t_aligned';
RMSE = sqrt(mean2((gt_t - our_t_aligned).^2));
figure(2);
clf;
hold on;
plot3( our_t_aligned(:,1), our_t_aligned(:,2), our_t_aligned(:,3), 'r' );
plot3( gt_t(:,1), gt_t(:,2), gt_t(:,3), 'k' );
hold off;
xlabel(['RMSE = ', num2str(RMSE)]);
grid on;

rmpath(genpath('./functions'));