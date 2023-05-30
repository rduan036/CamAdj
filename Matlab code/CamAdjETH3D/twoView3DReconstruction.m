%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% demo of CamAdj filtering for two view triangulation

clc;
clear all;
close all;

addpath(genpath('./functions'));
dataset_path = './datasets/';
camera_path = '/dslr_calibration_undistorted/cameras.txt';
gt_path = '/dslr_calibration_undistorted/images.txt';
point3D_path = '/dslr_calibration_undistorted/points3D.txt';
img_folder = '/images/dslr_images_undistorted/';
dataset_list = dir(dataset_path);

minimum_np = 50;
% CamAdj_para = [0.001, 0.5, 1, 30, 1, 30]; % boulders, terrace
CamAdj_para = [0.001, 0.5, 1, 15, 1, 15]; % bridge, courtyard, relief_2, terrace_2

experiment_number = 5;
no_CamAdj_re = [];
no_CamAdj_te = [];
CamAdj_re = [];
CamAdj_te = [];
reject_percentage = [];

delimiterIn = ' ';
headerlinesIn = 3;

% dataset_name = 'boulders';
% dataset_name = 'bridge';
dataset_name = 'courtyard';
% dataset_name = 'relief_2';
% dataset_name = 'terrace';
% dataset_name = 'terrace_2';

img_list = dir([dataset_path, dataset_name, img_folder,'*.jpg']);
camera_para_read = importdata([dataset_path, dataset_name, camera_path]);
gt_data = importdata([dataset_path, dataset_name, gt_path]);
point3D_fileID = fopen([dataset_path, dataset_name, point3D_path]);
point3D_data = [];
line_idx = 1;
while ~feof(point3D_fileID)
    tline = fgetl(point3D_fileID);
    point3D_data{line_idx} = tline;
    line_idx = line_idx + 1;
end
fclose(point3D_fileID);

img_width = str2double(camera_para_read.textdata(4,3));
img_height = str2double(camera_para_read.textdata(4,4));
fx = camera_para_read.data(2);
fy = str2double(camera_para_read.textdata(5,1));
cx = str2double(camera_para_read.textdata(5,2));
cy = str2double(camera_para_read.textdata(5,3));

K = [ fx, 0, cx;
    0, fy, cy;
    0,  0,  1 ];
cameraParams = cameraParameters('IntrinsicMatrix',K');

for image_idx = 1:length(img_list) - 1
    [pose1, image_path1] = extract_camera_pose(img_list(image_idx).name, gt_data);
    [pose2, image_path2] = extract_camera_pose(img_list(image_idx+1).name, gt_data);
    
    img1 = imread([dataset_path, dataset_name, '/images/', image_path1]);
    img2 = imread([dataset_path, dataset_name, '/images/', image_path2]);
    
    [point2D_1, Cam1_id] = load2Dpoint(img_list(image_idx).name, gt_data);
    [point2D_2, Cam2_id] = load2Dpoint(img_list(image_idx+1).name, gt_data);
    [points3D, colormap] = load3DpointOfTwoView(Cam1_id, Cam2_id, point3D_data, headerlinesIn);
    
    % Rc, tc: camera pose
    [Rc1,tc1] = extrinsicsToCameraPose(quat2rotm(pose1.q)',pose1.t);
    [Rc2,tc2] = extrinsicsToCameraPose(quat2rotm(pose2.q)',pose2.t);
    
    % Re, te: extrinsics
    [Re1, te1] = cameraPoseToExtrinsics(Rc1, tc1);
    [Re2, te2] = cameraPoseToExtrinsics(Rc2, tc2);
    %     % plot 3D map
    %     ptCloud = pointCloud(points3D, 'Color', colormap);
    %     figure;
    %     plotCamera('Location', tc1, 'Orientation', Rc1, 'Size', 0.3, ...
    %         'Color', 'r', 'Label', 'k+1', 'Opacity', 0);
    %     hold on;
    %     grid on;
    %     plotCamera('Location', tc2, 'Orientation', Rc2, 'Size', 0.3, ...
    %         'Color', 'b', 'Label', 'k+2', 'Opacity', 0);
    %     pcshow(ptCloud, 'MarkerSize', 45);
    
    % load 2D points
    boundary = 10;
    fps1 = point2DonImg(point2D_1, img1, boundary);
    fps2 = point2DonImg(point2D_2, img2, boundary);
    % point matching using SURF feature
    feature1 = SURFPoints(fps1);
    feature2 = SURFPoints(fps2);
    discriptors1 = extractFeatures(rgb2gray(img1), feature1);
    discriptors2 = extractFeatures(rgb2gray(img2), feature2);
    indexPairs12 = matchFeatures(discriptors1, discriptors2, 'MaxRatio', .7, 'Unique',  true);
    mp1 = fps1(indexPairs12(:, 1),:);
    mp2 = fps2(indexPairs12(:, 2),:);
    
    figure;
    showMatchedFeatures(img1,img2,mp1,mp2);
    title('Two-view feature matching using SURF discriptor');
    
    fail_counter = 0;
    
    for experiment_idx = 1:experiment_number
        % without CamAdj
        [orient2, loc2, inlierIdx2] = CamAdjFindRelativePose(mp1, mp2, cameraParams, cameraParams);
        %             Re2_without = Re1*orient2';
        points3D_without = CamAdj3Dreconstruct(Re1, te1, Re2, te2, mp1(inlierIdx2,:), mp2(inlierIdx2,:), cameraParams, cameraParams);
        % convert X_world to X_cam
        % it's supposed to be X_cam = R_cam' * X_world + (- R_cam' * t)
        % however in matlab, everything is transposed
        points3D_cam1 = Rc1 * points3D_without' + repmat((-tc1*Rc1')',1,length(points3D_without'));
        points3D_cam1 = points3D_cam1';
        % remove the 3D points with unreasonable distance
        neg_idx = find(points3D_cam1(:,3) <= 0);
        points3D_without(neg_idx,:) = [];
        points3D_cam1(neg_idx,:) = [];
        mp1(neg_idx,:) = [];
        mp2(neg_idx,:) = [];
        
        % with CamAdj
        T1 = [Rc1', (-tc1*Rc1')'; 0 0 0 1];
        T2 = [Rc2', (-tc2*Rc2')'; 0 0 0 1];
        T21 = CamAdjTransformInverse(T2) * T1;
        inlier_refined = CamAdjFiltering_R8(mp1', mp2', T21(1:3,1:3), ...
            T21(1:3,4), K, CamAdj_para);
        
        mp1_refine = mp1(inlier_refined,:);
        mp2_refine = mp2(inlier_refined,:);
        
        [orient2, loc2, inlierIdx2] = CamAdjFindRelativePose(mp1_refine, mp2_refine, cameraParams, cameraParams);
        %             Re2_with = Re1*orient2';
        points3D_with = CamAdj3Dreconstruct(Re1, te1, Re2, te2, mp1_refine(inlierIdx2,:), mp2_refine(inlierIdx2,:), cameraParams, cameraParams);
        
        %             cmatrix = ones(size(points3D)).*[0 1 0];
        ptCloud = pointCloud(points3D, 'Color', colormap);
        cmatrix = ones(size(points3D_without)).*[1 0 0];
        ptCloud_without = pointCloud(points3D_without, 'Color', cmatrix);
        cmatrix = ones(size(points3D_with)).*[0 1 0];
        ptCloud_with = pointCloud(points3D_with, 'Color', cmatrix);
        
        if length(points3D_without) < 50 || length(points3D_with) < 50
            disp('========skipped due to insufficient points...=========');
            experiment_idx = experiment_idx - 1;
            fail_counter = fail_counter + 1;
            if fail_counter > 3
                disp('stopped due to insufficient points...');
                break;
            end
        else
            max_error = 3; % the errors that larger than 2m will be counted as 2m
            error3D_without = compute3DpointError(ptCloud, points3D_without, max_error);
            error3D_with = compute3DpointError(ptCloud, points3D_with, max_error);
            rejection_rate = (length(mp1) - length(inlier_refined))/length(mp1)*100;
            disp(['======== Dataset ', dataset_name, ' Image ', img_list(image_idx).name,...
                ' Test No.', num2str(experiment_idx) '=========']);
            disp(['CamAdj rejection rate = ', num2str(rejection_rate), '%']);
            disp(['Number of reconstructed points: MSAC = ', ...
                num2str(length(points3D_without)), ...
                ', CamAdj + MSAC = ', num2str(length(points3D_with))]);
            disp(['3D reconstruction error: MSAC = ', ...
                num2str(error3D_without), ...
                ', CamAdj + MSAC = ', num2str(error3D_with)]);
            
            figure;
            plotCamera('Location', tc1, 'Orientation', Rc1, 'Size', 0.5, ...
                'Color', 'k', 'Label', 'k', 'Opacity', 0);
            hold on;
            grid on;
            plotCamera('Location', tc2, 'Orientation', Rc2, 'Size', 0.5, ...
                'Color', 'k', 'Label', 'k+1', 'Opacity', 0);
            pcshow(ptCloud, 'MarkerSize', 30);
            pcshow(ptCloud_without, 'MarkerSize', 80);
            pcshow(ptCloud_with, 'MarkerSize', 100);
            hold off;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            set(gcf,'color','w');
            set(gca,'color','w');
            drawnow;
        end
    end
end

rmpath(genpath('./functions'));