%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% demo of CamAdj filtering for 3rd view

clc;
close all;
clear all;

addpath(genpath('./functions'));
dataset_path = './datasets/';
camera_path = '/dslr_calibration_undistorted/cameras.txt';
gt_path = '/dslr_calibration_undistorted/images.txt';
point3D_path = '/dslr_calibration_undistorted/points3D.txt';
img_folder = '/images/dslr_images_undistorted/';
dataset_list = dir(dataset_path);

% feature detection (ETH3D benchmark provides 2D points, no need to detect)
% image_grid = [2, 2];
% max_np = 2000;

% initial para
CamAdj_init_bounds = [0.0087, 0.6];
bounds_shrink_rate = [0.5, 0.9];
bounds_expand_rate = [1.5, 1.1];
max_iter = 20;
minimum_points = 50;
% maximum_rejection_percentage = 25;
maximum_rejection_percentage = 30;
% number_of_experiments = 50; % for each 3-view group
number_of_experiments = 500;
MSAC_confidence = 99;

% init data storage
no_CamAdj_rpe_a = [];
no_CamAdj_rpe_g2 = [];
no_CamAdj_re = [];
no_CamAdj_te = [];
CamAdj_rpe_a = [];
CamAdj_rpe_g2 = [];
CamAdj_re = [];
CamAdj_te = [];
dataset_idx_list = [];
image_idx_list = [];
reject_percentage = [];
CamAdj_para_list = [];
% process all datasets 
for dataset_idx = 3 : length(dataset_list)
    dataset_name = dataset_list(dataset_idx).name;
    img_list = dir([dataset_path, dataset_name, img_folder,'*.jpg']);
    camera_para_read = importdata([dataset_path, dataset_name, camera_path]);
    gt_data = importdata([dataset_path, dataset_name, gt_path]);
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
% process every 3-view group of current dataset
    for image_idx = 1 :length(img_list)-2
        % load data
        disp(['Dataset <', dataset_name, '> image group: ', ...
            img_list(image_idx).name,' ',img_list(image_idx+1).name, ...
            ' ', img_list(image_idx+2).name]);
        % load groundtruth poses
        [pose1, image_path1] = extract_camera_pose(img_list(image_idx).name, gt_data);
        [pose2, image_path2] = extract_camera_pose(img_list(image_idx+1).name, gt_data);
        [pose3, image_path3] = extract_camera_pose(img_list(image_idx+2).name, gt_data);
        % load 2D points 
        [point2D_1, Cam1_id] = load2Dpoint(img_list(image_idx).name, gt_data);
        [point2D_2, Cam2_id] = load2Dpoint(img_list(image_idx+1).name, gt_data);
        [point2D_3, Cam3_id] = load2Dpoint(img_list(image_idx+2).name, gt_data);
        % load images
        img1 = imread([dataset_path, dataset_name, '/images/', image_path1]);
        img2 = imread([dataset_path, dataset_name, '/images/', image_path2]);
        img3 = imread([dataset_path, dataset_name, '/images/', image_path3]);
        % Rc, tc: camera pose
        [Rc1,tc1] = extrinsicsToCameraPose(quat2rotm(pose1.q)',pose1.t);
        [Rc2,tc2] = extrinsicsToCameraPose(quat2rotm(pose2.q)',pose2.t);
        [Rc3,tc3] = extrinsicsToCameraPose(quat2rotm(pose3.q)',pose3.t);
        % Re, te: extrinsics
        [Re1, te1] = cameraPoseToExtrinsics(Rc1, tc1);
        [Re2, te2] = cameraPoseToExtrinsics(Rc2, tc2);
        [Re3, te3] = cameraPoseToExtrinsics(Rc3, tc3);
        % init CamAdj
        CamAdj_bounds = CamAdj_init_bounds;
        fail_counter = 0;
        % repeat experiment on the same data group
        for experiment_idx = 1:number_of_experiments
            if fail_counter > 5
                break;  % exclude the data group due to insufficient inliers
            end
            try
                % detect orb features
%                 [fps1, discriptors1] = detectFeature(img1, image_grid, max_np);
%                 [fps2, discriptors2] = detectFeature(img2, image_grid, max_np);
                % use keypoints from benchmark data
                boundary = 10;
                fps1 = point2DonImg(point2D_1, img1, boundary);
                fps2 = point2DonImg(point2D_2, img2, boundary);
                fps3 = point2DonImg(point2D_3, img3, boundary);
                feature1 = SURFPoints(fps1);
                feature2 = SURFPoints(fps2);
                feature3 = SURFPoints(fps3);
                discriptors1 = extractFeatures(rgb2gray(img1), feature1);
                discriptors2 = extractFeatures(rgb2gray(img2), feature2);
                discriptors3 = extractFeatures(rgb2gray(img3), feature3);
                
                indexPairs12 = matchFeatures(discriptors1, discriptors2, ...
                    'MaxRatio', .7, 'Unique',  true);
                
                mp1 = fps1(indexPairs12(:, 1),:);
                mp2 = fps2(indexPairs12(:, 2),:);
                
                points3D = CamAdj3Dreconstruct(Re1, te1, Re2, te2, mp1, mp2, cameraParams, cameraParams);
                
                % convert X_world to X_cam
                % it's supposed to be X_cam = R_cam' * X_world + (- R_cam' * t)
                % however in matlab, everything is transposed
                points3D_cam1 = Rc1 * points3D' + repmat((-tc1*Rc1')',1,length(points3D'));
                points3D_cam1 = points3D_cam1';
                % remove the 3D points with unreasonable distance
                neg_idx = find(points3D_cam1(:,3) <= 1);
                points3D(neg_idx,:) = [];
                points3D_cam1(neg_idx,:) = [];
                mp1(neg_idx,:) = [];
                mp2(neg_idx,:) = [];
                
%                 ptCloud = pointCloud(points3D);
%                 figure;
%                 plotCamera('Location', tc1, 'Orientation', Rc1, 'Size', 0.3, ...
%                     'Color', 'r', 'Label', 'k', 'Opacity', 0);
%                 hold on;
%                 grid on;
%                 plotCamera('Location', tc2, 'Orientation', Rc2, 'Size', 0.3, ...
%                     'Color', 'b', 'Label', 'k+1', 'Opacity', 0);
%                 pcshow(ptCloud, 'MarkerSize', 45);
%                 hold off;
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % test 1: MSAC
                % KLT tracking for the third view
%                 [mp3, validIdx] = CamAdjPointTracker(mp2, img2, img3);
%                 points3D_test1 = points3D(validIdx,:);
%                 mp3_test1 = mp3(validIdx,:);
                % by feature matching
                feature2 = SURFPoints(mp2);
                discriptors2 = extractFeatures(rgb2gray(img2), feature2);
                indexPairs23 = matchFeatures(discriptors2, discriptors3, ...
                    'MaxRatio', .7, 'Unique',  true);
                mp2_test1 = mp2(indexPairs23(:,1),:);
                mp3_test1 = fps3(indexPairs23(:,2),:);
                points3D_test1 = points3D(indexPairs23(:,1),:);
                % PnP for camera 3 (MSAC)
                [orient3, loc3, inlierIdx3] = estimateWorldCameraPose(mp3_test1, ...
                    points3D_test1, cameraParams, 'Confidence', MSAC_confidence);
                N_of_point = length(points3D_test1(inlierIdx3,:)');
                if N_of_point >= minimum_points
                    uv_rep = K * (Rc3 * points3D_test1(inlierIdx3,:)' + repmat((-tc3*Rc3')',1,N_of_point));
                    uv_rep = uv_rep./repmat(uv_rep(3,:),3,1);
                    uv_rep = uv_rep(1:2,:)';
                    
                    e_rpe = (mp3_test1(inlierIdx3,:) - uv_rep).^2;
                    e_rpe_a = sqrt(sum(sum(e_rpe))/N_of_point);
                    R_error = sqrt(sum((cayley_R2c(orient3) - cayley_R2c(Rc3)).^2));
                    % 0.2679 in cayley is 30 degree in euler
                    R_error = min(R_error, 0.2679); 
                    t_error = sqrt(sum((loc3 - tc3).^2));
                    e_rpe_g2 = CamAdjReprojectionCost(mp3_test1(inlierIdx3,:), points3D_test1(inlierIdx3,:), orient3, -loc3*orient3', K);
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % test 2: CamAdj + MSAC
                    T1 = [Rc1', (-tc1*Rc1')'; 0 0 0 1];
                    T2 = [Rc2', (-tc2*Rc2')'; 0 0 0 1];
                    T21 = CamAdjTransformInverse(T2) * T1;
                    reject_percent = 0;
                    for iter = 1:max_iter
                        tic
                        inlier_refined = CamAdjFiltering(mp2', points3D_cam1', T21(1:3,1:3), ...
                            T21(1:3,4), K, CamAdj_bounds);
                        end_time = 1/toc;
                        points3D_CamAdj = points3D(inlier_refined,:);
                        mp1_CamAdj = mp1(inlier_refined,:);
                        mp2_CamAdj = mp2(inlier_refined,:);
                        reject_percent = (length(mp2) - length(mp2_CamAdj))/length(mp2)*100;
                        disp(['Number of processed pairs = ', num2str(length(mp2)), ...
                            ', rejection rate = ', num2str(reject_percent), '%, FPS = ', num2str(end_time)]);
                        if reject_percent < 1
                            CamAdj_bounds = CamAdj_bounds.*bounds_shrink_rate;
                        elseif reject_percent > maximum_rejection_percentage
                            CamAdj_bounds = CamAdj_bounds.*bounds_expand_rate;
                        else
%                             disp(['reject ', num2str(reject_percent), '% points']);
                            break;
                        end
                    end
%                     KLT tracking for the third view
%                     [mp3, validIdx] = CamAdjPointTracker(mp2_CamAdj, img2, img3);
%                     mp3_test2 = mp3(validIdx,:);
%                     points3D_test2 = points3D_CamAdj(validIdx,:);
                    % by feature matching
                    feature2 = SURFPoints(mp2_CamAdj);
                    discriptors2 = extractFeatures(rgb2gray(img2), feature2);
                    indexPairs23 = matchFeatures(discriptors2, discriptors3, ...
                    'MaxRatio', .7, 'Unique',  true);
                    mp2_test2 = mp2_CamAdj(indexPairs23(:,1),:);
                    mp3_test2 = fps3(indexPairs23(:,2),:);
                    points3D_test2 = points3D_CamAdj(indexPairs23(:,1),:);
                    
                    % PnP for camera 3 (MSAC)
                    [orient3, loc3, inlierIdx3] = estimateWorldCameraPose(mp3_test2, ...
                        points3D_test2, cameraParams, 'Confidence', MSAC_confidence);
                    
                    N_of_point = length(points3D_test2(inlierIdx3,:)');
                    if N_of_point >= minimum_points
                        uv_rep = K * (Rc3 * points3D_test2(inlierIdx3,:)' + repmat((-tc3*Rc3')',1,N_of_point));
                        uv_rep = uv_rep./repmat(uv_rep(3,:),3,1);
                        uv_rep = uv_rep(1:2,:)';
                        
                        e_rpe = (mp3_test2(inlierIdx3,:) - uv_rep).^2;
                        CamAdj_e_rpe_a = sqrt(sum(sum(e_rpe))/N_of_point);
                        CamAdj_R_error = sqrt(sum((cayley_R2c(orient3) - cayley_R2c(Rc3)).^2));
                        CamAdj_R_error = min(CamAdj_R_error, 0.2679); 
                        CamAdj_t_error = sqrt(sum((loc3 - tc3).^2));
                        CamAdj_e_rpe_g2 = CamAdjReprojectionCost(mp3_test2(inlierIdx3,:), points3D_test2(inlierIdx3,:), orient3, -loc3*orient3', K);
                        
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        % record results
                        no_CamAdj_rpe_a = [no_CamAdj_rpe_a; e_rpe_a];
                        no_CamAdj_rpe_g2 = [no_CamAdj_rpe_g2; e_rpe_g2];
                        no_CamAdj_re = [no_CamAdj_re; R_error];
                        no_CamAdj_te = [no_CamAdj_te; t_error];
                        CamAdj_rpe_a = [CamAdj_rpe_a; CamAdj_e_rpe_a];
                        CamAdj_rpe_g2 = [CamAdj_rpe_g2; CamAdj_e_rpe_g2];
                        CamAdj_re = [CamAdj_re; CamAdj_R_error];
                        CamAdj_te = [CamAdj_te; CamAdj_t_error];
                        dataset_idx_list = [dataset_idx_list; dataset_idx];
                        image_idx_list = [image_idx_list; image_idx];
                        reject_percentage = [reject_percentage; reject_percent];
                        Better_R_percent = length(find(CamAdj_re < no_CamAdj_re))/length(no_CamAdj_re)*100;
                        figure(1), hold on;
                        plot(no_CamAdj_re,'b');
                        plot(CamAdj_re,'r');
                        hold off;
                        legend(['MSAC, mean R error=', num2str(mean(no_CamAdj_re)),', t error=', num2str(mean(no_CamAdj_te))],...
                            ['CamAdj+MSAC, R error=',num2str(mean(CamAdj_re)),', t error=', num2str(mean(CamAdj_te))]);
                        title(['Current dataset:<', dataset_name, '>, better R rate=', num2str(Better_R_percent), '%']);
                        ylabel('Rotation MSE in cayley');
                        xlabel('number of experiments');
                        fail_counter = 0;
                    else
                        disp('CamAdj+MSAC: skipped due to insufficient inliers...');
                        fail_counter = fail_counter + 1;
                    end
                else
                    disp('MSAC: skipped due to insufficient inliers...');
                    fail_counter = fail_counter + 1;
                end
            catch
                disp('Fail to match enough feature points...');
                fail_counter = fail_counter + 1;
            end
        end
    end
end

%% save data
data.no_CamAdj_rpe_a = no_CamAdj_rpe_a;
data.no_CamAdj_rpe_g2 = no_CamAdj_rpe_g2;
data.no_CamAdj_re = no_CamAdj_re;
data.no_CamAdj_te = no_CamAdj_te;
data.CamAdj_rpe_a = CamAdj_rpe_a;
data.CamAdj_rpe_g2 = CamAdj_rpe_g2;
data.CamAdj_re = CamAdj_re;
data.CamAdj_te = CamAdj_te;
data.dataset_idx_list = dataset_idx_list;
data.image_idx_list = image_idx_list;
data.dataset_list = dataset_list;
data.reject_percentage = reject_percentage;
save('./results/CamAdj_3views_results.mat','data');
disp('save result to ./results');

%% plot AUC
% pi/180 euler = 0.0087 cayley
% pi/2 euler = 1 cayley
threshold = 0.001:0.0001:0.087;
AUC_no_CamAdj = generate_AUC(no_CamAdj_re, threshold);
AUC_CamAdj = generate_AUC(CamAdj_re, threshold);
figure;
% subplot(1,2,1);
hold on;
plot(threshold,AUC_no_CamAdj,'b','LineWidth',2);
plot(threshold,AUC_CamAdj,'r','LineWidth',2);
hold off;
grid on;
legend('MSAC','CamAdj+MSAC')
title('Rotation estimation error AUC (third view)');
ylabel('Proportion');
xlabel('Error threshold in Cayley');

rmpath(genpath('./functions'));