%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% demo of CamAdj filtering for two view triangulation (paper Fig.5 right) 

clc;
clear all;
close all;

addpath(genpath('./functions'));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% demo of CamAdj filtering for two view
dataset_path = './datasets/';
camera_path = '/dslr_calibration_undistorted/cameras.txt';
gt_path = '/dslr_calibration_undistorted/images.txt';
point3D_path = '/dslr_calibration_undistorted/points3D.txt';
img_folder = '/images/dslr_images_undistorted/';
dataset_list = dir(dataset_path);

image_grid = [2, 2];
max_np = 2000;
CamAdj_para = [0.001, 0.5, 1, 15, 1, 15];
minimum_points = 16;
experiment_number = 200;
no_CamAdj_re = [];
no_CamAdj_te = [];
CamAdj_re = [];
CamAdj_te = [];
reject_percentage = [];
MSAC_confidence = 85;

dataset_name = 'courtyard';
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

for image_idx = 1:length(img_list) - 1
    img_list(image_idx).name;
    [pose1, image_path1] = extract_camera_pose(img_list(image_idx).name, gt_data);
    [pose2, image_path2] = extract_camera_pose(img_list(image_idx+1).name, gt_data);
    
    img1 = imread([dataset_path, dataset_name, '/images/', image_path1]);
    img2 = imread([dataset_path, dataset_name, '/images/', image_path2]);
    
    % Rc, tc: camera pose
    [Rc1,tc1] = extrinsicsToCameraPose(quat2rotm(pose1.q)',pose1.t);
    [Rc2,tc2] = extrinsicsToCameraPose(quat2rotm(pose2.q)',pose2.t);
    
    % Re, te: extrinsics
    [Re1, te1] = cameraPoseToExtrinsics(Rc1, tc1);
    [Re2, te2] = cameraPoseToExtrinsics(Rc2, tc2);
    fail_counter = 0;
    
    for experiment_idx = 1:experiment_number
        try
            % detecting ORB features
            [fps1, discriptors1] = detectFeature(img1, image_grid, max_np);
            [fps2, discriptors2] = detectFeature(img2, image_grid, max_np);
            indexPairs12 = matchFeatures(discriptors1, discriptors2, 'MaxRatio', .7, 'Unique',  true);
            % Select matched points.
            mp1 = fps1(indexPairs12(:, 1),:);
            mp2 = fps2(indexPairs12(:, 2),:);
            [orient2, loc2, inlierIdx2] = CamAdjFindRelativePose(mp1, mp2, cameraParams, cameraParams, MSAC_confidence);
            
            T1 = [Rc1', (-tc1*Rc1')'; 0 0 0 1];
            T2 = [Rc2', (-tc2*Rc2')'; 0 0 0 1];
            T21 = CamAdjTransformInverse(T2) * T1;
            inlier_refined = CamAdjFiltering_R8(mp1', mp2', T21(1:3,1:3), ...
                T21(1:3,4), K, CamAdj_para);
            
            mp1_refine = mp1(inlier_refined,:);
            mp2_refine = mp2(inlier_refined,:);

            [orient2_refine, loc2_refine, inlierIdx2] = CamAdjFindRelativePose(mp1_refine, mp2_refine, cameraParams, cameraParams, MSAC_confidence);
            p_reject = (length(mp2) - length(mp2_refine))/length(mp2)*100;
            if p_reject > 1 && p_reject <= 25 && sum(inlierIdx2) >= minimum_points
                reject_percentage = [reject_percentage; p_reject];
                R_error = sqrt(sum((cayley_R2c(orient2) - cayley_R2c(T21(1:3,1:3))).^2));
                CamAdj_R_error = sqrt(sum((cayley_R2c(orient2_refine) - cayley_R2c(T21(1:3,1:3))).^2));
                R_error = min(R_error, 0.2679); % 0.2679 in cayley is 30 degree in euler
                CamAdj_R_error = min(CamAdj_R_error, 0.2679);
                no_CamAdj_re = [no_CamAdj_re; R_error];
                CamAdj_re = [CamAdj_re; CamAdj_R_error];
                Better_R_percent = length(find(CamAdj_re < no_CamAdj_re))/length(no_CamAdj_re)*100;
                figure(1), hold on;
                plot(no_CamAdj_re,'b');
                plot(CamAdj_re,'r');
                hold off;
                legend(['MSAC, mean=', num2str(mean(no_CamAdj_re))],...
                    ['CamAdj+MSAC, mean=',num2str(mean(CamAdj_re))]);
                title(['Rotation estimation error',', better R rate=', num2str(Better_R_percent), '%']);
                ylabel('MSE in cayley');
                xlabel('number of experiments');
                fail_counter = 0;
            else
                disp('insufficient inliers');
            end
            
        catch
            fail_counter = fail_counter + 1;
            if fail_counter > 5
                break;
            end
        end
        
    end
end

%% save data
data.no_CamAdj_re = no_CamAdj_re;
data.CamAdj_re = CamAdj_re;
data.reject_percentage = reject_percentage;
save('./results/CamAdj_2views_results.mat','data');
disp('save result to ./results');
%% plot AUC
% pi/180 euler = 0.0087 cayley
% pi/2 euler = 1 cayley
threshold = 0.001:0.0001:0.0087;
AUC_no_CamAdj = generate_AUC(no_CamAdj_re, threshold);
AUC_CamAdj = generate_AUC(CamAdj_re, threshold);
figure;
hold on;
plot(threshold,AUC_no_CamAdj,'b','LineWidth',2);
plot(threshold,AUC_CamAdj,'r','LineWidth',2);
hold off;
grid on;
legend('MSAC','CamAdj+MSAC')
title('Rotation estimation error AUC (two view)');
ylabel('Proportion');
xlabel('Error threshold in Cayley');

rmpath(genpath('./functions'));