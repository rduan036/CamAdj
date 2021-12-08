%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% demo for paper Table 1

clc;
close all;
clear all;

addpath(genpath('./functions'));

% generate synthetic camera
fx = 800; % focal length (mm) / pixel_width_x (mm) = pixels
fy = 800; % focal length (mm) / pixel_height_y (mm) = pixels
cx = 1024; % image center x-coordinate
cy = 1024; % image center y-coordinate
K = [ fx, 0, cx;
    0, fy, cy;
    0,  0,  1 ];

cameraParams = cameraParameters('IntrinsicMatrix',K');

eul = [0 0 0];
R = eul2rotm(eul);
% R = eye(3);
eul = rotm2eul(R).*180./pi;
t = zeros(3,1);
R_error = pi/180;
t_error = 0.05;

% generate 3D-2D corresponds
P = [1,0,0,0;
    0,1,0,0;
    0,0,1,0];
N_of_point = 2000;
idx_of_pure_inlier = 1:500;
idx_of_general_error = 501:1000;
idx_of_rotation_error = 1001:1500;
idx_of_translation_error = 1501:2000;

Xw = rand(3,N_of_point);
Xw(1:2,:) = 10.*Xw(1:2,:);
Xw(3,:) = 5.*ones(1,N_of_point) + 2.*Xw(3,:);

T = [R,t; 0 0 0 1];
uv_gt = K*P*T*[Xw;ones(1,N_of_point)];
uv_gt = uv_gt./repmat(uv_gt(3,:),3,1);
uv_gt = uv_gt(1:2,:);
uv = [];
inlier_reprojection_error = 0.5; % outherwise definitely outlier
reprojection_error_table = [];
max_propose = 10000; % for point generation
point_generation_flag = 1;
min_delta_R = 0.02;
min_delta_t = 0.01;

for ii = 1:length(idx_of_pure_inlier)
    jj = 1;
    while jj < max_propose
        eul_noisy = eul + (min_delta_R/2).*R_error.*(rand(1,3) - rand(1,3));%(-2,2)
        R_with_noise = eul2rotm(eul_noisy);
        t_with_noise = t + (min_delta_t/2).*t_error*(rand(3,1) - rand(3,1));
        T_with_noise = [R_with_noise,t_with_noise; 0 0 0 1];
        uv_with_noise = K*P*T_with_noise*[Xw(:,idx_of_pure_inlier(ii));1];
        uv_with_noise = uv_with_noise./uv_with_noise(3,:);
        uv_with_noise = uv_with_noise(1:2,:);
        reprojection_error_inlier = sqrt(sum((uv_with_noise - uv_gt(:,idx_of_pure_inlier(ii))).^2));
        if reprojection_error_inlier < inlier_reprojection_error
            uv = [uv, uv_with_noise];
            reprojection_error_table = [reprojection_error_table, reprojection_error_inlier];
            break;
        end
        jj = jj + 1;
    end
    if jj == max_propose
        point_generation_flag = 0;
        disp('fail to generate point...');
        break;
    end
end

for ii = 1:length(idx_of_general_error)
    jj = 1;
    while jj < max_propose
        delta_R = (rand(1,3) - rand(1,3));
        delta_t = (rand(3,1) - rand(3,1));
        if mean(abs(delta_R)) > min_delta_R && mean(abs(delta_t)) > min_delta_t
            eul_noisy = eul + R_error.*delta_R;
            R_with_noise = eul2rotm(eul_noisy);
            t_with_noise = t + t_error*delta_t;
            T_with_noise = [R_with_noise,t_with_noise; 0 0 0 1];
            uv_with_noise = K*P*T_with_noise*[Xw(:,idx_of_general_error(ii));1];
            uv_with_noise = uv_with_noise./uv_with_noise(3,:);
            uv_with_noise = uv_with_noise(1:2,:);
            reprojection_error_general = sqrt(sum((uv_with_noise - uv_gt(:,idx_of_general_error(ii))).^2));
            if reprojection_error_general < 4*inlier_reprojection_error
                uv = [uv, uv_with_noise];
                reprojection_error_table = [reprojection_error_table, reprojection_error_general];
                break;
            end
        end
        jj = jj + 1;
    end
    if jj == max_propose
        point_generation_flag = 0;
        disp('fail to generate point...');
        break;
    end
end

for ii = 1:length(idx_of_rotation_error)
    jj = 1;
    while jj < max_propose
        delta_R = (rand(1,3) - rand(1,3));
        if mean(abs(delta_R)) > min_delta_R
            eul_noisy = eul + R_error.*delta_R;
            R_with_noise = eul2rotm(eul_noisy);
            T_with_noise = [R_with_noise,t; 0 0 0 1];
            uv_with_noise = K*P*T_with_noise*[Xw(:,idx_of_rotation_error(ii));1];
            uv_with_noise = uv_with_noise./uv_with_noise(3,:);
            uv_with_noise = uv_with_noise(1:2,:);
            reprojection_error_rotation = sqrt(sum((uv_with_noise - uv_gt(:,idx_of_rotation_error(ii))).^2));
            if reprojection_error_rotation < 4*inlier_reprojection_error
                uv = [uv, uv_with_noise];
                reprojection_error_table = [reprojection_error_table, reprojection_error_rotation];
                break;
            end
        end
        jj = jj + 1;
    end
    if jj == max_propose
        point_generation_flag = 0;
        disp('fail to generate point...');
        break;
    end
end

for ii = 1:length(idx_of_translation_error)
    jj = 1;
    while jj < max_propose
        delta_t = (rand(3,1) - rand(3,1));
        if mean(abs(delta_t)) > min_delta_t
            t_with_noise = t + t_error*delta_t;
            T_with_noise = [R,t_with_noise; 0 0 0 1];
            uv_with_noise = K*P*T_with_noise*[Xw(:,idx_of_translation_error(ii));1];
            uv_with_noise = uv_with_noise./uv_with_noise(3,:);
            uv_with_noise = uv_with_noise(1:2,:);
            reprojection_error_translation = sqrt(sum((uv_with_noise - uv_gt(:,idx_of_translation_error(ii))).^2));
            if reprojection_error_translation < 4*inlier_reprojection_error
                uv = [uv, uv_with_noise];
                reprojection_error_table = [reprojection_error_table, reprojection_error_translation];
                break;
            end
        end
        jj = jj + 1;
    end
    if jj == max_propose
        point_generation_flag = 0;
        disp('fail to generate point...');
        break;
    end
end

if point_generation_flag
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % MSAC rejection
    disp('==================================================================');
    disp('inliers: x = K(RX + t)');
    disp('general error: x = K((R+e)X + (t+e))');
    disp('R error: x = K((R+e)X + t)');
    disp('t error: x = K(RX + (t+e))');
    disp('==================================================================');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % CamAdj rejection
    disp('==================================================================');
    disp('case 1: reduce CamAdj bounds on both rotation and translation');
    
    CamAdj_para = [0.0002, 0.0002];
    inliers_idx = CamAdjFiltering(uv, Xw, R, ...
        t, K, CamAdj_para);
    reject0 = 100 - length(find(inliers_idx < idx_of_general_error(1)))/length(idx_of_pure_inlier)*100;
    reject1 = 100 - length(find(inliers_idx >= idx_of_general_error(1) & inliers_idx < idx_of_rotation_error(1)))/length(idx_of_general_error)*100;
    reject2 = 100 - length(find(inliers_idx >= idx_of_rotation_error(1) & inliers_idx < idx_of_translation_error(1)))/length(idx_of_rotation_error)*100;
    reject3 = 100 - length(find(inliers_idx >= idx_of_translation_error(1)))/length(idx_of_translation_error)*100;
    disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
    disp(['CamAdj rejection [inliers, general error, R error, t error]:[', ...
        num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);
    
    CamAdj_para = [0.0001, 0.0001];
    inliers_idx = CamAdjFiltering(uv, Xw, R, ...
        t, K, CamAdj_para);
    reject0 = 100 - length(find(inliers_idx < idx_of_general_error(1)))/length(idx_of_pure_inlier)*100;
    reject1 = 100 - length(find(inliers_idx >= idx_of_general_error(1) & inliers_idx < idx_of_rotation_error(1)))/length(idx_of_general_error)*100;
    reject2 = 100 - length(find(inliers_idx >= idx_of_rotation_error(1) & inliers_idx < idx_of_translation_error(1)))/length(idx_of_rotation_error)*100;
    reject3 = 100 - length(find(inliers_idx >= idx_of_translation_error(1)))/length(idx_of_translation_error)*100;
    disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
    disp(['CamAdj rejection [inliers, general error, R error, t error]:[', ...
        num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % CamAdj rejection
    disp('==================================================================');
    disp('case 2: reduce CamAdj bounds on translation only');
    CamAdj_para = [0.0002, 0.0000001];
    inliers_idx = CamAdjFiltering(uv, Xw, R, ...
        t, K, CamAdj_para);
    reject0 = 100 - length(find(inliers_idx < idx_of_general_error(1)))/length(idx_of_pure_inlier)*100;
    reject1 = 100 - length(find(inliers_idx >= idx_of_general_error(1) & inliers_idx < idx_of_rotation_error(1)))/length(idx_of_general_error)*100;
    reject2 = 100 - length(find(inliers_idx >= idx_of_rotation_error(1) & inliers_idx < idx_of_translation_error(1)))/length(idx_of_rotation_error)*100;
    reject3 = 100 - length(find(inliers_idx >= idx_of_translation_error(1)))/length(idx_of_translation_error)*100;
    disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
    disp(['CamAdj rejection [inliers, general error, R error, t error]:[', ...
        num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']); 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % CamAdj rejection
    disp('==================================================================');
    disp('case 3: reduce CamAdj bounds on rotation only');   
    CamAdj_para = [0.00005, 0.0001];
    inliers_idx = CamAdjFiltering(uv, Xw, R, ...
        t, K, CamAdj_para);
    reject0 = 100 - length(find(inliers_idx < idx_of_general_error(1)))/length(idx_of_pure_inlier)*100;
    reject1 = 100 - length(find(inliers_idx >= idx_of_general_error(1) & inliers_idx < idx_of_rotation_error(1)))/length(idx_of_general_error)*100;
    reject2 = 100 - length(find(inliers_idx >= idx_of_rotation_error(1) & inliers_idx < idx_of_translation_error(1)))/length(idx_of_rotation_error)*100;
    reject3 = 100 - length(find(inliers_idx >= idx_of_translation_error(1)))/length(idx_of_translation_error)*100;
    disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
    disp(['CamAdj rejection [inliers, general error, R error, t error]:[', ...
        num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);
    
    CamAdj_para = [0.00003, 0.0001];
    inliers_idx = CamAdjFiltering(uv, Xw, R, ...
        t, K, CamAdj_para);
    reject0 = 100 - length(find(inliers_idx < idx_of_general_error(1)))/length(idx_of_pure_inlier)*100;
    reject1 = 100 - length(find(inliers_idx >= idx_of_general_error(1) & inliers_idx < idx_of_rotation_error(1)))/length(idx_of_general_error)*100;
    reject2 = 100 - length(find(inliers_idx >= idx_of_rotation_error(1) & inliers_idx < idx_of_translation_error(1)))/length(idx_of_rotation_error)*100;
    reject3 = 100 - length(find(inliers_idx >= idx_of_translation_error(1)))/length(idx_of_translation_error)*100;
    disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
    disp(['CamAdj rejection [inliers, general error, R error, t error]:[', ...
        num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % rejection rate using reprojection error
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % threshold = 1
    error_threshold = 1;
    error_list = reprojection_error_table(idx_of_pure_inlier);
    reject0 = length(find(error_list > error_threshold))/length(idx_of_pure_inlier)*100;
    error_list = reprojection_error_table(idx_of_general_error);
    reject1 = length(find(error_list > error_threshold))/length(idx_of_general_error)*100;
    error_list = reprojection_error_table(idx_of_rotation_error);
    reject2 = length(find(error_list > error_threshold))/length(idx_of_rotation_error)*100;
    error_list = reprojection_error_table(idx_of_translation_error);
    reject3 = length(find(error_list > error_threshold))/length(idx_of_translation_error)*100;
    disp(['Reprojection error threshold = ', num2str(error_threshold)]);
    disp(['Reprojection error rejection [inliers, general error, R error, t error]:[', ...
        num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);
    
    error_threshold = 0.5;
    error_list = reprojection_error_table(idx_of_pure_inlier);
    reject0 = length(find(error_list > error_threshold))/length(idx_of_pure_inlier)*100;
    error_list = reprojection_error_table(idx_of_general_error);
    reject1 = length(find(error_list > error_threshold))/length(idx_of_general_error)*100;
    error_list = reprojection_error_table(idx_of_rotation_error);
    reject2 = length(find(error_list > error_threshold))/length(idx_of_rotation_error)*100;
    error_list = reprojection_error_table(idx_of_translation_error);
    reject3 = length(find(error_list > error_threshold))/length(idx_of_translation_error)*100;
    disp(['Reprojection error threshold = ', num2str(error_threshold)]);
    disp(['Reprojection error rejection [inliers, general error, R error, t error]:[', ...
        num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);
    
    error_threshold = 0.3;
    error_list = reprojection_error_table(idx_of_pure_inlier);
    reject0 = length(find(error_list > error_threshold))/length(idx_of_pure_inlier)*100;
    error_list = reprojection_error_table(idx_of_general_error);
    reject1 = length(find(error_list > error_threshold))/length(idx_of_general_error)*100;
    error_list = reprojection_error_table(idx_of_rotation_error);
    reject2 = length(find(error_list > error_threshold))/length(idx_of_rotation_error)*100;
    error_list = reprojection_error_table(idx_of_translation_error);
    reject3 = length(find(error_list > error_threshold))/length(idx_of_translation_error)*100;
    disp(['Reprojection error threshold = ', num2str(error_threshold)]);
    disp(['Reprojection error rejection [inliers, general error, R error, t error]:[', ...
        num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);
end

rmpath(genpath('./functions'));

