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
N_of_point = 200;
idx_of_noise_free = 1:50;
idx_of_general_error = 51:100;
idx_of_rotation_error = 101:150;
idx_of_translation_error = 151:200;

Xw = rand(3,N_of_point);
Xw(1:2,:) = 10.*Xw(1:2,:);
Xw(3,:) = 5.*ones(1,N_of_point) + 2.*Xw(3,:);

% inliers
T = [R,t; 0 0 0 1];
uv_gt = K*P*T*[Xw;ones(1,N_of_point)];
uv_gt = uv_gt./repmat(uv_gt(3,:),3,1);
uv_gt = uv_gt(1:2,:);
uv = uv_gt(1:2,idx_of_noise_free);

for ii = 1:length(idx_of_general_error)
    jj = 1;
    while jj < 10000 
        eul_noisy = eul + R_error.*(rand(1,3) - rand(1,3));
        R_with_noise = eul2rotm(eul_noisy);
        t_with_noise = t + t_error*(rand(3,1) - rand(3,1));
        T_with_noise = [R_with_noise,t_with_noise; 0 0 0 1];
        uv_with_noise = K*P*T_with_noise*[Xw(:,idx_of_general_error(ii));1];
        uv_with_noise = uv_with_noise./uv_with_noise(3,:);
        uv_with_noise = uv_with_noise(1:2,:);
        reprojection_error_general = sqrt((uv_with_noise - uv_gt(:,idx_of_general_error(ii))).^2);
        if reprojection_error_general <= 5
            uv = [uv, uv_with_noise];
            break;
        end
        jj = jj + 1;
    end
    if jj == 10000
        disp('need change error threshold...');
    end
end

for ii = 1:length(idx_of_rotation_error)
    jj = 1;
    while jj < 10000
        eul_noisy = eul + R_error.*(rand(1,3) - rand(1,3));
        R_with_noise = eul2rotm(eul_noisy);
        T_with_noise = [R_with_noise,t; 0 0 0 1];
        uv_with_noise = K*P*T_with_noise*[Xw(:,idx_of_rotation_error(ii));1];
        uv_with_noise = uv_with_noise./uv_with_noise(3,:);
        uv_with_noise = uv_with_noise(1:2,:);
        reprojection_error_rotation = sqrt((uv_with_noise - uv_gt(:,idx_of_rotation_error(ii))).^2);
        if reprojection_error_rotation <= 5
            uv = [uv, uv_with_noise];
            break;
        end
        jj = jj + 1;
    end
    if jj == 10000
        disp('need change error threshold...');
    end
end

for ii = 1:length(idx_of_translation_error)
    jj = 1;
    while jj < 10000
        t_with_noise = t + t_error*(rand(3,1) - rand(3,1));
        T_with_noise = [R,t_with_noise; 0 0 0 1];
        uv_with_noise = K*P*T_with_noise*[Xw(:,idx_of_translation_error(ii));1];
        uv_with_noise = uv_with_noise./uv_with_noise(3,:);
        uv_with_noise = uv_with_noise(1:2,:);
        reprojection_error_translation = sqrt((uv_with_noise - uv_gt(:,idx_of_translation_error(ii))).^2);
        if reprojection_error_translation <= 5
            uv = [uv, uv_with_noise];
            break;
        end
        jj = jj + 1;
        if jj == 10000
            disp('need change error threshold...');
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MSAC rejection
disp('==================================================================');
disp('error free: x = K(RX + t)');
disp('general error: x = K((R+e)X + (t+e))');
disp('R error: x = K((R+e)X + t)');
disp('t error: x = K(RX + (t+e))');
disp('==================================================================');
disp('case 0: reprojection error + MSAC');
% [orient, loc, inlierIdx] = estimateWorldCameraPose(uv', Xw', cameraParams);
% reject0 = 100 - sum(inlierIdx(1:50))/50*100;
% reject1 = 100 - sum(inlierIdx(51:100))/50*100;
% reject2 = 100 - sum(inlierIdx(101:150))/50*100;
% reject3 = 100 - sum(inlierIdx(151:200))/50*100;
% R_error = sqrt(sum((rotm2eul(orient').*180./pi - rotm2eul(R).*180./pi).^2));
% t_error = sqrt(sum(((-loc*orient')' - t).^2));
% disp(['MSAC rejection [error free, general error, R error, t error]:[', ...
%     num2str(reject0),'%,', num2str(reject1),'%,',num2str(reject2),'%,',num2str(reject3),'%]']);
% disp(['pose error [rotation, translation]: [', num2str(R_error),',', num2str(t_error), ']']);
[orient, loc, inlierIdx] = estimateWorldCameraPose(uv(:,51:end)', Xw(:,51:end)', cameraParams);
inlierIdx = [zeros(50,1); inlierIdx];
reject1 = 100 - sum(inlierIdx(51:100))/50*100;
reject2 = 100 - sum(inlierIdx(101:150))/50*100;
reject3 = 100 - sum(inlierIdx(151:200))/50*100;
R_error = sqrt(sum((rotm2eul(orient').*180./pi - rotm2eul(R).*180./pi).^2));
t_error = sqrt(sum(((-loc*orient')' - t).^2));
disp('Error free points do not exist in practice, test on all 3 error groups:');
disp(['MSAC rejection [general error, R error, t error]:[', ...
    num2str(reject1),'%,',num2str(reject2),'%,',num2str(reject3),'%]']);
disp(['pose error [rotation, translation]: [', num2str(R_error),',', num2str(t_error), ']']);

idx_of_general_and_R = [idx_of_general_error, idx_of_rotation_error];
[orient, loc, inlierIdx] = estimateWorldCameraPose(uv(:,idx_of_general_and_R)', Xw(:,idx_of_general_and_R)', cameraParams);
reject = 100 - sum(inlierIdx);
R_error = sqrt(sum((rotm2eul(orient').*180./pi - rotm2eul(R).*180./pi).^2));
t_error = sqrt(sum(((-loc*orient')' - t).^2));
disp('remove t error group:');
disp(['MSAC rejection:', ...
    num2str(reject),'%,']);
disp(['pose error [rotation, translation]: [', num2str(R_error),',', num2str(t_error), ']']);

idx_of_general_and_t = [idx_of_general_error, idx_of_translation_error];
[orient, loc, inlierIdx] = estimateWorldCameraPose(uv(:,idx_of_general_and_t)', Xw(:,idx_of_general_and_t)', cameraParams);
reject = 100 - sum(inlierIdx);
R_error = sqrt(sum((rotm2eul(orient').*180./pi - rotm2eul(R).*180./pi).^2));
t_error = sqrt(sum(((-loc*orient')' - t).^2));
disp('remove R error group:');
disp(['MSAC rejection:', ...
    num2str(reject),'%,']);
disp(['pose error [rotation, translation]: [', num2str(R_error),',', num2str(t_error), ']']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CamAdj rejection
disp('==================================================================');
disp('case 1: reduce CamAdj bounds on both rotation and translation');

CamAdj_para = [0.01, 0.01];
inlier_refined_case1 = CamAdjFiltering(uv, Xw, R, ...
                                            t, K, CamAdj_para);
reject0 = 100 - length(find(inlier_refined_case1 < 51))/50*100;
reject1 = 100 - length(find(inlier_refined_case1 > 50 & inlier_refined_case1 < 100))/50*100;
reject2 = 100 - length(find(inlier_refined_case1 > 100 & inlier_refined_case1 < 151))/50*100;
reject3 = 100 - length(find(inlier_refined_case1 > 150))/50*100;
disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
disp(['CamAdj rejection [error free, general error, R error, t error]:[', ...
    num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);

CamAdj_para = [0.001, 0.001];
inlier_refined_case1 = CamAdjFiltering(uv, Xw, R, ...
                                            t, K, CamAdj_para);
reject0 = 100 - length(find(inlier_refined_case1 < 51))/50*100;
reject1 = 100 - length(find(inlier_refined_case1 > 50 & inlier_refined_case1 < 100))/50*100;
reject2 = 100 - length(find(inlier_refined_case1 > 100 & inlier_refined_case1 < 151))/50*100;
reject3 = 100 - length(find(inlier_refined_case1 > 150))/50*100;
disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
disp(['CamAdj rejection [error free, general error, R error, t error]:[', ...
    num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);

CamAdj_para = [0.0005, 0.0005];
inlier_refined_case1 = CamAdjFiltering(uv, Xw, R, ...
                                            t, K, CamAdj_para);
reject0 = 100 - length(find(inlier_refined_case1 < 51))/50*100;
reject1 = 100 - length(find(inlier_refined_case1 > 50 & inlier_refined_case1 < 100))/50*100;
reject2 = 100 - length(find(inlier_refined_case1 > 100 & inlier_refined_case1 < 151))/50*100;
reject3 = 100 - length(find(inlier_refined_case1 > 150))/50*100;
disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
disp(['CamAdj rejection [error free, general error, R error, t error]:[', ...
    num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);

CamAdj_para = [0.0001, 0.0001];
inlier_refined_case1 = CamAdjFiltering(uv, Xw, R, ...
                                            t, K, CamAdj_para);
reject0 = 100 - length(find(inlier_refined_case1 < 51))/50*100;
reject1 = 100 - length(find(inlier_refined_case1 > 50 & inlier_refined_case1 < 100))/50*100;
reject2 = 100 - length(find(inlier_refined_case1 > 100 & inlier_refined_case1 < 151))/50*100;
reject3 = 100 - length(find(inlier_refined_case1 > 150))/50*100;
disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
disp(['CamAdj rejection [error free, general error, R error, t error]:[', ...
    num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CamAdj rejection
disp('==================================================================');
disp('case 2: reduce CamAdj bounds on translation only');
CamAdj_para = [0.001, 0.00001];
inlier_refined_case2 = CamAdjFiltering(uv, Xw, R, ...
                                            t, K, CamAdj_para);
reject0 = 100 - length(find(inlier_refined_case2 < 51))/50*100;
reject1 = 100 - length(find(inlier_refined_case2 > 50 & inlier_refined_case2 < 100))/50*100;
reject2 = 100 - length(find(inlier_refined_case2 > 100 & inlier_refined_case2 < 151))/50*100;
reject3 = 100 - length(find(inlier_refined_case2 > 150))/50*100;
disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
disp(['CamAdj rejection [error free, general error, R error, t error]:[', ...
    num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);

CamAdj_para = [0.001, 0.000001];
inlier_refined_case2 = CamAdjFiltering(uv, Xw, R, ...
                                            t, K, CamAdj_para);
reject0 = 100 - length(find(inlier_refined_case2 < 51))/50*100;
reject1 = 100 - length(find(inlier_refined_case2 > 50 & inlier_refined_case2 < 100))/50*100;
reject2 = 100 - length(find(inlier_refined_case2 > 100 & inlier_refined_case2 < 151))/50*100;
reject3 = 100 - length(find(inlier_refined_case2 > 150))/50*100;
disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
disp(['CamAdj rejection [error free, general error, R error, t error]:[', ...
    num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);

CamAdj_para = [0.001, 0.0000001];
inlier_refined_case2 = CamAdjFiltering(uv, Xw, R, ...
                                            t, K, CamAdj_para);
reject0 = 100 - length(find(inlier_refined_case2 < 51))/50*100;
reject1 = 100 - length(find(inlier_refined_case2 > 50 & inlier_refined_case2 < 100))/50*100;
reject2 = 100 - length(find(inlier_refined_case2 > 100 & inlier_refined_case2 < 151))/50*100;
reject3 = 100 - length(find(inlier_refined_case2 > 150))/50*100;
disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
disp(['CamAdj rejection [error free, general error, R error, t error]:[', ...
    num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CamAdj rejection
disp('==================================================================');
disp('case 3: reduce CamAdj bounds on rotation only');
CamAdj_para = [0.0005, 0.001];
inlier_refined_case3 = CamAdjFiltering(uv, Xw, R, ...
                                            t, K, CamAdj_para);
reject0 = 100 - length(find(inlier_refined_case3 < 51))/50*100;
reject1 = 100 - length(find(inlier_refined_case3 > 50 & inlier_refined_case3 < 100))/50*100;
reject2 = 100 - length(find(inlier_refined_case3 > 100 & inlier_refined_case3 < 151))/50*100;
reject3 = 100 - length(find(inlier_refined_case3 > 150))/50*100;
disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
disp(['CamAdj rejection [error free, general error, R error, t error]:[', ...
    num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);

CamAdj_para = [0.0001, 0.001];
inlier_refined_case3 = CamAdjFiltering(uv, Xw, R, ...
                                            t, K, CamAdj_para);
reject0 = 100 - length(find(inlier_refined_case3 < 51))/50*100;
reject1 = 100 - length(find(inlier_refined_case3 > 50 & inlier_refined_case3 < 100))/50*100;
reject2 = 100 - length(find(inlier_refined_case3 > 100 & inlier_refined_case3 < 151))/50*100;
reject3 = 100 - length(find(inlier_refined_case3 > 150))/50*100;
disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
disp(['CamAdj rejection [error free, general error, R error, t error]:[', ...
    num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);

CamAdj_para = [0.00005, 0.001];
inlier_refined_case3 = CamAdjFiltering(uv, Xw, R, ...
                                            t, K, CamAdj_para);
reject0 = 100 - length(find(inlier_refined_case3 < 51))/50*100;
reject1 = 100 - length(find(inlier_refined_case3 > 50 & inlier_refined_case3 < 100))/50*100;
reject2 = 100 - length(find(inlier_refined_case3 > 100 & inlier_refined_case3 < 151))/50*100;
reject3 = 100 - length(find(inlier_refined_case3 > 150))/50*100;
disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
disp(['CamAdj rejection [error free, general error, R error, t error]:[', ...
    num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);

CamAdj_para = [0.00001, 0.001];
inlier_refined_case3 = CamAdjFiltering(uv, Xw, R, ...
                                            t, K, CamAdj_para);
reject0 = 100 - length(find(inlier_refined_case3 < 51))/50*100;
reject1 = 100 - length(find(inlier_refined_case3 > 50 & inlier_refined_case3 < 100))/50*100;
reject2 = 100 - length(find(inlier_refined_case3 > 100 & inlier_refined_case3 < 151))/50*100;
reject3 = 100 - length(find(inlier_refined_case3 > 150))/50*100;
disp(['CamAdj bounds [R t]: [', num2str(CamAdj_para(1)),',', num2str(CamAdj_para(2)),']']);
disp(['CamAdj rejection [error free, general error, R error, t error]:[', ...
    num2str(reject0),'% ', num2str(reject1),'% ',num2str(reject2),'% ',num2str(reject3),'%]']);

rmpath(genpath('./functions'));

