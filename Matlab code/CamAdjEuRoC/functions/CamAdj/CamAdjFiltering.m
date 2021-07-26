function rejection_idx = CamAdjFiltering(point2D, points3D, orientation, location, K, sos_para)

[ub, lb] = CamAdjGetBoundary(orientation, location, sos_para(1), sos_para(2));
point2D(3,:) = 1;
point2D = pinv(K) * point2D;
point2D = point2D./repmat(point2D(3,:),3,1);
infeasible_index = CamAdjLMIsFeasibleCheck(points3D, point2D, lb, ub);
rejection_idx = find(infeasible_index == 0);

return