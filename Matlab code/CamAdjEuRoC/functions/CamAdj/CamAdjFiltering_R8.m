function inlier_refined = CamAdjFiltering_R8(point2Du, point2Dup, orientation, location, K, sos_para)

[ub, lb] = CamAdjGetBoundary_R8(orientation, location, sos_para(1), sos_para(2), sos_para(3), sos_para(4), sos_para(5), sos_para(6));
point2Du(3,:) = 1;
point2Du = pinv(K) * point2Du;
point2Du = point2Du./repmat(point2Du(3,:),3,1);
point2Dup(3,:) = 1;
point2Dup = pinv(K) * point2Dup;
point2Dup = point2Dup./repmat(point2Dup(3,:),3,1);
infeasible_index = CamAdjLMIsFeasibleCheck_R8(point2Du, point2Dup, lb, ub);
inlier_refined = find(infeasible_index == 1);

return