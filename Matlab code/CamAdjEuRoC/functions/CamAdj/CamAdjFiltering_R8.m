function singular_idx = CamAdjFiltering_R8(point2Du, point2Dup, orientation, location, sos_para)

[ub, lb] = CamAdjGetBoundary_R8(orientation, location, sos_para(1), sos_para(2), sos_para(3), sos_para(4), sos_para(5), sos_para(6));
infeasible_index = CamAdjLMIsFeasibleCheck_R8(point2Du, point2Dup, lb, ub);
singular_idx = find(infeasible_index == 0);

return