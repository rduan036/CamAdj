% to check the pose of camera 1:
% T_c1c0 = [pre_orient, pre_loc; 0 0 0 1]
function [ub, lb] = CamAdjGetBoundary(pre_orient, pre_loc, r_var, t_var)
    x_var = ones(6,1);
    x_var(1:3) = r_var .* x_var(1:3);
    x_var(4:6) = t_var .* x_var(4:6);
    c_pre = cayley_R2c(pre_orient);
    t_pre = (eye(3) - skew(c_pre)) * pre_loc;
    x = [c_pre; t_pre];
    ub = x + x_var;
    lb = x - x_var;
return