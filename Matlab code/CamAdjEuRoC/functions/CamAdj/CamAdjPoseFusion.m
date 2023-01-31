function T_uav = CamAdjPoseFusion(T_uav0, T_uav1, outlier_percent0, outlier_percent1)
T_uav = eye(4);
if outlier_percent0 < 0.5 && outlier_percent1 > 0.5
    T_uav = T_uav0;
elseif outlier_percent0 > 0.5 && outlier_percent1 < 0.5
    T_uav = T_uav1;
else
    c0 = cayley_R2c(T_uav0(1:3,1:3));
    c1 = cayley_R2c(T_uav1(1:3,1:3));
    s = outlier_percent0 + outlier_percent1;
    if s == 0
        c = (c0 + c1).*0.5;
        t = (T_uav0(1:3,4) + T_uav1(1:3,4)).*0.5;
        T_uav(1:3,1:3) = cayley_c2R(c);
        T_uav(1:3,4) = t;
    else
        w0 = (s - outlier_percent0)/s;
        w1 = 1 - w0;
        c = w0 * c0 + w1 * c1;
        t = w0 * T_uav0(1:3,4) + w1 * T_uav1(1:3,4);
        T_uav(1:3,1:3) = cayley_c2R(c);
        T_uav(1:3,4) = t;
    end
end
return