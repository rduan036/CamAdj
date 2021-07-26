function [orient, loc] = CamAdjVisualServoing(target_point2D, points3D, orient, loc, K)

backup_orient = orient;
backup_loc = loc;

t = (-loc*orient')';

eul = rotm2eul(orient, 'XYZ');
[u, theta, ~] = euler2quat(eul);
u = u/norm(u);
dq_current = uthetat2dq(u, theta, t);
depths = points3D(3,:);
lamda = 1;

if size(target_point2D,1) == 2
    target_point2D(3,:) = 1;
end

target_point2D = pinv(K) * target_point2D;
target_point2D = target_point2D./repmat(target_point2D(3,:),3,1);

dt = 1; % control sampling time
dq_error = 1;
iter = 1;
while dq_error > 0.000001 && iter < 7

    PointsReproject = (orient * points3D + repmat(t,1,length(points3D)));
    PointsReproject = PointsReproject./repmat(PointsReproject(3,:),3,1);
%     figure(4);
%     clf;
%     hold on;
%     plot(target_point2D(1,:), target_point2D(2,:), 'bo');
%     plot(PointsReproject(1,:), PointsReproject(2,:), 'r+');
%     hold off;
    % get jacobian
    L = image_jacobian(PointsReproject, depths);
    % get error vector
    s_target = pattern_pix2metric(target_point2D);
    s_current = pattern_pix2metric(PointsReproject);
    e = s_target - s_current;
    % compute control law
    T_control = -lamda.*pinv(L)*e;
    T_current = [orient, skew(t)*orient;
                zeros(3,3), orient];
    T_new = T_current * T_control;
    % move camera
    v = T_new(1:3);
    w = T_new(4:6);
    theta = norm(w);
    if( theta == 0 ) u = [0;0;1]; else u = w/norm(w); end
    dq_update = uthetat2dq( u, dt*theta, dt*v );
    new_dq = muldualpq( dq_update, dq_current );

    [ u, theta, orient, t ] = dualq2uthetaRt( new_dq );
%     sum(abs(e))
    dq_error = sum(abs(dq_current - new_dq));
    dq_current = new_dq;
    iter = iter + 1;
%     dt = 0.9*dt;
    lamda = max(0.9*lamda, 0.5);
end

loc = -t'*orient;
if isnan(orient)
    orient = backup_orient ;
    loc = backup_loc;
end

return