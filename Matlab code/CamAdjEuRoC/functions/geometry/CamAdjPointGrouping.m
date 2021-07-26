function [servoPoint1, servoPoint2, servoPoint3D, weights] = CamAdjPointGrouping(inlierPoints1, inlierPoints2, point3D, img, Num)

[m, n] = size(img);
h = m/Num;
w = n/Num;
r = max(h,w);

[x, y] = meshgrid(w/2:w:n, h/2:h:m);
samples = [x(:), y(:)];

Num = Num^2;

servoPoint1 = zeros(Num,2);
servoPoint2 = zeros(Num,2);
servoPoint3D = zeros(Num,3);
weights = zeros(Num,1);

for ii = 1:Num
    x_min = samples(ii,1) - r;
    x_max = samples(ii,1) + r;
    y_min = samples(ii,2) - r;
    y_max = samples(ii,2) + r;
    idx = find(inlierPoints1(:,1) > x_min & inlierPoints1(:,1) < x_max & inlierPoints1(:,2) > y_min & inlierPoints1(:,2) < y_max);
    weights(ii) = length(idx);
    if weights(ii) > 0
        servoPoint1(ii,:) = mean(inlierPoints1(idx,:));
        servoPoint2(ii,:) = mean(inlierPoints2(idx,:));
        servoPoint3D(ii,:) = mean(point3D(idx,:));
    end
end

return

% ptCloud = pointCloud(point3D);
% [] = pcfitplane()
