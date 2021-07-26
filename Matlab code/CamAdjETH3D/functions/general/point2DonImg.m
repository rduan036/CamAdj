function [point2D, reject_idx] = point2DonImg(point2D, img, boundary)

if nargin < 3
    boundary = 0;
end

if size(img,3) == 3
    img = rgb2gray(img);
end
[m,n] = size(img);

reject_idx = find( point2D(:,1) < boundary | point2D(:,1) < boundary | ...
   point2D(:,1) > n - boundary | point2D(:,1) > n - boundary | ...
   point2D(:,2) < boundary | point2D(:,2) < boundary | ...
   point2D(:,2) > m - boundary | point2D(:,2) > m - boundary );

point2D(reject_idx,:) = [];

% figure;
% imshow(img);
% hold on;
% plot(point2D(:,1), point2D(:,2), 'g+');
% hold off;

return