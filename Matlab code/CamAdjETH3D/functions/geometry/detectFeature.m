function [points_loc, discriptors] = detectFeature(img, image_grid, Top_N)

if nargin < 3
    Top_N = 0;
end

if length(size(img)) == 3
    img0 = rgb2gray(img);
else
    img0 = img;
end

points_loc = [];
border = 5;
M = image_grid(1);
N = image_grid(2);
if M > 1 || N > 1
    np = round(Top_N/(M*N));
    [h, w] = size(img0);
    h = h - 2*border;
    w = w - 2*border;
    rect_h = fix(h/M);
    rect_w = fix(w/N);
    for ii = 1:M
        for jj = 1:N
            x = (jj - 1) * rect_w + border;
            y = (ii - 1) * rect_h + border;
            roi = [x,y,rect_w,rect_h];
            points = detectORBFeatures(img0,'ROI', roi);
            if np > 0
                points = selectStrongest(points, np);
            end
            points_loc = [points_loc;points.Location];
        end
    end
    points = ORBPoints(points_loc);
    discriptors = extractFeatures(img0, points);
else
    points = detectORBFeatures(img0);
    if Top_N > 0
        points = selectStrongest(points, Top_N);
    end
    discriptors = extractFeatures(img0, points);
    points_loc = points.Location;
end

return