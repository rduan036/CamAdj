% input: image, M * N grids, number of points in each grids, edge, minQuality
function points_loc = CamAdjDetectFeature(img, featurePara)

MinQuality = featurePara(1);
M = featurePara(2);
N = featurePara(3);
np = featurePara(4);
edge = featurePara(5);
points_loc = [];

[h, w] = size(img);
h = h - 2*edge;
w = w - 2*edge;
rect_h = fix(h/M);
rect_w = fix(w/N);
for ii = 1:M
    for jj = 1:N
        x = (jj - 1) * rect_w + edge;
        y = (ii - 1) * rect_h + edge;
        roi = [x,y,rect_w,rect_h];
        points = detectMinEigenFeatures(img, 'MinQuality', MinQuality, 'ROI', roi);
        points = selectStrongest(points, np);
        points_loc = [points_loc;points.Location];
    end
end

return