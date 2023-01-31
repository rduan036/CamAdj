function inViewIdx = CamAdjPointInView(frame, points)

[h, w] = size(frame);
inViewIdx = find(points(:,1) > 0 & points(:,1) < w & points(:,2) > 0 & points(:,2) < h);

return