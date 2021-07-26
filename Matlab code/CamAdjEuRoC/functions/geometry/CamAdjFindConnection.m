function [connection, outliers] = CamAdjFindConnection(pointKeyFrame, pointReproject, noiseLevel)

[connection, d] = dsearchn(pointReproject, pointKeyFrame);
outliers = find(d > noiseLevel);

return