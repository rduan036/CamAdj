function [quadrantVectors, availableQuad] = CamAdjVectorDistribution(vectors)
    availableQuad = ones(4,1);
    firstQuad = find(vectors(:,1) > 0 & vectors(:,2) > 0 );
    secondQuad = find(vectors(:,1) > 0 & vectors(:,2) < 0 );
    thirdQuad = find(vectors(:,1) < 0 & vectors(:,2) < 0 );
    forthQuad = find(vectors(:,1) < 0 & vectors(:,2) > 0 );
    if size(firstQuad) < 10
        availableQuad(1) = 0;
        quadrantVectors(1,:) = [0 0];
    else
%         availableQuad(1) = length(firstQuad);
        quadrantVectors(1,:) = mean(vectors(firstQuad,:));
    end
    if size(secondQuad) < 10
        availableQuad(2) = 0;
        quadrantVectors(2,:) = [0 0];
    else
%         availableQuad(2) = length(secondQuad);
        quadrantVectors(2,:) = mean(vectors(secondQuad,:));
    end
    if size(thirdQuad) < 10
        availableQuad(3) = 0;
        quadrantVectors(3,:) = [0 0];
    else
%         availableQuad(3) = length(thirdQuad);
        quadrantVectors(3,:) = mean(vectors(thirdQuad,:));
    end
    if size(forthQuad) < 10
        availableQuad(4) = 0;
        quadrantVectors(4,:) = [0 0];
    else
%         availableQuad(4) = length(forthQuad);
        quadrantVectors(4,:) = mean(vectors(forthQuad,:));
    end
return