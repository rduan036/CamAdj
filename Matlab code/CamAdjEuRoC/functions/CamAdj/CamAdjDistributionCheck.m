function bad_distribution = CamAdjDistributionCheck(Points, img)
[m,n] = size(img);
% N = size(Points,1);
[coeff, score, latent] = pca(Points);
% figure(4);
% clf;
% subplot(1,2,1);
% hold on;
% plot(Points(:,1), Points(:,2),'b.');
% plot(score(:,1), score(:,2), 'r.');
% hold off;
% subplot(1,2,2);
% plotv(coeff,'-');
% disp(['pca latent: ', num2str(latent')]);
% mean_point = [n/2, m/2];
% vectors = Points - repmat(mean_point,N,1);
[quadrantVectors, availableQuad] = CamAdjVectorDistribution(score);
if sum(availableQuad) < 3 || sum(vecnorm(quadrantVectors') > norm(m/8,n/8)) < 3
    bad_distribution = 1;
elseif latent(1)/latent(2) > 50 || latent(2)/latent(1) > 50
    bad_distribution = 1;
else
    bad_distribution = 0;
end
return

% fp_std = std(inlierPoints2);
% if fp_std(1) < n/10 || fp_std(2) < m/10
%     bad_distribution = 1;
% end