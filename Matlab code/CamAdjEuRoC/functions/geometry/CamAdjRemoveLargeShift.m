function validIdx = CamAdjRemoveLargeShift(Points1, Points2, validIdx)

shift_vector = Points1 - Points2;
dist_vector = vecnorm(shift_vector');
mean_shift = mean(medfilt1(dist_vector(validIdx)));
var_shift = std(medfilt1(dist_vector(validIdx)));
outliers = find(dist_vector > mean_shift + 3*var_shift);
validIdx(outliers) = 0;

return