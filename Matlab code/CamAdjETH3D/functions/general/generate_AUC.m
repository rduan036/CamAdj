function precision = generate_AUC(data, thresholds)

precision = zeros(length(thresholds),1);

for ii = 1:length(thresholds)
    threshold = thresholds(ii);
    idx = find(data <= threshold);
    precision(ii) = length(idx)/length(data);
end

return