function [ste, std_intra, std_inter, std_global] = estimateError(rawScores)
    cost = min(1, max(1-rawScores(:,:,:,1), rawScores(:,:,:,7)));
    rawScores = cat(4, rawScores, cost);
    
    std_intra = reshape(std(rawScores, 0, 3, 'omitnan'), [], size(rawScores, 4));
    std_intra = sqrt(mean(std_intra.^2, 'omitnan'));
    ste = std_intra/sqrt(size(rawScores, 4));
    
    mean_inter = reshape(mean(rawScores, [2,3], 'omitnan'), [], size(rawScores, 4));
    std_inter = std(mean_inter, 'omitnan');
    
    std_global = std(reshape(rawScores, [], size(rawScores, 4)), 'omitnan');
    
    meanScores = mean(rawScores, 3, 'omitnan');
    normalized = reshape(rawScores - meanScores, [], size(rawScores, 4));
%     histogram(normalized(:, end), 20);
    sum(abs(normalized(:,end))<0.0880)/size(normalized, 1)
end