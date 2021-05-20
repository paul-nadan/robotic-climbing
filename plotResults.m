function plotResults(meanScores, SWEEP1, SWEEP2, SCORES, PLOT_SCORES, ...
    AXIS_LABELS)

    if 0
        for score = 1:length(PLOT_SCORES)
            figure();
            hold on;
            for i2 = 1:length(SWEEP2)
                plot(SWEEP1, meanScores(:,i2,PLOT_SCORES(score)), 'linewidth', 3);
            end
            xlabel(AXIS_LABELS{1});
            ylabel(SCORES{PLOT_SCORES(score)});
            title(SCORES{PLOT_SCORES(score)});
        end
    elseif size(meanScores,1) > 1 && size(meanScores,2) > 1
        for score = 1:length(PLOT_SCORES)
            figure();
            data = meanScores(:,:,1,PLOT_SCORES(score));
            data(isnan(data)) = max(data, [], 'all');
            imagesc(SWEEP1, SWEEP2, data');
            set(gca,'YDir','normal');
            colorbar;
            title(SCORES{PLOT_SCORES(score)});
            xlabel(AXIS_LABELS{1});
            ylabel(AXIS_LABELS{2});
        end
    elseif size(meanScores,1) > 1 || size(meanScores,2) > 1
        % If multiple scores, use separate figures for each score
        for score = 1:length(PLOT_SCORES)
            figure();
            if size(meanScores,1) > 1
                plot(SWEEP1, meanScores(:,1,PLOT_SCORES(score)), 'linewidth', 3);
                xlabel(AXIS_LABELS{1});
            else
                plot(SWEEP2, meanScores(1,:,PLOT_SCORES(score)), 'linewidth', 3);
                xlabel(AXIS_LABELS{2});
            end
            ylabel(SCORES{PLOT_SCORES(score)});
            title(SCORES{PLOT_SCORES(score)});
        end
    end
end