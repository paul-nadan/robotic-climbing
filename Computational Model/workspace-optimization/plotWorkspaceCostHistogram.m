A = 0:.001:.2;
plot(A, area2cost(A), 'k');
hold on;
plot(cost2area(scores(:,1)), scores(:,1), 'r.', 'markersize', 20);
plot(cost2area(scores(:,2)), scores(:,2), 'g.', 'markersize', 20);
plot(cost2area(scores(:,3)), scores(:,3), 'b.', 'markersize', 20);

histogram(cost2area(reshape(scores,[],1)),'Normalization', 'Probability')

legend('Cost', 'Front Legs', 'Middle Legs', 'Back Legs', 'Frequency')
xlabel('Workspace Area (m$^2$)')
ylabel('Adhesion Cost')