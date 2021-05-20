function [cost, costs] = workspace2cost(results, res)
    if width(results) == 1
        cost = NaN;
        costs = NaN;
        return
    end
    z0 = min(results{:,1}(:,3));
    zmax = max(results{:,1}(:,3));
    z = z0:res:zmax;
    costs = zeros(length(z), 1);
    for i = 1:length(z)
        layer = abs(results{:,1}(:,3) - z(i)) < res/2;
        A = sum(results{layer,2})*res.^2;
        costs(i) = area2cost(A);
    end
    cost = sum(area2cost(0) - costs);
    cost = (area2cost(0) - cost/8);
%     cost = min(costs);
end