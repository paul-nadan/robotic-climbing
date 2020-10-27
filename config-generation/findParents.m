% Helper function for initializing configurations
function parents = findParents(count)
    parents = zeros(1, sum(count));
    for i = 1:length(parents)
        p = find(count(1:i), 1, 'last');
        count(p) = count(p) - 1;
        parents(i) = p-1;
    end
end