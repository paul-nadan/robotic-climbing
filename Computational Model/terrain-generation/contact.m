% Returns distance to nearest collision with terrain
function distance = contact(X, grid)
    gDist = (X(3,:)-f(X(1,:), X(2,:), grid.z, grid))';
    distance = gDist;
    for i = 1:size(grid.obstacles, 2)/2
        obstacle = grid.obstacles(:, 2*i-1:2*i);
        oDist = max(max(obstacle(:,1) - X(:,:), X(:,:) - obstacle(:,2)))';
        distance = min(distance, oDist);
    end
end