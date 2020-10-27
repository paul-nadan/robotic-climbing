function plotTerrain(grid)
    cla();
    zq = grid.z;
    for i = 1:size(grid.obstacles, 2)/2
        obstacle = grid.obstacles(:, 2*i-1:2*i);
        iObstacle = (grid.x>=obstacle(1,1)&grid.x<=obstacle(1,2))&...
            (grid.y>=obstacle(2,1)&grid.y<=obstacle(2,2));
        zq(iObstacle) = obstacle(3,2);
    end
    mesh(grid.x(1:2:end,1:2:end),-zq(1:2:end,1:2:end),grid.y(1:2:end,1:2:end), zq(1:2:end,1:2:end));
    hold on
    axis equal
end