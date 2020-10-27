function plotForces(robot, F, count, color, scale)
    i = mod(count, size(robot.gait.angles, 2))+1;
    feet = robot.vertices(:, robot.gait.feet(:,i) > 0);
    for i = 1:size(F, 2)-1
        foot = feet(:, i);
        quiver3(foot(1), -foot(3), foot(2), F(1, i)*scale, ...
            -F(3, i)*scale, F(2, i)*scale, color, 'linewidth', 2);
    end
    c = robot.origin;
    quiver3(c(1), -c(3), c(2), F(1, end)*scale, -F(3, end)*scale, ...
        F(2, end)*scale, color, 'linewidth', 2);
end