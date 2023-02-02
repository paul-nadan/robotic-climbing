function plotTorques(robot, T, color, scale)
    joints = robot.vertices - robot.links;
    for i = 1:size(T, 2)-1
        joint = joints(:, i);
        quiver3(joint(1), -joint(3), joint(2), T(1, i)*scale, ...
            -T(3, i)*scale, T(2, i)*scale, color, 'linewidth', 2);
    end
end