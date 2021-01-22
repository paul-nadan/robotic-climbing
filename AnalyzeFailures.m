grid = terrain([-1.5, 1.5], ...
    [-1.5 3.5], .01, 1*[1,1,0.5], [1, .25, 0.0625], 0, [0;-.75;0], SEED);
cost = zeros(STEPS+1,1);
skips = 0;
for i = 1:STEPS+1
    robot = robots{1}(i);
    if isfield(robot, 'skip')
        skips = skips + robot.skip;
    end
%     figure(i);
    plotTerrain(grid);
    plotRobot(robot);
    title(i);
    [F, Fnorm, Ftang, ~, N] = quasiStaticDynamicsKnownForce(robot, i+skips-1, robot.F, grid);
    Fnorm = max(Fnorm,0);
    cost(i) = gripperMargin(Fnorm, Ftang);
    plotForces(robot, F, i+skips-1, 'g', 0.016);
    plotForces(robot, [N, [0;0;0]], i+skips-1, 'r', 0.16);
end
disp(cost);