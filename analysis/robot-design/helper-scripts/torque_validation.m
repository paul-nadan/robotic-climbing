grid = obstacle([-1, 1], ...
        [-1 1], .01, .15, 0.1, [0;-.75;0], 42);
[F, Fnorm, Ftang, T, N] = quasiStaticDynamicsKnownForce(robot, 1, robot.F, grid);
plotTerrain(grid);
plotRobot(robot);
T





F1, F2, F3, F4
