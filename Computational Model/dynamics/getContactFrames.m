function g_fc = getContactFrames(robot, feet, g_pl, g_wp, grid)
    g_fc = cell(length(feet), 1);
    for iFoot = 1:length(feet)
        foot = robot.vertices(:, feet(iFoot));
        N = [-f(foot(1),foot(2),grid.dzdx,grid);...
                       -f(foot(1),foot(2),grid.dzdy,grid); 1];
        N = N/norm(N);
        T = cross(N, [1; 0; 0]);
        T = T/norm(T);
        B = cross(N, T);
        R_wc = [N, T, B];
        R_fc = g_pl{iFoot}(1:3,1:3)'*g_wp(1:3,1:3)'*R_wc;
        g_fc{iFoot} = [R_fc, zeros(3,1); 0 0 0 1];
    end
end