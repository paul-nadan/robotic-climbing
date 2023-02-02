% index = 1;
% sample = 4;
% 
% r = robots{index,sample}(2);
% grid = terrain([-1.5, 1.5], [-1.5 3.5], .01, 1*[1,1,0.5], ...
%         [1, .25, 0.0625], 0, [0;-.5;0], seeds(1,1,sample));
% % plotTerrain(grid);
% plotRobot(r);
% [F1, ~, ~, ~] = quasiStaticDynamicsKnownForce(r, 1, r.F, grid);
% plotForces(r, F1, 1, 'r', 0.016);
% axis equal;
% 
% [pitch, roll, yaw] = getAngle(r, 1, grid, 1)
% [pitch, roll, yaw] = getAngle(r, 2, grid, 1)
% [pitch, roll, yaw] = getAngle(r, 3, grid, 1)
% [pitch, roll, yaw] = getAngle(r, 4, grid, 1)
% return

EVALUATE = 0;
CODE = '000000000';
PERCENTILE = 5;

config = simpleWalker(CODE, 0.2, 0.5, 0.1);
for iconfig = 1:length(robots)
    if isequal(abs(robots{iconfig,1}(1).config.joints)>0, abs(config.joints)>0)
        break
    end
end

if EVALUATE
    tic();
    pitches = zeros(size(robots, 1), size(robots,3), length(robots{1,1,1})-2, 6);
    rolls = zeros(size(robots, 1), size(robots,3), length(robots{1,1,1})-2, 6);
    yaws = zeros(size(robots, 1), size(robots,3), length(robots{1,1,1})-2, 6);
    for i = 1:size(pitches, 1) % config
        for j = 1:size(pitches, 2) % sample
            grid = terrain([-1.5, 1.5], [-1.5 3.5], .01, 1*[1,1,0.5], ...
                [1, .25, 0.0625], 0, [0;-.5;0], seeds(1,1,j));
            for k = 1:size(pitches, 3) % step
                skips = 0;
                for l = 1:size(pitches, 4) % foot
                    r = robots{i,j}(k+2);
                    if r.skip
                        skips = skips + robot.skip;
                    end
                    [pitch, roll, yaw] = getAngle(r, l, grid, k+skips-1+2);
                    pitches(i,j,k,l) = pitch;
                    rolls(i,j,k,l) = roll;
                    yaws(i,j,k,l) = yaw;
                end
            end
        end
    end
    toc()
end

pitches2 = reshape(pitches, size(pitches, 1), []);
rolls2 = reshape(rolls, size(rolls, 1), []);
yaws2 = reshape(yaws, size(yaws, 1), []);

pitchBound = [prctile(pitches2(iconfig,:), PERCENTILE);
              prctile(pitches2(iconfig,:), 100-PERCENTILE)];
rollBound = [prctile(rolls2(iconfig,:), PERCENTILE);
              prctile(rolls2(iconfig,:), 100-PERCENTILE)];
yawBound = [prctile(yaws2(iconfig,:), PERCENTILE);
              prctile(yaws2(iconfig,:), 100-PERCENTILE)];

bins = -175:10:175;
figure(1);
histogram(pitches2(iconfig,:), bins, 'Normalization','probability')
title('Pitch Distribution (Knees)');
xlabel('Pitch Angle (deg)');
ylabel('Frequency');
ylim([0,.14]);
vline(pitchBound);
figure(2);
histogram(rolls2(iconfig,:), bins, 'Normalization','probability');
xlabel('Roll Angle (deg)');
ylabel('Frequency');
ylim([0,.14]);
vline(rollBound);
title('Roll Distribution (Knees)');
figure(4);
plot(pitches2(:), rolls2(:), '.');
xlabel('Pitch Angle (deg)');
ylabel('Roll Angle (deg)');
title('Roll vs Pitch Distribution (All Designs)');
figure(3);
histogram(yaws2(iconfig,:), bins, 'Normalization','probability');
xlabel('Yaw Angle (deg)');
ylabel('Frequency');
ylim([0,.14]);
vline(yawBound);
title('Yaw Distribution (No Knees)');
fprintf('Pitch %d%% bound: [%d, %d]\n', [PERCENTILE;round(pitchBound)]);
fprintf('Roll %d%% bound: [%d, %d]\n', [PERCENTILE;round(rollBound)]);
fprintf('Yaw %d%% bound: [%d, %d]\n', [PERCENTILE;round(yawBound)]);

% Positive pitch -> angled downwards, positive roll -> angled downwards
function [pitch, roll, yaw] = getAngle(robot, foot, grid, step)
    iFeet = find(any(robot.config.gait.feet == 1, 2));
    if foot > length(iFeet)
        pitch = NaN;
        roll = NaN;
        yaw = NaN;
        return
    end
    iFoot = iFeet(foot);
    R = robot.R(:,:,iFoot);
    pos = robot.vertices(:,iFoot);
    N = [-f(pos(1),pos(2),grid.dzdx,grid);...
                   -f(pos(1),pos(2),grid.dzdy,grid); 1];
    N = N/norm(N);

    x = R*[1;0;0];
    y = R*[0;1;0];
    z = R*[0;0;1];
%     quiver3(pos(1),-pos(3),pos(2),-N(1)/3,N(3)/3,-N(2)/3, 'linewidth',2);
%     quiver3(pos(1),-pos(3),pos(2),y(1)/3,-y(3)/3,y(2)/3, 'r','linewidth',2);
%     quiver3(pos(1),-pos(3),pos(2),z(1)/3,-z(3)/3,z(2)/3, 'b','linewidth',2);
    
    if mod(foot,2) == 1
        pitch = atan2d(N'*z, N'*x);
        Rp = vrrotvec2mat([y; deg2rad(pitch)]);
%         Np = Rp*N;
%         quiver3(pos(1),-pos(3),pos(2),-Np(1)/3,Np(3)/3,-Np(2)/3, 'g','linewidth',2);

        roll = atan2d((Rp*N)'*y, (Rp*N)'*x);
        Rr = vrrotvec2mat([z; -deg2rad(roll)]);
    else
        pitch = atan2d(N'*z, -N'*x);
        Rp = vrrotvec2mat([y; -deg2rad(pitch)]);
%         Np = Rp*N;
%         quiver3(pos(1),-pos(3),pos(2),-Np(1)/3,Np(3)/3,-Np(2)/3, 'g','linewidth',2);
        roll = atan2d((Rp*N)'*y, -(Rp*N)'*x);
        Rr = vrrotvec2mat([z; deg2rad(roll)]);
    end
    i = mod(step, size(robot.gait.angles, 2))+1;
    iFeetF = robot.gait.feet(:,i) > 0;
    iFeetF = iFeetF(iFeet);
    if iFeetF(foot)
        y2 = Rp'*Rr'*y;
        z2 = Rp'*Rr'*z;
        F = robot.F(:,sum(iFeetF(1:foot)));
%         quiver3(pos(1),-pos(3),pos(2),y2(1)/5,-y2(3)/5,y2(2)/5, 'b','linewidth',2);
%         quiver3(pos(1),-pos(3),pos(2),z2(1)/5,-z2(3)/5,z2(2)/5, 'm','linewidth',2);
        yaw = atan2d(F'*y2, F'*z2);
        Fn = F'*N/norm(F);
        mu = sqrt(1-Fn.^2)/Fn;
        if Fn > 0 && mu < .7 % coefficient of friction
            yaw = NaN;
        end
    else
        yaw = NaN;
    end
end