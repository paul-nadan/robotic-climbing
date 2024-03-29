function [meanScores, rawScores, allScores, seeds, robots] = simulate(getConfig, ...
    getTerrain, getScores, averageScores, SWEEP1, SWEEP2, SAMPLES, STEPS, TIME_STEP, ...
    ABORT_STRIKES, IGNORE_FAILS, SEED, SCORES, REUSE_DATA, fixedFootPlacements)

    obstacleStep = 0;
    
    global FRAMES ANIMATE RECORD PLOT
    N1 = length(SWEEP1);
    N2 = length(SWEEP2);
    startTime = tic();
    SIMULATE = isequal(0, REUSE_DATA);
    ANIMATE_ = ANIMATE || RECORD;
    PLOT_ = PLOT;
    
    % Set view window size
    close all;
    if RECORD
        figure('units','normalized','outerposition',[0 0 .3 1]);
        FRAMES = struct('cdata',{},'colormap',{});
    elseif ANIMATE || PLOT
        figure('units','normalized','outerposition',...
            [0 0.2 0.3 0.7]);
    end
    
    % Set random seed
    if SEED >= 0
        rng(SEED, 'twister');
    else
        rng shuffle
        s = rng;
        SEED = s.Seed;
    end
    fprintf('Simulation Seed: %d\n\n', SEED);
    seeds = repmat(rand(1, 1, SAMPLES)*1000, N1, N2);
    if N1 == 1 && N2 == 1 && SAMPLES == 1
        seeds = SEED;
    end
    
    % Initialization
    rawScores = zeros(N1, N2, SAMPLES, length(SCORES));
    allScores = zeros(N1, N2, SAMPLES, STEPS, length(SCORES));
    aborted = zeros(N1, N2);
    if SIMULATE
        robots = cell(N1, N2, SAMPLES);
    else
        robots = REUSE_DATA;
    end
    
    % Iterate over parameters
    workers = 4;%(N1 > 1)*N1*SIMULATE;
%     parfor (i1 = 1:N1,workers)
    for i1 = 1:N1
        for i2 = 1:N2
            sweep1 = SWEEP1(i1);
            sweep2 = SWEEP2(i2); %#ok<PFBNS>
            % Generate configuration
            config = feval(getConfig, sweep1, sweep2);
            
            % Iterate over samples
            strikes = 0;
            for i3 = 1:SAMPLES
                % Generate terrain
%                 grid = feval(getTerrain, sweep1, sweep2, seeds(i1, i2, i3));
                grid = feval(getTerrain, i3, sweep2, seeds(i1, i2, i3));

                % Initialize robot
                if SIMULATE
                    robot = spawnRobot(grid.spawn, eye(3), config, grid, obstacleStep);
                    robot.skip = 0;
                    robots{i1, i2, i3} = repmat(robot, STEPS + 1, 1);
                end
                
                % Run simulation
                stepScores = zeros(STEPS, length(SCORES));
                fprintf('Scores:  [');
                fprintf('%10s, ', string(SCORES));
                fprintf(']\n');
                skips = 0 + obstacleStep-1;
                if obstacleStep == 0
                    skips = 0;
                end
                for i = 1:STEPS
                    if i < 10
                        fprintf('Step %d:  ', i);
                    else
                        fprintf('Step %d: ', i);
                    end
                    dt = NaN;
                    lastRobot = robots{i1, i2, i3}(i);
                    % Obstacles
%                     lastRobot = spawnRobot(grid.spawn, eye(3), config, grid, i);
%                     lastRobot.skip = 0;
%                     skips = 0;
                    %
                    if 0&&lastRobot.fail
                        robot = lastRobot;
                        robots{i1, i2, i3}(i+1) = robot;
                    elseif SIMULATE
                        tic();
                        if length(fixedFootPlacements) >= i+1
                            robot = step2(lastRobot, i+skips, grid, 0, fixedFootPlacements{i+1});
                        else
                            robot = step2(lastRobot, i+skips, grid, 0, []);
                        end
                        dt = toc();
                        robots{i1, i2, i3}(i+1) = robot;
                    else
                        robot = robots{i1, i2, i3}(i+1);
                    end
                    if ~isfield(robot, 'skip')
                        robot.skip = 0;
                    end
                    skips = skips + robot.skip;
                    if i==-1
                        stepScores(i,:) = NaN;
                        stepScores(i,end) = dt;
                        fprintf('Ignored\n');
                    elseif robot.fail
                        stepScores(i,:) = NaN;
                        stepScores(i,1) = 0;
                        stepScores(i,end) = dt;
                        fprintf('Failed to converge\n');
                    else
                        stepScores(i,:) = [feval(getScores, robot, lastRobot, i+skips, grid), dt];
                        fprintf('[');
                        fprintf('%10.3f, ', stepScores(i,:));
                        fprintf(']\n');
                    end
                    if ANIMATE_
%                         [F1, ~, ~, ~] = quasiStaticDynamicsKnownForce(robot, i+skips-robot.skip, robot.F, grid);
%                         plotForces(robot, F1, i+skips-robot.skip, 'g', 0.016);
                        animateStep(lastRobot, robot, TIME_STEP, i+skips-robot.skip, grid);
%                         if ~SIMULATE
%                             pause();
%                         end
                    end
                    if robot.fail && ~lastRobot.fail
                        strikes = strikes + 1;
                    end
%                     if stepScores(i,3) >= 0.3
%                         stepScores(i+1:end,:) = NaN;
%                         break
%                     end
                end

                % Evaluate iteration results
                tempScores = mean(stepScores, 1, 'omitnan');
%                 tempScores(2) = max(stepScores(:,2), [], 'omitnan');
                rawScores(i1, i2, i3, :) = tempScores;
                allScores(i1, i2, i3, :, :) = stepScores;
                if IGNORE_FAILS && robot.fail
                    rawScores(i1, i2, i3, :) = NaN;
                end
%                 robot = robots{i1, i2, i3}(end);
                [F1, ~, ~, ~] = quasiStaticDynamicsKnownForce(robot, STEPS, robot.F, grid);
%                 F2 = quasiStaticDynamics(robot, STEPS, grid);
                path = [robots{i1, i2, i3}.origin];
%                 com = [robots{i1, i2, i3}.com];
%                 com = com(:,1:2:end);
%                 path = com+(com - path);

                % Plot final robot state
                if PLOT_
                    plotTerrain(grid);
                    plotRobot(robot);
%                     plotForces(robot, F2, STEPS, 'r', 0.016);
                    plotForces(robot, F1, STEPS+skips, 'g', 0.016);
    %                 plotTorques(robot, T, 'c', 0.1);
                    plot3(path(1,:), -path(3,:), path(2,:),'k','linewidth', 2);
                    title(seeds(i1, i2, i3));
                    drawnow();
                end

                % Print iteration results
                fprintf('Average: [');
                fprintf('%10.3f, ', rawScores(i1, i2, i3, :));
                fprintf(']\n');
                fprintf('Sweep 1: %.3f, Sweep 2: %.3f, Sample: %d\n\n', sweep1, sweep2, i3);
                if ABORT_STRIKES > 0 && strikes >= ABORT_STRIKES
                    fprintf('Aborted after sample: %d\n', i3);
                    aborted(i1, i2) = 1;
                    break
                end
            end
        end
    end
    rawScores(:, :, :, 1) = rawScores(:, :, :, 1).*(~aborted);
    [meanScores, rawScores] = feval(averageScores, rawScores, allScores, robots);
    fprintf('Total simulation runtime: %.3f seconds\n', toc(startTime));
end