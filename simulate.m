function [meanScores, rawScores, seeds, robots] = simulate(getConfig, ...
    getTerrain, getScores, SWEEP1, SWEEP2, SAMPLES, STEPS, TIME_STEP, ...
    ABORT_STRIKES, IGNORE_FAILS, SEED, SCORES)

    global FRAMES ANIMATE RECORD PLOT
    N1 = length(SWEEP1);
    N2 = length(SWEEP2);
    
    % Set view window size
    close all;
    if RECORD
        figure('units','normalized','outerposition',[0 0 1 1]);
        FRAMES = struct('cdata',{},'colormap',{});
    elseif ANIMATE || PLOT
        figure('units','normalized','outerposition',...
            [0 0.2 0.3 0.7]);
    end
    
    % Set random seed
    if SEED >= 0
        rng(SEED);
    else
        rng shuffle
        s = rng;
        SEED = s.Seed;
    end
    fprintf('Simulation Seed: %d\n\n', SEED);
    seeds = repmat(rand(1, 1, SAMPLES)*1000, N1, N2);
    
    % Initialization
    rawScores = zeros(N1, N2, SAMPLES, length(SCORES));
    robots = cell(N1, N2, SAMPLES);
    
    % Iterate over parameters
    workers = (N1 > 1)*N1;
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
                grid = feval(getTerrain, sweep1, sweep2, seeds(i1, i2, i3));

                % Initialize robot
                robot = spawnRobot(grid.spawn, eye(3), config, grid);
                robots{i1, i2, i3} = repmat(robot, STEPS + 1, 1);

                % Run simulation
                stepScores = zeros(STEPS, length(SCORES));
                fprintf('Scores:  [');
                fprintf('%10s, ', string(SCORES));
                fprintf(']\n');
                for i = 1:STEPS
                    if i < 10
                        fprintf('Step %d:  ', i);
                    else
                        fprintf('Step %d: ', i);
                    end
                    dt = NaN;
                    lastRobot = robots{i1, i2, i3}(i);
                    if lastRobot.fail
                        robot = lastRobot;
                        robots{i1, i2, i3}(i+1) = robot;
                    else
                        tic();
                        robot = step(lastRobot, i, grid);
                        dt = toc();
                        robots{i1, i2, i3}(i+1) = robot;
                    end
                    if robot.fail
                        stepScores(i,:) = NaN;
                        stepScores(i,1) = 0;
                        fprintf('Failed to converge\n');
                    else
                        stepScores(i,:) = [feval(getScores, robot, lastRobot, i, grid), dt];
                        fprintf('[');
                        fprintf('%10.3f, ', stepScores(i,:));
                        fprintf(']\n');
                    end
                    animateStep(lastRobot, robot, TIME_STEP, i, grid);
                    if robot.fail
                        strikes = strikes + 1;
                    end
                end

                % Evaluate iteration results
                rawScores(i1, i2, i3, :) = mean(stepScores, 'omitnan');
                if IGNORE_FAILS && robot.fail
                    rawScores(i1, i2, i3, :) = NaN;
                end
                robot = robots{i1, i2, i3}(end);
                [F1, ~, ~, ~] = quasiStaticDynamicsKnownForce(robot, STEPS, robot.F, grid);
                F2 = quasiStaticDynamics(robot, STEPS, grid);
                path = [robots{i1, i2, i3}.origin];
                % Plot final robot state
                if PLOT
                    plotTerrain(grid);
                    plotRobot(robots{i1, i2, i3}(end));
                    plotForces(robot, F2, STEPS, 'r', 0.016);
                    plotForces(robot, F1, STEPS, 'g', 0.016);
    %                 plotTorques(robot, T, 'c', 0.1);
                    plot3(path(1,:), -path(3,:), path(2,:),'k','linewidth', 2);
                    title(int(seeds(i1, i2, i3)));
                    drawnow();
                end

                % Print iteration results
                fprintf('Average: [');
                fprintf('%10.3f, ', rawScores(i1, i2, i3, :));
                fprintf(']\n');
                fprintf('Sweep 1: %.3f, Sweep 2: %.3f, Sample: %d\n\n', sweep1, sweep2, i3);
                if ABORT_STRIKES > 0 && strikes >= ABORT_STRIKES
                    fprintf('Aborted after sample: %d\n', i3);
                    break
                end
            end
        end
    end
    meanScores = mean(rawScores, 3, 'omitnan');
end