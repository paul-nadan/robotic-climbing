function results = optimize(getConfig, ...
    getTerrain, getScores, SWEEP1, SWEEP2, SAMPLES, STEPS, TIME_STEP, ...
    ABORT_STRIKES, IGNORE_FAILS, SEED, SCORES, REUSE_DATA)
    
    FAIL_COST = 1;
    startTime = tic();
    
    Sweep1 = optimizableVariable('Var1', [min(SWEEP1), max(SWEEP1)]);
    Sweep2 = optimizableVariable('Var2', [min(SWEEP2), max(SWEEP2)]);
    results = bayesopt(@cost, [Sweep1, Sweep2], 'PlotFcn', ...
        @plotObjectiveModel, 'UseParallel', 1, 'Verbose', 2, ...
        'MaxObjectiveEvaluations', 50)
    fprintf('Total optimization runtime: %.3f seconds\n', toc(startTime));
    
    
    function Objective = cost(X)
        try
        sweep1 = X.Var1;
        sweep2 = X.Var2;
%         objective = rand()*.001+(sweep1-.16)^2 +(sweep2-.18)^2;
%         x = objective(2);
%         return;
        config = feval(getConfig, sweep1, sweep2);
        strikes = 0;
        scores = zeros(STEPS, SAMPLES);
        for sample = 1:SAMPLES
            grid = feval(getTerrain, sweep1, sweep2, -1);
            robot = spawnRobot(grid.spawn, eye(3), config, grid);
            robots = repmat(robot, STEPS + 1, 1);
            for i = 1:STEPS
                lastRobot = robots(i);
                if lastRobot.fail
                    robot = lastRobot;
                    robots(i+1) = robot;
                else
                    robot = step(lastRobot, i, grid);
                    robots(i+1) = robot;
                end
                scores(i, sample) = feval(getScores, robot, lastRobot, i, grid);
                if robot.fail && ~lastRobot.fail
                    strikes = strikes + 1;
                end
            end
            if IGNORE_FAILS && robot.fail
                scores(:, sample) = NaN;
            end
            if ABORT_STRIKES > 0 && strikes >= ABORT_STRIKES
                Objective = FAIL_COST;
                return
            end
        end
        Objective = mean(scores, 'all', 'omitnan');
        catch ME
            message = sprintf('%s\n', ME.message);
            for i = 1:length(ME.stack)
                trace = sprintf('%s, %s, line %d\n', ME.stack(i).file, ME.stack(i).name, ME.stack(i).line);
                message = [message, trace];
            end
            error(message);
        end
    end
end