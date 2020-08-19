SIMULATE = ~~1; % flag to re-run simulation instead of using existing data

if SIMULATE
    clear;
    SIMULATE = 1;
end
close all;
global X Y Z dZdx dZdy FRAMES ANIMATE RECORD PLOT

% VISUALIZATION FLAGS
ANIMATE = ~~0; % flag to animate robot motion
RECORD = ~~0; % flag to save animations as video files
PLOT = ~~1; % flag to plot final robot condition and path

% SIMULATION PARAMETERS
CONFIGURATIONS = {@step}; % step functions to compare
LABELS = {'Normal', 'Tangential', 'Ratio', 'Normal (ideal)', ...
    'Tangential (ideal)'}; % legend entries for configurations
COLORS = ['b', 'r', 'k'];
SCORES = 3; % number of output variables
SWEEP = -180:5:180; % values for parameter being swept
SAMPLES = 1; % number of duplicate samples to average at each value
STEPS = 1; % number of robot steps to simulate per trial
TIME_STEP = 0.25; % delay between frame updates for animation

% SET VIEW WINDOW SIZE
if RECORD
    for config = 1:length(CONFIGURATIONS)
        figure('units','normalized','outerposition',[0 0 1 1]);
    end
    FRAMES = cellfun(@(~) {struct('cdata',{},'colormap',{})},...
                      cell(size(CONFIGURATIONS)));
elseif ANIMATE || PLOT
    figure('units','normalized','outerposition',...
        [0 0.2, 0.3*length(CONFIGURATIONS) 0.7]);
end

% SWEEP PARAMETER
rawScores = zeros(length(SWEEP)*SAMPLES, length(CONFIGURATIONS)*SCORES);
sumScores = zeros(length(SWEEP), length(CONFIGURATIONS)*SCORES+1);
if SIMULATE
    robots = cell(size(rawScores));
end
for iter = 1:size(rawScores,1)
    iSweep = ceil(iter/SAMPLES);
    sweep = SWEEP(iSweep);

    % GENERATE TERRAIN
    if SIMULATE
        [X, Y, Z, dZdx, dZdy] = terrain([-1.5, 1.5], [-1.5 2.5], .02, ...
            [1,1,0.5]*0, [1, .25, 0.0625], 0);
    end
    
    % SETUP SIMULATION
    for config = 1:length(CONFIGURATIONS)
        if ~SIMULATE
            break
        end
        stepFunc = CONFIGURATIONS{config};
        robot = placeRobot([0;-0.8;0], stepFunc, sweep);
        robots{iter, config} = repmat(robot, STEPS + 1, 1);
    end
    
    % RUN SIMULATION
    fail = 0;
    for i = 1:STEPS
        if ~SIMULATE
            break
        end
        for config = 1:length(CONFIGURATIONS)
            stepFunc = CONFIGURATIONS{config};
            lastRobot = robots{iter, config}(i);
            robot = stepFunc(lastRobot, mod(i,2), Z);
            robots{iter, config}(i+1) = robot;
            if ~min(robot.c)
                fail = 1;
                break
            end
            if RECORD
                figure(config);
        elseif ANIMATE
                subplot(1, length(CONFIGURATIONS), config);
                title(LABELS{config});
            end
            animateStep(lastRobot, robot, TIME_STEP, mod(i,2), config);
        end
        if fail
            break
        end
    end
    
    % EVALUATE ITERATION RESULTS
    if ~fail
        for config = 1:length(CONFIGURATIONS)
            if fail
                break
            end
            
            robot = robots{iter, config}(end);
            odd = 1-mod(length(robots{iter, config}),2);
            footFront = robot.feet(:,odd+1) - robot.centroid;
            footBack = robot.feet(:,odd+3) - robot.centroid;
            footBack2 = robot.feet(:,4-odd) - robot.centroid;
            dig = 0.5*max(0,-cosd(sweep))*(~odd-odd);
            grav = [0;-sind(sweep);-cosd(sweep)];
            [F1, F2, N3] = forces(footFront, footBack, footBack2, ...
                [0;0;-1], dig, grav);
            f1 = robot.feet(:,odd+1);
            f2 = robot.feet(:,odd+3);
            n3 = robot.feet(:,4-odd);
            
            if N3'*[0;0;-1] > 0
                footBack2 = robot.feet(:,2-odd) - robot.centroid;
                n3 = robot.feet(:,2-odd);
                [F1, F2, N3] = forces(footFront, footBack, footBack2, ...
                    [0;0;-1], dig, grav);
            end
            
            normalForce = max(-F1(3), -F2(3));
            normalForce(normalForce <= 0) = 0;
            tangentForce = max(norm(F1(1:2)), norm(F2(1:2)));
            ratio = normalForce/tangentForce;
            
            path = [robots{iter, config}.centroid];
            distance = path(2,end) - path(2,1);
            pathLength = sum(vecnorm(diff(path, 1, 2)));
%             rawScores(iter, config) = distance/pathLength;
            rawScores(iter, 1) = normalForce;
            rawScores(iter, 2) = tangentForce;
            rawScores(iter, 3) = ratio;
            
            if PLOT
                if RECORD
                    figure(config);
                else
                    subplot(1, length(CONFIGURATIONS), config);
                end
                plotTerrain();
                plotRobot(robots{iter, config}(end));
                
                c = robot.centroid;
                k = 0.8;
                quiver3(f1(1), -f1(3), f1(2), F1(1)*k, -F1(3)*k, F1(2)*k,'linewidth', 2);
                quiver3(f2(1), -f2(3), f2(2), F2(1)*k, -F2(3)*k, F2(2)*k,'linewidth', 2);
                quiver3(n3(1), -n3(3), n3(2), N3(1)*k, -N3(3)*k, N3(2)*k,'linewidth', 2);
                quiver3(c(1), -c(3), c(2), grav(1)*k, -grav(3)*k, grav(2)*k,'linewidth', 2);
                
                plot3(path(1,:), -path(3,:), path(2,:),'k','linewidth', 2);
                title(LABELS{config});
                drawnow();
            end
        end
        sumScores(iSweep, :) = sumScores(iSweep, :) + ...
            [rawScores(iter, :), 1];
    end
    fprintf('Sweep: %.3f, Sample: %d, Scores: [', sweep, 1+mod(iter, SAMPLES));
    fprintf('%.3f, ', rawScores(iter, :));
    fprintf(']\n');
end

% EVALUATE CUMULATIVE RESULTS
meanScores = sumScores(:, 1:end-1)./sumScores(:, end);
if size(meanScores,1) > 1
    if RECORD
        figure(length(CONFIGURATIONS)+1);
    elseif PLOT || ANIMATE
        figure(2);
    end
    for config = 1:size(rawScores, 2)
        plot(SWEEP, meanScores(:,config), COLORS(config), 'linewidth', 3);
        hold on;
    end
    xlabel('Orientation (degrees)');
    ylabel('Force');
    plot(SWEEP, max(0,-cosd(SWEEP)/2), 'b--', 'linewidth', 3);
    plot(SWEEP, abs(sind(SWEEP)/2), 'r--', 'linewidth', 3);
    legend(LABELS);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55

% TERRAIN GENERATION FUNCTIONS

% Generates random terrain composed of planar segments
function [xq, yq, zq, dzdx, dzdy] = terrain(x, y, res, slope, roughness, corner)
    % Fine height grid
    [xq,yq] = meshgrid(x(1):res:x(2), y(1):res:y(2));
    zq = zeros(size(xq));
    for i = 1:length(slope)
        % Sparse height grid
        [X,Y] = meshgrid(x(1):roughness(i):x(2), y(1):roughness(i):y(2));
        z = (slope(i)*roughness(i))*rand(size(X));
        zq = zq + griddata(X,Y,z,xq,yq);
    end
    
    zq = zq + corner*yq.*(yq>0) - corner*yq.*(yq<0);
    
    % Slopes
    dzdy = diff(zq)/res;
    dzdx = diff(zq')'/res;
    % Trim off-by-one indices
    xq = xq(1:end-1, 1:end-1);
    yq = yq(1:end-1, 1:end-1);
    zq = zq(1:end-1, 1:end-1);
    dzdx = dzdx(1:end-1, :);
    dzdy = dzdy(:, 1:end-1);
end

% Find value of function Z at (x,y) by rounding indices
function z = f(x, y, Z)
    global X Y
    if x > max(X(1,:))
        x = max(X(1,:));
    elseif x < min(X(1,:))
        x = min(X(1,:));
    end
    if y > max(Y(:,1))
        y = max(Y(:,1));
    elseif y < min(Y(:,1))
        y = min(Y(:,1));
    end
    i = round(1+(x-X(1,1))/(X(2,2)-X(1,1)));
    j = round(1+(y-Y(1,1))/(Y(2,2)-Y(1,1)));
    z = Z(sub2ind(size(Z), j, i));
end

% VISUALIZATION FUNCTIONS

function plotTerrain()
    global X Y Z PLOT
    if ~PLOT
        return
    end
    cla();
    mesh(X,-Z,Y);
    hold on
    axis equal
end

function plotRobot(r)
    global PLOT
    if ~PLOT
        return
    end
    fill3(r.body(1,:),-r.body(3,:), r.body(2,:), 'r');
    plotPoints(r.feet(:,r.c), 'b.');
    plotPoints(r.feet(:,~r.c), 'r.');
    for i = 1:size(r.feet,2)
        if r.dof(i) == 3
            knee = getKnee(r.shoulders(:,i), r.feet(:,i), r.L2, r.L3, r.R);
            plotLine(r.feet(:,i), knee, 'b');
            plotLine(r.shoulders(:,i), knee, 'b');
        else
            plotLine(r.feet(:,i), r.shoulders(:,i), 'b');
        end
    end
end

function g = plotLine(p1, p2, c)
     g = plot3([p1(1), p2(1)],...
          -[p1(3), p2(3)],...
           [p1(2), p2(2)], c, 'linewidth', 2);
end

function g = plotPoints(p, c)
     g = plot3(p(1,:),-p(3,:), p(2,:), c, 'markersize', 20);
end

function animateStep(r1, r2, dt, odd, window)
    global FRAMES RECORD ANIMATE
    if ~ANIMATE && ~RECORD
        return
    end
    dc = r2.centroid - r1.centroid;
    df = r2.feet - r1.feet;
    plotTerrain();
    p = mod((0:3)+odd,2);
    plotPoints(r1.feet(:,r1.c&p), 'b.');
    plotPoints(r1.feet(:,~r1.c&p), 'r.');
    c = ['b', 'r'];
    for t = 0:dt:1
        g.body = fill3(r1.body(1,:)+t*dc(1), -r1.body(3,:)-t*dc(3), r1.body(2,:)+t*dc(2), 'r');
        for i = 1:size(r1.feet,2)
            ci = c(mod(i+odd,2)+1);
            if r1.dof(i) == 3
                knee = getKnee(r1.shoulders(:,i)+t*dc, r1.feet(:,i)+t*df(:,i), r1.L2, r1.L3, r1.R);
                g.feet(i) = plotLine(r1.feet(:,i)+t*df(:,i), knee, ci);
                g.feet(i+4) = plotLine(r1.shoulders(:,i)+t*dc, knee, ci);
            else
                g.feet(i) = plotLine(r1.feet(:,i)+t*df(:,i), ...
                                     r1.shoulders(:,i)+t*dc, ci);
            end
        end
        drawnow();
        if RECORD
            FRAMES{window}(length(FRAMES{window})+1) = getframe;
        end
        if t ~= 1
            delete(g.body);
            for i = 1:length(g.feet)
                delete(g.feet(i));
            end
        end
    end
end

function knee = getKnee(shoulder, foot, L1, L2, R)
    leg = R'*(foot-shoulder);
    r = norm(leg(1:2));
    z = leg(3);
    r1 = (r^3+r*z^2+r*L1^2-r*L2^2-z*sqrt(-r^4-z^4-(L1^2-L2^2)^2+2*z^2*...
         (L1^2+L2^2)+2*r^2*(-z^2+L1^2+L2^2)))/(2*(r^2+z^2));
    z1 = (z^3+z*r^2+z*L1^2-z*L2^2+r*sqrt(-r^4-z^4-(L1^2-L2^2)^2+2*z^2*...
         (L1^2+L2^2)+2*r^2*(-z^2+L1^2+L2^2)))/(2*(r^2+z^2));
    knee = R*[r1*leg(1:2)/r; z1] + shoulder;
end

% ROBOT KINEMATICS FUNCTIONS

% Instantiate a new robot at given location
function r = placeRobot(centroid, stepFunc, sweep)
    global Z
    r.w = 0.1; % width (m)
    r.h = 0.3; % height (m)
    r.L1 = 0.2; % arm length (m)
    r.L2 = r.L1*.8; % arm upper segment length (m)
    r.L3 = r.L1*.8; % arm lower segment (m)
    r.clearance = r.L1*.2;
    r.dx = 0.07; % step size (m)
    r.heading = 0; % desired yaw (radians)
    r.dyaw = deg2rad(5); % maximum yaw per step (radians)
    r.ifeet = [1 2 4 5];
    
    r.R = eye(3); % rotation matrix from robot to world frame
    r.alpha = deg2rad(0); % body joint anglge
    r.Ralpha = vrrotvec2mat([1 0 0 r.alpha]);
    % FL FR R BR BL L
    r.body = [-r.w/2,r.w/2,r.w/2,r.w/2,-r.w/2,-r.w/2; 
              r.h/2*cos(r.alpha/2),r.h/2*cos(r.alpha/2),0,...
                    -r.h/2*cos(r.alpha/2),-r.h/2*cos(r.alpha/2),0;
              0,0,r.h/2*sin(r.alpha/2),0,0,r.h/2*sin(r.alpha/2)] + centroid;
    % Adjust initial orientation
    z = f(r.body(1, r.ifeet), r.body(2, r.ifeet), Z);
    pitch = atan2(z(1)+z(2)-z(3)-z(4),2*r.h);
    roll = atan2(z(1)-z(2)-z(3)+z(4),2*r.w);
    [r.body, r.R] = rotate(r.body, [1 0 0 pitch], centroid, r.R);
    [r.body, r.R] = rotate(r.body, [0 1 0 roll], centroid, r.R);
    % Adjust initial height
    r.body(3,:) = r.body(3,:) + 0.05*cos(pitch)*cos(roll) + ...
        max(f([centroid(1),r.body(1,:)], [centroid(2),r.body(2,:)], Z));
    r.feet = r.body(:,r.ifeet) + r.R*[-r.L1, r.L1, r.L1, -r.L1; 
                                      -r.dx r.dx -r.dx r.dx;
                                      0 0 0 0];
    % Place feet
    for i = [2,4]
        [~, r.feet(:,i), r.c(i)] = contact(r.feet(:,i),[0;1;0], ...
                                           r.body(:,r.ifeet(i)), Z, r.R);
    end
    r = update(r);
    r = stepFunc(r, 0, Z);
end

function r = step(r, odd, Z)
    r.dof = [3 3 2 2];
    r.heading = 2*r.centroid(1);
    r.body = r.body + r.R*[0; r.dx*2; 0];
    r = update(r);
    r = bodyPitchJoint(r, -r.alpha);
%     r = bodyPitchJoint(r, deg2rad(45)*(abs(r.body(2,3))<0.1));
%     r = bodyPitchJoint(r, -deg2rad(45)*(abs(r.body(2,3))<0.15)*(r.body(2,3)<-0.05));
    headingVec = (r.shoulders(:,1) + r.shoulders(:,2) - r.shoulders(:,3) - r.shoulders(:,4))/2;  
    heading = atan2(-headingVec(1), headingVec(2));
    yawErr = max(-r.dyaw, min(r.dyaw, r.heading-heading));
    r = yaw(r, yawErr, odd);
    
    if odd
        r.feet(:,2) = r.shoulders(:,2) + r.R*r.Ralpha*[r.L1; r.dx; 0];
        r.feet(:,4) = r.shoulders(:,4) + r.R*[-r.L1; r.dx; 0];
        [~, r.feet(:,2), r.c(2)] = contact(r.feet(:,2),[0;1;0],r.body(:,2), Z, r.R*r.Ralpha);
        [~, r.feet(:,4), r.c(4)] = contact(r.feet(:,4),[0;1;0],r.body(:,5), Z, r.R);
    else
        r.feet(:,1) = r.shoulders(:,1) + r.R*r.Ralpha*[-r.L1; r.dx; 0];
        r.feet(:,3) = r.shoulders(:,3) + r.R*[r.L1; r.dx; 0];
        [~, r.feet(:,1), r.c(1)] = contact(r.feet(:,1),[0;1;0],r.body(:,1), Z, r.R*r.Ralpha);
        [~, r.feet(:,3), r.c(3)] = contact(r.feet(:,3),[0;1;0],r.body(:,4), Z, r.R);
    end
    
    lastError = 1000;
    count = 0;
    fail = 0;
    
    for i = 1:200
        count = count + 1;
        if odd
            r.feet(:,2) = r.shoulders(:,2) + r.R*r.Ralpha*[r.L1; r.dx; 0];
            r.feet(:,4) = r.shoulders(:,4) + r.R*[-r.L1; r.dx; 0];
            [~, r.feet(:,2), r.c(2)] = contact(r.feet(:,2),[0;1;0],r.body(:,2), Z, r.R*r.Ralpha);
            [~, r.feet(:,4), r.c(4)] = contact(r.feet(:,4),[0;1;0],r.body(:,5), Z, r.R);
            frontleg = 2;
        else
            r.feet(:,1) = r.shoulders(:,1) + r.R*r.Ralpha*[-r.L1; r.dx; 0];
            r.feet(:,3) = r.shoulders(:,3) + r.R*[r.L1; r.dx; 0];
            [~, r.feet(:,1), r.c(1)] = contact(r.feet(:,1),[0;1;0],r.body(:,1), Z, r.R*r.Ralpha);
            [~, r.feet(:,3), r.c(3)] = contact(r.feet(:,3),[0;1;0],r.body(:,4), Z, r.R);
            frontleg = 1;
        end
        
%         plotRobot(r);
        r = update(r);
        
        % Terrain compensation
        if i < 200
            % Front leg compensation with pitch/roll
            legF = r.Ralpha'*r.R'*r.legs(:,frontleg);
            legB = r.R'*r.legs(:,frontleg+2);
            error = legF(3) - legB(3);
            % Back leg compensation with body X-Z
            leg = r.R'*r.legs(:,frontleg + 2);
            errorBack = leg(3) + 0.15;
            if ~r.c(frontleg+2)
                if f(r.feet(1,frontleg+2),r.feet(2,frontleg+2), Z) < r.feet(3,frontleg+2)
                    errorBack = -0.1;
                else
                    errorBack = 0.1;
                end
            end
            leg2 = r.R'*r.legs(:,5-frontleg);
            % Check for convergence
            currError = max(abs(errorBack), abs(error));
            if abs(lastError - currError) < 0.001 && r.c(frontleg+2) && r.c(frontleg)
                break
            end
            lastError = currError;
    
            % Move
            r = pitchRoll(r, -error*2, odd);
            if (odd && leg2(1) > 0) || (~odd && leg2(1) < 0)
                r = moveX(r, errorBack*2, odd);
            end
        end
        if ~(r.c(frontleg) && r.c(frontleg+2))
            fail = i;
        end
    end
%     if count > 20
%         fail
%         count
%     end
    % Front leg extension
    if ~r.c(frontleg)
        [r.feet(:,frontleg), r.c(frontleg)] = extend(r.feet(:,frontleg),...
            r.shoulders(:,frontleg), r.L2+r.L3, Z);
        r = update(r);
    end
    
    % Validate
%     for i = 1:2
%         leg = r.R'*r.legs(:,i);
%         if norm(leg) > r.L2+r.L3
%             norm(leg)
%         end
%     end
end

function r = step12DOF(r, odd, Z)
    r.dof = [3 3 3 3];
    r.heading = 2*r.centroid(1);
    r.body = r.body + r.R*[0; r.dx*2; 0];
    r = update(r);
    headingVec = (r.shoulders(:,1) + r.shoulders(:,2) - r.shoulders(:,3) - r.shoulders(:,4))/2;  
    heading = atan2(-headingVec(1), headingVec(2));
    yawErr = max(-r.dyaw, min(r.dyaw, r.heading-heading));
    r = yaw(r, yawErr, odd);
    
    % Lower feet
    if odd
        r.feet(:,2) = r.shoulders(:,2) + r.R*r.Ralpha*[r.L1; r.dx; 0];
        r.feet(:,4) = r.shoulders(:,4) + r.R*[-r.L1; r.dx; 0];
        [~, r.feet(:,2), r.c(2)] = contact(r.feet(:,2),[0;1;0],r.body(:,2), Z, r.R*r.Ralpha);
        [~, r.feet(:,4), r.c(4)] = contact(r.feet(:,4),[0;1;0],r.body(:,5), Z, r.R);
        frontleg = 2;
    else
        r.feet(:,1) = r.shoulders(:,1) + r.R*r.Ralpha*[-r.L1; r.dx; 0];
        r.feet(:,3) = r.shoulders(:,3) + r.R*[r.L1; r.dx; 0];
        [~, r.feet(:,1), r.c(1)] = contact(r.feet(:,1),[0;1;0],r.body(:,1), Z, r.R*r.Ralpha);
        [~, r.feet(:,3), r.c(3)] = contact(r.feet(:,3),[0;1;0],r.body(:,4), Z, r.R);
        frontleg = 1;
    end
    
    lastError = 1000;
    count = 0;
    fail = 0;
    
    for i = 1:200
        count = count + 1;
        % Contact ground
        if odd
            r.feet(:,2) = r.shoulders(:,2) + r.R*r.Ralpha*[r.L1; r.dx; 0];
            r.feet(:,4) = r.shoulders(:,4) + r.R*[-r.L1; r.dx; 0];
            [~, r.feet(:,2), r.c(2)] = contact(r.feet(:,2),[0;1;0],r.body(:,2), Z, r.R*r.Ralpha);
            [~, r.feet(:,4), r.c(4)] = contact(r.feet(:,4),[0;1;0],r.body(:,5), Z, r.R);
        else
            r.feet(:,1) = r.shoulders(:,1) + r.R*r.Ralpha*[-r.L1; r.dx; 0];
            r.feet(:,3) = r.shoulders(:,3) + r.R*[r.L1; r.dx; 0];
            [~, r.feet(:,1), r.c(1)] = contact(r.feet(:,1),[0;1;0],r.body(:,1), Z, r.R*r.Ralpha);
            [~, r.feet(:,3), r.c(3)] = contact(r.feet(:,3),[0;1;0],r.body(:,4), Z, r.R);
        end
        r = update(r);
        % Extend swing legs
        for l = 1:4
            if ~r.c(l)
                [r.feet(:,l), r.c(l)] = extend(r.feet(:,l),...
                    r.shoulders(:,l), r.L2+r.L3, Z);
                r = update(r);
            end
        end
%         plotRobot(r);
        
        % Terrain compensation
        if i < 200
            % Front leg compensation with pitch
            legF = r.Ralpha'*r.R'*r.legs(:,frontleg);
            legB = r.R'*r.legs(:,frontleg+2);
            legF2 = r.Ralpha'*r.R'*r.legs(:,3-frontleg);
            legB2 = r.R'*r.legs(:,5-frontleg);
            error = legF(3) - legB(3);
            % Back leg compensation with body Z
            errorZ = (legF(3) + legB(3) + legF2(3) + legB2(3) + 0.6)/4;
            % Check for convergence
            currError = max(abs(errorZ), abs(error));
            if abs(lastError - currError) < 0.001 && r.c(frontleg+2) && r.c(frontleg)
                break
            end
            lastError = currError;
    
            % Move
            r = pitchBody(r, -error*2);
            r = moveBody(r, [0;0;errorZ]);
        end
        if ~(r.c(frontleg) && r.c(frontleg+2))
            fail = i;
        end
    end
    % Extend swing legs
    for l = 1:4
        if ~r.c(l)
            [r.feet(:,l), r.c(l)] = extend(r.feet(:,l),...
                r.shoulders(:,l), r.L2+r.L3, Z);
            r = update(r);
        end
    end
    if count > 20
%         fail
%         count
    end
end

function r = solveStep(r, odd, Z)
    r.heading = 2*r.centroid(1);
    r.heading
    r.body = r.body + r.R*[0; r.dx*2; 0];
    r = update(r);
    r = bodyPitchJoint(r, -r.alpha);
    headingVec = (r.shoulders(:,1) + r.shoulders(:,2) - r.shoulders(:,3) - r.shoulders(:,4))/2;  
    heading = atan2(-headingVec(1), headingVec(2));
    yawErr = max(-r.dyaw, min(r.dyaw, r.heading-heading));
    r = yaw(r, yawErr, odd);
    
    if odd
        r.feet(:,2) = r.shoulders(:,2) + r.R*r.Ralpha*[r.L1; r.dx; 0];
        r.feet(:,4) = r.shoulders(:,4) + r.R*[-r.L1; r.dx; 0];
        [~, r.feet(:,2), r.c(2)] = contact(r.feet(:,2),[0;1;0],r.body(:,2), Z, r.R*r.Ralpha);
        [~, r.feet(:,4), r.c(4)] = contact(r.feet(:,4),[0;1;0],r.body(:,5), Z, r.R);
    else
        r.feet(:,1) = r.shoulders(:,1) + r.R*r.Ralpha*[-r.L1; r.dx; 0];
        r.feet(:,3) = r.shoulders(:,3) + r.R*[r.L1; r.dx; 0];
        [~, r.feet(:,1), r.c(1)] = contact(r.feet(:,1),[0;1;0],r.body(:,1), Z, r.R*r.Ralpha);
        [~, r.feet(:,3), r.c(3)] = contact(r.feet(:,3),[0;1;0],r.body(:,4), Z, r.R);
    end
    
    T0 = (r.shoulders(:,1) + r.shoulders(:,2))/2 - r.centroid;
    N0 = (r.shoulders(:,1) + r.shoulders(:,4))/2 - r.centroid;
    X0 = [r.centroid; T0; N0; reshape(r.legs, [12,1])];
    normal = 100*N0(1:2);
    if odd
        Aeq = [eye(3), eye(3), eye(3), eye(3), zeros(3,9);
               eye(3), -eye(3), -eye(3), zeros(3,6), eye(3), zeros(3);
               zeros(1,3), normal',0, zeros(1,15);
               0 1 0, 0 1 0, 0 -1 0, zeros(1,3), 0 1 0, zeros(1,6);
               0 1 0, 0 -1 0, 0 1 0, zeros(1,9), 0 1 0];
        beq = [r.feet(:,1); r.feet(:,3); 0; r.feet(2, 2); r.feet(2, 4)];
    else
        Aeq = [eye(3), eye(3), -eye(3), zeros(3), eye(3), zeros(3,6);
               eye(3), -eye(3), eye(3), zeros(3,9), eye(3);
               zeros(1,3), normal',0, zeros(1,15);
               0 1 0, 0 1 0, 0 1 0, 0 1 0, zeros(1,9);
               0 1 0, 0 -1 0, 0 -1 0, zeros(1,6), 0 1 0, zeros(1, 3)];
        beq = [r.feet(:,2); r.feet(:,4); 0; r.feet(2, 1); r.feet(2, 3)];
    end
    options = optimoptions('fmincon','MaxFunctionEvaluations',1e4,'Algorithm','interior-point','ConstraintTolerance',1e-3);
    delta = [r.w; r.w; r.h];
    b1 = [r.L2+r.L3; r.L2+r.L3; Inf];
    b2 = [0; r.L2+r.L3; Inf];
    lb = [r.centroid-delta;-1;-1;-1;-1;-1;-1;-b1;-b2;-b2;-b1];
    ub = [r.centroid+delta;1;1;1;1;1;1;b2;b1;b1;b2];
    [x,~,~,output] = fmincon(@(x)cost(x,r),X0,[],[],Aeq,beq,lb,ub,@(x)constraints(x,r,odd),options);
    if output.constrviolation < options.ConstraintTolerance
        r.c = [1 1 1 1];
    else
        r.c = ~[1 1 1 1];
    end
    output.constrviolation
    [cFinal,ceqFinal] = constraints(x, r, odd)
    costFinal = cost(x, r)
    linearFinal = Aeq*x - beq
    
    r.body(:,1) = x(1:3)+x(4:6)+x(7:9);
    r.body(:,2) = x(1:3)+x(4:6)-x(7:9);
    r.body(:,4) = x(1:3)-x(4:6)-x(7:9);
    r.body(:,5) = x(1:3)-x(4:6)+x(7:9);
    r.body(:,3) = (r.body(:,2) + r.body(:,4))/2;
    r.body(:,6) = (r.body(:,1) + r.body(:,5))/2;
    for i = 1:4
        r.feet(:, i) = r.body(:,r.ifeet(i))+x(i*3+7:i*3+9);
    end
    B = cross(x(4:6), x(7:9));
    r.R = [-x(7:9)/norm(x(7:9)), x(4:6)/norm(x(4:6)), B/norm(B)];
    r = update(r);
end

function c = cost(x, r)
%     global Z
%     z = f(r.centroid(1), r.centroid(2), Z);
    c = norm(r.centroid(1:2) - x(1:2));
    return
%     B = cross(x(4:6), x(7:9));
    body = [x(1:3)+x(4:6)+x(7:9);
            x(1:3)+x(4:6)-x(7:9);
            x(1:3)-x(4:6)-x(7:9);
            x(1:3)-x(4:6)+x(7:9);
            x(1:3)+x(7:9);
            x(1:3)-x(7:9)];
    z = [f(body(1), body(2), Z);f(body(4), body(5), Z);
         f(body(7), body(8), Z);f(body(10), body(11), Z);
         f(body(13), body(14), Z);f(body(16), body(17), Z)];
%     legs = [x(10:12)'*B; x(13:15)'*B; x(16:18)'*B; x(19:21)'*B]./norm(B);
    c = sum((body([3,6,9,12,15,18])-z-r.clearance).^2);
end

function [c,ceq] = constraints(x, r, odd)
    global Z
    if odd
        foot1 = x(1:3)+x(4:6)-x(7:9)+x(13:15); %2
        foot3 = x(1:3)-x(4:6)+x(7:9)+x(19:21); %4
    else
        foot1 = x(1:3)+x(4:6)+x(7:9)+x(10:12);
        foot3 = x(1:3)-x(4:6)-x(7:9)+x(16:18);
    end
    ceq = [%norm(x(16:18))-r.L1;
%            norm(x(19:21))-r.L1;
           x(4:6)'*x(7:9);
           norm(x(4:6))-r.h/2;
           norm(x(7:9))-r.w/2;
           foot1(3)-f(foot1(1), foot1(2), Z);
           foot3(3)-f(foot3(1), foot3(2), Z)];
%     lb = [min(X(1,:));min(Y(:,1));-Inf];
%     ub = [max(X(1,:));max(Y(:,1));Inf];

    
    body = [x(1:3)+x(4:6)+x(7:9);
            x(1:3)+x(4:6)-x(7:9);
            x(1:3)-x(4:6)-x(7:9);
            x(1:3)-x(4:6)+x(7:9);
            x(1:3)+x(7:9);
            x(1:3)-x(7:9)];
    z = [f(body(1), body(2), Z);f(body(4), body(5), Z);
         f(body(7), body(8), Z);f(body(10), body(11), Z);
         f(body(13), body(14), Z);f(body(16), body(17), Z)];
    B = cross(x(4:6), x(7:9));

    c = [norm(x(10:12))-(r.L2+r.L3);
         norm(x(13:15))-(r.L2+r.L3);
         norm(x(16:18))-(r.L2+r.L3);
         norm(x(19:21))-(r.L2+r.L3);
         z-body([3,6,9,12,15,18])+r.clearance;
         -B(3)];
%          [lb;lb;lb;lb]-x(10:21);
%          x(10:21)-[ub;ub;ub;ub]];
end

function robot = update(robot)
    i = robot.ifeet;
    robot.centroid = (robot.body(:,i(1))+robot.body(:,i(2))+robot.body(:,i(3))+robot.body(:,i(4)))/4;
    robot.shoulders = robot.body(:,i);
    robot.legs = robot.feet - robot.shoulders;
end

% Rotate body by rvec in given frame around origin and return new frame
function [body, frame] = rotate(body, rvec, origin, frame)
    R = vrrotvec2mat(rvec);
    body = frame*R*frame'*(body-origin)+origin;
    frame = frame*R;
end

% Rotate leg around axis until foot is contacting surface
function [angle, foot, hit] = contact(foot, axis, origin, Z, R)
    df = rotate(foot, [axis; 0.001], origin, R)-foot;
    if df(3) < 0
        axis = -axis;
    end
    angle = 0;
    up = 0;
    down = 0;
    for dtheta = pi/2*2.^(-1:-1:-10)
        z = f(foot(1), foot(2), Z);
        if foot(3) > z
            foot = rotate(foot, [axis; -dtheta], origin, R);
            angle = angle - dtheta;
            up = 1;
        else
            foot = rotate(foot, [axis; dtheta], origin, R);
            angle = angle + dtheta;
            down = 1;
        end
    end
    hit = up && down;
end

function [foot, hit] = extend(foot, shoulder, Lmax, Z)
    leg = foot-shoulder;
    dLmax = Lmax - norm(leg);
    up = 0;
    down = 0;
    for dL = dLmax*2.^(-1:-1:-10)
        z = f(foot(1), foot(2), Z);
        if foot(3) > z
            foot = foot + leg*dL/norm(leg);
            up = 1;
        else
            foot = foot - leg*dL/norm(leg);
            down = 1;
        end
    end
    hit = up && down;
end

% Rotate body without extending/retracting front leg
function r = pitchRoll(r, angle, odd)
    if odd
        axis = -r.R'*(r.feet(:,3)-r.feet(:,1));
        [r.body, r.R] = rotate(r.body, [axis; angle], r.feet(:,3), r.R);
    else
        axis = r.R'*(r.feet(:,4)-r.feet(:,2));
        [r.body, r.R] = rotate(r.body, [axis; angle], r.feet(:,4), r.R);
    end
    r = update(r);
end

% Pivot body around back feet
function r = pitchBody(r, angle)
    axis = -r.R'*(r.body(:,4)-r.body(:,5));
    [r.body, r.R] = rotate(r.body, [axis; angle], r.body(:,4), r.R);
    r = update(r);
end

% Raise or lower body relative to feet
function r = moveBody(r, dx)
    r.body = r.body + dx;
    r = update(r);
end

function r = yaw(r, angle, odd)
    if odd
        [r.body, r.R] = rotate(r.body, [0 0 1 angle], r.feet(:,3), r.R);
    else
        [r.body, r.R] = rotate(r.body, [0 0 1 angle], r.feet(:,4), r.R);
    end
    r = update(r);
end

function r = bodyPitchJoint(r, angle)
    r.alpha = r.alpha + angle;
    r.Ralpha = vrrotvec2mat([1 0 0 angle])*r.Ralpha;
    % FL FR R BR BL L
    [r.body(:,1:2), ~] = rotate(r.body(:,1:2), [1 0 0 angle], r.body(:,3), r.R);
end

% Moves laterally by pivoting on back foot
function r = moveX(r, angle, odd)
    if odd
        [shoulder, ~] = rotate(r.shoulders(:,3), [0 1 0 angle], r.feet(:,3), r.R);
        dx = shoulder - r.shoulders(:,3);
        r.body = r.body + dx;
    else
        [shoulder, ~] = rotate(r.shoulders(:,4), [0 -1 0 angle], r.feet(:,4), r.R);
        dx = shoulder - r.shoulders(:,4);
        r.body = r.body + dx;
    end
    r = update(r);
end

% QUASI-STATIC DYNAMICS

% Compute forces on feet given displacement from CoM and gravity vector
function [F1, F2, N3] = forces(r1, r2, r3, N, DIG, g)
    A = [eye(3), eye(3), N;
         [cross(r1', [1 0 0]),  cross(r2', [1 0 0]);
          cross(r1', [0 1 0]),  cross(r2', [0 1 0]);
          cross(r1', [0 0 1]),  cross(r2', [0 0 1])], cross(N, r3);
          1, zeros(1,6)];
    b = [-g;zeros(3,1);DIG];
    F = A\b;
    F1 = F(1:3);
    F2 = F(4:6);
    N3 = F(7)*N;
%     max(abs(A*F - b))
%     F1 + F2 + N3 + g
%     cross(r1,F1) + cross(r2,F2) + cross(r3,N3)
end