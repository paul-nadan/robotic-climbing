N = 20;
offsets = [0:.05:.5; 0:.05:.5];
% offsets = [0:.05:.5];
sim = 1;

if sim
tic()
scores = zeros(size(offsets,2),N,size(offsets,2));
% scores = zeros(size(offsets,2),N);
for i = 1:size(scores,2)
    grid = terrain([-1, 1],[-1 1], .01, 1*[1,1,0.5], [1, .25, 0.0625], 0, [0;0;0], i);
    x0xy = grid.spawn(1:2)+rand(2,1)*.1;
    x0 = [x0xy; f(x0(1),x0(2),grid.z,grid)];
    for j = 1:size(scores,1)
        for k = 1:size(scores,1)
            offset1 = [-1;1]*offsets(1,j);
            offset2 = [-1;1]*offsets(2,k);
            [x1,f1] = optimizeGrid(grid, x0, x0(1)+offset1, x0(2)+offset1, offset2);
            scores(j,i,k) = f1;
%             scores(j,i) = f1;
        end
    end
end
toc()
end
% A = (offsets*2).^2;
% plot(A, s1);
% hold on
% plot(A, s2);
% legend('Original','Modified')
% xlabel('Area')
% ylabel('Cost')

% i = 44
% grid = terrain([-1, 1],[-1 1], .01, 1*[1,1,0.5], [1, .25, 0.0625], 0, [0;0;0], i);
% x0 = grid.spawn(1:2)+rand(2,1)*.1;
% offset1 = [-1;1]*0.2;
% offset2 = [-1;1]*0.2;
% [x1,f1] = optimizeGrid(grid, x0, x0(1), x0(2)+offset2);
% [x2,f2] = optimizeGrid(grid, x0, x0(1)+offset1, x0(2)+offset2);
% plotSolution(x0, x1, x2, grid);
% return

% improvement = scores(2:end,:,:)-scores(1,:,1);
% histogram(improvement,20);
close all;
% A = (offsets*2).^2;
% y = mean(scores,2);
% ft = fittype('a*exp(-b*sqrt(x))+c');
% f = fit(A', y, ft, 'StartPoint', [y(1)-y(end),0,y(end)])
% 
% plot(A,y);
% hold on;
% a = 0:.001:max(A);
% plot(a, workspace2Cost(a));
% xlabel('Workspace Area (m$^2$)');
% ylabel('Cost');
% title('Workspace vs Cost (1000 trials)');
% legend('Simulation Data', 'Exponential Fit');


% plot(offsets,mean(scores(:,:,1),2))
% hold on;
% plot(offsets,mean(scores(:,:,2),2))
% legend('2D','1D')
% xlabel('Workspace (m)')
% ylabel('Cost')
% title('Cost vs Workspace (single foot)')
% disp(mean(scores,2));
imagesc(offsets(1,:), offsets(2,:), squeeze(mean(scores,2))')
colorbar
xlabel('X Workspace (m)')
ylabel('Y Workspace (m)')
title('Cost vs Workspace (single foot)')

% failRate = sum(scores>1,2)/N;
% figure();
% plot(offsets,failRate(:,:,1))
% hold on;
% plot(offsets,failRate(:,:,2))
% ylim([0,max(failRate,[],'all')+.01]);
% xlabel('Workspace (m)');
% ylabel('Failure Rate');
% title('Failures vs Workspace (single foot)');
% 
% theta = -90:1:90;
% Ny = -1:.01:1;
% Fn = -sind(theta);
% Fn = -Ny;
% Ft = sqrt(1-Fn.^2);
% Fn = max(0, Fn);
% Ft = max(Ft, Fn/tand(20));
% cost = sqrt(Ft.^2+Fn.^2)*(50/3)/25;
% figure();
% plot(Ny, cost);
% xlabel('Vertical Component of Normal Vector');
% ylabel('Cost');
% title('Cost vs Wall Angle (single foot)');
% 
% 
% figure();
% plot(Ny, -Ny*(50/3), 'r--');
% hold on
% plot(Ny, Fn*(50/3), 'r');
% plot(Ny, sqrt(1-Ny.^2)*(50/3), 'b--');
% plot(Ny, Ft*(50/3), 'b');
% plot(Ny, cost*25, 'k');
% plot(Ny, Ny*0+25, 'k--');
% xlabel('Vertical Component of Normal Vector');
% ylabel('Force');
% legend('Normal Force', 'Adhesion Force', 'Tangential Force', 'Increased Tangential Force', 'Magnitude','Magnitude Limit')
% title('Force vs Wall Angle (single foot)');
   
function [x, fval] = optimizeGrid(grid, x0, xbound, ybound, zbound)

    options = optimoptions('fmincon','MaxFunctionEvaluations',1e4,...
        'Algorithm','sqp','ConstraintTolerance',1e-4,...
        'Display','off','SpecifyObjectiveGradient',false, ...
        'CheckGradients', false, 'FiniteDifferenceType', 'central');
    lb = [min(xbound), min(ybound), min(zbound)];
    ub = [max(xbound), max(ybound), max(zbound)];
    [x,fval,flag,output] = fmincon(@(x)costSlope(x,grid),x0,[],[],...
        [],[],lb,ub,@(x)constraints(x,grid),options);
end

function cost = costSlope(x, grid)
    N = [-f(x(1),x(2),grid.dzdx,grid); -f(x(1),x(2),grid.dzdy,grid); 1];
    N = N/norm(N);
    Fn = -N(2);
    Ft = sqrt(1-Fn^2);
    Fn = max(0, Fn);
    Ft = max(Ft, Fn/tand(20));
    cost = norm([Ft,Fn*1.01])*(50/3)/25;
%     cost = -N(2);
end

function [c, ceq] = constraints(x, grid)
    c = [];
    ceq = x(3) - f(x(1),x(2),grid.z,grid);
end

function plotSolution(x0, x1, x2, grid)
    z0 = f(x0(1), x0(2), grid.z, grid);
    z1 = f(x1(1), x1(2), grid.z, grid);
    z2 = f(x2(1), x2(2), grid.z, grid);

    figure();
    plotTerrain(grid);
    plot3(x0(1), -z0, x0(2), 'k.', 'MarkerSize', 20);
    plot3(x1(1), -z1, x1(2), 'r.', 'MarkerSize', 20);
    plot3(x2(1), -z2, x2(2), 'b.', 'MarkerSize', 20);
    
    N1 = getN(x1, grid);
    N2 = getN(x2, grid);
    scale = 0.2;
    quiver3(x1(1), -z1, x1(2), N1(1, end)*scale, -N1(3, end)*scale, ...
            N1(2, end)*scale, 'r', 'linewidth', 2, 'markersize', 10);
    quiver3(x2(1), -z2, x2(2), N2(1, end)*scale, -N2(3, end)*scale, ...
            N2(2, end)*scale, 'b', 'linewidth', 2, 'markersize', 10);
end

function N = getN(x, grid)
    N = [-f(x(1),x(2),grid.dzdx,grid); -f(x(1),x(2),grid.dzdy,grid); 1];
    N = N/norm(N);
end