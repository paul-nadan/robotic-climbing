close all
hold on
t_full = min(t):t(2)-t(1):max(t)*STEPS;
plot(t_full,x1(:,1)-mean(x1(:,1)), 'r','linewidth',2)
plot(t_full,x1(:,2)-mean(x1(:,2)), 'b','linewidth',2)
plot(t_full,x1(:,3)-mean(x1(:,3)), 'g','linewidth',2)
plot(t_full,x1(:,1)-mean(x1(:,1)), 'r.','markersize',10)
plot(t_full,x1(:,2)-mean(x1(:,2)), 'b.','markersize',10)
plot(t_full,x1(:,3)-mean(x1(:,3)), 'g.','markersize',10)
plot(t_full,x1(:,1)-mean(x0(:,1)), 'r--','linewidth',1)
plot(t_full,x1(:,2)-mean(x0(:,2)), 'b--','linewidth',1)
plot(t_full,x1(:,3)-mean(x0(:,3)), 'g--','linewidth',1)
legend('Optimal x','Optimal y','Optimal z','Feasible x','Feasible y','Feasible z','Interpolated x','Interpolated y','Interpolated z')
title('Center of Mass Position')
xlabel('Time (s)')
ylabel('Position (m)')

% Decompose forces into components
Fnorm = zeros(length(t_full),3);
Ftang = zeros(length(t_full),3);
Fmag = zeros(length(t_full),3);
FmagN = zeros(length(t_full),3);
fmat = x(:,end-9:end-1);
for n = 1:size(x,1)
    iStep = step0 + floor((n-1)/N);
    nStep = floor((n-1)/N)+1;
    iF = mod(iStep, size(robots(nStep).gait.angles, 2))+1; % next step in gait cycle
    feet = robots(nStep).vertices(:, robots(nStep).gait.feet(:,iF) > 0);

    Normals = zeros(3,size(feet, 2));
    for iFoot = 1:size(feet, 2)
        foot = feet(:,iFoot);
        Normals(:,iFoot) = [-f(foot(1),foot(2),grid.dzdx,grid);...
                       -f(foot(1),foot(2),grid.dzdy,grid); 1];
        Normals(:,iFoot) = Normals(:,iFoot)/norm(Normals(:,iFoot));
    end
    
    F = reshape(fmat(n,:),3,3);
    for iFoot = 1:size(feet,2)
        Fnorm(n, iFoot) = -F(:,iFoot)'*Normals(:,iFoot);
        Ftang(n, iFoot) = norm(cross(F(:,iFoot),Normals(:,iFoot)));
        Fmag(n, iFoot) = sqrt(Fnorm(n, iFoot).^2+Ftang(n, iFoot).^2);
        Fmag(n, iFoot) = sqrt(max(Fnorm(n, iFoot),0).^2+Ftang(n, iFoot).^2);
    end
end
ratio = Fnorm./Ftang/tand(20);
ratio = max(ratio, 0);
mag = Fmag/25;
cost = max(ratio, mag);

figure()
hold on
plot(t_full,Fmag(:,1), 'r','linewidth',2)
plot(t_full,Fmag(:,2), 'b','linewidth',2)
plot(t_full,Fmag(:,3), 'g','linewidth',2)
plot(t_full,Ftang(:,1), 'r--','linewidth',2)
plot(t_full,Ftang(:,2), 'b--','linewidth',2)
plot(t_full,Ftang(:,3), 'g--','linewidth',2)
plot(t_full,Fnorm(:,1), 'r:','linewidth',2)
plot(t_full,Fnorm(:,2), 'b:','linewidth',2)
plot(t_full,Fnorm(:,3), 'g:','linewidth',2)

legend('Force 1', 'Force 2', 'Force 3', 'Tangential 1', 'Tangential 2', ...
    'Tangential 3', 'Normal 1', 'Normal 2', 'Normal 3')
title('Contact Force')
xlabel('Time (s)')
ylabel('Force (N)')

figure()
hold on
plot(t_full,cost(:,1), 'r','linewidth',2)
plot(t_full,cost(:,2), 'b','linewidth',2)
plot(t_full,cost(:,3), 'g','linewidth',2)
plot(t_full,ratio(:,1), 'r--','linewidth',2)
plot(t_full,ratio(:,2), 'b--','linewidth',2)
plot(t_full,ratio(:,3), 'g--','linewidth',2)
plot(t_full,mag(:,1), 'r:','linewidth',2)
plot(t_full,mag(:,2), 'b:','linewidth',2)
plot(t_full,mag(:,3), 'g:','linewidth',2)

legend('Cost 1', 'Cost 2', 'Cost 3', 'Ratio Cost 1', 'Ratio Cost 2', ...
    'Ratio Cost 3', 'Magnitude Cost 1', 'Magnitude Cost 2', 'Magnitude Cost 3')

title('Cost')
xlabel('Time (s)')
ylabel('Cost')