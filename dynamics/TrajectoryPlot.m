close all
hold on
plot(t,x(:,1)-mean(x1(:,1)), 'r','linewidth',2)
plot(t,x(:,2)-mean(x1(:,2)), 'b','linewidth',2)
plot(t,x(:,3)-mean(x1(:,3)), 'g','linewidth',2)
plot(t,x1(:,1)-mean(x1(:,1)), 'r.','markersize',10)
plot(t,x1(:,2)-mean(x1(:,2)), 'b.','markersize',10)
plot(t,x1(:,3)-mean(x1(:,3)), 'g.','markersize',10)
plot(t,x1(:,1)-mean(x0(:,1)), 'r--','linewidth',1)
plot(t,x1(:,2)-mean(x0(:,2)), 'b--','linewidth',1)
plot(t,x1(:,3)-mean(x0(:,3)), 'g--','linewidth',1)
legend('Optimal x','Optimal y','Optimal z','Feasible x','Feasible y','Feasible z','Interpolated x','Interpolated y','Interpolated z')
title('Center of Mass Position')
xlabel('Time (s)')
ylabel('Position (m)')

iF = mod(iStep, size(r0.gait.angles, 2))+1; % next step in gait cycle
feet = r1.vertices(:, r0.gait.feet(:,iF) > 0);        

N = zeros(3,size(feet, 2));
for iFoot = 1:size(feet, 2)
    foot = feet(:,iFoot);
    N(:,iFoot) = [-f(foot(1),foot(2),grid.dzdx,grid);...
                   -f(foot(1),foot(2),grid.dzdy,grid); 1];
    N(:,iFoot) = N(:,iFoot)/norm(N(:,iFoot));
end

% Decompose forces into components
Fnorm = zeros(size(x,1),3);
Ftang = zeros(size(x,1),3);
Fmag = zeros(size(x,1),3);
FmagN = zeros(size(x,1),3);
fmat = x1(:,end-8:end);
for n = 1:size(x,1)
    F = reshape(fmat(n,:)',3,3);
    for iFoot = 1:size(feet,2)
        Fnorm(n, iFoot) = -F(:,iFoot)'*N(:,iFoot);
        Ftang(n, iFoot) = norm(cross(F(:,iFoot),N(:,iFoot)));
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
plot(t,Fmag(:,1), 'r','linewidth',2)
plot(t,Fmag(:,2), 'b','linewidth',2)
plot(t,Fmag(:,3), 'g','linewidth',2)
plot(t,Ftang(:,1), 'r--','linewidth',2)
plot(t,Ftang(:,2), 'b--','linewidth',2)
plot(t,Ftang(:,3), 'g--','linewidth',2)
plot(t,Fnorm(:,1), 'r:','linewidth',2)
plot(t,Fnorm(:,2), 'b:','linewidth',2)
plot(t,Fnorm(:,3), 'g:','linewidth',2)

legend('Force 1', 'Force 2', 'Force 3', 'Tangential 1', 'Tangential 2', ...
    'Tangential 3', 'Normal 1', 'Normal 2', 'Normal 3')
title('Contact Force')
xlabel('Time (s)')
ylabel('Force (N)')

figure()
hold on
plot(t,cost(:,1), 'r','linewidth',2)
plot(t,cost(:,2), 'b','linewidth',2)
plot(t,cost(:,3), 'g','linewidth',2)
plot(t,ratio(:,1), 'r--','linewidth',2)
plot(t,ratio(:,2), 'b--','linewidth',2)
plot(t,ratio(:,3), 'g--','linewidth',2)
plot(t,mag(:,1), 'r:','linewidth',2)
plot(t,mag(:,2), 'b:','linewidth',2)
plot(t,mag(:,3), 'g:','linewidth',2)

legend('Cost 1', 'Cost 2', 'Cost 3', 'Ratio Cost 1', 'Ratio Cost 2', ...
    'Ratio Cost 3', 'Magnitude Cost 1', 'Magnitude Cost 2', 'Magnitude Cost 3')

title('Cost')
xlabel('Time (s)')
ylabel('Cost')