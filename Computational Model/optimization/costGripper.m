% Gripper adhesion metric
function [c, g] = costGripper(x, rd, i, grid)
    iF = mod(i(1), size(rd(1).gait.angles, 2))+1; % next step in gait cycle
    nF = 3*sum(rd(2).gait.feet(:,iF) > 0); % number of force variables
    xF = x(end-nF+1:end); % force components
    Nx = (length(x)-nF)/(length(rd)-1); % length of robot state vector
    robot = state2robot(x(1:Nx), rd(1).config);
    feet = robot.vertices(:, robot.gait.feet(:,iF) > 0);
    
    F = reshape(xF, 3, []);
    
    % Find normal vectors
    N = zeros(3,size(feet, 2));
    for iFoot = 1:size(feet, 2)
        foot = feet(:,iFoot);
        N(:,iFoot) = [-f(foot(1),foot(2),grid.dzdx,grid);...
                       -f(foot(1),foot(2),grid.dzdy,grid); 1];
        N(:,iFoot) = N(:,iFoot)/norm(N(:,iFoot));
    end
    
    % Decompose forces into components
    Fnorm = zeros(1,size(feet,2));
    Ftang = zeros(1,size(feet,2));
    for iFoot = 1:size(feet,2)
        Fnorm(iFoot) = -F(:,iFoot)'*N(:,iFoot);
        Ftang(iFoot) = norm(cross(F(:,iFoot),N(:,iFoot)));
        Fnorm(iFoot) = max(0, Fnorm(iFoot));
    end
    
    % Compute margin
    c = gripperMargin(Fnorm, Ftang);
    g = zeros(size(x));
end