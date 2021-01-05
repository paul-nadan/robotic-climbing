% Generates random terrain composed of planar segments
function grid = terrain(x, y, res, slope, roughness, corner, spawn, seed)
    % Set random seed
    if seed < 0
        rng('shuffle');
    else
        rng(seed, 'twister');
    end
    % Generate fine height grid
    [xq,yq] = meshgrid(x(1):res:x(2), y(1):res:y(2));
    zq = zeros(size(xq));
    for i = 1:length(slope)
        % Generate sparse height grids
        [X,Y] = meshgrid(x(1):roughness(i):x(2), y(1):roughness(i):y(2));
        z = (slope(i)*roughness(i))*rand(size(X));
        zq = zq + griddata(X,Y,z,xq,yq);
    end
    % Add corner
    zq = zq + corner*yq.*(yq>0) - corner*yq.*(yq<0);
    grid.obstacles = [];%[[-Inf, Inf; 0, 0.2; -Inf, 0.4],[-Inf, Inf; -0.2, 0; -Inf, 0.25],[-Inf, Inf; 0.2, 0.4; -Inf, 0.25]];
    % Slopes
%     figure(); plot(xq(1, 1:end-1), zq(1, 1:end-1)); title('Unsmoothed'); xlabel('x'); ylabel('z');
%     zq = imgaussfilt(zq, 2);
    dzdy = diff(zq)/res;
    dzdx = diff(zq')'/res;
%     figure(); plot(xq(1, 1:end-1), dzdx(1,:)); title('Unsmoothed'); xlabel('x'); ylabel('dzdx');
%     zq = imgaussfilt(zq, 2);
%     dzdx = diff(zq')'/res;
%     figure(); plot(xq(1, 1:end-1), zq(1, 1:end-1)); title('Smoothed'); xlabel('x'); ylabel('z');
%     figure(); plot(xq(1, 1:end-1), dzdx(1,:)); title('Smoothed'); xlabel('x'); ylabel('dzdx');
    % Trim off-by-one indices
    grid.x = xq(1:end-1, 1:end-1);
    grid.y = yq(1:end-1, 1:end-1);
    grid.z = zq(1:end-1, 1:end-1);
    grid.dzdx = dzdx(1:end-1, :);
    grid.dzdy = dzdy(:, 1:end-1);
    grid.spawn = spawn;
    grid.seed = seed;
    grid.params = {x, y, res, slope, roughness, corner};
    grid.f = @(x, y, Z) f(x, y, Z, grid);
    grid.contact = @contact;
end