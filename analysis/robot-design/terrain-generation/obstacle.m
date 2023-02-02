% Generates random terrain composed of planar segments
function grid = obstacle(x, y, res, height, slope, spawn, seed)
    % Set random seed
    if seed < 0
        rng('shuffle');
    else
        rng(seed, 'twister');
    end
    % Generate fine height grid
    [xq,yq] = meshgrid(x(1):res:x(2), y(1):res:y(2));
    zq = zeros(size(xq));

    % Add step
%     zq = zq + height*(yq-res*3>0);

    % Add bump
    if height~=0
        yrad = (yq-res*2)/abs(height) - 1;
        yrad(abs(yrad)>1) = 1;
        zq = zq + height*sqrt(1-yrad.^2);
    end
    
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
    grid.params = {x, y, res, height, slope};
    grid.f = @(x, y, Z) f(x, y, Z, grid);
    grid.contact = @contact;
    grid.obstacles = [];
end