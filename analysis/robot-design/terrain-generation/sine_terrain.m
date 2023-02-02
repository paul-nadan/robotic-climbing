% Generates random terrain composed of planar segments
function grid = sine_terrain(periods, magnitudes, phases, x, y)
    res = x(2)-x(1);
    [xq, yq] = meshgrid(x, y);
    periods(~periods) = inf;
    zq = interpolate(xq, yq, periods, magnitudes, phases);
    
    % Slopes
%     figure(); plot(xq(1, 1:end-1), zq(1, 1:end-1)); title('Unsmoothed'); xlabel('x'); ylabel('z');
%     zq = imgaussfilt(zq, 2);
    dzdy = diff(zq)/res;
    dzdx = diff(zq')'/res;
%     figure(); plot(xq(1, 1:end-1), zq(1, 1:end-1)); title('Unsmoothed'); xlabel('x'); ylabel('z');
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
    grid.obstacles = [];
    grid.params = {periods, magnitudes, phases, x, y};
    grid.f = @(x, y, Z) f(x, y, Z, grid);
    grid.contact = @contact;
end

function z = interpolate(x, y, periods, magnitudes, phases)
    z = x.*y*0;
    for i = 1:length(periods)
        z = z + magnitudes(1,i)*sin(2*pi*(y-phases(1,i))/periods(1,i));
        z = z - magnitudes(2,i)*sin(2*pi*(x-phases(2,i))/periods(2,i));
    end
end