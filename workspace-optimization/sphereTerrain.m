% Generates random terrain composed of planar segments
function grid = sphereTerrain(x, y, res, center, r, sign, spawn)
    [xq,yq] = meshgrid(x(1):res:x(2), y(1):res:y(2));
    zq = center(3) + sign*sqrt(r^2 - (xq-center(1)).^2 - (yq-center(2)).^2);
    zq(imag(zq)~=0) = center(3);
    zq(isnan(zq)) = center(3);
%     zq(zq*sign < 0) = 0;
    
    grid.obstacles = [];
    dzdy = diff(zq)/res;
    dzdx = diff(zq')'/res;
    grid.x = xq(1:end-1, 1:end-1);
    grid.y = yq(1:end-1, 1:end-1);
    grid.z = zq(1:end-1, 1:end-1);
    grid.dzdx = dzdx(1:end-1, :);
    grid.dzdy = dzdy(:, 1:end-1);
    grid.spawn = spawn;
    grid.seed = 0;
    grid.params = {x, y, res, center, r, sign, spawn};
    grid.f = @(x, y, Z) f(x, y, Z, grid);
    grid.contact = @contact;
end