% Find value of function Z at (x,y) by interpolation
function z = f(x, y, Z, grid)
    X = grid.x;
    Y = grid.y;
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
    dx = X(2,2)-X(1,1);
    x0 = x-X(1,1);
    y0 = y-Y(1,1);    
    i1 = floor(1+x0/dx);
    j1 = floor(1+y0/dx);
    i2 = ceil(1+x0/dx);
    j2 = ceil(1+y0/dx);
    c = Z(sub2ind(size(Z), [j1; j1; j2; j2], [i1; i2; i1; i2]));
    xp = x0/dx - i1 + 1;
    yp = y0/dx - j1 + 1;
    y1 = c(1,:).*(1-xp) + c(2,:).*xp;
    y2 = c(3,:).*(1-xp) + c(4,:).*xp;
    z = y1.*(1-yp) + y2.*yp;
end