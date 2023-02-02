function [center, radius] = getSphere(P)
    x = P(1,:)';
    y = P(2,:)';
    z = P(3,:)';
    if ~any(z-z(1))
        radius = NaN;
        center = [0;0;z(1)];
        return
    end
    x1 = circshift(x,1);
    y1 = circshift(y,1);
    z1 = circshift(z,1);
    A = 2*[x-x1, y-y1, z-z1];
    b = x.^2 + y.^2 + z.^2 - x1.^2 - y1.^2 - z1.^2;
    center = A\b;
    radius = norm(P(:,1)-center);
end