function L = getMinLegLengths(configs, radius, incline, pitch)
    if nargin < 2
        radius = 0;
    end
    if nargin < 3
        incline = 90-1e-10;
    end
    if nargin < 4
        pitch = 90;
    end
    L = zeros(size(configs));
    for i = 1:length(configs)
        code = dec2bin(configs(i)) == '1';
        code = [zeros(1,9-length(code)),code];
        p = code(1)*pitch;
        
        x0 = radius.*cosd(90-incline);
        y0 = radius.*sind(90-incline);
        xf = cosd(p/2)/2;
        yf = (xf<x0).*sqrt(radius.^2-xf.^2) + (xf>=x0).*(y0-(xf-x0).*tand(incline));

        xb = radius.*cosd(90-p/2);
        yb = sqrt(radius.^2-xb.^2);
        ymax = yb + xb.*sind(p/2);
        ys = ymax - xf.*sind(p/2);
        l = ys-yf;

        L1 = l.*cosd(incline);
        L2 = sqrt(xf.^2+ys.^2)-radius;
        
        select = 90-incline > atan2d(ys,xf);
        L(i) = select.*L1 + ~select.*L2;
        
%         if code(8) % Temporary, value for r = 0, pitch = 90
%             L(i) = cosd(p)/3 + 1/6;
%         end
    end

end