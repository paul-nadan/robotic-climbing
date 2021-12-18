function L = getMinLegLengths2(configs, w, h, incline, pitch, roll)
    if nargin < 4
        incline = 70;
    end
    if nargin < 5
        P = 90;
    end
    if nargin < 6
        R = 90;
    end
    L = zeros(size(configs));
    for i = 1:length(configs)
        code = dec2bin(configs(i)) == '1';
        code = [zeros(1,9-length(code)),code];
        segs = 1+any(code(1:3))*(1+code(8));
        
        pitch = P*code(1);
        roll = R*code(2);

        Lx = w*sind(incline)/2;

        if segs == 1
            Ly = h*sind(incline)/2;
        elseif segs == 2
            Ly = h*sind(incline-pitch/2)/2;
        else
            Ly = h*sind(incline-pitch)/3 + h*sind(incline)/6;
        end

        if w > h
            Lxy = h*sind(incline);
        elseif segs == 1
            Lxy = w*sind(incline);
        elseif segs == 2
            Lxy = w*sind(incline-roll/2);
        elseif segs == 3
            Lxy = w*sind(incline-roll);
        end
%         [Lx,Ly,Lxy]
        L(i) = max([Lx,Ly,Lxy]);
    end
end