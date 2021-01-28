function [c, c1, c2] = gripperMargin(Fnorm, Ftang, Fmax)
    if nargin < 3
        Fmax = 25;
    end
    Fnorm = max(Fnorm, 0);
    c1 = Fnorm./max(Ftang, 1e-6)/tand(20);
    c2 = sqrt(Fnorm.^2+Ftang.^2)./Fmax;
    c = max([c1, c2]);
end