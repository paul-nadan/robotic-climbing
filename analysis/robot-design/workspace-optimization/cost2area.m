function area = cost2area(cost)
    a = 0.07791;
    b = 8.641;
    c = 0.6413;
    area = (-log((cost-c)/a)/b).^2;
    cost = a.*exp(-b.*sqrt(area))+c;
end
