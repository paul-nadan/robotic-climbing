function cost = area2cost(area)
    a = 0.07791;
    b = 8.641;
    c = 0.6413;
    cost = a.*exp(-b.*sqrt(area))+c;
end
