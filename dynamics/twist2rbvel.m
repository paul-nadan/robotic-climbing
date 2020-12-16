% Convert [v; w] to [w_hat, v; 0, 0, 0, 0]
function x = twist2rbvel(x)
    x = [angvel2skew(x(4:6)), x(1:3); zeros(1,4)];
end