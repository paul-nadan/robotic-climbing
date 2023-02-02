% Convert [w_hat, v; 0, 0, 0, 0] to [v; w]
function x = rbvel2twist(x)
    x = [x(1:3,4); skew2angvel(x(1:3,1:3))];
end