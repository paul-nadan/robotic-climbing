% Convert w to w_hat
function w = angvel2skew(w)
    w = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
end