% Convert w_hat to w
function w = skew2angvel(w)
    w = [w(3,2); w(1,3); w(2,1)];
end