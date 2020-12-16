% Convert [R, p; 0, 0, 0, 1] to [R, p_hat*R; 0_3x3, R]
function Adg = tform2adjoint(g)
    R = g(1:3,1:3);
    Adg = [R, angvel2skew(g(1:3,4))*R; zeros(3), R];
end