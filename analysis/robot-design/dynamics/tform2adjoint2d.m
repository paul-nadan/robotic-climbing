% Convert [R, p; 0, 0, 1] to [R, p_hat*R; 0_3x3, R]
function Adg = tform2adjoint2d(g)
    R = g(1:2,1:2);
    p = g(1:2,3);
    Adg = [R, [p(2); -p(1)]; 0 0 1];
end