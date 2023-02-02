function R = vrrotvec2mat_sym(k)
    k(1:3) = k(1:3)/norm(k(1:3));
    K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
    R = simplify(eye(3) + sin(k(4))*K + (1-cos(k(4)))*K^2);
end