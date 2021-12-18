% Compute Coriolis matrix C from inertia matrix M
function C = coriolis(M, q, q_dot)
    syms C [length(q) length(q)] real
    k = 1:length(q);
    for i = 1:length(q)
        for j = 1:length(q)
            C(i,j) = 1/2*sum((gradientF(M(i,j), q(k))+diff(M(i,k)', q(j))...
                -diff(M(k,j), q(i))).*q_dot(k));
        end
    end
    C = simplify(C);
end