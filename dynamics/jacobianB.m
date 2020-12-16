function Jb = jacobianB(g, q)
    syms Jb [6, length(q)] real
    for i = 1:length(q)
        Jb(:, i) = rbvel2twist(inv(g)*diff(g, q(i)));
    end
end