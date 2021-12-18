function Js = jacobianS(g, q)
    syms Jb [6, length(q)] real
    for i = 1:length(q)
        Js(:, i) = rbvel2twist(diff(g, q(i))*inv(g));
    end
end