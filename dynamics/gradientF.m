% Compute the gradient of an expression with respect to a vector function
function grad = gradientF(f, q)
    syms grad [length(q) 1] real
    for i = 1:length(q)
        grad(i) = diff(f, q(i));
    end
end