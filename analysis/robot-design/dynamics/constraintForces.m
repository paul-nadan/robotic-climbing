function lambda = constraintForces(A, M, C, N, Y, q, q_dot, q_ddot, t)
    lambda = inv(A*inv(M)*A')*(A*inv(M)*(Y-C*q_dot-N)+diff(A, t)*q_dot);
    lambda = subs(lambda, [diff(q); diff(q_dot)], [q_dot; q_ddot]);
    lambda = simplify(lambda);
end