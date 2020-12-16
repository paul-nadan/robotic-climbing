function lambda = lagrangeMultipliers(A, M, C, N, Y, A_dot, q_dot)
    lambda = inv(A*inv(M)*A')*(A*inv(M)*(Y-C*q_dot-N)+A_dot*q_dot);
    lambda = simplify(lambda);
end