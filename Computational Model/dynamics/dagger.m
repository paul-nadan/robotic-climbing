function A_dagger = dagger(A, M)
    M_inv = inv(M);
    A_dagger = inv(A*M_inv*A')*A*M_inv;
end