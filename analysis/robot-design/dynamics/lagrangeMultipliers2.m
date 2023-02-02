function [lambda, q_ddot] = lagrangeMultipliers2(A, M, C, N, Y, A_dot, q_dot, t)
    block = [M A'; A zeros(size(A, 1), size(A, 1))];
    rhs = [Y-C*q_dot-N; -A_dot*q_dot];
    solution = inv(block)*rhs;
    solution = solution(t);
    q_ddot = solution(1:length(q_dot));
    lambda = solution(length(q_dot)+1:end);
end