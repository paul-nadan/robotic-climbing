% Compute equations of motion from Lagrangian as a vector equal to zero
function eom = lagrangian2eom(L, q, q_dot, q_ddot, t, Upsilon)
    eom = simplify(diff(gradientF(L, q_dot), t) - gradientF(L, q)-Upsilon);
    eom = subs(eom, [diff(q); diff(q_dot)], [q_dot; q_ddot]);
end