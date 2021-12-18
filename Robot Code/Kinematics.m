syms LINK1 LINK2A LINK2B LINK3 a1 a2 a3 vx vy vz fx fy fz t1 t2 t3 w1 w2 w3
L1r = LINK1;
L2r = LINK2A*cos(a2) - LINK2B*sin(a2);
L3r = -LINK3*sin(a2 + a3);

L1z = 0;
L2z = -LINK2A*sin(a2) - LINK2B*cos(a2);
L3z = -LINK3*cos(a2 + a3);

r = L1r + L2r + L3r;
z = L1z + L2z + L3z;
x = r*cos(a1);
y = r*sin(a1);

J = jacobian([x,y,z], [a1,a2,a3])
Jinv = simplify(inv(J))

W = simplify(Jinv*[vx;vy;vz])
T = simplify(J.'*[fx;fy;fz])
F = simplify(Jinv.'*[t1;t2;t3])
V = simplify(J*[w1;w2;w3])

X = [x;y;z]

syms r1 z1
solve([r;z] == [r1;z1], [a2, a3])