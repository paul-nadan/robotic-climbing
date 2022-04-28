% Grasp map
clear all;
syms x1 y1 z1 x2 y2 z2 x3 y3 z3 x4 y4 z4 real
syms fx1 fy1 fz1 fx2 fy2 fz2 fx3 fy3 fz3 fx4 fy4 fz4 real
W = sqrt(diag(sym('w', [12,1],'positive')));
F = sym('F', [6,1],'real');
f = [fx1 fy1 fz1 fx2 fy2 fz2 fx3 fy3 fz3 fx4 fy4 fz4]';
X1 = [x1; y1; z1];
X2 = [x2; y2; z2];
X3 = [x3; y3; z3];
X4 = [x4; y4; z4];
G = [eye(3), eye(3), eye(3), eye(3);
     getSkew(X1), getSkew(X2), getSkew(X3), getSkew(X4)]
Ginv = W*pinv(G*W)

return
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


function V = getSkew(v)
    V = [0,    -v(3),  v(2);
         v(3),  0,    -v(1);
        -v(2),  v(1),  0];
end