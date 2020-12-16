syms q1 q2 q3 q4 lw lh l1 l2 ls theta real

B = [1 0; 0 1; 0 0];
goc1 = [0 1 -lw/2; -1 0 0; 0 0 1];
goc2 = [0 -1 lw/2; 1 0 0; 0 0 1];
Ado1c1inv = [0 -1 lw/2; 1 0 0; 0 0 1];
Ado2c2inv = [0 1 lw/2; -1 0 0; 0 0 1];
G = [Ado1c1inv'*B, Ado2c2inv'*B]

Js1f1 = [0 sin(q1)*l1; 0 -cos(q1)*l1; 1 1];
Js2f2 = [0 sin(q3)*l1; 0 -cos(q3)*l1; 1 1];
gs1c1 = [sin(theta), cos(theta), l1*cos(q1)+l2*cos(q1+q2);
       -cos(theta), sin(theta), l1*sin(q1)+l2*sin(q1+q2);
        0 0 1];
gs1c2 = [-sin(theta), -cos(theta), l1*cos(q3)+l2*cos(q3+q4);
         cos(theta), -sin(theta), l1*sin(q3)+l2*sin(q3+q4);
         0 0 1];
Ads1c1inv = Adjoint2D(inv(gs1c1));
Ads2c2inv = Adjoint2D(inv(gs1c2));
Jh1 = simplify(B'*Ads1c1inv*Js1f1);
Jh2 = simplify(B'*Ads2c2inv*Js2f2);
Jh = [Jh1, zeros(2); zeros(2), Jh2]

function Ad = Adjoint2D(g)
   Ad = [g(1:2,1:2), [g(2,3); -g(1,3)]; 0 0 1];
end