%% Question 1.2
syms q1 q2 q3 q4 lw lh l1 l2 ls x y theta real

goc1 = [ sin(-theta),  cos(-theta), -ls-x+ls-lw/2*cos(-theta);
        -cos(-theta),  sin(-theta), -y-lw/2*sin(-theta);
         0,           0,          1];
goc2 = [-sin(-theta), -cos(-theta), ls-x-ls+lw/2*cos(-theta);
         cos(-theta), -sin(-theta), -y+lw/2*sin(-theta);
         0,           0,          1];
Ad1 = tform2adjoint2d(goc1^-1);
Ad2 = tform2adjoint2d(goc2^-1);
Bc = [1 0; 0 1; 0 0];
Gs = simplify(-[Ad1'*Bc, Ad2'*Bc])

%% Question 1.3
subs(-Gs, {x,y,theta}, [0,0,0])

%% Question 1.4
gs1c1 = [ sin(-theta),  cos(-theta), l1*cos(q1)+l2*cos(q1+q2);
        -cos(-theta),  sin(-theta), l1*sin(q1)+l2*sin(q1+q2);
         0,           0,          1];
gs2c2 = [-sin(-theta), -cos(-theta), l1*cos(q3)+l2*cos(q3+q4);
         cos(-theta), -sin(-theta), l1*sin(q3)+l2*sin(q3+q4);
         0,           0,          1];
         
Js1f1 = [0 l1*sin(q1); 0 -l1*cos(q1); 1 1];
Js2f2 = [0 l1*sin(q3); 0 -l1*cos(q3); 1 1];

Jh = simplify([Bc'*tform2adjoint2d(gs1c1^-1)*Js1f1, zeros(2);
               zeros(2), Bc'*tform2adjoint2d(gs2c2^-1)*Js2f2])

%% Question 2.3
syms phi psi l real
A = [0, -sin(phi), cos(phi), 0; 0, -sin(phi+psi), cos(phi+psi), l*cos(psi)];
H = [0 1; cos(phi) 0; sin(phi) 0; tan(psi)/l 0];
simplify(A*H)