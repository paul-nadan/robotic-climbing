%% 2.1)
fprintf('2.1)\n');
gst0_handCalc = [1 0 0 1407; 0 1 0 0; 0 0 1 1855; 0 0 0 1]

%% 2.2)
syms q1 q2 q3 q4 q5 q6 real
q = [q1; q2; q3; q4; q5; q6];

Rs1 = [cos(q1) -sin(q1) 0; sin(q1) cos(q1) 0; 0 0 1]; % +Z
R12 = [cos(q2) 0 sin(q2); 0 1 0; -sin(q2) 0 cos(q2)]; % +Y
R23 = [cos(q3) 0 sin(q3); 0 1 0; -sin(q3) 0 cos(q3)]; % +Y
R34 = [1 0 0; 0 cos(q4) -sin(q4); 0 sin(q4) cos(q4)]; % +X
R45 = [cos(q5) 0 sin(q5); 0 1 0; -sin(q5) 0 cos(q5)]; % +Y
R5t = [1 0 0; 0 cos(q6) -sin(q6); 0 sin(q6) cos(q6)]; % +X

ps1 = [0; 0; 0];
p12 = [320; 0; 680];
p23 = [0; 0; 975];
p34 = [0; 0; 200];
p45 = [887; 0; 0];
p5t = [200; 0; 0];

gs1 = [Rs1 ps1; 0 0 0 1];
g12 = [R12 p12; 0 0 0 1];
g23 = [R23 p23; 0 0 0 1];
g34 = [R34 p34; 0 0 0 1];
g45 = [R45 p45; 0 0 0 1];
g5t = [R5t p5t; 0 0 0 1];

fprintf('2.2)\n');
gst = gs1*g12*g23*g34*g45*g5t
Gst = matlabFunction(gst, 'vars', {q});
gst0 = Gst(zeros(6,1))
comparison = gst0 == gst0_handCalc

%% 2.3
w1 = [0; 0; 1];
w2 = [0; 1; 0];
w3 = [0; 1; 0];
w4 = [1; 0; 0];
w5 = [0; 1; 0];
w6 = [1; 0; 0];

eta1 = [-cross(w1, ps1); w1];
eta2 = [-cross(w2, ps1+p12); w2];
eta3 = [-cross(w3, ps1+p12+p23); w3];
eta4 = [-cross(w4, ps1+p12+p23+p34); w4];
eta5 = [-cross(w5, ps1+p12+p23+p34+p45); w5];
eta6 = [-cross(w6, ps1+p12+p23+p34+p45+p5t); w6];

exp1 = expm(twist2rbvel(eta1)*q1);
exp2 = expm(twist2rbvel(eta2)*q2);
exp3 = expm(twist2rbvel(eta3)*q3);
exp4 = expm(twist2rbvel(eta4)*q4);
exp5 = expm(twist2rbvel(eta5)*q5);
exp6 = expm(twist2rbvel(eta6)*q6);

fprintf('\n2.3)\n');
gst_exp = exp1*exp2*exp3*exp4*exp5*exp6*gst0_handCalc
comparison = simplify(gst_exp == gst)

%% 2.4)
fprintf('\n2.4)\n');
gst_des = gst0 + twist2rbvel([0; 100; 0; 0; 0; 0])
q_sol = fminunc(@(x) norm(Gst(x)-gst_des), zeros(6,1))
gst_sol = Gst(q_sol)
comparison = abs(gst_sol - gst_des) < 0.1

%% 2.5)
Js1 = rbvel2twist(diff(gst, 'q1')*inv(gst));
Js2 = rbvel2twist(diff(gst, 'q2')*inv(gst));
Js3 = rbvel2twist(diff(gst, 'q3')*inv(gst));
Js4 = rbvel2twist(diff(gst, 'q4')*inv(gst));
Js5 = rbvel2twist(diff(gst, 'q5')*inv(gst));
Js6 = rbvel2twist(diff(gst, 'q6')*inv(gst));

fprintf('\n2.5)\n');
Js = [Js1 Js2 Js3 Js4 Js5 Js6]

%% 2.6)
Js1_exp = eta1;
Js2_exp = tform2adjoint(exp1)*eta2;
Js3_exp = tform2adjoint(exp1*exp2)*eta3;
Js4_exp = tform2adjoint(exp1*exp2*exp3)*eta4;
Js5_exp = tform2adjoint(exp1*exp2*exp3*exp4)*eta5;
Js6_exp = tform2adjoint(exp1*exp2*exp3*exp4*exp5)*eta6;
fprintf('\n2.6)\n');
Js_exp = [Js1_exp Js2_exp Js3_exp Js4_exp Js5_exp Js6_exp]
comparison = eval(simplify(Js_exp - Js) == 0)

%% 2.7)
fprintf('\n2.7)\n');
Vb = [0; 1; 0; 0; 0; 0]
Vs = tform2adjoint(gst0)*Vb
q_des = inv(subs(Js,q,zeros(6,1)))*Vs