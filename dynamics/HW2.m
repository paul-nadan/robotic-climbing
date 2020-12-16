%% 2.1) Unit Test
fprintf('2.1)\n');
w1 = (rand(3,1)-0.5)*10
w2 = skew2angvel(angvel2skew(w1))
fprintf("Unit test passed: %s\n", string(isequal(w1, w2)));

%% 2.2) Unit Test
fprintf('--------------\n2.2)\n');
x1 = (rand(6,1)-0.5)*10
x2 = rbvel2twist(twist2rbvel(x1))
fprintf("Unit test passed: %s\n", string(isequal(x1, x2)));

%% 2.3)
fprintf('--------------\n2.3)\n');
tform2adjoint([eye(3), [1;2;3]; 0 0 0 1])

%% 2.4)
fprintf('--------------\n2.4)\n');
w = (rand(3,1)-0.5);
w = w/norm(w);
t = (rand()-0.5)*2*pi;
R1 = axang2rotm([w;t]')
R2 = expm(angvel2skew(w)*t)

%% 2.5)
fprintf('--------------\n2.5)\n');
w = (rand(3,1)-0.5);
w = w/norm(w);
t = (rand()-0.5)*2*pi;
v = (rand()-0.5)*10;

g1_w = axang2tform([w;t]')
g2_w = expm(twist2rbvel([zeros(3,1); w])*t)

g1_v = trvec2tform(w'*v)
g2_v = expm(twist2rbvel([w; zeros(3,1)])*v)

g1 = axang2tform([w;t]')*trvec2tform(w'*v)
g2 = expm(twist2rbvel([w*v/t; w])*t)

%% 2.6)
fprintf('--------------\n2.6)\n');
w = (rand(3,1)-0.5);
w = w/norm(w);
t = (rand()-0.5)*2*pi;
v = (rand()-0.5)*10;
g = expm(twist2rbvel([w*v/t; w])*t);
V = (rand(6,1)-0.5)*10;

Vs1 = tform2adjoint(g)*V
Vs2 = rbvel2twist(g*twist2rbvel(V)*inv(g))

Vb1 = inv(tform2adjoint(g))*V
Vb2 = rbvel2twist(inv(g)*twist2rbvel(V)*g)