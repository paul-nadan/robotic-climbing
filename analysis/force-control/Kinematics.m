% Grasp map
clear all;
% syms x1 y1 z1 x2 y2 z2 x3 y3 z3 x4 y4 z4 real
% syms fx1 fy1 fz1 fx2 fy2 fz2 fx3 fy3 fz3 fx4 fy4 fz4 real
% W = sqrt(diag(sym('w', [12,1],'positive')));
% F = sym('F', [6,1],'real');
% f = [fx1 fy1 fz1 fx2 fy2 fz2 fx3 fy3 fz3 fx4 fy4 fz4]';
% X1 = [x1; y1; z1];
% X2 = [x2; y2; z2];
% X3 = [x3; y3; z3];
% X4 = [x4; y4; z4];
% G = [eye(3), eye(3), eye(3), eye(3);
%      getSkew(X1), getSkew(X2), getSkew(X3), getSkew(X4)]
% Ginv = W*pinv(G*W)
% 
% return

syms LINK1 LINK2A LINK2B LINK3 a1 a2 a3 vx vy vz fx fy fz t1 t2 t3 w1 w2 w3

LINK1 = 23.5;
LINK2A = 111;
LINK2B = 22.5;
LINK3 = 98.7;

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
% solve([r;z] == [r1;z1], [a2, a3])


function V = getSkew(v)
    V = [0,    -v(3),  v(2);
         v(3),  0,    -v(1);
        -v(2),  v(1),  0];
end
% 
% clear
% syms LINK1 LINK2A LINK2B LINK3 TAIL WIDTH LENGTH HEIGHT...
%      a1 a2 a3 ab at x1 x2 x3...
%      vx vy vz w1 w2 w3 fx fy fz t1 t2 t3 real
% 
% % Morphology
% robot.rotate = [1, 1, -1, -1, -1]/2;                    % Apply body joint
% robot.corners = [-1 1 -1 1 1; 1 1 -1 -1 1; 1 1 1 1 1]; % Reverse X0
% robot.mirror = [-1 1 -1 1 1; 1 1 1 1 1; 1 1 1 1 1];    % Reverse X123
% 
% % robot.rotate = [1, 1, 0, 0, -1, -1, -1]/2;                 % Hexapod!
% % robot.corners = [-1 1 -1 1 -1 1 1; 1 1 0 0 -1 -1 1; 1 1 1 1 1 1 1];
% % robot.mirror = [-1 1 -1 1 -1 1 1; 1 1 1 1 1 1 1; 1 1 1 1 1 1 1];
% 
% robot.tail = 1;                                         % Tail present
% robot.n = length(robot.rotate);                         % Number of limbs
% 
% % Geometry (comment out for symbolic math)
% LINK1 = 23.5e-3;        % Shoulder link length
% LINK2A = 111e-3;        % Upper leg link length
% LINK2B = 22.5e-3;       % Upper leg link vertical offset
% LINK3 = 90.3e-3;        % Lower leg link length
% WIDTH = 154e-3;         % Body width
% LENGTH = 283.4e-3;      % Body length
% HEIGHT = 54e-3;         % Body height
% TAIL = 350e-3;          % Tail length
% robot.h = HEIGHT;
% 
% % Body joint kinematics
% R = [1, 0, 0; 0, cos(ab), -sin(ab); 0, sin(ab), cos(ab)];   % Pitch joint
% 
% % Limb kinematics
% L1r = LINK1;
% L2r = LINK2A*cos(a2) - LINK2B*sin(a2);
% L3r = -LINK3*sin(a2 + a3);
% 
% L1z = 0;
% L2z = -LINK2A*sin(a2) - LINK2B*cos(a2);
% L3z = -LINK3*cos(a2 + a3);
% 
% % Joint positions
% X0 = R*[WIDTH/2; LENGTH/2; 0];
% X1 = R*[L1r*cos(a1); L1r*sin(a1); L1z];
% X2 = R*[L2r*cos(a1); L2r*sin(a1); L2z];
% X3 = R*[L3r*cos(a1); L3r*sin(a1); L3z];
% X = X0 + X1 + X2 + X3;
% 
% X0r = R'*[WIDTH/2; -LENGTH/2; 0];
% X1r = R'*[L1r*cos(a1); L1r*sin(a1); L1z];
% X2r = R'*[L2r*cos(a1); L2r*sin(a1); L2z];
% X3r = R'*[L3r*cos(a1); L3r*sin(a1); L3z];
% Xr = X0r + X1r + X2r + X3r;
% 
% Xt0 = R'*[0; -LENGTH/2; 0];
% Xt1 = R'*[0; -cos(at)*TAIL; sin(at)*TAIL];
% Xt = Xt0 + Xt1;
% 
% % Jacobians
% J = jacobian(X, [a1, a2, a3, ab]);
% Jr = jacobian(Xr, [a1, a2, a3, ab]);
% Jt = jacobian(Xt, [ab, at]);
% 
% % Grasp Map
% G = [eye(3); skew([x1, x2, x3])];
% 
% % Individual limb functions
% robot.getR = matlabFunction(R, 'vars', ab);
% robot.getJ = matlabFunction(J, 'vars', {[a1; a2; a3], ab});
% robot.getJr = matlabFunction(Jr, 'vars', {[a1; a2; a3], ab});
% robot.getX0 = matlabFunction(X0, 'vars', {[a1; a2; a3], ab});
% robot.getX1 = matlabFunction(X1, 'vars', {[a1; a2; a3], ab});
% robot.getX2 = matlabFunction(X2, 'vars', {[a1; a2; a3], ab});
% robot.getX3 = matlabFunction(X3, 'vars', {[a1; a2; a3], ab});
% 
% robot.getJt = matlabFunction(Jt, 'vars', {at, ab});
% robot.getXt0 = matlabFunction(Xt0, 'vars', {at, ab});
% robot.getXt1 = matlabFunction(Xt1, 'vars', {at, ab});
% 
% robot.getG = matlabFunction(G, 'vars', {[x1; x2; x3]});
% 
% % Full robot functions
% robot.getJoints = @(a)getJoints(a, robot);
% robot.getJacobian = @(a)getJacobian(a, robot);
% robot.getGraspMap = @(a)getGraspMap(a, robot);
% 
% QuasiStaticSim
% 
% function [X, corner, shoulder, knee, foot, base, tail] = getJoints(a, robot)
%     a = deg2rad(a);
%     nlegs = robot.n - robot.tail;
%     corner = zeros(3, nlegs);
%     shoulder = zeros(3, nlegs);
%     knee = zeros(3, nlegs);
%     foot = zeros(3, nlegs);
%     ab = a(2, end);
%     for i = 1:nlegs
%         ab_corner = ab*robot.rotate(i)*robot.corners(2, i);
%         ab_mirror = ab*robot.rotate(i)*robot.mirror(2, i);
%         corner(:, i) = robot.corners(:, i).*robot.getX0(a(:, i), ab_corner);
%         shoulder(:, i) = corner(:, i) + robot.mirror(:, i).*robot.getX1(a(:, i), ab_mirror);
%         knee(:, i) = shoulder(:, i) + robot.mirror(:, i).*robot.getX2(a(:, i), ab_mirror);
%         foot(:, i) = knee(:, i) + robot.mirror(:, i).*robot.getX3(a(:, i), ab_mirror);
%     end
%     
%     if robot.tail
%         ab_corner = -ab*robot.rotate(i)*robot.corners(2, end);
%         ab_mirror = -ab*robot.rotate(i)*robot.mirror(2, end);
%         at = a(3, end);
%         base = robot.corners(:, end).*robot.getXt0(at, ab_corner);
%         tail = base + robot.mirror(:, end).*robot.getXt1(at, ab_mirror);
%     else
%        base = zeros(3, 0);
%        tail = zeros(3, 0);
%     end
%     X = [foot, tail];
% end
% 
% function J = getJacobian(a, robot)
%     a = deg2rad(a);
%     J = zeros(robot.n*3, robot.n*3 - 1);
%     nlegs = robot.n - robot.tail;
%     ab = a(2, end);
%     for i = 1:nlegs
% %         R = robot.getR(ab * robot.rotate(i));
% %         Ji = robot.getJ(a(:, i), 0);
% %         J(i*3-2:i*3, i*3-2:i*3) = robot.mirror(:, i).*R*Ji(:, 1:3);
% %         J(i*3-2:i*3, end-1) = R*robot.corners(:, i).*Ji(:, end)*robot.rotate(i);
%         
% %         if robot.rotate(i) > 0
% %             J(i*3-2:i*3, [i*3-2:i*3, end-1]) = robot.mirror(:, i).*robot.getJ(a(:, i), ab/2);
% %             J(i*3-2:i*3, end-1) = J(i*3-2:i*3, end-1)/2;
% %         elseif robot.rotate(i) < 0
% %             J(i*3-2:i*3, [i*3-2:i*3, end-1]) = robot.mirror(:, i).*robot.getJr(a(:, i), ab/2);
% %             J(i*3-2:i*3, end-1) = J(i*3-2:i*3, end-1)/2;
% %         else
%             J(i*3-2:i*3, [i*3-2:i*3, end-1]) = robot.mirror(:, i).*robot.getJ(a(:, i), 0);
%             J(i*3-2:i*3, end-1) = J(i*3-2:i*3, end-1)*0;
% %         end
%     end
%     if robot.tail
%         at = a(3, end);
%         
% %         R = robot.getR(ab * robot.rotate(end));
% %         J(end-2:end, end-1:end) = robot.mirror(:, end).*R*robot.getJt(at, 0);
% %         J(end-2:end, end-1) = J(end-2:end, end-1)*robot.rotate(end);
% %         J(end-2:end, end-1:end) = robot.getJt(at, ab/2);
%         
%         J(end-2:end, end-1:end) = [1;1;-1].*robot.getJt(at, 0);
% 
%         
%         J(end-2:end, end-1) = J(end-2:end, end-1)/2*0;
%     end
%     J = deg2rad(J);
% end
% 
% function G = getGraspMap(x, robot)
%     G = zeros(6, size(x, 2)*3);
%     for i = 1:size(x, 2)
%         G(:, i*3-2:i*3) = robot.getG(x(:, i));
%     end
% end
% 
% function V = skew(v)
%     V = [0,    -v(3),  v(2);
%          v(3),  0,    -v(1);
%         -v(2),  v(1),  0];
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Jinv = simplify(inv(J));
% 
% W = simplify(Jinv*[vx;vy;vz]);
% T = simplify(J.'*[fx;fy;fz]);
% F = simplify(Jinv.'*[t1;t2;t3]);
% V = simplify(J*[w1;w2;w3]);
% 
% R = [1, 0, 0; 0, cos(ab), -sin(ab); 0, sin(ab), cos(ab)];
% 
% X1 = R*[x1;y1;z1];
% X2 = R*[x2;y2;z2];
% X3 = R*[x3;y3;z3];
% 
% Xt = R'*[0;-LENGTH/2-cos(at)*TAIL;sin(at)*TAIL];
% Jt = jacobian(Xt, at);
% 
% % Kinematics functions are in meters and degrees
% a = [a1, a2, a3, ab, w1, w2, w3];
% getJ = matlabFunction(subs(J, a, deg2rad(a))/1000 * pi/180, 'vars', {[a1; a2; a3], ab});
% getJb = matlabFunction(subs(Jb, a, deg2rad(a))/1000 * pi/180, 'vars', {[a1; a2; a3], ab});
% getJt = matlabFunction(subs(Jt, [at, ab], deg2rad([at, ab]))/1000 * pi/180, 'vars', {at; ab});
% getX1 = matlabFunction(subs(X1, a, deg2rad(a))/1000, 'vars', {[a1; a2; a3], ab});
% getX2 = matlabFunction(subs(X2, a, deg2rad(a))/1000, 'vars', {[a1; a2; a3], ab});
% getX3 = matlabFunction(subs(X3, a, deg2rad(a))/1000, 'vars', {[a1; a2; a3], ab});
% getXt = matlabFunction(subs(Xt, [at, ab], deg2rad([at, ab]))/1000, 'vars', {at, ab});
% getV = matlabFunction(subs(V, a, deg2rad(a))/1000, 'vars', {[a1; a2; a3], [w1; w2; w3], ab});
% getT = matlabFunction(subs(T, a, deg2rad(a))/1000, 'vars', {[a1; a2; a3], [fx; fy; fz], ab});
% 
% getJinv = matlabFunction(subs(Jinv, a, deg2rad(a))*1000 * 180/pi, 'vars', {[a1; a2; a3]});
% getW = matlabFunction(subs(W, a, deg2rad(a))*1000, 'vars', {[a1; a2; a3], [vx; vy; vz], ab});
% getF = matlabFunction(subs(F, a, deg2rad(a))*1000, 'vars', {[a1; a2; a3], [t1; t2; t3], ab});
% 
% body = [WIDTH; LENGTH; HEIGHT]/1000;
% getJoints = @(a)getJoints_(a, body, getX1, getX2, getX3, getXt);
% getTorques = @(a, F)allFeet(a, F, getT);
% getForces = @(a, v)allFeet(a, T, getF);
% getVels = @(a, w)allFeet(a, w, getV);
% getAngvels = @(a, v)allFeet(a, v, getW);
% getJacobian = @(a)getJacobian_(a, getJ);
% getJacobianTail = getJt;
% getGrasp = @getG;
% getGraspTail = @getGt;
% 
% % syms r1 z1
% % solve([r;z] == [r1;z1], [a2, a3])
% 
% 
% 
% function [feet, knees, shoulders, corners] = getJoints_(a, getX0, getX1, getX2, getX3, getXt)
%     X0 = zeros(3, 4);
%     X1 = zeros(3, 4);
%     X2 = zeros(3, 4);
%     X3 = zeros(3, 4);
%     ab = a(2, end);
%     for i = 1:4
%         X0(:,i) = getX0(ab);
%         X1(:,i) = getX1(a(:, i), ab);
%         X2(:,i) = getX2(a(:, i), ab);
%         X3(:,i) = getX3(a(:, i), ab);
%     end
%     corners = [-1 1 -1 1; 1 1 -1 -1; 0 0 0 0] .* X0;
%     shoulders = [-1 1 -1 1; 1 1 1 1; 1 1 1 1] .* X1 + corners;
%     knees = [-1 1 -1 1; 1 1 1 1; 1 1 1 1] .* X2 + corners;
%     feet = [-1 1 -1 1; 1 1 1 1; 1 1 1 1] .* X3 + corners;
%     
%     if size(a, 2) >= 2
%         knees = [knees, [0; -1; 0] .* body/2];
%         feet = [feet, getXt(a(3, end), ab) + knees(:,end)];
%     end
% end
% 
% function output = allFeet(a, input, get)
%     output = zeros(3, 4);
%     for i = 1:4
%         output(:, i) = get(a(:, i), input(:, i), a(2, end));
%     end
% end
% 
% function J = getJacobian_(a, getJ)
%     J = zeros(12, 13);
%     for i = 1:4
%         J(i*3-2:i*3, i*3-2:i*3) = getJ(a(:, i), a(2, end));
%     end
%     J([1, 7], :) = -J([1, 7], :);
% end
% 
% function G = getG(feet)
%     G = zeros(6, 12);
%     for i = 1:4
%         G(1:3, i*3-2:i*3) = eye(3);
%         G(4:6, i*3-2:i*3) = skew(feet(:, i));
%     end
% end
% 
% function Gt = getGt(tail)
%     Gt = [eye(3); skew(tail)];
% end