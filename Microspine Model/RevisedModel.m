set(0,'defaultfigureposition',[400 100 900 750])

clear
close all
syms K [2 2] real
syms rx Dx a real

Dx = .01:.01:1;
Fsol = zeros(size(Dx));
asol = zeros(size(Dx));
for i = 1:length(Dx)
    dx = Dx(i);
    rx = 0;
    % Dx = 1;
    theta = deg2rad(45);
    K0 = diag([2,0]);
    K1 = diag([20,0]);
    R0 = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    K = R0'*K0*R0 + K1;
    r = [rx; 0];
    D = [dx; 0];
    R = [cos(a), -sin(a); sin(a), cos(a)];
    Fw = R'*(K*(R*(r+D) - r));
    asol(i) = rad2deg(double(vpasolve(Fw(2) == 0, a, deg2rad(-10))));
    F = double(subs(Fw, a, deg2rad(asol(i))));
    Fsol(i) = F(1);
    dx = double(subs(R*(r+D) - r, a, deg2rad(asol(i))));
    if abs(F(2)) > 1e-10
        disp('Error!');
    end
end
clf;
hold on;
subplot(2,2,1);
plot(Dx, Fsol./Dx);
xlabel('Displacement');
ylabel('Stiffness');

subplot(2,2,2);
plot(Dx, Fsol);
xlabel('Displacement');
ylabel('Force');

subplot(2,2,3);
plot(Dx, asol);
xlabel('Displacement');
ylabel('Rotation (deg)');

subplot(2,2,4);
plot(Dx(1:end-1), (Fsol(2:end)-Fsol(1:end-1))./(Dx(2:end)-Dx(1:end-1)));
xlabel('Displacement');
ylabel('Marginal Stiffness');