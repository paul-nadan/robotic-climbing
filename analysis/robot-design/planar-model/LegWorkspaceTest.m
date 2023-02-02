L = [1, 1.4];
lim = [-70, -60, -90;
        70,  90,  60];

a2 = lim(1,2):1:lim(2,2);
a3 = (lim(1,3):1:lim(2,3))';

r = -L(2)*sind(a2 + a3) + L(1)*cosd(a2);
z = -L(2)*cosd(a2 + a3) - L(1)*sind(a2);

clf;
figure(1);
plot(reshape(r,1,[]), reshape(z,1,[]), 'r.');
axis equal;
hold on;
xlabel('R');
ylabel('Z');

a2 = [lim(1,2); lim(1,2); lim(2,2); lim(2,2)];
a3 = [lim(1,3); lim(2,3); lim(1,3); lim(2,3)];
r = [0*a2, L(1)*cosd(a2), -L(2)*sind(a2 + a3) + L(1)*cosd(a2)];
z = [0*a2, - L(1)*sind(a2), -L(2)*cosd(a2 + a3) - L(1)*sind(a2)];
plot(r', z');

y = 0.5:.01:2;
yaw = (0:70)';
% y = 1/2./sind(yaw);
yaw = 30;
y = 1.6;

rmax = y./cosd(yaw);
upper = -L(1)*sind(lim(1,2)) - sqrt(L(2)^2 - (y-L(1)*cosd(lim(1,2))).^2);
lower = -sqrt((L(1)+L(2))^2 - rmax.^2);
step = 2*y.*sind(yaw);
A = step.*(upper - lower);
plot([y, y, rmax, rmax, y], [upper, lower, lower, upper, upper], 'k');

% figure(2);
% plot(y, step, 'k');
% hold on;
% plot(y, upper, 'r');
% plot(y, lower, 'b');
% plot(y, A, 'm');
% figure(3);
% % step(abs(imag(A))>0) = 0;
% h = min(0,upper) - lower;
% h(abs(imag(h))>0) = 0;
% A(abs(imag(A))>0) = 0;
% imagesc(yaw, y, h');
% figure(4);
% imagesc(yaw, y, step');
% 
% 
% figure(5);
% imagesc(yaw, y, (step.*h)');

