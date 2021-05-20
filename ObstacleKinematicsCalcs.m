% r = radius of fillet over the corner
% theta = angle of flat section downward from horizontal
% w = length of robot body
% l = leg length
% a = maximum body angle
% origin is at center of circle
% if a > 2*theta then any leg length works
% Given (theta, r/w, alpha), solve for l

w = 1; % Normalize lengths based on body length
% r = 0:.01:1;
% theta = [0:1:89,89.99999]';
r = .5;
theta = 60;
% a = [0,15,30,45,60,75,90];
a = 0;

x0 = r.*cosd(90-theta);
y0 = r.*sind(90-theta);
xf = w.*cosd(a/2)/2;
yf = (xf<x0).*sqrt(r.^2-xf.^2) + (xf>=x0).*(y0-(xf-x0).*tand(theta));

xb = r.*cosd(90-a/2);
yb = sqrt(r.^2-xb.^2);
ymax = yb + xb.*sind(a/2);
ys = ymax - xf.*sind(a/2);
l = ys-yf;

L1 = l.*cosd(theta);
lx1 = L1.*sind(theta);
ly1 = L1.*cosd(theta);

L2 = sqrt(xf.^2+ys.^2)-r;
lx2 = L2.*xf./sqrt(xf.^2+ys.^2);
ly2 = L2.*ys./sqrt(xf.^2+ys.^2);

select = 90-theta > atan2d(ys,xf);
L = select.*L1 + ~select.*L2;
lx = select.*lx1 + ~select.*lx2;
ly = select.*ly1 + ~select.*ly2;

disp(l/w);
disp(L/w);

% plot(r/w, L/w);
imagesc(r/w, theta, L/w);
colorbar;
set(gca,'YDir','normal');
xlabel('Radius of Curvature');
ylabel('Angle of slope');
title('Minimum Leg Length');
caxis([0,0.5]);
figure;
% plot(theta, max(L,[],2)/w);
plot(theta, L/w);
legend('0°','15°','30°','45°','60°','75°','90°');
ylim([0,0.5]);
xlabel('Angle of slope');
ylabel('Minimum Leg Length');


if length(r) > 1 || length(theta) > 1
    return
end
close all;
hold on;
% plot([xf, xf], [yf,yf+l], 'b', 'linewidth', 2);
% plot([-xf, -xf], [yf,yf+l], 'b', 'linewidth', 2);
plot([0,xf], [ymax,ys], 'r', 'linewidth', 2);
plot([0,-xf], [ymax,ys], 'r', 'linewidth', 2);
t = -theta:0.1:theta;
plot(r*sind(t), r*cosd(t), 'k', 'linewidth', 2);
if xf > x0
    plot([x0, xf], [y0, yf], 'k', 'linewidth', 2);
    plot([-x0, -xf], [y0, yf], 'k', 'linewidth', 2);
    plot([x0, -x0], [y0, y0], 'k.', 'markersize', 20);
end
plot([xf, xf-lx], [ys, ys-ly],'b', 'linewidth', 2);
plot(-[xf, xf-lx], [ys, ys-ly],'b', 'linewidth', 2);
axis equal;
