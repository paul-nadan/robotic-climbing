incline = 70;
w = .2;
h = .5;

segs = 2;
pitch = 90*(segs > 1);
roll = 90*(segs > 1);

Lx = w*sind(incline)/2;

if segs == 1
    Ly = h*sind(incline)/2;
elseif segs == 2
    Ly = h*sind(incline-pitch/2)/2;
else
    Ly = h*sind(incline-pitch)/3 + h*sind(incline)/6;
end

if w > h
    Lxy = h*sind(incline);
elseif segs == 1
    Lxy = w*sind(incline);
elseif segs == 2
    Lxy = w*sind(incline-roll/2);
elseif segs == 3
    Lxy = w*sind(incline-roll);
end

code = '111000000';
code = code == '1';
code = [zeros(1,9-length(code)),code];
rx = simpleWalker(code, w, h, Lx);
ry = simpleWalker(code, w, h, Ly);
rxy = simpleWalker(code, w, h, Lxy);

originx = [0,0,h/2];
originy = [0,-sind(pitch/2)*h/2,cosd(pitch/2)*h/2];
originxy = [0,w/2*cosd(roll/2)*tand(incline)-w/2*sind(roll/2),h/2];
T = [0,0,1];
N = [1,0,0];
Tpitch = [0,-sind(pitch/2),1*cosd(pitch/2)];
Nroll = [cosd(roll/2),-sind(roll/2),0];
thetax = [0, incline+90, 0, incline+90, 0, 0, 0, 0, incline+90, 0, incline+90];
thetay = [90, incline+90-pitch/2, 90, incline+90-pitch/2, pitch, 0, 0, -90, incline+90-pitch/2, -90, incline+90-pitch/2];
thetaxy = [0, incline+90-roll/2, 0, -incline+roll/2, 0, roll, 0, 0, -incline+roll/2, 0, incline+90-roll/2];
Rx = state2robot([originx, T, N, thetax]', rx);
Ry = state2robot([originy, Tpitch, N, thetay]', ry);
Rxy = state2robot([originxy, T, Nroll, thetaxy]', rxy);

[X, Y] = meshgrid(-.15:.01:.15, -.3:.01:.3);
Zx = -abs(X)*tand(incline);
Zy = -abs(Y)*tand(incline);
Zxy = sign(X.*Y).*min(abs(X), abs(Y))*tand(incline);

close all;
surf(X,Y,Zx);
plotRobot(Rx);
axis equal;

figure();
surf(X,Y,Zy);
plotRobot(Ry);
axis equal;

figure();
surf(X,Y,Zxy);
plotRobot(Rxy);
axis equal;

[Lx, Ly, Lxy]

%%%
return
sweep = -.1:0.01:0;
for x = sweep
% x = -.08;
% for offset = -60:5:0
    y = -abs(x)*tand(incline);
    x1 = x-Ly*sind(incline+offset);
    y1 = y+Ly*cosd(incline+offset);
    ang1 = atan2d(-y1, -x1);
    x2 = x1+h/2*cosd(ang1);
    y2 = y1+h/2*sind(ang1);
    x3 = x2+h/2*(cosd(ang1-pitch));
    y3 = y2+h/2*(sind(ang1-pitch));
    x4 = x3-Ly*sind(incline);
    y4 = y3-Ly*cosd(incline);
    figure;
    plot([x,x1,x2,x3,x4], [y,y1,y2,y3,y4], 'r');
    hold on
    plot(-.15:.01:.25, -abs(-.15:.01:.25)*tand(incline), 'k');
    axis equal;
end