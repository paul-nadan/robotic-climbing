% Body joints x3 (pitch/roll/yaw), knees joints x3 (front, middle, back),
% tail, second body joint, middle legs
r = dec2bin(0:2^9-1) == '1';
fail = (r(:,8) > sum(r(:,1:3), 2)) | (r(:,5) > r(:,9));
dof = sum(r(:,1:3), 2).*(1+r(:,8)) + sum(r(:,4:6), 2)*2 + r(:,7) + r(:,9)*4 + 8;
dof(r(:,9)) = NaN;
dof(fail) = NaN;
b3 = r(:,8);
b1 = sum(r(:,1:3), 2) == 0;
b2 = ~b3 & ~b1;
l6 = r(:,9);
N = 8:25;
close all;
% histogram(dof,N);
plot(N, sum(dof<=N, 1));
hold on;
plot(N, sum(dof(b1)<=N, 1));
plot(N, sum(dof(b2)<=N, 1));
plot(N, sum(dof(b3)<=N, 1));
plot(N, sum(dof(~l6)<=N, 1));
plot(N, sum(dof(l6)<=N, 1));
% plot(N, sum(dof(logical(b1.*~l6))<=N, 1));
xlim([8,25]);
xlabel('Max DoF');
ylabel('Number of Robots');
legend('All', '1 Body Segment', '2 Body Segments', '3 Body Segments', '4 Legs', '6 Legs');
codes = num2str(r(dof<=14,:)*1);
codes = codes(:,codes(1,:)~= ' ');

origin = [0;0;0;0;1;0;-1;0;0];
for i = 110%[]%randsample(1:size(codes,1),20)
    code = codes(i,:);
    config = simpleWalker(code, .2, .6, .25);
    angles = config.gait.angles(:,1);
    angles(~angles) = 20;
    robot = state2robot([origin;angles],config);
    figure();
    plotRobot(robot);
    axis equal
    v = [-3 -5 5];
    [caz,cel] = view(v);
    title([code,': ',num2str(sum(config.count)), ' DoF']);
end