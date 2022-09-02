%% New
n = 2;
k = 1;

% Theta = linspace(0, 90, 1800+1);
% Theta = Theta(2:end-1);
% Phi = linspace(0, 90, 1800*k+1)';
% Phi = Phi(1:end-1);
% dphi = Phi(1+k) - Phi(1);
Ft = 0:.0001:1;
Fb = -.5:.0001:.5;

W = zeros(length(Ft), length(Fb));

for i = 1:length(Ft)
    for j = 1:length(Fb)
        phi = atan2d(Fb(j), Ft(i));
        theta = 45;
        Tmax = cosd(theta);
        Bmax = sind(theta);
        W(i, j) = Ft(i)/Tmax + abs(Fb(j))/Bmax;
        if abs(phi) > theta || W(i, j) > 1
            W(i, j) = NaN;
        end
%         W(i, j) = 1/(cosd(phi)/Tmax + sind(phi)/Bmax);
    end
end

% theta = Theta;
% phi = Phi(1:k:end);
% W(theta < phi) = NaN;
% W(W<=0) = NaN;
% F = 1./W;
% F(F > 1.5) = 0;
imagesc(Fb, Ft, W);
% axis equal
ylabel('Tangential Load');
xlabel('Lateral Load');
title('Effective Load')
colorbar
set(gca,'YDir','normal')
return

figure(5);
imagesc(theta, phi, W)
xlabel('Splay Angle')
ylabel('Force Angle')
title(['Gripper ' num2str(n) ': Maximum Load'])
colorbar
caxis([0,1])
set(gca,'YDir','normal')

figure(2);
[~,I] = min(F, [], 2, 'omitnan');
theta_opt = theta(I);
theta_opt(I==1) = NaN;
plot(phi, theta_opt);
xlabel('Force Angle');
ylabel('Optimal Splay Angle');
title(['Gripper ' num2str(n)])
ylim([0,90]);
xlim([0,90]);

figure(3);
Fopt = F(:,I);
for i = 1:length(phi)
    Fopt(i+1:end, i) = 0;
end
Fmax = max(Fopt, [], 1);
Fmax(I==1) = Inf;
plot(phi, 1./Fmax);
xlabel('Force Angle');
ylabel('Maximum Load');
title(['Gripper ' num2str(n)])
ylim([0,1]);
return


%% Fixed theta
n = 2;
k = 1000;
theta = 45;
Phi = linspace(0, 90, 180*k+1)';
Phi = Phi(1:end-1);
dphi = Phi(1+k) - Phi(1);

W = zeros(length(Phi)/k, 1);
for i = 1:length(Phi)
    phi = Phi(i);
    angles = linspace(-theta, theta, n);
    F = 1/sum(cosd(angles-phi).^2);
    Fi = F*cosd(angles-phi);
    Fmax = max(Fi);
    Fb = sum(Fi.*cosd(angles-phi));
    Ft = sum(Fi.*sind(angles-phi));
    w = [1, Ft];
    phi_real = phi + atan2d(w(2), w(1));
    iphi = round((phi_real-Phi(1))/dphi) + 1;
    if min(Fi) >= 0 && iphi > 0
        W(iphi) = norm(w)/Fmax/n;
    end
end
phi = Phi(1:k:end);
W(W<=0) = NaN;
T = cosd(phi);
B = sind(phi);
F = 1./cosd(phi);
% plot(B.*F, W./F);
% plot(phi, W);
hold on
f = 0:.01:1;
for i = 1:length(phi)
    t = f*cosd(phi(i))*W(i);
    b = f*sind(phi(i))*W(i);
    plot(t, b, 'b.');
end
axis equal
k = sqrt(2)/4;
x = 0:.1:1;
plot(x*k, x*k, 'r');
plot((1+x)*k, (1-x)*k, 'r');

%% Original
n = 2;
k = 1000;

Theta = linspace(0, 90, 180+1);
Theta = Theta(2:end-1);
Phi = linspace(0, 90, 180*k+1)';
Phi = Phi(1:end-1);
dphi = Phi(1+k) - Phi(1);

W = zeros(length(Phi)/k, length(Theta));

for i = 1:length(Phi)
    for j = 1:length(Theta)
        phi = Phi(i);
        theta = Theta(j);
        angles = linspace(-theta, theta, n);
        F = 1/sum(cosd(angles-phi).^2);
        Fi = F*cosd(angles-phi);
        Fmax = max(Fi);
        Fb = sum(Fi.*cosd(angles-phi));
        Ft = sum(Fi.*sind(angles-phi));
        w = [1, Ft];
        phi_real = phi + atan2d(w(2), w(1));
        iphi = round((phi_real-Phi(1))/dphi) + 1;
        if min(Fi) >= 0 && iphi > 0
            W(iphi, j) = norm(w)/Fmax/n;
        end
    end
end

theta = Theta;
phi = Phi(1:k:end);
W(theta < phi) = NaN;
W(W<=0) = NaN;
F = 1./W;

figure(1);
imagesc(theta, phi, W)
xlabel('Splay Angle')
ylabel('Force Angle')
title(['Gripper ' num2str(n) ': Maximum Load'])
colorbar
caxis([0,1])
set(gca,'YDir','normal')

figure(2);
[~,I] = min(F, [], 2, 'omitnan');
theta_opt = theta(I);
theta_opt(I==1) = NaN;
plot(phi, theta_opt);
xlabel('Force Angle');
ylabel('Optimal Splay Angle');
title(['Gripper ' num2str(n)])
ylim([0,90]);
xlim([0,90]);

figure(3);
Fopt = F(:,I);
for i = 1:length(phi)
    Fopt(i+1:end, i) = 0;
end
Fmax = max(Fopt, [], 1);
Fmax(I==1) = Inf;
plot(phi, 1./Fmax);
xlabel('Force Angle');
ylabel('Maximum Load');
title(['Gripper ' num2str(n)])
ylim([0,1]);
return