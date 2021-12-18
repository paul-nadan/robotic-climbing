% Theta = linspace(0, pi/2, 9+1);
% Theta = Theta(2:end-1);
% Phi = linspace(0, pi/2, 9+1)';
% Phi = Phi(1:end-1);
% 
% W = zeros(length(Phi), length(Theta));
% 
% syms f psi
% for i = 1:length(Phi)
%     for j = 1:length(Theta)
%         phi = Phi(i);
%         theta = Theta(j);
%         f1 = f*cos(psi-theta);
%         f2 = f*cos(psi);
%         f3 = f*cos(psi+theta);
%         eq1 = f1*cos(theta) + f2 + f3*cos(theta) == cos(phi);
%         eq2 = f1*sin(theta) - f3*sin(theta) == sin(phi);
%         psi0 = solve(eq1, psi);
%         psi0 = psi0(subs(psi0, f, 1) > 0);
%         F = double(solve(subs(eq2, psi, psi0(1)), f));
%         F = F(F > 0);
%         Psi1 = double(solve(subs(eq1, 'f', F), psi));
%         Psi2 = double(solve(subs(eq2, 'f', F), psi));
%         % Need to get f1 not f, and check that f3 > 0
%         W(i, j) = 1/(3*F);
%     end
% end
% theta = Theta;
% phi = Phi;
% W(theta < phi) = NaN;
% F = 1./W;
% 
% figure(1);
% imagesc(rad2deg(theta), rad2deg(phi), W)
% xlabel('Splay Angle')
% ylabel('Force Angle')
% title('Maximum Load')
% colorbar
% caxis([0,1])
% set(gca,'YDir','normal')
% 
% figure(2);
% [~,I] = min(F, [], 2, 'omitnan');
% theta_opt = theta(I);
% plot(rad2deg(phi), rad2deg(theta_opt));
% xlabel('Force Angle');
% ylabel('Optimal Splay Angle');
% 
% figure(3);
% Fopt = F(:,I);
% for i = 1:length(phi)
%     Fopt(i+1:end, i) = 0;
% end
% Fmax = max(Fopt, [], 1);
% plot(rad2deg(phi), 1./Fmax);
% xlabel('Force Angle');
% ylabel('Maximum Load');
% return

theta = linspace(0, pi/2, 900+1);
theta = theta(2:end-1);
phi = linspace(0, pi/2, 900+1)';
phi = phi(1:end-1);
F = 2*sin(phi+theta)./(2*cos(theta).*sin(theta));
F2 = 2*sin(-phi+theta)./(2*cos(theta).*sin(theta));
F(theta < phi) = NaN;
W = 1./F;

figure(1);
imagesc(rad2deg(theta), rad2deg(phi), W)
xlabel('Splay Angle')
ylabel('Force Angle')
title('Maximum Load')
colorbar
caxis([0,1])
set(gca,'YDir','normal')

figure(2);
[~,I] = min(F, [], 2, 'omitnan');
theta_opt = theta(I);
plot(rad2deg(phi), rad2deg(theta_opt));
xlabel('Force Angle');
ylabel('Optimal Splay Angle');

figure(3);
Fopt = F(:,I);
for i = 1:length(phi)
    Fopt(i+1:end, i) = 0;
end
Fmax = max(Fopt, [], 1);
plot(rad2deg(phi), 1./Fmax);
xlabel('Force Angle');
ylabel('Maximum Load');

figure(4);
y = [-1; 0; 1];
torque = cos(theta) + y*sin(theta);
plot(rad2deg(theta), torque);
legend('-1', '0', '1')