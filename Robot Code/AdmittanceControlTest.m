% for w = 0:.1:1
w = 0.2;
rng(1);
G = rand(6, 12);
dx = rand(12,1);

G2 = G(:, 4:end);
dx2 = dx(4:end);
A2 = (eye(9) - G2'*inv(G2*G2')*G2);
dy2 = A2*dx2;

I = eye(12);
W = [w,w,w,1,1,1,1,1,1,1,1,1];
% I(1:3, 1:3) = w;
% G(:,1:3) = G(:,1:3)*w;
I = I.*W;
G = G.*W;
A = (I - G'*inv(G*G')*G)
dy = A*dx;

dy(4:end) - dy2;

% plot(w, dy(1), 'r.');
% hold on
% plot(w, dy(2), 'b.');
% plot(w, dy(3), 'g.');
% plot(w, dy(4), 'rs');
% plot(w, dy(5), 'bs');
% plot(w, dy(6), 'gs');
% plot(w, dy(7), 'r*');
% plot(w, dy(8), 'b*');
% plot(w, dy(9), 'g*');
% plot(w, dy(10), 'ro');
% plot(w, dy(11), 'bo');
% plot(w, dy(12), 'go');
% end