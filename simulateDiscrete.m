addpath('terrain-generation', 'config-generation', 'robot-kinematics', ...
    'optimization', 'visualization', 'discrete-model');
close all
config = quadruped([3,3,2,2], 0.1, 0.3, {.2, [.16, .16]}, 0, 2);
res = 0.05;
k = 3;

tic();
[X, Y, Z] = meshgrid([-1 0 1], [-1 0 1], [-1 0 1]);
neighbors = [reshape(X,[],1), reshape(Y,[],1), reshape(Z,[],1)]*res;
neighbors = neighbors(any(neighbors, 2),:);
% neighbors = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1]*res;
f0 = [.3,.4,-.3];
r0 = discreteStep(f0, k, 1, config, []);
if r0.fail
    fprintf('Initial point failed');
    return
end
results = table(f0, r0);
edges = table(f0+neighbors, repmat(r0, length(neighbors), 1));

while size(edges, 1) > 0
    fprintf('Nodes: %d, Edges: %d\n', size(results,1), size(edges,1));
    f = edges{1,1};
    x = robot2state(edges{1,2});
    edges = edges(2:end,:);
    clf;
    r = discreteStep(f, k, 1, config, x);
    results(end+1,:) = {f, r};
    if ~r.fail
%         drawnow();
        for i = 1:length(neighbors)
            if ~any(all(f + neighbors(i,:) == [results{:,1};edges{:,1}],2))
                edges(end+1, :) = {f + neighbors(i,:), r};
            end
        end
    end
end
runtime = toc();
fprintf('Runtime: %f\n', runtime);
f = results{:,1};
r = results{:,2};
success = ~[r.fail]';
score = sum(success*res^3);
fprintf('Score: %d\n', score);

clf;
plot3(f(success,1),f(success,2),f(success,3),'b.');
hold on; axis equal;
plot3(f(~success,1),f(~success,2),f(~success,3),'r.');

f1 = min(f(:,1)):res:max(f(:,1));
f2 = min(f(:,2)):res:max(f(:,2));
f3 = min(f(:,3)):res:max(f(:,3));
[X,Y,Z] = meshgrid(f1, f2, f3);
D = Z*0;
for i = 1:size(f,1)
    i1 = round((f(i,1)-f1(1))/res + 1);
    i2 = round((f(i,2)-f2(1))/res + 1);
    i3 = round((f(i,3)-f3(1))/res + 1);
    D(i2, i1, i3) = 1.001*(~r(i).fail);    
end

[fo,vo] = isosurface(f1,f2,f3,D,1);               % isosurface for the outside of the volume
[fe,ve,ce] = isocaps(f1,f2,f3,D,1);               % isocaps for the end caps of the volume

p1 = patch('Faces', fo, 'Vertices', vo, 'FaceAlpha',.1);       % draw the outside of the volume
p1.FaceColor = 'red';
p1.EdgeColor = 'black';

p2 = patch('Faces', fe, 'Vertices', ve, ...    % draw the end caps of the volume
   'FaceVertexCData', ce, 'FaceAlpha',.1);
p2.FaceColor = 'yellow';
p2.EdgeColor = 'none';

xlim([f1(1), f1(end)])
ylim([f2(1), f2(end)])
zlim([f3(1), f3(end)])