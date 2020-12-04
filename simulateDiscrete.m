close all
k = 3;
config = quadruped([3,3,2,2], 0.1, 0.3, {.2, [.16, .16]}, 0, 2);

f = [.3;.4;-.3];
r = discreteStep(f, k, 1, config, []);
return
            
f1 = -.2:.05:.5;
f2 = -.4:.05:.6;
f3 = -.6:.05:0;
res = zeros(length(f1),length(f2),length(f3));
points = zeros(3,0);
ipoints = zeros(3,0);
tic();
for i1 = 1:length(f1)
    for i2 = 1:length(f2)
        for i3 = 1:length(f3)
            clf;
            f = [f1(i1); f2(i2); f3(i3)];
            r = discreteStep(f, k, 1, config, []);
            res(i1, i2, i3) = ~r.fail;
            if ~r.fail
                points = [points, f];
                ipoints = [ipoints, [i1; i2; i3]];
                drawnow();
%                 fprintf('Success [%d, %d, %d]\n', f1, f2, f3);
            end
        end
    end
end
toc()
% imagesc(res(:,:,1));
% colorbar;
score = sum(res, 'all')/sum(res*0+1, 'all')
clf;

[X,Y,Z] = meshgrid(f1,f2,f3);
D = Z*0;
for i = 1:size(ipoints,2)
    D(ipoints(2,i), ipoints(1,i), ipoints(3,i)) = 1.001;    
end

[fo,vo] = isosurface(f1,f2,f3,D,1);               % isosurface for the outside of the volume
[fe,ve,ce] = isocaps(f1,f2,f3,D,1);               % isocaps for the end caps of the volume

figure
plot3(points(1,:), points(2,:), points(3,:), 'r.');
figure
plot3(points(1,:), points(2,:), points(3,:), 'r.');

p1 = patch('Faces', fo, 'Vertices', vo, 'FaceAlpha',.1);       % draw the outside of the volume
p1.FaceColor = 'red';
p1.EdgeColor = 'black';

p2 = patch('Faces', fe, 'Vertices', ve, ...    % draw the end caps of the volume
   'FaceVertexCData', ce, 'FaceAlpha',.1);
p2.FaceColor = 'yellow';
p2.EdgeColor = 'none';

xlim([min(f1) max(f1)])
ylim([min(f2) max(f2)])
zlim([min(f3) max(f3)])