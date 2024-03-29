function animateStep(r1, r2, dt, count, grid)
    i = mod(count-1, size(r1.gait.angles, 2))+1;
    global FRAMES RECORD ANIMATE
%     if ~ANIMATE && ~RECORD
%         return
%     end
    dBody = cell(1,length(r1.bodies));
    for iBody = 1:length(r1.bodies)
        dBody{iBody} = r2.bodies{iBody} - r1.bodies{iBody};
    end
    dJoints = (r2.vertices-r2.links) - (r1.vertices-r1.links);
    dVertices = r2.vertices - r1.vertices;
    plotTerrain(grid);
    
    if count > 1
        [F1, ~, ~, ~] = quasiStaticDynamicsKnownForce(r1, count-1, r1.F, grid);
        plotForces(r1, F1, count-1, 'g', 0.016);
    end
    
    if r1.fail
        plotPoints(r1.vertices(:,(r1.gait.feet(:,i)==1)>0), 'r.');        
    elseif r1.skip
        plotPoints(r1.vertices(:,(r1.gait.feet(:,i)==1)>0), 'm.');
    else
        plotPoints(r1.vertices(:,(r1.gait.feet(:,i)==1)>0), 'b.');
    end
    for t = 0:dt:1
        for iBody = 1:length(r1.bodies)
            body = r1.bodies{iBody}+t*dBody{iBody};
            g.bodies(iBody) = fill3(body(1,:), -body(3,:), body(2,:), 'r');
        end
        vertices = r1.vertices + dVertices*t;
        joints = r1.vertices - r1.links + dJoints*t;
        for iVertex = 1:size(r1.vertices,2)
            g.links(iVertex) = plotLine(joints(:,iVertex),...
                vertices(:,iVertex), 'b');
        end
        drawnow();
        if RECORD
            if isempty(FRAMES)
                FRAMES = [getframe(gcf)];
            else
                FRAMES = [FRAMES,getframe(gcf)];
%                 FRAMES(length(FRAMES)+1) = getframe(gcf);
            end
        end
        if t ~= 1
            for iBody = 1:length(g.bodies)
                delete(g.bodies(iBody));
            end
            for iLink = 1:length(g.links)
                delete(g.links(iLink));
            end
        end
    end
end