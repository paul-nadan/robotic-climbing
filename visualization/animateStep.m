function animateStep(r1, r2, dt, count, grid)
%     return
    i = mod(count-1, size(r1.gait.angles, 2))+1;
    global FRAMES RECORD ANIMATE
    if ~ANIMATE && ~RECORD
        return
    end
    dBody = cell(1,length(r1.bodies));
    for iBody = 1:length(r1.bodies)
        dBody{iBody} = r2.bodies{iBody} - r1.bodies{iBody};
    end
    dJoints = (r2.vertices-r2.links) - (r1.vertices-r1.links);
    dVertices = r2.vertices - r1.vertices;
    plotTerrain(grid);
    plotPoints(r1.vertices(:,r1.gait.feet(:,i)>0), 'b.');
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
            FRAMES(length(FRAMES)+1) = getframe(gcf);
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