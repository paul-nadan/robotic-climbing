function plotRobot(r)
    global PLOT
    if ~PLOT
        return
    end
    hold on;
    for iBody = 1:length(r.bodies)
        body = r.bodies{iBody};
        fill3(body(1,:),-body(3,:), body(2,:), 'r');
    end
    plotPoints(r.vertices(:,sum(r.gait.feet, 2)>0), 'b.');
%     plotPoints(r.feet(:,~r.c), 'r.');
    for i = 1:size(r.vertices,2)
       plotLine(r.vertices(:,i), r.vertices(:,i)-r.links(:,i), 'b');
    end
end