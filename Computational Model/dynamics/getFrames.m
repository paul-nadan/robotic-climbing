% g contains transforms from object frame to each link frame
% m_l contains mass matrices for each link
% m_o contains mass matrices for the robot body
function [g_pl, g_sl, m_l, m_o] = getFrames(config)
    N = size(config.joints, 3);
    syms q(t) [N+6, 1]
    q = q(t);
    g_pl = cell(N,1);
    g_sl = cell(N,1);
    for iLink = 1:N
        R = vrrotvec2mat_sym([config.joints(:,2,iLink); q(iLink)]);
        if config.parents(iLink) == 0
            Rp = eye(3);
            xp = zeros(3,1);
            x_sl = R*config.joints(:,3,iLink);
        else
            Rp = g_pl{config.parents(iLink)}(1:3, 1:3);
            xp = g_pl{config.parents(iLink)}(1:3, 4);
            xp_sl = g_sl{config.parents(iLink)}(1:3, 4);
            link = Rp*R*config.joints(:,3,iLink);
            x_sl = xp_sl + Rp*config.joints(:,1,iLink)+link;
        end
        link = Rp*R*config.joints(:,3,iLink);
        x = xp + Rp*config.joints(:,1,iLink)+link;
        g_pl{iLink} = [Rp*R x; 0 0 0 1];
        g_sl{iLink} = [Rp*R x_sl; 0 0 0 1];
    end
    m_l = cell(N, 1);
    for iLink = 1:N
        m_l{iLink} = eye(6)*(sum(config.gait.feet(iLink, :), 2) > 0);
    end
    m_o = 5*diag([1, 1, 1, 1, 1, 1]);
end