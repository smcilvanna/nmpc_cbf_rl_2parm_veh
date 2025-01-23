function rewardout = getReward(simdata)
% From the simdata, calculate the reward value passed to RL
    min_sep = min(simdata.sep);
    if min_sep < 0
        reward = -1;
        optDist = -1;
        pathDist = -1;
        finishSep = -1;
    else
        tx = simdata.target(1);
        ty = simdata.target(2);
        ox = simdata.obstacle(1);
        oy = simdata.obstacle(2);
        orad = simdata.obstacle(3);
        vrad = simdata.vrad;
        
        % Calculate absolute minimum path to clear
        optrad = orad+vrad;
        % opx = ox + optrad*cos(deg2rad(135));
        % opy = oy + optrad*sin(deg2rad(135));
        % l1 = sqrt(opx^2 + opy^2);
        % l2 = sqrt( (tx-opx)^2 + (ty-opy)^2 );

        p1 = sqrt(ox^2 + oy^2);
        p2 = sqrt( (tx-ox)^2 + (ty-oy)^2);
        l1 = sqrt(p1^2 + orad^2);
        l2 = sqrt(p2^2 + orad^2);
        
        optDist = l1 + l2;
        
        % Calculate path travelled
        diffs = diff(simdata.states(1:2,:),1,2);
        distances = sqrt(sum(diffs.^2,1));
        pathDist = sum(distances);
        reward = optDist/pathDist;
        reward = min(reward,1);
        if reward > 0
            reward = reward^11;
        end
        % check finish position
        finishSep = norm([tx ty] - simdata.states(1:2,end));

        if reward > 0 && finishSep > 0.5
            reward = reward * 0.5/finishSep;
        end
    end
    rewardout.reward = reward;
    rewardout.min_sep = min_sep;
    rewardout.optDist = optDist;
    rewardout.pathDist = pathDist;
    rewardout.finishSep = finishSep;
end