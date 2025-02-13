function rewardout = getReward(simdata,weights)
% From the simdata, calculate the reward value passed to RL
        
    if exist("weights","var")
        if numel(weights) ~= 3
            fprintf("[ERROR] Need 3 element weight vector for reward function.\n[Path-Efficiency  End-Seperation  Average-Velocity]");
            return
        end
    else
        weights = [1 1 1];  % default to equal weighting
    end
    % Normalise weights so sum = 1
    tw = sum(weights);  
    wp = weights(1)/tw;
    ws = weights(2)/tw;
    wv = weights(3)/tw;
    vrad = 0.55;

    %min_sep = min(simdata.sep);
    nObs = size(simdata.obstacle,2);
    minseps = ones(nObs,1)*100;
    for o = 1:nObs
        for s = 1:size(simdata.states,2)
            stepsep = norm(simdata.obstacle(1:2,o) - simdata.states(1:2,s)) - vrad - simdata.obstacle(3,o) ;
            if stepsep < minseps(o)
                minseps(o) = stepsep;
            end
        end
    end

    min_sep = min(minseps);

    if min_sep < 0
        reward = -1;
        optDist = -1;
        pathDist = -1;
        endSep = -1;
        maxEndSep = -1;
        maxVel = -1;
        aveVel = -1;
        simtime = -1;
        rp = 0; rs = 0; rv = 0;
        wp = 0; ws = 0; wv = 0;
    else
        tx = simdata.target(1);
        ty = simdata.target(2);
        % ox = simdata.obstacle(1);
        % oy = simdata.obstacle(2);
        % orad = simdata.obstacle(3);
        % vrad = simdata.vrad;
        
        % % Calculate absolute minimum path to clear
        % optrad = orad+vrad;
        % p1 = sqrt(ox^2 + oy^2);
        % p2 = sqrt( (tx-ox)^2 + (ty-oy)^2);
        % l1 = sqrt(p1^2 + orad^2);
        % l2 = sqrt(p2^2 + orad^2);
        % optDist = l1 + l2;

        % Calculate direct path to target
        optDist = norm([tx ty]);
        
        % Calculate path travelled and end seperation
        diffs = diff(simdata.states(1:2,:),1,2);
        distances = sqrt(sum(diffs.^2,1));
        pathDist = sum(distances);
        endSep = norm([tx ty]' - simdata.states(1:2,end) ); % Calculate seperation at end of simulation
        pathDist = pathDist + endSep; % need to add on any remaning distance to the target also

        % Calculate average velocity
        simtime = size(simdata.usafe,1) * simdata.dt; 
        aveVel = pathDist/simtime;

        % Calculate Reward
        % Reward = Path-Efficiency + End-Position + Average-Velocity

        rp = optDist/pathDist;                          % Path-efficiency reward
        
        maxEndSep = 0.5;
        rs = 1 - ( min(endSep,maxEndSep) / maxEndSep ); % End Position reward

        if isfield(simdata, 'maxVel') && ~isempty(simdata.maxVel)
            maxVel = simdata.maxVel;
        else
            maxVel = 10;
        end
        
        if aveVel >= maxVel
            aveVel = maxVel;
            disp("Check average velocity reward calculation!!");
            % ave should always be < max, put in this to alert for rogue result
        end
        
        rv = aveVel/maxVel;                             % Average-velocity reward
        % wp = 4/6; 
        % ws = 1/6; 
        % wv = 2/6;                   % Reward component weightings
        reward = rp*wp + rs*ws + rv*wv;                 % total reward
        
    end
    rewardout.reward = reward;
    rewardout.rp = rp;
    rewardout.rs = rs;
    rewardout.rv = rv;
    rewardout.weights = [wp ws wv];    
    rewardout.min_sep = min_sep;
    rewardout.optDist = optDist;
    rewardout.pathDist = pathDist;
    rewardout.endSep = endSep;
    rewardout.endSepMax = maxEndSep;
    rewardout.maxVel = maxVel;
    rewardout.aveVel = aveVel;
    rewardout.simtime = simtime;
end


%% OLD
% 
% function rewardout = getReward(simdata)
% % From the simdata, calculate the reward value passed to RL
%     min_sep = min(simdata.sep);
%     if min_sep < 0
%         reward = -1;
%         optDist = -1;
%         pathDist = -1;
%         finishSep = -1;
%     else
%         tx = simdata.target(1);
%         ty = simdata.target(2);
%         ox = simdata.obstacle(1);
%         oy = simdata.obstacle(2);
%         orad = simdata.obstacle(3);
%         vrad = simdata.vrad;
% 
%         % Calculate absolute minimum path to clear
%         optrad = orad+vrad;
%         % opx = ox + optrad*cos(deg2rad(135));
%         % opy = oy + optrad*sin(deg2rad(135));
%         % l1 = sqrt(opx^2 + opy^2);
%         % l2 = sqrt( (tx-opx)^2 + (ty-opy)^2 );
% 
%         p1 = sqrt(ox^2 + oy^2);
%         p2 = sqrt( (tx-ox)^2 + (ty-oy)^2);
%         l1 = sqrt(p1^2 + orad^2);
%         l2 = sqrt(p2^2 + orad^2);
% 
%         optDist = l1 + l2;
% 
%         % Calculate path travelled
%         diffs = diff(simdata.states(1:2,:),1,2);
%         distances = sqrt(sum(diffs.^2,1));
%         pathDist = sum(distances);
%         finishSep = norm([tx ty]' - simdata.states(1:2,end) ); % check end seperation
%         %need to add on any remaning distance to the target also
%         pathDist = pathDist + finishSep;
% 
%         reward = optDist/pathDist;
%         reward = min(reward,1);
%         if reward >= 1
%             disp("reward?")
%         end
%         if reward > 0
%             reward = reward^11;
%         end
% 
% 
% 
%         if reward > 0 && finishSep > 0.5
%             reward = reward * 0.5/finishSep;
%         end
%     end
%     rewardout.reward = reward;
%     rewardout.min_sep = min_sep;
%     rewardout.optDist = optDist;
%     rewardout.pathDist = pathDist;
%     rewardout.finishSep = finishSep;
% end