function reward = getStepReward(simdata,lastActions)
    % GETSTEPREWARD Calculate the reward for episode step
    %
    %
    %
    
    % Initalise reward components at 0
    [rProgress , rVelocity , rComp , rParmStability , rCollision , rMPCtimeout , rTermGoal , rTermTime , rTermEpTimeout] = deal(0);

    % step reward weights
    w = [   1,...   % (1) Step progress reward/penalty          [-1 1]
            1,...   % (2) Step forward velocity reward/penalty  [-1 1]
            1,...   % (3) Step computation time reward/penalty  [-1 1]
            1,...   % (4) parameter stability                   [-1 1]
            10,...  % (5) Terminal collision penalty            [-1 or 0] 
            8,...   % (6) Terminal reached goal reward          [ 0 or 1]
            3,...   % (7) Terminal time reward                  [0 1]
            3 ];    % (8) Terminal timeout penalty              [-1 or 0]

    sum_weights = sum(w);
    w = w / sum_weights;


    maxDist = simdata.maxVel * (simdata.numSteps * simdata.dt); % distance could have travelled in this step
    maxDist = maxDist * 0.85; % set slightly lower max distance

    % Progress reward: Reward for getting closer to the target
    %   r_progress = w(1) * (previous_distance_to_goal - current_distance_to_goal)
    startPos = simdata.states(:,1);
    endPos   = simdata.end_current_state;
    targetPos = simdata.target';
    prevGoalDist = norm(targetPos(1:2)-startPos(1:2));
    currentGoalDist = norm(targetPos(1:2)-endPos(1:2));
    distTravel = (prevGoalDist - currentGoalDist);
    if distTravel > 0
        % Moving closer: reward [0, 1]
        distTravel = min(maxDist,distTravel); % upper limit for travel distance for next calculation
        rProgress = distTravel / maxDist;   % Reward [0 1]
    elseif distTravel <=0
        % Moving away: penalty [-1, 0]  
        distTravel = min(maxDist,abs(distTravel-0.1));
        rProgress = - (distTravel / maxDist);
    end



    % Velocity maintenance reward: Reward for maintaining desired speed
    %       r_velocity = β * (1 - |desired_velocity - current_velocity| / desired_velocity)
    desVel = simdata.maxVel;  % Use full max velocity as target
    currentVel = endPos(4);   % Use actual velocity (no capping)
    
    if currentVel >= 0
        % Forward movement reward/penalty (peaks at desVel)
        velError = abs(desVel - currentVel);
        rVelocity = 1 - (velError / desVel);  % [1 - 2*|error|] → [-1, 1]
    else
        % Backward movement penalty (asymmetric)
        reverseSpeed = abs(currentVel);
        % Base penalty + 30% extra penalty for any reverse movement
        rVelocity = - (reverseSpeed + 0.3*desVel) / desVel;  
        % Clamp to [-1.3, -0.3] then scale to [-1, -0.23]
        rVelocity = max(-1, min(-0.23, rVelocity));
    end
    
    
    % Computational efficiency reward: Penalize long compute times asymmetrically
    % maxCompTime = 200;  % 200 ms → -1 reward
    stepCompTime = simdata.average_mpc_time * 1000;  % Convert to ms
    
    if stepCompTime <= 5
        % Maximum reward for fastest compute times [0-5ms]
        rComp = 1;
    elseif stepCompTime <= 100
        % Linear decay from 1→0 between 5-100ms
        rComp = 1 - ((stepCompTime - 5) / (100 - 5));
    elseif stepCompTime <= 200
        % Linear penalty from 0→-1 between 100-200ms
        rComp = -((stepCompTime - 100) / (200 - 100));
    else
        % Minimum penalty for exceeding 200ms
        rComp = -1;
    end
    
    % Parameter stability reward: Small reward for not drastically changing parameters
    %     r_stability = δ * (1 - |current_parameters - previous_parameters| / parameter_range)
    %   calculate seperate rewards for each cbf and horizon parameter, would work for real or normalised actions
    nStabReward  = ( 1 - abs(simdata.Naction - lastActions.N) );   % normalised range is 0 1
    k1StabReward = ( 1 - max(abs(simdata.cbf(:,1)-lastActions.cbf(:,1))));
    krStabReward = ( 1 - max(abs(simdata.cbf(:,2)-lastActions.cbf(:,2))));
    rParmStability = (nStabReward + k1StabReward + krStabReward)/3;

     % >>>> Terminal Rewards

    % Collision penalty: Large negative reward for collisions
    if simdata.endHitObs
        rCollision = -1;
    end

    if simdata.endAtTarget
        rTermGoal = 1;
        rTermTime = ( (simdata.maxEpTime-simdata.end_current_time) / simdata.maxEpTime );
    end

    if simdata.endEpTimeout
        rTermEpTimeout = -1;
    end
   
    % Calculate the total reward
    rTotal = w(1)*rProgress + w(2)*rVelocity + w(3)*rComp + w(4)*rParmStability + w(5)*rCollision + w(6)*rTermGoal + w(7)*rTermTime + w(8)*rTermEpTimeout ;
    reward = struct;
    reward.weights = w;
    reward.reward = rTotal;
    reward.rProgress = rProgress;
    reward.rVelocity = rVelocity;
    reward.rComp = rComp;
    reward.rParmStability = rParmStability;
    reward.rCollision = rCollision;
    reward.rMPCtimeout = rMPCtimeout;
    reward.rTermGoal = rTermGoal;
    reward.rTermTime = rTermTime;
    reward.rTermEpTimeout = rTermEpTimeout;

end