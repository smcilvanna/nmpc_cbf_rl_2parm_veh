function reward = getStepReward(simdata,lastActions)
    % GETSTEPREWARD Calculate the reward for episode step
    %
    %
    %
    
    % Initalise reward components at 0
    [rProgress , rVelocity , rComp , rParmStability , rCollision , rMPCtimeout , rTermGoal , rTermTime , rTermEpTimeout] = deal(0);

    % step reward weights
    w = [   1,...   % progress
            1,...   % velocity
            1,...   % computation
            1,...   % nStab
            1,...   % k1Stab
            1];     % krStab

    % penalty weights, negative values are assigned below
    pen = [ 100,... % collision
            10,...  % computation time
            10];    % not used
    
    % Terminal rewards/penalty weights, any negative values assigned below
    wt = [  100,... % at target
            100,... % efficiency bonus (at target faster)
            100 ];  % episode timeout penalty

    % Progress reward: Reward for getting closer to the target
    %   r_progress = w(1) * (previous_distance_to_goal - current_distance_to_goal)
    startPos = simdata.states(:,1);
    endPos   = simdata.end_current_state;
    targetPos = simdata.target';
    prevGoalDist = norm(targetPos(1:2)-startPos(1:2));
    currentGoalDist = norm(targetPos(1:2)-endPos(1:2));
    rProgress = w(1) * (prevGoalDist - currentGoalDist);

    % Velocity maintenance reward: Reward for maintaining desired speed
    %       r_velocity = β * (1 - |desired_velocity - current_velocity| / desired_velocity)

    desVel = 2.0*0.9;                                       % desired velocity as 90% of max velocity
    currentVel = min(endPos(4),desVel);                     % cap the current velocity at desired velocity for next calculation
    rVelocity = w(2) * (1 - abs(desVel-currentVel)/desVel); % velocity reward
    
    
    % Computational efficiency reward: Reward for keeping computation time low
    %   r_computation = γ * (1 - computation_time / max_allowed_computation_time)
    
    maxCompTime = 100;                                  % dont want to exceed 100ms
    stepCompTime = simdata.average_mpc_time*1000;       % average comp time in previous steps
    stepCompTime = min(stepCompTime,maxCompTime);       % cap at max for next calculation
    rComp = w(3) * (1 - stepCompTime/maxCompTime);      % calculate comp reward
    
    % Parameter stability reward: Small reward for not drastically changing parameters
    %     r_stability = δ * (1 - |current_parameters - previous_parameters| / parameter_range)
    %   calculate seperate rewards for each cbf and horizon parameter, would work for real or normalised actions
    nStabReward = w(4) * ( 1 - abs(simdata.N - lastActions.N)/lastActions.Nrange );
    k1StabReward = w(5) * ( 1 - max(abs(simdata.cbf(:,1)-lastActions.cbf(:,1)))/lastActions.k1range );
    krStabReward = w(6) * ( 1 - max(abs(simdata.cbf(:,2)-lastActions.cbf(:,2)))/lastActions.krrange );
    rParmStability = nStabReward + k1StabReward + krStabReward;

    % Computational timeout penalty: Penalty if MPC doesn't solve within time limit
    if simdata.average_mpc_time*1000 > 200
        rMPCtimeout = -pen(2);
    end

     % >>>> Terminal Rewards

    % Collision penalty: Large negative reward for collisions
    if simdata.endHitObs
        rCollision = -pen(1);
    end

    if simdata.endAtTarget
        rTermGoal = wt(1);
        rTermTime = wt(2)*(( simdata.maxEpTime-simdata.end_current_time)/simdata.maxEpTime);
    end

    if simdata.endEpTimeout
        rTermEpTimeout = -wt(3);
    end
   
    % Calculate the total reward
    rTotal = rProgress + rVelocity + rComp + rParmStability + rCollision + rMPCtimeout + rTermGoal + rTermTime + rTermEpTimeout ;
    reward = struct;
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