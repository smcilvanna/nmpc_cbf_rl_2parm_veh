function reward = getStepReward(simdata, lastActions)
% GETSTEPREWARD Calculate safety-first reward with efficiency incentives
%   Safety priorities: Collision avoidance > Parameter stability > Computation reliability
%   Efficiency goals: Target progress > Velocity maintenance > Compute time

%% Initialize reward components
[rProgress, rVelocity, rComp, rParmStability, rCollision, ...
 rMPCtimeout, rTermGoal, rTermTime, rTermEpTimeout] = deal(0);

%% Safety-Critical Parameters
TERMINAL_GOAL_REWARD =  10;  % Unnormalized, outside weights
COLLISION_PENALTY    = -100;      % Absolute penalty dominates all other rewards
MAX_SAFE_COMP_TIME   =  150;       % ms - Threshold for safety-critical computations
PARM_CHANGE_RATE_PENALTY = 0.5;% Penalty weight for rapid parameter changes

%% Step Reward Components (Normalized weights)
% Weights define priority order: Safety > Progress > Stability > Velocity > Computation
w = [ 1.0,...  % (1) Progress toward target
      1.0,...  % (2) Velocity maintenance
      0.5,...  % (3) Computation efficiency
      0.5,...  % (4) Parameter stability 
      3.0 ];   % (5) Terminal time bonus
sum_weights = sum(w(1:5));
w = w / sum_weights;

%% [1] Progress Reward: Primary efficiency driver
maxDist = simdata.maxVel * (simdata.numSteps * simdata.dt) * 0.85; 
prevGoalDist = norm(simdata.target(1:2) - simdata.states(1:2,1));
currentGoalDist = norm(simdata.target(1:2) - simdata.end_current_state(1:2));
distTravel = prevGoalDist - currentGoalDist;

if distTravel > 0
    % Positive progress: [0,1]
    rProgress = min(distTravel/maxDist, 1); 
else
    % Regress penalty: [-1,0]
    rProgress = max(-1, distTravel/maxDist); 
end

%% [2] Velocity Reward: Balance speed and safety
desVel = simdata.maxVel;
currentVel = simdata.end_current_state(4);

if currentVel >= 0
    % Forward speed reward [0,1]
    rVelocity = min(currentVel/desVel, 1); 
else
    % Reverse penalty [-1,0] 
    rVelocity = max(-1, currentVel/desVel);
end

%% [3] Computation Reward: Prevent safety-critical delays
stepCompTime = simdata.average_mpc_time * 1000; % ms

if stepCompTime <= 5
    rComp = 1;
elseif stepCompTime <= 150
    rComp = 1 - (stepCompTime-5)/145; 
else
    rComp = -1;
end

% Extra penalty for safety-critical delays
if stepCompTime > MAX_SAFE_COMP_TIME
    rComp = rComp - 0.5*abs(stepCompTime/MAX_SAFE_COMP_TIME);
end

%% [4] Parameter Stability: Prevent unsafe jumps
currentParams = [simdata.Naction; simdata.cbf(:)];
lastParams = [lastActions.N; lastActions.cbf(:)];
paramChange = mean(abs(currentParams - lastParams)./currentParams);

% Two-part stability reward
rParmStability = (1 - paramChange) - PARM_CHANGE_RATE_PENALTY*paramChange;

%% Terminal Rewards (Unnormalized Safety Signals)
if simdata.endHitObs
    rCollision = COLLISION_PENALTY; % Overrides all other rewards
end

if simdata.endAtTarget
    rTermGoal = TERMINAL_GOAL_REWARD * simdata.endAtTarget; 
    rTermTime = w(5)*(simdata.maxEpTime - simdata.end_current_time)/simdata.maxEpTime;
end

if simdata.endEpTimeout && ~simdata.endAtTarget
    rTermEpTimeout = -1; % Penalize timeout without reaching goal
end

%% Total Reward Calculation
% Clamp step rewards to [-1,1] for training stability
stepReward = w(1)*rProgress + w(2)*rVelocity + w(3)*rComp + w(4)*rParmStability;
stepReward = max(-1, min(1, stepReward));

% Combine with terminal rewards (safety-critical penalties/rewards)
rTotal = stepReward + rCollision + rTermGoal + rTermTime + rTermEpTimeout;

%% Output Structure
reward = struct(...
    'weights', w,...
    'reward', rTotal,...
    'rProgress', rProgress,...
    'rVelocity', rVelocity,...
    'rComp', rComp,...
    'rParmStability', rParmStability,...
    'rCollision', rCollision,...
    'rTermGoal', rTermGoal,...
    'rTermTime', rTermTime,...
    'rTermEpTimeout', rTermEpTimeout);
end



% function reward = getStepReward(simdata,lastActions)
%     % GETSTEPREWARD Calculate the reward for episode step
%     %
%     %
%     %
% 
%     % Initalise reward components at 0
%     [rProgress , rVelocity , rComp , rParmStability , rCollision , rMPCtimeout , rTermGoal , rTermTime , rTermEpTimeout] = deal(0);
% 
%     % step reward weights
%     w = [   1,...   % (1) Step progress reward/penalty          [-1 1]
%             1,...   % (2) Step forward velocity reward/penalty  [-1 1]
%             1,...   % (3) Step computation time reward/penalty  [-1 1]
%             1,...   % (4) parameter stability                   [-1 1]
%             10,...  % (5) Terminal collision penalty            [-1 or 0] 
%             8,...   % (6) Terminal reached goal reward          [ 0 or 1]
%             3,...   % (7) Terminal time reward                  [0 1]
%             3 ];    % (8) Terminal timeout penalty              [-1 or 0]
% 
%     sum_weights = sum(w);
%     w = w / sum_weights;
% 
% 
%     maxDist = simdata.maxVel * (simdata.numSteps * simdata.dt); % distance could have travelled in this step
%     maxDist = maxDist * 0.85; % set slightly lower max distance
% 
%     % Progress reward: Reward for getting closer to the target
%     %   r_progress = w(1) * (previous_distance_to_goal - current_distance_to_goal)
%     startPos = simdata.states(:,1);
%     endPos   = simdata.end_current_state;
%     targetPos = simdata.target';
%     prevGoalDist = norm(targetPos(1:2)-startPos(1:2));
%     currentGoalDist = norm(targetPos(1:2)-endPos(1:2));
%     distTravel = (prevGoalDist - currentGoalDist);
%     if distTravel > 0
%         % Moving closer: reward [0, 1]
%         distTravel = min(maxDist,distTravel); % upper limit for travel distance for next calculation
%         rProgress = distTravel / maxDist;   % Reward [0 1]
%     elseif distTravel <=0
%         % Moving away: penalty [-1, 0]  
%         distTravel = min(maxDist,abs(distTravel-0.1));
%         rProgress = - (distTravel / maxDist);
%     end
% 
% 
% 
%     % Velocity maintenance reward: Reward for maintaining desired speed
%     %       r_velocity = β * (1 - |desired_velocity - current_velocity| / desired_velocity)
%     desVel = simdata.maxVel;  % Use full max velocity as target
%     currentVel = endPos(4);   % Use actual velocity (no capping)
% 
%     if currentVel >= 0
%         % Forward movement reward/penalty (peaks at desVel)
%         velError = abs(desVel - currentVel);
%         rVelocity = 1 - (velError / desVel);  % [1 - 2*|error|] → [-1, 1]
%     else
%         % Backward movement penalty (asymmetric)
%         reverseSpeed = abs(currentVel);
%         % Base penalty + 30% extra penalty for any reverse movement
%         rVelocity = - (reverseSpeed + 0.3*desVel) / desVel;  
%         % Clamp to [-1.3, -0.3] then scale to [-1, -0.23]
%         rVelocity = max(-1, min(-0.23, rVelocity));
%     end
% 
% 
%     % Computational efficiency reward: Penalize long compute times asymmetrically
%     % maxCompTime = 200;  % 200 ms → -1 reward
%     stepCompTime = simdata.average_mpc_time * 1000;  % Convert to ms
% 
%     if stepCompTime <= 5
%         % Maximum reward for fastest compute times [0-5ms]
%         rComp = 1;
%     elseif stepCompTime <= 100
%         % Linear decay from 1→0 between 5-100ms
%         rComp = 1 - ((stepCompTime - 5) / (100 - 5));
%     elseif stepCompTime <= 200
%         % Linear penalty from 0→-1 between 100-200ms
%         rComp = -((stepCompTime - 100) / (200 - 100));
%     else
%         % Minimum penalty for exceeding 200ms
%         rComp = -1;
%     end
% 
%     % Parameter stability reward: Small reward for not drastically changing parameters
%     %     r_stability = δ * (1 - |current_parameters - previous_parameters| / parameter_range)
%     %   calculate seperate rewards for each cbf and horizon parameter, would work for real or normalised actions
%     nStabReward  = ( 1 - abs(simdata.Naction - lastActions.N) );   % normalised range is 0 1
%     k1StabReward = ( 1 - max(abs(simdata.cbf(:,1)-lastActions.cbf(:,1))));
%     krStabReward = ( 1 - max(abs(simdata.cbf(:,2)-lastActions.cbf(:,2))));
%     rParmStability = (nStabReward + k1StabReward + krStabReward)/3;
% 
%      % >>>> Terminal Rewards
% 
%     % Collision penalty: Large negative reward for collisions
%     if simdata.endHitObs
%         rCollision = -1;
%     end
% 
%     if simdata.endAtTarget
%         rTermGoal = 1;
%         rTermTime = ( (simdata.maxEpTime-simdata.end_current_time) / simdata.maxEpTime );
%     end
% 
%     if simdata.endEpTimeout
%         rTermEpTimeout = -1;
%     end
% 
%     % Calculate the total reward
%     rTotal = w(1)*rProgress + w(2)*rVelocity + w(3)*rComp + w(4)*rParmStability + w(5)*rCollision + w(6)*rTermGoal + w(7)*rTermTime + w(8)*rTermEpTimeout ;
%     reward = struct;
%     reward.weights = w;
%     reward.reward = rTotal;
%     reward.rProgress = rProgress;
%     reward.rVelocity = rVelocity;
%     reward.rComp = rComp;
%     reward.rParmStability = rParmStability;
%     reward.rCollision = rCollision;
%     reward.rMPCtimeout = rMPCtimeout;
%     reward.rTermGoal = rTermGoal;
%     reward.rTermTime = rTermTime;
%     reward.rTermEpTimeout = rTermEpTimeout;
% 
% end