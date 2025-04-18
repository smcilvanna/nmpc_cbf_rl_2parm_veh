
return
%% 
blah

%% Linux
cd /home/sm/matlab/cbfRL/nmpc_cbf_rl_2parm_veh
addpath("functions/");
addpath('/home/sm/matlab/com/casadi-3.6.7/');
clc; disp("Done")
%% Windows (Win 11 Laptop)
cd("C:\Users\14244039\OneDrive - Queen's University Belfast\win_11_qub\Documents\MATLAB\nmpc_cbf_rl_2parm_veh");
addpath("functions\");
addpath("C:\Users\14244039\AppData\Roaming\MathWorks\MATLAB Add-Ons\Collections\casadi-3.7.0-windows64-matlab2018b");
clc; disp("Done");


%% Setup Environment For NMPC-CBF 2 parameter training
% This environment will train to learn the optimal NMPC-CBF controller CBF parameters (2) and Horizon-N length.
% Reward function is defined in matlab function getStepReward()
% The environment will be randomly created and difficulty progressivly increased during the training

% Observations = [ Vehicle State(7) ; Obstacle-Information(25)       ; MPC-Information(1) ; Target-Information(1) ]
%                   Lin/Ang Vel (2)          dist/ang obs (18)[3*6]       prevCompTime(1)       %progress2goal(1)
%                     prev vels (2)             obs radii (6)
%               target dist/ang (3)        density curLvl (1)

%% create solver stack
import casadi.*
settings.Nvals = 10:10:100;
settings.nObs = 6;
solvers = createSolversMultiObs(settings);
fprintf("%d NMPC solvers created\nN-Min : %d\nN-max: %d\n\n",numel(settings.Nvals),min(settings.Nvals),max(settings.Nvals)); 
clearvars settings; 

%% load solvers
load("temp_data/10_10_100_solvers.mat","solvers");
%% Training environment

% Create curriculum handle object
curriculum = CurriculumLevel();

% Define Observation Specifications 
obsInfo = rlNumericSpec([34 1],...
    'LowerLimit', [-1 -1 -1 -1 0 -1 -1 zeros(1,24) 0 0 0]',...
    'UpperLimit', [ 1  1  1  1 1  1  1  ones(1,24) 1 1 1]');

% Define Action Specifications 6*2 cbf parameters (for 6 obstacles) + 1 horizonN value
actInfo = rlNumericSpec([13 1], 'LowerLimit',0,'UpperLimit',1);

% Anonymous function handles 
resetEpisode = @() resetFcn(curriculum);
stepEpisode = @(action, Info) stepFcn(action, Info, solvers, curriculum);

% Create environment
env = rlFunctionEnv(obsInfo, actInfo, stepEpisode, resetEpisode);
disp("Environment Created")

%% TD3 AGENT INITALISATION
% actor network
actorLayers = [
    featureInputLayer(34, 'Name', 'state', 'Normalization', 'none')
    fullyConnectedLayer(256, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(256, 'Name', 'fc2') 
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(13, 'Name', 'action')  % Matches action dimension
    tanhLayer('Name', 'tanh_out')              % Constrain to [-1,1]
];

actor = rlContinuousDeterministicActor(layerGraph(actorLayers), obsInfo, actInfo);
disp("Actor Network Created")
%% critics networks

% State pathway
statePath = [
    featureInputLayer(34, 'Name', 'state', 'Normalization', 'none')
    fullyConnectedLayer(256, 'Name', 'state_fc1')
    reluLayer('Name', 'state_relu')
];

% Action pathway  
actionPath = [
    featureInputLayer(13, 'Name', 'action', 'Normalization', 'none')
    fullyConnectedLayer(256, 'Name', 'action_fc1')
    reluLayer('Name', 'action_relu')
];

% Common fusion path
commonPath = [
    concatenationLayer(1, 2, 'Name', 'concat')
    reluLayer('Name', 'common_relu')
    fullyConnectedLayer(256, 'Name', 'common_fc2')
    reluLayer('Name', 'common_relu2')
    fullyConnectedLayer(1, 'Name', 'QValue') % Single output
];

% Assemble critic network
criticNet = layerGraph(statePath);
criticNet = addLayers(criticNet, actionPath);
criticNet = addLayers(criticNet, commonPath);

% Explicit connections
criticNet = connectLayers(criticNet, 'state_relu', 'concat/in1');
criticNet = connectLayers(criticNet, 'action_relu', 'concat/in2');

% Create twin critics with verified network
critic1 = rlQValueFunction(criticNet, obsInfo, actInfo,...
    'ObservationInputNames','state',...
    'ActionInputNames','action');

critic2 = rlQValueFunction(criticNet, obsInfo, actInfo,...
    'ObservationInputNames','state',...
    'ActionInputNames','action');

% Apply weight perturbation
params = getLearnableParameters(critic1);
perturbedParams = cellfun(@(x) x + 0.01*randn(size(x)), params, 'UniformOutput', false);
critic2 = setLearnableParameters(critic2, perturbedParams);
disp("Critics Networks Created")
%% TD3 agent
agentOpts = rlTD3AgentOptions(...
    'SampleTime', 1,...
    'ExperienceBufferLength', 1e6,...
    'MiniBatchSize', 256,...
    'DiscountFactor', 0.99,...
    'TargetSmoothFactor', 0.005,...
    'TargetUpdateFrequency', 2,...
    'ActorOptimizerOptions', rlOptimizerOptions('LearnRate',1e-4),...
    'CriticOptimizerOptions', rlOptimizerOptions('LearnRate',1e-3));

agentOpts.ExplorationModel.StandardDeviation = 0.1*ones(13,1);
agentOpts.ExplorationModel.StandardDeviationDecayRate = 1e-5;

agent = rlTD3Agent(actor, [critic1 critic2], agentOpts);
disp("New TD3 Agent Created")
%% Traning Options
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 8000,...
    'MaxStepsPerEpisode', 30,...     % 150s total / 5s steps
    'ScoreAveragingWindowLength', 50,...
    'StopTrainingCriteria', 'None',...
    'UseParallel', false,...
    'Verbose', true,...
    'Plots', 'training-progress');

disp("Training Options Set")

%% TRAIN
disp(">>> TRAINING START <<<")
trainID = "1-1"; verID = "v1"; fname = "train_multiObs_td3_" + verID + "_" + trainID + ".mat";
trainingStats = train(agent, env, trainOpts);
save(fname);

%% LOCAL FUNCTIONS

%% RESET FUNCTION

function [InitialObservation, Info] = resetFcn(curriculum)
%%RESETFN : reset scenario for new episode
%               - Create a new environment map based on current curriculum level
%               - Return the initial set of observations to RL-agent

    map = generateCurriculumEnvironment(curriculum.level, false);   % generate random environment
    simdata = initSimdata(map);                                 % create simdata for random environment
    InitialObservation = getObservations(simdata);              % get the intitial observations
    Info = struct('map',map,'simdata',simdata);

end

function simdata = initSimdata(map)
%%INITSIMDATA :  Return simdata to generate observations from intial random map environment
    simdata = struct;
    simdata.obstacles = map.obstacles;
    simdata.target = [ map.targetPos , deg2rad(45) , 0, 0 ]';
    simdata.cLevel = map.cLevel;
    simdata.vrad = 0.55;    % HARDCODED value!
    simdata.end_current_state = [0 , 0 , deg2rad(45) , 0 , 0 ];
    simdata.mpcIter = 0;
    simdata.numSteps = 0;
    simdata.states = [0 , 0 , deg2rad(0.45) , 0 , 0 ]';
    simdata.average_mpc_time = 0;   % HARDCODED value!
    simdata.end_current_time = 0.00;
end

%% STEP FUNCTION
function [nextObs, reward, isDone, Info] = stepFcn(action, Info, solvers, curriculum)
    

    solverIdx = Normalizer.denormalize01(action(13),1,numel(solvers.solverN));     % get solver index from rl-action
    solverIdx = round(solverIdx);
    nmpcSolver = solvers.solverStack(solverIdx);                        % select solver for this step from action
    
    [simSettings, lastActions] = initStep(action,Info.simdata,nmpcSolver.settings,Info.map);
    
    
    % run the simulation step
    Info.simdata = simulationStepDyn(nmpcSolver, simSettings);
    
    % Update logged signals
    
    isDone = Info.simdata.endAtTarget || Info.simdata.endEpTimeout || Info.simdata.endHitObs;
    
    % Check termination and update curriculum
    if isDone
            % Update curriculum only on successful episodes
        if Info.simdata.endAtTarget
            curriculum.episodeCount = curriculum.episodeCount + 1;
            % Increase level after number of successes at current difficulty
            if mod(curriculum.episodeCount, 200) == 0
                curriculum.level = min(curriculum.level + 1, 5);
                % curriculum.episodeCount = 0; % Reset counter for new level
                fprintf("[TRAIN-INFO] Curriculum Level [%d]\n",curriculum.level)
            end
        end
    end
    
    % Get observations for this step
    nextObs = getObservations(Info.simdata);

    % Calculate reward value for this step
    rw = getStepReward(Info.simdata, lastActions);
    reward = rw.reward;   


end


function [simSettings, lastActions] = initStep(action,simdata,solverSettings,map)
    
    epStepTime = 5; % seconds per episode step
    epMaxTime = 150; % seconds per episode (timeout)
    
    simSettings.normalisedActions = true;               % enable normalised actions flag
    simSettings.realCbfMin  = [0.1 0.01];               % [k1 kr] min real values
    simSettings.realCbfMax  = [100 2.0 ];               % [k1 kr] max real values
    simSettings.cbfParms = reshape(action(1:12) ,6,2 );     % cbf actions into array
    simSettings.N = solverSettings.N;
    simSettings.DT = solverSettings.DT;
    simSettings.veh_rad = solverSettings.veh_rad;
    simSettings.loopSteps = epStepTime / simSettings.DT;
    simSettings.maxSimTime = epMaxTime;
    simSettings.maxEpSteps = simSettings.maxSimTime / simSettings.DT;
    simSettings.endSepTol = 0.1;
    simSettings.currentState = simdata.end_current_state;
    simSettings.obstacles = map.obstacles;
    simSettings.target = [ map.targetPos , deg2rad(45) , 0, 0 ]';
    simSettings.currentTime = simdata.end_current_time;
    simSettings.mpcIter = simdata.mpcIter;
    simSettings.cLevel = map.cLevel;
    if simSettings.mpcIter == 0
        simSettings.ctrlHistory = NaN(simSettings.maxEpSteps,2);
        simSettings.ssHistory = NaN(simSettings.maxEpSteps,1);
        simSettings.stateHistory = NaN(5,simSettings.maxEpSteps);
        simSettings.stateHistory(:,1) = simSettings.currentState';
        simSettings.simTimeHistory = zeros(simSettings.maxEpSteps,1);
        simSettings.controlHorizon = zeros(simSettings.N,2);
        simSettings.X0 = repmat(simSettings.currentState,1,simSettings.N+1)';
        lastActions.N = action(13);
        lastActions.cbf = simSettings.cbfParms;
    else
        simSettings.ctrlHistory = simdata.usafe;
        simSettings.ssHistory = simdata.sep;
        simSettings.stateHistory = simdata.states;
        simSettings.simTimeHistory = zeros(simSettings.maxEpSteps,1);
        [simSettings.X0, simSettings.controlHorizon] = resizeX0newN(simdata.end_X0,...
                                                                    simSettings.N,...
                                                                    simdata.end_control_horizon); % reshape mpc arrays for new horizon
    
        lastActions.N = simdata.Naction;
        lastActions.cbf = simSettings.cbfParms;
    end

end


function [X0 , controlHorizon] = resizeX0newN(lastX0,N,lastCH)
%RESIZEX0NEWN : for changing horizon length need to update size of state and control arrays for mpc sovler
%
    lastN = height(lastX0)-1;   % find horizon length of last step
    if N <= lastN               % if new N is smaller take subset of arrays
        X0 = lastX0(1:N+1,:);
        controlHorizon = lastCH(1:N,:);
    else                        % if new N is larger, expand array using last iteration values
        extraRows = N-lastN;
        X0 = [lastX0; repmat(lastX0(end,:),extraRows,1)];
        controlHorizon = [lastCH ; repmat(lastCH(end,:),extraRows,1)];
    end
end
















































%% TD3 Network Setup

% Actor Network
actorLayers = [
    featureInputLayer(1, 'Name', 'state', 'Normalization', 'none')
    fullyConnectedLayer(64, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(64, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(2, 'Name', 'action')  % Output layer must match action dimensions
];

% Validate and create actor
assert(isequal(obsInfo.Dimension, [1 1]), 'Observation spec mismatch')
assert(isequal(actInfo.Dimension, [2 1]), 'Action spec mismatch')
actor = rlContinuousDeterministicActor(layerGraph(actorLayers), obsInfo, actInfo);
disp("RL Actor Created");


% TD3 Critic Networks Setup
% Critic Network Construction

% Create input layers for state and action
obsInput = featureInputLayer(1, 'Name', 'state', 'Normalization', 'none');
actInput = featureInputLayer(2, 'Name', 'action', 'Normalization', 'none');

% Build separate processing paths
obsPath = [
    obsInput
    fullyConnectedLayer(64, 'Name', 'obs_fc1')
    reluLayer('Name', 'obs_relu')
];

actPath = [
    actInput
    fullyConnectedLayer(64, 'Name', 'act_fc1')
    reluLayer('Name', 'act_relu')
];

% Common processing after concatenation
commonPath = [
    concatenationLayer(1, 2, 'Name', 'concat')
    fullyConnectedLayer(64, 'Name', 'fc2')
    reluLayer('Name', 'common_relu')
    fullyConnectedLayer(1, 'Name', 'QValue')
];

% Assemble layer graph properly
criticNet = layerGraph();
criticNet = addLayers(criticNet, obsPath);
criticNet = addLayers(criticNet, actPath);
criticNet = addLayers(criticNet, commonPath);

% Connect layers explicitly
criticNet = connectLayers(criticNet, 'obs_relu', 'concat/in1');
criticNet = connectLayers(criticNet, 'act_relu', 'concat/in2');

% Create Twin Critics with Different Initial Weights
% First critic
critic1 = rlQValueFunction(criticNet, obsInfo, actInfo,...
    'ObservationInputNames','state',...
    'ActionInputNames','action');

% Second critic with perturbed weights
critic2 = rlQValueFunction(criticNet, obsInfo, actInfo,...
    'ObservationInputNames','state',...
    'ActionInputNames','action');

% Apply small random perturbation to second critic's weights
params = getLearnableParameters(critic1);
perturbedParams = cellfun(@(x) x + 0.01*randn(size(x)), params, 'UniformOutput', false);
critic2 = setLearnableParameters(critic2, perturbedParams);
disp("RL Critics Created");


% TD3 Agent Configuration
agentOpts = rlTD3AgentOptions(...
    'SampleTime', 1,...
    'ExperienceBufferLength', 10000,...
    'MiniBatchSize', 64,...
    'DiscountFactor', 0.0001,...
    'TargetSmoothFactor', 0.05,...  
    'TargetUpdateFrequency', 2,...
    'ActorOptimizerOptions', rlOptimizerOptions('LearnRate',5e-4), ...
    'CriticOptimizerOptions',rlOptimizerOptions('LearnRate',5e-4), ...
    'PolicyUpdateFrequency', 2);
agentOpts.ExplorationModel.StandardDeviation = [0.4 ; 0.4];
agentOpts.ExplorationModel.StandardDeviationDecayRate = 0.01;

% Create TD3 Agent
agent = rlTD3Agent(actor, [critic1 critic2], agentOpts);
disp("RL TD3 Agent Created");
%% Training Configuration
numEps = 8000;
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', numEps,...                       % Run for set number of episodes
    'MaxStepsPerEpisode', 1,...
    'ScoreAveragingWindowLength', 100,...
    'Verbose', true,...
    'Plots', 'none',...
    'StopTrainingCriteria', 'None');
disp("Training Options Set")

%% Create Data Logger for episode information

global episodeInfo 
episodeInfo = zeros(numEps,5); % observation, reward, k1, k1/k2, k2

logger = rlDataLogger();

% Configure Episode Finished Callback
% logger.EpisodeFinishedFcn = @(info) episodeFinishedCallback(info, logger);
logger.EpisodeFinishedFcn = @episodeFinishedCallback;
disp("Episode Data Logger Enabled");

%% Train the agent
disp(">>> TRAINING START <<<")
trainID = "3-2"; verID = "v2"; fname = "train_td3" + verID + "_" + trainID + ".mat";
trainingStats = train(agent, env, trainOpts, 'Logger', logger);
save(fname);


%%


%%
[obs, logs] = resetFunction([0.1, 10.0]);
disp(obs); % Should be 0.1 or 10.0
disp(logs); % Should not error

action = [0.5; -0.3]; % 2x1 vector
[nextObs, reward, done, logs] = stepFunction(action, struct(), nmpcSolver);


%% LOCAL FUNCTIONS

% Step function (one step per episode)
function [nextObs, reward, isDone, LoggedSignals] = stepFunction(action, loggedSignals, nmpcSolver)
    assert(numel(action) == 2, 'Action must be a 2-element vector');
    settings = struct;
    % Need this to handle the initial stepFunction validation before the reset function is run
    if ~isfield(loggedSignals, 'obs') || isempty(loggedSignals.obs)
        settings.obs_rad = 1.0;
    else
        settings.obs_rad = denormaliseObservation(loggedSignals.obs);
    end
    % create settings struct for simulation scenario
    dact = denormaliseAction(action);
    k1 = dact(1);
    k2 = k1/dact(2);
    if k1 > 0 && k2 > 0
        settings.cbfParms = [ k1 ; k2 ];
        settings.veh_rad = nmpcSolver.settings.veh_rad;
        settings.N = nmpcSolver.settings.N;
        settings.DT = nmpcSolver.settings.DT;
        settings.endSepTol = 0.1;
        settings.maxSimTime = 100;
        % Run simulation with current settings
        simdata = simulationLoopDyn(nmpcSolver.solver,nmpcSolver.args,nmpcSolver.f, settings);
        rewardout = getReward(simdata);    % set weights in getReward function
        reward = rewardout.reward;
    else
        reward = -1;                % skip invalid negative parameters
    end
    nextObs = settings.obs_rad;     % In this case, obs doesn't change as will be modified by reset function
    isDone = true;                  % One step per episode so done after each step
    LoggedSignals = loggedSignals;
end

% Define the reset function
function [initialObs, loggedSignals] = resetFunction(obsSet)
    % obsSet = [0.5 1.0 5.0 10.0];
    % initialObs = obsSet(randi(length(obsSet)));           % discrete observations
    minObs = obsSet(1) ; maxObs = obsSet(2);
    realObs = round(minObs + (maxObs - minObs) * rand(), 1);     % continious observations
    initialObs = normaliseObservation(realObs);                 % normalise the observation
    loggedSignals.obs = initialObs;
end

% define logger to record actions/observations during training
function dataToLog = episodeFinishedCallback(info)
    global episodeInfo % observation, reward, k1, k1/k2, k2
    i = info.EpisodeCount;
    episodeInfo(i,1) = info.Experience.Observation{1}; 
    episodeInfo(i,2) = info.Experience.Reward; 
    episodeInfo(i,3) = info.Experience.Action{1}(1); 
    episodeInfo(i,4) = info.Experience.Action{1}(2); 
    episodeInfo(i,5) = (info.Experience.Action{1}(1) / info.Experience.Action{1}(2)); 
    fprintf(" >> Observation  %5.2f | K1 %7.3f K1/K2 %7.3f\n", info.Experience.Observation{1}, info.Experience.Action{1}(1), info.Experience.Action{1}(2)  );

    dataToLog = [];  % Prevents file saving
end
