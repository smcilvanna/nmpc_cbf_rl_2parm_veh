
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

%%

    map = generateCurriculumEnvironment(3, false);   % generate random environment
    simdata = initSimdata(map);                                 % create simdata for random environment
    InitialObservation = getObservations(simdata);              % get the intitial observations
    Info = struct('map',map,'simdata',simdata);

%% LOCAL FUNCTIONS

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
    stepTime = 5; % seconds per episode step
    simdata.DT = 0.1;       % HARDCODED value!
    simdata.loopSteps = stepTime/simdata.DT;
    simdata.maxSimTime = 150;
    simdata.maxEpSteps = simdata.maxSimTime / simdata.DT;
    simdata.endSepTol = 0.1;
    simdata.currentState = [0.0, 0.0, deg2rad(45), 0, 0]';
    simdata.obstacles = map.obstacles;
    simdata.target = [ map.targetPos , deg2rad(45) , 0, 0 ]';
    simdata.currentTime = 0.00;
    simdata.mpcIter = 0;
    simdata.ctrlHistory = NaN(simdata.maxEpSteps,2);
    simdata.ssHistory = NaN(simdata.maxEpSteps,1);
    simdata.stateHistory = NaN(5,simdata.maxEpSteps);
    simdata.stateHistory(:,1) = simdata.currentState';
    simdata.simTimeHistory = zeros(simdata.maxEpSteps,1);
    simdata.cLevel = map.cLevel;
    simdata.vrad = 0.55;    % HARDCODED value!
    simdata.end_current_state = [0 , 0 , deg2rad(0.45) , 0 , 0 ];
    simdata.mpcIter = 1;
    simdata.numSteps = 1;
    simdata.states = [0 , 0 , deg2rad(0.45) , 0 , 0 ]';
    simdata.average_mpc_time = 0;   % HARDCODED value!
end

%%
function [nextObs, reward, isDone, Info] = stepFcn(action, Info, solvers, curriculum)
    
    simSettings = initStep(action,solvers);
    

    [newState, simData] = runMPCStep(Info.simdata, action);
    
    % Update logged signals
    Info.simdata = simData;
    
    % Check termination and update curriculum
    if isDone
        curriculum.episodeCount = curriculum.episodeCount + 1;
        updateCurriculum(curriculum); % Your curriculum update logic
    end
    
    % Generate next observation
    nextObs = getObservations(Info.simdata);
    reward = calculateReward(simData);
    isDone = checkTermination(simData);
end


function [simSettings nmpcSolver lastActions] = initStep(action,solvers)
    
    stepTime = 5; % seconds : how long the episode step runs for
    simSettings.solverIdx = Normalizer.denormalize01(action(13),1,numel(nSet));     % get solver index from rl-action
    nmpcSolver = solvers.solverStack(simSettings.solverIdx);                        % return solver struct for rl-action N
    simSettings = initialSimSettings(nmpcSolver,map);   % local function to initalise settings for step sim
    simSettings.loopSteps = stepTime /simSettings.DT;   % set number of steps per loop
    simSettings.normalisedActions = true;               % enable normalised actions flag
    simSettings.realCbfMin  = [0.1 0.01];               % [k1 kr] min real values
    simSettings.realCbfMax  = [100 2.0 ];               % [k1 kr] max real values
    
    simSettings.cbfParms = rand(map.mpcReqObs,2);       % randomise cbf values
    simSettings.realN = solvers.solverN(simSettings.randSolverIdx); % random Nvalue from solver set
    lastActions.N = simSettings.N;              % init last actions to current actions for reward calculation
    lastActions.cbf = simSettings.cbfParms;


        simSettings.controlHorizon = zeros(simSettings.N,2);
    simSettings.X0 = repmat(simSettings.currentState,1,simSettings.N+1)';
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
