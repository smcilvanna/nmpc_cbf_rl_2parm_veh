
return
%% 
blah

%% 
cd /home/sm/matlab/cbfRL/nmpc_cbf_rl_2parm_veh
addpath("functions/");
addpath('/home/sm/matlab/com/casadi-3.6.7/');
clc; disp("Done")
%% Setup the NMPC Solver for Environment
import casadi.*
setnmpc.DT = 0.1; 
setnmpc.velMax = 2;
setnmpc.accMax = 5;
setnmpc.cbfParms = [1.0, 1.0]; % initial values, gets updated in parameters args
setnmpc.obs_rad = 1;           % initial values, gets updated in parameters args
setnmpc.veh_rad = 0.55;
setnmpc.N = 20;
nmpcSolver = struct;
nmpcSolver.settings = setnmpc;
[nmpcSolver.solver , nmpcSolver.args, nmpcSolver.f] = createMPCDynamicSolver(setnmpc);
clearvars setnmpc; disp("NMPC Solver Created");
% Setup Environment For NMPC-CBF 2 parameter training

obsInfo = rlFiniteSetSpec([0.5 1.0 5.0 10.0]);
actInfo = rlNumericSpec([2 1], 'LowerLimit', [1; 1], 'UpperLimit', [100; 100]);
env = rlFunctionEnv(obsInfo, actInfo, @(action, loggedSignals) stepFunction(action, loggedSignals, nmpcSolver), @() resetFunction());
disp("RL Environment Created");
% TD3 Network Setup

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
    'MiniBatchSize', 32,...
    'DiscountFactor', 0.0001,...
    'TargetSmoothFactor', 0.05,...  
    'TargetUpdateFrequency', 2,...
    'ActorOptimizerOptions', rlOptimizerOptions('LearnRate',5e-4), ...
    'CriticOptimizerOptions',rlOptimizerOptions('LearnRate',5e-4), ...
    'PolicyUpdateFrequency', 2);
agentOpts.ExplorationModel.StandardDeviation = [0.2 ; 0.2];
agentOpts.ExplorationModel.StandardDeviationDecayRate = 0;

% Create TD3 Agent
agent = rlTD3Agent(actor, [critic1 critic2], agentOpts);
disp("RL TD3 Agent Created");
%% Training Configuration
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 5000,...                       % Run for set number of episodes
    'MaxStepsPerEpisode', 1,...
    'ScoreAveragingWindowLength', 100,...
    'Verbose', true,...
    'Plots', 'training-progress',...
    'StopTrainingCriteria', 'None',...            % Don't stop based on reward
    'SaveAgentCriteria', 'Episodes',...           % Save based on episode count
    'SaveAgentValue', 1000,...                    % Save every 1000 episodes
    'SaveAgentDirectory', 'trained_agents');
disp("Training Options Set")
%% Train the agent
disp(">>> TRAINING START <<<")
trainingStats = train(agent, env, trainOpts);


%% temp
agent_5kint5k = agent
trainingStats5i5= trainingStats
%%

%% LOCAL FUNCTIONS

% Step function (one step per episode)
function [nextObs, reward, isDone, loggedSignals] = stepFunction(action, loggedSignals, nmpcSolver)
    settings = struct;
    % Need this to handle the initial stepFunction validation before the reset function is run
    if ~isfield(loggedSignals, 'obs') || isempty(loggedSignals.obs)
        settings.obs_rad = 1.0;
    else
        settings.obs_rad = loggedSignals.obs;
    end
    % create settings struct for simulation scenario
    settings.cbfParms = [ action(1) ; action(2) ];
    settings.veh_rad = nmpcSolver.settings.veh_rad;
    settings.N = nmpcSolver.settings.N;
    settings.DT = nmpcSolver.settings.DT;
    settings.endSepTol = 0.1;
    settings.maxSimTime = 40;
    % Run simulation with current settings
    simdata = simulationLoopDyn(nmpcSolver.solver,nmpcSolver.args,nmpcSolver.f, settings);
    rewardout = getReward(simdata);    % set weights in getReward function
    reward = rewardout.reward;
    nextObs = settings.obs_rad;     % In this case, obs doesn't change as will be modified by reset function
    isDone = true;                  % One step per episode so done after each step
end

% Define the reset function
function [initialObs, loggedSignals] = resetFunction()
    obsSet = [0.5 1.0 5.0 10.0];
    initialObs = obsSet(randi(length(obsSet)));
    loggedSignals.obs = initialObs;
end