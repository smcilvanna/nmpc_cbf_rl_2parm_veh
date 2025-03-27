
return
%% 
blah

%% 
cd /home/sm/matlab/cbfRL/nmpc_cbf_rl_2parm_veh
addpath("functions/");
addpath('/home/sm/matlab/com/casadi-3.6.7/');   % ### ADJUST PATH TO CASADI PACKAGE LOACTION #### 

%% Setup the NMPC Solver for Environment
import casadi.*
setnmpc.DT = 0.1; 
setnmpc.velMax = 2;
setnmpc.accMax = 5;
setnmpc.cbfParms = [1.0, 1.0]; % initial values, gets updated in parameters args
setnmpc.obs_rad = 1;           % initial values, gets updated in parameters args
setnmpc.veh_rad = 0.55;
setnmpc.N = 20;
[solver, args, f] = createMPCDynamicSolver(setnmpc);


%% Setup Environment For NMPC-CBF 2 parameter training

obsInfo = rlFiniteSetSpec([0.5 1.0 5.0 10.0]);
actInfo = rlNumericSpec([2 1], 'LowerLimit', [1; 1], 'UpperLimit', [100; 100]);
env = rlFunctionEnv(obsInfo, actInfo, @(action, loggedSignals) stepFunction(action, loggedSignals, solver, args, f), @() resetFunction());

%% TD3 Network Setup
% % Create the actor and critic networks
% statePath = [
%     imageInputLayer([1 1 1], 'Normalization', 'none', 'Name', 'state')
%     fullyConnectedLayer(64, 'Name', 'fc1')
%     reluLayer('Name', 'relu1')
%     fullyConnectedLayer(64, 'Name', 'fc2')
%     reluLayer('Name', 'relu2')
%     fullyConnectedLayer(2, 'Name', 'out')
% ];
% actorNet = layerGraph(statePath);
% 
% statePath = [
%     imageInputLayer([1 1 1], 'Normalization', 'none', 'Name', 'state')
%     fullyConnectedLayer(64, 'Name', 'fc1')
%     reluLayer('Name', 'relu1')
%     fullyConnectedLayer(64, 'Name', 'fc2')
%     reluLayer('Name', 'relu2')
% ];
% actionPath = [
%     imageInputLayer([2 1 1], 'Normalization', 'none', 'Name', 'action')
%     fullyConnectedLayer(64, 'Name', 'fc3')
% ];
% commonPath = [
%     additionLayer(2, 'Name', 'add')
%     reluLayer('Name', 'relu3')
%     fullyConnectedLayer(1, 'Name', 'out')
% ];
% criticNet = layerGraph(statePath);
% criticNet = addLayers(criticNet, actionPath);
% criticNet = addLayers(criticNet, commonPath);
% criticNet = connectLayers(criticNet, 'relu2', 'add/in1');
% criticNet = connectLayers(criticNet, 'fc3', 'add/in2');
% 
% % Create the agent
% agent = rlTD3Agent(actor(actorNet, obsInfo, actInfo), critic(criticNet, obsInfo, actInfo), ...
%     'SampleTime', 1, ...
%     'ExperienceBufferLength', 1e6, ...
%     'MiniBatchSize', 128);
% 
% % Set up training options
% trainOpts = rlTrainingOptions(...
%     'MaxEpisodes', 1000, ...
%     'MaxStepsPerEpisode', 1, ...
%     'ScoreAveragingWindowLength', 100, ...
%     'Verbose', false, ...
%     'Plots', 'training-progress');

% Define the actor network
actorLayers = [
    imageInputLayer([1 1 1], 'Normalization', 'none', 'Name', 'state')
    fullyConnectedLayer(64, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(64, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(2, 'Name', 'out')
];
actorNet = layerGraph(actorLayers);

% Define the critic network (two Q-networks for TD3)
criticLayers = [
    imageInputLayer([1 1 1], 'Normalization', 'none', 'Name', 'state')
    fullyConnectedLayer(64, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(64, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(1, 'Name', 'out')
];
criticNet = layerGraph(criticLayers);

% Create the TD3 agent
agent = rlTD3Agent(actor(actorNet, obsInfo, actInfo), critic(criticNet, obsInfo, actInfo), ...
    'SampleTime', 1, ...
    'ExperienceBufferLength', 1000, ...
    'MiniBatchSize', batch_size, ...
    'DiscountFactor', gamma, ...
    'ActorLearningRate', learning_rate, ...
    'CriticLearningRate', learning_rate, ...
    'TargetSmoothFactor', 0.05, ...
    'ExplorationModel', rlAdditiveGaussianExploration(action_noise), ...
    'PolicyUpdateFrequency', 1);

% Set up training options
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 1000, ...
    'MaxStepsPerEpisode', 1, ...
    'ScoreAveragingWindowLength', 100, ...
    'Verbose', true, ...
    'Plots', 'training-progress', ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 200, ...
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 200);


%% Train the agent
trainingStats = train(agent, env, trainOpts);


%%

%%

%%

%% LOCAL FUNCTIONS

% Step function (one step per episode)
function [nextObs, reward, isDone, loggedSignals] = stepFunction(action, loggedSignals, solver, args, f)
    settings.cbfParms = [ action(1) ; action(2) ];
    settings.obs_rad = loggedSignals.obs;
    simdata = simulationLoopDyn(solver,args,f, settings);
    reward = getReward(simdata);    % set weights in getReward function
    nextObs = settings.obs;         % In this case, obs doesn't change
    isDone = true;                  % One step per episode so done after each step
end

% Define the reset function
function [initialObs, loggedSignals] = resetFunction()
    obsSet = [0.5 1.0 5.0 10.0];
    initialObs = obsSet(randi(length(obsSet)));
    loggedSignals.obs = initialObs;
end