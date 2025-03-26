
return
%% 
blah

%% 
cd /home/sm/matlab/cbfRL/nmpc_cbf_rl_2parm_veh
addpath("functions/")
%%

obsInfo = rlFiniteSetSpec([0.5 1.0 5.0 10.0]);
actInfo = rlNumericSpec([2 1], 'LowerLimit', [1; 1], 'UpperLimit', [100; 100]);
env = rlFunctionEnv(obsInfo, actInfo, @(action, loggedSignals) stepFunction(action, loggedSignals), @() resetFunction());

% Create the actor and critic networks
statePath = [
    imageInputLayer([1 1 1], 'Normalization', 'none', 'Name', 'state')
    fullyConnectedLayer(64, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(64, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(2, 'Name', 'out')
];
actorNet = layerGraph(statePath);

statePath = [
    imageInputLayer([1 1 1], 'Normalization', 'none', 'Name', 'state')
    fullyConnectedLayer(64, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(64, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
];
actionPath = [
    imageInputLayer([2 1 1], 'Normalization', 'none', 'Name', 'action')
    fullyConnectedLayer(64, 'Name', 'fc3')
];
commonPath = [
    additionLayer(2, 'Name', 'add')
    reluLayer('Name', 'relu3')
    fullyConnectedLayer(1, 'Name', 'out')
];
criticNet = layerGraph(statePath);
criticNet = addLayers(criticNet, actionPath);
criticNet = addLayers(criticNet, commonPath);
criticNet = connectLayers(criticNet, 'relu2', 'add/in1');
criticNet = connectLayers(criticNet, 'fc3', 'add/in2');

% Create the agent
agent = rlTD3Agent(actor(actorNet, obsInfo, actInfo), critic(criticNet, obsInfo, actInfo), ...
    'SampleTime', 1, ...
    'ExperienceBufferLength', 1e6, ...
    'MiniBatchSize', 128);

% Set up training options
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 1000, ...
    'MaxStepsPerEpisode', 1, ...
    'ScoreAveragingWindowLength', 100, ...
    'Verbose', false, ...
    'Plots', 'training-progress');

% Train the agent
trainingStats = train(agent, env, trainOpts);




%% LOCAL FUNCTIONS

% Step function (one step per episode)
function [nextObs, reward, isDone, loggedSignals] = stepFunction(action, loggedSignals)
    settings.k1 = action(1);
    settings.k2 = action(2);
    settings.obs = loggedSignals.obs;
    reward = simulateRun(settings);
    nextObs = settings.obs;  % In this case, obs doesn't change
    isDone = true;  % Always done after one step
end

% Define the reset function
function [initialObs, loggedSignals] = resetFunction()
    obsSet = [0.5 1.0 5.0 10.0];
    initialObs = obsSet(randi(length(obsSet)));
    loggedSignals.obs = initialObs;
end