return
%% 
disp("")

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

%% Setup Random Environment
close all;
targetPos = [30 , 30];
env = generateRandomEnvironment(4, 1.0, 30, targetPos,[5.0;5.0;5.0] );
figure(env.fig);
clearvars targetPos
%% Run Dynamic Solver Step Loop [Dynamic Solver]
% create solver stack
import casadi.*
settings.Nvals = 10:10:50;
settings.nObs = height(env.obstacles);
solvers = createSolversMultiObs(settings);
fprintf("%d NMPC solvers created\nN-Min : %d\nN-max: %d\n\n",numel(settings.Nvals),min(settings.Nvals),max(settings.Nvals)); 
clearvars settings; 
%% Initial Step Sim Settings [Dynamic Solver]

simSettings = initialSimSettings(solvers,env);  % local function to initalise settings for step sim

%% Single Shot Sim for number of steps [Dynamic Solver]
disp("Starting Simulation")
simdata = simulationStepDyn(nmpcSolver, simSettings);
disp("Simulation Complete")

%% Static Plot for Dynamic Multi Obstacle [Dynamic Solver]
close all; staticPlot= true; viewOnScreen = false;
fig = visualiseSimulationDyn(simdata,staticPlot,viewOnScreen);
figure(fig);


%%
clearvars fig staticPlot targetPos viewOnScreen
%% Loop Step Sim Until Done [Dynamic Solver]
simSettings = initialSimSettings(solvers,env);  % local function to initalise settings for step sim
disp("Starting Simulation")
isDone = false;
simdata = [];
allSimdata = [];
stepRewards = [];
lastActions.N = simSettings.N;
lastActions.Nrange = 100;
lastActions.cbf = simSettings.cbfParms;
lastActions.k1range = 100;
lastActions.krrange = 1;

nmpcSolver = solvers.solverStack(1);

while ~isDone
    % run the simulation step
    simdata = simulationStepDyn(nmpcSolver, simSettings);
    % log simdata
    allSimdata = [allSimdata ; simdata ];
    % calculate rewards for this step
    stepReward = getStepReward(simdata,lastActions);
    stepRewards = [stepRewards , stepReward ];
    % update simSettings for next step
    simSettings.X0 = simdata.end_X0;
    simSettings.currentTime = simdata.end_current_time;
    simSettings.currentState = simdata.end_current_state;
    simSettings.mpcIter = simdata.mpcIter;
    simSettings.ctrlHistory = simdata.usafe;
    simSettings.ssHistory = simdata.sep;
    simSettings.stateHistory = simdata.states;
    simSettings.cbfParms = simSettings.cbfParms + 1;
    % update last actions for next step
    lastActions.N = simdata.N;
    lastActions.cbf = simdata.cbf;
    % check if done
    isDone = simdata.endAtTarget || simdata.endEpTimeout || simdata.endHitObs;
    fprintf(".");
end
fprintf("\nSimulation Complete\n\n");

%%
close all; staticPlot= true; viewOnScreen = false;
fig = visualiseSimulationDyn(simdata,staticPlot,viewOnScreen);
figure(fig);





%%
%%

%% >>> Environment Validation <<<
if ~exist("agent","var")
    agent = loadAgentFile();
end
% Setup Random Environment
close all;
targetPos = [50 , 50];
env = generateRandomEnvironment(5, 1.0, 30, targetPos,[5.0;5.0;5.0] );
figure(env.fig);
clearvars targetPos
   
%% create solver
import casadi.*
nmpc.DT = 0.1; 
nmpc.N = 20;
nmpc.velMax = 2;
nmpc.accMax = 5;
nmpc.cbfParms = [0, 0];
nmpc.veh_rad = 0.55;
nmpc.nObs = height(env.obstacles);
nmpcSolver = createMPCDynamicObsSolver(nmpc);
clearvars nmpc; disp("Solver Created")
%% create sim settings
simSettings = setInitialStepSimSettings(100,1000,nmpcSolver,env);
disp("Simulation Settings Created")

%% Set CBF parameters
for i = 1:height(env.obstacles)
    test.obs = simSettings.obstacles(i,3);
    test.actionN = getAction(agent, {test.obs});
    test.action = denormaliseAction(test.actionN{1});
    simSettings.cbfParms(i,1) = test.action(1); %k1
    simSettings.cbfParms(i,2) = test.action(1)/test.action(2); %k2 (k1/kr)
end

% simSettings.cbfParms = simSettings.cbfParms .* 0.8;
%% Single Shot Sim for number of steps [Dynamic Solver]
disp("Starting Simulation")
simdata = simulationStepDyn(nmpcSolver, simSettings);
disp("Simulation Complete")

%% Static Plot for Dynamic Multi Obstacle [Dynamic Solver]
close all; staticPlot= true; viewOnScreen = false;
fig = visualiseSimulationDyn(simdata,staticPlot,viewOnScreen);
figure(fig);


%% LOCAL FUNCTIONS

function [ agent ] = loadAgentFile()
    [fileName, filePath] = uigetfile('*.mat', 'Select a MAT-file', './temp_data');
    if isequal(fileName, 0)
        disp('File selection canceled.');   % Check if the user selected a file or canceled the operation
    else
        load( fullfile(filePath, fileName) , 'agent', 'verID','trainID'); % construct full path and load into workspace
        disp(['Loaded MAT-file: ', fullfile(filePath, fileName)]);
        fprintf(" Trained Agent TD3%s-%s\n",verID,trainID);
    end
end


function simSettings = initialSimSettings(solvers,env)
    simSettings.cbfParms = repmat([22 , 86],5,1);
    simSettings.N = solvers.solverN(1);
    simSettings.DT = solvers.solverStack(1).settings.DT;
    simSettings.veh_rad = solvers.solverStack(1).settings.veh_rad;
    simSettings.loopSteps = 3;
    simSettings.maxSimTime = 100;
    simSettings.maxEpSteps = simSettings.maxSimTime / simSettings.DT;
    simSettings.endSepTol = 0.1;
    simSettings.currentState = [0.0, 0.0, deg2rad(45), 0, 0]';
    simSettings.obstacles = env.obstacles;
    simSettings.target = [ env.targetPos , deg2rad(45) , 0, 0 ]';
    simSettings.currentTime = 0.00;
    simSettings.mpcIter = 0;
    simSettings.ctrlHistory = NaN(simSettings.maxEpSteps,2);
    simSettings.ssHistory = NaN(simSettings.maxEpSteps,1);
    simSettings.stateHistory = NaN(5,simSettings.maxEpSteps);
    simSettings.stateHistory(:,1) = simSettings.currentState';
    simSettings.simTimeHistory = zeros(simSettings.maxEpSteps,1);
    simSettings.controlHorizon = zeros(simSettings.N,2);
    simSettings.X0 = repmat(simSettings.currentState,1,simSettings.N+1)';

    disp("Simulation INITIAL Settings Created");

end