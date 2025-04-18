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
% close all;
% targetPos = [40 , 40];
% env = generateRandomEnvironment(5, 1.0, 20, targetPos, (1.0:1.0:10) );
% figure(env.fig);
% clearvars targetPos
%% ############################################################## 
%% Setup For Step Simulation Loop [Dynamic Variable N Solver]
% create environment map
close all
cLevel = 4;
map = generateCurriculumEnvironment(cLevel,true);
figure(map.fig);
ax = findall(map.fig, 'type', 'axes');  % Find all axes in the figure
axes(ax); axis square; grid on;
%% create solver stack
import casadi.*
settings.Nvals = 10:20:100;
settings.nObs = map.mpcReqObs;
solvers = createSolversMultiObs(settings);
fprintf("%d NMPC solvers created\nN-Min : %d\nN-max: %d\n\n",numel(settings.Nvals),min(settings.Nvals),max(settings.Nvals)); 
clearvars settings; 

%% Loop Step Sim Until Done [Dynamic Solver]
stepTime = 5; % seconds : how long the episode step runs for
randSolverIdx = randi(height(solvers.solverN));     % random solver index
nmpcSolver = solvers.solverStack(randSolverIdx);    % choose random solver
simSettings = initialSimSettings(nmpcSolver,map);   % local function to initalise settings for step sim
simSettings.loopSteps = stepTime /simSettings.DT;   % set number of steps per loop
simSettings.normalisedActions = true;               % enable normalised actions flag
simSettings.realCbfMin  = [0.1 0.01];               % [k1 kr] min real values
simSettings.realCbfMax  = [100 2.0 ];               % [k1 kr] max real values
simSettings.cbfParms = rand(map.mpcReqObs,2);       % randomise cbf values
simSettings.randSolverIdx = randSolverIdx;          % choose random index for solver
simSettings.realN = solvers.solverN(simSettings.randSolverIdx); % random Nvalue from solver set
temp = Normalizer.normalize01(simSettings.realN, min(solvers.solverN), max(solvers.solverN)); % random normalised N
isDone = false;                             % init flag to false
simdata = [];                               % init empty arrays
allSimdata = [];                            
stepRewards = [];
lastActions.N = simSettings.N;              % init last actions to current actions for reward calculation
lastActions.cbf = simSettings.cbfParms;

disp("Starting Simulation")
while ~isDone
    % run the simulation step
    simdata = simulationStepDyn(nmpcSolver, simSettings);
    
    % log simdata
    allSimdata = [allSimdata ; simdata ];
    
    % calculate rewards for this step
    stepReward = getStepReward(simdata,lastActions);
    stepRewards = [stepRewards , stepReward ];

    % check if done
    isDone = simdata.endAtTarget || simdata.endEpTimeout || simdata.endHitObs;
    % isDone = true;        % early stop for one step
    
    % update simSettings for next step
    simSettings.currentTime = simdata.end_current_time;     % start time
    simSettings.currentState = simdata.end_current_state;   % start vehicle state
    simSettings.mpcIter = simdata.mpcIter;                  % start mpc iteration count
    simSettings.ctrlHistory = simdata.usafe;                % control history array
    simSettings.ssHistory = simdata.sep;                    % safe seperation history array
    simSettings.stateHistory = simdata.states;              % state history
    simSettings.cbfParms = rand(map.mpcReqObs,2);           % normalised random cbf parms
    nmpcSolver = solvers.solverStack(randi(height(solvers.solverStack)));   % randomised solver for next step (rl-action)
    [simSettings.X0 simSettings.controlHorizon] = resizeX0newN(simdata.end_X0,nmpcSolver.settings.N, simdata.end_control_horizon); % reshape mpc arrays for new horizon
    simSettings.normalised.actions = rand(map.mpcReqObs,2)*2 -1; % randomise cbf values (rl-actions)
    lastActions.N = simdata.N;                              % store last N action for reward calculation
    lastActions.cbf = simdata.cbf;                          % store last cbfs for reward calculation
    fprintf(".");

    observations = getObservations(simdata);
end

% Print Info To Console
fprintf("\n  Total Episode Reward : %.2f\n", sum([stepRewards.reward]));
fprintf("         Progress Reward : %.2f %.3f \n", sum([stepRewards.rProgress])      , stepReward.weights(1));
fprintf("         Velocity Reward : %.2f %.3f \n", sum([stepRewards.rVelocity])      , stepReward.weights(2));
fprintf("      Computation Reward : %.2f %.3f \n", sum([stepRewards.rComp])          , stepReward.weights(3));
fprintf("        Parameter Reward : %.2f %.3f \n", sum([stepRewards.rParmStability]) , stepReward.weights(4));
fprintf("       Collision Penalty : %.2f      \n", sum([stepRewards.rCollision]));
fprintf("    Goal Terminal Reward : %.2f      \n", sum([stepRewards.rTermGoal]));
fprintf("    Time Terminal Reward : %.2f %.3f \n", sum([stepRewards.rTermTime])      , stepReward.weights(5));
fprintf(" Timeout Terminal Penalty: %.2f      \n", sum([stepRewards.rTermEpTimeout]));
fprintf("\nSimulation of %.2f seconds -> Complete!\n\n",simdata.end_current_time);

%%
close all; staticPlot= true; viewOnScreen = false;
fig = visualiseSimulationDyn(simdata,staticPlot,viewOnScreen);
figure(fig);



%% OLD CODE
%%
%%
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


function simSettings = initialSimSettings(solver,env)
    simSettings.cbfParms = repmat([22 , 86],env.mpcReqObs,1);
    simSettings.N = solver.settings.N;
    simSettings.DT = solver.settings.DT;
    simSettings.veh_rad = solver.settings.veh_rad;
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
    simSettings.cLevel = env.cLevel;
    disp("Simulation INITIAL Settings Created");
end

function [X0 , controlHorizon] = resizeX0newN(lastX0,N,lastCH)
    lastN = height(lastX0)-1;
    if N <= lastN
        X0 = lastX0(1:N+1,:);
        controlHorizon = lastCH(1:N,:);
    else
        extraRows = N-lastN;
        X0 = [lastX0; repmat(lastX0(end,:),extraRows,1)];
        controlHorizon = [lastCH ; repmat(lastCH(end,:),extraRows,1)];
    end
end