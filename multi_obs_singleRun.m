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
targetPos = [50 , 50];
env = generateRandomEnvironment(4, 1.0, 20, targetPos,[5.0;5.0;5.0] );
figure(env.fig);
clearvars targetPos
%% Run Dynamic Solver Step Loop [Dynamic Solver]

% create solver
import casadi.*
nmpc.DT = 0.1; 
nmpc.N = 20;
nmpc.velMax = 2;
nmpc.accMax = 5;
nmpc.cbfParms = [10, 40];
nmpc.veh_rad = 0.55;
nmpc.nObs = height(env.obstacles);
nmpcSolver = createMPCDynamicObsSolver(nmpc);
clearvars nmpc; disp("Solver Created")
%% Initial Step Sim Settings [Dynamic Solver]

simSettings.cbfParms = repmat([22 , 86],5,1);
simSettings.N = nmpcSolver.settings.N;
simSettings.DT = nmpcSolver.settings.DT;
simSettings.veh_rad = nmpcSolver.settings.veh_rad;
simSettings.loopSteps = 20;
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
disp("Starting Simulation")
isDone = false;
allSimdata = [];
while ~isDone

    simdata = simulationStepDyn(nmpcSolver, simSettings);

    allSimdata = [allSimdata ; simdata ];

    % update simSettings for next step
    simSettings.X0 = simdata.end_X0;
    simSettings.currentTime = simdata.end_current_time;
    simSettings.currentState = simdata.end_current_state;
    simSettings.mpcIter = simdata.mpcIter;
    simSettings.ctrlHistory = simdata.usafe;
    simSettings.ssHistory = simdata.sep;
    simSettings.stateHistory = simdata.states;

    isDone = simdata.endAtTarget || simdata.endEpTimeout || simdata.endHitObs;
end
disp("Simulation Complete")