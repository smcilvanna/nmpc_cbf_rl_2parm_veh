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
%%

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





%% ########################################################################################################
%% Run full loop sim for single parameter [Dynamic Solver]
clc
% firstrun = ~exist("solver","var") || ~exist("args","var") || ~exist("f","var");
firstrun = true;
if firstrun
    tStart = tic;          
    import casadi.*
    settings.DT = 0.1; 
    settings.N = 20;
    settings.velMax = 2;
    settings.accMax = 5;
    settings.cbfParms = [22, 86];
    % settings.mpcParms = ones(14,1);
    % settings.mpcParms(7:9) = settings.cbfParms;
    settings.obs_rad = 5;
    settings.veh_rad = 0.55;
    [obstacle, target] = setupObstacleScenario(settings.obs_rad ,settings.veh_rad,[0,0,deg2rad(45)],false);
    % obstacle = [1000 1000 1];
    [solver, args, f] = createMPCDynamicSolver(settings);
    tSolver = toc(tStart);
    % toc
end

% ii = 1;
% solver = solverStack(ii);
% settings.cbfParms    = [0.2, 1.5, 0.01]; 
% settings.obs_rad    = 1;
% settings.N          = N%Nvals(ii);
% settings.DT         = 0.1;
% settings.mpcParms   = zeros(14,1);
% settings.veh_rad    =0.55;

settings.maxSimTime = 100;
settings.endSepTol = 0.1;

simdata = simulationLoopDyn(solver,args,f, settings);


% Print some time data about sim
tsteps = size(simdata.states(1:3,:),2) -1;
tend = tsteps*settings.DT;
ltime = simdata.looptime;
steptime = ltime/tsteps * 1000;
fprintf("\n\n###########################################################\n")
fprintf(" Time to create solver    : %f seconds\n",tSolver);
fprintf(" Time Period Simulated    : %f seconds\n",tend);
fprintf(" Simulation Time          : %f seconds\n",ltime);
fprintf(" Average loop time        : %f ms\n",steptime);
fprintf("###########################################################\n\n")

% finish
% clearvars -except simdata* 
disp("Simulation Done")
return



%% Run for single parameter [Kinematic Solver]
clear;
firstrun = ~exist("solver","var") || ~exist("args","var") || ~exist("f","var");
if firstrun
    clc, close all, clear all
    addpath('/home/sm/matlab/com/casadi-3.6.7/');   % ### ADJUST PATH TO CASADI PACKAGE LOACTION ####          
    import casadi.*
    DT = 0.1; N = 20;
    velMax = 2;
    cbfParms = [0.01, 0.01, 0.01];
    mpcParms = [10              % Qx[x+y]
                1               % Qx[yaw]
                0.1             % R[v]
                0.1             % R[w]
                100             % Q[x+y]
                1              % Q[yaw]
                zeros(12,1)];   % spare parms
    obs_rad = 5;
    veh_rad = 0.55;
    [obstacle, target] = setupObstacleScenario(obs_rad,veh_rad,[0,0,deg2rad(45)],false);
    % obstacle = [1000 1000 1];
    [solver, args, f] = createMPCKinematicSolver(DT,N,velMax,1);
end

% cbfParms = [1,2,1];
simdata = simulationLoop(solver,args,f, cbfParms, obs_rad, N, DT, false, mpcParms);

% input("Press ENTER to continue to Plots..")
clearvars -except simdata

%% Plots (animated)
close all;
staticPlot = false; viewOnScreen = true;
visualiseSimulation(simdata, staticPlot,viewOnScreen);
clearvars staticPlot viewOnScreen

%% Plot (static)
close all; staticPlot= true; viewOnScreen = false;
fig = visualiseSimulation(simdata,staticPlot,viewOnScreen);
figure(fig);
%%
fig2 = figure();
t = tiledlayout(3, 2);
nexttile
plot(simdata.usafe(:,1));
subtitle("Applied Control Longitudinal")
nexttile
plot(simdata.usafe(:,2))
subtitle("Applied Control Yaw")

nexttile
plot(simdata.ucbf(:,1));
subtitle("CBF-QP Action Longitudinal")
nexttile
plot(simdata.ucbf(:,2))
subtitle("CBF-QP Action Yaw")
hold off

nexttile
plot(simdata.umpc(:,1));
subtitle("MPC Action Longitudinal")
nexttile
plot(simdata.umpc(:,2))
subtitle("MPC Action Yaw")
hold off