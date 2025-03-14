
addpath("/home/sm/matlab/cbfRL/nmpc_cbf_rl_2parm_veh/functions/")

%% Run for single parameter [Dynamic Solver]

% firstrun = ~exist("solver","var") || ~exist("args","var") || ~exist("f","var");
firstrun = true;
if firstrun
    tStart = tic;
    addpath('/home/sm/matlab/com/casadi-3.6.7/');   % ### ADJUST PATH TO CASADI PACKAGE LOACTION ####          
    import casadi.*
    DT = 0.1; N = 10;
    velMax = 2;
    accMax = 5;
    cbfParms = [0.2, 1.5, 0.01];
    mpcParms = zeros(14,1);
    mpcParms(1:4) = [10 ; 1 ; 1 ; 1];   % Qx[xy yaw v w]
    mpcParms(5:6) = [10 ; 1];           % R[a alpha]
    mpcParms(7:9) = cbfParms;
    obs_rad = 1;
    veh_rad = 0.55;
    [obstacle, target] = setupObstacleScenario(obs_rad,veh_rad,[0,0,deg2rad(45)],false);
    % obstacle = [1000 1000 1];
    [solver, args, f] = createMPCDynamicSolver(DT,N,velMax,accMax,1);
    tSolver = toc(tStart);
    % toc
end

% cbfParms = [1,2,1];
simdata = simulationLoopDyn(solver,args,f, cbfParms, obs_rad, N, DT, false, mpcParms);


% Print some time data about sim
tsteps = size(simdata.states(1:3,:),2) -1;
tend = tsteps*DT;
ltime = simdata.looptime;
steptime = ltime/tsteps * 1000;
fprintf("\n\n###########################################################\n")
fprintf(" Time to create solver    : %f seconds\n",tSolver);
fprintf(" Time Period Simulated    : %f seconds\n",tend);
fprintf(" Simulation Time          : %f seconds\n",ltime);
fprintf(" Average loop time        : %f ms\n",steptime);
fprintf("###########################################################\n\n")

% finish
clearvars -except simdata*
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
fig = visualiseSimulation(simdata10,staticPlot,viewOnScreen);
figure(fig);
fig2 = visualiseSimulation(simdata50,staticPlot,viewOnScreen);
figure(fig2);
fig3 = visualiseSimulation(simdata90,staticPlot,viewOnScreen);
figure(fig3);
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